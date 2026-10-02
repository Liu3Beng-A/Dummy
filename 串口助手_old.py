import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import serial
import serial.tools.list_ports
import threading
import queue
import time


BAUDRATE = 115200  # 固件 usart.c 中 huart4 波特率硬编码，PC 端必须一致

# 各关节软限位（v3.1 同步到 firmware_new 构造参数 ±175/±75/(0,180)/±270/±100/±180）
JOINT_RANGES = [(-175, 175), (-75, 90), (0, 180), (-270, 270), (-100, 100), (-180, 180)]
# 默认关节角 = 固件 REST_POSE = {0, -75, 180, 0, 0, 0}
JOINT_DEFAULTS = [0, -75, 180, 0, 0, 0]
# v3.1 默认速度 35（slider 35 ≈ 10.5 r/s，与新版串口助手对齐）
DEFAULT_SPEED = 35

# [v3.1 新增] 地轨（J7）参数：直连丝杆1605，5mm/圈，行程 ±250 mm
RAIL_RANGE = (-250, 250)
RAIL_DEFAULT = 0

# MoveL 滑块范围（笛卡尔空间覆盖范围）
POSE_LABELS = ['X', 'Y', 'Z', 'A', 'B', 'C']
POSE_RANGES = [(-50, 300), (-100, 260), (-150, 250), (-180, 180), (-180, 180), (-180, 180)]
# 默认位姿 = 关节 [0,-45,90,0,0,0] 的正运动学解，稳定落在所有关节限位内
POSE_DEFAULTS = [175, 187, -31, 90, -45, 180]


class RobotSerialAssistantOld:
    """旧固件（firmware_old）专用的 6 轴机械臂串口助手"""

    def __init__(self, root):
        self.root = root
        self.root.title("Dummy Robot 串口助手 (旧固件)")
        self.root.geometry("1680x1000")
        self.root.minsize(1560, 940)

        # 模式显示名表（本地缓存，固件 OLED 上同步刷新）
        self._MODE_NAME = {1: "顺序(SEQ)", 2: "打断(INT)", 3: "连续(TRJ)"}
        self.current_cmdmode = 2

        # 关节角 / 位姿回读同步状态（由 RX 线程写标志，主线程取数据）
        # 旧固件的 #GETJPOS 与 #GETLPOS 回包格式完全相同（"ok" + 6 个数），无法从回包
        # 内容区分来源，因此同一时刻只允许一个待同步请求，避免张冠李戴。
        self._pending_sync = None      # None | "j"(关节角) | "l"(位姿)
        self._sync_data = None
        # 往滑块回填数值时抑制拖发，否则 set() 触发的滑块回调会把刚读回的位置
        # 当成新的运动指令又发回给机械臂。
        self._suppress_drag_send = False

        # 点位示教
        self.stored_positions = []  # [(name, [j1..j6])]
        self._seq_running = False   # 顺序发送运行标志
        self._seq_pending = []      # 待发送点位
        self._seq_idx = 0           # 当前发送索引
        self._seq_speed = DEFAULT_SPEED

        # 主题与配色（与新版助手保持同一套色板）
        try:
            style = ttk.Style()
            if 'clam' in style.theme_names():
                style.theme_use('clam')

            style.configure("TNotebook", background="#e8edf5", borderwidth=0)
            style.configure("TNotebook.Tab", font=("Arial", 10, "bold"), padding=[14, 6])
            style.map("TNotebook.Tab",
                background=[("selected", "#3b5bdb"), ("!selected", "#c5cfd8")],
                foreground=[("selected", "white"), ("!selected", "#343a40")])

            # 基础色板（浅色主题 + 蓝紫强调色）
            bg = "#f4f6fb"
            panel = "#ffffff"
            border = "#d8dce8"
            primary = "#3b5bdb"
            primary_hover = "#4263eb"
            danger = "#e03131"
            danger_hover = "#c92a2a"
            warning = "#f76707"
            success = "#2b8a3e"
            text_main = "#1f2937"
            text_sub = "#4b5563"
            accent_soft = "#eef2ff"

            self.root.configure(background=bg)

            style.configure(".", background=bg, foreground=text_main, font=("Arial", 10))
            style.configure("TFrame", background=bg)
            style.configure("TLabelframe", background=panel, foreground=text_main,
                            bordercolor=border, borderwidth=1)
            style.configure("TLabelframe.Label", background=panel, foreground=text_main,
                            font=("Arial", 10, "bold"))
            style.configure("TLabel", background=bg, foreground=text_main)
            style.configure("TCheckbutton", background=bg)
            style.configure("TRadiobutton", background=bg)

            style.configure("TButton", font=("Arial", 10), padding=6)
            style.map("TButton", background=[("active", "#e5e7eb")])

            style.configure("Accent.TButton", foreground="#ffffff", background=primary)
            style.map("Accent.TButton", background=[("active", primary_hover)])

            style.configure("Danger.TButton", foreground="#ffffff", background=danger)
            style.map("Danger.TButton", background=[("active", danger_hover)])

            style.configure("Warning.TButton", foreground="#ffffff", background=warning)
            style.map("Warning.TButton", background=[("pressed", "#e8590c")])

            style.configure("Success.TButton", foreground="#ffffff", background=success)
            style.map("Success.TButton", background=[("active", "#2f9e44")])

            style.configure("TEntry", fieldbackground=panel, bordercolor=border,
                            lightcolor=border, darkcolor=border)
            style.configure("TCombobox", fieldbackground=panel, bordercolor=border,
                            lightcolor=border, darkcolor=border)
            style.configure("Horizontal.TScale", background=bg, troughcolor=border)

        except Exception:
            pass

        # 图标
        new_icon_data = "R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7"
        try:
            self.icon_img = tk.PhotoImage(data=new_icon_data)
            self.root.iconphoto(True, self.icon_img)
        except Exception:
            pass

        self.serial_port = None
        self._tx_queue = queue.Queue()
        self._serial_write_lock = threading.Lock()
        self._tx_thread = None
        self.is_connected = False
        self._motor_enabled = False  # 电机使能状态

        self.create_widgets()
        self.refresh_ports()

        self.receive_thread = None
        self.stop_thread = False

    # ==============================================
    # 界面构建
    # ==============================================
    def create_widgets(self):
        # ==============================================
        # 顶部状态栏（始终可见，急停最醒目）
        # ==============================================
        top_bar = tk.Frame(self.root, bg="#2d2d2d", height=56)
        top_bar.pack(fill=tk.X)
        top_bar.pack_propagate(False)

        # 左侧：连接控制区
        left_top = tk.Frame(top_bar, bg="#2d2d2d")
        left_top.pack(side=tk.LEFT, padx=12, pady=8)

        tk.Label(left_top, text="端口:", bg="#2d2d2d", fg="#cccccc", font=("Arial", 10)).pack(side=tk.LEFT)
        self.cb_ports = ttk.Combobox(left_top, width=14, font=("Arial", 10))
        self.cb_ports.pack(side=tk.LEFT, padx=(4, 8))

        ttk.Button(left_top, text="刷新", command=self.refresh_ports, width=5).pack(side=tk.LEFT, padx=6)

        self.btn_connect = tk.Button(left_top, text="连接", font=("Arial", 10, "bold"),
                                     bg="#2b8a3e", fg="white", activebackground="#2f9e44",
                                     relief=tk.FLAT, padx=14, command=self.toggle_connection)
        self.btn_connect.pack(side=tk.LEFT, padx=(4, 0))

        # 中间：连接状态 LED
        status_frame = tk.Frame(top_bar, bg="#2d2d2d")
        status_frame.pack(side=tk.LEFT, padx=20, pady=8)

        self.led_canvas = tk.Canvas(status_frame, width=16, height=16, bg="#2d2d2d",
                                    highlightthickness=0)
        self.led_canvas.pack(side=tk.LEFT)
        self._draw_led(False)
        self.lbl_status = tk.Label(status_frame, text="未连接", bg="#2d2d2d",
                                   fg="#ff6b6b", font=("Arial", 10, "bold"))
        self.lbl_status.pack(side=tk.LEFT, padx=6)

        # 电机使能状态指示
        self.lbl_motor_state = tk.Label(top_bar, text="● 电机已失能", font=("Arial", 12, "bold"),
                                         bg="#495057", fg="#ff6b6b", padx=20, pady=8)
        self.lbl_motor_state.pack(side=tk.RIGHT, padx=(0, 8), pady=8)

        # 右侧：急停按钮（最醒目）
        self.btn_emergency = tk.Button(top_bar, text="[ !STOP 急停 ]", font=("Arial", 12, "bold"),
                                       bg="#c92a2a", fg="white", activebackground="#a02222",
                                       relief=tk.RAISED, padx=20, pady=8,
                                       command=lambda: self.send_cmd("!STOP"))
        self.btn_emergency.pack(side=tk.RIGHT, padx=16, pady=8)

        # ==============================================
        # 主区域：控制台 + 终端（上下分栏）
        # ==============================================
        main_paned = ttk.PanedWindow(self.root, orient=tk.VERTICAL)
        main_paned.pack(fill=tk.BOTH, expand=True, padx=8, pady=(4, 6))

        control_area = ttk.Frame(main_paned)
        main_paned.add(control_area, weight=2)

        # 左栏窄、右栏宽（右栏是主要工作区）
        control_area.columnconfigure(0, weight=1)  # 左栏
        control_area.columnconfigure(1, weight=4)  # 右栏（关节控制 / MoveL）
        control_area.rowconfigure(0, weight=1)

        # ==============================================
        # 左栏：系统控制 + 模式 + 查询 + 电机
        # ==============================================
        left_col = ttk.Frame(control_area)
        left_col.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        left_col.columnconfigure(0, minsize=320)

        # --- 系统控制 ---
        sys_f = ttk.LabelFrame(left_col, text="系统控制", padding=6)
        sys_f.pack(fill=tk.X, pady=(0, 6))

        sys_row1 = ttk.Frame(sys_f)
        sys_row1.pack(fill=tk.X)
        tk.Button(sys_row1, text="启动", font=("Arial", 10, "bold"), bg="#2b8a3e", fg="white",
                  relief=tk.FLAT, pady=6, command=self._cmd_start
                  ).pack(side=tk.LEFT, expand=True, fill=tk.BOTH, padx=1)
        tk.Button(sys_row1, text="失能", font=("Arial", 10), bg="#495057", fg="white",
                  relief=tk.FLAT, pady=6, command=self._cmd_disable
                  ).pack(side=tk.LEFT, expand=True, fill=tk.BOTH, padx=1)
        tk.Button(sys_row1, text="回零", font=("Arial", 10), bg="#5c7cfa", fg="white",
                  relief=tk.FLAT, pady=6, command=lambda: self.send_cmd("!HOME")
                  ).pack(side=tk.LEFT, expand=True, fill=tk.BOTH, padx=1)
        tk.Button(sys_row1, text="休息", font=("Arial", 10), bg="#868e96", fg="white",
                  relief=tk.FLAT, pady=6, command=lambda: self.send_cmd("!RESET")
                  ).pack(side=tk.LEFT, expand=True, fill=tk.BOTH, padx=1)

        ttk.Separator(sys_f, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=6)

        # 置零(校准)：固件执行完会重启控制器，必须先确认
        tk.Button(sys_f, text="置零(校准)", font=("Arial", 10, "bold"), bg="#c2255c", fg="white",
                  relief=tk.FLAT, pady=5, command=self._cmd_calibration
                  ).pack(fill=tk.X, padx=1)

        # --- 模式（旧固件只支持 1/2/3）---
        mode_f = ttk.LabelFrame(left_col, text="模式", padding=6)
        mode_f.pack(fill=tk.X, pady=(0, 6))
        ttk.Label(mode_f, text="指令模式:", font=("Arial", 10, "bold")).pack(anchor="w")
        mode_btn_f = ttk.Frame(mode_f)
        mode_btn_f.pack(fill=tk.X)
        for label, val in [("1:顺序", 1), ("2:打断", 2), ("3:连续", 3)]:
            tk.Button(mode_btn_f, text=label, font=("Arial", 10), bg="#495057", fg="white",
                      relief=tk.FLAT, pady=5, command=lambda v=val: self._set_cmdmode(v)
                      ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=1)

        # --- 查询 ---
        query_f = ttk.LabelFrame(left_col, text="查询", padding=6)
        query_f.pack(fill=tk.X, pady=(0, 6))
        tk.Button(query_f, text="获取关节角", font=("Arial", 10), bg="#495057", fg="white",
                  relief=tk.FLAT, pady=5, command=self.read_and_sync_from_robot
                  ).pack(fill=tk.X, padx=1, pady=1)
        tk.Button(query_f, text="获取位姿", font=("Arial", 10), bg="#495057", fg="white",
                  relief=tk.FLAT, pady=5, command=self.read_pose_from_robot
                  ).pack(fill=tk.X, padx=1, pady=(1, 0))

        # --- 电机（旧固件只有 J1~J6，节点 1~6）---
        motor_f = ttk.LabelFrame(left_col, text="电机", padding=6)
        motor_f.pack(fill=tk.X)
        node_f = ttk.Frame(motor_f)
        node_f.pack(fill=tk.X)
        ttk.Label(node_f, text="节点:", font=("Arial", 10, "bold")).pack(side=tk.LEFT)
        self.cb_node = ttk.Combobox(node_f, width=5,
                                    values=[str(i) for i in range(1, 7)],
                                    state="readonly", font=("Arial", 10))
        self.cb_node.current(0)
        self.cb_node.pack(side=tk.LEFT, padx=4)
        tk.Button(node_f, text="电机重启", font=("Arial", 10), bg="#c92a2a", fg="white",
                  relief=tk.FLAT, command=self._cmd_reboot
                  ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        # ==============================================
        # 右栏：Notebook（关节控制 / MoveL）
        # ==============================================
        right_col = ttk.Frame(control_area)
        right_col.grid(row=0, column=1, sticky="nsew", padx=(0, 0))
        right_col.rowconfigure(0, weight=1)
        right_col.columnconfigure(0, weight=1)

        self.joint_nb = ttk.Notebook(right_col, height=460)
        self.joint_nb.pack(fill=tk.BOTH, expand=True)

        movej_page = ttk.Frame(self.joint_nb, padding=5)
        self.joint_nb.add(movej_page, text=" 关节控制 ")
        self._build_movej_page(movej_page)

        movel_page = ttk.Frame(self.joint_nb, padding=5)
        self.joint_nb.add(movel_page, text=" MoveL ")
        self._build_movel_page(movel_page)

        # ==============================================
        # 底部终端
        # ==============================================
        log_f = ttk.LabelFrame(main_paned, text="终端日志", padding=6)
        main_paned.add(log_f, weight=1)

        cmd_row = ttk.Frame(log_f)
        cmd_row.pack(side=tk.BOTTOM, fill=tk.X, pady=(4, 0))

        self.cmd_history = []
        self.cmd_history_idx = [-1]

        self.ent_custom_cmd = ttk.Entry(cmd_row, font=("Consolas", 10))
        self.ent_custom_cmd.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.ent_custom_cmd.bind("<Return>", lambda e: self._send_with_history())
        self.ent_custom_cmd.bind("<Up>", self._history_up)
        self.ent_custom_cmd.bind("<Down>", self._history_down)
        tk.Button(cmd_row, text="发送", font=("Arial", 10, "bold"), bg="#3b5bdb", fg="white",
                  relief=tk.FLAT, command=self._send_with_history).pack(side=tk.LEFT, padx=4)
        tk.Button(cmd_row, text="清空", font=("Arial", 10), bg="#495057", fg="white",
                  relief=tk.FLAT, command=lambda: self.txt_log.delete(1.0, tk.END)).pack(side=tk.LEFT)

        self.txt_log = scrolledtext.ScrolledText(log_f, wrap=tk.WORD, font=("Consolas", 10),
                                                  bg="#1e1e1e", fg="#d4d4d4", insertbackground="white")
        self.txt_log.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self.txt_log.tag_config("TX", foreground="#4ec9b0")
        self.txt_log.tag_config("RX", foreground="#9cdcfe")
        self.txt_log.tag_config("ERROR", foreground="#f48771")
        self.txt_log.tag_config("WARN", foreground="#dcdcaa")
        self.txt_log.tag_config("INFO", foreground="#6a9955")

    # ==============================================
    # 关节控制页（MoveJ）
    # ==============================================
    def _build_movej_page(self, parent):
        jgrid = ttk.Frame(parent)
        jgrid.pack(fill=tk.X)
        jgrid.columnconfigure(2, weight=1)

        self.ent_joints = []
        self.scl_joints = []
        self.lbl_joints = []
        self.movej_drag_enable = tk.BooleanVar(value=False)
        self.last_movej_send_time = 0

        for i in range(6):
            rng, df = JOINT_RANGES[i], JOINT_DEFAULTS[i]
            ttk.Label(jgrid, text=f"J{i+1}:", font=("Arial", 10, "bold")
                      ).grid(row=i, column=0, padx=4, pady=2, sticky="e")
            ent = ttk.Entry(jgrid, width=7, font=("Arial", 10))
            ent.insert(0, str(df))
            ent.grid(row=i, column=1, padx=4, pady=2)
            self.ent_joints.append(ent)
            scl = ttk.Scale(jgrid, from_=rng[0], to=rng[1], orient=tk.HORIZONTAL)
            scl.set(df)
            scl.grid(row=i, column=2, sticky="ew", padx=8, pady=2)
            self.scl_joints.append(scl)
            lbl = ttk.Label(jgrid, text=f"{df:.1f}", width=6, font=("Arial", 9))
            lbl.grid(row=i, column=3, padx=4, pady=2)
            self.lbl_joints.append(lbl)

            def make_jcb(lbl, ent):
                def cb(val):
                    v = float(val)
                    lbl.config(text=f"{v:.1f}")
                    s = f"{v:.1f}"
                    if ent.get() != s:
                        ent.delete(0, tk.END)
                        ent.insert(0, s)
                    if self.movej_drag_enable.get() and not self._suppress_drag_send:
                        now = time.time()
                        if now - self.last_movej_send_time > 0.1:
                            self.last_movej_send_time = now
                            self.root.after(1, self.send_movej)
                return cb
            scl.config(command=make_jcb(lbl, ent))

            def make_jentrycb(idx, scl):
                def cb(event):
                    try:
                        scl.set(float(self.ent_joints[idx].get()))
                    except ValueError:
                        pass
                return cb
            ent.bind("<Return>", make_jentrycb(i, scl))
            ent.bind("<FocusOut>", make_jentrycb(i, scl))

        # [v3.1 新增] Rail（J7）行 —— 单独第 7 行，与 J1~J6 同布局
        ttk.Label(jgrid, text="Rail:", font=("Arial", 10, "bold"), foreground="#007ACC"
                  ).grid(row=6, column=0, padx=4, pady=2, sticky="e")
        self.ent_j7 = ttk.Entry(jgrid, width=7, font=("Arial", 10))
        self.ent_j7.insert(0, str(RAIL_DEFAULT))
        self.ent_j7.grid(row=6, column=1, padx=4, pady=2)
        self.scl_j7 = ttk.Scale(jgrid, from_=RAIL_RANGE[0], to=RAIL_RANGE[1], orient=tk.HORIZONTAL)
        self.scl_j7.set(RAIL_DEFAULT)
        self.scl_j7.grid(row=6, column=2, sticky="ew", padx=8, pady=2)
        self.lbl_j7 = ttk.Label(jgrid, text=f"{RAIL_DEFAULT:.1f} mm", width=8, font=("Arial", 9))
        self.lbl_j7.grid(row=6, column=3, padx=4, pady=2)

        def j7_scl_cb(val):
            v = float(val)
            self.lbl_j7.config(text=f"{v:.1f} mm")
            s = f"{v:.1f}"
            if self.ent_j7.get() != s:
                self.ent_j7.delete(0, tk.END)
                self.ent_j7.insert(0, s)
            if self.movej_drag_enable.get() and not self._suppress_drag_send:
                now = time.time()
                if now - self.last_movej_send_time > 0.1:
                    self.last_movej_send_time = now
                    self.root.after(1, self.send_movej)
        self.scl_j7.config(command=j7_scl_cb)

        def j7_entry_cb(event):
            try:
                self.scl_j7.set(float(self.ent_j7.get()))
            except ValueError:
                pass
        self.ent_j7.bind("<Return>", j7_entry_cb)
        self.ent_j7.bind("<FocusOut>", j7_entry_cb)

        # 控制行：速度 + 拖发 + 发送
        ctrl_f = ttk.Frame(parent)
        ctrl_f.pack(fill=tk.X, pady=(8, 0))
        ttk.Label(ctrl_f, text="Speed:", font=("Arial", 9)).pack(side=tk.LEFT, padx=(0, 2))
        self.ent_j_speed = ttk.Entry(ctrl_f, width=6, font=("Arial", 9))
        self.ent_j_speed.insert(0, str(DEFAULT_SPEED))
        self.ent_j_speed.pack(side=tk.LEFT)
        self._make_toggle_button(
            ctrl_f, self.movej_drag_enable,
            text_off="拖发 OFF", text_on="拖发 ON",
            bg_off="#e9ecef", bg_on="#2b8a3e",
            fg_off="#495057", fg_on="white",
            side=tk.LEFT, padx=2,
            on_change=self._warn_drag_mode,
        )
        tk.Button(ctrl_f, text="发送 MoveJ", font=("Arial", 10, "bold"), bg="#3b5bdb", fg="white",
                  relief=tk.FLAT, command=self.send_movej).pack(side=tk.RIGHT, padx=2)

        # --- 点位存储（示教）---
        ttk.Separator(parent, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=(8, 6))

        pos_top = ttk.Frame(parent)
        pos_top.pack(fill=tk.X, pady=(0, 4))
        tk.Button(pos_top, text="读取机械臂位置→滑块", font=("Arial", 10), bg="#3b5bdb", fg="white",
                  relief=tk.FLAT, command=self.read_and_sync_from_robot
                  ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 2))
        tk.Button(pos_top, text="存储当前滑块", font=("Arial", 10), bg="#2b8a3e", fg="white",
                  relief=tk.FLAT, command=self.save_current_position
                  ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        tk.Button(pos_top, text="读取滑块值", font=("Arial", 10), bg="#495057", fg="white",
                  relief=tk.FLAT, command=self.read_current_position
                  ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(2, 0))

        ttk.Label(parent, text="已存储的点位:", font=("Arial", 9, "bold")).pack(anchor="w")
        lb_frame = ttk.Frame(parent)
        lb_frame.pack(fill=tk.X, pady=2)
        scroll_y = ttk.Scrollbar(lb_frame, orient=tk.VERTICAL)
        scroll_y.pack(side=tk.RIGHT, fill=tk.Y)
        self._pos_listbox = tk.Listbox(lb_frame, font=("Arial", 9), height=5, yscrollcommand=scroll_y.set)
        self._pos_listbox.pack(side=tk.LEFT, fill=tk.X, expand=True)
        scroll_y.config(command=self._pos_listbox.yview)
        self._pos_listbox.bind("<Double-Button-1>", lambda e: self._on_select_position())
        self._pos_listbox.bind("<Delete>", lambda e: self._delete_position())

        pos_ops = ttk.Frame(parent)
        pos_ops.pack(fill=tk.X, pady=(0, 4))
        tk.Button(pos_ops, text="发送选中", font=("Arial", 10), bg="#3b5bdb", fg="white",
                  relief=tk.FLAT, command=self.send_selected_position
                  ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 2))
        tk.Button(pos_ops, text="删除", font=("Arial", 10), bg="#c92a2a", fg="white",
                  relief=tk.FLAT, command=self._delete_position
                  ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        tk.Button(pos_ops, text="清空", font=("Arial", 10), bg="#495057", fg="white",
                  relief=tk.FLAT, command=self.clear_all_positions
                  ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(2, 0))

        pos_bot = ttk.Frame(parent)
        pos_bot.pack(fill=tk.X)
        ttk.Label(pos_bot, text="速度:", font=("Arial", 10)).pack(side=tk.LEFT)
        self.ent_pos_speed = ttk.Entry(pos_bot, width=6, font=("Arial", 10))
        self.ent_pos_speed.insert(0, str(DEFAULT_SPEED))
        self.ent_pos_speed.pack(side=tk.LEFT, padx=4)
        tk.Button(pos_bot, text="顺序发送全部→机械臂", font=("Arial", 10, "bold"),
                  bg="#495057", fg="white", relief=tk.FLAT, command=self.send_all_positions
                  ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(8, 0))

    # ==============================================
    # MoveL 页
    # ==============================================
    def _build_movel_page(self, parent):
        pgrid = ttk.Frame(parent)
        pgrid.pack(fill=tk.X)
        pgrid.columnconfigure(2, weight=1)

        self.ent_pose, self.scl_pose, self.lbl_pose = [], [], []
        self.movel_drag_enable = tk.BooleanVar(value=False)
        self.last_movel_send_time = 0

        for i in range(6):
            df, rng = POSE_DEFAULTS[i], POSE_RANGES[i]
            ttk.Label(pgrid, text=f"{POSE_LABELS[i]}:", font=("Arial", 10, "bold")
                      ).grid(row=i, column=0, padx=4, pady=2, sticky="e")
            ent = ttk.Entry(pgrid, width=7, font=("Arial", 10))
            ent.insert(0, str(df))
            ent.grid(row=i, column=1, padx=4, pady=2)
            self.ent_pose.append(ent)
            scl = ttk.Scale(pgrid, from_=rng[0], to=rng[1], orient=tk.HORIZONTAL)
            scl.set(df)
            scl.grid(row=i, column=2, sticky="ew", padx=8, pady=2)
            self.scl_pose.append(scl)
            lbl = ttk.Label(pgrid, text=f"{df:.1f}", width=6, font=("Arial", 9))
            lbl.grid(row=i, column=3, padx=4, pady=2)
            self.lbl_pose.append(lbl)

            def make_pcb(lbl, ent):
                def cb(val):
                    v = float(val)
                    lbl.config(text=f"{v:.1f}")
                    s = f"{v:.1f}"
                    if ent.get() != s:
                        ent.delete(0, tk.END)
                        ent.insert(0, s)
                    if self.movel_drag_enable.get() and not self._suppress_drag_send:
                        now = time.time()
                        if now - self.last_movel_send_time > 0.05:
                            self.last_movel_send_time = now
                            self.root.after(1, self.send_movel)
                return cb
            scl.config(command=make_pcb(lbl, ent))

            def make_pentrycb(idx, scl):
                def cb(event):
                    try:
                        scl.set(float(self.ent_pose[idx].get()))
                    except ValueError:
                        pass
                return cb
            ent.bind("<Return>", make_pentrycb(i, scl))
            ent.bind("<FocusOut>", make_pentrycb(i, scl))

        movel_ctrl = ttk.Frame(parent)
        movel_ctrl.pack(fill=tk.X, pady=(8, 0))
        ttk.Label(movel_ctrl, text="Speed:", font=("Arial", 10)).pack(side=tk.LEFT, padx=(0, 2))
        self.ent_l_speed = ttk.Entry(movel_ctrl, width=6, font=("Arial", 10))
        self.ent_l_speed.insert(0, str(DEFAULT_SPEED))
        self.ent_l_speed.pack(side=tk.LEFT)
        self._make_toggle_button(
            movel_ctrl, self.movel_drag_enable,
            text_off="拖发 OFF", text_on="拖发 ON",
            bg_off="#e9ecef", bg_on="#2b8a3e",
            fg_off="#495057", fg_on="white",
            font=("Arial", 10, "bold"),
            side=tk.LEFT, padx=2,
            on_change=self._warn_drag_mode,
        )
        tk.Button(movel_ctrl, text="发送 MoveL", font=("Arial", 10, "bold"), bg="#3b5bdb",
                  fg="white", relief=tk.FLAT, command=self.send_movel
                  ).pack(side=tk.RIGHT, padx=2)
        tk.Button(movel_ctrl, text="读取当前位姿", font=("Arial", 10), bg="#495057", fg="white",
                  relief=tk.FLAT, command=self.read_pose_from_robot
                  ).pack(side=tk.RIGHT, padx=2)

    # ==============================================
    # 通用小部件 / 工具
    # ==============================================
    def _make_toggle_button(self, parent, var, text_off, text_on,
                            bg_off=None, bg_on="#2b8a3e",
                            fg_off="black", fg_on="white",
                            font=("Arial", 9, "bold"),
                            side=tk.LEFT, padx=2, pady=2, fill=None,
                            on_change=None):
        """创建一个 toggle 风格按钮（按下/弹起自动切换文字与背景色）。

        参数:
            parent: 父容器
            var   : tk.BooleanVar，True=ON 态，False=OFF 态
            text_off / text_on: 弹起/按下时显示的文字
            bg_off / bg_on   : 弹起/按下时的背景色（None 表示使用父容器背景）
            fg_off / fg_on   : 弹起/按下时的前景色
            on_change: 可选回调，仅在用户点击切换时触发（初始应用外观不触发）
        """
        def _apply(trigger_cb=False):
            if var.get():
                btn.configure(text=text_on, bg=bg_on if bg_on else parent.cget("bg"),
                              fg=fg_on, activebackground=bg_on, relief=tk.SUNKEN)
            else:
                btn.configure(text=text_off, bg=bg_off if bg_off else parent.cget("bg"),
                              fg=fg_off, activebackground=bg_off, relief=tk.RAISED)
            if trigger_cb and on_change:
                on_change()
        btn = tk.Checkbutton(
            parent, text=text_off, variable=var, indicatoron=False,
            selectcolor=bg_on if bg_on else parent.cget("bg"),
            font=font, relief=tk.RAISED, padx=6, pady=pady,
            command=lambda: _apply(trigger_cb=True),
        )
        if fill:
            btn.pack(side=side, padx=padx, pady=pady, fill=fill, expand=True)
        else:
            btn.pack(side=side, padx=padx)
        _apply(trigger_cb=False)  # 初始应用外观，不触发回调
        return btn

    def _draw_led(self, on):
        """顶部状态 LED（绿=已连接，灰=未连接）"""
        self.led_canvas.delete("all")
        color = "#4ade80" if on else "#6b7280"
        self.led_canvas.create_oval(2, 2, 14, 14, fill=color, outline="")

    @staticmethod
    def _parse_numbers(payload):
        """把一段响应文本解析成浮点列表；出现任何非数字字段就返回 None。"""
        nums = []
        for tok in payload.replace(",", " ").split():
            try:
                nums.append(float(tok))
            except ValueError:
                return None
        return nums

    def log(self, msg, tag="INFO"):
        """终端日志（所有线程的输出最终都要通过 root.after 走到这里）"""
        time_str = time.strftime("%H:%M:%S")
        tag_color = {"TX": "TX", "RX": "RX", "ERROR": "ERROR", "WARN": "WARN"}.get(tag, "INFO")
        self.txt_log.insert(tk.END, f"[{time_str}] [{tag}] {msg}\n", tag_color)
        self.txt_log.see(tk.END)

    # ==============================================
    # 串口连接 / 收发线程
    # ==============================================
    def refresh_ports(self):
        ports = serial.tools.list_ports.comports()
        self.cb_ports['values'] = [p.device for p in ports]
        if ports:
            self.cb_ports.current(0)

    def toggle_connection(self):
        if not self.is_connected:
            port = self.cb_ports.get()
            try:
                self.serial_port = serial.Serial(port, BAUDRATE, timeout=0.5)
                self.serial_port.write_timeout = 1.0
                self.is_connected = True
                self.btn_connect.config(text="断开", bg="#c92a2a", activebackground="#a02222")
                self._draw_led(True)
                self.lbl_status.config(text=f"已连接 {port}  {BAUDRATE}", fg="#4ade80")
                self.log(f"成功连接 {port} @ {BAUDRATE}")

                self.stop_thread = False
                self.receive_thread = threading.Thread(target=self.serial_receive_loop, daemon=True)
                self.receive_thread.start()
                self._start_tx_thread()
            except Exception as e:
                messagebox.showerror("连接失败", str(e))
                self.log(f"连接失败: {e}", "ERROR")
        else:
            self.stop_thread = True
            self._stop_tx_thread()
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
            self.is_connected = False
            self.btn_connect.config(text="连接", bg="#2b8a3e", activebackground="#2f9e44")
            self._draw_led(False)
            self.lbl_status.config(text="未连接", fg="#ff6b6b")
            self._update_motor_state(False)
            # 清掉所有等待中的异步状态，避免重连后被上一轮的残留状态卡住
            self._pending_sync = None
            self._sync_data = None
            self._seq_running = False
            self.log("已断开串口")

    def serial_receive_loop(self):
        """接收轮询线程：in_waiting 轮询，周期 10ms。所有 UI 操作走 root.after。"""
        while not self.stop_thread and self.serial_port and self.serial_port.is_open:
            try:
                # 优先用 in_waiting 避免 readline 阻塞（部分 MCU 不发 \n）
                if self.serial_port.in_waiting:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    if data:
                        text = data.decode('utf-8', errors='replace')
                        for line in self._split_rx_lines(text):
                            self._handle_rx_line(line)
                # 无数据时短暂休眠，不阻塞主循环（接收轮询周期 10ms，降低交互延迟）
                time.sleep(0.01)
            except serial.SerialException as e:
                self.root.after(0, self.log, f"串口异常: {e}", "ERROR")
                break
            except Exception as e:
                self.root.after(0, self.log, f"读取异常: {e}", "ERROR")
                break

    @staticmethod
    def _split_rx_lines(text):
        """把一次串口读取切成若干条独立回执。

        固件每条 Respond 都以 "\\r\\n" 结尾，正常情况下一行一条；但 CAN 回调里的
        printf 可能不带换行，把多条回执粘到一行，所以这里额外按 "ok " 拆一次。
        """
        out = []
        for raw in text.split('\n'):
            raw = raw.strip()
            if not raw:
                continue
            if raw == "ok":
                out.append("ok")
            elif raw.startswith("ok "):
                parts = raw.split("ok ")
                out.extend("ok " + p.strip() for p in parts[1:] if p.strip())
            else:
                out.append(raw)
        return out

    def _handle_rx_line(self, line):
        """单行回执处理（在 RX 线程调用，严禁直接操作控件）"""
        # 裸数字 = 固件指令 FIFO 剩余空位：255 说明队列已满，其余仅是余量计数，静默过滤
        if line.isdigit():
            if line == "255":
                self.root.after(0, self.log,
                                "指令队列已满(FIFO 16)，该指令被固件丢弃，请降低发送速率", "WARN")
            return

        self.root.after(0, self.log, line, "RX")

        # #GETJPOS / #GETLPOS 回包 → 同步滑块（"ok" + 6 个数）
        # 注意：这里只落数据，不清 _pending_sync。两种回包格式完全相同，无法从内容
        # 区分来源，必须由 500ms 定时器统一收尾时再连同 kind 一起取走。
        if self._pending_sync and line.startswith("ok"):
            nums = self._parse_numbers(line[2:].strip())
            if nums is not None and len(nums) == 6:
                self._sync_data = nums

        # 顺序发送：固件每执行完一条 ">" 回一个 "ok"，收到后再发下一条
        if self._seq_running and line == "ok":
            self._seq_running = False
            self._seq_idx += 1
            if self._seq_idx < len(self._seq_pending):
                self.root.after(0, self._send_next_position)
            else:
                total = len(self._seq_pending)
                self.root.after(0, lambda t=total: self.log(f"顺序发送完成,共{t}个点位", "INFO"))

    def _serial_write_loop(self):
        """守护发送线程：从队列取命令，经串口写出。"""
        while True:
            cmd = self._tx_queue.get()
            if cmd is None:  # 停止哨兵
                break
            try:
                with self._serial_write_lock:
                    port_obj = self.serial_port
                    if port_obj is None or not port_obj.is_open:
                        self.root.after(0, self.log, f"发送失败(串口未打开): {cmd}", "ERROR")
                        continue
                    port_obj.write((cmd + "\n").encode('utf-8'))
            except Exception as e:
                self.root.after(0, self.log, f"发送失败: {e}", "ERROR")

    def _start_tx_thread(self):
        self._tx_thread = threading.Thread(target=self._serial_write_loop, daemon=True)
        self._tx_thread.start()

    def _stop_tx_thread(self):
        """停止发送线程：先丢弃队列残留命令，再投哨兵，最后 join 等线程真正退出。
        顺序不能颠倒：先投哨兵再清空会把哨兵一起吞掉，导致线程收不到停止信号而永久泄漏。
        """
        while True:
            try:
                self._tx_queue.get_nowait()
            except queue.Empty:
                break
        self._tx_queue.put(None)
        if self._tx_thread is not None:
            self._tx_thread.join(timeout=2.0)
            self._tx_thread = None

    def send_cmd(self, cmd):
        """所有指令的唯一出口：入队 + 记 TX 日志，由写线程串行发出"""
        if not self.is_connected or not self.serial_port:
            self.log("未连接串口", "WARN")
            return
        try:
            self._tx_queue.put(cmd)
            self.log(cmd, "TX")
        except Exception as e:
            self.log(f"入队失败: {e}", "ERROR")

    # ==============================================
    # 系统控制
    # ==============================================
    def _update_motor_state(self, enabled):
        self._motor_enabled = enabled
        if enabled:
            self.lbl_motor_state.config(text="● 电机已使能", bg="#2b8a3e", fg="white")
        else:
            self.lbl_motor_state.config(text="● 电机已失能", bg="#495057", fg="#ff6b6b")

    def _cmd_start(self):
        self._update_motor_state(True)
        self.send_cmd("!START")

    def _cmd_disable(self):
        self._update_motor_state(False)
        self.send_cmd("!DISABLE")

    def _cmd_calibration(self):
        """置零(校准)：固件写完校准值后主控会重启，必须先确认"""
        if not messagebox.askyesno(
            "确认置零",
            "置零(校准) 会写入当前位置作为关节零点，\n"
            "并且控制器随后会重新启动。\n\n确定继续吗？"
        ):
            return
        self._update_motor_state(False)
        self.send_cmd("!CALIBRATION")

    def _cmd_reboot(self):
        node = self.cb_node.get()
        self.send_cmd(f"#REBOOT {node}")

    def _set_cmdmode(self, mode):
        """点击「模式」按钮：更新本地缓存 + 转发指令到固件。
        固件 OLED 上同步刷新，串口助手本地缓存跟随。
        """
        self.current_cmdmode = mode
        self.send_cmd(f"#CMDMODE {mode}")
        self.log(f"已切换至模式 {mode} ({self._MODE_NAME.get(mode, '?')})", "INFO")

    def _warn_drag_mode(self):
        """拖发开关切换后回调。若不是「连续模式(3)」则弹提示（不自动切换）。"""
        if (self.movej_drag_enable.get() or self.movel_drag_enable.get()) \
                and self.current_cmdmode != 3:
            cur = self._MODE_NAME.get(self.current_cmdmode, f"模式{self.current_cmdmode}")
            messagebox.showwarning(
                "模式提示",
                f"拖发建议使用「连续模式」(模式 3)\n"
                f"当前是「{cur}」\n\n"
                f"请在「模式」区域手动点击「3:连续」按钮切换。"
            )

    def _warn_seqsend_mode(self):
        """顺序发送按钮按下时回调。若不是「顺序模式(1)」或「连续模式(3)」则弹提示。"""
        if self.current_cmdmode not in (1, 3):
            cur = self._MODE_NAME.get(self.current_cmdmode, f"模式{self.current_cmdmode}")
            messagebox.showwarning(
                "模式提示",
                f"顺序发送建议使用「顺序模式」(模式 1)\n"
                f"当前是「{cur}」\n\n"
                f"请在「模式」区域手动点击「1:顺序」按钮切换。"
            )

    # ==============================================
    # 运动指令
    # ==============================================
    def send_movej(self):
        """>j1,j2,j3,j4,j5,j6,j7(rail),speed  （8 个值，v3.1 新协议）"""
        try:
            j = [float(ent.get()) for ent in self.ent_joints]
            j7 = float(self.ent_j7.get())
            speed = float(self.ent_j_speed.get())
            cmd = ">" + ",".join(f"{v:g}" for v in j) + f",{j7:g},{speed:g}"
            self.send_cmd(cmd)
        except ValueError:
            messagebox.showerror("错误", "请输入有效的数字")

    def send_movel(self):
        """@X,Y,Z,A,B,C,speed  （7 个值）"""
        try:
            p = [float(ent.get()) for ent in self.ent_pose]
            speed = float(self.ent_l_speed.get())
            cmd = "@" + ",".join(f"{v:g}" for v in p) + f",{speed:g}"
            self.send_cmd(cmd)
        except ValueError:
            messagebox.showerror("错误", "请输入有效的数字")

    # ==============================================
    # 关节角 / 位姿回读同步
    # ==============================================
    def read_and_sync_from_robot(self):
        """发送 #GETJPOS 并把返回的 6 个关节角同步到滑块"""
        if not self.is_connected:
            self.log("未连接串口", "WARN")
            return
        if self._start_sync("j", "#GETJPOS"):
            self.root.after(500, self._check_sync_response)

    def read_pose_from_robot(self):
        """发送 #GETLPOS 并把返回的 6 个位姿分量同步到 MoveL 滑块"""
        if not self.is_connected:
            self.log("未连接串口", "WARN")
            return
        if self._start_sync("l", "#GETLPOS"):
            self.root.after(500, self._check_sync_response)

    def _start_sync(self, kind, cmd):
        """登记一次回读同步。回包格式相同，因此同一时刻只允许一个待同步请求。"""
        if self._pending_sync:
            self.log("上一次回读还没有返回，请稍候再试", "WARN")
            return False
        self._pending_sync = kind
        self._sync_data = None
        self.send_cmd(cmd)
        return True

    def _check_sync_response(self):
        """500ms 定时器：取走 kind + data，一并清空"""
        kind, data = self._pending_sync, self._sync_data
        self._pending_sync = None
        self._sync_data = None
        if kind is None:
            return          # 等待期间已断开或被取消，无需提示
        if data:
            if kind == "j":
                self._fill_joints(data)
                self.log("已同步机械臂关节角", "INFO")
            else:
                self._fill_pose(data)
                self.log("已同步机械臂当前位姿", "INFO")
        else:
            what = "关节角" if kind == "j" else "位姿"
            self.log(f"{what}同步超时，请确认机械臂已使能（!START）", "ERROR")

    def _fill_joints(self, nums):
        """把 7 个数写进滑块（v3.1：nums[0]=Rail(mm), nums[1..6]=J1..J6(°)）"""
        self._suppress_drag_send = True
        try:
            # 同步 J1~J6 (nums[1..6])
            for i in range(6):
                v = nums[i + 1]
                self.scl_joints[i].set(v)
                self.ent_joints[i].delete(0, tk.END)
                self.ent_joints[i].insert(0, f"{v:.2f}")
                self.lbl_joints[i].config(text=f"{v:.2f}")
            # 同步 Rail (nums[0])
            rv = nums[0]
            self.scl_j7.set(rv)
            self.ent_j7.delete(0, tk.END)
            self.ent_j7.insert(0, f"{rv:.2f}")
            self.lbl_j7.config(text=f"{rv:.2f} mm")
        finally:
            self._suppress_drag_send = False

    def _fill_pose(self, nums):
        """把 6 个位姿分量写进笛卡尔滑块 / 输入框 / 数值标签"""
        self._suppress_drag_send = True
        try:
            for i in range(6):
                self.scl_pose[i].set(nums[i])
                self.ent_pose[i].delete(0, tk.END)
                self.ent_pose[i].insert(0, f"{nums[i]:.2f}")
                self.lbl_pose[i].config(text=f"{nums[i]:.2f}")
        finally:
            self._suppress_drag_send = False

    # ==============================================
    # 点位示教
    # ==============================================
    def _current_joint_values(self):
        """读取 6 个关节 + 地轨（J7）的滑块值；非法输入时记录日志并返回 None"""
        try:
            jvals = [round(float(e.get()), 2) for e in self.ent_joints]
            j7 = round(float(self.ent_j7.get()), 2)
            return jvals + [j7]
        except ValueError:
            self.log("关节/地轨输入框中存在非法数字，请先修正后再操作", "ERROR")
            return None

    def read_current_position(self):
        """读取当前滑块值并打印到日志（v3.1：J1~J6 + Rail）"""
        pos = self._current_joint_values()
        if pos is None:
            return
        names = ["J1", "J2", "J3", "J4", "J5", "J6", "Rail"]
        msg = ", ".join(f"{n}={v}" for n, v in zip(names, pos))
        self.log(f"当前滑块: {msg}", "INFO")

    def save_current_position(self):
        """把当前 6 个关节滑块值存为一个点位"""
        pos = self._current_joint_values()
        if pos is None:
            return
        idx = len(self.stored_positions) + 1
        short_name = f"#{idx}"  # 列表用短名，日志用全量
        full_name = f"#{idx} [{','.join(str(v) for v in pos)}]"
        self.stored_positions.append((short_name, pos))
        self._pos_listbox.insert(tk.END, short_name)
        self.log(f"已存储: {full_name}", "INFO")

    def _delete_position(self):
        idx = self._pos_listbox.curselection()
        if not idx:
            return
        idx = idx[0] if isinstance(idx, tuple) else idx
        self._pos_listbox.delete(idx)
        del self.stored_positions[idx]
        self.log(f"已删除第 {idx+1} 个点位", "INFO")

    def clear_all_positions(self):
        self.stored_positions.clear()
        self._pos_listbox.delete(0, tk.END)
        self.log("已清空所有存储点位", "INFO")

    def _on_select_position(self):
        idx = self._pos_listbox.curselection()
        if not idx:
            return
        idx = idx[0]
        name, pos = self.stored_positions[idx]
        # [v3.1] pos 现在是 7 个值（6 关节 + Rail）
        self._fill_joints(pos[:7])
        full = f"{name} [{','.join(str(v) for v in pos)}]"
        self.log(f"已加载: {full}", "INFO")

    def _get_pos_speed(self):
        try:
            return float(self.ent_pos_speed.get())
        except ValueError:
            return DEFAULT_SPEED

    def send_selected_position(self):
        """发送列表中选中的点位"""
        idx = self._pos_listbox.curselection()
        if not idx:
            self.log("请先选择一个点位", "WARN")
            return
        idx = idx[0]
        _name, pos = self.stored_positions[idx]
        speed = self._get_pos_speed()
        # [v3.1] >j1,j2,j3,j4,j5,j6,j7(rail),speed （8 个值）
        self.send_cmd(">" + ",".join(f"{v:g}" for v in pos[:7]) + f",{speed:g}")

    def send_all_positions(self):
        """顺序发送全部点位：旧固件每条 ">" 执行完才回 "ok"，
        因此严格逐条下发——发第 N 条，等回 ok，再发第 N+1 条。"""
        if not self.stored_positions:
            self.log("没有存储的点位", "WARN")
            return
        if self._seq_running:
            self.log("顺序发送正在进行中，请等待完成", "WARN")
            return
        self._seq_pending = list(self.stored_positions)
        self._seq_idx = 0
        self._seq_speed = self._get_pos_speed()
        total = len(self._seq_pending)

        # 模式提示（不自动切换，不阻塞发送）
        self._warn_seqsend_mode()

        self._seq_running = True
        self.log(f"开始顺序发送 {total} 个点位 (逐条等 ok)...", "INFO")
        self._send_next_position()

    def _send_next_position(self):
        """发下一个点位，等固件回 ok 后由 RX 线程再次调用"""
        if self._seq_idx >= len(self._seq_pending):
            return
        _name, pos = self._seq_pending[self._seq_idx]
        # 必须在 send_cmd 之前置位标志，避免固件 ok 先到导致漏接
        self._seq_running = True
        # [v3.1] >j1,j2,j3,j4,j5,j6,j7(rail),speed （8 个值）
        self.send_cmd(">" + ",".join(f"{v:g}" for v in pos[:7]) + f",{self._seq_speed:g}")

    # ==============================================
    # 命令历史
    # ==============================================
    def _send_with_history(self):
        cmd = self.ent_custom_cmd.get().strip()
        if cmd:
            if not self.cmd_history or self.cmd_history[-1] != cmd:
                self.cmd_history.append(cmd)
            self.cmd_history_idx[-1] = len(self.cmd_history) - 1
            self.send_cmd(cmd)
            self.ent_custom_cmd.delete(0, tk.END)

    def _history_up(self, event):
        if not self.cmd_history:
            return
        idx = self.cmd_history_idx[-1]
        if idx > 0:
            idx -= 1
            self.cmd_history_idx[-1] = idx
            self.ent_custom_cmd.delete(0, tk.END)
            self.ent_custom_cmd.insert(0, self.cmd_history[idx])

    def _history_down(self, event):
        if not self.cmd_history:
            return
        idx = self.cmd_history_idx[-1]
        if idx < len(self.cmd_history) - 1:
            idx += 1
            self.cmd_history_idx[-1] = idx
            self.ent_custom_cmd.delete(0, tk.END)
            self.ent_custom_cmd.insert(0, self.cmd_history[idx])
        else:
            self.cmd_history_idx[-1] = len(self.cmd_history)
            self.ent_custom_cmd.delete(0, tk.END)


if __name__ == "__main__":
    root = tk.Tk()
    app = RobotSerialAssistantOld(root)
    root.mainloop()
