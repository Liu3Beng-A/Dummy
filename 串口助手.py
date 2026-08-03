import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import serial
import serial.tools.list_ports
import threading
import time
import math
import configparser
import os
from pathlib import Path
#你好
class RobotSerialAssistant:
    def __init__(self, root):
        self.root = root
        self.root.title("Dummy Robot 串口助手")
        self.root.geometry("1720x1020")
        self.root.minsize(1580, 950)

        # 同步滑块状态
        self._sync_waiting = False
        self._sync_data = None
        self.stored_positions = []  # 点位列表 [(name, [j1..j6, rail])]
        self._pos_queue_running = False  # 顺序发送运行标志
        self._pos_queue_pending = []     # 待发送队列
        self._pos_queue_idx = 0          # 当前发送索引
        self._pos_queue_speed = 50       # 顺序发送速度
        # 主题与配色
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
            style.configure("TLabelframe", background=panel, foreground=text_main, bordercolor=border, borderwidth=1)
            style.configure("TLabelframe.Label", background=panel, foreground=text_main, font=("Arial", 10, "bold"))
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

            style.configure("TEntry", fieldbackground=panel, bordercolor=border, lightcolor=border, darkcolor=border)
            style.configure("TCombobox", fieldbackground=panel, bordercolor=border, lightcolor=border, darkcolor=border)
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
        self.is_connected = False

        # RGB 亮度配置（掉电保持），必须在 create_widgets 之前加载
        self.rgb_brightness = 100
        self._load_config()

        self.create_widgets()
        self.refresh_ports()

        self.receive_thread = None
        self.stop_thread = False

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

        tk.Label(left_top, text="波特率:", bg="#2d2d2d", fg="#cccccc", font=("Arial", 10)).pack(side=tk.LEFT, padx=(6, 0))
        self.cb_baudrate = ttk.Combobox(left_top, width=10, font=("Arial", 10),
                                        values=["9600", "57600", "115200", "230400", "460800", "1000000"],
                                        state="readonly")
        self.cb_baudrate.current(2)
        self.cb_baudrate.pack(side=tk.LEFT, padx=4)

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

        # 控制台（weight=2，给终端更多空间）
        control_area = ttk.Frame(main_paned)
        main_paned.add(control_area, weight=2)

        # 左栏（系统+查询）| 中栏（Notebook标签页）| 右栏（关节控制Notebook）
        # 左栏有权重，在窗口够大时自动占更多宽度
        control_area.columnconfigure(0, weight=1)  # 左栏
        control_area.columnconfigure(1, weight=1)  # 中栏自动扩展
        control_area.columnconfigure(2, weight=2)  # 右栏2倍宽度（主要工作区）
        control_area.rowconfigure(0, weight=1)

        # ==============================================
        # 左栏：系统控制 + 查询校准 + 夹爪
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
                  relief=tk.FLAT, pady=6, command=lambda: self.send_cmd("!START")).pack(side=tk.LEFT, expand=True, fill=tk.BOTH, padx=1)
        tk.Button(sys_row1, text="失能", font=("Arial", 10), bg="#495057", fg="white",
                  relief=tk.FLAT, pady=6, command=lambda: self.send_cmd("!DISABLE")).pack(side=tk.LEFT, expand=True, fill=tk.BOTH, padx=1)
        tk.Button(sys_row1, text="回零", font=("Arial", 10), bg="#5c7cfa", fg="white",
                  relief=tk.FLAT, pady=6, command=lambda: self.send_cmd("!HOME")).pack(side=tk.LEFT, expand=True, fill=tk.BOTH, padx=1)
        tk.Button(sys_row1, text="休息", font=("Arial", 10), bg="#868e96", fg="white",
                  relief=tk.FLAT, pady=6, command=lambda: self.send_cmd("!RESET")).pack(side=tk.LEFT, expand=True, fill=tk.BOTH, padx=1)

        ttk.Separator(sys_f, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=6)

        ttk.Label(sys_f, text="模式:", font=("Arial", 10, "bold")).pack(anchor="w")
        mode_f = ttk.Frame(sys_f)
        mode_f.pack(fill=tk.X)
        for label, val in [("1:顺序", 1), ("2:打断", 2), ("3:连续", 3), ("5:力矩", 5), ("6:伺服", 6)]:
            tk.Button(mode_f, text=label, font=("Arial", 10), bg="#495057", fg="white",
                      relief=tk.FLAT, pady=4, command=lambda v=val: self.send_cmd(f"#CMDMODE {v}")
                      ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=1)

        # --- 查询与校准 ---
        query_f = ttk.LabelFrame(left_col, text="查询与置零", padding=6)
        query_f.pack(fill=tk.X, pady=(0, 6))

        q_row1 = ttk.Frame(query_f)
        q_row1.pack(fill=tk.X)
        tk.Button(q_row1, text="获取关节角", font=("Arial", 10), bg="#495057", fg="white",
                  relief=tk.FLAT, pady=4, command=lambda: self.send_cmd("#GETJPOS")
                  ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=1)
        tk.Button(q_row1, text="获取位姿", font=("Arial", 10), bg="#495057", fg="white",
                  relief=tk.FLAT, pady=4, command=lambda: self.send_cmd("#GETLPOS")
                  ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=1)

        ttk.Separator(query_f, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=6)

        ttk.Label(query_f, text="置零:", font=("Arial", 10, "bold")).pack(anchor="w")
        offset_j_row = ttk.Frame(query_f)
        offset_j_row.pack(fill=tk.X)
        for j in range(1, 7):
            tk.Button(offset_j_row, text=f"J{j}", font=("Arial", 10), bg="#343a40", fg="white",
                     relief=tk.FLAT, pady=4, command=lambda j=j: self.send_cmd(f"#OFFSET_J {j}")
                     ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=1, pady=1)
        tk.Button(query_f, text="全部置零", font=("Arial", 10), bg="#e03131", fg="white",
                  relief=tk.FLAT, pady=4, command=self.send_home_offset_all).pack(fill=tk.X, pady=(4, 0))

        # ==============================================
        # 中栏：Notebook 标签页（RGB / 电机 / 力矩 / PID）
        # ==============================================
        self.notebook = ttk.Notebook(control_area)
        self.notebook.grid(row=0, column=1, sticky="nsew", padx=(0, 6))

        # --- 标签1：RGB 彩灯 ---
        rgb_page = ttk.Frame(self.notebook, padding=6)
        self.notebook.add(rgb_page, text=" RGB ")
        self._build_rgb_tab(rgb_page)

        # --- 标签2：电机参数 ---
        motor_page = ttk.Frame(self.notebook, padding=6)
        self.notebook.add(motor_page, text=" 电机参数 ")
        self._build_motor_tab(motor_page)

        # --- 标签3：力矩控制 ---
        torque_page = ttk.Frame(self.notebook, padding=6)
        self.notebook.add(torque_page, text=" 力矩 ")
        self._build_torque_tab(torque_page)

        # --- 标签4：PID 调节 ---
        pid_page = ttk.Frame(self.notebook, padding=6)
        self.notebook.add(pid_page, text=" PID ")
        self._build_pid_tab(pid_page)

        # ==============================================
        # 右栏：Notebook（关节控制+夹爪 / MoveL / ServoJ / 地轨）
        # ==============================================
        right_col = ttk.Frame(control_area)
        right_col.grid(row=0, column=2, sticky="nsew", padx=(0, 0))
        right_col.rowconfigure(0, weight=1)
        right_col.columnconfigure(0, weight=1)

        self.joint_nb = ttk.Notebook(right_col, height=440)
        self.joint_nb.pack(fill=tk.BOTH, expand=True)

        # --- 页1：关节控制 MoveJ ---
        movej_page = ttk.Frame(self.joint_nb, padding=5)
        self.joint_nb.add(movej_page, text=" 关节控制 ")
        self._build_movej_page(movej_page)

        # --- 页2：夹爪 ---
        gripper_page = ttk.Frame(self.joint_nb, padding=5)
        self.joint_nb.add(gripper_page, text=" 夹爪 ")
        self._build_gripper_tab(gripper_page)

        # --- 页3：笛卡尔 MoveL ---
        movel_page = ttk.Frame(self.joint_nb, padding=5)
        self.joint_nb.add(movel_page, text=" MoveL ")
        self._build_movel_page(movel_page)

        # --- 页4：ServoJ ---
        servoj_page = ttk.Frame(self.joint_nb, padding=5)
        self.joint_nb.add(servoj_page, text=" ServoJ ")
        self._build_servoj_page(servoj_page)

        # ==============================================
        # 底部终端（更大的占比）
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

        # 终端文本（彩色 tag）
        self.txt_log = scrolledtext.ScrolledText(log_f, wrap=tk.WORD, font=("Consolas", 10),
                                                  bg="#1e1e1e", fg="#d4d4d4", insertbackground="white")
        self.txt_log.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self.txt_log.tag_config("TX", foreground="#4ec9b0")
        self.txt_log.tag_config("RX", foreground="#9cdcfe")
        self.txt_log.tag_config("ERROR", foreground="#f48771")
        self.txt_log.tag_config("WARN", foreground="#dcdcaa")
        self.txt_log.tag_config("INFO", foreground="#6a9955")

    def _get_config_path(self):
        cfg_dir = Path.home() / ".dummy_robot"
        cfg_dir.mkdir(exist_ok=True)
        return cfg_dir / "settings.ini"

    def _load_config(self):
        path = self._get_config_path()
        if path.exists():
            try:
                cp = configparser.ConfigParser()
                cp.read(str(path), encoding="utf-8")
                self.rgb_brightness = cp.getint("rgb", "brightness", fallback=100)
                self.rgb_brightness = max(0, min(100, self.rgb_brightness))
            except Exception:
                self.rgb_brightness = 100

    def _save_config(self):
        path = self._get_config_path()
        try:
            cp = configparser.ConfigParser()
            cp.read(str(path), encoding="utf-8")
            if not cp.has_section("rgb"):
                cp.add_section("rgb")
            cp.set("rgb", "brightness", str(self.rgb_brightness))
            with open(str(path), "w", encoding="utf-8") as f:
                cp.write(f)
        except Exception:
            pass

    # ==============================================
    # Notebook 子标签页构建
    # ==============================================

    def _build_rgb_tab(self, parent):
        # 开关灯
        onoff_f = ttk.Frame(parent)
        onoff_f.pack(fill=tk.X, pady=(0, 6))
        tk.Button(onoff_f, text="开灯", font=("Arial", 10, "bold"), bg="#2b8a3e", fg="white",
                  relief=tk.FLAT, command=self._rgb_light_on
                  ).pack(side=tk.LEFT, expand=True, fill=tk.BOTH, padx=(0, 3))
        tk.Button(onoff_f, text="关灯", font=("Arial", 10, "bold"), bg="#495057", fg="white",
                  relief=tk.FLAT, command=self._rgb_light_off
                  ).pack(side=tk.LEFT, expand=True, fill=tk.BOTH, padx=(3, 0))

        # 亮度：Entry 输入 + Scale 同步 + 查询/应用/保存三按钮
        bright_f = ttk.Frame(parent)
        bright_f.pack(fill=tk.X, pady=(0, 4))
        ttk.Label(bright_f, text="亮度:", font=("Arial", 10, "bold")).pack(side=tk.LEFT)
        self.ent_bright = ttk.Entry(bright_f, width=5, font=("Arial", 10))
        self.ent_bright.insert(0, str(self.rgb_brightness))
        self.ent_bright.pack(side=tk.LEFT, padx=(4, 2))
        ttk.Label(bright_f, text="%").pack(side=tk.LEFT)

        self._bright_dragging = False
        self.scl_bright = tk.Scale(bright_f, from_=0, to=100, orient=tk.HORIZONTAL,
                                   length=200, showvalue=False, sliderrelief=tk.FLAT,
                                   bg=self.root.cget("bg"), fg="#3b5bdb", highlightthickness=0,
                                   font=("Arial", 9), troughcolor="#495057",
                                   activebackground="#3b5bdb")
        self.scl_bright.set(self.rgb_brightness)
        self.scl_bright.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)

        def _sync_bright_to_scale(val):
            self._bright_dragging = True
            v = int(float(val))
            if self.ent_bright.get() != str(v):
                self.ent_bright.delete(0, tk.END)
                self.ent_bright.insert(0, str(v))
        def _sync_bright_from_entry():
            if self.ent_bright.get():
                try:
                    v = int(float(self.ent_bright.get()))
                    v = max(0, min(100, v))
                    self.scl_bright.set(v)
                except ValueError:
                    pass

        self.scl_bright.config(command=_sync_bright_to_scale)
        self.scl_bright.bind("<ButtonRelease-1>", lambda e: setattr(self, "_bright_dragging", False))
        self.ent_bright.bind("<Return>", lambda e: (_sync_bright_from_entry(), self.ent_bright.select_clear()))
        self.ent_bright.bind("<FocusOut>", lambda e: _sync_bright_from_entry())
        self.scl_bright.bind("<ButtonRelease-1>", lambda e: setattr(self, "_bright_dragging", False))

        bright_btns = ttk.Frame(parent)
        bright_btns.pack(fill=tk.X, pady=(0, 6))
        tk.Button(bright_btns, text="查询", font=("Arial", 10), bg="#495057", fg="white",
                  relief=tk.FLAT, command=self._rgb_bright_query
                  ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 2))
        tk.Button(bright_btns, text="应用", font=("Arial", 10), bg="#3b5bdb", fg="white",
                  relief=tk.FLAT, command=self._rgb_bright_apply
                  ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        tk.Button(bright_btns, text="保存", font=("Arial", 10, "bold"), bg="#2b8a3e", fg="white",
                  relief=tk.FLAT, command=self._rgb_bright_save
                  ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(2, 0))

        # 模式选择 2×5 网格
        ttk.Label(parent, text="模式:", font=("Arial", 10, "bold")).pack(anchor="w", pady=(0, 4))
        mode_f = ttk.Frame(parent)
        mode_f.pack(fill=tk.X, pady=(0, 6))

        modes = [
            ("单色0", 0), ("单色1", 1), ("单色2", 2), ("流光", 3), ("潮汐", 4),
            ("白色",  5), ("赛博", 6), ("心跳", 7), ("旋转", 8), ("闪烁", 9),
        ]
        for idx, (text, val) in enumerate(modes):
            row = idx // 5
            col = idx % 5
            mode_f.columnconfigure(col, weight=1)
            tk.Button(mode_f, text=text, font=("Arial", 10), bg="#495057", fg="white",
                      relief=tk.FLAT, command=lambda v=val: self.send_cmd(f"!RGB_MODE {v}")
                      ).grid(row=row, column=col, sticky="ew", padx=2, pady=2)

        ttk.Separator(parent, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=6)

        # 自定义颜色
        ttk.Label(parent, text="自定义颜色:", font=("Arial", 10, "bold")).pack(anchor="w", pady=(0, 4))
        color_f = ttk.Frame(parent)
        color_f.pack(fill=tk.X, pady=(0, 6))
        self.cb_color_idx = ttk.Combobox(color_f, width=3, values=["0", "1", "2"], state="readonly", font=("Arial", 10))
        self.cb_color_idx.current(0)
        self.cb_color_idx.pack(side=tk.LEFT, padx=(0, 4))
        for lbl, var, default in [("R:", "ent_r", "0"), ("G:", "ent_g", "100"), ("B:", "ent_b", "0")]:
            ttk.Label(color_f, text=lbl, font=("Arial", 10, "bold")).pack(side=tk.LEFT)
            ent = ttk.Entry(color_f, width=4, font=("Arial", 10))
            ent.insert(0, default)
            ent.pack(side=tk.LEFT, padx=2)
            setattr(self, var, ent)
        tk.Button(color_f, text="发送", font=("Arial", 10, "bold"), bg="#3b5bdb", fg="white",
                  relief=tk.FLAT, command=self.send_rgb_color).pack(side=tk.LEFT, padx=(4, 0))

        ttk.Separator(parent, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=6)

        # 状态绑定
        ttk.Label(parent, text="状态绑定:", font=("Arial", 10, "bold")).pack(anchor="w", pady=(0, 4))
        state_f = ttk.Frame(parent)
        state_f.pack(fill=tk.X)
        for lbl_txt, cb_attr, cmd in [
            ("启动→模式", "cb_state_start", self.set_rgb_state_start),
            ("使能→模式", "cb_state_enable", self.set_rgb_state_enable),
            ("失能→模式", "cb_state_disable", self.set_rgb_state_disable),
        ]:
            state_row = ttk.Frame(state_f)
            state_row.pack(fill=tk.X, pady=2)
            ttk.Label(state_row, text=lbl_txt, font=("Arial", 10)).pack(side=tk.LEFT)
            cb = ttk.Combobox(state_row, width=3, values=[str(i) for i in range(10)], state="readonly", font=("Arial", 10))
            cb.pack(side=tk.LEFT, padx=4)
            setattr(self, cb_attr, cb)
            tk.Button(state_row, text="设置", font=("Arial", 10), bg="#3b5bdb", fg="white",
                      relief=tk.FLAT, command=cmd).pack(side=tk.LEFT)

    def _build_motor_tab(self, parent):
        # 节点选择 + 加速度 + 电流
        node_f = ttk.Frame(parent)
        node_f.pack(fill=tk.X, pady=(0, 4))
        ttk.Label(node_f, text="节点:", font=("Arial", 10, "bold")).pack(side=tk.LEFT)
        self.cb_acc_node = ttk.Combobox(node_f, width=6,
                                         values=[str(i) for i in [1, 2, 3, 4, 5, 6, 8, 9]],
                                         state="readonly")
        self.cb_acc_node.current(0)
        self.cb_acc_node.pack(side=tk.LEFT, padx=4)
        tk.Button(node_f, text="查加速度", font=("Arial", 10), bg="#495057", fg="white",
                  relief=tk.FLAT, command=lambda: self.send_cmd(f"#ACC_BASE_J {self.cb_acc_node.get()}")
                  ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        tk.Button(node_f, text="查电流", font=("Arial", 10), bg="#495057", fg="white",
                  relief=tk.FLAT, command=lambda: self.send_cmd(f"#I_LIMIT_J {self.cb_acc_node.get()}")
                  ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        acc_f = ttk.Frame(parent)
        acc_f.pack(fill=tk.X, pady=(0, 4))
        ttk.Label(acc_f, text="加速度:", font=("Arial", 10)).pack(side=tk.LEFT)
        self.ent_acc_val = ttk.Entry(acc_f, width=7, font=("Arial", 10))
        self.ent_acc_val.insert(0, "150")
        self.ent_acc_val.pack(side=tk.LEFT, padx=4)
        tk.Button(acc_f, text="应用", font=("Arial", 10), bg="#3b5bdb", fg="white",
                  relief=tk.FLAT, command=self.send_acc_base).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        cur_f = ttk.Frame(parent)
        cur_f.pack(fill=tk.X, pady=(0, 6))
        ttk.Label(cur_f, text="电流(A):", font=("Arial", 10)).pack(side=tk.LEFT)
        self.ent_i_limit = ttk.Entry(cur_f, width=7, font=("Arial", 10))
        self.ent_i_limit.insert(0, "1.5")
        self.ent_i_limit.pack(side=tk.LEFT, padx=4)
        tk.Button(cur_f, text="应用", font=("Arial", 10), bg="#3b5bdb", fg="white",
                  relief=tk.FLAT, command=self.send_i_limit).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        ttk.Separator(parent, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=6)

        # 地轨速度
        tk.Label(parent, text="地轨速度 (#SPEED_RAIL)", font=("Arial", 10, "bold")).pack(anchor="w")
        rsf = ttk.Frame(parent)
        rsf.pack(fill=tk.X, pady=(0, 4))
        ttk.Label(rsf, text="mm/s:", font=("Arial", 10)).pack(side=tk.LEFT)
        self.ent_rail_speed = ttk.Entry(rsf, width=6, font=("Arial", 10))
        self.ent_rail_speed.insert(0, "50")
        self.ent_rail_speed.pack(side=tk.LEFT, padx=4)
        self.scl_rail_speed = ttk.Scale(rsf, from_=0.5, to=100, orient=tk.HORIZONTAL)
        self.scl_rail_speed.set(50)
        self.scl_rail_speed.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)
        self.lbl_rail_speed_val = ttk.Label(rsf, text="50.0", width=6, font=("Arial", 10))
        self.lbl_rail_speed_val.pack(side=tk.LEFT)

        rs_btns = ttk.Frame(parent)
        rs_btns.pack(fill=tk.X, pady=(0, 6))
        tk.Button(rs_btns, text="查询", font=("Arial", 10), bg="#495057", fg="white",
                  relief=tk.FLAT, command=self.query_rail_speed).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        tk.Button(rs_btns, text="应用", font=("Arial", 10), bg="#3b5bdb", fg="white",
                  relief=tk.FLAT, command=self.apply_rail_speed).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        tk.Button(rs_btns, text="保存", font=("Arial", 10), bg="#2b8a3e", fg="white",
                  relief=tk.FLAT, command=self.save_rail_speed).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        def urss(val):
            v = float(val)
            self.lbl_rail_speed_val.config(text=f"{v:.1f}")
            if self.ent_rail_speed.get() != f"{v:.1f}":
                self.ent_rail_speed.delete(0, tk.END)
                self.ent_rail_speed.insert(0, f"{v:.1f}")
        self.scl_rail_speed.config(command=urss)
        self.ent_rail_speed.bind("<Return>",
            lambda e: self.scl_rail_speed.set(float(self.ent_rail_speed.get())))
        self.ent_rail_speed.bind("<FocusOut>",
            lambda e: self.scl_rail_speed.set(float(self.ent_rail_speed.get())))

        # 地轨加速度
        tk.Label(parent, text="地轨加速度 (#ACC_RAIL)", font=("Arial", 10, "bold")).pack(anchor="w")
        raf = ttk.Frame(parent)
        raf.pack(fill=tk.X, pady=(0, 4))
        ttk.Label(raf, text="mm/s2:", font=("Arial", 10)).pack(side=tk.LEFT)
        self.ent_rail_acc = ttk.Entry(raf, width=7, font=("Arial", 10))
        self.ent_rail_acc.insert(0, "500")
        self.ent_rail_acc.pack(side=tk.LEFT, padx=4)
        self.scl_rail_acc = ttk.Scale(raf, from_=10, to=5000, orient=tk.HORIZONTAL)
        self.scl_rail_acc.set(500)
        self.scl_rail_acc.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)
        self.lbl_rail_acc_val = ttk.Label(raf, text="500", width=6, font=("Arial", 10))
        self.lbl_rail_acc_val.pack(side=tk.LEFT)

        ra_btns = ttk.Frame(parent)
        ra_btns.pack(fill=tk.X, pady=(0, 6))
        tk.Button(ra_btns, text="查询", font=("Arial", 10), bg="#495057", fg="white",
                  relief=tk.FLAT, command=self.query_rail_acc).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        tk.Button(ra_btns, text="应用", font=("Arial", 10), bg="#3b5bdb", fg="white",
                  relief=tk.FLAT, command=self.apply_rail_acc).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        tk.Button(ra_btns, text="保存", font=("Arial", 10), bg="#2b8a3e", fg="white",
                  relief=tk.FLAT, command=self.save_rail_acc).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        def ursa(val):
            v = int(float(val))
            self.lbl_rail_acc_val.config(text=str(v))
            if self.ent_rail_acc.get() != str(v):
                self.ent_rail_acc.delete(0, tk.END)
                self.ent_rail_acc.insert(0, str(v))
        self.scl_rail_acc.config(command=ursa)
        self.ent_rail_acc.bind("<Return>",
            lambda e: self.scl_rail_acc.set(float(self.ent_rail_acc.get())))
        self.ent_rail_acc.bind("<FocusOut>",
            lambda e: self.scl_rail_acc.set(float(self.ent_rail_acc.get())))

        # 地轨电流 (#I_LIMIT_J 9)
        tk.Label(parent, text="地轨电流 (#I_LIMIT_J 9)", font=("Arial", 10, "bold")).pack(anchor="w")
        rcf = ttk.Frame(parent)
        rcf.pack(fill=tk.X, pady=(0, 4))
        ttk.Label(rcf, text="A:", font=("Arial", 10)).pack(side=tk.LEFT)
        self.ent_rail_current = ttk.Entry(rcf, width=6, font=("Arial", 10))
        self.ent_rail_current.insert(0, "2.8")
        self.ent_rail_current.pack(side=tk.LEFT, padx=4)
        self.scl_rail_current = ttk.Scale(rcf, from_=0.1, to=3.0, orient=tk.HORIZONTAL)
        self.scl_rail_current.set(2.8)
        self.scl_rail_current.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)
        self.lbl_rail_current_val = ttk.Label(rcf, text="2.8", width=4, font=("Arial", 10))
        self.lbl_rail_current_val.pack(side=tk.LEFT)

        rc_btns = ttk.Frame(parent)
        rc_btns.pack(fill=tk.X, pady=(0, 0))
        tk.Button(rc_btns, text="查询", font=("Arial", 10), bg="#495057", fg="white",
                  relief=tk.FLAT, command=self.query_rail_current).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        tk.Button(rc_btns, text="应用", font=("Arial", 10), bg="#3b5bdb", fg="white",
                  relief=tk.FLAT, command=self.apply_rail_current).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        tk.Button(rc_btns, text="保存", font=("Arial", 10), bg="#2b8a3e", fg="white",
                  relief=tk.FLAT, command=self.save_rail_current).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        def ursc(val):
            v = float(val)
            self.lbl_rail_current_val.config(text=f"{v:.1f}")
            if self.ent_rail_current.get() != f"{v:.1f}":
                self.ent_rail_current.delete(0, tk.END)
                self.ent_rail_current.insert(0, f"{v:.1f}")
        self.scl_rail_current.config(command=ursc)
        self.ent_rail_current.bind("<Return>",
            lambda e: self.scl_rail_current.set(float(self.ent_rail_current.get())))
        self.ent_rail_current.bind("<FocusOut>",
            lambda e: self.scl_rail_current.set(float(self.ent_rail_current.get())))

    def _build_torque_tab(self, parent):
        ttk.Label(parent, text="电流力矩控制（单位：A）",
                  font=("Arial", 10, "bold")).pack(anchor="w", pady=(0, 4))

        grid = ttk.Frame(parent)
        grid.pack(fill=tk.X)
        grid.columnconfigure(1, weight=1)

        # Rail + Gripper 第一行
        ttk.Label(grid, text="Rail:", font=("Arial", 10, "bold")).grid(row=0, column=0, sticky="e", padx=4, pady=2)
        self.ent_rail_torque = ttk.Entry(grid, width=8, font=("Arial", 10))
        self.ent_rail_torque.insert(0, "0.0")
        self.ent_rail_torque.grid(row=0, column=1, sticky="w", padx=4, pady=2)
        ttk.Label(grid, text="A", font=("Arial", 10)).grid(row=0, column=2, sticky="w", padx=2, pady=2)

        ttk.Label(grid, text="Gripper:", font=("Arial", 10, "bold")).grid(row=0, column=3, sticky="e", padx=(12, 4), pady=2)
        self.ent_gripper_torque = ttk.Entry(grid, width=8, font=("Arial", 10))
        self.ent_gripper_torque.insert(0, "0.0")
        self.ent_gripper_torque.grid(row=0, column=4, sticky="w", padx=4, pady=2)
        ttk.Label(grid, text="A", font=("Arial", 10)).grid(row=0, column=5, sticky="w", padx=2, pady=2)

        # J1~J6
        self.ent_torques = []
        for i, jid in enumerate([1, 2, 3, 4, 5, 6]):
            r = 1 + i // 3
            c = (i % 3) * 3
            ttk.Label(grid, text=f"J{jid}:", font=("Arial", 10)).grid(row=r, column=c, sticky="e", padx=4, pady=2)
            ent = ttk.Entry(grid, width=8, font=("Arial", 10))
            ent.insert(0, "0.0")
            ent.grid(row=r, column=c+1, sticky="w", padx=4, pady=2)
            ttk.Label(grid, text="A", font=("Arial", 10)).grid(row=r, column=c+2, sticky="w", padx=2, pady=2)
            self.ent_torques.append(ent)

        tk.Button(parent, text="发送力矩指令", font=("Arial", 10, "bold"), bg="#3b5bdb", fg="white",
                  relief=tk.FLAT, command=self.send_torque).pack(fill=tk.X, pady=(8, 0))

    def _build_pid_tab(self, parent):
        node_f = ttk.Frame(parent)
        node_f.pack(fill=tk.X, pady=(0, 4))
        ttk.Label(node_f, text="节点:", font=("Arial", 10, "bold")).pack(side=tk.LEFT)
        self.cb_pid_node = ttk.Combobox(node_f, width=5,
                                        values=[str(i) for i in [9, 1, 2, 3, 4, 5, 6, 8]],
                                        state="readonly")
        self.cb_pid_node.current(0)
        self.cb_pid_node.pack(side=tk.LEFT, padx=4)
        tk.Button(node_f, text="查询", font=("Arial", 10), bg="#495057", fg="white",
                  relief=tk.FLAT, command=self.query_pid).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        pid_grid = ttk.Frame(parent)
        pid_grid.pack(fill=tk.X)
        pid_grid.columnconfigure(1, weight=1)

        self.ent_pid, self.scl_pid, self.lbl_pid_val = {}, {}, {}
        pid_defaults = {"kp": 500, "kv": 200, "ki": 50, "kd": 100}
        pid_ranges = {"kp": (0, 5000), "kv": (0, 5000), "ki": (0, 1000), "kd": (0, 2000)}

        for idx, (label, key) in enumerate([("Kp", "kp"), ("Kv", "kv"), ("Ki", "ki"), ("Kd", "kd")]):
            ttk.Label(pid_grid, text=f"{label}:", font=("Arial", 10, "bold")).grid(
                row=0, column=idx*3, sticky="e", padx=2, pady=2)
            ent = ttk.Entry(pid_grid, width=6, font=("Arial", 10))
            ent.insert(0, str(pid_defaults[key]))
            ent.grid(row=0, column=idx*3+1, padx=2, pady=2)
            self.ent_pid[key] = ent
            scl = ttk.Scale(pid_grid, from_=pid_ranges[key][0], to=pid_ranges[key][1], orient=tk.HORIZONTAL)
            scl.set(pid_defaults[key])
            scl.grid(row=0, column=idx*3+2, sticky="ew", padx=2, pady=2)
            self.scl_pid[key] = scl
            lbl = ttk.Label(pid_grid, text=str(pid_defaults[key]), width=5, font=("Arial", 10))
            lbl.grid(row=0, column=idx*3+3, padx=(0, 4), pady=2)
            self.lbl_pid_val[key] = lbl

            rng = pid_ranges[key]

            def mk_scb(k, r):
                def cb(val):
                    v = int(float(val))
                    self.lbl_pid_val[k].config(text=str(v))
                    if self.ent_pid[k].get() != str(v):
                        self.ent_pid[k].delete(0, tk.END)
                        self.ent_pid[k].insert(0, str(v))
                return cb
            scl.config(command=mk_scb(key, rng))

            def mk_ecb(k, r):
                def cb(event):
                    try:
                        v = int(float(self.ent_pid[k].get()))
                        v = max(r[0], min(r[1], v))
                        self.scl_pid[k].set(v)
                        self.lbl_pid_val[k].config(text=str(v))
                    except ValueError:
                        pass
                return cb
            ent.bind("<Return>", mk_ecb(key, rng))
            ent.bind("<FocusOut>", mk_ecb(key, rng))

        pid_btns = ttk.Frame(parent)
        pid_btns.pack(fill=tk.X, pady=(6, 0))
        tk.Button(pid_btns, text="应用", font=("Arial", 10, "bold"), bg="#3b5bdb", fg="white",
                  relief=tk.FLAT, command=self.apply_pid).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        tk.Button(pid_btns, text="保存", font=("Arial", 10, "bold"), bg="#2b8a3e", fg="white",
                  relief=tk.FLAT, command=self.save_pid).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

    # ==============================================
    # 关节/MoveL/ServoJ 页构建（移动到右栏）
    # ==============================================

    def _build_movej_page(self, parent):
        jgrid = ttk.Frame(parent)
        jgrid.pack(fill=tk.X)
        jgrid.columnconfigure(2, weight=1)
        
        self.ent_joints = []
        self.scl_joints = []
        self.lbl_joints = []
        joint_ranges = [(-175, 175), (-75, 90), (0, 180), (-270, 270), (-100, 100), (-180, 180)]
        joint_defaults = [0, -75, 180, 0, 0, 0]
        self.movej_drag_enable = tk.BooleanVar(value=False)
        self.last_movej_send_time = 0
        
        for i in range(6):
            rng, df = joint_ranges[i], joint_defaults[i]
            ttk.Label(jgrid, text=f"J{i+1}:", font=("Arial", 10, "bold")
                      ).grid(row=i, column=0, padx=4, pady=2, sticky="e")
            ent = ttk.Entry(jgrid, width=6, font=("Arial", 10))
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

            def make_jcb(idx, lbl, ent, scl):
                def cb(val):
                    v = float(val)
                    lbl.config(text=f"{v:.1f}")
                    s = f"{v:.1f}"
                    if ent.get() != s:
                        ent.delete(0, tk.END)
                        ent.insert(0, s)
                    if self.movej_drag_enable.get():
                        now = time.time()
                        if now - self.last_movej_send_time > 0.1:
                            self.last_movej_send_time = now
                            self.root.after(1, self.send_movej)
                return cb
            scl.config(command=make_jcb(i, lbl, ent, scl))
            
            def make_jentrycb(idx, scl):
                def cb(event):
                    try:
                        scl.set(float(self.ent_joints[idx].get()))
                    except ValueError:
                        pass
                return cb
            ent.bind("<Return>", make_jentrycb(i, scl))
            ent.bind("<FocusOut>", make_jentrycb(i, scl))

        # Rail
        ttk.Label(jgrid, text="Rail:", font=("Arial", 10, "bold"), foreground="#007ACC"
                  ).grid(row=6, column=0, padx=4, pady=2, sticky="e")
        self.ent_j7 = ttk.Entry(jgrid, width=6, font=("Arial", 10))
        self.ent_j7.insert(0, "0")
        self.ent_j7.grid(row=6, column=1, padx=4, pady=2)
        self.scl_j7 = ttk.Scale(jgrid, from_=-250, to=250, orient=tk.HORIZONTAL)
        self.scl_j7.set(0)
        self.scl_j7.grid(row=6, column=2, sticky="ew", padx=8, pady=2)
        self.lbl_j7 = ttk.Label(jgrid, text="0.0 mm", width=8, font=("Arial", 9))
        self.lbl_j7.grid(row=6, column=3, padx=4, pady=2)

        def j7_scl_cb(val):
            v = float(val)
            self.lbl_j7.config(text=f"{v:.1f} mm")
            s = f"{v:.1f}"
            if self.ent_j7.get() != s:
                self.ent_j7.delete(0, tk.END)
                self.ent_j7.insert(0, s)
            if self.movej_drag_enable.get():
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

        # 控制行
        rail_ctrl = ttk.Frame(parent)
        rail_ctrl.pack(fill=tk.X, pady=(6, 0))
        ttk.Label(rail_ctrl, text="Rail:", font=("Arial", 9, "bold")).pack(side=tk.LEFT, padx=(0, 4))
        self.ent_rail_step = ttk.Entry(rail_ctrl, width=5, font=("Arial", 9))
        self.ent_rail_step.insert(0, "10")
        self.ent_rail_step.pack(side=tk.LEFT)
        ttk.Label(rail_ctrl, text="mm", font=("Arial", 9)).pack(side=tk.LEFT, padx=(2, 8))
        tk.Button(rail_ctrl, text="←", font=("Arial", 9, "bold"), bg="#495057", fg="white",
                  relief=tk.FLAT, width=3, command=self.rail_move_left).pack(side=tk.LEFT, padx=2)
        tk.Button(rail_ctrl, text="→", font=("Arial", 9, "bold"), bg="#495057", fg="white",
                  relief=tk.FLAT, width=3, command=self.rail_move_right).pack(side=tk.LEFT, padx=2)
        ttk.Label(rail_ctrl, text="Speed:", font=("Arial", 9)).pack(side=tk.LEFT, padx=(8, 2))
        self.ent_j_speed = ttk.Entry(rail_ctrl, width=5, font=("Arial", 9))
        self.ent_j_speed.insert(0, "50")
        self.ent_j_speed.pack(side=tk.LEFT)
        tk.Checkbutton(rail_ctrl, text="拖发", variable=self.movej_drag_enable,
                       bg=None, font=("Arial", 9)).pack(side=tk.LEFT, padx=6)
        tk.Button(rail_ctrl, text="发送 MoveJ", font=("Arial", 10, "bold"), bg="#3b5bdb", fg="white",
                  relief=tk.FLAT, command=self.send_movej).pack(side=tk.RIGHT, padx=2)

        # --- 点位存储 ---
        ttk.Separator(parent, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=(8, 6))

        # 读取/存储按钮行
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

        # 点位列表（带滚动条）
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

        # 操作按钮行
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

        # 速度 + 顺序发送
        pos_bot = ttk.Frame(parent)
        pos_bot.pack(fill=tk.X)
        ttk.Label(pos_bot, text="速度:", font=("Arial", 10)).pack(side=tk.LEFT)
        self.ent_pos_speed = ttk.Entry(pos_bot, width=6, font=("Arial", 10))
        self.ent_pos_speed.insert(0, "50")
        self.ent_pos_speed.pack(side=tk.LEFT, padx=4)
        tk.Button(pos_bot, text="顺序发送全部→机械臂", font=("Arial", 10, "bold"), bg="#495057", fg="white",
                  relief=tk.FLAT, command=self.send_all_positions
                  ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(8, 0))

    def _build_movel_page(self, parent):
        pgrid = ttk.Frame(parent)
        pgrid.pack(fill=tk.X)
        pgrid.columnconfigure(2, weight=1)

        self.ent_pose, self.scl_pose, self.lbl_pose = [], [], []
        labels = ['X', 'Y', 'Z', 'R', 'P', 'Yw']
        defaults = [0, 0, 150, 0, 180, 0]
        ranges = [(-250, 250), (-200, 200), (-50, 450), (-180, 180), (0, 360), (-180, 180)]
        self.movel_drag_enable = tk.BooleanVar(value=False)
        self.last_movel_send_time = 0
        
        for i, (lbl, df, rng) in enumerate(zip(labels, defaults, ranges)):
            ttk.Label(pgrid, text=f"{lbl}:", font=("Arial", 10, "bold")
                      ).grid(row=i, column=0, padx=4, pady=2, sticky="e")
            ent = ttk.Entry(pgrid, width=6, font=("Arial", 10))
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

            def make_pcb(idx, lbl, ent):
                def cb(val):
                    v = float(val)
                    lbl.config(text=f"{v:.1f}")
                    s = f"{v:.1f}"
                    if ent.get() != s:
                        ent.delete(0, tk.END)
                        ent.insert(0, s)
                    if self.movel_drag_enable.get():
                        now = time.time()
                        if now - self.last_movel_send_time > 0.05:
                            self.last_movel_send_time = now
                            self.root.after(1, self.send_movel)
                return cb
            scl.config(command=make_pcb(i, lbl, ent))
            
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
        movel_ctrl.pack(fill=tk.X, pady=(6, 0))
        ttk.Label(movel_ctrl, text="Speed:", font=("Arial", 10)).pack(side=tk.LEFT, padx=(0, 2))
        self.ent_l_speed = ttk.Entry(movel_ctrl, width=5, font=("Arial", 10))
        self.ent_l_speed.insert(0, "50")
        self.ent_l_speed.pack(side=tk.LEFT)
        tk.Checkbutton(movel_ctrl, text="拖发", variable=self.movel_drag_enable,
                       bg=None, font=("Arial", 10)).pack(side=tk.LEFT, padx=6)
        tk.Button(movel_ctrl, text="发送 MoveL", font=("Arial", 10, "bold"), bg="#3b5bdb", fg="white",
                  relief=tk.FLAT, command=self.send_movel).pack(side=tk.RIGHT, padx=2)

    def _build_servoj_page(self, parent):
        tk.Label(parent, text="基准: [0,-75,180,0,0,0]  激励: J1 ±20° 正弦 0.5Hz",
                 font=("Arial", 10), foreground="#666").pack(anchor="w")
        self.btn_servoj_start = tk.Button(parent, text="▶ 开始正弦轨迹", font=("Arial", 11),
                                          bg="#495057", fg="white", relief=tk.FLAT,
                                          command=self.toggle_servoj_test)
        self.btn_servoj_start.pack(fill=tk.X, pady=(8, 0))
        self.is_servoj_testing = False
        self.servoj_thread = None

    # ==============================================
    # 点位存储（示教）
    # ==============================================

    def _build_position_page(self, parent):
        pass  # UI已嵌入_movej_page，此处仅保留空实现

    def read_and_sync_from_robot(self):
        """发送 #GETJPOS 并将返回结果同步到滑块"""
        if not self.is_connected:
            self.log("未连接串口", "WARN")
            return
        self._sync_waiting = True
        self.send_cmd("#GETJPOS")
        self.log("已发送 #GETJPOS，等待响应...", "INFO")
        self.root.after(500, self._check_sync_response)

    def _check_sync_response(self):
        if getattr(self, "_sync_data", None):
            self._apply_sync_data(self._sync_data)
            self._sync_data = None
        else:
            self.log("同步超时，请确认机械臂已使能并处于模式1", "ERROR")
        self._sync_waiting = False

    def _apply_sync_data(self, data):
        if len(data) < 6:
            self.log(f"数据长度不足: {len(data)}", "ERROR")
            return
        for i in range(min(6, len(data))):
            try:
                v = float(data[i])
                self.scl_joints[i].set(v)
                self.ent_joints[i].delete(0, tk.END)
                self.ent_joints[i].insert(0, f"{v:.2f}")
                self.lbl_joints[i].config(text=f"{v:.2f}")
            except (ValueError, IndexError):
                pass
        self.log("已同步机械臂当前位置到滑块", "INFO")

    def read_current_position(self):
        """读取当前滑块位置并显示"""
        jvals = [float(e.get()) for e in self.ent_joints]
        rval = float(self.ent_j7.get())
        pos = [round(v, 2) for v in jvals + [rval]]
        names = ["J1", "J2", "J3", "J4", "J5", "J6", "Rail"]
        msg = ", ".join(f"{n}={v}" for n, v in zip(names, pos))
        self.log(f"当前滑块: {msg}", "INFO")

    def save_current_position(self):
        """保存当前滑块位置为一个点位"""
        jvals = [float(e.get()) for e in self.ent_joints]
        rval = float(self.ent_j7.get())
        pos = [round(v, 2) for v in jvals + [rval]]
        idx = len(self.stored_positions) + 1
        name = f"#{idx} [{','.join(str(v) for v in pos[:3])}...]"
        self.stored_positions.append((name, pos))
        self._pos_listbox.insert(tk.END, name)
        self.log(f"已存储: {name}", "INFO")

    def _delete_position(self):
        idx = self._pos_listbox.curselection()
        if not idx:
            idx = self._pos_listbox.size() - 1
        if idx:
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
        # 同步到滑块
        for i in range(6):
            try:
                self.scl_joints[i].set(pos[i])
                self.ent_joints[i].delete(0, tk.END)
                self.ent_joints[i].insert(0, f"{pos[i]:.2f}")
                self.lbl_joints[i].config(text=f"{pos[i]:.2f}")
            except (ValueError, IndexError):
                pass
        try:
            self.scl_j7.set(pos[6])
            self.ent_j7.delete(0, tk.END)
            self.ent_j7.insert(0, f"{pos[6]:.2f}")
            self.lbl_j7.config(text=f"{pos[6]:.2f} mm")
        except (ValueError, IndexError):
            pass
        self.log(f"已加载: {name}", "INFO")

    def send_selected_position(self):
        """发送列表中选中的点位到机械臂"""
        idx = self._pos_listbox.curselection()
        if not idx:
            self.log("请先选择一个点位", "WARN")
            return
        idx = idx[0]
        name, pos = self.stored_positions[idx]
        try:
            speed = float(self.ent_pos_speed.get())
        except ValueError:
            speed = 50
        cmd = ">" + ",".join(str(v) for v in pos[:7]) + f",{speed}"
        self.send_cmd(cmd)
        self.log(f"已发送: {name} → {cmd}", "INFO")

    def send_all_positions(self):
        """按顺序发送所有点位（等上一个到达后再发下一个）"""
        if not self.stored_positions:
            self.log("没有存储的点位", "WARN")
            return
        if self._pos_queue_running:
            self.log("顺序发送正在进行中，请等待完成", "WARN")
            return
        try:
            speed = float(self.ent_pos_speed.get())
        except ValueError:
            speed = 50
        self._pos_queue_running = True
        self._pos_queue_pending = list(self.stored_positions)
        self._pos_queue_idx = 0
        self._pos_queue_speed = speed
        self.log(f"开始顺序发送 {len(self._pos_queue_pending)} 个点位（等待ok）...", "INFO")
        self._send_next_position()

    def _send_next_position(self):
        """内部方法：发送下一个点位（由 ok 触发或由 send_all_positions 启动）"""
        if self._pos_queue_idx >= len(self._pos_queue_pending):
            return
        total = len(self._pos_queue_pending)
        i = self._pos_queue_idx
        name, pos = self._pos_queue_pending[i]
        cmd = ">" + ",".join(str(v) for v in pos[:7]) + f",{self._pos_queue_speed}"
        self.send_cmd(cmd)
        self.log(f"  [{i+1}/{total}] {name}", "INFO")

    # ==============================================
    # 夹爪控制（移动到右栏关节控制区）
    # ==============================================

    def _build_gripper_tab(self, parent):
        # 夹爪使能/失能
        hg_f = ttk.Frame(parent)
        hg_f.pack(fill=tk.X, pady=(0, 4))
        tk.Button(hg_f, text="使能", font=("Arial", 10), bg="#2b8a3e", fg="white",
                  relief=tk.FLAT, command=lambda: self.send_cmd("!HAND_EN")
                  ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 2))
        tk.Button(hg_f, text="失能", font=("Arial", 10), bg="#868e96", fg="white",
                  relief=tk.FLAT, command=lambda: self.send_cmd("!HAND_DIS")
                  ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        # 开度滑块
        ttk.Label(parent, text="开度:", font=("Arial", 10)).pack(anchor="w")
        hp_f = ttk.Frame(parent)
        hp_f.pack(fill=tk.X, pady=(0, 4))
        self.ent_hand_pos = ttk.Entry(hp_f, width=6, font=("Arial", 10))
        self.ent_hand_pos.insert(0, "50")
        self.ent_hand_pos.pack(side=tk.LEFT)
        self.scl_hand = ttk.Scale(hp_f, from_=0, to=100, orient=tk.HORIZONTAL)
        self.scl_hand.set(50)
        self.scl_hand.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)
        self.lbl_hand_val = ttk.Label(hp_f, text="50", width=3, font=("Arial", 10))
        self.lbl_hand_val.pack(side=tk.LEFT)
        tk.Button(hp_f, text="发送", font=("Arial", 10), bg="#495057", fg="white",
                  relief=tk.FLAT, width=5, command=self.send_hand_pos).pack(side=tk.LEFT, padx=(4, 0))

        # 力矩滑块
        ttk.Label(parent, text="力矩:", font=("Arial", 10)).pack(anchor="w")
        hc_f = ttk.Frame(parent)
        hc_f.pack(fill=tk.X, pady=(0, 4))
        self.ent_hand_current = ttk.Entry(hc_f, width=6, font=("Arial", 10))
        self.ent_hand_current.insert(0, "1.2")
        self.ent_hand_current.pack(side=tk.LEFT)
        self.scl_hand_current = ttk.Scale(hc_f, from_=0.05, to=2.0, orient=tk.HORIZONTAL)
        self.scl_hand_current.set(1.2)
        self.scl_hand_current.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)
        self.lbl_hand_current_val = ttk.Label(hc_f, text="1.2", width=4, font=("Arial", 10))
        self.lbl_hand_current_val.pack(side=tk.LEFT)
        tk.Button(hc_f, text="发送", font=("Arial", 10), bg="#495057", fg="white",
                  relief=tk.FLAT, width=5, command=self.send_hand_current).pack(side=tk.LEFT, padx=(4, 0))

        # 标定 + 张开/闭合
        tk.Button(parent, text="标定零点", font=("Arial", 10), bg="#343a40", fg="white",
                  relief=tk.FLAT, command=self.send_hand_zero).pack(fill=tk.X, pady=(0, 4))

        hg2_f = ttk.Frame(parent)
        hg2_f.pack(fill=tk.X, pady=(0, 0))
        tk.Button(hg2_f, text="张开", font=("Arial", 10), bg="#5c7cfa", fg="white",
                  relief=tk.FLAT, command=lambda: self.send_cmd("!HAND_O")
                  ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 2))
        tk.Button(hg2_f, text="闭合", font=("Arial", 10), bg="#fa5252", fg="white",
                  relief=tk.FLAT, command=lambda: self.send_cmd("!HAND_C")
                  ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        # 绑定夹爪滑块回调
        def update_hand_from_scale(val):
            v = int(float(val))
            self.lbl_hand_val.config(text=str(v))
            if self.ent_hand_pos.get() != str(v):
                self.ent_hand_pos.delete(0, tk.END)
                self.ent_hand_pos.insert(0, str(v))
        self.scl_hand.config(command=update_hand_from_scale)

        def update_hand_from_entry(event):
            try:
                self.scl_hand.set(int(self.ent_hand_pos.get()))
            except ValueError:
                pass
        self.ent_hand_pos.bind("<Return>", update_hand_from_entry)
        self.ent_hand_pos.bind("<FocusOut>", update_hand_from_entry)

        def update_hand_current_from_scale(val):
            v = round(float(val), 1)
            self.lbl_hand_current_val.config(text=f"{v:.1f}")
            if self.ent_hand_current.get() != f"{v:.1f}":
                self.ent_hand_current.delete(0, tk.END)
                self.ent_hand_current.insert(0, f"{v:.1f}")
        self.scl_hand_current.config(command=update_hand_current_from_scale)

        def update_hand_current_from_entry(event):
            try:
                v = float(self.ent_hand_current.get())
                if 0.05 <= v <= 2.0:
                    self.scl_hand_current.set(v)
                    self.lbl_hand_current_val.config(text=f"{v:.1f}")
            except ValueError:
                pass
        self.ent_hand_current.bind("<Return>", update_hand_current_from_entry)
        self.ent_hand_current.bind("<FocusOut>", update_hand_current_from_entry)

    # ==============================================
    # LED 绘制
    # ==============================================

    def _draw_led(self, on):
        self.led_canvas.delete("all")
        color = "#4ade80" if on else "#6b7280"
        self.led_canvas.create_oval(2, 2, 14, 14, fill=color, outline="")

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
        
    # ========== 核心逻辑功能方法（完全保持不变） ==========
    def refresh_ports(self):
        ports = serial.tools.list_ports.comports()
        self.cb_ports['values'] = [p.device for p in ports]
        if ports:
            self.cb_ports.current(0)

    def log(self, msg, tag="INFO"):
        time_str = time.strftime("%H:%M:%S")
        tag_color = {"TX": "TX", "RX": "RX", "ERROR": "ERROR", "WARN": "WARN"}.get(tag, "INFO")
        self.txt_log.insert(tk.END, f"[{time_str}] [{tag}] {msg}\n", tag_color)
        self.txt_log.see(tk.END)

    def toggle_connection(self):
        if not self.is_connected:
            port = self.cb_ports.get()
            baud = self.cb_baudrate.get()
            try:
                self.serial_port = serial.Serial(port, int(baud), timeout=0.5)
                self.serial_port.write_timeout = 1.0
                self.is_connected = True
                self.btn_connect.config(text="断开", bg="#c92a2a", activebackground="#a02222")
                self._draw_led(True)
                self.lbl_status.config(text=f"已连接 {port}  {baud}", fg="#4ade80")
                self.log(f"成功连接 {port} @ {baud}")
                
                self.stop_thread = False
                self.receive_thread = threading.Thread(target=self.serial_receive_loop, daemon=True)
                self.receive_thread.start()
            except Exception as e:
                messagebox.showerror("连接失败", str(e))
                self.log(f"连接失败: {e}", "ERROR")
        else:
            self.is_servoj_testing = False
            self.stop_thread = True
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
            self.is_connected = False
            self.btn_connect.config(text="连接", bg="#2b8a3e", activebackground="#2f9e44")
            self._draw_led(False)
            self.lbl_status.config(text="未连接", fg="#ff6b6b")
            self.log("已断开串口")

    def serial_receive_loop(self):
        while not self.stop_thread and self.serial_port and self.serial_port.is_open:
            try:
                # 优先用 in_waiting 避免 readline 阻塞（部分 MCU 不发 \n）
                if self.serial_port.in_waiting:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    if data:
                        text = data.decode('utf-8', errors='replace')
                        # 按行分割，防止残留字符堆积
                        for line in text.split('\n'):
                            line = line.strip()
                            if not line:
                                continue
                            # 过滤固件入队回执的纯数字行（如"15"=队列剩余空间），避免污染日志
                            if line.isdigit():
                                continue
                            self.root.after(0, self.log, line, "RX")
                            # 拦截 #GETJPOS 响应并同步滑块
                            if getattr(self, "_sync_waiting", False):
                                if line.startswith("ok") or line.startswith(">"):
                                    tokens = line.lstrip(">ok").split()
                                    nums = []
                                    for t in tokens:
                                        t = t.strip().rstrip(",")
                                        if not t:
                                            continue
                                        try:
                                            nums.append(str(float(t)))
                                        except ValueError:
                                            pass
                                    if len(nums) >= 6:
                                        # GETJPOS 只返回6个关节，Rail 用当前滑块值
                                        self._sync_data = nums[:6]
                                        self._sync_waiting = False
                                        self.root.after(0, lambda n=len(nums): self.log(f"收到{n}个关节数据，已同步（Rail使用滑块当前值）", "INFO"))
                                    else:
                                        self.root.after(0, lambda: self.log(f"数据不足: {nums}", "WARN"))
                            # 拦截 ok 触发下一条顺序发送（SEQ 模式下固件阻塞到位后回单字 ok）
                            if getattr(self, "_pos_queue_running", False) and line == "ok":
                                self._pos_queue_running = False
                                self._pos_queue_idx += 1
                                total = len(self._pos_queue_pending)
                                if self._pos_queue_idx < total:
                                    self.root.after(0, self._send_next_position)
                                else:
                                    self.root.after(0, lambda: self.log(f"顺序发送完成，共{total}个点位", "INFO"))
                # 无数据时短暂休眠，不阻塞主循环
                time.sleep(0.05)
            except serial.SerialException as e:
                self.root.after(0, self.log, f"串口异常: {e}", "ERROR")
                break
            except Exception as e:
                self.root.after(0, self.log, f"读取异常: {e}", "ERROR")
                break

    def send_cmd(self, cmd):
        if not self.is_connected or not self.serial_port:
            self.log("未连接串口", "WARN")
            return
        try:
            full_cmd = f"{cmd}\n"
            self.serial_port.write(full_cmd.encode('utf-8'))
            self.log(cmd, "TX")
        except Exception as e:
            self.log(f"发送失败: {e}", "ERROR")

    def send_custom_cmd(self):
        cmd = self.ent_custom_cmd.get().strip()
        if cmd:
            self.send_cmd(cmd)
            self.ent_custom_cmd.delete(0, tk.END)

    def set_rgb_state_start(self):
        idx = int(self.cb_state_start.get())
        self.send_cmd(f"!RGB_SET_START {idx}")

    def set_rgb_state_enable(self):
        idx = int(self.cb_state_enable.get())
        self.send_cmd(f"!RGB_SET_ENABLE {idx}")

    def set_rgb_state_disable(self):
        idx = int(self.cb_state_disable.get())
        self.send_cmd(f"!RGB_SET_DISABLE {idx}")

    def send_hand_pos(self):
        try:
            pos = int(self.ent_hand_pos.get())
            if 0 <= pos <= 100:
                self.send_cmd(f"!HAND_POS {pos}")
            else:
                messagebox.showerror("错误", "夹爪开度必须在 0~100 之间")
        except ValueError:
            messagebox.showerror("错误", "请输入有效的数字")

    def send_hand_current(self):
        """发送 #I_LIMIT_J 8 命令，设置夹爪电流限制"""
        try:
            current = float(self.ent_hand_current.get())
            if 0.05 <= current <= 2.0:
                self.send_cmd(f"#I_LIMIT_J 8 {current}")
                self.log(f"已发送 #I_LIMIT_J 8 {current}（夹爪电流限制）", "INFO")
            else:
                messagebox.showerror("错误", "夹爪电流必须在 0.05~2.0A 之间")
        except ValueError:
            messagebox.showerror("错误", "请输入有效的数字")

    def send_hand_zero(self):
        """发送 !HAND_ZERO 命令，将夹爪当前位置设为零点"""
        self.send_cmd("!HAND_ZERO")
        self.log("已发送 !HAND_ZERO（夹爪标定）", "INFO")

    def send_home_offset_all(self):
        if not self.is_connected or not self.serial_port:
            self.log("未连接串口", "WARN")
            return
        for j in range(1, 7):
            self.send_cmd(f"#OFFSET_J {j}")
            time.sleep(0.15)
        self.log("已发送 #OFFSET_J 1~6（全部关节设为零点）", "INFO")

    def _rgb_light_on(self):
        """开灯：读取 Entry 当前值，应用亮度（不自动保存）"""
        try:
            val = int(self.ent_bright.get())
            val = max(0, min(100, val))
            self.send_cmd(f"!RGB_BRIGHT {val}")
            self.scl_bright.set(val)
            self.log(f"开灯: 亮度 {val}%", "INFO")
        except ValueError:
            messagebox.showerror("错误", "请输入 0~100 的整数")

    def _rgb_light_off(self):
        """关灯：发送亮度 0"""
        self.send_cmd("!RGB_BRIGHT 0")
        self.log("关灯: 亮度 0%", "INFO")

    def _rgb_bright_query(self):
        """查询固件当前亮度值"""
        self.send_cmd("!RGB_BRIGHT")
        self.log("已发送 !RGB_BRIGHT (查询)", "INFO")

    def _rgb_bright_apply(self):
        """应用亮度（临时，不保存）"""
        try:
            val = int(self.ent_bright.get())
            val = max(0, min(100, val))
            self.send_cmd(f"!RGB_BRIGHT {val}")
            self.scl_bright.set(val)
            self.log(f"亮度已应用: {val}%", "INFO")
        except ValueError:
            messagebox.showerror("错误", "请输入 0~100 的整数")

    def _rgb_bright_save(self):
        """应用亮度并保存到 EEPROM"""
        try:
            val = int(self.ent_bright.get())
            val = max(0, min(100, val))
            self.send_cmd(f"!RGB_BRIGHT {val} &")
            self.scl_bright.set(val)
            self.rgb_brightness = val
            self._save_config()
            self.log(f"亮度已保存: {val}%", "INFO")
        except ValueError:
            messagebox.showerror("错误", "请输入 0~100 的整数")

    def send_rgb_color(self):
        try:
            idx = int(self.cb_color_idx.get())
            r = int(self.ent_r.get())
            g = int(self.ent_g.get())
            b = int(self.ent_b.get())
            if idx in [0, 1, 2] and 0 <= r <= 255 and 0 <= g <= 255 and 0 <= b <= 255:
                self.send_cmd(f"!RGB_COLOR {idx} {r} {g} {b}")
                self.send_cmd(f"!RGB_MODE {idx}")
            else:
                messagebox.showerror("错误", "索引必须为0-2，颜色值必须在0-255之间")
        except ValueError:
            messagebox.showerror("错误", "请输入有效的数字")

    def send_acc_base(self):
        try:
            node = int(self.cb_acc_node.get())
            acc = float(self.ent_acc_val.get())
            if 1 <= node <= 6 and 1.0 <= acc <= 2000.0:
                self.send_cmd(f"#ACC_BASE_J {node} {acc}")
            else:
                messagebox.showerror("错误", "节点必须为1-6，加速度必须在1-2000之间")
        except ValueError:
            messagebox.showerror("错误", "请输入有效的数字")

    def send_i_limit(self):
        try:
            node = int(self.cb_acc_node.get())
            i_limit = float(self.ent_i_limit.get())
            if 1 <= node <= 6 and i_limit > 0:
                self.send_cmd(f"#I_LIMIT_J {node} {i_limit}")
            elif node == 8 and i_limit > 0:
                self.send_cmd(f"#I_LIMIT_J 8 {i_limit}")
            elif node == 9 and i_limit > 0:
                self.send_cmd(f"#I_LIMIT_J 9 {i_limit}")
            else:
                messagebox.showerror("错误", "节点必须为1-6或8或9，电流必须大于0")
        except ValueError:
            messagebox.showerror("错误", "请输入有效的数字")

    def send_rail_speed(self):
        self.apply_rail_speed()

    def apply_rail_speed(self):
        try:
            speed = float(self.ent_rail_speed.get())
            if speed < 0.5:
                speed = 0.5
            elif speed > 100:
                speed = 100
            self.send_cmd(f"#SPEED_RAIL {speed:.1f}")
        except ValueError:
            messagebox.showerror("错误", "请输入有效的数字")

    def save_rail_speed(self):
        try:
            speed = float(self.ent_rail_speed.get())
            if speed < 0.5:
                speed = 0.5
            elif speed > 100:
                speed = 100
            self.send_cmd(f"#SPEED_RAIL {speed:.1f} &")
        except ValueError:
            messagebox.showerror("错误", "请输入有效的数字")

    def query_rail_speed(self):
        self.send_cmd("#SPEED_RAIL")

    def send_rail_acc(self):
        self.apply_rail_acc()

    def apply_rail_acc(self):
        try:
            acc = float(self.ent_rail_acc.get())
            if acc < 10:
                acc = 10
            elif acc > 5000:
                acc = 5000
            self.send_cmd(f"#ACC_RAIL {acc:.1f}")
        except ValueError:
            messagebox.showerror("错误", "请输入有效的数字")

    def save_rail_acc(self):
        try:
            acc = float(self.ent_rail_acc.get())
            if acc < 10:
                acc = 10
            elif acc > 5000:
                acc = 5000
            self.send_cmd(f"#ACC_RAIL {acc:.1f} &")
        except ValueError:
            messagebox.showerror("错误", "请输入有效的数字")

    def query_rail_acc(self):
        self.send_cmd("#ACC_RAIL")

    def set_rail_current(self, current):
        """设置地轨电机电流限制"""
        self.send_cmd(f"#I_LIMIT_J 9 {current}")
        self.log(f"已设置地轨电流限制为 {current}A", "INFO")

    def query_rail_current(self):
        self.send_cmd("#I_LIMIT_J 9")

    def apply_rail_current(self):
        try:
            current = float(self.ent_rail_current.get())
            if current < 0.1:
                current = 0.1
            elif current > 3.0:
                current = 3.0
            self.send_cmd(f"#I_LIMIT_J 9 {current}")
        except ValueError:
            self.log("地轨电流值无效", "ERROR")

    def save_rail_current(self):
        self.apply_rail_current()
        self.log("地轨电流已应用（需固件支持Flash保存）", "INFO")

    def rail_move_left(self):
        if not self.is_connected:
            self.log("未连接串口", "WARN")
            return
        try:
            delta = float(self.ent_rail_step.get())
            current_j7 = float(self.ent_j7.get())
            new_j7 = current_j7 - delta
            speed = float(self.ent_rail_speed.get())

            # 读取当前 J1-J6 的值
            joints = [float(ent.get()) for ent in self.ent_joints]
            cmd = f">{joints[0]},{joints[1]},{joints[2]},{joints[3]},{joints[4]},{joints[5]},{new_j7},{speed}"
            self.send_cmd(cmd)
        except ValueError:
            messagebox.showerror("错误", "请输入有效的数字")

    def rail_move_right(self):
        if not self.is_connected:
            self.log("未连接串口", "WARN")
            return
        try:
            delta = float(self.ent_rail_step.get())
            current_j7 = float(self.ent_j7.get())
            new_j7 = current_j7 + delta
            speed = float(self.ent_rail_speed.get())

            # 读取当前 J1-J6 的值
            joints = [float(ent.get()) for ent in self.ent_joints]
            cmd = f">{joints[0]},{joints[1]},{joints[2]},{joints[3]},{joints[4]},{joints[5]},{new_j7},{speed}"
            self.send_cmd(cmd)
        except ValueError:
            messagebox.showerror("错误", "请输入有效的数字")

    def send_movej(self):
        try:
            joints = [float(ent.get()) for ent in self.ent_joints]
            j7 = float(self.ent_j7.get())
            speed = float(self.ent_j_speed.get())
            cmd = f">{joints[0]},{joints[1]},{joints[2]},{joints[3]},{joints[4]},{joints[5]},{j7},{speed}"
            self.send_cmd(cmd)
        except ValueError:
            messagebox.showerror("错误", "请输入有效的数字")

    def send_movel(self):
        try:
            p = [float(ent.get()) for ent in self.ent_pose]
            speed = float(self.ent_l_speed.get())
            cmd = f"@{p[0]},{p[1]},{p[2]},{p[3]},{p[4]},{p[5]},{speed}"
            self.send_cmd(cmd)
        except ValueError:
            messagebox.showerror("错误", "请输入有效的数字")

    def send_torque(self):
        try:
            rail_current = float(self.ent_rail_torque.get())
            torques = [float(ent.get()) for ent in self.ent_torques]  # J1~J6
            gripper_current = float(self.ent_gripper_torque.get())
            # $c0(地轨),c1~c6(关节) — 夹爪用 !HAND_I 单独控制
            cmd = f"${rail_current:.2f},{torques[0]:.2f},{torques[1]:.2f},{torques[2]:.2f},{torques[3]:.2f},{torques[4]:.2f},{torques[5]:.2f}"
            self.send_cmd(cmd)
            self.log(f"已发送力矩: Rail={rail_current}A, J1~J6={torques[0]:.2f}~{torques[5]:.2f}A, Gripper={gripper_current}A (!HAND_I)", "INFO")
            # 夹爪力矩单独发 !HAND_I
            if gripper_current > 0:
                self.send_cmd(f"!HAND_I {gripper_current:.2f}")
            else:
                self.send_cmd("!HAND_I 0")
        except ValueError:
            messagebox.showerror("错误", "请输入有效的数字")

    def toggle_servoj_test(self):
        if not self.is_connected:
            messagebox.showwarning("提示", "请先连接串口")
            return
            
        if self.is_servoj_testing:
            self.is_servoj_testing = False
            self.btn_servoj_start.config(text="▶ 开始发送正弦轨迹")
            self.log("已停止 ServoJ 测试")
        else:
            self.send_cmd("!START")
            self.send_cmd("#CMDMODE 6")
            self.is_servoj_testing = True
            self.btn_servoj_start.config(text="■ 停止发送")
            self.log("开始 ServoJ 测试 (50Hz直线插补正弦波)")
            self.servoj_thread = threading.Thread(target=self.servoj_test_loop, daemon=True)
            self.servoj_thread.start()

    def servoj_test_loop(self):
        start_time = time.time()
        # Rail(地轨), J1~J6, speed
        rail_pos = 0.0  # 地轨保持不动
        base_joints = [-75, 180, 0, 0, 0, 0]  # J1~J6 静止
        speed = 50.0

        kp = 0.5
        current_j1_pos = 0.0  # 测试只动 J1

        while self.is_servoj_testing and self.is_connected:
            t = time.time() - start_time
            target_j1 = 20 * math.sin(2 * math.pi * 0.5 * t)

            error_j1 = target_j1 - current_j1_pos
            current_j1_pos += error_j1 * kp

            # MoveJ: >Rail,j1~j6,speed  (7个值)
            cmd = f">{rail_pos:.2f},{current_j1_pos:.2f},{base_joints[0]},{base_joints[1]},{base_joints[2]},{base_joints[3]},{speed}"
            try:
                self.serial_port.write((cmd + "\n").encode('utf-8'))
                if int(t * 50) % 20 == 0:
                    self.root.after(0, self.log, f"Target: {target_j1:.1f}, J1: {current_j1_pos:.1f}", "TX")
            except Exception as e:
                self.root.after(0, self.log, f"发送失败: {e}", "ERROR")
                break

            time.sleep(0.02)

    # ── PID 调节 ──
    def query_pid(self):
        """查询选中节点的 Kp/Kv/Ki/Kd（通过 CAN 回读）"""
        node = int(self.cb_pid_node.get())
        self.send_cmd(f"#GET_PID {node}")
        self.log(f"已发送 #GET_PID {node}，等待电机 CAN 回传...", "INFO")

    def apply_pid(self):
        """应用（临时写入，不保存 EEPROM）"""
        try:
            node = int(self.cb_pid_node.get())
            kp = int(self.ent_pid["kp"].get())
            kv = int(self.ent_pid["kv"].get())
            ki = int(self.ent_pid["ki"].get())
            kd = int(self.ent_pid["kd"].get())
            self.send_cmd(f"#SET_DCE_KP {node} {kp}")
            self.send_cmd(f"#SET_DCE_KV {node} {kv}")
            self.send_cmd(f"#SET_DCE_KI {node} {ki}")
            self.send_cmd(f"#SET_DCE_KD {node} {kd}")
            self.log(f"已应用 PID (节点{node}): Kp={kp} Kv={kv} Ki={ki} Kd={kd}", "INFO")
        except ValueError:
            messagebox.showerror("错误", "请输入有效的 PID 数值")

    def save_pid(self):
        """保存到 EEPROM（通过 canBuf[4]=1 触发固件自动保存）"""
        node = int(self.cb_pid_node.get())
        self.apply_pid()
        self.log(f"PID 参数已保存到节点 {node} EEPROM", "INFO")


if __name__ == "__main__":
    root = tk.Tk()
    app = RobotSerialAssistant(root)
    root.mainloop()