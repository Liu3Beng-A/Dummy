import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import serial
import serial.tools.list_ports
import threading
import queue
import time
import re
import math


BAUDRATE = 115200  # 固件 usart.c 中 huart4 波特率硬编码

# 各关节软限位（v3.1 同步到 firmware_new 构造参数 ±175/±75/(0,180)/±270/±100/±180）
JOINT_RANGES = [(-175, 175), (-75, 90), (0, 180), (-270, 270), (-100, 100), (-180, 180)]
# 默认关节角 = 固件 REST_POSE = {0, -75, 180, 0, 0, 0}
JOINT_DEFAULTS = [0, -75, 180, 0, 0, 0]
# v3.1 默认速度 35（slider 35 ≈ 10.5 r/s，与新版串口助手对齐）
DEFAULT_SPEED = 35

# [v3.1 新增] 地轨（J7）参数：直连丝杆1605，5mm/圈，行程 ±250 mm
RAIL_RANGE = (-250, 250)
RAIL_DEFAULT = 0

# 普通堵转参数：电流为控制器命令值，并非实测电流。
# 界面初值仅用于编辑；最终限额及整组合法性由实际节点检查。
STALL_PARAMETERS = (
    ("ENABLE", "普通堵转检测开关", "0=关闭 / 1=开启", 1, 0, 1),
    ("CURRENT_A", "电流命令阈值", "A", 1.0, 0.1, None),
    ("SPEED_RPS", "实际低速阈值", "电机 r/s", 0.02, 0.001, 0.2),
    ("ERROR_R", "跟随误差阈值", "电机圈", 0.5 / 360 * 50, 0.0002, 1),
    ("TRIGGER_MS", "连续触发时间", "ms", 100, 20, 2000),
    ("STARTUP_MS", "启动豁免时间", "ms", 200, 0, 500),
    ("CURRENT_FILTER_MS", "电流命令滤波", "ms", 10, 1, 100),
    ("SPEED_FILTER_MS", "速度滤波", "ms", 20, 1, 100),
    ("BACK_DEG", "关节回退距离", "关节 ° (J1–J6)", 5, 0, None),
    ("BACK_MM", "地轨回退距离", "mm (J7)", 5, 0, None),
    ("BACK_RPS", "回退速度上限", "电机 r/s", 3, 0, None),
    ("BACK_ACCEL", "回退加速度", "电机 r/s²", 5, 0.1, 100),
    ("BACK_CURRENT_A", "回退电流命令上限", "A", 2.5, 0, None),
    ("HOLD_CURRENT_A", "保持电流命令上限", "A", 2.5, 0, None),
    ("BACK_POS_TOL_R", "实际到位容差", "电机圈", 0.001, 0.0002, 0.01),
    ("BACK_TIMEOUT_MS", "回退超时", "ms", 5000, 250, 30000),
    ("REBLOCK_MS", "回退再次受阻时间", "ms", 100, 20, 500),
    ("GRANT_WAIT_MS", "等待保护授权", "ms", 500, 100, 2000),
)

# 校准参数使用独立 #CAL_* 命令，J8 的接触判定不加入普通堵转保护。
# 未回读的编辑初值对应2026-10-07实测默认；下表为J7，J8在节点切换时覆盖。
CAL_PARAMETERS = (
    ("SEEK_RPS", "寻限位速度上限", "电机 r/s", 5, 0, None),
    ("RETURN_RPS", "返回速度上限", "电机 r/s", 5, 0, None),
    ("RETOUCH_RPS", "复触速度上限", "电机 r/s", 0.5, 0, None),
    ("ACCEL", "校准加速度", "电机 r/s²", 3, 0.1, 100),
    ("CURRENT_A", "校准电流命令上限", "A", 2.5, 0, None),
    ("CONTACT_RATIO", "接触电流命令比率", "本次限流比例", 0.3, 0, 1),
    ("SPEED_RPS", "接触低速阈值", "电机 r/s", 0.02, 0.001, 0.2),
    ("ERROR_R", "参考/实际误差阈值", "电机圈", 0.02, 0.0002, 1),
    ("TRIGGER_MS", "接触持续时间", "ms", 100, 20, 2000),
    ("STARTUP_MS", "启动豁免时间", "ms", 100, 0, 500),
    ("MAX_TRAVEL_R", "最大搜索行程", "电机圈", 101, 0, None),
    ("SEEK_TIMEOUT_MS", "首次搜索超时", "ms", 220000, 1, None),
    ("RETURN_TIMEOUT_MS", "返回超时", "ms", 120000, 1, None),
    ("RETOUCH_TIMEOUT_MS", "复触超时", "ms", 5000, 1, None),
    ("BACKOFF_R", "复触前退让距离", "电机圈", 1, 0, None),
    ("REPEAT_TOL_R", "两次接触容差", "电机圈", 0.05, 0, None),
    ("POSITION_TOL_R", "实际返回到位容差", "电机圈", 0.005, 0.0002, 0.01),
    ("CONTACT_WINDOW_R", "已知端点接触窗口", "电机圈", 1, 0, None),
)

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
        self._seq_running = False   # 当前点位等待状态
        self._seq_active = False
        self._seq_motion_id = None
        self._seq_id_floor = 0
        self._seq_session = 0
        self._mode_request_counter = 0
        self._mode_request = None
        self._mode_pending = None
        self._mode_set_token = None
        self._seq_wait_mode = False
        self._seq_pending = []      # 待发送点位
        self._seq_idx = 0           # 当前发送索引
        self._seq_speed = DEFAULT_SPEED

        # [调整] 夹爪开度拖发状态字段（仿 movej/movel：开关 + 节流时间戳）
        # 必须在 _build_hand_page 之前初始化（_warn_drag_mode 要能读到 hand_pos_drag_enable）
        self.hand_pos_drag_enable = tk.BooleanVar(value=False)
        self.last_hand_pos_send_time = 0

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
        self._motor_enabled = False  # 仅为 UI 缓存，未知状态不能证明八轴失能
        self._motor_state_known = False
        self._stop_locked = False
        self._calibration_busy = False
        self._protection_window = None
        self._protection_entries = {"STALL": {}, "CAL": {}}
        self._protection_values = {"STALL": {}, "CAL": {}}
        self._tx_generation = 0
        self._stop_rx_fragment = ""
        self._acc_sync_waiting = False
        self._acc_sync_timeout_id = None

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
        self.lbl_motor_state = tk.Label(top_bar, text="● 电机状态未知", font=("Arial", 12, "bold"),
                                         bg="#495057", fg="#ff6b6b", padx=20, pady=8)
        self.lbl_motor_state.pack(side=tk.RIGHT, padx=(0, 8), pady=8)

        # 右侧：急停按钮（最醒目）
        self.btn_emergency = tk.Button(top_bar, text="[ !STOP ]", font=("Arial", 12, "bold"),
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

        # 三栏布局：左(窄)= 系统/查询/电机  中= 电机参数  右(宽)= 关节控制+MoveL+夹爪
        control_area.columnconfigure(0, weight=1)  # 左栏
        control_area.columnconfigure(1, weight=2)  # 中栏（电机参数）
        control_area.columnconfigure(2, weight=3)  # 右栏（关节控制 / MoveL / 夹爪）
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

        # 校准入口由主控检查八轴失能，完成后自动失能并保存，不重启。
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
        self.btn_motion_status = tk.Button(query_f, text="查询运动状态", font=("Arial", 10),
                                          bg="#495057", fg="white", relief=tk.FLAT, pady=5,
                                          command=lambda: self.send_cmd("#MOTION_STATUS"))
        self.btn_motion_status.pack(fill=tk.X, padx=1, pady=(2, 0))

        # --- 电机（旧固件只有 J1~J6，节点 1~6）---
        motor_f = ttk.LabelFrame(left_col, text="电机", padding=6)
        motor_f.pack(fill=tk.X)
        node_f = ttk.Frame(motor_f)
        node_f.pack(fill=tk.X)
        ttk.Label(node_f, text="节点:", font=("Arial", 10, "bold")).pack(side=tk.LEFT)
        self.cb_node = ttk.Combobox(node_f, width=5,
                                    values=[str(i) for i in range(0, 9)],   # 0=ALL(全部+夹爪), 1~6=J1~J6, 7=地轨, 8=夹爪 [Step 5]
                                    state="readonly", font=("Arial", 10))
        self.cb_node.current(0)
        self.cb_node.pack(side=tk.LEFT, padx=4)
        tk.Button(node_f, text="电机重启", font=("Arial", 10), bg="#c92a2a", fg="white",
                  relief=tk.FLAT, command=self._cmd_reboot
                  ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        # ==============================================
        # 中栏：电机参数（查询 / 设置）
        # ==============================================
        middle_col = ttk.Frame(control_area)
        middle_col.grid(row=0, column=1, sticky="nsew", padx=(0, 6))
        middle_col.columnconfigure(0, weight=1)
        middle_col.rowconfigure(0, weight=1)

        # 查询与设置合并为一个面板，共享同一个节点下拉。
        # 写入语义（firmware_new 决策 #1/#2，2026-10-04 拍板）：
        #   - #SET_PID        ：永远写 EEPROM（无 RAM/EEPROM 区分，固件基类固定 canBuf[4]=1）
        #   - #ACC_J          ：默认不写 EEPROM；末尾追加 `&` 走 SetAcceleration_persist 写 EEPROM
        #   - #I_LIMIT_J      ：默认不写 EEPROM；末尾追加 `&` 走 SetCurrentLimit_persist 写 EEPROM
        # UI 区分两个按钮：「应用」= 不带 &（运行时 RAM，掉电丢），「保存」= 带 &（写 EEPROM 掉电保持）。
        # PID 永远写 EEPROM，因此只配「保存」一个按钮（与「应用」同义）。
        # 节点 0=ALL 仅用于查询广播；节点=0 时写入按钮禁用，避免误写所有节点。
        # 回包统一追加到底部终端日志，不再单独显示。
        param_f = ttk.LabelFrame(middle_col, text="电机参数（查询 / 设置）", padding=6)
        param_f.pack(fill=tk.BOTH, expand=True)

        # 节点下拉框（0=ALL, 1~6=J1~J6, 7=地轨, 8=夹爪）
        top_p = ttk.Frame(param_f)
        top_p.pack(fill=tk.X)
        ttk.Label(top_p, text="节点:", font=("Arial", 10, "bold")).pack(side=tk.LEFT)
        self.param_node = tk.IntVar(value=1)
        cb_p = ttk.Combobox(top_p, textvariable=self.param_node, width=4, state="readonly",
                            values=[0, 1, 2, 3, 4, 5, 6, 7, 8])
        cb_p.pack(side=tk.LEFT, padx=4)
        ttk.Label(top_p, text="(0=ALL 仅查询, 7=地轨, 8=夹爪)",
                  font=("Arial", 8), foreground="#868e96").pack(side=tk.LEFT, padx=4)

        # PID：4 个 Entry（横向紧凑布局）
        pid_f = ttk.Frame(param_f)
        pid_f.pack(fill=tk.X, pady=(6, 2))
        ttk.Label(pid_f, text="PID:", font=("Arial", 10, "bold")).pack(side=tk.LEFT)
        for key, label in [("kp", "Kp"), ("kv", "Kv"), ("ki", "Ki"), ("kd", "Kd")]:
            ttk.Label(pid_f, text=f"{label}:", font=("Arial", 9)).pack(side=tk.LEFT, padx=(6, 2))
            ent = ttk.Entry(pid_f, width=6, font=("Arial", 10))
            ent.insert(0, "0")
            ent.pack(side=tk.LEFT)
            setattr(self, f"ent_set_pid_{key}", ent)

        # PID 读取 / 保存 按钮
        # 固件侧 #SET_PID 永远写 EEPROM（无 RAM/EEPROM 区分），因此只配「保存」一个按钮
        # ——文本不再写「不写 EEPROM」（与实际行为矛盾的旧文字已纠正）。
        pid_btn = ttk.Frame(param_f)
        pid_btn.pack(fill=tk.X, pady=2)
        ttk.Button(pid_btn, text="读取 PID", command=self._qry_pid).pack(side=tk.LEFT, padx=2)
        self.btn_pid_save = ttk.Button(pid_btn, text="保存", command=self._set_pid)
        self.btn_pid_save.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        # 加速度（r/s²）Entry + 读取 / 应用 / 保存 按钮
        # 应用 = #ACC_J n v（不写 EEPROM，掉电丢）；保存 = #ACC_J n v &（走 SetAcceleration_persist 写 EEPROM）
        acc_f = ttk.Frame(param_f)
        acc_f.pack(fill=tk.X, pady=2)
        ttk.Label(acc_f, text="ACC (r/s²):", font=("Arial", 10)).pack(side=tk.LEFT)
        self.ent_set_acc = ttk.Entry(acc_f, width=8, font=("Arial", 10))
        self.ent_set_acc.insert(0, "150")
        self.ent_set_acc.pack(side=tk.LEFT, padx=4)
        ttk.Button(acc_f, text="读取 ACC", command=self._qry_acc).pack(side=tk.LEFT, padx=2)
        self.btn_acc_apply = ttk.Button(acc_f, text="应用", command=self._set_acc)
        self.btn_acc_apply.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        self.btn_acc_save = ttk.Button(acc_f, text="保存", command=self._save_acc)
        self.btn_acc_save.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        # 异步回读八节点；主控确认本次回包齐全后返回 SYNC_ACC finish。
        self.btn_acc_sync = tk.Button(param_f, text="同步加速度至主控", font=("Arial", 9, "bold"),
                                      bg="#1971c2", fg="white", relief=tk.FLAT, command=self._sync_acc)
        self.btn_acc_sync.pack(fill=tk.X, padx=2, pady=(2, 4))
        ttk.Label(param_f, text="提示：建议在失能状态下同步加速度。\n"
                               "使能时若超时，请支撑好机械臂后失能重试。",
                  font=("Arial", 8), foreground="#b26a00", justify=tk.LEFT
                  ).pack(fill=tk.X, padx=2, pady=(0, 4))

        # 电流限制（A）Entry + 读取 / 应用 / 保存 按钮
        # 应用 = #I_LIMIT_J n v（不写 EEPROM，掉电丢）；保存 = #I_LIMIT_J n v &（走 SetCurrentLimit_persist 写 EEPROM）
        i_f = ttk.Frame(param_f)
        i_f.pack(fill=tk.X, pady=2)
        ttk.Label(i_f, text="I_LIMIT (A):", font=("Arial", 10)).pack(side=tk.LEFT)
        self.ent_set_i = ttk.Entry(i_f, width=8, font=("Arial", 10))
        self.ent_set_i.insert(0, "1.5")
        self.ent_set_i.pack(side=tk.LEFT, padx=4)
        ttk.Button(i_f, text="读取 I_LIMIT", command=self._qry_i).pack(side=tk.LEFT, padx=2)
        self.btn_i_apply = ttk.Button(i_f, text="应用", command=self._set_i)
        self.btn_i_apply.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        self.btn_i_save = ttk.Button(i_f, text="保存", command=self._save_i)
        self.btn_i_save.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        self.btn_encoder_cal = ttk.Button(param_f, text="磁编码器校准（选中节点）",
                                          command=self._cmd_encoder_calibration)
        self.btn_encoder_cal.pack(fill=tk.X, padx=2, pady=(6, 2))
        ttk.Label(param_f, text="先失能；校准会正反转，成功后该电机自动重启。",
                  font=("Arial", 8), foreground="#b26a00").pack(fill=tk.X, padx=2, pady=2)

        # 节点=0 (ALL) 时禁用参数写入和磁编码器校准，避免广播。
        self._write_buttons = [
            self.btn_pid_save,
            self.btn_acc_apply, self.btn_acc_save,
            self.btn_i_apply, self.btn_i_save,
            self.btn_encoder_cal,
        ]
        self.param_node.trace_add("write", lambda *_: self._update_write_btn_state())
        self._update_write_btn_state()

        ttk.Separator(param_f).pack(fill=tk.X, pady=6)
        ttk.Button(param_f, text="堵转 / 校准参数…", command=self._open_protection_parameters
                   ).pack(fill=tk.X, padx=2)
        ttk.Label(param_f, text="普通保护仅 J1–J7；J8 仅校准寻限位。",
                  font=("Arial", 8), foreground="#868e96").pack(fill=tk.X, padx=2, pady=2)

        # ==============================================
        # 右栏：Notebook（关节控制 / MoveL / 夹爪）
        # ==============================================
        right_col = ttk.Frame(control_area)
        right_col.grid(row=0, column=2, sticky="nsew", padx=(0, 0))
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

        hand_page = ttk.Frame(self.joint_nb, padding=5)
        self.joint_nb.add(hand_page, text=" 夹爪 ")
        self._build_hand_page(hand_page)

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
                            self._schedule_motion_callback(self.send_movej)
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
                    self._schedule_motion_callback(self.send_movej)
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
        ttk.Label(ctrl_f, text="速度(1–100):", font=("Arial", 9)).pack(side=tk.LEFT, padx=(0, 2))
        self.ent_j_speed = ttk.Entry(ctrl_f, width=6, font=("Arial", 9))
        self.ent_j_speed.insert(0, str(DEFAULT_SPEED))
        self.ent_j_speed.pack(side=tk.LEFT)
        ttk.Label(ctrl_f, text="100 = 30 r/s", font=("Arial", 9)).pack(side=tk.LEFT, padx=4)
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
        ttk.Label(pos_bot, text="速度(1–100):", font=("Arial", 10)).pack(side=tk.LEFT)
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
                            self._schedule_motion_callback(self.send_movel)
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
        ttk.Label(movel_ctrl, text="速度(1–100):", font=("Arial", 10)).pack(side=tk.LEFT, padx=(0, 2))
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
        now = time.time()
        time_str = time.strftime("%H:%M:%S", time.localtime(now)) + f".{int(now % 1 * 1000):03d}"
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
                self._stop_rx_fragment = ""
                self.current_cmdmode = None
                self._mode_pending = self._mode_request = None
                self._calibration_busy = False
                self._stop_locked = False
                self._update_motor_state(None)
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
            self._calibration_busy = False
            self._stop_locked = False
            self._update_motor_state(None)
            # 清掉所有等待中的异步状态，避免重连后被上一轮的残留状态卡住
            self._pending_sync = None
            self._sync_data = None
            self._cancel_motion_for_stop()
            self.current_cmdmode = None
            self._set_acc_sync_waiting(False)
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
                        for line in self._split_rx_stop_status(text):
                            self._handle_rx_line(line)
                # 无数据时短暂休眠，不阻塞主循环（接收轮询周期 10ms，降低交互延迟）
                time.sleep(0.01)
            except serial.SerialException as e:
                self.root.after(0, self.log, f"串口异常: {e}", "ERROR")
                break
            except Exception as e:
                self.root.after(0, self.log, f"读取异常: {e}", "ERROR")
                break

    def _split_rx_stop_status(self, text):
        """所有回复均按完整行解析，部分读取不能形成运动完成事件。"""
        text = self._stop_rx_fragment + text
        rows = text.split('\n')
        self._stop_rx_fragment = rows.pop()
        if len(self._stop_rx_fragment) > 1024:
            rows.append(self._stop_rx_fragment)
            self._stop_rx_fragment = ""
        return self._split_rx_lines('\n'.join(rows))

    @staticmethod
    def _split_rx_lines(text):
        """保留完整回复；不猜测 ok 子串边界。"""
        return [row.strip() for row in text.split('\n') if row.strip()]

    def _handle_rx_line(self, line):
        """RX 线程只投递完整行，Tk 变量、顺序发送和控件均由主线程处理。"""
        self.root.after(0, self._process_rx_line, line)

    def _process_rx_line(self, line):
        """在 Tk 主线程处理回执，避免从 RX 线程读取 IntVar 等 Tk 对象。"""
        # 裸数字 = 固件指令 FIFO 剩余空位：255 说明队列已满，其余仅是余量计数，静默过滤
        if line.isdigit():
            if line == "255":
                self.root.after(0, self.log,
                                "指令队列已满(FIFO 16)，该指令被固件丢弃，请降低发送速率", "WARN")
            return

        self.root.after(0, self.log, line, "RX")
        if line in ("ok SYNC_ACC pending", "ok SYNC_ACC finish") or line.startswith("error SYNC_ACC "):
            self.root.after(0, self._handle_acc_sync_status, line)
        if line in ("Stopping ok", "STOP locked", "STOP locked disabled", "Start pending",
                    "Disable pending", "Started ok", "Disabled ok", "error STOP_LOCKED") or line.startswith("STOP fault "):
            self._handle_stop_status(line)

        if line.startswith(("!CALIBRATION ", "CAL ", "error CALIBRATION ")):
            self._handle_calibration_status(line)
        elif line.startswith("STALL "):
            self._handle_stall_status(line)
        elif line.startswith("PROTECT ") and " failed reason=" in line:
            self._handle_communication_protection(line)
        elif line == "error START busy":
            self.log("参数任务正在进行，START 未受理；完成后请重新发送，不会自动使能", "WARN")
        elif re.fullmatch(r"ok ENC_CAL \[[1-8]\] dispatched", line):
            self.log("磁编码器校准 CAN 帧已提交；协议没有完成回执，请观察电机校准及重启结果", "INFO")
        self._handle_protection_parameter_status(line)

        # 电机参数查询回包解析：节点匹配且非 0 时，把数值回填到右侧 Entry（仅 UI 辅助，不影响主日志）
        m = re.match(r'ok PID \[(\d+)\] kp=(-?\d+) kv=(-?\d+) ki=(-?\d+) kd=(-?\d+)', line)
        if m:
            n = int(m.group(1))
            if n != 0 and n == self.param_node.get():
                kp, kv, ki, kd = m.group(2), m.group(3), m.group(4), m.group(5)
                self.root.after(0, self._fill_pid_entries, kp, kv, ki, kd)
        m = re.match(r'ok ACC \[(\d+)\] = ([\d.]+) r/s\^2', line)
        if m:
            n = int(m.group(1))
            if n != 0 and n == self.param_node.get():
                v = m.group(2)
                self.root.after(0, self._fill_acc_entry, v)
        m = re.match(r'ok I_LIMIT \[(\d+)\] = ([\d.]+) A', line)
        if m:
            n = int(m.group(1))
            if n != 0 and n == self.param_node.get():
                v = m.group(2)
                self.root.after(0, self._fill_i_entry, v)
        # ok GET_* ALL dispatched / ok SET ... / error SET ... 等行已走 RX 日志，无需额外处理

        # #GETJPOS / #GETLPOS 回包 → 同步滑块（"ok" + 6 个数）
        # 注意：这里只落数据，不清 _pending_sync。两种回包格式完全相同，无法从内容
        # 区分来源，必须由 500ms 定时器统一收尾时再连同 kind 一起取走。
        if self._pending_sync and line.startswith("ok"):
            nums = self._parse_numbers(line[2:].strip())
            expected = 7 if self._pending_sync == "j" else 6
            if nums is not None and len(nums) == expected:
                self._sync_data = nums

        self._handle_motion_receipt(line)

    def _stop_sequence(self, reason):
        self._seq_active = self._seq_running = self._seq_wait_mode = False
        self._seq_motion_id = None
        self._seq_session += 1
        self.log(f"顺序发送停止：{reason}", "WARN")

    def _query_command_mode(self):
        self._mode_request_counter = (self._mode_request_counter + 1) & 0xFFFFFFFF or 1
        self._mode_request = self._mode_request_counter
        if not self.send_cmd(f"#GET_CMDMODE {self._mode_request}"):
            self._mode_request = None
            if self._seq_active:
                self._stop_sequence("模式查询未发送")
            return
        self.root.after(3000, self._mode_query_timeout, self._mode_request)

    def _mode_query_timeout(self, request):
        if self._mode_request != request:
            return
        self._mode_request = None
        self.log("主控模式查询超时；未启动顺序发送", "WARN")
        if self._seq_active:
            self._stop_sequence("模式未确认")

    def _mode_set_timeout(self, mode, token):
        if self._mode_pending == mode and self._mode_set_token == token:
            self._mode_pending = None
            self.current_cmdmode = None
            self.log("模式设置回执超时，模式状态未知，请查询确认", "WARN")

    def _sequence_queue_timeout(self, session, index):
        if self._seq_active and session == self._seq_session and index == self._seq_idx and self._seq_motion_id is None:
            self._stop_sequence("未收到本条点位的 queued 编号；请检查固件版本和日志")

    def _handle_motion_receipt(self, line):
        m = re.fullmatch(r'ok CMDMODE mode=(\d+) busy=(\d+) issued=(\d+) req=(\d+)', line)
        if m:
            mode, busy, issued, request = map(int, m.groups())
            if request != self._mode_request:
                return
            self._mode_request = None
            self.current_cmdmode = mode
            if self._seq_wait_mode:
                self._seq_wait_mode = False
                if mode != 1 or busy:
                    self._stop_sequence("主控必须处于空闲 SEQ；请切换模式并等待当前运动完成")
                else:
                    self._seq_id_floor = issued
                    self.log(f"开始顺序发送 {len(self._seq_pending)} 个点位，等待各自编号完成", "INFO")
                    self._send_next_position()
            return
        m = re.fullmatch(r'ok CMDMODE mode=(\d+)', line)
        if m:
            mode = int(m.group(1))
            if mode == self._mode_pending:
                self._mode_pending = None
                self.current_cmdmode = mode
                self.log(f"主控确认模式 {mode} ({self._MODE_NAME.get(mode, '?')})", "INFO")
            return
        if line.startswith('error CMDMODE'):
            self._mode_pending = self._mode_request = None
            if self._seq_active:
                self._stop_sequence("主控拒绝模式请求")
            return
        if not self._seq_active or not self._seq_running:
            return
        m = re.fullmatch(r'queued MOVEJ id=(\d+) mode=(\w+)', line)
        if m:
            motion_id, mode = int(m.group(1)), m.group(2)
            delta = (motion_id - self._seq_id_floor) & 0xFFFFFFFF
            if self._seq_motion_id is None and 0 < delta < 0x80000000:
                if mode != 'SEQ':
                    self._stop_sequence("点位实际使用模式不是 SEQ")
                    self.send_cmd("!STOP")
                else:
                    self._seq_motion_id = motion_id
                    self._seq_id_floor = motion_id
            return
        m = re.fullmatch(r'ok MOVEJ id=(\d+)(?: ms=\d+)?', line)
        if m and int(m.group(1)) == self._seq_motion_id:
            self._seq_running = False
            self._seq_motion_id = None
            self._seq_idx += 1
            if self._seq_idx < len(self._seq_pending):
                session = self._seq_session
                self.root.after(0, lambda: self._send_next_position() if self._seq_active and session == self._seq_session else None)
            else:
                self._seq_active = False
                self.log(f"顺序发送完成，共 {len(self._seq_pending)} 个点位", "INFO")
            return
        m = re.fullmatch(r'error MOVEJ id=(\d+) reason=(\w+)', line)
        if m and int(m.group(1)) == self._seq_motion_id:
            self._stop_sequence(m.group(2))
        elif line == 'error motion queue full' and self._seq_motion_id is None:
            self._stop_sequence("主控队列已满")

    def _serial_write_loop(self):
        """守护发送线程：从队列取命令，经串口写出。"""
        while True:
            item = self._tx_queue.get()
            if item is None:  # 停止哨兵
                break
            generation, cmd = item
            try:
                with self._serial_write_lock:
                    if generation != self._tx_generation:
                        continue
                    diagnostic = self._is_protection_status_query(cmd)
                    if self._stop_locked and not diagnostic and cmd not in ("!STOP", "!START", "!DISABLE"):
                        continue
                    if self._calibration_busy and not diagnostic and cmd not in ("!CALIBRATION", "!STOP", "!DISABLE"):
                        continue
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

    # ==============================================
    # 电机参数查询（!GET_PID / !GETJACC / !GETI）
    # ==============================================
    def _qry_pid(self):
        self.send_cmd(f"#GET_PID {self.param_node.get()}")

    def _qry_acc(self):
        self.send_cmd(f"#GETJACC {self.param_node.get()}")

    def _qry_i(self):
        self.send_cmd(f"#GETI {self.param_node.get()}")

    def _sync_acc(self):
        """发起独立异步同步；查询按钮仍沿用 #GETJACC <node>。"""
        if self._acc_sync_waiting:
            return
        if not self.is_connected or not self.serial_port or self._stop_locked:
            self.send_cmd("#SYNC_ACC")  # 复用未连接 / STOP 锁定提示。
            return
        self._set_acc_sync_waiting(True)
        self.send_cmd("#SYNC_ACC")

    def _set_acc_sync_waiting(self, waiting):
        """只由 Tk 主线程调用；2 秒无终结回执时允许重试。"""
        if self._acc_sync_timeout_id is not None:
            self.root.after_cancel(self._acc_sync_timeout_id)
            self._acc_sync_timeout_id = None
        self._acc_sync_waiting = waiting
        self.btn_acc_sync.config(text="同步中…" if waiting else "同步加速度至主控",
                                 state=tk.DISABLED if waiting else tk.NORMAL)
        if waiting:
            self._acc_sync_timeout_id = self.root.after(2000, self._acc_sync_no_reply)

    def _acc_sync_no_reply(self):
        self._acc_sync_timeout_id = None
        self._set_acc_sync_waiting(False)
        self.log("加速度同步未收到完成回执，请确认主控固件支持 #SYNC_ACC 后重试", "WARN")

    def _handle_acc_sync_status(self, line):
        if line == "ok SYNC_ACC pending":
            self._set_acc_sync_waiting(True)
        elif line == "ok SYNC_ACC finish":
            self._set_acc_sync_waiting(False)
            self.log("八节点加速度已回读并同步至主控缓存", "INFO")
        elif line == "error SYNC_ACC busy":
            # 已有同一 USB 通道的同步任务时，继续等它的终结回执。
            if self._acc_sync_waiting:
                self.log("主控已有加速度同步任务，继续等待结果", "WARN")
        elif line.startswith("error SYNC_ACC "):
            self._set_acc_sync_waiting(False)
            self.log(f"加速度同步失败：{line}", "ERROR")

    def _fill_pid_entries(self, kp, kv, ki, kd):
        """PID 回包解析后回填 Entry（仅 UI 辅助，不影响主日志）"""
        for ent, val in [(self.ent_set_pid_kp, kp), (self.ent_set_pid_kv, kv),
                         (self.ent_set_pid_ki, ki), (self.ent_set_pid_kd, kd)]:
            ent.delete(0, tk.END)
            ent.insert(0, val)

    def _fill_acc_entry(self, val):
        """ACC 回包解析后回填 Entry"""
        self.ent_set_acc.delete(0, tk.END)
        self.ent_set_acc.insert(0, val)

    def _fill_i_entry(self, val):
        """I_LIMIT 回包解析后回填 Entry"""
        self.ent_set_i.delete(0, tk.END)
        self.ent_set_i.insert(0, val)

    def _update_write_btn_state(self):
        """节点=0 (ALL) 时禁用写入按钮，避免误写广播"""
        state = "disabled" if self.param_node.get() == 0 else "normal"
        for btn in self._write_buttons:
            btn.config(state=state)

    def _open_protection_parameters(self):
        """独立参数窗口，避免挤压原有控制面板；读取值按节点缓存。"""
        if self._protection_window is not None and self._protection_window.winfo_exists():
            self._protection_window.lift()
            return
        window = tk.Toplevel(self.root)
        self._protection_window = window
        window.title("堵转 / 校准参数")
        window.geometry("840x810")
        window.minsize(800, 620)
        notebook = ttk.Notebook(window)
        notebook.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)
        self._protection_nodes = {}
        self._protection_rows = {"STALL": {}, "CAL": {}}
        self._protection_entries = {"STALL": {}, "CAL": {}}
        self._protection_save_buttons = {}
        for kind, title, descriptors in (("STALL", "普通堵转 J1–J7", STALL_PARAMETERS),
                                         ("CAL", "校准寻限位 J7/J8", CAL_PARAMETERS)):
            page = ttk.Frame(notebook, padding=8)
            notebook.add(page, text=title)
            top = ttk.Frame(page)
            top.pack(fill=tk.X)
            ttk.Label(top, text="节点:").pack(side=tk.LEFT)
            node_var = tk.IntVar(value=1 if kind == "STALL" else 7)
            self._protection_nodes[kind] = node_var
            ttk.Combobox(top, textvariable=node_var, width=5, state="readonly",
                         values=list(range(9)) if kind == "STALL" else [7, 8]
                         ).pack(side=tk.LEFT, padx=6)
            ttk.Button(top, text="读取全部参数", command=lambda k=kind: self._query_protection(k)
                       ).pack(side=tk.LEFT, padx=6)
            if kind == "STALL":
                ttk.Button(top, text="读取运行快照", command=self._query_stall_status
                           ).pack(side=tk.LEFT, padx=6)
                ttk.Button(top, text="读取触发快照", command=lambda: self._query_stall_status(last=True)
                           ).pack(side=tk.LEFT, padx=6)
            else:
                ttk.Button(top, text="读取校准快照", command=self._query_cal_status
                           ).pack(side=tk.LEFT, padx=6)
                ttk.Button(top, text="读取失败快照", command=lambda: self._query_cal_status(last=True)
                           ).pack(side=tk.LEFT, padx=6)
            save = ttk.Button(top, text="保存节点 EEPROM", command=lambda k=kind: self._save_protection(k))
            save.pack(side=tk.LEFT, padx=6)
            self._protection_save_buttons[kind] = save
            hint = ("0=ALL 展开 J1–J7；J8 排除普通检测。回退速度为减速箱前电机轴。"
                    if kind == "STALL" else "J8 接触检测仅用于 CALIBRATION，不参与普通堵转。")
            ttk.Label(page, text=hint, foreground="#868e96").pack(anchor="w", pady=(8, 2))
            ttk.Label(page, text="数值为建议初值，先读取实际节点。应用仅 RAM；保存必须失能。",
                      foreground="#b26a00").pack(anchor="w", pady=(0, 8))
            canvas_frame = ttk.Frame(page)
            canvas_frame.pack(fill=tk.BOTH, expand=True)
            canvas = tk.Canvas(canvas_frame, highlightthickness=0)
            scroll = ttk.Scrollbar(canvas_frame, orient=tk.VERTICAL, command=canvas.yview)
            canvas.configure(yscrollcommand=scroll.set)
            scroll.pack(side=tk.RIGHT, fill=tk.Y)
            canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
            table = ttk.Frame(canvas)
            table_window = canvas.create_window((0, 0), window=table, anchor="nw")
            table.bind("<Configure>", lambda event, c=canvas: c.configure(scrollregion=c.bbox("all")))
            canvas.bind("<Configure>", lambda event, c=canvas, w=table_window: c.itemconfigure(w, width=event.width))
            for col, label in enumerate(("参数 / 命令键", "单位", "值", "")):
                ttk.Label(table, text=label, font=("Arial", 10, "bold")).grid(row=0, column=col, sticky="w", padx=4, pady=4)
            table.columnconfigure(0, weight=1)
            for row, (key, label, unit, default, _minimum, _maximum) in enumerate(descriptors, 1):
                ttk.Label(table, text=f"{label}\n{key}").grid(row=row, column=0, sticky="w", padx=4, pady=3)
                ttk.Label(table, text=unit).grid(row=row, column=1, sticky="w", padx=4)
                entry = ttk.Entry(table, width=12)
                entry.insert(0, f"{default:g}")
                entry.grid(row=row, column=2, padx=6)
                apply_button = ttk.Button(table, text="应用", width=7,
                                          command=lambda k=kind, p=key: self._set_protection(k, p))
                apply_button.grid(row=row, column=3, padx=4)
                self._protection_entries[kind][key] = entry
                self._protection_rows[kind][key] = apply_button
            node_var.trace_add("write", lambda *_, k=kind: self._refresh_protection_node(k))
            self._refresh_protection_node(kind)

    def _refresh_protection_node(self, kind):
        node = self._protection_nodes[kind].get()
        descriptors = STALL_PARAMETERS if kind == "STALL" else CAL_PARAMETERS
        cached = self._protection_values[kind].get(node, {})
        for key, _label, _unit, default, _minimum, _maximum in descriptors:
            applicable = not (kind == "STALL" and (node == 8 or
                             (key == "BACK_DEG" and node == 7) or
                             (key == "BACK_MM" and node not in (0, 7))))
            self._protection_rows[kind][key].config(state=tk.NORMAL if applicable else tk.DISABLED)
            entry = self._protection_entries[kind][key]
            if kind == "STALL" and key == "BACK_RPS" and node == 7:
                default = 0.5
            elif kind == "STALL" and key == "ERROR_R":
                default = 0.1 if node == 7 else 0.5 / 360 * (30 if node == 6 else 50)
            elif kind == "CAL" and node == 8:
                default = {"SEEK_RPS": 2, "RETURN_RPS": 2, "ACCEL": 5,
                           "CURRENT_A": 1, "CONTACT_RATIO": 0.2, "MAX_TRAVEL_R": 6,
                           "SEEK_TIMEOUT_MS": 30000, "RETURN_TIMEOUT_MS": 15000,
                           "BACKOFF_R": 0.5, "REPEAT_TOL_R": 0.1, "POSITION_TOL_R": 0.01,
                           "CONTACT_WINDOW_R": 5 / 100 * 100 / 360 * 16}.get(key, default)
            value = cached.get(key, f"{default:g}")
            entry.delete(0, tk.END)
            entry.insert(0, value)
        self._protection_save_buttons[kind].config(state=tk.DISABLED if kind == "STALL" and node == 8 else tk.NORMAL)

    def _query_protection(self, kind):
        self.send_cmd(f"#{kind}_GET {self._protection_nodes[kind].get()}")

    @staticmethod
    def _is_stall_status_query(cmd):
        return re.fullmatch(r"#STALL_STATUS\s+[1-7](?:\s+LAST)?", cmd) is not None

    @staticmethod
    def _is_cal_status_query(cmd):
        return re.fullmatch(r"#CAL_STATUS\s+[78](?:\s+LAST)?", cmd) is not None

    def _is_protection_status_query(self, cmd):
        return self._is_stall_status_query(cmd) or self._is_cal_status_query(cmd)

    def _query_cal_status(self, last=False):
        node = self._protection_nodes["CAL"].get()
        if node not in (7, 8):
            self.log("校准快照仅支持单个节点 J7、J8，请切换节点", "WARN")
            return
        self.send_cmd(f"#CAL_STATUS {node}" + (" LAST" if last else ""))

    def _query_stall_status(self, last=False):
        node = self._protection_nodes["STALL"].get()
        if node not in range(1, 8):
            self.log("堵转快照仅支持单个节点 J1–J7，请切换节点", "WARN")
            return
        self.send_cmd(f"#STALL_STATUS {node}" + (" LAST" if last else ""))

    def _set_protection(self, kind, key):
        node = self._protection_nodes[kind].get()
        descriptors = STALL_PARAMETERS if kind == "STALL" else CAL_PARAMETERS
        descriptor = next(p for p in descriptors if p[0] == key)
        if kind == "STALL" and (node == 8 or (key == "BACK_DEG" and node == 7) or
                                (key == "BACK_MM" and node not in (0, 7))):
            self.log("该普通堵转参数不适用于所选节点", "WARN")
            return
        try:
            value = float(self._protection_entries[kind][key].get())
            minimum, maximum = descriptor[4:6]
            if (not math.isfinite(value) or value < minimum or
                    (minimum == 0 and key not in ("STARTUP_MS", "ENABLE") and value == 0) or
                    (maximum is not None and value > maximum)):
                raise ValueError("超出参数范围")
            if (key.endswith("_MS") or key == "ENABLE") and not value.is_integer():
                raise ValueError("时间/开关参数必须为整数")
        except ValueError as exc:
            messagebox.showerror("参数错误", f"{key}: {exc}\n单位：{descriptor[2]}", parent=self._protection_window)
            return
        self.send_cmd(f"#{kind}_SET {node} {key} {value:g}")

    def _save_protection(self, kind):
        if self._motor_state_known and self._motor_enabled:
            messagebox.showwarning("保存需要失能", "请先 DISABLE 并等待确认，再保存 EEPROM。",
                                   parent=self._protection_window)
            return
        self.send_cmd(f"#{kind}_SAVE {self._protection_nodes[kind].get()}")

    def _handle_protection_parameter_status(self, line):
        match = re.fullmatch(r"ok (STALL|CAL)(?:_SET)? \[(\d+)\] ([A-Z_0-9]+)=([^\s]+)(?:\s+.*)?", line)
        if not match:
            return
        kind, node, key, value = match.group(1), int(match.group(2)), match.group(3), match.group(4)
        try:
            if not math.isfinite(float(value)):
                return
        except ValueError:
            return
        if kind == "STALL" and key == "BACK_DISTANCE_R" and 1 <= node <= 7:
            motor_r = float(value)
            key = "BACK_MM" if node == 7 else "BACK_DEG"
            value = f"{motor_r * 5 if node == 7 else motor_r * 360 / (30 if node == 6 else 50):.6g}"
        self._protection_values[kind].setdefault(node, {})[key] = value
        window = self._protection_window
        if (window is not None and window.winfo_exists() and
                self._protection_nodes[kind].get() == node and key in self._protection_entries[kind]):
            entry = self._protection_entries[kind][key]
            entry.delete(0, tk.END)
            entry.insert(0, value)

    # ==============================================
    # 电机参数设置与磁编码器校准（#SET_PID / #ACC_J / #I_LIMIT_J / #ENC_CAL）
    # 与查询共用 param_node 下拉；写入语义由 panel 注释写明（决策 #1/#2，2026-10-04）：
    #   - #SET_PID        ：永远写 EEPROM（固件无 RAM/EEPROM 区分）
    #   - #ACC_J          ：默认不写 EEPROM；末尾追加 `&` 才写 EEPROM
    #   - #I_LIMIT_J      ：默认不写 EEPROM；末尾追加 `&` 才写 EEPROM
    # 因此 ACC/I 各有两个回调：_set_* 发不带 & 的命令（应用），
    # _save_* 复用同一份入参校验，仅在 send_cmd 字符串末尾追加 `&`（保存）。
    # ==============================================
    def _cmd_encoder_calibration(self):
        """对电机参数区选中的单个节点触发磁编码器校准。"""
        try:
            node = self.param_node.get()
        except (ValueError, tk.TclError):
            self.log("请选择单个节点 1～8 进行磁编码器校准", "WARN")
            return
        if node not in range(1, 9):
            self.log("磁编码器校准仅支持单个节点 1～8，禁止 ALL 广播", "WARN")
            return
        cmd = f"#ENC_CAL {node}"
        if not self.is_connected or not self.serial_port:
            self.send_cmd(cmd)
            return
        if (self._stop_locked or self._calibration_busy or self._seq_active or self._acc_sync_waiting or
                not self._motor_state_known or self._motor_enabled):
            self.log("请先结束当前任务、支撑好机械臂并失能，等待 Disabled ok 后再校准磁编码器", "WARN")
            return
        target = f"J{node}" if node <= 6 else "地轨 J7" if node == 7 else "夹爪 J8"
        if not messagebox.askyesno(
            "确认磁编码器校准",
            f"将校准 {target}（节点 {node}）的磁编码器。\n"
            "该电机会主动正反转采样、写入校准表，成功后自动重启。\n"
            "请支撑好机械臂，并确认该轴有足够运动空间。\n"
            "校准期间请勿发送其他控制命令；完成结果需观察电机。\n\n"
            "确定开始吗？",
            parent=self.root,
        ):
            return
        self.send_cmd(cmd)

    def _set_pid(self):
        """写 4 个 PID 值到指定节点（写 EEPROM，#SET_PID 一次发 4 个值）"""
        try:
            node = self.param_node.get()
            kp = int(self.ent_set_pid_kp.get())
            kv = int(self.ent_set_pid_kv.get())
            ki = int(self.ent_set_pid_ki.get())
            kd = int(self.ent_set_pid_kd.get())
        except ValueError:
            messagebox.showerror("错误", "PID 必须为整数")
            return
        self.send_cmd(f"#SET_PID {node} {kp} {kv} {ki} {kd}")

    def _set_acc(self):
        """应用节点加速度（不写 EEPROM）：发送 #ACC_J <node> <r/s²>"""
        try:
            node = self.param_node.get()
            acc = float(self.ent_set_acc.get())
        except ValueError:
            messagebox.showerror("错误", "加速度必须为数值（r/s²）")
            return
        if acc < 0:
            messagebox.showerror("错误", "加速度不能为负数")
            return
        if acc > 5000.0:
            acc = 5000.0
        self.send_cmd(f"#ACC_J {node} {acc}")

    def _save_acc(self):
        """保存节点加速度到 EEPROM：发送 #ACC_J <node> <r/s²> &
        末尾 `&` 触发固件解析 canBuf[4]=1 走 SetAcceleration_persist 写 EEPROM（决策#2）。"""
        try:
            node = self.param_node.get()
            acc = float(self.ent_set_acc.get())
        except ValueError:
            messagebox.showerror("错误", "加速度必须为数值（r/s²）")
            return
        if acc < 0:
            messagebox.showerror("错误", "加速度不能为负数")
            return
        if acc > 5000.0:
            acc = 5000.0
        self.send_cmd(f"#ACC_J {node} {acc} &")

    def _set_i(self):
        """应用节点电流限制（不写 EEPROM）：发送 #I_LIMIT_J <node> <A>"""
        try:
            node = self.param_node.get()
            i_limit = float(self.ent_set_i.get())
        except ValueError:
            messagebox.showerror("错误", "电流必须为数值（A）")
            return
        if i_limit <= 0:
            messagebox.showerror("错误", "电流必须大于 0")
            return
        if i_limit > 3.0:
            i_limit = 3.0
        self.send_cmd(f"#I_LIMIT_J {node} {i_limit}")

    def _save_i(self):
        """保存节点电流限制到 EEPROM：发送 #I_LIMIT_J <node> <A> &
        末尾 `&` 触发固件解析 canBuf[4]=1 走 SetCurrentLimit_persist 写 EEPROM（决策#2）。"""
        try:
            node = self.param_node.get()
            i_limit = float(self.ent_set_i.get())
        except ValueError:
            messagebox.showerror("错误", "电流必须为数值（A）")
            return
        if i_limit <= 0:
            messagebox.showerror("错误", "电流必须大于 0")
            return
        if i_limit > 3.0:
            i_limit = 3.0
        self.send_cmd(f"#I_LIMIT_J {node} {i_limit} &")

    # ==============================================
    # 夹爪控制页（!HAND_*）
    # ==============================================
    def _build_hand_page(self, parent):
        """夹爪控制面板：开 / 闭 / 设电流 / 设位置 4 个功能。
        命令映射（与 firmware_new ascii_protocol.cpp OnUsbAsciiCmd + OnUart4AsciiCmd 一致）：
          !HAND_O         → 张开（不带参数，用 self.hand_current 作为幅值；不写 EEPROM）
          !HAND_C         → 闭合（同 !HAND_O 语义，电流取正）
          !HAND_I <A>     → 设运行时电流幅值 0.05~2.5A（纯 RAM 变量，掉电丢；与基线 1.0A 一致）
          !HAND_POS <0-100> → 设位置 0~100%（override 后 pos=电机圈数；限速 30 r/s；不写 EEPROM）
        注意：!HAND_VEL/ACC/KP/KV/KI/KD/ONBOOT/ERASE 8 条配置类本期不暴露 UI，
        等配置需求时再加折叠面板（避免误操作改写电机端 EEPROM）。
        """
        # --- 电流设置 ---
        cur_f = ttk.LabelFrame(parent, text="电流设置（运行时，掉电丢）", padding=6)
        cur_f.pack(fill=tk.X, pady=(0, 6))
        cur_top = ttk.Frame(cur_f)
        cur_top.pack(fill=tk.X)
        ttk.Label(cur_top, text="电流(A):", font=("Arial", 10, "bold")).pack(side=tk.LEFT)
        self.hand_current = tk.DoubleVar(value=1.0)  # 默认 1.0A（与 firmware_new Q1 决策一致）
        self.spb_current = tk.Spinbox(cur_top, from_=0.05, to=2.5, increment=0.05,
                                      textvariable=self.hand_current, width=6,
                                      format="%.2f", font=("Arial", 10))
        self.spb_current.pack(side=tk.LEFT, padx=4)
        ttk.Label(cur_top, text="范围 0.05~2.5A（步长 0.05）", font=("Arial", 9),
                  foreground="#666").pack(side=tk.LEFT, padx=4)
        cur_btn_row = ttk.Frame(cur_f)
        cur_btn_row.pack(fill=tk.X, pady=(4, 0))
        tk.Button(cur_btn_row, text="设置电流", font=("Arial", 10, "bold"),
                  bg="#3b5bdb", fg="white", relief=tk.FLAT,
                  command=self._cmd_hand_set_i
                  ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 2))
        tk.Button(cur_btn_row, text="开（张开）", font=("Arial", 10, "bold"),
                  bg="#2b8a3e", fg="white", relief=tk.FLAT,
                  command=self._cmd_hand_open
                  ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        tk.Button(cur_btn_row, text="闭（闭合）", font=("Arial", 10, "bold"),
                  bg="#e67700", fg="white", relief=tk.FLAT,
                  command=self._cmd_hand_close
                  ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(2, 0))
        # --- 位置设置 ---
        pos_f = ttk.LabelFrame(parent, text="位置设置（0=闭合，100=张开）", padding=6)
        pos_f.pack(fill=tk.X, pady=(0, 6))
        pos_top = ttk.Frame(pos_f)
        pos_top.pack(fill=tk.X)
        ttk.Label(pos_top, text="位置:", font=("Arial", 10, "bold")).pack(side=tk.LEFT)
        self.hand_pos = tk.IntVar(value=100)
        self.scl_hand_pos = ttk.Scale(pos_top, from_=0, to=100, orient=tk.HORIZONTAL,
                                      variable=self.hand_pos)
        self.scl_hand_pos.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)
        self.lbl_hand_pos = ttk.Label(pos_top, text="100", width=4, font=("Arial", 10, "bold"))
        self.lbl_hand_pos.pack(side=tk.LEFT)
        def hand_pos_cb(val):
            v = int(float(val))
            self.lbl_hand_pos.config(text=f"{v}")
            # [调整] 拖发节流：开关 ON + 非抑制 + 距上次 > 0.1s → 异步调 _cmd_hand_set_pos
            if self.hand_pos_drag_enable.get() and not self._suppress_drag_send:
                now = time.time()
                if now - self.last_hand_pos_send_time > 0.1:
                    self.last_hand_pos_send_time = now
                    self._schedule_motion_callback(self._cmd_hand_set_pos)
        self.scl_hand_pos.config(command=hand_pos_cb)
        pos_btn_row = ttk.Frame(pos_f)
        pos_btn_row.pack(fill=tk.X, pady=(4, 0))
        tk.Button(pos_btn_row, text="设置位置", font=("Arial", 10, "bold"),
                  bg="#3b5bdb", fg="white", relief=tk.FLAT,
                  command=self._cmd_hand_set_pos
                  ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 2))
        # [调整] 拖发开关（仿 movej/movel 的 _make_toggle_button，on_change 触发模式提示）
        self._make_toggle_button(
            pos_btn_row, self.hand_pos_drag_enable,
            text_off="拖发 OFF", text_on="拖发 ON",
            bg_off="#e9ecef", bg_on="#2b8a3e",
            fg_off="#495057", fg_on="white",
            font=("Arial", 10, "bold"),
            side=tk.LEFT, padx=2,
            on_change=self._warn_drag_mode,
        )
        # --- 配置类说明（折叠提示） ---
        info_f = ttk.LabelFrame(parent, text="说明", padding=6)
        info_f.pack(fill=tk.X)
        info_text = (
            "• 开/闭：电流透传（张开=负电流 / 闭合=正电流，电流幅值=Spinbox 值）\n"
            "• 电流：仅改 RAM 变量 hand->current，不发 CAN 帧；掉电丢\n"
            "• 位置：pos 直接对应电机圈数；限速 30 r/s；不写 EEPROM\n"
            "• 配置类（!HAND_VEL/ACC/KP/KV/KI/KD/ONBOOT/ERASE）需谨慎，本期不暴露\n"
            "• UART4 限制：仅开/闭/设位置/设电流 4 条可发；配置类串口助手用 USB 通道"
        )
        ttk.Label(info_f, text=info_text, font=("Arial", 9),
                  foreground="#444", justify=tk.LEFT).pack(anchor="w")

    # ==============================================
    # 夹爪控制回调
    # ==============================================
    def _cmd_hand_open(self):
        """!HAND_O — 张开方向电流（-A），电流幅值用 self.hand_current。"""
        a = self.hand_current.get()
        self.send_cmd(f"!HAND_O {a:.2f}")

    def _cmd_hand_close(self):
        """!HAND_C — 闭合方向电流（+A），电流幅值用 self.hand_current。"""
        a = self.hand_current.get()
        self.send_cmd(f"!HAND_C {a:.2f}")

    def _cmd_hand_set_i(self):
        """!HAND_I — 设置运行时电流幅值（纯 RAM，掉电丢）。"""
        a = self.hand_current.get()
        # 上限 2.5A / 下限 0.05A 已被 Spinbox 限定；额外夹一道防御
        if a < 0.05:
            a = 0.05
        elif a > 2.5:
            a = 2.5
        self.hand_current.set(a)
        self.send_cmd(f"!HAND_I {a:.2f}")

    def _cmd_hand_set_pos(self):
        """!HAND_POS — 设置位置 0~100（int）。"""
        p = self.hand_pos.get()
        if p < 0:
            p = 0
        elif p > 100:
            p = 100
        self.hand_pos.set(p)
        self.send_cmd(f"!HAND_POS {p}")

    def send_cmd(self, cmd, sequence=False):
        """所有指令的唯一出口：入队 + 记 TX 日志，由写线程串行发出"""
        if not self.is_connected or not self.serial_port:
            self.log("未连接串口", "WARN")
            return False
        cmd = cmd.strip()
        if self._seq_active and not sequence and (cmd.startswith(('>', '&', '@', '#CMDMODE')) or cmd in ('!HOME', '!RESET')):
            self.log("顺序发送中，请等待完成或先 STOP；当前运动回执由顺序任务占用", "WARN")
            return False
        diagnostic = self._is_protection_status_query(cmd)
        if self._calibration_busy and not diagnostic and cmd not in ("!STOP", "!DISABLE"):
            self.log("校准及保存中，START 返回 busy，不挂起；可发送 STOP 或 DISABLE", "WARN")
            return False
        if self._stop_locked and not diagnostic and cmd not in ("!STOP", "!START", "!DISABLE"):
            self.log("保护锁定中，仅接受 START、DISABLE；STOP 可中断保护动作", "WARN")
            return False
        if cmd.startswith("#ENC_CAL"):
            if re.fullmatch(r"#ENC_CAL[ \t]+[1-8]", cmd) is None:
                self.log("磁编码器校准格式为 #ENC_CAL <1～8>，禁止 ALL 广播", "WARN")
                return False
            if (self._seq_active or self._acc_sync_waiting or
                    not self._motor_state_known or self._motor_enabled):
                self.log("请先结束当前任务并失能，等待 Disabled ok 后再校准磁编码器", "WARN")
                return False
        if cmd == "!CALIBRATION":
            if self._motor_state_known and self._motor_enabled:
                self._warn_calibration_disabled()
                return False
            self._cancel_motion_for_stop()
            self._calibration_busy = True
            self.lbl_motor_state.config(text="● 校准入口检查中", bg="#495057", fg="#ffd43b")
        if cmd in ("!STOP", "!DISABLE"):
            self._cancel_motion_for_stop()
            self._stop_locked = True
            self._handle_stop_status("Stopping ok" if cmd == "!STOP" else "Disable pending")
        try:
            self._tx_queue.put((self._tx_generation, cmd))
            self.log(cmd, "TX")
            return True
        except Exception as e:
            self.log(f"入队失败: {e}", "ERROR")
            return False

    # ==============================================
    # 系统控制
    # ==============================================
    def _cancel_motion_for_stop(self):
        self._set_acc_sync_waiting(False)
        self._tx_generation += 1
        self._seq_running = self._seq_active = self._seq_wait_mode = False
        self._seq_motion_id = None
        self._seq_session += 1
        self._mode_pending = self._mode_request = None
        self._seq_pending = []
        self._seq_idx = 0
        self._pending_sync = None
        self._sync_data = None
        # 解锁不重新开启拖发；迟到的 after 回调也受 send_cmd 门禁限制。
        self.movej_drag_enable.set(False)
        self.movel_drag_enable.set(False)
        self.hand_pos_drag_enable.set(False)
        while True:
            try:
                self._tx_queue.get_nowait()
            except queue.Empty:
                break

    def _schedule_motion_callback(self, callback):
        """拖发 after 回调带世代；STOP/保护取消后，即使解锁也不能复活旧回调。"""
        generation = self._tx_generation

        def run_if_current():
            if (generation == self._tx_generation and not self._stop_locked and
                    not self._calibration_busy):
                callback()

        self.root.after(1, run_if_current)

    def _handle_stop_status(self, line):
        if line == "Started ok":
            if self._calibration_busy:
                self.log("校准忙态收到 Started ok，保持运动门禁并核查主控状态", "WARN")
                return
            self._stop_locked = False
            self._update_motor_state(True)
            if self._mode_request is None and self._mode_pending is None:
                self._query_command_mode()
        elif line == "Disabled ok":
            self._stop_locked = False
            self._update_motor_state(False)
            if self._calibration_busy:
                self.lbl_motor_state.config(text="● 校准中 / 已失能，等待保存结果", fg="#ffd43b")
        else:
            if not self._stop_locked:
                self._cancel_motion_for_stop()
            self._stop_locked = True
            labels = {
                "Stopping ok": "● 软停减速中",
                "STOP locked": "● STOP 锁定 / 停点保持",
                "STOP locked disabled": "● STOP 锁定 / 电机失能",
                "Start pending": "● 等待停稳并解锁",
                "Disable pending": "● 等待八轴失能确认",
                "error STOP_LOCKED": "● STOP 锁定",
            }
            self.lbl_motor_state.config(text=labels.get(line, "● STOP 故障 / 保持锁定"),
                                        bg="#495057", fg="#ffd43b")

    def _update_motor_state(self, enabled):
        self._motor_state_known = enabled is not None
        self._motor_enabled = enabled is True
        if enabled is None:
            self.lbl_motor_state.config(text="● 电机状态未知", bg="#495057", fg="#ffd43b")
        elif enabled:
            self.lbl_motor_state.config(text="● 电机已使能", bg="#2b8a3e", fg="white")
        else:
            self.lbl_motor_state.config(text="● 电机已失能", bg="#495057", fg="#ff6b6b")

    def _cmd_start(self):
        self.send_cmd("!START")

    def _cmd_disable(self):
        self.send_cmd("!DISABLE")

    def _cmd_calibration(self):
        """失能入口检查由主控最终确认，不自动 DISABLE 或排队校准。"""
        if self._calibration_busy:
            self.send_cmd("!CALIBRATION")
            return
        if self._stop_locked or (self._motor_state_known and self._motor_enabled):
            self._warn_calibration_disabled()
            return
        if not self.is_connected or not self.serial_port:
            self.send_cmd("!CALIBRATION")
            return
        if not messagebox.askyesno(
            "确认置零",
            "J1～J6 保持失能，仅将当前位置定义为 REST_POSE。\n"
            "六轴坐标修正完成后等待1秒，J7、J8 并行自动校准。\n"
            "J8 寻闭合限位并返回开度100；J7 寻正限位并返回0。\n"
            "全部完成后自动八轴失能并保存 EEPROM，不重启。\n\n"
            "请支撑好当前六轴姿态，并留出地轨、夹爪运动空间。\n确定继续吗？",
            parent=self.root,
        ):
            return
        self.send_cmd("!CALIBRATION")

    def _warn_calibration_disabled(self):
        messagebox.showwarning(
            "校准需要失能",
            "校准需要八轴失能。请先点击 DISABLE 并等待失能确认，\n"
            "再重新点击校准。不会自动失能或排队校准。\n"
            "校准完成后会自动失能并保存 EEPROM。",
            parent=self.root,
        )

    def _handle_calibration_status(self, line):
        if line == "!CALIBRATION pending" or line.startswith("CAL "):
            if not self._calibration_busy:
                self._cancel_motion_for_stop()
            self._calibration_busy = True
            stage = "校准进行中" if line == "!CALIBRATION pending" else line[4:]
            self.lbl_motor_state.config(text=f"● 校准 {stage}", bg="#495057", fg="#ffd43b")
        elif line == "!CALIBRATION ok":
            self._cancel_motion_for_stop()
            self._calibration_busy = False
            self._stop_locked = False
            self._update_motor_state(False)
            self._fill_joints(JOINT_DEFAULTS + [RAIL_DEFAULT])
            self.hand_pos.set(100)
            self.lbl_hand_pos.config(text="100")
            self.log("校准、八轴失能与 EEPROM 保存完成；再次 START 将保持当前位置", "INFO")
        elif line == "error CALIBRATION busy":
            # 拒绝 START/重复指令不能结束仍在进行的校准。
            self._calibration_busy = True
        elif line.startswith("error CALIBRATION "):
            self._calibration_busy = False
            self._cancel_motion_for_stop()
            if "require DISABLE" in line or "state unavailable" in line:
                self._update_motor_state(None)
                self._warn_calibration_disabled()
            else:
                self._stop_locked = True
                self.lbl_motor_state.config(text="● 校准失败或取消 / 保护锁定", bg="#495057", fg="#ffd43b")

    def _handle_stall_status(self, line):
        if "EXCLUDED" in line or re.search(r"\bphase=(?:IDLE|0)(?:\s|$)", line):
            return
        if not self._stop_locked:
            self._cancel_motion_for_stop()
        self._calibration_busy = False
        self._stop_locked = True
        self.lbl_motor_state.config(text=f"● 堵转保护 {line[6:]}", bg="#495057", fg="#ffd43b")

    def _handle_communication_protection(self, line):
        if not self._stop_locked:
            self._cancel_motion_for_stop()
        self._calibration_busy = False
        self._stop_locked = True
        self.lbl_motor_state.config(text=f"● 通信/状态保护 {line[8:]}", bg="#495057", fg="#ffd43b")

    def _cmd_reboot(self):
        node = self.cb_node.get()
        self.send_cmd(f"#REBOOT {node}")

    def _set_cmdmode(self, mode):
        """仅主控确认后更新模式，活动任务中不切换。"""
        if self._seq_active or self._mode_pending is not None or self._mode_request is not None:
            self.log("顺序发送或模式确认中，请等待完成", "WARN")
            return
        self._mode_pending = mode
        self._mode_request_counter = (self._mode_request_counter + 1) & 0xFFFFFFFF or 1
        self._mode_set_token = self._mode_request_counter
        if not self.send_cmd(f"#CMDMODE {mode}"):
            self._mode_pending = None
            return
        self.root.after(3000, self._mode_set_timeout, mode, self._mode_set_token)

    def _warn_drag_mode(self):
        """拖发开关切换后回调。若不是「连续模式(3)」则弹提示（不自动切换）。"""
        if (self.movej_drag_enable.get() or self.movel_drag_enable.get()
                or self.hand_pos_drag_enable.get()) \
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
    @staticmethod
    def _validated_speed(value):
        speed = float(value)
        if not math.isfinite(speed) or not 1 <= speed <= 100:
            raise ValueError("速度档位必须为 1–100 的有限数值（100 = 30 r/s）")
        return speed

    def send_movej(self):
        """>j1,j2,j3,j4,j5,j6,j7(rail),speed  （8 个值，v3.1 新协议）"""
        try:
            j = [float(ent.get()) for ent in self.ent_joints]
            j7 = float(self.ent_j7.get())
            speed = self._validated_speed(self.ent_j_speed.get())
            cmd = ">" + ",".join(f"{v:g}" for v in j) + f",{j7:g},{speed:g}"
            self.send_cmd(cmd)
        except ValueError:
            messagebox.showerror("错误", "请输入有效数字；速度档位必须为 1–100（100 = 30 r/s）")

    def send_movel(self):
        """@X,Y,Z,A,B,C,speed  （7 个值）"""
        try:
            p = [float(ent.get()) for ent in self.ent_pose]
            speed = self._validated_speed(self.ent_l_speed.get())
            cmd = "@" + ",".join(f"{v:g}" for v in p) + f",{speed:g}"
            self.send_cmd(cmd)
        except ValueError:
            messagebox.showerror("错误", "请输入有效数字；速度档位必须为 1–100（100 = 30 r/s）")

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
        """把 7 个数写进滑块（新协议：nums[0..5]=J1..J6(°), nums[6]=Rail(mm)）"""
        self._suppress_drag_send = True
        try:
            # 同步 J1~J6 (nums[0..5])
            for i in range(6):
                v = nums[i]
                self.scl_joints[i].set(v)
                self.ent_joints[i].delete(0, tk.END)
                self.ent_joints[i].insert(0, f"{v:.2f}")
                self.lbl_joints[i].config(text=f"{v:.2f}")
            # 同步 Rail (nums[6])
            rv = nums[6]
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
            return self._validated_speed(self.ent_pos_speed.get())
        except ValueError:
            messagebox.showerror("错误", "速度档位必须为 1–100 的有限数值（100 = 30 r/s）")
            return None

    def send_selected_position(self):
        """发送列表中选中的点位"""
        idx = self._pos_listbox.curselection()
        if not idx:
            self.log("请先选择一个点位", "WARN")
            return
        idx = idx[0]
        _name, pos = self.stored_positions[idx]
        speed = self._get_pos_speed()
        if speed is None:
            return
        # [v3.1] >j1,j2,j3,j4,j5,j6,j7(rail),speed （8 个值）
        self.send_cmd(">" + ",".join(f"{v:g}" for v in pos[:7]) + f",{speed:g}")

    def send_all_positions(self):
        """先确认主控 SEQ 且无任务，再逐条等待匹配编号的完成。"""
        if self._stop_locked or self._calibration_busy:
            self.log("保护锁定或校准忙，不能启动顺序发送", "WARN")
            return
        if not self.stored_positions:
            self.log("没有存储的点位", "WARN")
            return
        if self._seq_active or self._mode_pending is not None or self._mode_request is not None:
            self.log("顺序发送或模式确认正在进行中", "WARN")
            return
        speed = self._get_pos_speed()
        if speed is None:
            return
        self._seq_pending = list(self.stored_positions)
        self._seq_idx = 0
        self._seq_speed = speed
        self._seq_active = self._seq_wait_mode = True
        self._seq_session += 1
        self._query_command_mode()

    def _send_next_position(self):
        """只发当前会话的一条点位，旧 after 回调不能恢复取消的任务。"""
        if not self._seq_active or self._stop_locked or self._calibration_busy:
            return
        if self._seq_idx >= len(self._seq_pending):
            return
        _name, pos = self._seq_pending[self._seq_idx]
        self._seq_running = True
        self._seq_motion_id = None
        if not self.send_cmd(">" + ",".join(f"{v:g}" for v in pos[:7]) + f",{self._seq_speed:g}", sequence=True):
            self._stop_sequence("点位发送失败")
            return
        session, index = self._seq_session, self._seq_idx
        self.root.after(3000, self._sequence_queue_timeout, session, index)

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
