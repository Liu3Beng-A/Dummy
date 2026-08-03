import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import serial
import serial.tools.list_ports
import threading
import time
import math

class RobotSerialAssistant:
    def __init__(self, root):
        self.root = root
        self.root.title("Dummy Robot 串口助手")
        self.root.geometry("1280x820")
        self.root.minsize(1000, 600)

        # 根窗口列权重，使主内容区可伸缩
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)

        # ─────────────────────────────────────────────
        #  可调类变量：Home / Rest 位姿（单位：度）
        # ─────────────────────────────────────────────
        self.HOME_POSE = [0, 0, 90, 0, 0, 0]
        self.REST_POSE = [0, -75, 180, 0, 0, 0]
        self.HOME_SPEED = 10      # Home 回零专用速度（建议 5~15）
        self.REST_SPEED = 10      # Rest 复位专用速度（建议 5~15）

        # 图标
        new_icon_data = "R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7"

        try:
            self.icon_img = tk.PhotoImage(data=new_icon_data)
            self.root.iconphoto(True, self.icon_img) # True代表强制覆盖所有子窗体图标
        except Exception:
            pass
            
        self.serial_port = None
        self.is_connected = False
        
        self.create_widgets()
        self.refresh_ports()
        
        self.receive_thread = None
        self.stop_thread = False

    def create_widgets(self):
        # 顶部：连接控制（单行紧凑）
        conn_frame = ttk.LabelFrame(self.root, text="串口连接", padding=(10, 6))
        conn_frame.grid(row=0, column=0, sticky="ew", padx=8, pady=6)
        
        conn_inner = ttk.Frame(conn_frame)
        conn_inner.pack(fill=tk.X, padx=4, pady=4)
        ttk.Label(conn_inner, text="端口:").pack(side=tk.LEFT, padx=(0, 4))
        self.cb_ports = ttk.Combobox(conn_inner, width=12)
        self.cb_ports.pack(side=tk.LEFT, padx=(0, 4))
        ttk.Button(conn_inner, text="刷新", command=self.refresh_ports).pack(side=tk.LEFT, padx=(0, 12))
        ttk.Label(conn_inner, text="波特率:").pack(side=tk.LEFT, padx=(0, 4))
        self.cb_baudrate = ttk.Combobox(conn_inner, width=10, values=["9600", "115200", "1000000"])
        self.cb_baudrate.current(1)
        self.cb_baudrate.pack(side=tk.LEFT, padx=(0, 12))
        self.btn_connect = ttk.Button(conn_inner, text="连接串口", command=self.toggle_connection)
        self.btn_connect.pack(side=tk.LEFT, padx=(0, 12))
        self.lbl_status = ttk.Label(conn_inner, text="未连接", foreground="red")
        self.lbl_status.pack(side=tk.LEFT)

        # 中间：控制面板（左右分栏）
        main_frame = ttk.Frame(self.root, padding=(8, 4))
        main_frame.grid(row=1, column=0, sticky="nsew", padx=8, pady=4)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(0, weight=1)

        # 左侧面板（固定宽度，可滚动感由内部终端撑开）
        left_panel = ttk.Frame(main_frame)
        left_panel.grid(row=0, column=0, sticky="ns", padx=(0, 8))

        # 系统控制区
        sys_frame = ttk.LabelFrame(left_panel, text="系统与模式控制", padding=(10, 6))
        sys_frame.pack(fill=tk.X, pady=(0, 6))
        
        btn_grid1 = ttk.Frame(sys_frame)
        btn_grid1.pack(fill=tk.X, padx=6, pady=4)
        ttk.Button(btn_grid1, text="启动 (!START)", command=lambda: self.send_cmd("!START")).grid(row=0, column=0, padx=4, pady=4)
        ttk.Button(btn_grid1, text="失能 (!DISABLE)", command=lambda: self.send_cmd("!DISABLE")).grid(row=0, column=1, padx=4, pady=4)
        ttk.Button(btn_grid1, text="急停 (!STOP)", command=lambda: self.send_cmd("!STOP")).grid(row=0, column=2, padx=4, pady=4)
        ttk.Button(btn_grid1, text="Home", command=self.send_home).grid(row=1, column=0, padx=4, pady=4)
        ttk.Label(btn_grid1, text="速度:").grid(row=1, column=1, padx=(8,2), sticky="e")
        self.ent_home_speed = ttk.Entry(btn_grid1, width=4)
        self.ent_home_speed.insert(0, str(self.HOME_SPEED))
        self.ent_home_speed.grid(row=1, column=2, padx=(0,4), sticky="w")
        ttk.Button(btn_grid1, text="Reset", command=self.send_reset).grid(row=1, column=3, padx=4, pady=4)
        ttk.Label(btn_grid1, text="速度:").grid(row=1, column=4, padx=(8,2), sticky="e")
        self.ent_rest_speed = ttk.Entry(btn_grid1, width=4)
        self.ent_rest_speed.insert(0, str(self.REST_SPEED))
        self.ent_rest_speed.grid(row=1, column=5, padx=(0,4), sticky="w")

        mode_frame = ttk.Frame(sys_frame)
        mode_frame.pack(fill=tk.X, padx=6, pady=4)
        ttk.Label(mode_frame, text="模式:").pack(side=tk.LEFT, padx=(0, 4))
        modes = [("1:顺序", 1), ("2:可打断", 2), ("3:连续", 3), ("5:力矩", 5), ("6:伺服", 6)]
        for text, val in modes:
            ttk.Button(mode_frame, text=text, width=7, command=lambda v=val: self.send_cmd(f"#CMDMODE {v}")).pack(side=tk.LEFT, padx=2)

        # 夹爪控制区
        hand_frame = ttk.LabelFrame(left_panel, text="夹爪控制", padding=(10, 6))
        hand_frame.pack(fill=tk.X, pady=(0, 6))
        
        h_grid = ttk.Frame(hand_frame)
        h_grid.pack(fill=tk.X, padx=6, pady=4)
        ttk.Button(h_grid, text="使能", width=7, command=lambda: self.send_cmd("!HAND_EN")).grid(row=0, column=0, padx=4, pady=4)
        ttk.Button(h_grid, text="失能", width=7, command=lambda: self.send_cmd("!HAND_DIS")).grid(row=0, column=1, padx=4, pady=4)
        ttk.Button(h_grid, text="张开", width=7, command=lambda: self.send_cmd("!HAND_O")).grid(row=0, column=2, padx=4, pady=4)
        ttk.Button(h_grid, text="闭合", width=7, command=lambda: self.send_cmd("!HAND_C")).grid(row=0, column=3, padx=4, pady=4)
        ttk.Label(h_grid, text="开度(0-100):").grid(row=1, column=0, columnspan=2, sticky=tk.W, padx=4, pady=4)
        self.ent_hand_pos = ttk.Entry(h_grid, width=6)
        self.ent_hand_pos.insert(0, "50")
        self.ent_hand_pos.grid(row=1, column=2, padx=4, pady=4)
        ttk.Button(h_grid, text="发送", width=5, command=self.send_hand_pos).grid(row=1, column=3, padx=4)

        # 查询区
        query_frame = ttk.LabelFrame(left_panel, text="状态查询", padding=(10, 6))
        query_frame.pack(fill=tk.X, pady=(0, 6))
        ttk.Button(query_frame, text="关节角", command=lambda: self.send_cmd("#GETJPOS")).pack(side=tk.LEFT, padx=4, pady=4)
        ttk.Button(query_frame, text="末端位姿", command=lambda: self.send_cmd("#GETLPOS")).pack(side=tk.LEFT, padx=4, pady=4)
        ttk.Button(query_frame, text="调试输出", command=lambda: self.send_cmd("!PRINTPOSE")).pack(side=tk.LEFT, padx=4, pady=4)

        # 关节零点 (Home Offset)
        home_frame = ttk.LabelFrame(left_panel, text="关节零点 (Home Offset)", padding=(10, 6))
        home_frame.pack(fill=tk.X, pady=(0, 6))
        ttk.Label(home_frame, text="将当前电机位置设为零点 (#OFFSET_J 1~6)").pack(anchor="w", padx=6, pady=(0, 4))
        home_btn_frame = ttk.Frame(home_frame)
        home_btn_frame.pack(fill=tk.X, padx=6, pady=4)
        for i in range(1, 7):
            ttk.Button(home_btn_frame, text=f"J{i}", width=4,
                       command=lambda j=i: self.send_cmd(f"#OFFSET_J {j}")).pack(side=tk.LEFT, padx=2, pady=2)
        ttk.Button(home_btn_frame, text="全部零点", width=8,
                   command=self.send_home_offset_all).pack(side=tk.LEFT, padx=2, pady=2)

        # 左侧终端日志（指令行固定底部，日志区域在上方伸缩）
        terminal_frame = ttk.LabelFrame(left_panel, text="终端日志", padding=(8, 6))
        terminal_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 6))
        terminal_frame.columnconfigure(0, weight=1)
        terminal_frame.rowconfigure(0, weight=1)  # 日志区域占满剩余高度
        self.txt_log = scrolledtext.ScrolledText(terminal_frame, wrap=tk.WORD, width=38, state='normal', height=12)
        self.txt_log.grid(row=0, column=0, sticky="nsew", padx=6, pady=(6, 4))
        cmd_frame = ttk.Frame(terminal_frame)
        cmd_frame.grid(row=1, column=0, sticky="ew", padx=6, pady=(0, 6))
        cmd_frame.columnconfigure(1, weight=1)
        ttk.Label(cmd_frame, text="指令:").grid(row=0, column=0, padx=(0, 4), pady=2)
        self.ent_custom_cmd = ttk.Entry(cmd_frame)
        self.ent_custom_cmd.grid(row=0, column=1, sticky="ew", padx=4, pady=2)
        self.ent_custom_cmd.bind("<Return>", lambda e: self.send_custom_cmd())
        ttk.Button(cmd_frame, text="发送", command=self.send_custom_cmd).grid(row=0, column=2, padx=2, pady=2)
        ttk.Button(cmd_frame, text="清空", command=lambda: self.txt_log.delete(1.0, tk.END)).grid(row=0, column=3, padx=2, pady=2)

        # 右侧面板：运动控制（占满剩余空间）
        right_panel = ttk.Frame(main_frame)
        right_panel.grid(row=0, column=1, sticky="nsew", padx=(0, 0))

        # 全局加速度控制（只保留加速度，速度由各指令独立指定）
        global_frame = ttk.LabelFrame(right_panel, text="全局加速度", padding=(10, 6))
        global_frame.pack(fill=tk.X, pady=(0, 6))
        global_inner = ttk.Frame(global_frame)
        global_inner.pack(fill=tk.X, padx=6, pady=4)
        ttk.Label(global_inner, text="全局加速度:").pack(side=tk.LEFT, padx=(0, 4))
        self.ent_global_acc = ttk.Entry(global_inner, width=6)
        self.ent_global_acc.insert(0, "500")
        self.ent_global_acc.pack(side=tk.LEFT, padx=(0, 12))
        ttk.Button(global_inner, text="应用", command=self.apply_global_acc).pack(side=tk.LEFT, padx=4)

        # RGB 控制区
        rgb_frame = ttk.LabelFrame(right_panel, text="RGB 控制", padding=(10, 6))
        rgb_frame.pack(fill=tk.X, pady=(0, 6))
        rgb_grid = ttk.Frame(rgb_frame)
        rgb_grid.pack(fill=tk.X, padx=6, pady=4)
        ttk.Button(rgb_grid, text="开灯", width=8, command=lambda: self.send_cmd("!RGB_EN")).grid(row=0, column=0, padx=4, pady=4)
        ttk.Button(rgb_grid, text="关灯", width=8, command=lambda: self.send_cmd("!RGB_DIS")).grid(row=0, column=1, padx=4, pady=4)
        ttk.Button(rgb_grid, text="[0]单色0", command=lambda: self.send_cmd("!RGB_MODE 0")).grid(row=0, column=2, padx=4, pady=4)
        ttk.Button(rgb_grid, text="[1]单色1", command=lambda: self.send_cmd("!RGB_MODE 1")).grid(row=0, column=3, padx=4, pady=4)
        ttk.Button(rgb_grid, text="[2]单色2", command=lambda: self.send_cmd("!RGB_MODE 2")).grid(row=0, column=4, padx=4, pady=4)
        ttk.Button(rgb_grid, text="[3]彩虹", command=lambda: self.send_cmd("!RGB_MODE 3")).grid(row=0, column=5, padx=4, pady=4)
        ttk.Button(rgb_grid, text="[4]蓝潮", command=lambda: self.send_cmd("!RGB_MODE 4")).grid(row=1, column=0, padx=4, pady=4)
        ttk.Button(rgb_grid, text="[5]白呼吸", command=lambda: self.send_cmd("!RGB_MODE 5")).grid(row=1, column=1, padx=4, pady=4)
        ttk.Button(rgb_grid, text="[6]赛博", command=lambda: self.send_cmd("!RGB_MODE 6")).grid(row=1, column=2, padx=4, pady=4)
        ttk.Button(rgb_grid, text="[7]红心跳", command=lambda: self.send_cmd("!RGB_MODE 7")).grid(row=1, column=3, padx=4, pady=4)
        ttk.Button(rgb_grid, text="[8]绿旋转", command=lambda: self.send_cmd("!RGB_MODE 8")).grid(row=1, column=4, padx=4, pady=4)
        ttk.Button(rgb_grid, text="[9]闪烁", command=lambda: self.send_cmd("!RGB_MODE 9")).grid(row=1, column=5, padx=4, pady=4)

        color_frame = ttk.Frame(rgb_frame)
        color_frame.pack(fill=tk.X, padx=6, pady=4)
        ttk.Label(color_frame, text="设置单色(0/1/2):").pack(side=tk.LEFT, padx=6)
        self.cb_color_idx = ttk.Combobox(color_frame, width=2, values=["0", "1", "2"])
        self.cb_color_idx.current(0)
        self.cb_color_idx.pack(side=tk.LEFT, padx=6)
        ttk.Label(color_frame, text="R:").pack(side=tk.LEFT)
        self.ent_r = ttk.Entry(color_frame, width=4)
        self.ent_r.insert(0, "0")
        self.ent_r.pack(side=tk.LEFT, padx=6)
        ttk.Label(color_frame, text="G:").pack(side=tk.LEFT)
        self.ent_g = ttk.Entry(color_frame, width=4)
        self.ent_g.insert(0, "100")
        self.ent_g.pack(side=tk.LEFT, padx=6)
        ttk.Label(color_frame, text="B:").pack(side=tk.LEFT)
        self.ent_b = ttk.Entry(color_frame, width=4)
        self.ent_b.insert(0, "0")
        self.ent_b.pack(side=tk.LEFT, padx=6)
        ttk.Button(color_frame, text="发送颜色", command=self.send_rgb_color).pack(side=tk.LEFT, padx=10)

        state_frame = ttk.Frame(rgb_frame)
        state_frame.pack(fill=tk.X, padx=6, pady=4)
        ttk.Label(state_frame, text="状态绑定灯效:").pack(side=tk.LEFT, padx=6)
        self.cb_state_start = ttk.Combobox(state_frame, width=2, values=[str(i) for i in range(10)])
        self.cb_state_start.current(0)
        ttk.Button(state_frame, text="设为开机", command=lambda: self.send_cmd(f"!RGB_SET_START {self.cb_state_start.get()}")).pack(side=tk.LEFT, padx=1)
        self.cb_state_start.pack(side=tk.LEFT, padx=6)
        self.cb_state_enable = ttk.Combobox(state_frame, width=2, values=[str(i) for i in range(10)])
        self.cb_state_enable.current(1)
        ttk.Button(state_frame, text="设为使能", command=lambda: self.send_cmd(f"!RGB_SET_ENABLE {self.cb_state_enable.get()}")).pack(side=tk.LEFT, padx=1)
        self.cb_state_enable.pack(side=tk.LEFT, padx=6)
        self.cb_state_disable = ttk.Combobox(state_frame, width=2, values=[str(i) for i in range(10)])
        self.cb_state_disable.current(2)
        ttk.Button(state_frame, text="设为失能", command=lambda: self.send_cmd(f"!RGB_SET_DISABLE {self.cb_state_disable.get()}")).pack(side=tk.LEFT, padx=1)
        self.cb_state_disable.pack(side=tk.LEFT, padx=6)

        # 关节运动（初始角度: 0, -75, 180, 0, 0, 0）
        movej_frame = ttk.LabelFrame(right_panel, text="关节控制 (MoveJ)", padding=(10, 6))
        movej_frame.pack(fill=tk.X, pady=(0, 6))
        self.ent_joints = []
        movej_defaults = [0, -75, 180, 0, 0, 0]
        for i in range(6):
            row = 0
            col_offset = i * 2
            ttk.Label(movej_frame, text=f"J{i+1}:").grid(row=row, column=col_offset, padx=10, pady=10, sticky="e")
            ent = ttk.Entry(movej_frame, width=8)
            ent.insert(0, str(movej_defaults[i]))
            ent.grid(row=row, column=col_offset+1, padx=10, pady=10, sticky="w")
            self.ent_joints.append(ent)
            
        ctrl_j_frame = ttk.Frame(movej_frame)
        ctrl_j_frame.grid(row=1, column=0, columnspan=12, pady=10, sticky="ew")
        ttk.Label(ctrl_j_frame, text="Speed:").pack(side=tk.LEFT, padx=10)
        self.ent_j_speed = ttk.Entry(ctrl_j_frame, width=6)
        self.ent_j_speed.insert(0, "10")
        self.ent_j_speed.pack(side=tk.LEFT, padx=10)
        ttk.Button(ctrl_j_frame, text="发送 MoveJ", command=self.send_movej).pack(side=tk.LEFT, padx=15)

        # 笛卡尔运动
        movel_frame = ttk.LabelFrame(right_panel, text="笛卡尔控制 (MoveL)", padding=(10, 6))
        movel_frame.pack(fill=tk.X, pady=(0, 6))
        self.ent_pose = []
        self.scl_pose = []
        labels = ['X', 'Y', 'Z', 'R', 'P', 'Yw']
        defaults = ['160', '0', '150', '0', '180', '0']
        ranges = [(50, 300), (-200, 200), (-50, 300), (-180, 180), (0, 360), (-180, 180)]
        
        self.movel_drag_enable = tk.BooleanVar(value=False)
        self.last_movel_send_time = 0
        
        for i, (lbl, df, rng) in enumerate(zip(labels, defaults, ranges)):
            row = i // 3
            col_offset = (i % 3) * 3
            ttk.Label(movel_frame, text=f"{lbl}:").grid(row=row, column=col_offset, padx=10, pady=10, sticky="e")
            
            ent = ttk.Entry(movel_frame, width=8)
            ent.insert(0, df)
            ent.grid(row=row, column=col_offset+1, padx=10, pady=10)
            self.ent_pose.append(ent)
            
            scl = ttk.Scale(movel_frame, from_=rng[0], to=rng[1], orient=tk.HORIZONTAL, length=120)
            scl.set(float(df))
            scl.grid(row=row, column=col_offset+2, padx=10, pady=10)
            self.scl_pose.append(scl)
            
            def update_entry_and_send(val, idx=i):
                current_val = f"{float(val):.1f}"
                if self.ent_pose[idx].get() != current_val:
                    self.ent_pose[idx].delete(0, tk.END)
                    self.ent_pose[idx].insert(0, current_val)
                if self.movel_drag_enable.get():
                    now = time.time()
                    if now - self.last_movel_send_time > 0.05:
                        self.last_movel_send_time = now
                        self.root.after(1, self.send_movel)
                        
            scl.config(command=update_entry_and_send)
            
            def update_scl_from_entry(event, idx=i):
                try:
                    val = float(self.ent_pose[idx].get())
                    self.scl_pose[idx].set(val)
                except ValueError:
                    pass
            ent.bind("<Return>", update_scl_from_entry)
            ent.bind("<FocusOut>", update_scl_from_entry)

        ctrl_frame = ttk.Frame(movel_frame)
        ctrl_frame.grid(row=3, column=0, columnspan=6, pady=10, sticky="ew")
        
        ttk.Label(ctrl_frame, text="Speed:").pack(side=tk.LEFT, padx=6)
        self.ent_l_speed = ttk.Entry(ctrl_frame, width=6)
        self.ent_l_speed.insert(0, "10")
        self.ent_l_speed.pack(side=tk.LEFT, padx=6)
        ttk.Button(ctrl_frame, text="发送 MoveL", command=self.send_movel).pack(side=tk.LEFT, padx=10)
        ttk.Checkbutton(ctrl_frame, text="开启拖动实时发送 (建议在连续模式下使用)", variable=self.movel_drag_enable).pack(side=tk.LEFT, padx=10)

        # 电流力矩控制区
        torque_frame_main = ttk.LabelFrame(right_panel, text="电流力矩控制 ($)", padding=(10, 6))
        torque_frame_main.pack(fill=tk.X, pady=(0, 6))
        torque_grid = ttk.Frame(torque_frame_main)
        torque_grid.pack(fill=tk.X, padx=6, pady=4)
        ttk.Label(torque_grid, text="力矩指令 ($c1,c2,c3,c4,c5,c6):").grid(row=0, column=0, columnspan=12, sticky="w", pady=6)
        
        self.ent_torques = []
        for i in range(6):
            ttk.Label(torque_grid, text=f"J{i+1}(A):").grid(row=1, column=i*2, padx=6, sticky="e")
            ent = ttk.Entry(torque_grid, width=5)
            ent.insert(0, "0.0")
            ent.grid(row=1, column=i*2+1, padx=6, sticky="w")
            self.ent_torques.append(ent)
            
        ttk.Button(torque_grid, text="发送力矩 ($)", command=self.send_torque).grid(row=1, column=12, padx=10)

        # ServoJ 测试区
        servoj_frame = ttk.LabelFrame(right_panel, text="ServoJ 测试 (模式6)", padding=(10, 6))
        servoj_frame.pack(fill=tk.X, pady=(0, 6))
        ttk.Label(servoj_frame, text="正弦波轨迹测试通信与响应").pack(anchor="w", padx=6, pady=2)
        self.btn_servoj_start = ttk.Button(servoj_frame, text="开始 ServoJ 正弦波", command=self.toggle_servoj_test)
        self.btn_servoj_start.pack(anchor="w", padx=6, pady=6)
        self.is_servoj_testing = False
        self.servoj_thread = None

    def refresh_ports(self):
        ports = serial.tools.list_ports.comports()
        self.cb_ports['values'] = [p.device for p in ports]
        if ports:
            self.cb_ports.current(0)

    def log(self, msg, tag="INFO"):
        time_str = time.strftime("%H:%M:%S")
        self.txt_log.insert(tk.END, f"[{time_str}] [{tag}] {msg}\n")
        self.txt_log.see(tk.END)

    def toggle_connection(self):
        if not self.is_connected:
            port = self.cb_ports.get()
            baud = self.cb_baudrate.get()
            try:
                self.serial_port = serial.Serial(port, int(baud), timeout=0.1)
                self.is_connected = True
                self.btn_connect.config(text="断开连接")
                self.lbl_status.config(text=f"已连接: {port}", foreground="green")
                self.log(f"成功连接串口 {port} @ {baud}")
                
                # 开启接收线程
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
            self.btn_connect.config(text="连接串口")
            self.lbl_status.config(text="未连接", foreground="red")
            self.log("已断开串口")

    def serial_receive_loop(self):
        while not self.stop_thread and self.serial_port and self.serial_port.is_open:
            try:
                if self.serial_port.in_waiting:
                    data = self.serial_port.readline()
                    if data:
                        text = data.decode('utf-8', errors='replace').strip()
                        if text:
                            # 通过 after 将日志更新放入主线程
                            self.root.after(0, self.log, text, "RX")
            except Exception as e:
                self.root.after(0, self.log, f"读取异常: {e}", "ERROR")
                break
            time.sleep(0.01)

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

    def send_home_offset_all(self):
        """依次发送 #OFFSET_J 1~6，将 6 个关节当前电机位置均设为零点"""
        if not self.is_connected or not self.serial_port:
            self.log("未连接串口", "WARN")
            return
        for j in range(1, 7):
            self.send_cmd(f"#OFFSET_J {j}")
            time.sleep(0.15)
        self.log("已发送 #OFFSET_J 1~6（全部关节设为零点）", "INFO")

    def apply_global_acc(self):
        """向固件发送 #ACC，更新全局加速度参数"""
        if not self.is_connected or not self.serial_port:
            self.log("未连接串口", "WARN")
            return
        try:
            acc = float(self.ent_global_acc.get())
            if 0 <= acc <= 5000:
                self.send_cmd(f"#ACC {acc}")
                self.log(f"已发送 #ACC {acc}", "INFO")
            else:
                messagebox.showerror("错误", "加速度范围 0~5000")
        except ValueError:
            messagebox.showerror("错误", "请输入有效的数字")

    def send_home(self):
        """发送自定义回零 MoveJ，位姿由类变量 HOME_POSE 定义"""
        try:
            speed = float(self.ent_home_speed.get())
            speed = max(1, min(500, speed))  # 限制范围 1~500
        except ValueError:
            speed = self.HOME_SPEED
        cmd = ">" + ",".join(str(v) for v in self.HOME_POSE) + f",{speed}"
        self.send_cmd(cmd)
        self.log(f"已发送 Home 位姿 {self.HOME_POSE} @ 速度 {speed}", "INFO")

    def send_reset(self):
        """发送自定义复位 MoveJ，位姿由类变量 REST_POSE 定义"""
        try:
            speed = float(self.ent_rest_speed.get())
            speed = max(1, min(500, speed))  # 限制范围 1~500
        except ValueError:
            speed = self.REST_SPEED
        cmd = ">" + ",".join(str(v) for v in self.REST_POSE) + f",{speed}"
        self.send_cmd(cmd)
        self.log(f"已发送 Rest 位姿 {self.REST_POSE} @ 速度 {speed}", "INFO")

    def send_custom_cmd(self):
        cmd = self.ent_custom_cmd.get().strip()
        if cmd:
            self.send_cmd(cmd)
            self.ent_custom_cmd.delete(0, tk.END)

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

    def send_hand_pos(self):
        """发送夹爪位置，带输入验证"""
        try:
            pos = int(self.ent_hand_pos.get())
            if 0 <= pos <= 100:
                self.send_cmd(f"!HAND_POS {pos}")
            else:
                messagebox.showerror("错误", "夹爪开度必须在 0~100 之间")
        except ValueError:
            messagebox.showerror("错误", "请输入有效的整数")

    def send_movej(self):
        try:
            joints = [float(ent.get()) for ent in self.ent_joints]
            speed = float(self.ent_j_speed.get())
            cmd = f">{joints[0]},{joints[1]},{joints[2]},{joints[3]},{joints[4]},{joints[5]},{speed}"
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
            torques = [float(ent.get()) for ent in self.ent_torques]
            cmd = f"${torques[0]:.2f},{torques[1]:.2f},{torques[2]:.2f},{torques[3]:.2f},{torques[4]:.2f},{torques[5]:.2f}"
            self.send_cmd(cmd)
        except ValueError:
            messagebox.showerror("错误", "请输入有效的数字")

    def toggle_servoj_test(self):
        if not self.is_connected:
            messagebox.showwarning("提示", "请先连接串口")
            return
            
        if self.is_servoj_testing:
            self.is_servoj_testing = False
            self.btn_servoj_start.config(text="开始正弦波发送")
            self.log("已停止 ServoJ 测试")
        else:
            # 自动切换到模式6并使能
            self.send_cmd("!START")
            self.send_cmd("#CMDMODE 6")
            self.is_servoj_testing = True
            self.btn_servoj_start.config(text="停止发送")
            self.log("开始 ServoJ 测试 (50Hz直线插补正弦波，J1为基础轴)")
            self.servoj_thread = threading.Thread(target=self.servoj_test_loop, daemon=True)
            self.servoj_thread.start()

    def servoj_test_loop(self):
        start_time = time.time()
        # 基础关节角：[0, -75, 180, 0, 0, 0]
        base_joints = [0, -75, 180, 0, 0, 0]
        
        # 简单直线插补补偿算法所需的变量
        # 模拟由于重力或死区导致的理论位置与实际反馈的误差漂移
        # 实际使用中这里应该接入你的真实传感器数据反馈，这里仅做基于模型的软补偿演示
        kp = 0.5  # 位置纠偏比例系数
        current_joint_cache = base_joints.copy()
        
        while self.is_servoj_testing and self.is_connected:
            t = time.time() - start_time
            # 生成目标轨迹：J1 幅度 20度，频率 0.5Hz 正弦波
            target_j1 = base_joints[0] + 20 * math.sin(2 * math.pi * 0.5 * t)
            
            # 【初步直线补偿算法演示】
            # 在没有外部位置反馈的情况下，我们用简单的 P 环平滑生成下发指令
            # 把速度/加速度规划留给下位机的动态速度分配，这里做上位机轨迹插补
            error_j1 = target_j1 - current_joint_cache[0]
            current_joint_cache[0] += error_j1 * kp
            
            # 构造高频透传指令（不需要speed后缀）
            cmd = f">{current_joint_cache[0]:.2f},{base_joints[1]},{base_joints[2]},{base_joints[3]},{base_joints[4]},{base_joints[5]}\n"
            try:
                self.serial_port.write(cmd.encode('utf-8'))
                # 为避免日志刷屏，每20次打印一次
                if int(t * 50) % 20 == 0:
                    self.root.after(0, self.log, f"Target: {target_j1:.1f}, Send: {current_joint_cache[0]:.1f}", "TX_SERVOJ")
            except Exception as e:
                self.root.after(0, self.log, f"发送失败: {e}", "ERROR")
                break
                
            time.sleep(0.02) # 50Hz

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotSerialAssistant(root)
    root.mainloop()
