import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import serial
import serial.tools.list_ports
import threading
import time
import math
#你好
class RobotSerialAssistant:
    def __init__(self, root):
        self.root = root
        self.root.title("Dummy Robot 串口助手")
        self.root.geometry("1400x1150")
        self.root.minsize(1200, 800)
        
        # 尝试使用更现代的 ttk 主题
        try:
            style = ttk.Style()
            if 'clam' in style.theme_names():
                style.theme_use('clam')
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
        
        self.create_widgets()
        self.refresh_ports()
        
        self.receive_thread = None
        self.stop_thread = False

    def create_widgets(self):
        # ========== 顶部：连接控制 ==========
        top_frame = ttk.Frame(self.root)
        top_frame.pack(fill=tk.X, padx=10, pady=(8, 4))
        
        conn_frame = ttk.LabelFrame(top_frame, text="通信设置", padding=5)
        conn_frame.pack(fill=tk.X)
        
        ttk.Label(conn_frame, text="端口:", font=("Arial", 10)).pack(side=tk.LEFT, padx=(5, 5))
        self.cb_ports = ttk.Combobox(conn_frame, width=15, font=("Arial", 10))
        self.cb_ports.pack(side=tk.LEFT, padx=(0, 10))
        ttk.Button(conn_frame, text="刷新", command=self.refresh_ports, width=6).pack(side=tk.LEFT, padx=(0, 20))
        
        ttk.Label(conn_frame, text="波特率:", font=("Arial", 10)).pack(side=tk.LEFT, padx=(0, 5))
        self.cb_baudrate = ttk.Combobox(conn_frame, width=12, font=("Arial", 10), values=["9600", "115200", "1000000"])
        self.cb_baudrate.current(1)
        self.cb_baudrate.pack(side=tk.LEFT, padx=(0, 20))
        
        self.btn_connect = ttk.Button(conn_frame, text="连接串口", command=self.toggle_connection, width=12)
        self.btn_connect.pack(side=tk.LEFT, padx=(0, 15))
        
        self.lbl_status = ttk.Label(conn_frame, text="● 未连接", foreground="red", font=("Arial", 10, "bold"))
        self.lbl_status.pack(side=tk.LEFT, padx=(10, 0))

        # ========== 引入 PanedWindow 实现上下分栏 ==========
        main_paned = ttk.PanedWindow(self.root, orient=tk.VERTICAL)
        main_paned.pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 8))

        # --- 上半部分：控制区 ---
        controls_frame = ttk.Frame(main_paned)
        main_paned.add(controls_frame, weight=3) 

        controls_frame.columnconfigure(0, weight=1)  # 左栏：系统控制
        controls_frame.columnconfigure(1, weight=1)  # 中栏：配置
        controls_frame.columnconfigure(2, weight=2)  # 右栏：运动控制
        controls_frame.rowconfigure(0, weight=1)

        # ==================== 左栏：系统控制 ====================
        left_frame = ttk.Frame(controls_frame)
        left_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 5))
        
        # 1. 系统控制
        sys_frame = ttk.LabelFrame(left_frame, text="系统控制", padding=6)
        sys_frame.pack(fill=tk.X, pady=(0, 6))
        
        sys_grid = ttk.Frame(sys_frame)
        sys_grid.pack(fill=tk.X)
        sys_grid.columnconfigure((0, 1, 2), weight=1)
        
        ttk.Button(sys_grid, text="启动 !START", command=lambda: self.send_cmd("!START")).grid(row=0, column=0, padx=2, pady=2, sticky="ew")
        ttk.Button(sys_grid, text="失能 !DISABLE", command=lambda: self.send_cmd("!DISABLE")).grid(row=0, column=1, padx=2, pady=2, sticky="ew")
        ttk.Button(sys_grid, text="急停 !STOP", command=lambda: self.send_cmd("!STOP")).grid(row=0, column=2, padx=2, pady=2, sticky="ew")
        ttk.Button(sys_grid, text="回零 !HOME", command=lambda: self.send_cmd("!HOME")).grid(row=1, column=0, padx=2, pady=2, sticky="ew")
        ttk.Button(sys_grid, text="休息 !RESET", command=lambda: self.send_cmd("!RESET")).grid(row=1, column=1, padx=2, pady=2, sticky="ew")
        sys_grid.columnconfigure((0, 1, 2), weight=1)

        ttk.Separator(sys_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=6)

        ttk.Label(sys_frame, text="堵转检测:", font=("Arial", 9, "bold")).pack(anchor="w", pady=(0, 3))
        stall_inner = ttk.Frame(sys_frame)
        stall_inner.pack(fill=tk.X)
        ttk.Button(stall_inner, text="开启 !STALL_EN", command=lambda: self.send_cmd("!STALL_EN")).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 2))
        ttk.Button(stall_inner, text="关闭 !STALL_DIS", command=lambda: self.send_cmd("!STALL_DIS")).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(2, 0))

        ttk.Separator(sys_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=6)

        ttk.Label(sys_frame, text="模式切换:", font=("Arial", 9, "bold")).pack(anchor="w", pady=(0, 3))
        mode_inner = ttk.Frame(sys_frame)
        mode_inner.pack(fill=tk.X)
        modes = [("1:顺序", 1), ("2:打断", 2), ("3:连续", 3), ("5:力矩", 5), ("6:伺服", 6)]
        for i, (text, val) in enumerate(modes):
            ttk.Button(mode_inner, text=text, width=7, command=lambda v=val: self.send_cmd(f"#CMDMODE {v}")).grid(row=0, column=i, padx=2, pady=2)

        # 2. 查询与校准
        query_frame = ttk.LabelFrame(left_frame, text="查询与校准", padding=6)
        query_frame.pack(fill=tk.X, pady=(0, 6))
        
        query_btn_frame = ttk.Frame(query_frame)
        query_btn_frame.pack(fill=tk.X)
        ttk.Button(query_btn_frame, text="获取关节角", command=lambda: self.send_cmd("#GETJPOS")).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        ttk.Button(query_btn_frame, text="获取位姿", command=lambda: self.send_cmd("#GETLPOS")).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        ttk.Button(query_btn_frame, text="调试输出", command=lambda: self.send_cmd("!PRINTPOSE")).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        
        ttk.Separator(query_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=6)
        
        # 标定按钮行
        calib_frame = ttk.Frame(query_frame)
        calib_frame.pack(fill=tk.X)
        ttk.Button(calib_frame, text="标定零点 !CALIBRATION", command=self.send_calibration, 
                   style="Accent.TButton").pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        
        ttk.Separator(query_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=6)
        ttk.Label(query_frame, text="关节置零 (Home Offset):", font=("Arial", 9, "bold")).pack(anchor="w", pady=(0, 3))
        
        offset_inner = ttk.Frame(query_frame)
        offset_inner.pack(fill=tk.X)
        for i in range(1, 7):
            ttk.Button(offset_inner, text=f"J{i}", width=4, command=lambda j=i: self.send_cmd(f"#OFFSET_J {j}")).grid(row=0, column=i-1, padx=1, pady=2)
        ttk.Button(offset_inner, text="全部置零", command=self.send_home_offset_all).grid(row=0, column=6, padx=(5, 0), pady=2, sticky="ew")

        # 3. 夹爪控制 (增加 expand=True, fill=tk.BOTH 保证底部对齐)
        hand_frame = ttk.LabelFrame(left_frame, text="夹爪控制", padding=6)
        hand_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 6)) 
        
        hand_top = ttk.Frame(hand_frame)
        hand_top.pack(fill=tk.X, pady=(0, 6))
        ttk.Button(hand_top, text="使能", command=lambda: self.send_cmd("!HAND_EN")).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        ttk.Button(hand_top, text="失能", command=lambda: self.send_cmd("!HAND_DIS")).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        ttk.Button(hand_top, text="张开", command=lambda: self.send_cmd("!HAND_O")).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        ttk.Button(hand_top, text="闭合", command=lambda: self.send_cmd("!HAND_C")).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        
        hand_calib_frame = ttk.Frame(hand_frame)
        hand_calib_frame.pack(fill=tk.X, pady=(0, 6))
        ttk.Button(hand_calib_frame, text="标定 !HAND_ZERO", command=self.send_hand_zero).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        hand_current_frame = ttk.Frame(hand_frame)
        hand_current_frame.pack(fill=tk.X, pady=(0, 6))
        ttk.Label(hand_current_frame, text="力矩(A):", font=("Arial", 9)).pack(side=tk.LEFT)
        self.ent_hand_current = ttk.Entry(hand_current_frame, width=5, font=("Arial", 9))
        self.ent_hand_current.insert(0, "1.2")
        self.ent_hand_current.pack(side=tk.LEFT, padx=(5, 2))
        self.scl_hand_current = ttk.Scale(hand_current_frame, from_=0.05, to=2.0, orient=tk.HORIZONTAL)
        self.scl_hand_current.set(1.2)
        self.scl_hand_current.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        self.lbl_hand_current_val = ttk.Label(hand_current_frame, text="1.2", width=4, font=("Arial", 9))
        self.lbl_hand_current_val.pack(side=tk.LEFT, padx=2)
        ttk.Button(hand_current_frame, text="发送", width=5, command=self.send_hand_current).pack(side=tk.LEFT, padx=(5, 0))

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

        hand_slider = ttk.Frame(hand_frame)
        hand_slider.pack(fill=tk.X)
        ttk.Label(hand_slider, text="开度:", font=("Arial", 9)).pack(side=tk.LEFT)
        self.ent_hand_pos = ttk.Entry(hand_slider, width=5, font=("Arial", 9))
        self.ent_hand_pos.insert(0, "50")
        self.ent_hand_pos.pack(side=tk.LEFT, padx=(5, 5))
        
        self.scl_hand = ttk.Scale(hand_slider, from_=0, to=100, orient=tk.HORIZONTAL)
        self.scl_hand.set(50)
        self.scl_hand.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        
        self.lbl_hand_val = ttk.Label(hand_slider, text="50", width=3, font=("Arial", 9))
        self.lbl_hand_val.pack(side=tk.LEFT, padx=5)
        
        self.hand_drag_enable = tk.BooleanVar(value=False)
        ttk.Checkbutton(hand_slider, text="拖发", variable=self.hand_drag_enable).pack(side=tk.LEFT, padx=5)
        ttk.Button(hand_slider, text="发送", width=6, command=self.send_hand_pos).pack(side=tk.LEFT)
        
        def update_hand_from_scale(val):
            v = int(float(val))
            self.lbl_hand_val.config(text=str(v))
            if self.ent_hand_pos.get() != str(v):
                self.ent_hand_pos.delete(0, tk.END)
                self.ent_hand_pos.insert(0, str(v))
            if self.hand_drag_enable.get():
                self.send_cmd(f"!HAND_POS {v}")
        self.scl_hand.config(command=update_hand_from_scale)
        
        def update_hand_from_entry(event):
            try:
                v = int(self.ent_hand_pos.get())
                self.scl_hand.set(v)
            except ValueError:
                pass
        self.ent_hand_pos.bind("<Return>", update_hand_from_entry)
        self.ent_hand_pos.bind("<FocusOut>", update_hand_from_entry)

        # ==================== 中栏：配置 ====================
        middle_frame = ttk.Frame(controls_frame)
        middle_frame.grid(row=0, column=1, sticky="nsew", padx=5)
        
        # 1. RGB控制
        rgb_frame = ttk.LabelFrame(middle_frame, text="RGB 彩灯控制", padding=6)
        rgb_frame.pack(fill=tk.X, pady=(0, 6))
        
        rgb_top = ttk.Frame(rgb_frame)
        rgb_top.pack(fill=tk.X, pady=(0, 6))
        ttk.Button(rgb_top, text="开灯", command=lambda: self.send_cmd("!RGB_EN")).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        ttk.Button(rgb_top, text="关灯", command=lambda: self.send_cmd("!RGB_DIS")).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        
        rgb_modes_grid = ttk.Frame(rgb_frame)
        rgb_modes_grid.pack(fill=tk.X)
        rgb_modes_grid.columnconfigure((0,1,2,3,4), weight=1)
        rgb_modes1 = [("单色0", 0), ("单色1", 1), ("单色2", 2), ("流光", 3), ("潮汐", 4)]
        rgb_modes2 = [("白色", 5), ("赛博", 6), ("心跳", 7), ("旋转", 8), ("闪烁", 9)]
        for i, (text, val) in enumerate(rgb_modes1):
            ttk.Button(rgb_modes_grid, text=f"{val}:{text}", command=lambda v=val: self.send_cmd(f"!RGB_MODE {v}")).grid(row=0, column=i, padx=2, pady=2, sticky="ew")
        for i, (text, val) in enumerate(rgb_modes2):
            ttk.Button(rgb_modes_grid, text=f"{val}:{text}", command=lambda v=val: self.send_cmd(f"!RGB_MODE {v}")).grid(row=1, column=i, padx=2, pady=2, sticky="ew")
        
        ttk.Separator(rgb_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=6)
        
        rgb_color = ttk.Frame(rgb_frame)
        rgb_color.pack(fill=tk.X, pady=(0, 3))
        ttk.Label(rgb_color, text="自定义颜色(Idx/R/G/B):").pack(side=tk.LEFT, padx=(0, 5))
        self.cb_color_idx = ttk.Combobox(rgb_color, width=2, values=["0", "1", "2"], state="readonly")
        self.cb_color_idx.current(0)
        self.cb_color_idx.pack(side=tk.LEFT, padx=(0, 5))
        
        for lbl, var in [("R:", "ent_r"), ("G:", "ent_g"), ("B:", "ent_b")]:
            ttk.Label(rgb_color, text=lbl).pack(side=tk.LEFT)
            ent = ttk.Entry(rgb_color, width=4)
            ent.insert(0, "0" if lbl != "G:" else "100")
            ent.pack(side=tk.LEFT, padx=2)
            setattr(self, var, ent)
        ttk.Button(rgb_color, text="发送颜色", command=self.send_rgb_color).pack(side=tk.RIGHT)
        
        ttk.Separator(rgb_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=6)
        
        ttk.Separator(rgb_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=6)
        
        rgb_state = ttk.Frame(rgb_frame)
        rgb_state.pack(fill=tk.X)
        ttk.Label(rgb_state, text="状态绑定:").pack(side=tk.LEFT, padx=(0, 5))
        
        self.cb_state_start = ttk.Combobox(rgb_state, width=2, values=[str(i) for i in range(10)], state="readonly")
        self.cb_state_start.current(0)
        ttk.Label(rgb_state, text="开机:").pack(side=tk.LEFT)
        self.cb_state_start.pack(side=tk.LEFT, padx=2)
        ttk.Button(rgb_state, text="设", width=3, command=self.set_rgb_state_start).pack(side=tk.LEFT, padx=(0, 5))
        
        self.cb_state_enable = ttk.Combobox(rgb_state, width=2, values=[str(i) for i in range(10)], state="readonly")
        self.cb_state_enable.current(1)
        ttk.Label(rgb_state, text="使能:").pack(side=tk.LEFT)
        self.cb_state_enable.pack(side=tk.LEFT, padx=2)
        ttk.Button(rgb_state, text="设", width=3, command=self.set_rgb_state_enable).pack(side=tk.LEFT, padx=(0, 5))
        
        self.cb_state_disable = ttk.Combobox(rgb_state, width=2, values=[str(i) for i in range(10)], state="readonly")
        self.cb_state_disable.current(2)
        ttk.Label(rgb_state, text="失能:").pack(side=tk.LEFT)
        self.cb_state_disable.pack(side=tk.LEFT, padx=2)
        ttk.Button(rgb_state, text="设", width=3, command=self.set_rgb_state_disable).pack(side=tk.LEFT)

        # 2. 电机配置
        acc_frame = ttk.LabelFrame(middle_frame, text="电机参数配置", padding=6)
        acc_frame.pack(fill=tk.X, pady=(0, 6))
        
        acc_grid = ttk.Frame(acc_frame)
        acc_grid.pack(fill=tk.X)
        
        ttk.Label(acc_grid, text="节点(1-7):").grid(row=0, column=0, sticky="w", pady=3)
        self.cb_acc_node = ttk.Combobox(acc_grid, width=5, values=[str(i) for i in range(0, 8)], state="readonly")
        self.cb_acc_node.current(0)
        self.cb_acc_node.grid(row=0, column=1, padx=5, pady=3, sticky="w")

        ttk.Label(acc_grid, text="基础加速(1-2000):").grid(row=1, column=0, sticky="w", pady=3)
        self.ent_acc_val = ttk.Entry(acc_grid, width=8)
        self.ent_acc_val.insert(0, "150")
        self.ent_acc_val.grid(row=1, column=1, padx=5, pady=3, sticky="w")
        ttk.Button(acc_grid, text="设置加速", command=self.send_acc_base).grid(row=1, column=2, padx=10, pady=3)

        ttk.Label(acc_grid, text="最大电流(A):").grid(row=2, column=0, sticky="w", pady=3)
        self.ent_i_limit = ttk.Entry(acc_grid, width=8)
        self.ent_i_limit.insert(0, "1.5")
        self.ent_i_limit.grid(row=2, column=1, padx=5, pady=3, sticky="w")
        ttk.Button(acc_grid, text="设置电流", command=self.send_i_limit).grid(row=2, column=2, padx=10, pady=3)

        ttk.Separator(acc_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=6)

        # 地轨速度设置
        rail_speed_frame = ttk.LabelFrame(acc_frame, text="地轨速度设置 (#SPEED_RAIL)", padding=6)
        rail_speed_frame.pack(fill=tk.X)

        rail_speed_inner = ttk.Frame(rail_speed_frame)
        rail_speed_inner.pack(fill=tk.X)

        ttk.Label(rail_speed_inner, text="速度(mm/s):", font=("Arial", 9)).pack(side=tk.LEFT)
        self.ent_rail_speed = ttk.Entry(rail_speed_inner, width=6, font=("Arial", 9))
        self.ent_rail_speed.insert(0, "50")
        self.ent_rail_speed.pack(side=tk.LEFT, padx=(5, 5))

        self.scl_rail_speed = ttk.Scale(rail_speed_inner, from_=0.5, to=100, orient=tk.HORIZONTAL)
        self.scl_rail_speed.set(50)
        self.scl_rail_speed.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

        self.lbl_rail_speed_val = ttk.Label(rail_speed_inner, text="50.0", width=6, font=("Arial", 9))
        self.lbl_rail_speed_val.pack(side=tk.LEFT, padx=5)

        ttk.Button(rail_speed_inner, text="查询", width=5,
                   command=self.query_rail_speed).pack(side=tk.LEFT, padx=2)
        ttk.Button(rail_speed_inner, text="应用", width=5,
                   command=self.apply_rail_speed).pack(side=tk.LEFT, padx=2)
        ttk.Button(rail_speed_inner, text="保存", width=5,
                   command=self.save_rail_speed).pack(side=tk.LEFT, padx=2)

        def update_rail_speed_from_scale(val):
            v = float(val)
            self.lbl_rail_speed_val.config(text=f"{v:.1f}")
            if self.ent_rail_speed.get() != f"{v:.1f}":
                self.ent_rail_speed.delete(0, tk.END)
                self.ent_rail_speed.insert(0, f"{v:.1f}")

        self.scl_rail_speed.config(command=update_rail_speed_from_scale)

        def update_rail_speed_from_entry(event):
            try:
                v = float(self.ent_rail_speed.get())
                if v < 0.5:
                    v = 0.5
                elif v > 100:
                    v = 100
                self.scl_rail_speed.set(v)
                self.lbl_rail_speed_val.config(text=f"{v:.1f}")
            except ValueError:
                pass

        self.ent_rail_speed.bind("<Return>", update_rail_speed_from_entry)
        self.ent_rail_speed.bind("<FocusOut>", update_rail_speed_from_entry)

        # 地轨加速度设置
        rail_acc_inner_frame = ttk.Frame(acc_frame)
        rail_acc_inner_frame.pack(fill=tk.X, pady=(6, 0))

        rail_acc_frame = ttk.LabelFrame(rail_acc_inner_frame, text="地轨加速度设置 (#ACC_RAIL)", padding=6)
        rail_acc_frame.pack(fill=tk.X)

        rail_acc_control = ttk.Frame(rail_acc_frame)
        rail_acc_control.pack(fill=tk.X)

        ttk.Label(rail_acc_control, text="加速度(mm/s2):", font=("Arial", 9)).pack(side=tk.LEFT)
        self.ent_rail_acc = ttk.Entry(rail_acc_control, width=7, font=("Arial", 9))
        self.ent_rail_acc.insert(0, "500")
        self.ent_rail_acc.pack(side=tk.LEFT, padx=(5, 5))

        self.scl_rail_acc = ttk.Scale(rail_acc_control, from_=10, to=5000, orient=tk.HORIZONTAL)
        self.scl_rail_acc.set(500)
        self.scl_rail_acc.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

        self.lbl_rail_acc_val = ttk.Label(rail_acc_control, text="500", width=6, font=("Arial", 9))
        self.lbl_rail_acc_val.pack(side=tk.LEFT, padx=5)

        ttk.Button(rail_acc_control, text="查询", width=5,
                   command=self.query_rail_acc).pack(side=tk.LEFT, padx=2)
        ttk.Button(rail_acc_control, text="应用", width=5,
                   command=self.apply_rail_acc).pack(side=tk.LEFT, padx=2)
        ttk.Button(rail_acc_control, text="保存", width=5,
                   command=self.save_rail_acc).pack(side=tk.LEFT, padx=2)

        def update_rail_acc_from_scale(val):
            v = float(val)
            self.lbl_rail_acc_val.config(text=f"{int(v)}")
            if self.ent_rail_acc.get() != str(int(v)):
                self.ent_rail_acc.delete(0, tk.END)
                self.ent_rail_acc.insert(0, str(int(v)))

        self.scl_rail_acc.config(command=update_rail_acc_from_scale)

        def update_rail_acc_from_entry(event):
            try:
                v = float(self.ent_rail_acc.get())
                if v < 10:
                    v = 10
                elif v > 5000:
                    v = 5000
                self.scl_rail_acc.set(v)
                self.lbl_rail_acc_val.config(text=f"{int(v)}")
            except ValueError:
                pass

        self.ent_rail_acc.bind("<Return>", update_rail_acc_from_entry)
        self.ent_rail_acc.bind("<FocusOut>", update_rail_acc_from_entry)

        # 地轨专用电流设置
        rail_curr_frame = ttk.Frame(acc_frame)
        rail_curr_frame.pack(fill=tk.X, pady=(6, 0))
        ttk.Label(rail_curr_frame, text="地轨电流:", font=("Arial", 9, "bold")).pack(side=tk.LEFT, padx=(0, 5))
        ttk.Button(rail_curr_frame, text="设置 2.8A", width=10,
                   command=lambda: self.set_rail_current(2.8)).pack(side=tk.LEFT, padx=2)
        ttk.Button(rail_curr_frame, text="设置 1.5A", width=10,
                   command=lambda: self.set_rail_current(1.5)).pack(side=tk.LEFT, padx=2)

        # 3. 力矩控制 (增加 expand=True, fill=tk.BOTH 保证底部对齐)
        torque_frame = ttk.LabelFrame(middle_frame, text="电流力矩控制 ($)", padding=6)
        torque_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 6))
        
        torque_grid = ttk.Frame(torque_frame)
        torque_grid.pack(fill=tk.X, pady=(0, 6))
        self.ent_torques = []
        for i in range(7):
            r, c = divmod(i, 4)
            lbl_text = "J0(地轨)" if i == 0 else f"J{i+1}"
            ttk.Label(torque_grid, text=f"{lbl_text}(A):").grid(row=r, column=c*2, padx=(5, 2), pady=3, sticky="e")
            ent = ttk.Entry(torque_grid, width=6)
            ent.insert(0, "0.0")
            ent.grid(row=r, column=c*2+1, padx=2, pady=3, sticky="w")
            self.ent_torques.append(ent)

        ttk.Button(torque_frame, text="发送全关节力矩指令", command=self.send_torque).pack(fill=tk.X)

        # ==================== 右栏：运动控制 ====================
        right_frame = ttk.Frame(controls_frame)
        right_frame.grid(row=0, column=2, sticky="nsew", padx=(5, 0))

        # 1. 关节运动 MoveJ
        movej_frame = ttk.LabelFrame(right_frame, text="关节控制 (MoveJ: >j0(地轨),j1~j6,speed)", padding=6)
        movej_frame.pack(fill=tk.X, pady=(0, 6))
        
        movej_grid = ttk.Frame(movej_frame)
        movej_grid.pack(fill=tk.X)
        movej_grid.columnconfigure(2, weight=1) 
        
        self.ent_joints = []
        self.scl_joints = []
        self.lbl_joints = []
        joint_ranges = [(-175, 175), (-75, 90), (0, 180), (-270, 270), (-100, 100), (-180, 180)]
        joint_defaults = [0, -75, 180, 0, 0, 0]
        
        self.movej_drag_enable = tk.BooleanVar(value=False)
        self.last_movej_send_time = 0
        
        for i in range(6):
            rng = joint_ranges[i]
            df = joint_defaults[i]
            
            ttk.Label(movej_grid, text=f"J{i+1}:", font=("Arial", 9, "bold")).grid(row=i, column=0, padx=5, pady=1)
            ent = ttk.Entry(movej_grid, width=6)
            ent.insert(0, str(df))
            ent.grid(row=i, column=1, padx=5, pady=1)
            self.ent_joints.append(ent)
            
            scl = ttk.Scale(movej_grid, from_=rng[0], to=rng[1], orient=tk.HORIZONTAL)
            scl.set(df)
            scl.grid(row=i, column=2, sticky="ew", padx=10, pady=1)
            self.scl_joints.append(scl)
            
            val_lbl = ttk.Label(movej_grid, text=f"{df}.0", width=6, anchor="e")
            val_lbl.grid(row=i, column=3, padx=5, pady=1)
            self.lbl_joints.append(val_lbl)
            
            def update_joint_entry(val, i=i, l=val_lbl, s=scl, e=ent):
                v = float(val)
                l.config(text=f"{v:.1f}")
                current_val = f"{v:.1f}"
                if e.get() != current_val:
                    e.delete(0, tk.END)
                    e.insert(0, current_val)
                if self.movej_drag_enable.get():
                    now = time.time()
                    if now - self.last_movej_send_time > 0.1:
                        self.last_movej_send_time = now
                        self.root.after(1, self.send_movej)
            scl.config(command=update_joint_entry)
            
            def update_joint_scl(event, i=i, s=scl):
                try:
                    v = float(self.ent_joints[i].get())
                    s.set(v)
                except ValueError:
                    pass
            ent.bind("<Return>", update_joint_scl)
            ent.bind("<FocusOut>", update_joint_scl)
        
        # 地轨滑块 (J0=地轨, 单位: mm, 范围 -250~250，支持手动归位)
        ttk.Label(movej_grid, text="地轨(J0):", font=("Arial", 9, "bold"), foreground="#007ACC").grid(row=6, column=0, padx=5, pady=1)
        self.ent_j7 = ttk.Entry(movej_grid, width=6)
        self.ent_j7.insert(0, "0")
        self.ent_j7.grid(row=6, column=1, padx=5, pady=1)

        self.scl_j7 = ttk.Scale(movej_grid, from_=-250, to=250, orient=tk.HORIZONTAL)
        self.scl_j7.set(0)
        self.scl_j7.grid(row=6, column=2, sticky="ew", padx=10, pady=1)

        self.lbl_j7 = ttk.Label(movej_grid, text="0.0 mm", width=8, anchor="e")
        self.lbl_j7.grid(row=6, column=3, padx=5, pady=1)
        
        def update_j7_entry(val):
            v = float(val)
            self.lbl_j7.config(text=f"{v:.1f} mm")
            current_val = f"{v:.1f}"
            if self.ent_j7.get() != current_val:
                self.ent_j7.delete(0, tk.END)
                self.ent_j7.insert(0, current_val)
            if self.movej_drag_enable.get():
                now = time.time()
                if now - self.last_movej_send_time > 0.1:
                    self.last_movej_send_time = now
                    self.root.after(1, self.send_movej)
        self.scl_j7.config(command=update_j7_entry)
        
        def update_j7_scl(event):
            try:
                v = float(self.ent_j7.get())
                self.scl_j7.set(v)
            except ValueError:
                pass
        self.ent_j7.bind("<Return>", update_j7_scl)
        self.ent_j7.bind("<FocusOut>", update_j7_scl)

        # 地轨手动左移右移按钮
        rail_ctrl = ttk.Frame(movej_frame)
        rail_ctrl.pack(fill=tk.X, pady=(4, 0))
        ttk.Label(rail_ctrl, text="地轨手动:", font=("Arial", 9, "bold")).pack(side=tk.LEFT, padx=(5, 5))

        self.ent_rail_step = ttk.Entry(rail_ctrl, width=6, font=("Arial", 9))
        self.ent_rail_step.insert(0, "10")
        self.ent_rail_step.pack(side=tk.LEFT, padx=(0, 3))
        ttk.Label(rail_ctrl, text="mm/次", font=("Arial", 9)).pack(side=tk.LEFT, padx=(0, 10))

        ttk.Button(rail_ctrl, text="← 左移", width=8,
                   command=self.rail_move_left).pack(side=tk.LEFT, padx=2)
        ttk.Button(rail_ctrl, text="右移 →", width=8,
                   command=self.rail_move_right).pack(side=tk.LEFT, padx=2)

        movej_ctrl = ttk.Frame(movej_frame)
        movej_ctrl.pack(fill=tk.X, pady=(5, 0))
        ttk.Label(movej_ctrl, text="Speed:").pack(side=tk.LEFT, padx=(5, 2))
        self.ent_j_speed = ttk.Entry(movej_ctrl, width=6)
        self.ent_j_speed.insert(0, "50")
        self.ent_j_speed.pack(side=tk.LEFT, padx=5)
        ttk.Checkbutton(movej_ctrl, text="拖动实时发送", variable=self.movej_drag_enable).pack(side=tk.LEFT, padx=10)
        ttk.Button(movej_ctrl, text="发送 MoveJ", command=self.send_movej).pack(side=tk.RIGHT, padx=5)

        # 2. 笛卡尔运动 MoveL
        movel_frame = ttk.LabelFrame(right_frame, text="笛卡尔控制 (MoveL: @x,y,z,r,p,yw,speed)", padding=6)
        movel_frame.pack(fill=tk.X, pady=(0, 6))
        
        movel_grid = ttk.Frame(movel_frame)
        movel_grid.pack(fill=tk.X)
        movel_grid.columnconfigure(2, weight=1)
        
        self.ent_pose = []
        self.scl_pose = []
        self.lbl_pose = []
        labels = ['X', 'Y', 'Z', 'R', 'P', 'Yw']
        defaults = [0, 0, 150, 0, 180, 0]
        ranges = [(-250, 250), (-200, 200), (-50, 450), (-180, 180), (0, 360), (-180, 180)]
        
        self.movel_drag_enable = tk.BooleanVar(value=False)
        self.last_movel_send_time = 0
        
        for i, (lbl, df, rng) in enumerate(zip(labels, defaults, ranges)):
            ttk.Label(movel_grid, text=f"{lbl}:", font=("Arial", 9, "bold")).grid(row=i, column=0, padx=5, pady=1)
            ent = ttk.Entry(movel_grid, width=6)
            ent.insert(0, str(df))
            ent.grid(row=i, column=1, padx=5, pady=1)
            self.ent_pose.append(ent)
            
            scl = ttk.Scale(movel_grid, from_=rng[0], to=rng[1], orient=tk.HORIZONTAL)
            scl.set(df)
            scl.grid(row=i, column=2, sticky="ew", padx=10, pady=1)
            self.scl_pose.append(scl)
            
            val_lbl = ttk.Label(movel_grid, text=f"{df}.0", width=6, anchor="e")
            val_lbl.grid(row=i, column=3, padx=5, pady=1)
            self.lbl_pose.append(val_lbl)
            
            def update_pose_entry(val, i=i, l=val_lbl, e=ent):
                v = float(val)
                l.config(text=f"{v:.1f}")
                current_val = f"{v:.1f}"
                if e.get() != current_val:
                    e.delete(0, tk.END)
                    e.insert(0, current_val)
                if self.movel_drag_enable.get():
                    now = time.time()
                    if now - self.last_movel_send_time > 0.05:
                        self.last_movel_send_time = now
                        self.root.after(1, self.send_movel)
            scl.config(command=update_pose_entry)
            
            def update_pose_scl(event, i=i, s=scl):
                try:
                    v = float(self.ent_pose[i].get())
                    scl.set(v)
                except ValueError:
                    pass
            ent.bind("<Return>", update_pose_scl)
            ent.bind("<FocusOut>", update_pose_scl)
        
        movel_ctrl = ttk.Frame(movel_frame)
        movel_ctrl.pack(fill=tk.X, pady=(5, 0))
        ttk.Label(movel_ctrl, text="Speed:").pack(side=tk.LEFT, padx=(5, 2))
        self.ent_l_speed = ttk.Entry(movel_ctrl, width=6)
        self.ent_l_speed.insert(0, "50")
        self.ent_l_speed.pack(side=tk.LEFT, padx=5)
        ttk.Checkbutton(movel_ctrl, text="拖动实时发送", variable=self.movel_drag_enable).pack(side=tk.LEFT, padx=10)
        ttk.Button(movel_ctrl, text="发送 MoveL", command=self.send_movel).pack(side=tk.RIGHT, padx=5)

        # 3. ServoJ 测试 (增加 expand=True, fill=tk.BOTH 保证底部对齐)
        servoj_frame = ttk.LabelFrame(right_frame, text="ServoJ 连续通信测试", padding=6)
        servoj_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 6))
        
        ttk.Label(servoj_frame, text="* 需切换到模式6 (高频伺服)", foreground="gray").pack(anchor="w", pady=(0, 3))
        
        servoj_info = ttk.Frame(servoj_frame)
        servoj_info.pack(fill=tk.X, pady=(0, 6))
        ttk.Label(servoj_info, text="基准位姿: [0, -75, 180, 0, 0, 0]").pack(side=tk.LEFT, padx=(0, 10))
        ttk.Label(servoj_info, text="激励: J1 ±20° 正弦, 0.5Hz").pack(side=tk.LEFT)
        
        self.btn_servoj_start = ttk.Button(servoj_frame, text="▶ 开始发送正弦轨迹", command=self.toggle_servoj_test)
        self.btn_servoj_start.pack(fill=tk.X)
        
        self.is_servoj_testing = False
        self.servoj_thread = None

        # --- 下半部分：终端日志区 ---
        log_frame = ttk.LabelFrame(main_paned, text="终端日志与自定义指令", padding=8)
        main_paned.add(log_frame, weight=1) 

        # ！！核心修复点：优先将发送指令栏 pack 在底部 (side=tk.BOTTOM)，这样它就永远不会被日志框挤掉！！
        cmd_frame = ttk.Frame(log_frame)
        cmd_frame.pack(side=tk.BOTTOM, fill=tk.X, pady=(5, 0))
        
        ttk.Label(cmd_frame, text="快捷指令:", font=("Arial", 10, "bold")).pack(side=tk.LEFT, padx=(0, 10))
        self.ent_custom_cmd = ttk.Entry(cmd_frame, font=("Consolas", 10))
        self.ent_custom_cmd.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 10))
        self.ent_custom_cmd.bind("<Return>", lambda e: self.send_custom_cmd())
        ttk.Button(cmd_frame, text="发送 (Enter)", command=self.send_custom_cmd, width=12).pack(side=tk.LEFT, padx=(0, 5))
        ttk.Button(cmd_frame, text="清空日志", command=lambda: self.txt_log.delete(1.0, tk.END), width=10).pack(side=tk.LEFT)

        # ！！然后再将文本框 pack 填充剩下的上方空间 (side=tk.TOP)！！
        self.txt_log = scrolledtext.ScrolledText(log_frame, wrap=tk.WORD, font=("Consolas", 10), bg="#1e1e1e", fg="#d4d4d4")
        self.txt_log.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        
    # ========== 核心逻辑功能方法（完全保持不变） ==========
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
                self.lbl_status.config(text=f"● 已连接: {port}", foreground="green")
                self.log(f"成功连接串口 {port} @ {baud}")
                
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
            self.lbl_status.config(text="● 未连接", foreground="red")
            self.log("已断开串口")

    def serial_receive_loop(self):
        while not self.stop_thread and self.serial_port and self.serial_port.is_open:
            try:
                if self.serial_port.in_waiting:
                    data = self.serial_port.readline()
                    if data:
                        text = data.decode('utf-8', errors='replace').strip()
                        if text:
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

    def send_calibration(self):
        """发送 !CALIBRATION 命令，对所有6个关节应用当前电机位置作为零点偏移"""
        self.send_cmd("!CALIBRATION")
        self.log("已发送 !CALIBRATION（关节零点标定）", "INFO")

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
            if 0 <= node <= 7 and i_limit > 0:
                self.send_cmd(f"#I_LIMIT_J {node} {i_limit}")
            else:
                messagebox.showerror("错误", "节点必须为0-7，电流必须大于0")
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
        self.send_cmd(f"#I_LIMIT_J 0 {current}")
        self.log(f"已设置地轨电流限制为 {current}A", "INFO")

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
            torques = [float(ent.get()) for ent in self.ent_torques]
            # $c0(地轨),c1~c6(关节) — 共7轴；夹爪由!HAND_O/!HAND_C/!HAND_I控制
            cmd = f"${torques[0]:.2f},{torques[1]:.2f},{torques[2]:.2f},{torques[3]:.2f},{torques[4]:.2f},{torques[5]:.2f},{torques[6]:.2f}"
            self.send_cmd(cmd)
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
        # J0=地轨, J1~J6=关节
        base_joints = [0, -75, 180, 0, 0, 0, 0]  # 7个轴: J0(地轨mm)~J6
        
        kp = 0.5
        current_joint_cache = base_joints.copy()
        
        while self.is_servoj_testing and self.is_connected:
            t = time.time() - start_time
            target_j1 = base_joints[0] + 20 * math.sin(2 * math.pi * 0.5 * t)
            
            error_j1 = target_j1 - current_joint_cache[0]
            current_joint_cache[0] += error_j1 * kp
            
            cmd = f">{current_joint_cache[0]:.2f},{base_joints[1]},{base_joints[2]},{base_joints[3]},{base_joints[4]},{base_joints[5]},{base_joints[6]:.2f}"
            try:
                self.serial_port.write(cmd.encode('utf-8'))
                if int(t * 50) % 20 == 0:
                    self.root.after(0, self.log, f"Target: {target_j1:.1f}, Send: {current_joint_cache[0]:.1f}", "TX")
            except Exception as e:
                self.root.after(0, self.log, f"发送失败: {e}", "ERROR")
                break
                
            time.sleep(0.02)

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotSerialAssistant(root)
    root.mainloop()