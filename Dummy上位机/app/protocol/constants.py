"""Verified UI/protocol constants migrated from 串口助手_old.py (2026-10-09)."""
BAUDRATE = 115200

RGB_MODES = ("0 单色0", "1 单色1", "2 单色2", "3 彩虹", "4 潮汐",
             "5 呼吸", "6 赛博", "7 心跳", "8 旋转", "9 闪烁",
             "10 AUTO SEQ常亮", "11 AUTO INT常亮", "12 AUTO TRJ常亮")

RGB_AUTO_MODES = ("SEQ 顺序", "INT 打断", "TRJ 轨迹")

# Automatic movement/calibration animations are independent of manual preset 8.
RGB_MOTION_EFFECTS = ("圆圈旋转", "伸缩光弧", "双峰行波", "双向汇聚")
RGB_MOTION_TARGETS = ("SEQ 运动", "INT 运动", "TRJ 运动", "校准")
RGB_MOTION_STATES = ("RUN_SEQ", "RUN_INT", "RUN_TRJ", "CALIBRATION")
RGB_STATE_MODES = RGB_MODES + tuple(f"{13 + index} 自动 · {name}"
                                   for index, name in enumerate(RGB_MOTION_EFFECTS))

RGB_COLOR_SOURCES = ("预设 / 动画颜色", "该状态单独颜色", "AUTO_SEQ", "AUTO_INT", "AUTO_TRJ")

RGB_STATES = (
    ("BOOT", "上电初始化"), ("DISABLED", "已失能"),
    ("READY_SEQ", "SEQ 待机"), ("READY_INT", "INT 待机"), ("READY_TRJ", "TRJ 待机"), ("READY_TUN", "TUN 待机"),
    ("RUN_SEQ", "SEQ 执行"), ("RUN_INT", "INT 执行"), ("RUN_TRJ", "TRJ 执行"), ("RUN_TUN", "TUN 执行"),
    ("CALIBRATION", "校准 / 保存中"), ("STOP", "STOP / 失能确认中"), ("STARTING", "使能确认中"),
)

JOINT_RANGES = [(-175, 175), (-75, 90), (0, 180), (-270, 270), (-100, 100), (-180, 180)]

JOINT_DEFAULTS = [0, -75, 180, 0, 0, 0]

DEFAULT_SPEED = 35

RAIL_RANGE = (-250, 250)

RAIL_DEFAULT = 0

# Local unqueried PID defaults for the rail only. A returned motor value or an
# existing draft always takes precedence; selecting a node never sends these.
RAIL_PID_DEFAULTS = {"KP": 150, "KV": 180, "KI": 25, "KD": 50}

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

POSE_LABELS = ['X', 'Y', 'Z', 'A', 'B', 'C']

POSE_RANGES = [(-50, 300), (-100, 260), (-150, 300), (-180, 180), (-180, 180), (-180, 180)]

POSE_DEFAULTS = [146.52, 0, 288.92, 180, 0, 180]
