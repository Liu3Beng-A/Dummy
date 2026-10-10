"""Qt-owned gamepad control, sharing the controller's transport and transactions."""
from copy import deepcopy
import math
import re
import time

from PySide6.QtCore import QObject, QTimer, Signal
from app.gamepad.kinematics import forward_kinematics


def angle_difference(a, b):
    return (a - b + 180.0) % 360.0 - 180.0


def tool_translation(pose, length):
    r, p, y = map(math.radians, pose[3:])
    return [length * (math.cos(r) * math.sin(p) * math.cos(y) + math.sin(r) * math.sin(y)),
            length * (math.cos(r) * math.sin(p) * math.sin(y) - math.sin(r) * math.cos(y)),
            length * math.cos(p) * math.cos(r)]


def motion_command(prefix, values):
    """Keep the existing millesimal precision within the firmware's 63-byte body."""
    fields = []
    for value in values:
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError("运动目标包含无效数值")
        field = f"{value:.3f}".rstrip("0").rstrip(".")
        fields.append("0" if field == "-0" else field)
    command = prefix + ",".join(fields)
    if len(command.encode("ascii")) > 63:
        raise ValueError("运动指令超过主控 63 字节限制")
    return command


class GamepadSession(QObject):
    state_changed = Signal(dict)
    target_changed = Signal(dict)
    initial_pose_completed = Signal(list)

    def __init__(self, controller, config=None, clock=time.monotonic):
        super().__init__(controller)
        from app.gamepad.config import default_config, validate_config
        self.controller, self.clock = controller, clock
        self.config = validate_config(config if config is not None else default_config())
        self.claimed = self.release_pending = False
        self.stage, self.message = "disconnected", "未接管；手柄输入仅供检查"
        self.device_connected = self.centered = False
        self.last_input_at = self._last_sample = None
        self.raw = {}
        self.target_tcp = self.actual_tcp = None
        self.command_reference_tcp = None
        self.gripper_target = None
        self._last_move = self._last_hand = -math.inf
        self._last_pose_sent = self._last_hand_sent = None
        self._mode_query = self._mode_set = self._motion = None
        self._motion_status = None
        self._last_motion_query = -math.inf
        self._stream = self._hand = self._system = None
        self._accepted_motion = {}  # Bounded IDs in this ownership generation only.
        self._purpose = None
        self._issued = 0
        self._idle_retry = None
        self._motion_deadline = self._stop_deadline = None
        self._stop_confirmed = False
        self._fault_after_stop = False
        self._timer = QTimer(self)
        self._timer.setInterval(20)
        self._timer.timeout.connect(self.tick)

    @property
    def control(self):
        return self.config["control"]

    @property
    def robot(self):
        return self.config["robot"]

    @property
    def ready(self):
        c = self.controller
        return (self.claimed and not self.release_pending and self.stage == "ready" and
                self.device_connected and c.is_connected and c._motor_enabled and
                c.current_cmdmode == 2 and not c._stop_locked and not c._calibration_busy)

    def state(self):
        now = self.clock()
        return deepcopy({"stage": self.stage, "message": self.message, "ready": self.ready,
                         "claimed": self.claimed, "release_pending": self.release_pending,
                         "stop_confirmed": self._stop_confirmed,
                         "device_connected": self.device_connected, "centered": self.centered,
                         "last_input_at": self.last_input_at,
                         "input_age_s": None if self.last_input_at is None else max(0., now - self.last_input_at),
                         "target_tcp": self.target_tcp, "actual_tcp": self.actual_tcp,
                         "command_reference_tcp": self.command_reference_tcp,
                         "gripper_target": self.gripper_target, "raw": self.raw})

    def _publish(self):
        self.state_changed.emit(self.state())

    def _stage(self, stage, message):
        self.stage, self.message = stage, message
        self._publish()
        self.controller._emit_state()

    def _target_changed(self):
        if self.target_tcp is None:
            return
        offset = tool_translation(self.target_tcp, self.robot["tool_offset_z"])
        flange = [self.target_tcp[i] - offset[i] for i in range(3)] + self.target_tcp[3:]
        self.target_changed.emit({"flange": flange, "tcp": list(self.target_tcp), "gripper": self.gripper_target})

    def set_config(self, config):
        from app.gamepad.config import validate_config
        if self.claimed:
            raise ValueError("请先释放手柄控制并确认 STOP，再修改配置")
        self.config = validate_config(config)
        self.actual_tcp = self.target_tcp = None
        self.command_reference_tcp = None
        self.gripper_target = None
        self.device_connected = self.centered = False
        self.last_input_at = self._last_sample = None
        self._publish()
        return True

    def claim(self):
        if self.claimed:
            return not self.release_pending
        c = self.controller
        if not c.is_connected or not self.device_connected:
            self._stage("disconnected", "请连接机械臂并检查有效手柄输入，再接管")
            return False
        if self.last_input_at is None or self.clock() - self.last_input_at > .5:
            self._stage("connected", "手柄输入已过期，请重新检查输入")
            return False
        if (c._position_request is not None or c._seq_active or c._parameter_request is not None or
                c._acc_sync_waiting or c._mode_request is not None or c._mode_pending is not None or c._calibration_busy):
            self._stage("connected", "当前任务或位置回读未结束，请等待后接管")
            return False
        c._cancel_motion_for_stop()
        c._control_owner = "gamepad"
        self.claimed = True
        self.release_pending = False
        self._clear_pending()
        self.actual_tcp = self.target_tcp = None
        self._stop_confirmed = False
        self._timer.start()
        self._stage("connected", "已接管；第一次 Start 使能，第二次 Start 到初始姿态，地轨归 0")
        return True

    def _finish_release(self):
        self.claimed = self.release_pending = False
        self._timer.stop()
        self._clear_pending()
        self.controller._control_owner = "ui"
        self._stage("disconnected" if not self.controller.is_connected else "stopped", "手柄控制已释放；拖发保持关闭")

    def release(self):
        if not self.claimed or self.release_pending:
            return
        if not self.controller.is_connected:
            self._finish_release()
            return
        self.release_pending = True
        if self._stop_confirmed and self.stage in ("stopped", "disabled"):
            self._finish_release()
        elif self.stage in ("stopping", "disabling"):
            self._publish()
        else:
            self.stop("释放控制；等待 STOP 确认")

    def _clear_pending(self):
        self.controller._discard_gamepad_position_request()
        self._mode_query = self._mode_set = self._motion = None
        self._motion_status = None
        self._last_motion_query = -math.inf
        self._stream = self._hand = self._system = None
        self._accepted_motion.clear()
        self._purpose = None
        self.command_reference_tcp = None
        self._idle_retry = self._motion_deadline = self._stop_deadline = None
        self._last_pose_sent = self._last_hand_sent = self.gripper_target = None

    def _new_job(self, command, purpose):
        c = self.controller
        return {"command": command, "purpose": purpose, "generation": c._tx_generation,
                "connection": c._connection_id, "queued_at": self.clock(),
                "written": False, "deadline": None, "events": []}

    def _send(self, command, purpose, slot):
        job = self._new_job(command, purpose)
        setattr(self, slot, job)
        if not self.controller.send_cmd(command, source="gamepad"):
            setattr(self, slot, None)
            if not self.controller.is_connected:
                self.disconnected()
            elif self.stage in ("stopping", "disabling"):
                self._stage("fault", "停止或失能未能发送；控制门禁保持锁定")
            else:
                self.stop("指令被拒绝；保持控制门禁", fault=True)
            return None
        job["generation"] = self.controller._tx_generation
        return job

    def start(self):
        if not self.claimed or self.release_pending:
            return False
        if self.stage in ("enabled", "rested"):
            return self.initialize()
        if self.stage not in ("connected", "disabled", "stopped", "protected", "fault"):
            return False
        if self.stage == "fault" and not self._stop_confirmed:
            self._stage("fault", "停止未确认；请检查连接，不能重新使能")
            return False
        self._clear_pending()
        self._stop_confirmed = False
        self._fault_after_stop = False
        self._stage("starting", "等待 Started ok；尚未允许运动")
        return self._send("!START", "start", "_system") is not None

    def initialize(self):
        if self.stage == "init_confirmation":
            return self.confirm_initialization()
        if not self.claimed or self.release_pending or not self.controller._motor_enabled or self.stage not in ("enabled", "rested", "ready"):
            return False
        return self._begin_joint("init")

    def rest(self):
        return self._begin_joint("rest") if self.ready else False

    def _begin_joint(self, purpose):
        self._clear_pending()
        self._purpose = purpose
        if purpose == "init":
            # Initialization deliberately sends one MoveJ only. The queued
            # reply supplies its actual mode; INT has no completion reply.
            return self._send_joint(None)
        self._stage("wait_idle", "等待主控空闲；本轮 MoveJ 地轨目标固定为 0")
        return self._query_mode("idle")

    def _query_mode(self, purpose):
        request = self.controller.allocate_command_mode_request()
        job = self._send(f"#GET_CMDMODE {request}", purpose, "_mode_query")
        if job is not None:
            job["request"] = request
        return job is not None

    def _set_mode(self, mode):
        self._stage("int_mode", f"等待确认模式 {mode}")
        job = self._send(f"#CMDMODE {mode}", mode, "_mode_set")
        return job is not None

    def _send_joint(self, mode):
        joints = self.robot["init_joints" if self._purpose == "init" else "rest_joints"] + [0.]
        try:
            command = motion_command(">", joints + [self.robot["init_speed"]])
        except ValueError as error:
            self._clear_pending()
            self._stage("enabled", str(error) + "；未发送 MoveJ")
            return False
        self._stage("joint_queued", "等待 MoveJ 编号；按当前模式执行，MoveL 锁定")
        job = self._send(command, "joint", "_motion")
        if job is not None:
            job.update(id=None, floor=self._issued,
                       joints=[float(value) for value in command[1:].split(",")[:7]],
                       mode=mode, accepted=False)
        return job is not None

    def _joint_completed(self):
        job = self._motion
        if job is None:
            return
        if self._purpose == "init":
            job["completed"] = True
            job["deadline"] = self._motion_deadline = None
            self._stage("init_confirmation", "MoveJ 已完成；确认到位仅在本地解锁。连续手柄控制需要 INT 模式")
            return
        self._seed_joint_target(job)
        self._motion = self._motion_status = None
        self._motion_deadline = None
        self._stage("rested", "休息运动已完成；未查询实际位置，需 Start 重新初始化")

    def _seed_joint_target(self, job):
        flange = forward_kinematics(job["joints"])
        offset = tool_translation(flange, self.robot["tool_offset_z"])
        self.target_tcp = [flange[i] + offset[i] for i in range(3)] + flange[3:]
        self.command_reference_tcp = list(self.target_tcp)
        self._target_changed()

    def confirm_initialization(self):
        job = self._motion
        if (not self.claimed or self.release_pending or self.stage != "init_confirmation" or
                not job or not job["written"] or not (job["accepted"] or job.get("completed")) or
                not self.controller._motor_enabled or self.controller._stop_locked):
            return False
        if job["mode"] != 2 or self.controller.current_cmdmode != 2:
            self.message = "连续手柄控制需要 INT；请释放控制后设置 INT，再接管初始化。不会自动切换模式"
            self._publish()
            return False
        self._seed_joint_target(job)
        self._accepted_motion[job["id"]] = (job["generation"], job["connection"])
        self._motion = self._motion_status = None
        self._motion_deadline = None
        self.initial_pose_completed.emit(list(job["joints"]))
        self._stage("centering", "已手动确认到位；等待摇杆与扳机回中。未查询实际位置")
        return True

    def stop(self, reason="手柄软停", fault=False):
        if not self.claimed or self.stage in ("stopping", "disabling"):
            return False
        self._clear_pending()
        self._stop_confirmed = False
        self._fault_after_stop = fault
        self._stage("stopping", reason + "；等待 STOP 确认")
        return self._send("!STOP", "stop", "_system") is not None

    def disable(self):
        if not self.claimed or self.stage in ("stopping", "disabling"):
            return False
        self._clear_pending()
        self._stop_confirmed = False
        self._fault_after_stop = False
        self._stage("disabling", "等待八轴失能确认")
        return self._send("!DISABLE", "disable", "_system") is not None

    def external_stop(self, command):
        if not self.claimed:
            return
        self._clear_pending()
        self._stop_confirmed = False
        self._fault_after_stop = False
        self._stage("stopping" if command == "!STOP" else "disabling", "界面停止；等待设备确认")
        self._system = self._new_job(command, "stop" if command == "!STOP" else "disable")
        self._system["generation"] += 1  # Controller dispatch invalidates the old generation next.

    def controller_cancelled(self):
        if self.claimed and self.stage not in ("stopping", "disabling"):
            self._clear_pending()
            self._stage("protected", "运动已被停止或保护取消；等待设备状态")

    def disconnected(self):
        self._finish_release()
        self._issued = 0  # A reconnected/rebooted controller may restart its IDs.
        self.actual_tcp = self.target_tcp = None
        self._stage("disconnected", "机械臂连接已断开；未发送到断开的链路")

    def command_written(self, command, generation, connection):
        for slot in ("_system", "_mode_query", "_mode_set", "_motion", "_motion_status", "_stream", "_hand"):
            job = getattr(self, slot)
            if (job is None or job["written"] or job["command"] != command or
                    job["generation"] != generation or job["connection"] != connection):
                continue
            job["written"] = True
            job["deadline"] = self.clock() + (10.0 if job["purpose"] in ("stop", "disable") else self.control["response_timeout_s"])
            if slot == "_stream":
                self._last_move = self.clock()
            elif slot == "_hand":
                self._last_hand = self.clock()
            elif slot == "_motion_status":
                self._last_motion_query = self.clock()
            if slot == "_mode_query" and job["purpose"] == "idle" and self._motion_deadline is None:
                self._motion_deadline = self.clock() + self.robot["motion_timeout_s"]
            events, job["events"] = job["events"], []
            for event in events:
                self.handle_line(event)
            return

    def _defer(self, job, event):
        if not job["written"]:
            if len(job["events"]) < 16:
                job["events"].append(event)
            return True
        return False

    def handle_line(self, line):
        if not self.claimed:
            return
        if line in ("STOP locked", "STOP locked disabled", "Disabled ok"):
            if self.stage in ("stopping", "disabling") and self._system and self._defer(self._system, line):
                return
            self._clear_pending()
            self._stop_confirmed = True
            if self.release_pending:
                self._finish_release()
            else:
                self._stage("fault" if self._fault_after_stop else "disabled" if line == "Disabled ok" else "stopped", "停止已确认；需重新 Start 与初始化")
            return
        protection = (line.startswith("STALL ") and "EXCLUDED" not in line and
                      not re.search(r"\bphase=(?:IDLE|0)(?:\s|$)", line))
        if protection or line.startswith(("!CALIBRATION ", "error CALIBRATION ", "STOP fault ")) or (line.startswith("PROTECT ") and " failed reason=" in line) or line == "error STOP_LOCKED":
            self._clear_pending()
            self._stage("protected", line)
            return
        if line in ("Stopping ok", "Disable pending"):
            if self.stage not in ("stopping", "disabling"):
                self._clear_pending()
                self._stop_deadline = self.clock() + 10.
                self._stage("stopping" if line == "Stopping ok" else "disabling", line)
            return
        if line == "Started ok" and self.stage == "starting" and self._system:
            if not self._defer(self._system, line):
                self._system = None
                self._stage("enabled", "已使能；第二次 Start 将 MoveJ 到初始姿态，地轨归 0")
            return
        if self.stage in ("stopped", "disabled", "stopping", "disabling", "fault", "protected", "disconnected", "connected"):
            return
        match = re.fullmatch(r"ok CMDMODE mode=([1-4]) busy=([01]) issued=([0-9]+) req=([0-9]+)", line)
        if match and self._mode_query and int(match[4]) == self._mode_query["request"]:
            job = self._mode_query
            if self._defer(job, line):
                return
            mode, busy, issued = map(int, match.groups()[:3])
            if issued > 0xFFFFFFFF:
                return
            self._mode_query = None
            self.controller.current_cmdmode, self._issued = mode, issued
            if busy:
                if self.stage == "wait_idle":
                    self._idle_retry = self.clock() + .1
                else:
                    self.stop("模式确认时主控仍忙", fault=True)
            elif job["purpose"] == "idle":
                if mode not in (1, 2, 3):
                    self._clear_pending()
                    self._stage("enabled", "当前模式不支持 MoveJ；请释放控制后选择 SEQ、INT 或 TRJ")
                else:
                    self._send_joint(mode)
            elif mode != 2:
                self.stop("手柄未确认 INT 模式", fault=True)
            else:
                self._motion_deadline = None
                self._stage("centering", "等待摇杆与扳机回中；采用已完成初始化的软件目标，无位置回读")
            return
        match = re.fullmatch(r"ok CMDMODE mode=([1-4])", line)
        if match and self._mode_set and int(match[1]) == self._mode_set["purpose"]:
            if not self._defer(self._mode_set, line):
                self._mode_set = None
                self._stage("int_check", "核对实际模式与空闲状态")
                self._query_mode("int")
            return
        match = re.fullmatch(r"queued (MOVEJ|MOVEL) id=([0-9]+) mode=(SEQ|INT|TRJ|TUN)", line)
        if match:
            kind, motion_id, mode = match[1], int(match[2]), match[3]
            job = self._motion if kind == "MOVEJ" else self._stream
            if job is None or not 1 <= motion_id <= 0xFFFFFFFF:
                return
            if self._defer(job, line):
                return
            if not 0 < ((motion_id - job["floor"]) & 0xFFFFFFFF) < 0x80000000 or job.get("id") is not None:
                return
            if kind == "MOVEJ" and self._purpose == "init" and job["mode"] is None:
                job["mode"] = {"SEQ": 1, "INT": 2, "TRJ": 3, "TUN": 4}[mode]
                self.controller.current_cmdmode = job["mode"]
            expected_mode = {1: "SEQ", 2: "INT", 3: "TRJ", 4: "TUN"}[job["mode"]] if kind == "MOVEJ" else "INT"
            if mode != expected_mode:
                self.stop("运动实际模式与手柄会话不符", fault=True)
                return
            job["id"] = self._issued = motion_id
            if kind == "MOVEJ":
                if job["mode"] != 2:
                    job["deadline"] = None
                self._motion_deadline = self.clock() + self.robot["motion_timeout_s"]
                self._stage("joint_moving", f"MoveJ #{motion_id} 执行中")
            return
        match = re.fullmatch(r"ok MOVEJ id=([0-9]+)(?: ms=([0-9]+))?", line)
        if match and self._motion:
            job = self._motion
            if job["mode"] == 2:
                return  # INT's accepted acknowledgement is not completion.
            if self._defer(job, line):
                return
            if int(match[1]) == job.get("id") and (match[2] is None or int(match[2]) <= 0xFFFFFFFF):
                self._joint_completed()
            return
        match = re.fullmatch(r"accepted MOVEJ id=([0-9]+)", line)
        if match and self._motion:
            job = self._motion
            if not self._defer(job, line) and job["mode"] == 2 and int(match[1]) == job.get("id"):
                job["accepted"] = True
                job["deadline"] = None
                if self._purpose == "init":
                    self._motion_deadline = None
                    self._stage("init_confirmation", "MoveJ 已受理；请观察实物到位后点击“确认到位”，再将手柄回中。不会自动查询或切换模式")
            return
        match = re.fullmatch(r"MOTION id=([0-9]+) kind=(MOVEJ|MOVEL|HOME|RESET) phase=([0-5]) mode=([1-4]) ms=([0-9]+)", line)
        if match and self._motion_status and self._motion:
            job, status = self._motion, self._motion_status
            motion_id, kind, phase, mode, elapsed = int(match[1]), match[2], int(match[3]), int(match[4]), int(match[5])
            if motion_id != job.get("id") or kind != "MOVEJ" or mode != job["mode"] or elapsed > 0xFFFFFFFF:
                return
            if self._defer(status, line):
                return
            self._motion_status = None
            if phase == 3:
                self._joint_completed()
            elif phase == 5:
                # Device failure already starts its own protection/retreat.
                self.controller._cancel_motion_for_stop()
                self.controller._stop_locked = True
                self._stage("protected", "主控报告 MoveJ 失败；保留保护动作")
            elif phase in (0, 4):
                self.stop("主控报告 MoveJ 已取消或状态无效", fault=True)
            return
        match = re.fullmatch(r"accepted MOVEL id=([0-9]+)", line)
        if match and self._stream:
            if not self._defer(self._stream, line) and int(match[1]) == self._stream.get("id"):
                self._accepted_motion[int(match[1])] = (self._stream["generation"], self._stream["connection"])
                if len(self._accepted_motion) > 64:
                    del self._accepted_motion[next(iter(self._accepted_motion))]
                self.command_reference_tcp = list(self._stream["tcp"])
                self._stream = None
                self._publish()
            return
        match = re.fullmatch(r"MOTION fail id=([0-9]+) reason=([A-Z_0-9]+)", line)
        if match:
            motion_id = int(match[1])
            if not 1 <= motion_id <= 0xFFFFFFFF:
                return
            for job in (self._motion, self._stream):
                if job and (job.get("id") == motion_id or
                            (not job["written"] and 0 < ((motion_id - job["floor"]) & 0xFFFFFFFF) < 0x80000000)):
                    if self._defer(job, line):
                        return
            belongs = any(job and job.get("id") == motion_id for job in (self._motion, self._stream))
            belongs |= self._accepted_motion.get(motion_id) == (self.controller._tx_generation, self.controller._connection_id)
            if belongs:
                # Firmware already soft-stops without cancelling local retreat.
                # Cancel only our queue; a second STOP would abort that protection.
                self.controller._cancel_motion_for_stop()
                self.controller._stop_locked = True
                self._stage("protected", line)
            return
        match = re.fullmatch(r"ok hand pos=([0-9]+)", line)
        if match and self._hand:
            if not self._defer(self._hand, line) and int(match[1]) == self._hand["opening"]:
                self._hand = None
            return
        match = re.fullmatch(r"error (MOVEJ|MOVEL) id=([0-9]+) reason=([A-Z_0-9]+)", line)
        if match:
            job = self._motion if match[1] == "MOVEJ" else self._stream
            if job and not self._defer(job, line) and int(match[2]) == job.get("id"):
                self.stop(line, fault=True)
        elif (line.startswith("error CMDMODE") and (self._mode_set or self._mode_query) or
              line in ("error DISABLED", "error motion queue full")):
            self.stop(line, fault=True)

    def set_target(self, tcp):
        if not self.ready:
            return False
        if (not isinstance(tcp, (list, tuple)) or len(tcp) != 6 or
                any(isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) for value in tcp)):
            self.stop("目标包含无效数值", fault=True)
            return False
        offset = tool_translation(tcp, self.robot["tool_offset_z"])
        flange = [tcp[i] - offset[i] for i in range(3)]
        if tcp[2] < self.robot["z_min_height"] or any(not self.robot["flange_limits"][key][0] <= value <= self.robot["flange_limits"][key][1] for key, value in zip(("x", "y", "z"), flange)):
            self.message = "目标超过配置范围，已拒绝该步"
            return False
        reference = self.command_reference_tcp
        if reference is None or math.dist(tcp[:3], reference[:3]) > self.control["max_position_lead_mm"] or max(abs(angle_difference(a, b)) for a, b in zip(tcp[3:], reference[3:])) > self.control["max_rotation_lead_deg"]:
            self.message = "软件命令步长达到上限，等待上一指令受理；未检测真实跟随误差"
            return False
        self.target_tcp = list(tcp)
        self._target_changed()
        return True

    def set_gripper_base(self, opening):
        if not self.ready:
            return False
        if isinstance(opening, bool) or not isinstance(opening, (int, float)) or not math.isfinite(opening) or not 0 <= opening <= 100:
            raise ValueError("夹爪目标应为 0～100 的有限数值")
        self.gripper_target = float(opening)
        self._target_changed()
        self.tick()
        return True

    def handle_input(self, snapshot):
        now = self.clock()
        timestamp = snapshot.get("timestamp", now)
        if isinstance(timestamp, bool) or not isinstance(timestamp, (int, float)) or not math.isfinite(timestamp) or timestamp > now + .1:
            if self.claimed:
                self.stop("手柄输入时间戳无效", fault=True)
            return
        connected = bool(snapshot.get("connected"))
        decoded = snapshot.get("decoded")
        valid_input = connected and isinstance(decoded, dict) and not snapshot.get("error")
        if not valid_input:
            self.device_connected = self.centered = False
            self.raw = deepcopy(snapshot.get("raw", {}))
        if self.claimed and (not valid_input or now - timestamp > .5):
            self.stop("手柄断开或输入过期", fault=True)
        if self._last_sample is not None and timestamp <= self._last_sample:
            self._publish()
            return
        self.device_connected = bool(valid_input)
        self.centered = bool(valid_input and decoded.get("centered"))
        self.raw = deepcopy(snapshot.get("raw", {}))
        dt = 0. if self._last_sample is None else max(0., min(.05, timestamp - self._last_sample))
        self._last_sample = self.last_input_at = timestamp
        if self.claimed and self.device_connected and now - timestamp <= .5:
            actions = {"start": self.start, "initialize": self.initialize,
                       "rest": self.rest, "disable": self.disable}
            pressed = snapshot.get("pressed", [])
            if "stop" in pressed:
                self.stop()
            else:
                for action in pressed:
                    if action in actions:
                        actions[action]()
                        break
            if self.ready:
                from app.gamepad.input import motion_delta
                try:
                    pos, att = motion_delta(decoded, self.control, dt)
                    target = [self.target_tcp[i] + pos[key] for i, key in enumerate(("x", "y", "z"))]
                    target += [(self.target_tcp[i + 3] + att[key] + 180) % 360 - 180 for i, key in enumerate(("r", "p", "y"))]
                    if any(pos.values()) or any(att.values()):
                        self.set_target(target)
                    lt, rt = decoded["lt"], decoded["rt"]
                    if any(isinstance(value, bool) or not isinstance(value, (int, float)) or
                           not math.isfinite(value) or not 0 <= value <= 1 for value in (lt, rt)):
                        raise ValueError("invalid trigger")
                    if bool(lt) != bool(rt):
                        if self.gripper_target is None:
                            self.message = "请先明确建立夹爪开度目标；首次扳机不会发送默认开度"
                        else:
                            self.gripper_target = max(0., min(100., self.gripper_target + (lt - rt) * self.control["gripper_speed_percent_s"] * dt))
                            self._target_changed()
                except (ValueError, TypeError, KeyError, OverflowError):
                    self.stop("手柄输入无效", fault=True)
        self._publish()

    def tick(self):
        if not self.claimed:
            return
        now = self.clock()
        if not self.controller.is_connected:
            self.disconnected()
            return
        if self.stage not in ("stopping", "disabling", "stopped", "disabled", "protected", "fault") and (self.last_input_at is None or now - self.last_input_at > .5):
            self.stop("手柄输入超时", fault=True)
            return
        for slot in ("_system", "_mode_query", "_mode_set", "_motion", "_motion_status", "_stream", "_hand"):
            job = getattr(self, slot)
            if job and ((job["deadline"] is not None and now > job["deadline"]) or (not job["written"] and now - job["queued_at"] > 10.)):
                if self.stage in ("stopping", "disabling"):
                    self._clear_pending()
                    self._stage("fault", "停止或失能未确认；控制门禁保持锁定")
                else:
                    self.stop("指令发送或回执超时", fault=True)
                return
        if self._stop_deadline is not None and now > self._stop_deadline:
            self._clear_pending()
            self._stage("fault", "停止未确认；控制门禁保持锁定")
            return
        if self._motion_deadline is not None and now > self._motion_deadline:
            self.stop("初始化或休息运动超时", fault=True)
            return
        if self._idle_retry is not None and now >= self._idle_retry:
            self._idle_retry = None
            self._query_mode("idle")
        if (self._purpose != "init" and self.stage == "joint_moving" and self._motion and self._motion["accepted"] and
                self._motion_status is None and now - self._last_motion_query >= max(.2, self.control["feedback_interval_s"])):
            self._send("#MOTION_STATUS", "joint_status", "_motion_status")
        if self.stage == "centering" and self.device_connected and self.centered:
            self._last_pose_sent = tuple(round(value, 3) for value in self.target_tcp)
            self._stage("ready", "INT 已确认；手柄 MoveL 解锁")
        if not self.ready:
            return
        pose = tuple(round(value, 3) for value in self.target_tcp)
        if self._stream is None and pose != self._last_pose_sent and now - self._last_move >= max(.05, self.control["send_interval_s"]):
            offset = tool_translation(pose, self.robot["tool_offset_z"])
            flange = [pose[i] - offset[i] for i in range(3)] + list(pose[3:])
            try:
                command = motion_command("@", flange + [self.robot["cmd_speed"]])
            except ValueError as error:
                self.stop(str(error), fault=True)
                return
            job = self._send(command, "stream", "_stream")
            if job is not None:
                sent_flange = [float(value) for value in command[1:].split(",")[:6]]
                sent_offset = tool_translation(sent_flange, self.robot["tool_offset_z"])
                sent_tcp = [sent_flange[i] + sent_offset[i] for i in range(3)] + sent_flange[3:]
                job.update(id=None, floor=self._issued, tcp=tuple(sent_tcp))
                self._last_pose_sent, self._last_move = pose, now
            else:
                return
        opening = None if self.gripper_target is None else round(self.gripper_target)
        if self._hand is None and opening is not None and opening != self._last_hand_sent and now - self._last_hand >= max(.1, self.control["gripper_interval_s"]):
            job = self._send(f"!HAND_POS {opening}", "hand", "_hand")
            if job is not None:
                job["opening"] = opening
                self._last_hand_sent, self._last_hand = opening, now
        self._publish()

    def close(self):
        self._timer.stop()
        self._clear_pending()
