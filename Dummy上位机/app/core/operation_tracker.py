"""Read-only operation presentation; never authorizes or sends device commands.

Writer admission is reserved before serial.write, so RX may precede the Qt TX
notification. Unnumbered old-generation admissions remain as tombstones: a late
queued receipt must not acquire a newer request's identity.
"""
import re
import threading
import time

from app.protocol.parser import finite_numbers
from app.visualization.mapping import target_pose_matches


_MOTION = r"(MOVEJ|MOVEL|HOME|RESET)"
_NAMES = {"MOVEJ": "MoveJ", "MOVEL": "MoveL", "HOME": "HOME", "RESET": "REST"}
_TERMINAL = {"completed", "rejected", "cancelled"}


class OperationTracker:
    LIMIT = 128
    CONFIRM_SECONDS = 10.

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.lock = threading.RLock()
        self.reset(0)

    def reset(self, connection):
        with self.lock:
            self.connection = connection
            self.records = []
            self.serial = 0
            self.latest = None
            self.last_queued_id = None
            self.ambiguous = set()
            self.protection = None

    @staticmethod
    def command_kind(command):
        if command in ("!START", "!STOP", "!DISABLE"):
            return command[1:].lower()
        if command in ("!HOME", "!RESET"):
            return command[1:]
        if command.startswith((">", "&")):
            return "MOVEJ"
        if command.startswith("@"):
            return "MOVEL"
        return None

    def _new(self, command, generation, connection, source):
        kind = self.command_kind(command)
        if kind is None or connection != self.connection:
            return None
        self._invalidate_position_observations()
        self.serial += 1
        values = finite_numbers(command[1:]) if kind == "MOVEJ" else None
        if values is not None and len(values) not in (7, 8):
            values = None
        record = {"serial": self.serial, "command": command, "kind": kind,
                  "generation": generation, "connection": connection, "source": source,
                  "requested": False, "admitted": False, "written": False,
                  "motion_id": None, "phase": "queued", "since": self.clock(), "detail": "",
                  "position_target": tuple(values[:7]) if values is not None else None,
                  "position_matched": False, "position_invalidated": False,
                  "accepted_feedback_sequence": None, "accepted_feedback_after": None}
        self.records.append(record)
        self.latest = record
        # Numbered terminal records can safely be forgotten. Never silently evict
        # unnumbered admissions: mark the kind ambiguous until reconnect instead.
        if len(self.records) > self.LIMIT:
            removed = self.records.pop(0)
            if removed["admitted"] and removed["motion_id"] is None and removed["kind"] in _NAMES:
                self.ambiguous.add(removed["kind"])
        return record

    def _invalidate_position_observations(self):
        for record in self.records:
            record["position_invalidated"] = True
            record["position_matched"] = False

    def requested(self, command, generation, connection, source="ui"):
        with self.lock:
            # The writer can run between queue.put and this observer callback.
            record = next((r for r in self.records if not r["requested"] and
                           (r["command"], r["generation"], r["connection"]) ==
                           (command, generation, connection)), None)
            if record is None:
                record = self._new(command, generation, connection, source)
            if record is not None:
                record["requested"], record["source"] = True, source

    def admitted(self, command, generation, connection):
        with self.lock:
            record = next((r for r in self.records if not r["admitted"] and
                           (r["command"], r["generation"], r["connection"]) ==
                           (command, generation, connection)), None)
            if record is None:
                record = self._new(command, generation, connection, "unknown")
            if record is not None:
                record["admitted"] = True
                record["phase"] = "sending"
                record["since"] = self.clock()

    def written(self, command, generation, connection):
        with self.lock:
            record = next((r for r in self.records if r["admitted"] and not r["written"] and
                           (r["command"], r["generation"], r["connection"]) ==
                           (command, generation, connection)), None)
            if record is not None:
                record["written"] = True
                if record["phase"] == "sending":
                    record["phase"] = "waiting"

    def carry_start_generation(self, old_generation, new_generation):
        """The existing Start-pending handler retires the UI motion generation."""
        with self.lock:
            record = self._system("start", old_generation)
            latest = self.latest
            if (record is not None and record["phase"] == "waiting" and record["detail"] == "Start pending" and
                    latest is not None and latest["generation"] == old_generation):
                record["generation"] = new_generation
                # A UI target submitted while START was in flight is retired by
                # the existing handler. Its stale presentation must not hide the
                # still-live START. A newer STOP/DISABLE generation is untouched.
                self.latest = record

    def _system(self, kind, generation):
        return next((r for r in reversed(self.records) if r["kind"] == kind and r["admitted"] and
                     r["generation"] == generation and r["phase"] not in _TERMINAL), None)

    def _numbered(self, kind, motion_id, generation):
        if not 1 <= motion_id <= 0xFFFFFFFF:
            return None
        return next((r for r in self.records if r["kind"] == kind and r["motion_id"] == motion_id and
                     r["generation"] == generation and r["connection"] == self.connection), None)

    def receive(self, line, generation, connection, *, feedback_sequence=0, feedback_after=None):
        with self.lock:
            if connection != self.connection:
                return
            system = {"Start pending": ("start", "waiting"), "Started ok": ("start", "completed"),
                      "Stopping ok": ("stop", "waiting"), "STOP locked": ("stop", "completed"),
                      "STOP locked disabled": ("stop", "completed"),
                      "Disable pending": ("disable", "waiting"), "Disabled ok": ("disable", "completed")}
            if line in system:
                kind, phase = system[line]
                if kind in ("stop", "disable") or line == "Start pending":
                    self._invalidate_position_observations()
                record = self._system(kind, generation)
                if record is not None:
                    record["phase"], record["detail"] = phase, line
                if line in ("Started ok", "Disabled ok"):
                    self.protection = None
                return
            if (line.startswith("STOP fault ") or
                    line.startswith("PROTECT ") and " failed reason=" in line or
                    line.startswith("STALL ") and "EXCLUDED" not in line and
                    not re.search(r"\bphase=(?:IDLE|0)(?:\s|$)", line)):
                self._invalidate_position_observations()
                if self.protection is None:
                    self.protection = {"detail": line, "since": self.clock()}
                self.protection["last_request_serial"] = self.serial
                return
            match = re.fullmatch(r"error (START|DISABLE) (.+)", line)
            if match:
                record = self._system(match[1].lower(), generation)
                if record is not None:
                    record["phase"], record["detail"] = "rejected", line
                return
            match = re.fullmatch(r"queued " + _MOTION + r" id=([0-9]+) mode=(SEQ|INT|TRJ|TUN)", line)
            if match:
                kind, number, mode = match[1], int(match[2]), match[3]
                if not 1 <= number <= 0xFFFFFFFF:
                    return
                if self.last_queued_id is not None and not 0 < ((number-self.last_queued_id) & 0xFFFFFFFF) < 0x80000000:
                    return
                self.last_queued_id = number
                if kind in self.ambiguous:
                    return
                record = next((r for r in self.records if r["kind"] == kind and r["admitted"] and
                               r["motion_id"] is None), None)
                if record is not None:
                    record["motion_id"], record["mode"] = number, mode
                    if record["generation"] == generation:
                        record["phase"] = "queued"
                return
            match = re.fullmatch(r"(accepted|ok) " + _MOTION + r" id=([0-9]+)(?: ms=([0-9]+))?", line)
            if match:
                verb, kind, number, elapsed = match.groups()
                if elapsed is not None and (verb != "ok" or int(elapsed) > 0xFFFFFFFF):
                    return
                record = self._numbered(kind, int(number), generation)
                if record is not None and record["phase"] not in _TERMINAL:
                    if verb == "ok":
                        record["phase"], record["detail"] = "completed", line
                    else:
                        # Freeze the first acceptance boundary. A duplicate or
                        # delayed accepted line must not withdraw an observation.
                        if record["accepted_feedback_sequence"] is None:
                            record["accepted_feedback_sequence"] = feedback_sequence
                            record["accepted_feedback_after"] = self.clock() if feedback_after is None else feedback_after
                        if record["phase"] not in ("executing", "waiting_confirmation"):
                            record["phase"], record["detail"] = "accepted", line
                return
            match = re.fullmatch(r"error " + _MOTION + r" id=([0-9]+) reason=([A-Z_0-9]+)", line)
            if match:
                kind, number, reason = match.groups()
                record = self._numbered(kind, int(number), generation)
                if record is not None and record["phase"] not in _TERMINAL:
                    record["phase"], record["detail"] = "cancelled" if reason == "CANCELLED" else "rejected", line
                return
            match = re.fullmatch(r"MOTION id=([0-9]+) kind=" + _MOTION + r" phase=([0-5]) mode=([1-4]) ms=([0-9]+)", line)
            if match:
                number, kind, phase, mode, elapsed = match.groups()
                record = self._numbered(kind, int(number), generation)
                if (record is not None and record["phase"] not in _TERMINAL and
                        int(elapsed) <= 0xFFFFFFFF and
                        record.get("mode") == {"1": "SEQ", "2": "INT", "3": "TRJ", "4": "TUN"}[mode]):
                    if int(phase) < record.get("last_motion_phase", 0):
                        return
                    record["last_motion_phase"] = int(phase)
                    record["phase"] = {"0": "uncertain", "1": "waiting_confirmation", "2": "executing",
                                       "3": "completed", "4": "cancelled", "5": "rejected"}[phase]
                    record["detail"] = line
                return
            match = re.fullmatch(r"MOTION fail id=([0-9]+) reason=([A-Z_0-9]+)", line)
            if match:
                record = next((self._numbered(kind, int(match[1]), generation) for kind in _NAMES
                               if self._numbered(kind, int(match[1]), generation) is not None), None)
                if record is not None and record["phase"] not in _TERMINAL:
                    record["phase"], record["detail"] = "rejected", line
                return
            if line in ("error DISABLED", "error STOP_LOCKED", "error motion queue full"):
                if line in ("error DISABLED", "error STOP_LOCKED"):
                    self._invalidate_position_observations()
                pending = [r for r in self.records if r["kind"] in _NAMES and r["admitted"] and
                           r["generation"] == generation and r["motion_id"] is None]
                # These rejections carry no request token. With more than one
                # candidate we cannot identify the failed command safely.
                for record in pending:
                    self.ambiguous.add(record["kind"])
                    record["phase"], record["detail"] = "uncertain", line

    def _view(self, kind, phase, title, reason, next_step, tone="info", record=None, since=None, **extra):
        record = record or {}
        return {"kind": kind, "phase": phase, "title": title, "reason": reason, "next_step": next_step,
                "tone": tone, "elapsed_s": round(max(0., self.clock()-(record.get("since") if since is None else since)), 1)
                if since is not None or record.get("since") is not None else None,
                "command": record.get("command"), "motion_id": record.get("motion_id"),
                "source": record.get("source", "device"), **extra}

    def snapshot(self, context):
        with self.lock:
            return self._snapshot(context)

    @staticmethod
    def _position_matches(record, context):
        """Observe the sent INT target, without changing protocol completion.

        Both a new sample number and its original receive timestamp must follow
        acceptance. This excludes a cached matching pose or a buffered pre-send
        sample delivered late. The timestamp shares the controller feedback
        clock; it is deliberately separate from the elapsed-display clock.
        """
        floor = record.get("accepted_feedback_sequence")
        stamp = context.get("actual_received_at")
        sequence = context.get("actual_feedback_sequence", 0)
        eligible = (record["kind"] == "MOVEJ" and record.get("mode") == "INT" and
                    record.get("position_target") is not None and not record.get("position_invalidated") and
                    record["phase"] in ("accepted", "executing", "waiting_confirmation") and
                    floor is not None and sequence > floor and stamp is not None and
                    stamp >= record["accepted_feedback_after"] and not context.get("actual_stale", True) and
                    not context.get("polling_paused", False) and context.get("actual_pose") is not None)
        matched = False
        if eligible:
            try:
                matched = target_pose_matches(record["position_target"], context["actual_pose"],
                                              was_matched=record["position_matched"])
            except (ValueError, TypeError, OverflowError):
                pass
        record["position_matched"] = matched
        return matched

    def _snapshot(self, c):
        if not c.get("connected"):
            return self._view("connection", "disconnected", "未连接机械臂", "设备状态无法确认。", "选择串口并连接设备。", "warning")
        record = self.latest
        if record is not None and (record["connection"] != self.connection or record["generation"] != c.get("generation")):
            record = None
        if record and (c.get("actual_stale", True) or c.get("polling_paused", False)):
            # Reset display hysteresis even when RGB saving or another higher
            # priority status temporarily owns the panel.
            record["position_matched"] = False
        recovering_start = bool(record and record["kind"] == "start" and record["phase"] not in _TERMINAL and
                                (self.protection is None or record["serial"] > self.protection["last_request_serial"]))
        if self.protection is not None and c.get("stop_locked") and not recovering_start:
            return self._view("protection", "locked", "保护锁定", self.protection["detail"],
                              "检查保护原因和机械状态；确认条件恢复后再 START 或 DISABLE。", "error", since=self.protection["since"])
        if record and record["kind"] in ("start", "stop", "disable"):
            kind, phase = record["kind"], record["phase"]
            if phase not in _TERMINAL:
                action = {"start": "使能", "stop": "STOP 停稳", "disable": "失能"}[kind]
                uncertain = self.clock()-record["since"] >= self.CONFIRM_SECONDS
                title = f"{action}结果待确认" if uncertain else f"{action}请求待发送" if phase == "queued" else f"{action}确认中"
                reason = "尚未收到明确设备确认；不能据此判断操作完成。" if uncertain else "等待主控回执，尚未确认完成。"
                if kind == "stop" and not uncertain:
                    origin = {"ui": "界面控制通道", "gamepad": "手柄侧"}.get(record["source"])
                    if origin:
                        reason = origin + "请求 STOP；等待主控确认停稳。"
                    stop_reason = c.get("gamepad", {}).get("last_stop", {}).get("reason") if c.get("gamepad", {}).get("last_stop") else None
                    if record["source"] == "gamepad" and stop_reason:
                        reason += "原因：" + stop_reason
                return self._view(kind, "uncertain" if uncertain else phase, title, reason,
                                  "检查连接和设备状态；不要据此认为机械臂已停稳。" if uncertain else "等待设备确认。",
                                  "warning" if uncertain else "info", record)
            if phase == "rejected":
                return self._view(kind, phase, "主控未受理" + {"start": "使能", "stop": "STOP", "disable": "失能"}[kind],
                                  record["detail"], "等待当前任务结束并核查状态后重试；不会自动重发。", "warning", record)
        if c.get("stop_locked"):
            self._invalidate_position_observations()
            confirmed = c.get("stop_confirmed", False)
            return self._view("stop", "locked" if confirmed else "waiting_confirmation",
                              "STOP 锁定" if confirmed else "停止状态待确认",
                              "主控已确认 STOP 锁定。" if confirmed else "尚未收到停稳或失能确认；锁定原因待确认。",
                              "检查实物和锁定原因；需要恢复时再 START 或 DISABLE。" if confirmed else "等待明确回执并检查连接。", "warning", record,
                              **({"elapsed_s": 0.} if confirmed else {}))
        rgb = c.get("rgb_save", {})
        if rgb.get("active"):
            uncertain = rgb.get("stage") == "uncertain"
            return self._view("rgb_save", "uncertain" if uncertain else "waiting", "RGB 保存结果待确认" if uncertain else "RGB 配置保存中",
                              rgb.get("message", "等待设备确认。"), "断开重连后核实；不会自动重发保存。" if uncertain else "等待保存结果，后台查询暂时暂停。",
                              "warning" if uncertain else "info", since=rgb.get("started_at"))
        if c.get("calibration_busy"):
            return self._view("calibration", "waiting", "校准进行中", c.get("status", "等待主控校准结果。"), "等待校准、失能和保存的最终确认。")
        if c.get("cartesian_busy"):
            return self._view("diagnostic", "waiting", "诊断或模式恢复中", c.get("cartesian_message", "等待现有任务结束。"), "等待任务完成；需要中断时使用 STOP。")
        gamepad = c.get("gamepad", {})
        if gamepad.get("claimed"):
            stage = gamepad.get("stage")
            if stage == "init_confirmation":
                return self._view("gamepad", "manual_confirmation", "等待人工确认到位", gamepad.get("message", "初始化仍未解锁。"),
                                  "观察实物到位后点击“确认到位”，随后将摇杆和扳机回中。", "warning", record)
            if stage in ("centering", "target_rejected", "enabled", "rested", "connected", "disabled", "protected", "fault", "stopping", "disabling"):
                title = {"centering": "等待手柄回中", "target_rejected": "手柄目标未受理", "enabled": "手柄已使能，等待初始化", "rested": "休息完成，等待重新初始化"}.get(stage, "手柄控制")
                return self._view("gamepad", "waiting", title, gamepad.get("message", ""),
                                  "将摇杆和扳机回中。" if stage == "centering" else "按手柄面板说明完成当前步骤。", "warning" if stage in ("target_rejected", "fault", "protected") else "info", record)
        if record and record["kind"] in _NAMES:
            kind, phase = record["kind"], record["phase"]
            name = _NAMES[kind] + (f" #{record['motion_id']}" if record["motion_id"] is not None else "")
            if self._position_matches(record, c):
                return self._view("motion", "observed_reached", name + " · 回读位置已到目标",
                                  "七轴回读已进入目标位置容差。",
                                  "可以继续编辑或发送下一目标。", "success", record, motion_kind=kind)
            titles = {"queued": name + (" 已排队" if record["motion_id"] is not None else " 待发送"), "sending": name + " 正在发送",
                      "waiting": name + " 等待受理", "accepted": name + " 已受理 · 完成未确认", "executing": name + " 执行中",
                      "waiting_confirmation": name + " 等待完成确认", "completed": name + " 已完成", "rejected": name + " 未完成", "cancelled": name + " 已取消", "uncertain": name + " 状态待确认"}
            unconfirmed = phase in ("sending", "waiting") or phase == "queued" and record["motion_id"] is None
            if unconfirmed and self.clock()-record["since"] >= self.CONFIRM_SECONDS:
                phase = "uncertain"
            reason = {"queued": "主控已分配编号，可能仍在等待前一动作。" if record["motion_id"] else "指令尚未确认写入串口。",
                      "sending": "已进入串口写入阶段，尚未确认设备受理。", "waiting": "串口写入完成，等待主控编号或回执。",
                      "accepted": "主控已受理，尚未确认到位。",
                      "executing": "已有主控运行状态回报。", "waiting_confirmation": "等待主控完成确认，不按模型重合判断完成。",
                      "completed": "已收到与本次动作类型和编号匹配的完成回执。",
                      "rejected": record["detail"], "cancelled": "主控报告本动作取消；这不等于停稳确认。",
                      "uncertain": "回执未能明确关联或尚未收到确认；不能判断动作完成。"}[phase]
            step = "可以继续编辑或发送下一目标。" if phase == "completed" else "检查原因后重试；需要停止时使用 STOP。" if phase in ("rejected", "cancelled", "uncertain") else "等待对应完成回执；需要停止时使用 STOP。"
            return self._view("motion", phase, titles[phase], reason, step, "success" if phase == "completed" else "warning" if phase in ("rejected", "cancelled", "uncertain") else "info", record, motion_kind=kind)
        if c.get("motor_enabled") is True:
            return self._view("connection", "idle", "已使能 · 待命", "主控已确认使能。", "可以编辑并发送运动目标。", "success")
        if c.get("motor_enabled") is False:
            return self._view("connection", "idle", "已失能", "主控已确认电机失能。", "可以编辑目标；执行运动前点击 START。")
        return self._view("connection", "uncertain", "电机状态未知", "已连接，尚未收到使能或失能确认。", "核查设备状态；不会自动使能。", "warning")
