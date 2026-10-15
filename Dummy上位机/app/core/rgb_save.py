"""Exclusive RGB flash save, without retrying an ambiguous write."""
import threading
import time


class RGBSaveSession:
    PREPARE_SECONDS = 3.0
    CONFIRM_SECONDS = 6.0
    QUIET_SECONDS = .15
    SAFETY_COMMANDS = ("!STOP", "!DISABLE")

    def __init__(self, controller):
        self.c = controller
        self.lock = threading.RLock()
        self.timer = None
        self.token = 0
        self.stage = "idle"
        self.message = ""
        self.command = None
        self.admitted_at = None

    @property
    def active(self):
        return self.stage != "idle"

    @staticmethod
    def is_save(command):
        tokens = command.split()
        return bool(tokens and (tokens[0] == "!RGB_SAVE" or
                    (tokens[0].startswith("!RGB_") and tokens[-1] == "&")))

    def reset(self):
        with self.lock:
            self.token += 1
            if self.timer is not None:
                self.c.root.after_cancel(self.timer)
            self.timer = None
            self.stage, self.message, self.command = "idle", "", None
            self.admitted_at = None

    def _notify(self):
        self.c._emit_rgb()
        self.c._publish()

    def begin(self, command):
        with self.lock:
            if self.active:
                return False
            self.token += 1
            self.connection = self.c._connection_id
            self.generation = self.c._tx_generation
            self.command = command
            self.admitted_at = None
            self.started_at = self.last_activity = time.monotonic()
            self.stage = "preparing"
            self.message = "等待已有回读结束，随后保存 RGB；后台查询已暂停。"
            self.result = None
            self._schedule()
        self.c.log(self.message)
        self._notify()
        return True

    def _schedule(self):
        self.timer = self.c.root.after(50, self._tick, self.token)

    def allows_write(self, generation, command):
        # Called by the sole writer BEFORE its admission ledger is updated.
        with self.lock:
            if not self.active or command in self.SAFETY_COMMANDS:
                return True
            if self.stage == "preparing":
                return not self.is_save(command)  # Drain preceding FIFO edits/queries.
            return (self.stage == "waiting" and self.admitted_at is None and
                    generation == self.generation and command == self.command)

    def admitted(self, generation, command):
        with self.lock:
            if not self.is_save(command):
                return self.allows_write(generation, command)
            if (self.stage == "waiting" and self.admitted_at is None and
                    generation == self.generation and command == self.command):
                # Install BEFORE port.write: an immediate RX may beat the Qt TX signal.
                self.admitted_at = time.monotonic()
                return True
            return False

    def ready_to_write(self, generation, command):
        with self.lock:
            return not (command in self.SAFETY_COMMANDS and self.admitted_at is not None and
                        self.stage in ("waiting", "uncertain"))

    def activity(self, line=None):
        with self.lock:
            if not self.active:
                return
            self.last_activity = time.monotonic()
            if self.admitted_at is None or self.stage == "settling":
                return
            if line == "ok RGB saved":
                self.result = (True, "RGB 配置已保存。")
            elif line is not None and line.startswith("error RGB "):
                self.result = (False, f"RGB 保存未完成：{line[10:]}")
            else:
                return
            # Consume the status/configuration tail before resuming polling.
            self.stage = "settling"
            self.message = self.result[1]

    def cancel_unwritten(self):
        with self.lock:
            if self.active and self.admitted_at is None:
                self._finish(False, "RGB 保存尚未发送，已取消。")

    def _finish(self, success, message):
        self.reset()
        self.message = message
        self.c.log(message, "INFO" if success else "WARN")
        self._notify()

    def _tick(self, token):
        with self.lock:
            if token != self.token or not self.active:
                return
            self.timer = None
            if not self.c.is_connected or self.connection != self.c._connection_id:
                self.reset()
                return
            now = time.monotonic()
            if self.stage == "preparing":
                c = self.c
                transport = c._transport
                activity = max(self.last_activity, getattr(transport, "last_activity_at", 0))
                drained = (c._tx_queue.empty() and c._wire_admitted.empty() and
                           not getattr(transport, "pending_writes", 0) and
                           not getattr(transport, "write_in_progress", False) and
                           c._position_request is None and c._rgb_get_pending_until is None and
                           c._rgb_runtime_pending is None and
                           not c._rgb_full_active and now - activity >= self.QUIET_SECONDS)
                if drained:
                    if (not c._motor_state_known or c._motor_enabled or c._stop_locked or
                            c._calibration_busy or c._seq_active or c._acc_sync_waiting or
                            c._parameter_request is not None or c.cartesian.busy or
                            c.gamepad.claimed or self.generation != c._tx_generation):
                        self._finish(False, "设备状态已改变，RGB 保存未发送；请确认失能后重试。")
                        return
                    self.stage = "waiting"
                    self.started_at = now
                    self.message = "正在保存 RGB，等待设备确认；后台查询已暂停。"
                    c._tx_queue.put((self.generation, self.command))
                    self._notify()
                elif now - self.started_at >= self.PREPARE_SECONDS:
                    self._finish(False, "已有回读未结束，RGB 保存未发送；请稍后重试。")
                    return
            elif self.stage == "settling":
                if now - self.last_activity >= self.QUIET_SECONDS:
                    self._finish(*self.result)
                    return
            elif self.stage == "waiting" and now - self.started_at >= self.CONFIRM_SECONDS:
                self.stage = "uncertain"
                self.message = "6 秒内未确认 RGB 保存结果；查询保持暂停，请断开重连核实。不会自动重发保存。"
                self.c.log(self.message, "WARN")
                self._notify()
            # Uncertain saves remain fenced; a late explicit receipt can resolve them.
            self._schedule()
