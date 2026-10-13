"""UI ownership and feedback for native firmware or timed joint curves.

The ordinary MoveJ approximation backend has been removed. IK/model algorithms
remain in dummy_motion; only StreamSession may execute a PC-planned trajectory.
"""
import threading
import time
import re
import numpy as np
from PySide6.QtCore import QObject, QTimer
from dummy_motion import RobotModel, FeedbackSample, Status
from dummy_motion.feedback import FeedbackTracker
from dummy_motion.frames import from_wire_joints
from app.core.cartesian_backend import CartesianBackend


class CartesianSession(QObject):
    def __init__(self, controller, clock=time.perf_counter):
        super().__init__(controller)
        self.controller, self.clock = controller, clock
        self.backend = CartesianBackend.FIRMWARE
        self.preferred_mode = 2
        self.model = RobotModel()
        self.feedback = FeedbackTracker(self.model)
        self.motor_acceleration = np.full(7, 30.)
        self.acceleration_confirmed = np.zeros(7, dtype=bool)
        self.cancel_event = threading.Event()
        self.epoch = 0
        self.active = False
        self.stage, self.message = 'idle', '主控规划（默认）'
        self.pending_backend = None
        self.job = None
        self.sequence = False
        self.retarget = None
        self.kind = 'stream'
        self._timer = QTimer(self)  # No GUI timer clocks or executes motion.
        from app.core.stream_session import StreamSession
        self.stream = StreamSession(self)

    @property
    def busy(self):
        return self.active or self.pending_backend is not None

    def _state(self, stage, message):
        self.stage, self.message = stage, message
        self.controller._emit_state()

    def switch(self, backend):
        try:
            backend = CartesianBackend(backend)
        except ValueError:
            self.controller.log('普通 MoveJ 模拟后端已移除；请选择主控或曲线透传', 'WARN')
            return False
        mode = 7 if backend == CartesianBackend.STREAM else 2
        return self.select_mode(mode)

    def select_mode(self, mode):
        if type(mode) is not int or mode not in (1, 2, 3, 4, 7):
            self.controller.log('支持主控模式 1～4 和曲线透传模式 7；5/6 保留', 'WARN')
            return False
        backend = CartesianBackend.STREAM if mode == 7 else CartesianBackend.FIRMWARE
        c = self.controller
        if (self.busy or c.gamepad.claimed or c._seq_active or c._mode_pending is not None or
                c._mode_request is not None or c._parameter_request is not None or
                c._calibration_busy or c._acc_sync_waiting):
            c.log('请结束当前任务后切换模式；STOP 始终可用', 'WARN')
            return False
        if c._stop_locked and not getattr(c, '_stop_confirmed', False):
            c.log('请等待 STOP 停稳确认，再关闭会话或切换模式', 'WARN')
            return False
        if c._stop_locked and mode != 2:
            c.log('STOP 锁定时仅允许关闭透传并恢复主控 INT；运行前仍需 START', 'WARN')
            return False
        if not c.is_connected:
            self.preferred_mode = mode
            self.backend = backend
            self._state('idle', '主控规划' if mode != 7 else '50 Hz 曲线段透传 · 离线规划')
            return True
        if self._has_queued_control():
            c.log('控制队列尚未清空，请稍后切换', 'WARN')
            return False
        self._begin('stream')
        self.pending_backend = backend
        self.sequence = False
        self._state('switching', '确认停稳 → 关闭旧会话 → 切换主控模式')
        return self.stream.start_switch(backend, mode)

    def _available(self):
        c = self.controller
        if (not c.is_connected or not c._motor_state_known or not c._motor_enabled or
                c._stop_locked or c._calibration_busy or c._acc_sync_waiting or
                c._parameter_request is not None or not c._poll_enabled):
            c.log("上位机逆解需要已连接、已使能、无保护任务，并开启关节回读", "WARN")
            return False
        if self.feedback.estimate(self.clock())[0] != Status.OK:
            c.log("上位机逆解需要 300 ms 内的新关节回读", "WARN")
            c._poll_position()
            return False
        return True

    def _has_queued_control(self):
        # GETJPOS is read-only even if RX already retired its request before
        # the queued command_written callback clears the admitted ledger.
        # Keep GETJPOS out of the global diagnostic whitelist: STOP/calibration
        # and parameter transactions must retain their existing polling gates.
        from app.protocol import capabilities
        c = self.controller
        for pending in (c._tx_queue, c._wire_admitted):
            with pending.mutex:
                items = [item[:2] for item in pending.queue if item is not None]
            if any(not capabilities.diagnostic(command) and command != "#GETJPOS"
                   for generation, command in items):
                return True
        return False

    def start_points(self, points, speed=None, sequence=False):
        if self.backend != CartesianBackend.STREAM:
            self.controller.log('PC 轨迹仅允许从曲线透传后端执行', 'WARN')
            return False
        return self.stream.start_points(points, speed, sequence)

    def _begin(self, kind):
        self.cancel_event.set()
        self.cancel_event = threading.Event()
        self.epoch += 1
        self.kind, self.active = kind, True
        self.generation = self.controller._tx_generation
        self.connection = self.controller._connection_id
        self.job = None
        self.retarget = None

    def update_feedback(self, values, received_at=None):
        try:
            now = self.clock()
            sample = FeedbackSample(from_wire_joints(values), now if received_at is None else received_at)
            if sample.received_at > now:
                self.feedback.rejection_reason = "回读时间戳晚于当前单调时钟"
                return False
            return self.feedback.update(sample)
        except ValueError as exc:
            self.feedback.rejection_reason = f"回读值无效：{exc}"
            return False

    def can_write(self, generation, command):
        # StreamSession owns and admits every planning/mode transaction itself.
        return False

    def command_written(self, command, generation, connection):
        pass

    def invalidate_acceleration(self, command):
        if command.startswith(('#ACC', '!SYNC_ACC')):
            self.acceleration_confirmed[:] = False

    def handle_line(self, line):
        match = re.fullmatch(r'ok ACC \[([1-7])\] = ([0-9.eE+\-]+) r/s\^2(?: \(rail\))?', line)
        if match and np.isfinite(float(match[2])) and float(match[2]) > 0:
            self.motor_acceleration[int(match[1])-1] = float(match[2])
            self.acceleration_confirmed[int(match[1])-1] = True

    def cancel_remaining(self):
        if self.active and self.sequence:
            self.stream.cancel_tail = True
            self.retarget = None

    def fail(self, message, stop=True):
        had_motion = self.stream.active and getattr(self.stream, 'opened', False)
        self.cancel()
        self._state('error', message)
        self.controller.log(message, 'WARN')
        if stop and had_motion and self.controller.is_connected:
            self.controller.send_cmd('!STOP', source='cartesian')

    def cancel(self):
        self.stream.cancel()
        self.cancel_event.set()
        self.epoch += 1
        self.retarget = None
        self.active = False
        self.pending_backend = None
        self.job = None
        if self.sequence:
            self.controller._seq_active = self.controller._seq_running = False
        self.stage = 'idle'

    def close(self):
        self.stream.close()
        self.cancel()
        self._timer.stop()
