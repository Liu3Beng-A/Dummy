"""Native firmware mode ownership, FK/frames, and isolated diagnostics.

The application does not execute PC IK or timed point trajectories. Pure
offline algorithms remain in dummy_motion and have no live command owner.
"""
import threading
import time
import re
import numpy as np
from PySide6.QtCore import QObject
from dummy_motion import RobotModel, FeedbackSample
from dummy_motion.feedback import FeedbackTracker
from dummy_motion.frames import from_wire_joints
from app.core.cartesian_backend import CartesianBackend
from app.core.diagnostic_session import DiagnosticSession


class CartesianSession(QObject):
    def __init__(self, controller, clock=time.perf_counter):
        super().__init__(controller)
        self.controller, self.clock = controller, clock
        self.backend = CartesianBackend.FIRMWARE
        self.native_tcp = False
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
        self.retarget = None  # Compatibility with edit cancellation; never executed.
        self.kind = 'idle'
        self.diagnostics = DiagnosticSession(self)

    @property
    def stream(self):
        """Old diagnostic callers only; this object has no point-planning API."""
        return self.diagnostics

    @property
    def busy(self):
        return self.active or self.pending_backend is not None

    def _state(self, stage, message):
        self.stage, self.message = stage, message
        self.controller._emit_state()

    def switch(self, backend):
        if getattr(backend, 'value', backend) != 'firmware':
            self.controller.log('上位机运动执行后端已撤下；普通运动仅使用主控', 'WARN')
            return False
        return self.select_mode(2)

    def select_mode(self, mode):
        if type(mode) is not int or mode not in (1, 2, 3, 4):
            self.controller.log('普通运动仅开放主控模式 1～4；模式 7 仅用于独立诊断', 'WARN')
            return False
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
            c.log('STOP 锁定时仅允许关闭诊断透传并恢复主控 INT；运行前仍需 START', 'WARN')
            return False
        if not c.is_connected:
            self.preferred_mode = mode
            self.backend = CartesianBackend.FIRMWARE
            self._state('idle', '主控规划')
            return True
        if self._has_queued_control():
            c.log('控制队列尚未清空，请稍后切换', 'WARN')
            return False
        self._begin('mode_switch')
        self.pending_backend = CartesianBackend.FIRMWARE
        self._state('switching', '确认停稳 → 关闭旧诊断会话 → 切换主控模式')
        return self.diagnostics.start_switch(CartesianBackend.FIRMWARE, mode)

    def _has_queued_control(self):
        # Readonly GETJPOS keeps its original parameter/protection gates.
        from app.protocol import capabilities
        c = self.controller
        for pending in (c._tx_queue, c._wire_admitted):
            with pending.mutex:
                items = [item[:2] for item in pending.queue if item is not None]
            if any(not capabilities.diagnostic(command) and command != '#GETJPOS'
                   for generation, command in items):
                return True
        return False

    def _begin(self, kind):
        if kind not in ('mode_switch', 'open_test', 'axis_test'):
            raise ValueError('Only native-mode recovery or explicit diagnostics may own this session')
        self.cancel_event.set()
        self.cancel_event = threading.Event()
        self.epoch += 1
        self.kind, self.active = kind, True
        self.generation = self.controller._tx_generation
        self.connection = self.controller._connection_id
        self.job = self.retarget = None

    def update_feedback(self, values, received_at=None):
        try:
            now = self.clock()
            sample = FeedbackSample(from_wire_joints(values), now if received_at is None else received_at)
            if sample.received_at > now:
                self.feedback.rejection_reason = '回读时间戳晚于当前单调时钟'
                return False
            return self.feedback.update(sample)
        except ValueError as exc:
            self.feedback.rejection_reason = f'回读值无效：{exc}'
            return False

    def can_write(self, generation, command):
        return False  # DiagnosticSession admits only its own bounded commands.

    def command_written(self, command, generation, connection):
        pass

    def invalidate_acceleration(self, command):
        if command.startswith(('#ACC', '!SYNC_ACC')):
            self.acceleration_confirmed[:] = False

    def handle_line(self, line):
        if line == "KIN v=2 xyz=mm abc=deg joints=deg rail=mm" and not self.native_tcp:
            self.native_tcp = True
            gamepad = getattr(self.controller, "gamepad", None)
            if gamepad is not None and gamepad.claimed:
                gamepad.release("当前固件支持单个笛卡尔目标；连续操控暂不可用", source="firmware-capability", request_stop=True)
            if not self.busy:
                self._state("idle", "主控有限TCP目标；连续操控暂不可用")
            else:
                self.controller._emit_state()
        match = re.fullmatch(r'ok ACC \[([1-7])\] = ([0-9.eE+\-]+) r/s\^2(?: \(rail\))?', line)
        if match and np.isfinite(float(match[2])) and float(match[2]) > 0:
            self.motor_acceleration[int(match[1])-1] = float(match[2])
            self.acceleration_confirmed[int(match[1])-1] = True

    def fail(self, message, stop=True):
        had_motion = self.diagnostics.active and getattr(self.diagnostics, 'opened', False)
        self.cancel()
        self._state('error', message)
        self.controller.log(message, 'WARN')
        if stop and had_motion and self.controller.is_connected:
            self.controller.send_cmd('!STOP', source='cartesian')

    def cancel(self):
        self.diagnostics.cancel()
        self.cancel_event.set()
        self.epoch += 1
        self.retarget = None
        self.active = False
        self.pending_backend = self.job = None
        self.stage = 'idle'

    def close(self):
        self.diagnostics.close()
        self.cancel()
