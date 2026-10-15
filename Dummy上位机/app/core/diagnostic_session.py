"""Bounded diagnostic USB worker; no PC-planned motion execution.

Only SerialTransport writes the port. This worker authorizes bounded commands
under the controller's existing connection/generation gates and receives actual
RX timestamps directly. GUI notifications cannot advance device credits.
"""
from collections import Counter, deque
import queue
import re
import threading
import time
import numpy as np
from PySide6.QtCore import QObject, Signal, Qt, QTimer
from dummy_motion.frames import from_wire_joints
from dummy_motion.stream_protocol import (parse_stream_line, STREAM_MODE, STREAM_MASK, STREAM_VERSION,
                                          MOTOR_SCALE, MOTOR_ZERO, robot_coordinates, motor_coordinates, fault_description)
from app.protocol.parser import finite_numbers


from app.core.open_test import OpenTestMixin, format_open_test
from app.core.axis_communication_test import AxisCommunicationTestMixin, format_axis_test


class DiagnosticSession(AxisCommunicationTestMixin, OpenTestMixin, QObject):
    notification = Signal(int, str, object)
    open_test_changed = Signal(object)
    axis_test_changed = Signal(object)

    def __init__(self, parent):
        super().__init__(parent)
        self.owner, self.controller = parent, parent.controller
        self.notification.connect(self._notification, Qt.ConnectionType.QueuedConnection)
        self.lock = threading.RLock()
        self.cancel_event = threading.Event()
        self.thread = None
        self.active = False
        self.authorized = Counter()
        self.admitted = Counter()
        self.discarded = Counter()
        self.inbox = queue.Queue(maxsize=256)
        self.last_session = 0
        self.session_history = deque(maxlen=32)
        self.serial_next = 0


    def _prepare_worker(self, require_poll):
        o, c = self.owner, self.controller
        # receive() runs directly on the transport RX thread. Publish active
        # only after every field it may inspect belongs to this new worker.
        self.active = False
        self.require_poll = require_poll
        self.query_stream = require_poll
        self.epoch = o.epoch
        self.generation, self.connection = c._tx_generation, c._connection_id
        self.discarded = Counter({key: value for key, value in self.discarded.items() if key[0] == self.generation})
        self.cancel_event = threading.Event()
        self.authorized = Counter()
        self.inbox = queue.Queue(maxsize=256)
        self.fault = None
        self.primary_fault = None
        self.latest_feedback = ((o.feedback.sample.q.copy(), o.feedback.sample.received_at)
                                if o.feedback.sample is not None else (np.zeros(7), 0.))
        self.acceleration = np.zeros(7)
        self.motor_velocity = np.zeros(7)
        self.diagnostics = {}
        self.caps = self.state = None
        self.features = None
        self.observed_state = None
        self.mode_reply = None
        self.mode_set = None
        self.session = 0
        self.opened = self.started = False
        self.start_requested_at = 0.
        self.running_confirmed = False
        self.running_published = False
        self.closing = self.cancel_tail = False
        self.serial_next = 0
        self.accepted = -1
        self.reported_free = 0
        self.inflight = {}
        self.done_sequence = None
        self.last_status = time.perf_counter()
        self.last_poll = self.last_query = 0.
        self.last_progress = 0.
        self.poll_period = 1/max(30, min(50, c._poll_hz))
        self.request = c.allocate_command_mode_request()
        self.stop_confirmed = False
        self.recovering = False
        self.active = True

    def start_switch(self, backend, mode):
        # Only the facade may establish a native mode-recovery owner. Mode 7
        # is reserved for the constant-reference axis diagnostic itself.
        if (getattr(backend, 'value', backend) != 'firmware' or
                type(mode) is not int or mode not in (1, 2, 3, 4) or
                self.owner.kind != 'mode_switch' or not self.owner.active):
            self.controller.log('诊断恢复只允许已建立的主控模式事务', 'WARN')
            return False
        self.target_mode = mode
        if self.thread is not None and self.thread.is_alive():
            self.owner.fail('上一轮透传正在退出，请稍后切换', stop=False)
            return False
        self._prepare_worker(require_poll=False)
        self.thread = threading.Thread(target=self._switch_run, args=(backend, mode), name='Dummy close/mode switch', daemon=True)
        self.thread.start()
        return True

    def _notify(self, kind, value):
        self.notification.emit(self.epoch, kind, value)

    def _notification(self, epoch, kind, value):
        self._deliver_notification(epoch, kind, value, self.thread)

    def _deliver_notification(self, epoch, kind, value, worker):
        o, c = self.owner, self.controller
        if epoch != o.epoch:
            return
        if (kind in ('backend', 'fault', 'open_test_done', 'axis_test_done') and
                worker is not None and worker.is_alive()):
            # A queued Qt receipt can overtake the worker's finally/return.
            # Keep ownership until that exact worker exits; never advertise
            # ready while another diagnostic/mode worker must still be rejected.
            # STOP/disconnect advance the epoch and invalidate this retry.
            QTimer.singleShot(5, lambda: self._deliver_notification(epoch, kind, value, worker))
            return
        if kind in ('open_test', 'open_test_done'):
            self.open_test_result = value
            self.open_test_changed.emit(value)
            c.log(format_open_test(value), 'INFO' if value.get('passed', not value.get('done')) else 'WARN')
            if kind == 'open_test_done':
                o.cancel()
                o._state('complete' if value.get('passed') else 'error',
                         f"J{value['node']} OPEN→CLOSE：{value['stage']}")
        elif kind in ('axis_test', 'axis_test_done'):
            self.axis_test_display = dict(value)
            self.axis_test_changed.emit(dict(value))
            if kind == 'axis_test_done':
                c.log(format_axis_test(value), 'INFO' if value.get('passed') else 'WARN')
                o.cancel()
                o._state('complete' if value.get('passed') else 'error', '原地曲线通信测试：'+value['stage'])
        elif kind == 'state':
            o._state(*value)
        elif kind == 'mode':
            c.current_cmdmode = value
            c._emit_state()
        elif kind == 'log':
            c.log(value)
        elif kind == 'backend':
            o.cancel()
            backend, mode = value
            o.backend = backend
            o.preferred_mode = mode
            c.current_cmdmode = mode
            o._state('idle', '主控规划')
        elif kind == 'fault':
            o.cancel()
            o._state('error', value)

    def cancel(self):
        if self.active and self.owner.kind == 'axis_test':
            result = dict(getattr(self, 'axis_result', {}))
            result.update(stage='已中断', message='连接或任务已取消；CLOSE / 模式恢复结果未知，本次不判为通过。', done=True, passed=False)
            self.axis_result = result
            self.axis_test_changed.emit(result)
        if self.active and self.owner.kind == 'open_test':
            result = dict(getattr(self, 'open_test_result', {}))
            result.update(stage='已中断', message='测试被取消或连接中断；CLOSE 结果以主控再次查询为准，不能判为通过。', done=True, passed=False)
            self.open_test_result = result
            self.open_test_changed.emit(result)
        with self.lock:
            self.cancel_event.set()
            self.active = False
            self.discarded.update(self.authorized)
            self.authorized.clear()

    def owns_command(self, generation, command):
        """None means ordinary controller traffic; bool handles owned traffic."""
        key = (generation, command)
        with self.lock:
            if self.discarded[key]:
                self.discarded[key] -= 1
                if not self.discarded[key]:
                    del self.discarded[key]
                return False
            if not self.authorized[key]:
                # A cancelled stream packet must not become an ordinary command.
                if command.startswith(('~S', '!STREAM_')):
                    return False
                return None
            self.authorized[key] -= 1
            if not self.authorized[key]:
                del self.authorized[key]
            if (not self.active or self.cancel_event.is_set() or generation != self.generation or
                    generation != self.controller._tx_generation or self.connection != self.controller._connection_id or
                    not self.controller.is_connected or (self.controller._stop_locked and not self._locked_command(command))):
                return False
            if self.recovering and not self._locked_command(command):
                return False
            try:
                if not self._command_allowed(command):
                    return False
            except ValueError:
                return False
            self.admitted[key] += 1
            return True

    @staticmethod
    def _locked_command(command):
        # Read/status/CLOSE are required to retire a stopped session. No OPEN,
        # seed, START or motion command is admitted while protection is locked.
        return command.startswith(('#STREAM', '#GET_CMDMODE ', '!STREAM_CLOSE ')) or command in ('!STOP', '#CMDMODE 2')

    def _discard_curve_commands(self):
        # Called under self.lock, also directly on the transport RX thread.
        # Preserve recovery queries while retiring every motion authorization.
        for key in tuple(self.authorized):
            if not self._locked_command(key[1]) or key[1] == '#CMDMODE 7':
                self.discarded[key] += self.authorized.pop(key)

    def written(self, command, generation, connection):
        key = (generation, command)
        with self.lock:
            if not self.admitted[key]:
                return False
            self.admitted[key] -= 1
            if not self.admitted[key]:
                del self.admitted[key]
        if self.owner.kind == 'open_test' and command.startswith('!STREAM_TEST '):
            self._notify('log', '单电机测试已写入串口：'+command+'（只握手、自动关闭）')
        return True  # Avoid per-segment Qt receipts and its unrelated GUI ledger.

    def receive(self, line, connection, received_at):
        if not self.active or connection != self.connection:
            return False
        nums = finite_numbers(line[3:], 7) if line.startswith('ok ') else None
        if nums is not None:
            q = from_wire_joints(nums)
            if self.owner.kind == 'axis_test':
                with self.lock:
                    previous = self.axis_feedback[1]
                    if received_at >= previous:
                        if previous:
                            self.axis_rx_gap = max(self.axis_rx_gap, received_at-previous)
                        self.axis_feedback = (q.copy(), received_at)
                        for node, held in self.axis_result.get('held', {}).items():
                            reference = held/MOTOR_SCALE[node-1]+MOTOR_ZERO[node-1]
                            deviation = abs(q[node-1]-reference)
                            self.axis_deviation[node] = max(self.axis_deviation.get(node, 0.), deviation)
            if self.owner.model.valid(q):
                with self.lock:
                    if received_at >= self.latest_feedback[1]:
                        self.latest_feedback = (q, received_at)
            return False  # GUI still gets pose display data with the real stamp.
        suppress = line.startswith('STREAM_')
        if self.owner.kind == 'axis_test' and line in ('Stopping ok', 'STOP locked', 'STOP locked disabled', 'Disabled ok', 'error STOP_LOCKED'):
            with self.lock:
                self.recovering = True
                self._discard_curve_commands()
                if not self.axis_cleanup:
                    self.axis_result.setdefault('failure_stage', self.axis_result.get('stage', '检查主控'))
                    self.axis_error = self.axis_error or '主控进入 STOP / 失能状态，通信测试中断'
            if line == 'error STOP_LOCKED':
                # A duplicate STOP can be rejected while the first STOP is
                # decelerating. It is not a failure of read/CLOSE recovery.
                return False
        if line in ('STOP locked', 'STOP locked disabled', 'Disabled ok'):
            self.stop_confirmed = True
            if self.recovering:
                return False
        try:
            item = parse_stream_line(line)
            if item is None:
                m = re.fullmatch(r'ok CMDMODE mode=(\d+) busy=([01]) issued=(\d+) req=(\d+)', line)
                if m:
                    item = dict(kind='mode', mode=int(m[1]), busy=int(m[2]), request=int(m[4]))
                m = re.fullmatch(r'ok CMDMODE mode=(\d+)', line)
                if m:
                    item = dict(kind='mode_set', mode=int(m[1]))
                m = re.fullmatch(r'ok ACC \[([1-7])\] = ([0-9.eE+\-]+) r/s\^2(?: \(rail\))?', line)
                if m and np.isfinite(float(m[2])) and float(m[2]) > 0:
                    item = dict(kind='acc', axis=int(m[1])-1, value=float(m[2]))
                if (line.startswith(('error CMDMODE', 'error DISABLED', 'error STOP_LOCKED', 'COMM ', 'STALL ')) or
                        line in ('STOP locked', 'Disabled ok')):
                    item = dict(kind='fatal', message=line)
            if item is not None:
                if ((item['kind'] == 'fault' or
                        item['kind'] == 'state' and item.get('state') == 'FAULT') and
                        self.session and item.get('session') == self.session):
                    with self.lock:
                        self.recovering = True
                        self._discard_curve_commands()
                        if item['kind'] == 'fault' and self.primary_fault is None:
                            stage = (self.axis_result.get('failure_stage', self.axis_result.get('stage', '检查主控'))
                                     if self.owner.kind == 'axis_test' else self.owner.stage)
                            self.primary_fault = dict(session=self.session, code=item['code'], stage=stage,
                                                      received_at=received_at,
                                                      message='主控透传故障：'+fault_description(item['code']))
                        if self.owner.kind == 'axis_test' and self.primary_fault:
                            self.axis_result.update(code=self.primary_fault['code'],
                                                    primary_fault=dict(self.primary_fault),
                                                    failure_stage=self.primary_fault['stage'])
                            self.axis_error = self.primary_fault['message']
                if item['kind'] == 'rejected' and item.get('busy') and line.startswith('error STREAM_DIAG'):
                    # An accepted query is retried on CAN by firmware. The
                    # selected-axis scheduler retries only explicit mailbox
                    # BUSY receipts; other bounded diagnostic reads retain
                    # their existing timeout/retry correlator.
                    if self.owner.kind == 'axis_test' and self.axis_health_enabled:
                        item = dict(kind='diag_busy')
                    else:
                        return suppress
                item['received_at'] = received_at
                self.inbox.put_nowait(item)
        except (ValueError, queue.Full) as exc:
            with self.lock:
                self.fault = str(exc) or '透传回执缓冲溢出'
        return suppress

    def _valid(self):
        c = self.controller
        return (not self.cancel_event.is_set() and self.active and
                self.generation == c._tx_generation and self.connection == c._connection_id and c.is_connected)

    def _command_allowed(self, command):
        if self.owner.kind == 'axis_test':
            return self._axis_command_allowed(command)
        if self.owner.kind == 'open_test':
            return self._open_test_command_allowed(command)
        if self.owner.kind == 'mode_switch':
            return (command in ('#STREAM', '!STOP', f'#GET_CMDMODE {self.request}',
                                f'#CMDMODE {self.target_mode}') or
                    self.session != 0 and command == f'!STREAM_CLOSE {self.session}')
        return False

    def _send(self, command):
        if not self._valid():
            raise InterruptedError('Stream cancelled')
        if not command.isascii() or len(command) > 256 or any(ch in command for ch in '\r\n\0'):
            raise ValueError('Invalid bounded stream command')
        if self.owner.kind == 'open_test' and not self._open_test_command_allowed(command):
            raise ValueError('单电机测试禁止发送运动、START 或模式切换指令')
        if self.owner.kind == 'axis_test' and not self._axis_command_allowed(command):
            raise ValueError('原地通信测试禁止普通运动、使能 START、逆解或改变保持位置')
        if self.recovering and not self._locked_command(command):
            if self.owner.kind == 'axis_test' and self.axis_abort:
                raise InterruptedError('用户 STOP / DISABLE 已中断通信测试')
            raise RuntimeError(self.primary_fault['message'] if self.primary_fault else '透传已经停止，禁止继续发送曲线')
        key = (self.generation, command)
        with self.lock:
            if not self._command_allowed(command):
                raise ValueError('指令不属于当前诊断 / 主控模式恢复事务')
            if sum(self.authorized.values()) >= 32:
                raise RuntimeError('透传发送积压超过 32 条')
            self.authorized[key] += 1
        self.controller._tx_queue.put(key)

    def _drain(self):
        while True:
            try:
                item = self.inbox.get_nowait()
            except queue.Empty:
                return
            kind = item['kind']
            if self.owner.kind == 'axis_test':
                self._axis_drain(item)
                continue
            if kind == 'state':
                self.observed_state = item
            if kind in ('fatal', 'rejected'):
                raise RuntimeError(item.get('message', str(item)))
            if kind == 'test':
                self.test_reply = item
            elif kind == 'mode' and item['request'] == self.request:
                self.mode_reply = item
            elif kind == 'mode_set':
                self.mode_set = item['mode']
            elif kind == 'acc':
                self.acceleration[item['axis']] = item['value']
            elif kind == 'node':
                self.diagnostics[(item['node'], item['field'])] = item
            elif kind == 'cap':
                self.caps = item
            elif kind == 'features':
                if self.session and item['session'] == self.session:
                    self.features = item
            elif (self.session != 0 and item.get('session') == self.session) or (self.closing and kind == 'state' and item.get('session') == 0):
                self.last_status = item['received_at']
                if kind == 'fault':
                    if not self.closing:
                        raise RuntimeError('主控透传故障：' + fault_description(item['code']))
                if kind == 'state':
                    if item['state'] == 'FAULT' and not self.closing:
                        raise RuntimeError('主控透传状态为 FAULT')
                    if item['next'] > self.serial_next:
                        raise RuntimeError('主控返回未发送的透传序号')
                    self.state = item
                    if (self.started and item['state'] in ('RUNNING', 'DRAINING') and
                            item['received_at'] >= self.start_requested_at):
                        self.running_confirmed = True
                    self._accept(item['next']-1, item['free'])
                elif kind == 'ack':
                    if item['sequence'] >= self.serial_next:
                        raise RuntimeError('主控确认未发送的透传序号')
                    self._accept(item['sequence'], item['free'])
                elif kind == 'done':
                    self.done_sequence = item['sequence']

    def _accept(self, sequence, free):
        if sequence < self.accepted:
            return
        self.accepted, self.reported_free = sequence, free
        self.inflight = {seq: stamp for seq, stamp in self.inflight.items() if seq > sequence}

    def _pump(self, *, check_feedback=False):
        if not self._valid():
            raise InterruptedError('Stream cancelled')
        if self.require_poll and not self.controller._poll_enabled:
            raise RuntimeError('关节回读被关闭，停止透传')
        self._drain()
        if self.fault:
            raise RuntimeError(self.fault)
        now = time.perf_counter()
        if self.require_poll and now-self.last_poll >= self.poll_period:
            self._send('#GETJPOS')
            self.last_poll = now
        if self.query_stream and now-self.last_query >= .05:
            self._send('#STREAM')
            self.last_query = now
        if check_feedback and now-self.latest_feedback[1] > .3:
            raise RuntimeError(f'关节回读实际 RX 超过 300 ms（{(now-self.latest_feedback[1])*1000:.0f} ms），停止透传')
        if self.inflight and now-min(self.inflight.values()) > .5:
            raise RuntimeError('透传序号确认超过 500 ms')
        if self.opened and now-self.last_status > .5:
            raise RuntimeError('透传状态超过 500 ms 未更新')

    def _wait(self, condition, seconds, message, check_feedback=False):
        end = time.perf_counter()+seconds
        while True:
            self._pump(check_feedback=check_feedback)
            if condition():
                return
            if time.perf_counter() >= end:
                raise RuntimeError(message)
            self.cancel_event.wait(.004)

    def _read_diagnostic(self, node, field):
        for attempt in range(3):
            sent_at = time.perf_counter()
            self._send(f'#STREAM_DIAG {node} {field}')
            try:
                self._wait(lambda: (node, field) in self.diagnostics and
                           self.diagnostics[(node, field)]['received_at'] >= sent_at,
                           1.3, f'J{node} 诊断字段 {field} 未回读')
                return self.diagnostics[(node, field)]['value']
            except RuntimeError as exc:
                if '未回读' not in str(exc) or attempt == 2:
                    raise




    def _cleanup_existing_session(self, check_feedback):
        self._wait(lambda: self.observed_state is not None, 2., '透传状态查询超时', check_feedback=check_feedback)
        observed = self.observed_state
        if not observed['session']:
            return
        if observed['state'] not in ('IDLE', 'FAULT', 'OPEN'):
            raise RuntimeError('主控存在另一条未结束的透传会话；请 STOP 后重试')
        self.session = observed['session']
        if self.session not in self.session_history:
            self.session_history.append(self.session)
        self.serial_next = observed['next']
        self.accepted = observed['next']-1
        self.closing = True
        self.opened = True
        self.last_status = time.perf_counter()
        sent_at = time.perf_counter()
        self._send(f'!STREAM_CLOSE {self.session}')
        self._wait(lambda: self.state is not None and self.state['state'] == 'IDLE' and
                   self.state['session'] == 0 and self.state['received_at'] >= sent_at,
                   2., '残留透传会话未确认关闭；未开始新运动', check_feedback=check_feedback)
        self.opened = self.closing = False
        self.session = 0

    def _switch_run(self, backend, mode):
        try:
            self._send(f'#GET_CMDMODE {self.request}')
            self._wait(lambda: self.mode_reply is not None, 3., '主控模式查询超时')
            if self.mode_reply['busy']:
                raise RuntimeError('主控仍在运动；请先 STOP 并等待停止确认')
            if self.controller._stop_locked and not getattr(self.controller, '_stop_confirmed', False):
                raise RuntimeError('尚未收到 STOP 停稳确认，不能关闭会话或切换模式')
            if self.mode_reply['mode'] == STREAM_MODE:
                self.query_stream = True
                self._send('#STREAM')
                self.last_query = time.perf_counter()
                self._cleanup_existing_session(check_feedback=False)
            if self.mode_reply['mode'] != mode:
                self._send(f'#CMDMODE {mode}')
                self._wait(lambda: self.mode_set == mode, 3., '主控未确认目标模式')
            self._notify('backend', (backend, mode))
        except InterruptedError:
            pass
        except Exception as exc:
            message = f'模式切换未完成：{exc}'
            self.controller.log(message, 'WARN')
            self._notify('fault', message)
        finally:
            with self.lock:
                self.active = False
                self.discarded.update(self.authorized)
                self.authorized.clear()


    def close(self):
        self.cancel()
        if self.thread is not None and self.thread is not threading.current_thread():
            self.thread.join(timeout=.2)
