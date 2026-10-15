"""Bounded selected-axis held-reference communication exercise, independent of IK."""
import re
import secrets
import threading
import time
import numpy as np

from app.core.open_test import format_entry_snapshot, feature_supported, entry_field_allowed
from dummy_motion.stream_protocol import (StreamKnot, STREAM_SEED, STREAM_END,
                                         STREAM_VERSION, STREAM_MODE, MOTOR_SCALE,
                                         MOTOR_ZERO, decode_knot, fault_description)

AXIS_TEST_MASKS = (1, 2, 4, 8, 16, 32, 64, 65, 127)
AXIS_TEST_SEGMENTS = 100


def axis_test_nodes(mask):
    return tuple(node for node in range(1, 8) if mask & (1 << (node-1)))


def held_test_knots(mask, held):
    """Keep motor int32 references verbatim; inactive triples are zero."""
    if type(mask) is not int or mask not in AXIS_TEST_MASKS:
        raise ValueError('通信测试支持 J1～J7 单轴、J1+J7 或七轴整组')
    motor = np.zeros((7, 3), dtype=np.int64)
    for node in axis_test_nodes(mask):
        value = held[node]
        if type(value) is not int or not -2147483648 <= value <= 2147483647:
            raise ValueError(f'J{node} 保持参考超出 int32')
        motor[node-1, 0] = value
    return tuple(StreamKnot(seq, motor, STREAM_SEED if seq == 0 else
                            STREAM_END if seq == AXIS_TEST_SEGMENTS else 0)
                 for seq in range(AXIS_TEST_SEGMENTS+1))


def format_axis_test(result):
    nodes = '+'.join(f'J{node}' for node in axis_test_nodes(result.get('mask', 0))) or '未选择'
    lines = [f"{nodes} · {result.get('stage', '尚未测试')} · 会话 {result.get('session', 0) or '尚未创建'}",
             result.get('message', ''),
             '保持锁存目标，不下发位移；正常位置误差补偿仍可能有微动。',
             f"已发送 {result.get('sent', 0)}/101，已接受序号 {result.get('accepted', -1)}，"
             f"已执行序号 {result.get('executed', -1)}",
             f"CLOSE {'已确认' if result.get('closed') else '未确认'} / "
             f"模式恢复 {'已确认' if result.get('restored') else '未确认'}"
             + (f"（模式 {result['original_mode']}）" if 'original_mode' in result else '')]
    if 'current_mode' in result:
        lines.append(f"当前确认模式 {result['current_mode']} / 进入前模式 {result.get('original_mode', '未知')}")
    if 'failure_stage' in result:
        lines.append('故障发生阶段：'+result['failure_stage'])
    if result.get('primary_fault'):
        primary = result['primary_fault']
        lines.append(f"原始故障（会话 {primary['session']}，阶段 {primary['stage']}）：{fault_description(primary['code'])}")
    if 'max_rx_gap_ms' in result:
        lines.append(f"被测轴实际 RX 最大间隔 {result['max_rx_gap_ms']:.1f} ms（与界面刷新独立）")
    for node, deviation in sorted(result.get('max_actual_deviation', {}).items()):
        value = np.rad2deg(deviation) if node <= 6 else deviation*1000
        lines.append(f"J{node} 实际位置相对保持参考的最大偏差 {value:.4f} {'°' if node <= 6 else 'mm'}")
    for node, snapshot in sorted(result.get('snapshots', {}).items()):
        lines.append(f'J{node} 以下是 OPEN 准入快照；曲线 / START / 执行故障需结合发生阶段判断。')
        lines.append(format_entry_snapshot(node, result.get('session'), snapshot,
                                           steady_supported=node in result.get('steady_nodes', ())))
    for node, error in sorted(result.get('snapshot_errors', {}).items()):
        lines.append(f'J{node} 拒绝快照未确认：{error}')
    for node, snapshot in sorted(result.get('start_snapshots', {}).items()):
        lines.append(format_entry_snapshot(node, result.get('session'), snapshot, phase='START',
                                           steady_supported=node in result.get('steady_nodes', ())))
    for node, error in sorted(result.get('start_snapshot_errors', {}).items()):
        lines.append(f'J{node} START 快照未确认：{error}；OPEN 快照不能证明 START 准入。')
    lines.extend(result.get('contexts', ()))
    return '\n'.join(line for line in lines if line)


class AxisCommunicationTestMixin:
    def start_axis_test(self, mask=1):
        o, c = self.owner, self.controller
        reason = None
        if type(mask) is not int or mask not in AXIS_TEST_MASKS:
            reason = '当前支持 J1～J7 单轴、J1+J7 或七轴整组'
        elif not c.is_connected or c.connection_type != 'USB':
            reason = '请连接 USB 虚拟串口'
        elif not c._motor_state_known or not c._motor_enabled:
            reason = '请先人工使能并等待 Started ok；测试不会自动 !START'
        elif c._stop_locked or c._calibration_busy:
            reason = '保护 / 校准锁定中，不能开始曲线通信测试'
        elif (o.busy or c.gamepad.claimed or c._seq_active or c._mode_pending is not None or
              c._mode_request is not None or c._parameter_request is not None or c._acc_sync_waiting):
            reason = '控制器有其他任务，请等待结束'
        elif o._has_queued_control() or (self.thread is not None and self.thread.is_alive()):
            reason = '控制发送队列或上一轮工作线程尚未退出'
        if reason:
            result = dict(mask=mask if type(mask) is int else 0, stage='未开始', message=reason,
                          done=True, passed=False)
            self.axis_test_changed.emit(result)
            c.log('原地曲线通信测试未开始：'+reason, 'WARN')
            return False
        o._begin('axis_test')
        self.axis_mask = mask
        self.axis_nodes = axis_test_nodes(mask)
        self.axis_feedback = (np.zeros(7), 0.)
        self.axis_error = None
        self.axis_cap = None
        self.axis_cleanup = False
        self.axis_abort = False
        self.axis_contexts = []
        self.axis_executed = -1
        self.axis_rx_gap = 0.
        self.axis_deviation = {}
        self.axis_health_enabled = False
        self.axis_health_pending = None
        self.axis_health_at = {}
        self.axis_health_cursor = 0
        self.axis_health_next = 0.
        self.axis_result = dict(mask=mask, stage='检查主控', message='2 s 原地 q/v/a 曲线；锁存被测轴保持参考，目标位置不变。',
                                sent=0, accepted=-1, executed=-1, closed=False, restored=False, done=False)
        # Activation exposes receive() to the transport RX thread immediately.
        # Build every selected-axis attribute before setting self.active.
        self._prepare_worker(require_poll=False)
        self.axis_test_changed.emit(dict(self.axis_result))
        o._state('checking', '原地曲线通信测试：检查主控 / 被测电机')
        self.thread = threading.Thread(target=self._axis_test_run, name='Dummy selected-axis communication test', daemon=True)
        self.thread.start()
        return True

    def _axis_publish(self, stage=None, message=None, **extra):
        if stage is not None:
            self.axis_result['stage'] = stage
        if message is not None:
            self.axis_result['message'] = message
        self.axis_result.update(sent=self.serial_next, accepted=self.accepted, executed=self.axis_executed,
                                contexts=tuple(self.axis_contexts), max_rx_gap_ms=self.axis_rx_gap*1000,
                                max_actual_deviation=dict(self.axis_deviation), **extra)
        if not self.axis_result.get('done'):
            self._notify('axis_test', dict(self.axis_result))

    def _axis_command_allowed(self, command):
        if command in ('#STREAM', '#STREAM_AXES', '#GETJPOS', f'#GET_CMDMODE {self.request}', '!STOP'):
            return True
        if command == f'!STREAM_CLOSE {self.session}':
            return bool(self.session)
        if command == '#CMDMODE 7':
            return not (self.axis_cleanup or self.axis_abort or self.recovering)
        if command == '#CMDMODE 2':
            return self.axis_cleanup and bool(self.axis_result.get('closed'))
        if self.axis_cleanup or self.axis_abort or self.recovering:
            match = re.fullmatch(r'#STREAM_DIAG ([1-7]) (\d+)', command)
            return bool(match and int(match[1]) in self.axis_nodes and
                        entry_field_allowed(int(match[2]), self.features, self.session, int(match[1])))
        if command in (f'!STREAM_OPEN {self.session} {self.axis_mask}', f'!STREAM_START {self.session}'):
            return bool(self.session)
        if command.startswith('#STREAM_DIAG '):
            match = re.fullmatch(r'#STREAM_DIAG ([1-7]) (\d+)', command)
            return bool(match and int(match[1]) in self.axis_nodes and
                        entry_field_allowed(int(match[2]), self.features, self.session, int(match[1])))
        if command.startswith('~S'):
            token, knot = decode_knot(command)
            return (token == self.session and 0 <= knot.sequence <= AXIS_TEST_SEGMENTS and
                    np.array_equal(knot.motor, self.axis_knots[knot.sequence].motor) and
                    knot.flags == self.axis_knots[knot.sequence].flags)
        return False

    def _axis_drain(self, item):
        """A fault retires the stream; its latched diagnostics remain readable."""
        kind = item['kind']
        if kind in ('fatal', 'rejected'):
            if self.axis_cleanup:
                raise RuntimeError(item.get('message', str(item)))
            self.axis_error = self.axis_error or item.get('message', str(item))
        elif kind == 'axis_cap':
            self.axis_cap = item
        elif kind == 'mode' and item['request'] == self.request:
            self.mode_reply = item
        elif kind == 'mode_set':
            self.mode_set = item['mode']
        elif kind == 'node':
            self.diagnostics[(item['node'], item['field'])] = item
            pending = self.axis_health_pending
            if (pending is not None and item['node'] == pending['node'] and item['field'] == 7 and
                    item['received_at'] >= pending['sent_at'] and
                    0 < (item['nonce']-pending['nonce']) % 65536 < 32768):
                self.axis_health_at[item['node']] = item['received_at']
                self.axis_health_pending = None
                self.axis_health_cursor = (self.axis_health_cursor+1) % len(self.axis_nodes)
                # Aim for a 150 ms group cycle, measured from the request.
                # A reply slower than its slot advances the next node at once
                # rather than adding another slot of avoidable wait per axis.
                self.axis_health_next = pending['sent_at']+max(.02, .15/len(self.axis_nodes))
                if item['value'] != self.axis_result['held'][item['node']]:
                    self.axis_error = self.axis_error or f"J{item['node']} 原地执行期间保持参考改变"
        elif kind == 'diag_busy':
            if self.axis_health_pending is not None:
                self.axis_health_pending['retry_at'] = item['received_at']+.025
                self.axis_health_pending['send_needed'] = True
        elif kind == 'cap':
            self.caps = item
        elif kind == 'features':
            if self.session and item['session'] == self.session:
                self.features = item
                self.axis_result['steady_nodes'] = tuple(node for node in self.axis_nodes
                                                        if feature_supported(item, self.session, node, 'steady'))
        elif kind == 'diag':
            if self.session and not self.closing:
                self.axis_executed = item['executed']
        elif kind == 'fault_context':
            if f's={self.session} ' in item['message'] and item['message'] not in self.axis_contexts:
                self.axis_contexts.append(item['message'])
        elif kind == 'fault' and self.session and item.get('session') == self.session:
            primary = self.primary_fault
            self.axis_result['code'] = primary['code'] if primary else item['code']
            self.axis_error = primary['message'] if primary else '主控透传故障：'+fault_description(item['code'])
        elif kind == 'state':
            self.observed_state = item
            if ((self.session and item['session'] == self.session) or
                    (self.closing and item['session'] == 0)):
                self.last_status = item['received_at']
                self.state = item
                if (self.started and item['state'] in ('RUNNING', 'DRAINING') and
                        item['received_at'] >= self.start_requested_at):
                    self.running_confirmed = True
                    if not self.running_published and not self.axis_cleanup:
                        self.running_published = True
                        self._axis_publish('执行原地曲线', '主控已确认同步启动；被测电机执行原地曲线，目标位置保持不变。')
                if item['next'] > self.serial_next:
                    self.axis_error = self.axis_error or '主控返回未发送的透传序号'
                else:
                    self._accept(item['next']-1, item['free'])
                if item['state'] == 'FAULT':
                    self.axis_error = self.axis_error or '主控透传状态为 FAULT'
        elif kind == 'ack' and self.session and item.get('session') == self.session:
            if item['sequence'] >= self.serial_next:
                self.axis_error = self.axis_error or '主控确认未发送的透传序号'
            else:
                self._accept(item['sequence'], item['free'])
        elif kind == 'done' and self.session and item.get('session') == self.session:
            self.done_sequence = item['sequence']

    def _axis_pump(self, check_actual=False):
        self._pump(check_feedback=False)
        if self.axis_abort and not self.axis_cleanup:
            raise RuntimeError('用户 STOP / DISABLE 已中断通信测试')
        if self.axis_error and not self.axis_cleanup:
            raise RuntimeError(self.axis_error)
        now = time.perf_counter()
        if not self.axis_cleanup and now-self.last_poll >= .02:
            self._send('#GETJPOS')
            self.last_poll = now
        if check_actual and now-self.axis_feedback[1] > .3:
            raise RuntimeError('被测轴关节回读实际 RX 超过 300 ms；通信测试停止')
        if check_actual:
            for node, held in self.axis_result.get('held', {}).items():
                reference = held/MOTOR_SCALE[node-1]+MOTOR_ZERO[node-1]
                tolerance = np.deg2rad(.3) if node <= 6 else .0005
                if abs(self.axis_feedback[0][node-1]-reference) > tolerance:
                    raise RuntimeError(f'J{node} 原地保持位置偏差超限，通信测试停止')
            if self.axis_health_enabled:
                self._axis_health(now)

    def _axis_health(self, now):
        # One firmware diagnostic mailbox is shared by every selected axis.
        # Advance only after its correlated reply; checking ALL ages on every
        # pump detects a lost late-axis reply without starving earlier axes.
        for node in self.axis_nodes:
            if now-self.axis_health_at[node] > .6:
                raise RuntimeError(f'J{node} 电机 CAN 诊断回读超过 600 ms')
        pending = self.axis_health_pending
        if pending is None and now >= self.axis_health_next:
            node = self.axis_nodes[self.axis_health_cursor]
            pending = dict(node=node, sent_at=now, send_needed=True,
                           nonce=self.diagnostics[(node, 7)]['nonce'], retry_at=now)
            self.axis_health_pending = pending
        if pending is not None and pending['send_needed'] and now >= pending['retry_at']:
            self._send(f"#STREAM_DIAG {pending['node']} 7")
            # Core already retries its accepted CAN query every 20 ms. Only
            # retry an explicit mailbox BUSY rejection, never an unanswered
            # accepted query: duplicate queries can occupy the next axis slot.
            pending['send_needed'] = False

    def _axis_wait(self, condition, seconds, message, check_actual=False):
        end = time.perf_counter()+seconds
        while True:
            self._axis_pump(check_actual)
            if condition():
                return
            if time.perf_counter() >= end:
                raise RuntimeError(message)
            self.cancel_event.wait(.004)

    def _axis_diag(self, node, field, check_actual=False):
        # The common diagnostic correlator already retries mailbox-busy and
        # checks the actual RX stamp. It must use this test's health pump.
        for attempt in range(3):
            sent_at = time.perf_counter()
            self._send(f'#STREAM_DIAG {node} {field}')
            try:
                self._axis_wait(lambda: (node, field) in self.diagnostics and
                                self.diagnostics[(node, field)]['received_at'] >= sent_at,
                                1.3, f'J{node} 诊断字段 {field} 未回读', check_actual=check_actual)
                return self.diagnostics[(node, field)]['value']
            except RuntimeError as exc:
                if '未回读' not in str(exc) or attempt == 2:
                    raise

    def _axis_snapshot(self, node, phase='OPEN'):
        first_field, last_field = (23, 33) if phase == 'OPEN' else (34, 44)
        first = self._axis_diag(node, first_field)
        if first != self.session:
            raise RuntimeError(f'{phase} 快照会话 {first} 与本次 {self.session} 不匹配，未使用旧快照')
        snapshot = {first_field: first}
        for field in range(first_field+1, last_field+1):
            snapshot[field] = self._axis_diag(node, field)
        if feature_supported(self.features, self.session, node, 'steady'):
            metric = 45 if phase == 'OPEN' else 48
            for field in range(metric, metric+3):
                snapshot[field] = self._axis_diag(node, field)
        if self._axis_diag(node, first_field) != first:
            raise RuntimeError(f'读取期间 {phase} 快照变化')
        return snapshot

    def request_axis_stop(self):
        """An external STOP cancels queued curves but retains cleanup ownership."""
        with self.lock:
            self.axis_abort = True
            self.axis_result.setdefault('failure_stage', self.axis_result.get('stage', '检查主控'))
            self.recovering = True
            self.discarded.update(self.authorized)
            self.authorized.clear()

    def adopt_axis_generation(self):
        with self.lock:
            self.generation = self.controller._tx_generation

    def _axis_retire(self, failure=False):
        self.axis_cleanup = self.recovering = True
        self.axis_health_enabled = False
        self.axis_health_pending = None
        # Preserve the original parser/overflow error in the final result,
        # while allowing the stopped session's bounded cleanup queries.
        self.fault = None
        self.inflight.clear()
        with self.lock:
            self.discarded.update(self.authorized)
            self.authorized.clear()
        if self.opened:
            if failure:
                self._axis_publish('等待停稳', '停止曲线发送；等待 STOP 确认，再读取拒绝快照和关闭会话。')
                if not self.stop_confirmed and not self.controller._stop_locked:
                    self._send('!STOP')
                self._axis_wait(lambda: self.stop_confirmed or getattr(self.controller, '_stop_confirmed', False),
                                3., 'STOP 未确认停稳；未尝试 CLOSE / 模式恢复')
            if failure:
                snapshots, errors, start_snapshots, start_errors = {}, {}, {}, {}
                self._axis_publish('读取拒绝现场', '读取被测电机 OPEN 瞬间快照，按本次会话校验。')
                for node in self.axis_nodes:
                    self._axis_publish('读取拒绝现场', f'读取 J{node} OPEN 瞬间快照，按本次会话校验。')
                    try:
                        snapshots[node] = self._axis_snapshot(node)
                    except (RuntimeError, ValueError) as exc:
                        errors[node] = str(exc)
                    if not self.started:
                        start_errors[node] = '本次尚未发送 STREAM_START，没有本次 START 决策'
                    elif not feature_supported(self.features, self.session, node, 'start'):
                        start_errors[node] = '电机独立 START 快照能力未确认（旧固件或缺少本会话能力回包），未查询新增字段'
                    else:
                        self._axis_publish('读取拒绝现场', f'读取 J{node} 独立 START 决策快照，按本次会话校验。')
                        try:
                            start_snapshots[node] = self._axis_snapshot(node, 'START')
                        except (RuntimeError, ValueError) as exc:
                            start_errors[node] = str(exc)
                self.axis_result.update(snapshots=snapshots, snapshot_errors=errors,
                                        start_snapshots=start_snapshots, start_snapshot_errors=start_errors)
            self._axis_publish('等待 CLOSE', '等待被测轴会话关闭确认。')
            self.closing = True
            self.state = None
            self.last_status = time.perf_counter()
            closed_at = time.perf_counter()
            self._send(f'!STREAM_CLOSE {self.session}')
            self._axis_wait(lambda: self.state is not None and self.state['session'] == 0 and
                            self.state['state'] == 'IDLE' and self.state['received_at'] >= closed_at,
                            2., 'CLOSE 未确认；未恢复模式，不能继续下一轮测试')
            self.opened = self.started = False
            self.axis_result['closed'] = True
        if self.axis_result.get('original_mode') == 2 and self.axis_result.get('mode_changed'):
            self._axis_publish('恢复主控模式', 'CLOSE 已确认，恢复进入前的模式 2；STOP 锁定不会自动解锁。')
            self.mode_set = None
            self._send('#CMDMODE 2')
            self._axis_wait(lambda: self.mode_set == 2, 3., '主控未确认恢复模式 2')
            self._notify('mode', 2)
            self.axis_result['current_mode'] = 2
        self.axis_result['restored'] = 'original_mode' in self.axis_result

    def _axis_test_run(self):
        failure = None
        try:
            self._send(f'#GET_CMDMODE {self.request}')
            self._axis_wait(lambda: self.mode_reply is not None, 3., '主控模式查询未回复')
            if self.mode_reply['busy']:
                raise RuntimeError('主控仍在运动；未发送 OPEN')
            mode = self.mode_reply['mode']
            if mode not in (2, 7):
                raise RuntimeError('本阶段请先选主控 INT 模式 2（或模式 7）；未改变模式 / 未发送 OPEN')
            self.axis_result['original_mode'] = mode
            self.axis_result['current_mode'] = mode
            self._send('#STREAM_AXES')
            self._axis_wait(lambda: self.axis_cap is not None, 2., '主控未响应定向原地测试能力；请更新本次主控固件')
            if ((self.axis_cap['version'], self.axis_cap['hz'], self.axis_cap['targeted']) != (1, 50, 1) or
                    self.axis_cap['mask'] not in (65, 127)):
                raise RuntimeError('主控未确认定向原地曲线能力；未切换模式 / 未发送 OPEN')
            if self.axis_mask & ~self.axis_cap['mask']:
                raise RuntimeError('当前主控仅支持 J1 / J7 / J1+J7；其他单轴及七轴整组需要更新主控，未切换模式 / 未发送 OPEN')
            self.query_stream = True
            self._send('#STREAM')
            self.last_query = time.perf_counter()
            self._axis_wait(lambda: self.caps is not None and self.observed_state is not None,
                            2., '主控未响应 v2 透传查询')
            if (self.caps['version'], self.caps['hz'], self.caps['mode']) != (STREAM_VERSION, 50, STREAM_MODE):
                raise RuntimeError('主控能力不是 v2 / 50 Hz / 模式 7；未发送 OPEN')
            if self.observed_state['session']:
                raise RuntimeError('已有透传会话未关闭；未接管 / 未发送新 OPEN')
            self.axis_result['closed'] = True
            if mode != 7:
                self.axis_result['mode_changed'] = True
                self._send('#CMDMODE 7')
                self._axis_wait(lambda: self.mode_set == 7, 3., '主控未确认模式 7')
                self._notify('mode', 7)
                self.axis_result['current_mode'] = 7
            held = {node: self._axis_diag(node, 7) for node in self.axis_nodes}
            self.axis_knots = held_test_knots(self.axis_mask, held)
            self._axis_wait(lambda: time.perf_counter()-self.axis_feedback[1] <= .3,
                            3., '没有被测轴新回读；未发送 OPEN')
            q = self.axis_feedback[0]
            for node in self.axis_nodes:
                reference = held[node]/MOTOR_SCALE[node-1]+MOTOR_ZERO[node-1]
                tolerance = np.deg2rad(.3) if node <= 6 else .0005
                if abs(q[node-1]-reference) > tolerance:
                    raise RuntimeError(f'J{node} 实际位置未到保持参考；未发送 OPEN')
            self.session = secrets.randbelow(65535)+1
            while self.session in self.session_history:
                self.session = self.session % 65535+1
            self.session_history.append(self.session)
            self.last_session = self.session
            self.axis_result.update(session=self.session, held=dict(held))
            self.state = None
            self.last_status = time.perf_counter()
            self.opened = True
            self.axis_result['closed'] = False
            self._axis_publish('等待 OPEN', '只打开被测节点；锁存位置原样用作 SEED。')
            self._send(f'!STREAM_OPEN {self.session} {self.axis_mask}')
            self._axis_wait(lambda: self.state is not None and self.state['state'] == 'OPEN',
                            3., '被测轴 OPEN 未确认', check_actual=True)
            self._axis_wait(lambda: self.caps is not None and self.caps['mask'] == self.axis_mask,
                            1., '被测轴能力 / 掩码确认不一致', check_actual=True)
            for node in self.axis_nodes:
                if self._axis_diag(node, 7, check_actual=True) != held[node]:
                    raise RuntimeError(f'J{node} OPEN 前后保持参考改变；未发送 SEED')
            self._axis_publish('预填原地曲线', '发送精确 SEED 与 100 个 20 ms 原地段；速度 / 加速度均为零。')
            self.axis_health_at = {node: self.diagnostics[(node, 7)]['received_at'] for node in self.axis_nodes}
            self.axis_health_cursor = 0
            self.axis_health_next = time.perf_counter()
            self.axis_health_pending = None
            self.axis_health_enabled = True
            end = time.perf_counter()+8.
            while self.done_sequence is None:
                self._axis_pump(check_actual=True)
                if time.perf_counter() >= end:
                    raise RuntimeError('原地曲线执行 / DONE 超时')
                credit = self.reported_free-len(self.inflight)
                while self.serial_next < len(self.axis_knots) and credit > 0 and len(self.inflight) < 4:
                    knot = self.axis_knots[self.serial_next]
                    self._send(knot.encode(self.session))
                    self.inflight[self.serial_next] = time.perf_counter()
                    self.serial_next += 1
                    credit -= 1
                if not self.started and self.state and self.state['state'] == 'ARMED':
                    self.start_requested_at = time.perf_counter()
                    self._axis_publish('等待同步启动', '已请求 STREAM_START，等待主控确认同步启动；此时尚不能认定电机开始执行。')
                    self._send(f'!STREAM_START {self.session}')
                    self.started = True
                now = time.perf_counter()
                if now-self.last_progress >= .2:
                    self.last_progress = now
                    self._axis_publish()
                self.cancel_event.wait(.002)
            if not self.started or self.done_sequence != AXIS_TEST_SEGMENTS or self.accepted != AXIS_TEST_SEGMENTS:
                raise RuntimeError('DONE / 已接受序号与最终序号 100 不一致')
            self.axis_executed = self.done_sequence
            self._axis_publish('核对原地保持', 'DONE 已确认，核对所选轴实际位置新鲜、偏差和 150 ms 稳定窗口。')
            settled_at, previous_q, previous_stamp = None, None, None
            settle_deadline = time.perf_counter()+2.
            selected = np.array(self.axis_nodes)-1
            stable_tolerance = np.array([np.deg2rad(.05) if node <= 6 else .0001 for node in self.axis_nodes])
            while True:
                self._axis_pump(check_actual=True)
                q, stamp = self.axis_feedback
                if stamp != previous_stamp:
                    stable = previous_q is None or np.all(abs(q[selected]-previous_q[selected]) <= stable_tolerance)
                    if not stable:
                        settled_at = None
                    elif settled_at is None:
                        settled_at = stamp
                    previous_q, previous_stamp = q, stamp
                    if settled_at is not None and stamp-settled_at >= .15:
                        break
                if time.perf_counter() >= settle_deadline:
                    raise RuntimeError('所选轴原地保持实际位置未稳定')
                self.cancel_event.wait(.004)
            self.axis_health_enabled = False
            if self.axis_health_pending is not None:
                self._axis_wait(lambda: self.axis_health_pending is None, .6,
                                '最终保持参考核对前 CAN 诊断请求未完成', check_actual=True)
            self._axis_publish('核对最终保持参考', '逐轴读取最终保持参考，同时检查所选轴实际 RX 新鲜度和偏差。')
            for node in self.axis_nodes:
                if self._axis_diag(node, 7, check_actual=True) != held[node]:
                    raise RuntimeError(f'J{node} 原地曲线结束保持参考改变')
            self._axis_pump(check_actual=True)
            self._axis_retire()
            self._axis_publish('通过', 'SEED→缓冲→同步 START→执行 100 段→DONE→CLOSE→模式恢复均已确认。',
                               done=True, passed=True)
        except InterruptedError:
            if (self.axis_abort and not self.cancel_event.is_set() and self.controller.is_connected and
                    self.connection == self.controller._connection_id):
                self.adopt_axis_generation()
                try:
                    self._axis_retire(failure=True)
                    self._axis_publish('已中断', '用户停止，CLOSE 与模式恢复已确认；本次不判为通过。', done=True, passed=False)
                except Exception as cleanup:
                    self._axis_publish('已中断', '用户停止；恢复未完成：'+str(cleanup), done=True, passed=False)
            else:
                return
        except Exception as exc:
            failure = str(exc)
            self.axis_result.setdefault('failure_stage', self.axis_result.get('stage', '检查主控'))
            cleanup_error = None
            if self._valid():
                try:
                    self._axis_retire(failure=True)
                except Exception as cleanup:
                    cleanup_error = str(cleanup)
            if self.axis_abort:
                failure = '用户 STOP / DISABLE 已中断通信测试'
            elif self.primary_fault:
                failure = self.primary_fault['message']
            if cleanup_error:
                failure += '；恢复未完成：'+cleanup_error
            self._axis_publish('已中断' if self.axis_abort else '未通过', failure, done=True, passed=False)
        finally:
            with self.lock:
                # This is the completion point shared with request_axis_stop.
                # A user stop while cleanup is in progress cannot turn into a
                # PASS, even if CLOSE and mode restoration already succeeded.
                if self.axis_abort and self.axis_result.get('done'):
                    if self.axis_result.get('passed') or self.axis_result.get('stage') != '已中断':
                        self.axis_result['message'] = '用户 STOP / DISABLE 已中断通信测试；已确认的关闭 / 模式恢复结果见下方。'
                    self.axis_result.update(stage='已中断', passed=False)
                self.active = False
                self.discarded.update(self.authorized)
                self.authorized.clear()
                final_result = dict(self.axis_result)
        self._notify('axis_test_done', final_result)
