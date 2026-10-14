"""Explicit bounded held-reference or J3/J5 small-motion diagnostics; no IK."""
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
MOTION_TEST_SEGMENTS = 200
MOTION_TEST_DEGREES = {3: -.5, 5: .5}
TIMING_SCHEMA_FIELD = 51
TIMING_SCHEMA = 0x544d0001
TIMING_FIELDS = (2, 9, 12, 18, 19, *range(52, 64))


class TimingReadError(RuntimeError):
    """Measurement incomplete after motion completed; cleanup still required."""


def counter_delta(before, after):
    """Unsigned boot counter difference, including one uint32 wrap."""
    return (int(after)-int(before)) & 0xffffffff


def moving_test_knots(mask, held, velocity_limit, acceleration_limit):
    """Exact integer SEED; one 0.5 degree S5 excursion and exact return."""
    if type(mask) is not int or mask not in (4, 16):
        raise ValueError('非零曲线测试仅支持 J3 或 J5 单轴')
    node = 3 if mask == 4 else 5
    seed = held[node]
    if type(seed) is not int or not -2147483648 <= seed <= 2147483647:
        raise ValueError('保持参考超出 int32')
    if (type(velocity_limit) is not int or type(acceleration_limit) is not int or
            not 0 < velocity_limit <= 1536000 or not 0 < acceleration_limit <= 5120000):
        raise ValueError('电机实际速度 / 加速度限值无效')
    delta = int(np.rint(np.deg2rad(MOTION_TEST_DEGREES[node])*MOTOR_SCALE[node-1]))
    reference = seed/MOTOR_SCALE[node-1]+MOTOR_ZERO[node-1]
    target = (seed+delta)/MOTOR_SCALE[node-1]+MOTOR_ZERO[node-1]
    minimum, maximum = np.deg2rad((0., 180.) if node == 3 else (-100., 100.))
    # This explicit direction is never clipped or silently reversed near a limit.
    rounding = 1/abs(MOTOR_SCALE[node-1])
    if min(reference, target) < minimum-rounding or max(reference, target) > maximum+rounding:
        raise ValueError(f'J{node} 固定 {MOTION_TEST_DEGREES[node]:+.1f}° 往返超出硬限位；未发送 OPEN')
    limits = sorted(((minimum-MOTOR_ZERO[node-1])*MOTOR_SCALE[node-1],
                     (maximum-MOTOR_ZERO[node-1])*MOTOR_SCALE[node-1]))
    knots = []
    for sequence in range(MOTION_TEST_SEGMENTS+1):
        outward = sequence <= 100
        u = (sequence if outward else sequence-100)/100
        s = u*u*u*(10+u*(-15+6*u))
        ds = 30*u*u*(1-u)*(1-u)/2
        dds = (60*u-180*u*u+120*u*u*u)/4
        offset = delta*s if outward else delta*(1-s)
        direction = 1 if outward else -1
        motor = np.zeros((7, 3), dtype=np.int64)
        motor[node-1] = (seed+int(np.rint(offset)),
                         int(np.rint(direction*delta*ds)),
                         int(np.rint(direction*delta*dds)))
        flags = STREAM_SEED if not sequence else STREAM_END if sequence == MOTION_TEST_SEGMENTS else 0
        if sequence in (0, MOTION_TEST_SEGMENTS):
            motor[node-1] = (seed, 0, 0)
        knot = StreamKnot(sequence, motor, flags)
        if knots:
            q0, v0, a0 = map(int, knots[-1].motor[node-1])
            q1, v1, a1 = map(int, knot.motor[node-1])
            distance = q1-q0
            b = (0, 200*v0, 400*v0+a0, 50000*distance-400*v1+a1,
                 50000*distance-200*v1, 50000*distance)
            for i, value in enumerate(b):
                position = q0*50000+value
                if not (limits[0]-1)*50000 <= position <= (limits[1]+1)*50000:
                    raise ValueError(f'J{node} 曲线位置包络超出硬限位')
                if i < 5 and abs(b[i+1]-value) > velocity_limit*200:
                    raise ValueError(f'J{node} 曲线速度超过电机实际限值')
                if i < 4 and abs(b[i+2]-2*b[i+1]+value) > acceleration_limit:
                    raise ValueError(f'J{node} 曲线加速度超过电机实际限值')
        knots.append(knot)
    return tuple(knots)


def format_motion_timing(result):
    lines = []
    for node, before in sorted(result.get('timing_before', {}).items()):
        after = result.get('timing_after', {}).get(node)
        if not after:
            continue
        lines.append(f'J{node} 本轮 STREAM late 增量 {counter_delta(before[12], after[12])}，'
                     f'流回包丢弃增量 {counter_delta(before[18], after[18])}；'
                     f'曲线准备 boot 峰值 {after[2]/72000:.3f} ms，STREAM 回调 boot 峰值 {after[9]/72:.3f} µs')
        lines.append(f'J{node} 逐 tick 回调峰值：进入 {after[57]/72:.3f} / '
                     f'有效 RUNNING {after[58]/72:.3f} / 软停 {after[59]/72:.3f} µs；'
                     f'本轮有效 RUNNING 采样增量 {counter_delta(before[61], after[61])}')
        lines.append(f'J{node} 分段抽样峰值：反馈 {after[52]/72:.3f}、保护 {after[53]/72:.3f}、'
                     f'驱动 {after[54]/72:.3f}、参考 {after[55]/72:.3f}、到位 {after[56]/72:.3f} µs；'
                     f'采样次数进入/运行/软停 {after[60]}/{after[61]}/{after[62]}，除数 {after[63]}')
    if lines:
        lines.append('计时为本次上电累计峰值；分段为抽样观察值，不能相加或视作 WCET。计数增量仅覆盖前后只读查询间隔。')
    if result.get('performance_warning'):
        lines.append('性能待复查：'+result['performance_warning'])
    if result.get('motion_verified'):
        lines.append('协议 / 实际往返已确认；性能项：'+('通过' if result.get('performance_passed') else '未通过'))
    if result.get('timing_error'):
        lines.append('计时回读未完整确认：'+result['timing_error'])
    return lines


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
             (f"会真实移动：J{result.get('motion_node')} 固定 {result.get('motion_degrees', 0):+.1f}°，2 s 出去 + 2 s 返回；最终回到锁存参考。"
              if result.get('motion') else '保持锁存目标，不下发位移；正常位置误差补偿仍可能有微动。'),
             f"已发送 {result.get('sent', 0)}/{result.get('segments', 100)+1}，已接受序号 {result.get('accepted', -1)}，"
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
        label = '实际位置相对起点的最大位移' if result.get('motion') else '实际位置相对保持参考的最大偏差'
        lines.append(f"J{node} {label} {value:.4f} {'°' if node <= 6 else 'mm'}")
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
    if result.get('motion'):
        if 'motion_baseline_deg' in result:
            lines.append(f"启动前实际基线 {result['motion_baseline_deg']:.4f}°，RX 年龄 {result['motion_baseline_age_ms']:.1f} ms；只计 RUNNING 确认后的新样本")
        lines.append(f"正确方向实际位移峰值 {result.get('motion_actual_peak_deg', 0):.4f}°（通过要求 ≥0.15°）")
    lines.extend(format_motion_timing(result))
    lines.extend(result.get('contexts', ()))
    return '\n'.join(line for line in lines if line)


class AxisCommunicationTestMixin:
    def start_motion_axis_test(self, mask):
        return self.start_axis_test(mask, motion=True)

    def start_axis_test(self, mask=1, *, motion=False):
        o, c = self.owner, self.controller
        reason = None
        if type(motion) is not bool:
            reason = '测试类型无效'
        elif motion and (type(mask) is not int or mask not in (4, 16)):
            reason = '非零曲线测试仅支持 J3 或 J5 单轴'
        elif type(mask) is not int or mask not in AXIS_TEST_MASKS:
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
            c.log(('非零单轴曲线测试未开始：' if motion else '原地曲线通信测试未开始：')+reason, 'WARN')
            return False
        o._begin('axis_test')
        self.axis_mask = mask
        self.axis_nodes = axis_test_nodes(mask)
        self.axis_motion = motion
        self.axis_segments = MOTION_TEST_SEGMENTS if motion else AXIS_TEST_SEGMENTS
        self.axis_timing_supported = False
        self.axis_timing_after_attempted = False
        self.axis_receive_nonce = {}
        self.axis_reference_low = self.axis_reference_high = None
        self.axis_final_check = False
        self.axis_actual_peak = 0.
        self.axis_start_actual = None
        self.axis_start_stamp = 0.
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
        self.axis_result = dict(mask=mask, stage='检查主控',
                                message='4 s 单轴非零 q/v/a 往返；仅 J3/J5，会真实运动。' if motion else '2 s 原地 q/v/a 曲线；锁存被测轴保持参考，目标位置不变。',
                                motion=motion, segments=self.axis_segments,
                                sent=0, accepted=-1, executed=-1, closed=False, restored=False, done=False)
        if motion:
            node = self.axis_nodes[0]
            self.axis_result.update(motion_node=node, motion_degrees=MOTION_TEST_DEGREES[node])
        # Activation exposes receive() to the transport RX thread immediately.
        # Build every selected-axis attribute before setting self.active.
        self._prepare_worker(require_poll=False)
        self.axis_test_changed.emit(dict(self.axis_result))
        o._state('checking', ('非零单轴曲线测试' if motion else '原地曲线通信测试')+'：检查主控 / 被测电机')
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
                                max_actual_deviation=dict(self.axis_deviation),
                                motion_actual_peak_deg=np.rad2deg(self.axis_actual_peak), **extra)
        if not self.axis_result.get('done'):
            self._notify('axis_test', dict(self.axis_result))

    def _axis_field_allowed(self, node, field):
        if node not in self.axis_nodes:
            return False
        if getattr(self, 'axis_motion', False):
            if field == TIMING_SCHEMA_FIELD:
                return True
            if field in (2, 5, 6, 9, 12, 18, 19):
                return True
            if self.axis_timing_supported and 52 <= field <= 63:
                return True
        return entry_field_allowed(field, self.features, self.session, node)

    def _axis_motion_feedback_allowed(self, received_at):
        # A pre-existing reference tracking error is not observed movement.
        return (self.axis_motion and self.started and self.running_confirmed and
                not self.axis_cleanup and self.axis_start_actual is not None and
                0 < self.axis_start_stamp <= self.start_requested_at <= received_at)

    def _axis_reference_valid(self, node, value):
        seed = self.axis_result['held'][node]
        if not self.axis_motion or self.axis_final_check or not self.started:
            return value == seed
        return self.axis_reference_low-1 <= value <= self.axis_reference_high+1

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
            return bool(match and self._axis_field_allowed(int(match[1]), int(match[2])))
        if command in (f'!STREAM_OPEN {self.session} {self.axis_mask}', f'!STREAM_START {self.session}'):
            return bool(self.session)
        if command.startswith('#STREAM_DIAG '):
            match = re.fullmatch(r'#STREAM_DIAG ([1-7]) (\d+)', command)
            return bool(match and self._axis_field_allowed(int(match[1]), int(match[2])))
        if command.startswith('~S'):
            token, knot = decode_knot(command)
            return (token == self.session and 0 <= knot.sequence <= self.axis_segments and
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
            previous = self.axis_receive_nonce.get(item['node'])
            if previous is not None and not 0 < (item['nonce']-previous) % 65536 < 32768:
                return  # Duplicate/older replies cannot satisfy a fresh query.
            self.axis_receive_nonce[item['node']] = item['nonce']
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
                if not self._axis_reference_valid(item['node'], item['value']):
                    message = (f"J{item['node']} 曲线执行期间参考超出授权包络" if self.axis_motion
                               else f"J{item['node']} 原地执行期间保持参考改变")
                    self.axis_error = self.axis_error or message
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
                        self._axis_publish('执行非零往返' if self.axis_motion else '执行原地曲线',
                                           '主控已确认同步启动；仅所选单轴执行 0.5° 往返。' if self.axis_motion else '主控已确认同步启动；被测电机执行原地曲线，目标位置保持不变。')
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
                actual = self.axis_feedback[0][node-1]
                if self.axis_motion and self.started and not self.axis_final_check:
                    target = self.axis_reference_low/MOTOR_SCALE[node-1]+MOTOR_ZERO[node-1]
                    other = self.axis_reference_high/MOTOR_SCALE[node-1]+MOTOR_ZERO[node-1]
                    if not min(target, other)-tolerance <= actual <= max(target, other)+tolerance:
                        raise RuntimeError(f'J{node} 实际位置超出非零曲线授权包络，测试停止')
                elif abs(actual-reference) > tolerance:
                    raise RuntimeError(f'J{node} 最终返回位置偏差超限，测试停止' if self.axis_motion else f'J{node} 原地保持位置偏差超限，通信测试停止')
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

    def _axis_read_timing(self, *, before=False, check_actual=False):
        if not before:
            self.axis_timing_after_attempted = True
        readings = {}
        try:
            for node in self.axis_nodes:
                readings[node] = {field: self._axis_diag(node, field, check_actual=check_actual)
                                  for field in TIMING_FIELDS}
                if readings[node][63] != 64:
                    raise RuntimeError('电机计时抽样契约不匹配；未使用这些统计')
        except (RuntimeError, ValueError) as exc:
            self.axis_result['timing_error'] = str(exc)
            if (not before and not self.axis_error and not self.axis_abort and not self.recovering and
                    ('诊断字段' in str(exc) or '计时抽样契约' in str(exc))):
                raise TimingReadError(str(exc)) from exc
            raise
        self.axis_result['timing_before' if before else 'timing_after'] = readings

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
                if self.axis_motion and self.axis_result.get('timing_before') and not self.axis_timing_after_attempted:
                    try:
                        self._axis_read_timing()
                    except (RuntimeError, ValueError) as exc:
                        self.axis_result['timing_error'] = str(exc)
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
            if self.axis_motion:
                self._axis_publish('核对新版电机计时能力', '只读检查计时契约；未发送 OPEN / 运动点。')
                for node in self.axis_nodes:
                    if self._axis_diag(node, TIMING_SCHEMA_FIELD) != TIMING_SCHEMA:
                        raise RuntimeError(f'J{node} 未确认新版电机计时能力；请烧录本次固件，未发送 OPEN')
                self.axis_timing_supported = True
                self._axis_read_timing(before=True)
            if mode != 7:
                self.axis_result['mode_changed'] = True
                self._send('#CMDMODE 7')
                self._axis_wait(lambda: self.mode_set == 7, 3., '主控未确认模式 7')
                self._notify('mode', 7)
                self.axis_result['current_mode'] = 7
            held = {node: self._axis_diag(node, 7) for node in self.axis_nodes}
            if self.axis_motion:
                node = self.axis_nodes[0]
                velocity = self._axis_diag(node, 5)
                acceleration = self._axis_diag(node, 6)
                self.axis_knots = moving_test_knots(self.axis_mask, held, velocity, acceleration)
                values = [int(knot.motor[node-1, 0]) for knot in self.axis_knots]
                self.axis_reference_low, self.axis_reference_high = min(values), max(values)
            else:
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
            self._axis_publish('预填非零曲线' if self.axis_motion else '预填原地曲线',
                               '发送精确 SEED 与 200 个 20 ms 单轴往返段；其他轴不参与。' if self.axis_motion else '发送精确 SEED 与 100 个 20 ms 原地段；速度 / 加速度均为零。')
            self.axis_health_at = {node: self.diagnostics[(node, 7)]['received_at'] for node in self.axis_nodes}
            self.axis_health_cursor = 0
            self.axis_health_next = time.perf_counter()
            self.axis_health_pending = None
            self.axis_health_enabled = True
            end = time.perf_counter()+8.
            while self.done_sequence is None:
                self._axis_pump(check_actual=True)
                if time.perf_counter() >= end:
                    raise RuntimeError('非零往返曲线执行 / DONE 超时' if self.axis_motion else '原地曲线执行 / DONE 超时')
                credit = self.reported_free-len(self.inflight)
                while self.serial_next < len(self.axis_knots) and credit > 0 and len(self.inflight) < 4:
                    knot = self.axis_knots[self.serial_next]
                    self._send(knot.encode(self.session))
                    self.inflight[self.serial_next] = time.perf_counter()
                    self.serial_next += 1
                    credit -= 1
                if not self.started and self.state and self.state['state'] == 'ARMED':
                    if self.axis_motion:
                        baseline_requested_at = time.perf_counter()
                        self._axis_publish('读取启动实际基线', '缓冲已准备；先读取新鲜实际位置，尚未发送 STREAM_START。')
                        self._send('#GETJPOS')
                        self.last_poll = baseline_requested_at
                        self._axis_wait(lambda: self.axis_feedback[1] >= baseline_requested_at,
                                        1., '同步启动前实际基线未收到新鲜 RX；未发送 START', check_actual=True)
                        with self.lock:
                            actual, stamp = self.axis_feedback
                            self.axis_start_actual = actual.copy()
                            self.axis_start_stamp = stamp
                            self.axis_actual_peak = 0.
                            self.axis_deviation = {}
                            self.start_requested_at = time.perf_counter()
                            self.axis_result.update(motion_baseline_deg=float(np.rad2deg(actual[self.axis_nodes[0]-1])),
                                                    motion_baseline_age_ms=1000*(self.start_requested_at-stamp))
                    else:
                        self.start_requested_at = time.perf_counter()
                    self._axis_publish('等待同步启动', '已请求 STREAM_START，等待主控确认同步启动；此时尚不能认定电机开始执行。')
                    self._send(f'!STREAM_START {self.session}')
                    self.started = True
                now = time.perf_counter()
                if now-self.last_progress >= .2:
                    self.last_progress = now
                    self._axis_publish()
                self.cancel_event.wait(.002)
            if not self.started or self.done_sequence != self.axis_segments or self.accepted != self.axis_segments:
                raise RuntimeError(f'DONE / 已接受序号与最终序号 {self.axis_segments} 不一致')
            if self.axis_motion and not self.running_confirmed:
                raise RuntimeError('主控未确认 RUNNING，非零测试不判通过')
            if self.axis_motion and self.axis_actual_peak < np.deg2rad(.15):
                raise RuntimeError('未观察到正确方向至少 0.15° 的实际位移；不判通过')
            self.axis_final_check = True
            self.axis_executed = self.done_sequence
            self._axis_publish('核对返回位置' if self.axis_motion else '核对原地保持',
                               'DONE 已确认，核对所选轴实际返回位置、新鲜回读和 150 ms 稳定窗口。' if self.axis_motion else 'DONE 已确认，核对所选轴实际位置新鲜、偏差和 150 ms 稳定窗口。')
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
                    raise RuntimeError('所选轴最终返回实际位置未稳定' if self.axis_motion else '所选轴原地保持实际位置未稳定')
                self.cancel_event.wait(.004)
            self.axis_health_enabled = False
            if self.axis_health_pending is not None:
                self._axis_wait(lambda: self.axis_health_pending is None, .6,
                                '最终保持参考核对前 CAN 诊断请求未完成', check_actual=True)
            self._axis_publish('核对最终保持参考', '逐轴读取最终保持参考，同时检查所选轴实际 RX 新鲜度和偏差。')
            for node in self.axis_nodes:
                if self._axis_diag(node, 7, check_actual=True) != held[node]:
                    raise RuntimeError(f'J{node} 非零曲线结束未精确返回 SEED 参考' if self.axis_motion else f'J{node} 原地曲线结束保持参考改变')
            self._axis_pump(check_actual=True)
            performance_passed, performance_failure = True, None
            if self.axis_motion:
                self.axis_result['motion_verified'] = True
                self._axis_publish('读取本轮计时', '往返 / DONE / 实际返回已确认；只读核对计时和丢包计数。')
                try:
                    self._axis_read_timing(check_actual=True)
                    before = self.axis_result['timing_before'][self.axis_nodes[0]]
                    after = self.axis_result['timing_after'][self.axis_nodes[0]]
                    late = counter_delta(before[12], after[12])
                    drops = counter_delta(before[18], after[18])
                    running_samples = counter_delta(before[61], after[61])
                    self.axis_result['running_samples_delta'] = running_samples
                    running_evidence = after[58] > 0 and running_samples > 0
                    performance_passed = not (late or drops) and running_evidence
                    if late or drops:
                        performance_failure = f'性能项未通过：本轮 STREAM late 增量 {late}、流回包丢弃增量 {drops}，两项均须为 0'
                    elif not running_evidence:
                        performance_failure = '性能项未通过：缺少本轮有效 RUNNING 计时证据（回调峰值与采样增量必须均 >0）'
                    if (after[9] > before[9] and after[9] >= 3600 or
                            after[58] > before[58] and after[58] >= 3600):
                        self.axis_result['performance_warning'] = '本轮出现新的 ≥50 µs 回调 boot 峰值；需复查实际时限，目标为 RUNNING <40 µs'
                except TimingReadError as exc:
                    performance_passed = False
                    performance_failure = '性能项未通过：计时未完整确认；'+str(exc)
                self.axis_result['performance_passed'] = performance_passed
                self._axis_pump(check_actual=True)
            self._axis_retire()
            message = f'SEED→缓冲→同步 START→执行 {self.axis_segments} 段→DONE→CLOSE→模式恢复均已确认。'
            if performance_failure:
                message += performance_failure
            self._axis_publish('通过' if performance_passed else '未通过', message,
                               done=True, passed=performance_passed)
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
