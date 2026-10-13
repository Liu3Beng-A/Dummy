"""Single node OPEN/CLOSE handshake. Never calls a planner or enables motion."""
import re
import secrets
import threading
import time

from dummy_motion.stream_protocol import fault_description


OPEN_BLOCKS = (
    '已有活动透传会话', '透传故障尚需停稳', '软停锁定', '编码器反馈无效',
    '目标失能', '当前失能', '目标制动', '当前制动', '堵转锁定',
    '控制器正在校准', '校准任务未结束', '保护阶段非 IDLE',
    '瞬时估计速度超出 ±512 微步/s', '内部参考速度非零',
    '请求模式与运行模式不一致', '内部参考位置未到目标',
    '目标与实际位置偏差超限', '相对零点坐标超出 int32', '运行模式不支持 OPEN',
)
STAGES = {'IDLE': '尚未测试', 'CAPS': '核对电机诊断能力', 'OPENING': '等待 OPEN 决策',
          'PROBING': '等待能力 / OPEN', 'CLOSING': '等待 CLOSE 确认',
          'DONE': 'CLOSE 已确认', 'FAULT': 'CLOSE 未确认'}


def feature_supported(features, session, node, key):
    return bool(session and features and features.get('session') == session and features.get(key, 0) & (1 << (node-1)))


def entry_field_allowed(field, features, session, node):
    return (field == 7 or 23 <= field <= 33 or
            45 <= field <= 47 and feature_supported(features, session, node, 'steady') or
            (34 <= field <= 44 or 48 <= field <= 50) and feature_supported(features, session, node, 'start'))


def format_entry_snapshot(node, session, snapshot, phase='OPEN', steady_supported=False):
    base = 23 if phase == 'OPEN' else 34
    bits = snapshot[base+1]
    causes = []
    names = list(OPEN_BLOCKS)
    if steady_supported:
        names[12] = '新鲜 50 ms 稳态证据不足或过期'
    if phase == 'START':
        names.append('目标或零点与 OPEN 锁存参考不匹配（精确 SEED 绑定）')
    for index, name in enumerate(names):
        if bits & (1 << index):
            causes.append(name)
    if bits >> len(names):
        causes.append('未知拒绝位，请核对固件版本')
    mode = snapshot[base+2]
    lines = [f'J{node} · {phase} 瞬间快照 · 会话 {session}',
             f'{phase} 拒绝位 0x{bits:08X}：'+('；'.join(causes) if causes else '所有准入条件通过'),
             f'运行 / 请求模式 {mode & 255} / {(mode >> 8) & 255}，保护阶段 {(mode >> 16) & 255}，到位标志 {snapshot[base+10]}',
             f'瞬时速度 {snapshot[base+3]}，参考速度 {snapshot[base+4]} 微步/s；'+
             ('新稳态判据使用 50 ms 证据，瞬时速度仅供诊断' if steady_supported else '旧判据准入阈值 ±512'),
             f'目标 {snapshot[base+5]}，参考 {snapshot[base+6]}，实际 {snapshot[base+7]}，零点 {snapshot[base+8]} 微步；'
             f'目标−实际 {snapshot[base+5]-snapshot[base+7]}',
             f'以上均为 {phase} 瞬间快照：电机 tick={snapshot[base+9]}（20 kHz），会话={snapshot[base]}']
    metric = 45 if phase == 'OPEN' else 48
    if steady_supported and all(field in snapshot for field in range(metric, metric+3)):
        age, spread, mean = (snapshot[field] for field in range(metric, metric+3))
        age_text = '无有效证据' if age == 0xffffffff else f'{age/20:.2f} ms'
        range_text = '未成熟' if spread == 0xffffffff else str(spread)
        bound = 142 if node <= 5 else 85 if node == 6 else 204
        lines.append(f'50 ms 稳态证据：年龄 {age_text}（≤50 ms），位置范围 {range_text}（≤{bound} 微步），均速 {mean}（|均速|≤512 微步/s）')
    elif steady_supported:
        lines.append('本次稳态证据字段未确认，不能补推缺失数据。')
    else:
        lines.append('电机新稳态能力未确认；按旧瞬时速度判据解释 OPEN。' if phase == 'OPEN' else '电机新稳态能力未确认。')
    return '\n'.join(lines)


def format_open_test(result):
    lines = [f"J{result['node']} · {result['stage']} · 会话 {result.get('session', 0) or '尚未创建'}",
             result.get('message', '')]
    if 'opened' in result:
        open_state = ('已接受' if result['opened'] else '已拒绝' if result.get('snapshot', {}).get(24) else '未确认接受')
        lines.append(f"OPEN {open_state} / "
                     f"CLOSE {'已确认' if result.get('closed') else '未确认'} / "
                     f"结果 {fault_description(result['code']).split('；')[0]}")
    snapshot = result.get('snapshot')
    if snapshot:
        lines.append(format_entry_snapshot(result['node'], result.get('session', snapshot[23]), snapshot,
                                           steady_supported=result.get('steady_supported', False)))
    return '\n'.join(line for line in lines if line)


class OpenTestMixin:
    def start_open_test(self, node):
        o, c = self.owner, self.controller
        reason = None
        if type(node) is not int or not 1 <= node <= 7:
            reason = '仅支持 J1～J7，夹爪不参与本测试'
        elif not c.is_connected or c.connection_type != 'USB':
            reason = '请连接 USB 虚拟串口'
        elif c._stop_locked or c._calibration_busy:
            reason = '保护 / 校准锁定中；测试不会自动 START'
        elif (o.busy or c.gamepad.claimed or c._seq_active or c._mode_pending is not None or
              c._mode_request is not None or c._parameter_request is not None or c._acc_sync_waiting):
            reason = '控制器有其他任务，请等待结束'
        elif o._has_queued_control() or (self.thread is not None and self.thread.is_alive()):
            reason = '控制发送队列或上一轮工作线程尚未退出'
        if reason:
            result = dict(node=node, stage='未开始', message=reason, done=True, passed=False)
            self.open_test_changed.emit(result)
            c.log('单电机测试未开始：'+reason, 'WARN')
            return False
        # No _available(), feedback freshness, tool transform or IK requirement.
        o._begin('open_test')
        self._prepare_worker(require_poll=False)
        self.test_node = node
        self.test_reply = None
        self.open_test_result = dict(node=node, stage='检查主控', message='只进行能力查询、OPEN、自动 CLOSE 和快照读取。', done=False)
        self.open_test_changed.emit(self.open_test_result)
        o._state('checking', f'J{node} 单电机 OPEN→CLOSE 测试（无运动）')
        self.thread = threading.Thread(target=self._open_test_run, args=(node,), name='Dummy OPEN/CLOSE diagnostic', daemon=True)
        self.thread.start()
        return True

    def _open_test_run(self, node):
        result = dict(self.open_test_result)
        try:
            self._send(f'#GET_CMDMODE {self.request}')
            self._wait(lambda: self.mode_reply is not None, 3., '主控模式查询未回复；未发送 OPEN')
            if self.mode_reply['busy']:
                raise RuntimeError('主控报告运动忙；未发送 OPEN')
            self._send('#STREAM_TEST')
            self._wait(lambda: self.test_reply is not None, 2., '主控不支持单电机测试查询或未回复；请核对本次固件')
            if self.test_reply['state'] not in ('IDLE', 'DONE'):
                raise RuntimeError('上一轮单电机测试尚未确认关闭；未发送新 OPEN')
            self.session = secrets.randbelow(65535)+1
            while self.session in self.session_history or self.session == self.test_reply['session']:
                self.session = self.session % 65535+1
            self.session_history.append(self.session)
            self.test_reply = None
            result.update(session=self.session, stage='等待主控', message='主控将只对选中节点执行 OPEN→CLOSE；不切换主控模式。')
            self._notify('open_test', dict(result))
            self._send(f'!STREAM_TEST {self.session} {node}')
            deadline = time.perf_counter()+6.
            next_query, observed = time.perf_counter()+.1, None
            while True:
                self._pump()
                reply = self.test_reply
                if reply and reply['session'] == self.session and reply['node'] == node:
                    signature = tuple(reply[key] for key in ('state', 'opened', 'closed', 'code'))
                    if signature != observed:
                        observed = signature
                        result.update(stage=STAGES[reply['state']], opened=bool(reply['opened']),
                                      closed=bool(reply['closed']), code=reply['code'])
                        self._notify('open_test', dict(result))
                    if reply['state'] in ('DONE', 'FAULT'):
                        break
                now = time.perf_counter()
                if now >= deadline:
                    raise RuntimeError('测试状态超过 6 秒未完成；CLOSE 结果未知，不能判为通过。主控仍会自动尝试关闭。')
                if now >= next_query:
                    self._send('#STREAM_TEST')
                    next_query = now+.1
                self.cancel_event.wait(.004)
            # Read after cleanup, using the latched OPEN session before AND after
            # the other fields. Never attribute a stale snapshot to this test.
            if result['opened'] or result['code'] == 9:
                result.update(stage='读取 OPEN 快照', message='读取电机保存的准入条件，不进行运动。')
                self._notify('open_test', dict(result))
                first = self._read_diagnostic(node, 23)
                if first != self.session:
                    raise RuntimeError(f'OPEN 快照会话不匹配（收到 {first}）；拒绝使用旧快照')
                snapshot = {23: first}
                for field in range(24, 34):
                    snapshot[field] = self._read_diagnostic(node, field)
                if self._read_diagnostic(node, 23) != first:
                    raise RuntimeError('读取期间 OPEN 快照已变化；结果不能判为通过')
                steady = feature_supported(self.features, self.session, node, 'steady')
                result['steady_supported'] = steady
                if steady:
                    for field in range(45, 48):
                        snapshot[field] = self._read_diagnostic(node, field)
                    if self._read_diagnostic(node, 23) != first:
                        raise RuntimeError('读取稳态证据期间 OPEN 快照已变化')
                result['snapshot'] = snapshot
            passed = bool(result.get('opened') and result.get('closed') and result.get('code') == 0
                          and result.get('snapshot', {}).get(24) == 0)
            result.update(done=True, passed=passed, stage='通过' if passed else '未通过',
                          message='单电机 OPEN 接受、CLOSE 确认；本次未下发运动点。' if passed else
                          '请查看拒绝位和状态；本次未下发运动点。' if result.get('closed') else
                          'CLOSE 未确认，主控保留会话占用；请保留完整日志，不继续运动测试。')
        except InterruptedError:
            return  # cancel() publishes interruption; no stale callback after reconnect.
        except Exception as exc:
            message = str(exc)
            gates = {'DISABLED': '主控未使能', 'STOP_LOCKED': '主控保护 / 软停锁定',
                     'CALIBRATION': '主控校准中', 'PARAMETER': '主控保护参数任务忙',
                     'MOVING': '主控运动忙', 'COMMAND_BUSY': '主控命令队列未清空',
                     'SESSION_BUSY': '主控已有透传会话', 'BEGIN': '主控会话准入未通过'}
            match = re.search(r'gate=([A-Z_]+)', message)
            if match:
                message = gates.get(match[1], match[1])+'；未下发电机 OPEN。'+message
            result.update(done=True, passed=False, stage='未完成', message=message)
        finally:
            with self.lock:
                self.active = False
                self.discarded.update(self.authorized)
                self.authorized.clear()
        self._notify('open_test_done', result)

    def _open_test_command_allowed(self, command):
        match = re.fullmatch(rf'#STREAM_DIAG {self.test_node} (\d+)', command)
        diagnostic = bool(match and (23 <= int(match[1]) <= 33 or 45 <= int(match[1]) <= 47 and
                          feature_supported(self.features, self.session, self.test_node, 'steady')))
        return (command == '#STREAM_TEST' or command == f'#GET_CMDMODE {self.request}' or
                command == f'!STREAM_TEST {self.session} {self.test_node}' or
                diagnostic)
