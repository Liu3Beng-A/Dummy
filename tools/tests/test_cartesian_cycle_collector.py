"""Offline fake-wire tests. Never import pyserial or open a real device."""
from __future__ import annotations
import contextlib
import importlib.util
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import patch

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('cycle_collector_under_test', HERE / 'run_cartesian_cycle_diagnostic.py')
cycle = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cycle)
OUT = cycle.REPO / 'firmware_artifacts' / ('cartesian-cycle-collector-tests-' + cycle.stamp())
FORMATTER = Path(os.environ.get('CARTESIAN_CYCLE_FORMATTER_FIXTURES', ''))


def identity_lines(identity):
    fields = [f'v={identity["v"]} type=CORE', f'epoch={identity["epoch"]}', f'group={identity["group"]}',
              'h0=' + identity['source_sha256'][:32], 'h1=' + identity['source_sha256'][32:], 'end']
    return ['ok FW_VERSION [0] req=1 pending'] + ['FW_VERSION [0] req=1 ' + text for text in fields]


def motion_lines():
    rows = ['MOTION id=0 kind=MOVEJ phase=0 mode=2 ms=0', 'MOTION ready=000 confirmed=000 progress=000',
            'MOTION send=0 query=0 reject=0', 'MOTION plan_ms=0.0 moving=000 txok=000 pending=000',
            'MOTION spread tx=-1 settled_obs=-1']
    for node in range(1, 8):
        rows += [f'MOTION J{node} goal=0.0000 real=0.0000 f=00 age=4294967295',
                 f'MOTION J{node} tx=-1 ready=-1 try=0 fail=0', f'MOTION J{node} plan=-1 pos=-1 settled=-1',
                 f'MOTION J{node} v=0.00000 a=30.000 d=0.0000']
    return rows


def stack_lines():
    return ['ok STACK usb=6000 uart=6000 heap=13056', 'ok STACK cmd=6400 size=8192 heap_min=10432', 'ok STACK rt=3900 oled=700 rgb=1800']


def oled_lines(job_id=0, enabled=0):
    enabled = int(enabled)
    mode = 4 if enabled else 0
    rows = [f'CBOLED v=3 id={job_id} enabled={enabled} skip=0 mode={mode}', 'CBOLED caps samples=128 stages=5 lanes=2',
            f'CBOLED rate period_ticks={100 if enabled else 0} tick_hz=1000']
    for kind, count in (('F', 3), ('L', 2), ('M', 2), ('R', 2), ('G', 2), ('D', 3), ('J', 2), ('N', 2), ('U', 2), ('H', 2), ('S', 2), ('A', 3)):
        rows.append(f'CBOLED {kind} {job_id},' + ','.join(['0'] * count))
    rows += [f'CBOLED C {job_id},0,1,{16 if enabled else 24}', f'CBOLED K {job_id},1900,2048', f'CBOLED end id={job_id}']
    return rows


def full_lines(config=None, job_id=0, phase='IDLE', count=None, reason=0, source=0, would_stop=0, emitted=None):
    c = config or cycle.configuration(samples=4)
    n = (c['samples'] if phase == 'DONE' else 0) if count is None else count
    emitted = n if emitted is None else emitted
    return [f'CFULL v=1 id={job_id} phase={phase}', 'CFULL caps samples=160 scenarios=13 variants=3',
            f'CFULL cfg case={c["case"]} fixture={c["fixture"]} variant={c["variant"]} samples={c["samples"]}',
            'CFULL clock hz=168000000 tick_hz=1000', f'CFULL run n={n} planned={c["samples"]} budget=20000',
            f'CFULL fault reason={reason} source={source} would_stop={would_stop}', 'CFULL first seq=0',
            f'CFULL result emitted={emitted} accepted={emitted}', f'CFULL work ik={n} fk={n} peak={int(bool(n))}',
            'CFULL setup cycles=10000', 'CFULL no_motion=1 scope=full_cycle', f'CFULL end id={job_id}']


def read_lines(job_id, first, amount, wrap=False, flags=5, reason=0):
    rows = [f'CFULL read id={job_id} first={first} n={amount}']
    for index in range(first, first + amount):
        tick = ((cycle.U32 - 9 if wrap else 1000) + index * 20) & cycle.U32
        frame = {'index': index, 'q': {j: [100 + index + j] for j in range(7)}, 'va': {j: [j - 3, j] for j in range(7)}}
        checksum = cycle.record_checksum(frame)
        rows += [f'CFULL T {job_id},{index},{tick},{tick},{(tick+5)&cycle.U32}',
                 f'CFULL C {job_id},{index},800000,840000,{checksum}',
                 f'CFULL S {job_id},{index},1,1,{reason},{flags}',
                 f'CFULL P {job_id},{index},100000,200000,-30000',
                 f'CFULL I {job_id},{index},101000,200000,-30000', f'CFULL R {job_id},{index},1000,250000']
        for axis in range(7):
            rows += [f'CFULL Q {job_id},{index},{axis},{frame["q"][axis][0]}',
                     f'CFULL V {job_id},{index},{axis},{frame["va"][axis][0]},{frame["va"][axis][1]}']
        rows.append(f'CFULL E {job_id},{index}')
    return rows + [f'CFULL read_end id={job_id} first={first} n={amount}']


def parsed_status(config=None, **kwargs):
    boundary = cycle.FullStatus()
    for line in full_lines(config, **kwargs):
        boundary.feed(line)
    return boundary.values


class FakePort:
    def __init__(self, plan, mutate=None, initial='IDLE', result='DONE', partial=False, truncate=None, all_chunks=False):
        self.plan, self.mutate, self.phase, self.result = plan, mutate, initial, result
        self.partial, self.truncate, self.buffer, self.writes = partial, truncate, bytearray(), []
        self.closed = False
        self.all_chunks = all_chunks

    def write(self, data):
        command = data.decode('ascii').strip()
        self.writes.append(command)
        if command == '#FW_VERSION 0':
            rows = identity_lines(self.plan['identity'])
        elif command == '#MOTION_STATUS':
            rows = motion_lines()
        elif command == '#STACK':
            rows = stack_lines()
        elif command == '#CBENCH OLED':
            rows = oled_lines(0 if self.phase == 'IDLE' else 7, self.phase != 'IDLE')
        elif command == '#CBENCH FULL':
            if self.phase == 'RUNNING':
                self.phase = self.result
            rows = full_lines(self.plan['request'], 0 if self.phase == 'IDLE' else 7, self.phase)
        elif command.startswith('#CBENCH FULL_RUN '):
            self.phase = 'RUNNING'
            rows = full_lines(self.plan['request'], 7, 'RUNNING')
        elif command == '#CBENCH FULL_CANCEL':
            self.phase = 'CANCELLED'
            rows = full_lines(self.plan['request'], 7, 'CANCELLED', reason=8, source=1)
        elif command.startswith('#CBENCH FREAD '):
            first, amount = map(int, command.split()[-1].split(','))
            rows = read_lines(7, first, amount)
        else:
            raise AssertionError('Unexpected command ' + command)
        if self.mutate:
            rows = self.mutate(command, rows)
        if self.truncate and self.truncate == command:
            rows = rows[:-1]
        self.buffer.extend(('\n'.join(rows) + '\n').encode('ascii'))
        return len(data) - 1 if self.partial else len(data)

    def read(self, count):
        size = min(count, count if self.all_chunks else (1 if len(self.writes) % 2 else 173), len(self.buffer))
        data = bytes(self.buffer[:size])
        del self.buffer[:size]
        return data

    def close(self):
        self.closed = True


class FormatterPort(FakePort):
    """Supply immutable production formatter responses without editing payloads."""
    def __init__(self, plan, fixture):
        super().__init__(plan)
        rows = fixture.read_text(encoding='utf-8').splitlines()
        self.responses = {}
        end = next(i for i, row in enumerate(rows) if row.startswith('CFULL end ')) + 1
        self.responses['#CBENCH FULL'] = rows[:end]
        offset = end
        while rows[offset].startswith('CFULL read '):
            header = rows[offset]
            first = int(header.split(' first=')[1].split()[0])
            count = int(header.split(' n=')[1])
            tail = next(i for i in range(offset + 1, len(rows)) if rows[i].startswith('CFULL read_end ')) + 1
            self.responses[f'#CBENCH FREAD {first},{count}'] = rows[offset:tail]
            offset = tail
        self.responses['#CBENCH OLED'] = rows[offset:]

    def write(self, data):
        command = data.decode('ascii').strip()
        if command not in self.responses:
            return super().write(data)
        self.writes.append(command)
        self.buffer.extend(('\n'.join(self.responses[command]) + '\n').encode('ascii'))
        return len(data)


class FakeClock:
    def __init__(self):
        self.value = 1000.

    def monotonic(self):
        return self.value

    def sleep(self, seconds):
        self.value += seconds


class FormatterAfterRunPort(FormatterPort):
    def __init__(self, plan, fixture):
        super().__init__(plan, fixture)
        self.started = False

    def write(self, data):
        command = data.decode('ascii').strip()
        if command.startswith('#CBENCH FULL_RUN '):
            self.started = True
            self.writes.append(command)
            rows = full_lines(self.plan['request'], 1, 'RUNNING')
            self.buffer.extend(('\n'.join(rows) + '\n').encode('ascii'))
            return len(data)
        if not self.started and command in ('#CBENCH FULL', '#CBENCH OLED'):
            return FakePort.write(self, data)
        return super().write(data)


class CollectorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        OUT.mkdir(parents=True, exist_ok=False)
        cls.counter = 0
        cls.legacy = cycle.load_legacy()

    def setUp(self):
        type(self).counter += 1
        self.path = OUT / f'{self.counter:03d}-{self._testMethodName}'
        self.path.mkdir()
        self.manifests = self.path / 'synthetic-manifests'
        self.manifests.mkdir()
        for i, name in enumerate(cycle.MANIFESTS):
            (self.manifests / (name + '.manifest.txt')).write_text(
                'schema=1\nrelease=2026101702\nbuildset=0123456789ABCDEF\nutc_epoch=1792201000\nsource_sha256=' + '1' * 64
                + '\nproject=' + ('Core-STM32F4-fw' if i == 0 else 'Ctrl-Step-STM32-fw') + '\n', encoding='utf-8')
        self.out, self.plan = cycle.prepare(self.path, self.manifests, cycle.configuration(samples=4), ['OFFLINE-FAKE-WIRE'])

    def capture(self, clock=None, **kwargs):
        port_keys = ('mutate', 'initial', 'result', 'partial', 'truncate', 'all_chunks')
        port_args = {key: kwargs.pop(key) for key in port_keys if key in kwargs}
        self.port = FakePort(self.plan, **port_args)
        with contextlib.ExitStack() as stack:
            stack.enter_context(patch.object(cycle.time, 'sleep', side_effect=clock.sleep if clock else lambda seconds: None))
            if clock:
                stack.enter_context(patch.object(cycle.time, 'monotonic', side_effect=clock.monotonic))
            return cycle.collect(self.out, self.plan, 'FAKE-NO-DEVICE', timeout=.02, port_factory=lambda: self.port, **kwargs)

    def timed_plan(self, interval):
        self.out, self.plan = cycle.prepare(self.path, self.manifests, cycle.configuration(samples=160), ['EXPLICIT-FAKE-TIMER'], interval)

    def test_default_is_plan_only_no_serial_or_files(self):
        before = set(self.path.iterdir())
        with contextlib.redirect_stdout(io.StringIO()), patch.dict(sys.modules, {'serial': None}):
            self.assertEqual(cycle.main([]), 0)
        self.assertEqual(set(self.path.iterdir()), before)

    def test_allowlist_denies_motion_and_unbounded_commands(self):
        for command in ('!START', '!STOP', '!DISABLE', '@0,0,0', '#CBENCH RUN 2,0,1,16,20000', '#CBENCH FULL_RUN 1,0,0,4,20000',
                        '#CBENCH FULL_RUN 1,0,1,4,40000', '#CBENCH FREAD 159,4', '#CBENCH FREAD 0,5', '#CBENCH FULL\n!START'):
            with self.subTest(command=command):
                self.assertFalse(cycle.command_allowed(command))
        self.assertTrue(cycle.command_allowed(cycle.run_command(self.plan['request'])))

    def test_read_only_is_complete_and_never_runs(self):
        result = self.capture()
        self.assertTrue(result['collection_complete'])
        self.assertFalse(result['job_success'])
        self.assertTrue(self.port.closed)
        self.assertFalse(any('FULL_RUN' in command for command in self.port.writes))
        self.assertFalse(result['hardware_evidence'])

    def test_run_collect_replay_preserves_original_and_integer_axes(self):
        result = self.capture(run=True)
        self.assertTrue(result['collection_complete'], result['errors'])
        self.assertTrue(result['job_success'])
        self.assertEqual(result['window_us']['max'], 5000)
        before = {p.name: cycle.sha(p) for p in self.out.iterdir() if p.is_file()}
        with patch.dict(sys.modules, {'serial': None}):
            target, replay = cycle.analyze(self.out)
        self.assertTrue(target.is_dir())
        self.assertEqual(replay, result)
        self.assertEqual(before, {p.name: cycle.sha(p) for p in self.out.iterdir() if p.is_file()})
        self.assertIn('q0,v0,a0', (self.out / 'frames.csv').read_text())

    def test_duplicate_capture_never_overwrites(self):
        self.capture()
        before = (self.out / 'raw.jsonl').read_bytes()
        with self.assertRaises(cycle.ProtocolError):
            cycle.collect(self.out, self.plan, 'FAKE', port_factory=lambda: self.port)
        self.assertEqual((self.out / 'raw.jsonl').read_bytes(), before)

    def test_wrong_identity_preserved_and_run_not_sent(self):
        result = self.capture(run=True, mutate=lambda command, rows: [line.replace('20261017.02', '20261017.01') for line in rows] if command == '#FW_VERSION 0' else rows)
        self.assertFalse(result['collection_complete'])
        self.assertEqual(self.port.writes, ['#FW_VERSION 0'])
        self.assertTrue((self.out / 'raw.jsonl').is_file())
        self.assertIn('Board differs', result['errors'][0])
        target, replay = cycle.analyze(self.out)
        self.assertFalse(replay['collection_complete'])
        self.assertTrue(target.is_dir())

    def test_busy_identity_stops_without_retry(self):
        result = self.capture(run=True, mutate=lambda command, rows: ['error FW_VERSION J0 req=0 busy'] if command == '#FW_VERSION 0' else rows)
        self.assertFalse(result['collection_complete'])
        self.assertEqual(self.port.writes, ['#FW_VERSION 0'])

    def test_active_motion_rejects_before_run(self):
        result = self.capture(run=True, mutate=lambda command, rows: [line.replace('phase=0', 'phase=2') for line in rows] if command == '#MOTION_STATUS' else rows)
        self.assertFalse(result['collection_complete'])
        self.assertNotIn(cycle.run_command(self.plan['request']), self.port.writes)

    def test_oled_capability_rejects_before_run(self):
        result = self.capture(run=True, mutate=lambda command, rows: [line.replace('lanes=2', 'lanes=1') for line in rows] if command == '#CBENCH OLED' else rows)
        self.assertFalse(result['collection_complete'])
        self.assertNotIn(cycle.run_command(self.plan['request']), self.port.writes)

    def test_timeout_never_cancels_or_retries(self):
        result = self.capture(run=True, truncate=cycle.run_command(self.plan['request']))
        self.assertFalse(result['collection_complete'])
        self.assertEqual(self.port.writes.count(cycle.run_command(self.plan['request'])), 1)
        self.assertNotIn('#CBENCH FULL_CANCEL', self.port.writes)
        self.assertIn('not automatically cancelled', result['errors'][0])
        self.assertFalse(cycle.analyze(self.out)[1]['collection_complete'])

    def test_partial_write_preserved_no_retry(self):
        result = self.capture(run=True, partial=True)
        self.assertFalse(result['collection_complete'])
        self.assertEqual(len(self.port.writes), 1)
        self.assertIn('Partial write', result['errors'][0])

    def test_firmware_abort_collected_separately_from_success(self):
        result = self.capture(run=True, result='ABORTED')
        self.assertTrue(result['collection_complete'], result['errors'])
        self.assertFalse(result['job_success'])
        self.assertEqual(result['status']['phase'], 'ABORTED')

    def test_cancel_idle_refused_no_cancel_transmission(self):
        result = self.capture(cancel=True)
        self.assertFalse(result['collection_complete'])
        self.assertNotIn('#CBENCH FULL_CANCEL', self.port.writes)

    def test_explicit_cancel_active_only_logical_cancel(self):
        result = self.capture(cancel=True, initial='RUNNING', result='RUNNING')
        self.assertTrue(result['collection_complete'], result['errors'])
        self.assertFalse(result['job_success'])
        self.assertEqual(result['status']['phase'], 'CANCELLED')
        self.assertIn('#CBENCH FULL_CANCEL', self.port.writes)
        self.assertFalse(any('FULL_RUN ' in command for command in self.port.writes))
        self.assertEqual(cycle.analyze(self.out)[1], result)

    def test_explicit_same_port_cancel_first_active_response(self):
        self.timed_plan(0)
        result = self.capture(run=True, result='RUNNING', clock=FakeClock())
        self.assertTrue(result['collection_complete'], result['errors'])
        self.assertEqual(result['status']['phase'], 'CANCELLED')
        self.assertTrue(result['session']['explicit_cancel_sent'])
        self.assertEqual(result['session']['explicit_cancel_elapsed_ms'], 0)
        self.assertEqual(self.port.writes.count('#CBENCH FULL_CANCEL'), 1)
        self.assertEqual(self.port.writes[self.port.writes.index(cycle.run_command(self.plan['request'])) + 1], '#CBENCH FULL_CANCEL')
        self.assertEqual(cycle.analyze(self.out)[1], result)

    def test_explicit_same_port_midrun_cancel_uses_host_poll_clock(self):
        self.timed_plan(400)
        result = self.capture(run=True, result='RUNNING', clock=FakeClock())
        self.assertTrue(result['collection_complete'], result['errors'])
        self.assertEqual(result['session']['explicit_cancel_outcome'], 'sent')
        self.assertGreaterEqual(result['session']['explicit_cancel_elapsed_ms'], 400)
        self.assertLessEqual(result['session']['explicit_cancel_elapsed_ms'], 601)
        self.assertEqual(self.port.writes.count('#CBENCH FULL_CANCEL'), 1)
        self.assertGreaterEqual(self.port.writes.count('#CBENCH FULL'), 3)
        self.assertEqual(cycle.analyze(self.out)[1], result)

    def test_same_port_cancel_sent_once_when_ack_still_reports_running(self):
        self.timed_plan(0)
        def mutate(command, rows):
            if command == '#CBENCH FULL_CANCEL':
                return full_lines(self.plan['request'], 7, 'RUNNING')
            if command == '#CBENCH FULL' and any('phase=CANCELLED' in row for row in rows):
                return full_lines(self.plan['request'], 7, 'CANCELLED', reason=8, source=1)
            return rows
        result = self.capture(run=True, result='RUNNING', clock=FakeClock(), mutate=mutate)
        self.assertTrue(result['collection_complete'], result['errors'])
        self.assertEqual(self.port.writes.count('#CBENCH FULL_CANCEL'), 1)
        self.assertEqual(result['status']['phase'], 'CANCELLED')
        self.assertEqual(cycle.analyze(self.out)[1], result)

    def test_timed_cancel_not_sent_when_job_terminal_before_deadline(self):
        self.timed_plan(3000)
        result = self.capture(run=True, clock=FakeClock())
        self.assertTrue(result['collection_complete'], result['errors'])
        self.assertTrue(result['job_success'])
        self.assertFalse(result['session']['explicit_cancel_sent'])
        self.assertEqual(result['session']['explicit_cancel_outcome'], 'terminal_before_cancel')
        self.assertNotIn('#CBENCH FULL_CANCEL', self.port.writes)
        self.assertEqual(cycle.analyze(self.out)[1], result)

    def test_default_run_has_no_timed_cancel_intent(self):
        result = self.capture(run=True, clock=FakeClock())
        self.assertIsNone(self.plan['explicit_cancel_after_ms'])
        self.assertEqual(result['session']['explicit_cancel_outcome'], 'not_requested')
        self.assertFalse(result['session']['cancel_transmitted'])
        self.assertNotIn('#CBENCH FULL_CANCEL', self.port.writes)
        self.assertEqual(cycle.analyze(self.out)[1], result)

    def test_cli_timed_cancel_requires_explicit_run_port_and_range(self):
        cases = [['--cancel-after-ms', '400'], ['--port', 'FAKE', '--cancel-after-ms', '400'],
                 ['--run', '--cancel-after-ms', '400'], ['--port', 'FAKE', '--cancel', '--cancel-after-ms', '400'],
                 ['--analyze', 'FAKE', '--cancel-after-ms', '400'], ['--port', 'FAKE', '--run', '--cancel-after-ms', '-1'],
                 ['--port', 'FAKE', '--run', '--cancel-after-ms', '3001']]
        with contextlib.redirect_stderr(io.StringIO()), patch.dict(sys.modules, {'serial': None}):
            for argv in cases:
                with self.subTest(argv=argv), self.assertRaises(SystemExit):
                    cycle.main(argv)

    def test_timed_plan_cannot_collect_without_run(self):
        self.timed_plan(400)
        with self.assertRaises(cycle.ProtocolError):
            cycle.collect(self.out, self.plan, 'FAKE-NO-DEVICE', port_factory=lambda: None)

    def test_trailing_duplicate_full_line_fails_before_context_commit(self):
        result = self.capture(run=True, all_chunks=True,
                              mutate=lambda command, rows: rows + ['CFULL end id=7'] if 'FULL_RUN ' in command else rows)
        self.assertFalse(result['collection_complete'])
        self.assertFalse(result['session']['run_accepted'])
        self.assertEqual(result['status']['phase'], 'IDLE')
        self.assertEqual(cycle.analyze(self.out)[1], result)

    def test_late_cancel_can_accept_input_without_committing_output(self):
        def mutate(command, rows):
            if command == '#CBENCH FULL' and any('phase=CANCELLED' in row for row in rows):
                return [row.replace('accepted=0', 'accepted=1') for row in full_lines(self.plan['request'], 7, 'CANCELLED',
                                                                                   count=1, reason=8, source=1, emitted=0)]
            if command.startswith('#CBENCH FREAD'):
                return read_lines(7, 0, 1, flags=4 | 128, reason=8)
            return rows
        result = self.capture(run=True, result='CANCELLED', mutate=mutate)
        self.assertTrue(result['collection_complete'], result['errors'])
        self.assertFalse(result['job_success'])
        self.assertEqual(result['accepted_without_output_frames'], 1)
        self.assertEqual(result['tcp_valid_frames'], 0)
        self.assertEqual(result['tcp_intent_error_mm']['count'], 0)
        self.assertEqual(cycle.analyze(self.out)[1], result)

    def test_terminal_fread_truncation_is_incomplete(self):
        result = self.capture(run=True, truncate='#CBENCH FREAD 0,4')
        self.assertFalse(result['collection_complete'])
        self.assertEqual(result['kernel_us']['count'], 0)

    def test_run_config_echo_wrong_rejected(self):
        result = self.capture(run=True, mutate=lambda command, rows: [line.replace('fixture=0', 'fixture=1') for line in rows] if 'FULL_RUN ' in command else rows)
        self.assertFalse(result['collection_complete'])
        self.assertIn('configuration differs', result['errors'][0])

    def test_status_rejects_missing_duplicate_wrongid_bad_scope(self):
        rows = full_lines(self.plan['request'], 7, 'DONE')
        corruptions = [rows[:-1], rows[:2] + [rows[1]] + rows[2:], rows[:-1] + ['CFULL end id=8'],
                       [line.replace('scope=full_cycle', 'scope=components') for line in rows],
                       [line.replace('variants=3', 'variants=4') for line in rows],
                       [line.replace('peak=1', 'peak=6') for line in rows]]
        for bad in corruptions:
            with self.subTest(bad=bad):
                boundary = cycle.FullStatus(expected_id=7)
                try:
                    ended = any(boundary.feed(line) for line in bad)
                except cycle.ProtocolError:
                    ended = False
                self.assertFalse(ended)

    def test_fread_roundtrip_wrap_and_max_signed_axes(self):
        status = parsed_status(self.plan['request'], job_id=7, phase='DONE')
        boundary = cycle.FullRead(status, 0, 4)
        for line in read_lines(7, 0, 4, wrap=True):
            complete = boundary.feed(line)
        self.assertTrue(complete)
        self.assertEqual(len(boundary.frames), 4)
        self.assertEqual((boundary.frames[1]['T'][0] - boundary.frames[0]['T'][0]) & cycle.U32, 20)

    def test_pending_and_early_clock_failure_allow_zero_hz(self):
        for phase, reason in (('PENDING', 0), ('CLOCK_ERROR', 32), ('CANCELLED', 8)):
            boundary = cycle.FullStatus()
            for line in full_lines(self.plan['request'], 1, phase, reason=reason):
                ended = boundary.feed(line.replace('hz=168000000', 'hz=0'))
            self.assertTrue(ended)

    def test_old_legacy_oled_id_allowed_only_for_idle_full_capabilities(self):
        result = self.capture(mutate=lambda command, rows: oled_lines(123, 1) if command == '#CBENCH OLED' else rows)
        self.assertTrue(result['collection_complete'], result['errors'])
        self.assertEqual(result['oled']['id'], 123)

    @unittest.skipUnless(FORMATTER.is_dir() and (FORMATTER / 'precise/full-terminal.wire').is_file(), 'Set CARTESIAN_CYCLE_FORMATTER_FIXTURES to actual adapter artifact')
    def test_actual_firmware_formatter_precise_and_cmsis_done_and_fatal(self):
        for mode in ('precise', 'cmsis512'):
            for name, success in (('full-terminal.wire', True), ('full-fatal.wire', False)):
                with self.subTest(mode=mode, fixture=name):
                    fixture = FORMATTER / mode / name
                    saved = self.path / (mode + '-' + name)
                    saved.write_bytes(fixture.read_bytes())
                    out, plan = cycle.prepare(self.path, self.manifests, cycle.configuration(), ['PRODUCTION-FORMATTER-FAKE-TRANSPORT'])
                    port = FormatterPort(plan, saved)
                    result = cycle.collect(out, plan, 'FAKE-NO-DEVICE', timeout=.2, port_factory=lambda: port)
                    self.assertTrue(result['collection_complete'], result['errors'])
                    self.assertEqual(result['job_success'], success)
                    self.assertEqual(result['kernel_us']['count'], 16 if success else 57)
                    self.assertFalse(result['hardware_evidence'])
                    if not success:
                        self.assertEqual(result['status']['fault'], (12, 0, 1))
                        self.assertEqual(result['status']['result'][0], 56)
                    self.assertEqual(cycle.analyze(out)[1], result)

    @unittest.skipUnless(FORMATTER.is_dir() and (FORMATTER / 'precise/full-terminal.wire').is_file(), 'Set CARTESIAN_CYCLE_FORMATTER_FIXTURES to actual adapter artifact')
    def test_actual_formatter_completes_before_explicit_cancel_timer(self):
        for mode in ('precise', 'cmsis512'):
            with self.subTest(mode=mode):
                fixture = FORMATTER / mode / 'full-terminal.wire'
                out, plan = cycle.prepare(self.path, self.manifests, cycle.configuration(scenario=0, samples=16),
                                          ['ACTUAL-FORMATTER-EARLY-DONE'], 400)
                port = FormatterAfterRunPort(plan, fixture)
                clock = FakeClock()
                with patch.object(cycle.time, 'monotonic', side_effect=clock.monotonic), patch.object(cycle.time, 'sleep', side_effect=clock.sleep):
                    result = cycle.collect(out, plan, 'FAKE-NO-DEVICE', run=True, timeout=.2, port_factory=lambda: port)
                self.assertTrue(result['collection_complete'], result['errors'])
                self.assertTrue(result['job_success'])
                self.assertFalse(result['session']['explicit_cancel_sent'])
                self.assertEqual(result['session']['explicit_cancel_outcome'], 'terminal_before_cancel')
                self.assertNotIn('#CBENCH FULL_CANCEL', port.writes)
                self.assertEqual(cycle.analyze(out)[1], result)

    def test_fread_corruption_rejected(self):
        rows = read_lines(7, 0, 4)
        mutations = [lambda lines: lines[:1] + [lines[1]] + lines[1:],
                     lambda lines: [t.replace('CFULL Q 7,0,0,', 'CFULL Q 8,0,0,') for t in lines],
                     lambda lines: [t for t in lines if not t.startswith('CFULL V 7,0,6,')],
                     lambda lines: [t.replace('CFULL Q 7,0,0,100', 'CFULL Q 7,0,0,2147483648') for t in lines],
                     lambda lines: [t.replace('CFULL Q 7,0,0,100', 'CFULL Q 7,0,0,nan') for t in lines],
                     lambda lines: [t.replace('CFULL Q 7,0,0,100', 'CFULL Q 7,0,0,101') for t in lines],
                     lambda lines: lines[:-1] + ['CFULL read_end id=7 first=1 n=4'],
                     lambda lines: [t.replace('1000,250000', '1000,1000001') for t in lines]]
        status = parsed_status(self.plan['request'], job_id=7, phase='DONE')
        for mutate in mutations:
            with self.subTest(mutate=mutate):
                boundary = cycle.FullRead(status, 0, 4)
                with self.assertRaises(cycle.ProtocolError):
                    for line in mutate(rows):
                        boundary.feed(line)

    def test_false_end_never_completes(self):
        boundary = cycle.FullRead(parsed_status(self.plan['request'], job_id=7, phase='DONE'), 0, 4)
        with self.assertRaises(cycle.ProtocolError):
            boundary.feed('CFULL read_end id=7 first=0 n=4')

    def test_local_manifest_requires_all_five_exact_batches(self):
        file = self.manifests / 'motor-42.manifest.txt'
        file.write_text(file.read_text().replace('0123456789ABCDEF', '0123456789ABCDE0'), encoding='utf-8')
        with self.assertRaises(cycle.ProtocolError):
            cycle.manifests_identity(self.manifests)

    def test_frozen_source_and_original_capture_tampering_rejected(self):
        self.capture()
        raw = self.out / 'raw.jsonl'
        raw.write_text(raw.read_text() + '\n', encoding='utf-8')
        with self.assertRaises(cycle.ProtocolError):
            cycle.analyze(self.out)

    def test_500_background_lines_are_not_response_end(self):
        result = self.capture(run=True, mutate=lambda command, rows: ['boot debug unrelated'] * 500 + rows if command.startswith('#CBENCH FREAD') else rows)
        self.assertTrue(result['collection_complete'], result['errors'])
        self.assertEqual(result['kernel_us']['count'], 4)

    def test_cli_analyze_never_imports_serial(self):
        self.capture(run=True)
        with contextlib.redirect_stdout(io.StringIO()), patch.dict(sys.modules, {'serial': None}):
            self.assertEqual(cycle.main(['--analyze', str(self.out)]), 0)


if __name__ == '__main__':
    stream = io.StringIO()
    result = unittest.TextTestRunner(stream=stream, verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(CollectorTests))
    output = stream.getvalue()
    print(output)
    if OUT.exists():
        (OUT / 'test.log').write_text(output, encoding='utf-8')
        cycle.write_json(OUT / 'test-summary.json', {'tests': result.testsRun, 'failures': len(result.failures), 'errors': len(result.errors),
                                                  'success': result.wasSuccessful(), 'hardware_opened': False, 'kind': 'offline_fake_wire',
                                                  'collector_sha256': cycle.sha(HERE / 'run_cartesian_cycle_diagnostic.py'),
                                                  'test_sha256': cycle.sha(Path(__file__))})
        print(OUT)
    raise SystemExit(0 if result.wasSuccessful() else 1)
