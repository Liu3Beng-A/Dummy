"""No-device tests for bounded LONG collection, replay and evidence guards."""
from __future__ import annotations

import builtins
import contextlib
import copy
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import types
import unittest
from unittest import mock


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


end = load('bounded_endurance_tests', HERE / 'run_cartesian_endurance.py')
legacy = end.load_legacy()
helpers = load('bounded_endurance_fixtures', HERE / 'test_cartesian_benchmark_collector.py')
WIRE = REPO / 'firmware_artifacts/cartesian-benchmark-adapter-20261017T013331800412Z'


def fake_manifest(root):
    target = Path(root) / 'fake-only.manifest.txt'
    target.write_text('schema=1\nrelease=2026101701\nproject=Core-STM32F4-fw\n'
                      'buildset=0123456789ABCDEF\nutc_epoch=1792198800\nsource_sha256=' + 'ab' * 32 + '\n', encoding='utf-8')
    return target


def identity_lines(identity):
    payload = [f"v={identity['v']} type={identity['type']}", f"epoch={identity['epoch']}",
               f"group={identity['group']}", 'h0=' + identity['source_sha256'][:32].upper(),
               'h1=' + identity['source_sha256'][32:].upper(), 'end']
    return ['ok FW_VERSION [0] req=17 pending'] + ['FW_VERSION [0] req=17 ' + t for t in payload]


def status_lines(config=None, *, job=6, phase='DONE', n=None, elapsed=None, reason='NONE'):
    if phase == 'IDLE':
        job, config, n, elapsed, hz = 0, dict(op=0, fixture=0, mode=0, seconds=0, planned=0), 0, 0, 0
    else:
        config = config or end.request_config(seconds=1)
        n = config['planned'] if n is None else n
        elapsed = config['seconds'] * 1000 if elapsed is None else elapsed
        hz = 0 if phase == 'PENDING' else 168000000
    return [f'CBLONG v=1 id={job} phase={phase}',
            'CBLONG caps seconds=600 period_ms=20 work=5',
            'CBLONG caps bins=21 trends=60 history=8',
            f"CBLONG cfg op={config['op']} fixture={config['fixture']} mode={config['mode']} seconds={config['seconds']}",
            f"CBLONG cfg work=5 budget_us=20000 planned={config['planned']}",
            f'CBLONG clock hz={hz} tick_hz=1000', f'CBLONG run n={n} elapsed_ms={elapsed}',
            f'CBLONG stop reason={reason}', 'CBLONG no_motion=1 scope=components', f'CBLONG end id={job}']


def parsed_status(config=None, **kwargs):
    boundary = end.LongStatus()
    for line in status_lines(config, **kwargs):
        boundary.feed(line)
    return boundary.values


def split64(value):
    return value >> 32, value & end.UINT32_MAX


def read_lines(status, *, error=0, error_sequence=end.UINT32_MAX):
    job, n, elapsed = status['id'], status['n'], status['elapsed_ms']
    sample, window = 168000, 168100
    total, wall = n * sample, n * window
    trends, history = (n + 499) // 500, min(n, 8)
    rows = [f'CBLONG read v=1 id={job}', f'CBLONG total {job},{n},{elapsed}',
            f'CBLONG sum {job},' + ','.join(map(str, split64(total))),
            f'CBLONG window {job},' + ','.join(map(str, split64(wall))) + f',{window if n else 0}',
            f'CBLONG dwt {job},' + ','.join(map(str, split64(elapsed * 168000))),
            f'CBLONG minmax {job},{sample if n else 0},{sample if n else 0}',
            f'CBLONG late {job},0,0,{20 if n else 0}',
            f'CBLONG error {job},{error},{error_sequence}',
            f'CBLONG sig {job},10,5,40,5,0', f'CBLONG hash {job},12345',
            f'CBLONG counts {job},21,{trends},{history}',
            f'CBLONG overhead {job},0,{n * 1000},1000', f'CBLONG scan {job},2,100']
    rows += [f'CBLONG H {job},{i},{n if i == 1 else 0}' for i in range(21)]
    rows += [f'CBLONG P {job},{i},' + ','.join(map(str, split64(wall if i == 0 else 0))) for i in range(7)]
    rows += [f'CBLONG O {job},{lane},{stage},' + ','.join(map(str, split64(wall if lane in (1, 3) and stage == 4 else 0)))
             for lane in range(4) for stage in range(5)]
    for i in range(trends):
        count = min(500, n - i * 500)
        rows += [f'CBLONG T {job},{i},{count},' + ','.join(map(str, split64(count * sample))),
                 f'CBLONG U {job},{i},{sample},0,20', f'CBLONG K {job},{i},6492,692,772']
    for i, sequence in enumerate(range(n - history, n)):
        rows += [f'CBLONG F {job},{i},{sequence},{sequence * 20},{window}',
                 f'CBLONG G {job},{i},0,{sample},0', f'CBLONG J {job},{i},12345,0']
    rows += [f'CBLONG read_end id={job}']
    return rows


def parse_read(status, rows=None):
    boundary = end.LongRead(status)
    for line in read_lines(status) if rows is None else rows:
        boundary.feed(line)
    return boundary


class Clock:
    def __init__(self):
        self.now = 100.0

    def monotonic(self):
        return self.now

    def sleep(self, duration):
        self.now += duration


class FakeUSB(helpers.FakePort):
    """A deterministic no-device board; fake seconds advance without waiting."""
    def __init__(self, identity, config, clock, *, old_board=False, wrong_identity=False,
                 never_terminal=False, restore_fail=False, final_oled_timeout=False,
                 run_response_timeout=False, boundary_pending=False, prior_long=False,
                 wrong_poll_config=False, partial_write=False):
        super().__init__([])
        self.identity, self.config, self.clock = identity, config, clock
        self.old_board, self.wrong_identity = old_board, wrong_identity
        self.never_terminal, self.restore_fail = never_terminal, restore_fail
        self.final_oled_timeout = final_oled_timeout
        self.run_response_timeout, self.boundary_pending = run_response_timeout, boundary_pending
        self.prior_long, self.wrong_poll_config, self.partial_write = prior_long, wrong_poll_config, partial_write
        self.started, self.opened, self.begin = prior_long, False, clock.now if prior_long else None
        self.cancelled = False
        self.last_status = parsed_status(phase='IDLE')

    def open(self):
        self.opened = True

    def close(self):
        self.opened = False

    def read(self, size):
        if not self.chunks:
            self.clock.now += .05
        return super().read(size)

    def write(self, encoded):
        self.writes.append(encoded)
        command = encoded.decode('ascii').strip()
        if command == '#FW_VERSION 0':
            rows = identity_lines(self.identity)
            if self.wrong_identity:
                rows = [t.replace('20261017.01', '20261016.06') for t in rows]
        elif command == '#KIN_CONFIG':
            rows = helpers.native_kin()
        elif command == '#TOOL':
            rows = helpers.native_tool()
        elif command == '#MOTION_STATUS':
            rows = [t.replace('phase=0', 'phase=3').replace('moving=000', 'moving=0FE') for t in helpers.native_motion()]
        elif command == '#STACK':
            rows = helpers.native_stack()
        elif command == '#CBENCH':
            rows = helpers.native_status(job_id=6 if self.started else 5)
        elif command == '#CBENCH LONG' and self.old_board:
            rows = ['error CBENCH syntax']
        elif command == '#CBENCH LONG':
            if not self.started:
                rows = status_lines(phase='IDLE')
            else:
                elapsed = int((self.clock.now - self.begin) * 1000)
                terminal = not self.never_terminal and elapsed >= self.config['seconds'] * 1000
                pending_boundary = self.boundary_pending and elapsed == self.config['seconds'] * 1000
                elapsed = min(elapsed, self.config['seconds'] * 1000)
                n = min(elapsed // 20, self.config['planned'])
                if self.cancelled:
                    rows = status_lines(self.config, phase='CANCELLED', n=n, elapsed=elapsed, reason='CANCELLED')
                elif pending_boundary:
                    rows = status_lines(self.config, phase='RUNNING', n=n, elapsed=elapsed - 20)
                else:
                    rows = status_lines(self.config, phase='DONE' if terminal else 'RUNNING', n=n, elapsed=elapsed)
                if self.wrong_poll_config:
                    rows = [t.replace('fixture=2', 'fixture=1') for t in rows]
        elif command == end.long_command(self.config):
            self.started, self.begin = True, self.clock.now
            if self.run_response_timeout:
                return len(encoded)
            if self.partial_write:
                return len(encoded) - 1
            rows = status_lines(self.config, phase='PENDING', n=0, elapsed=0)
        elif command == '#CBENCH CANCEL':
            self.cancelled = True
            rows = status_lines(self.config, phase='RUNNING', n=0, elapsed=0)
        elif command == '#CBENCH LREAD':
            rows = read_lines(self.last_status, error=7 if self.cancelled else 0,
                              error_sequence=self.last_status['n'] if self.cancelled else end.UINT32_MAX)
        elif command == '#CBENCH OLED':
            if self.started and self.final_oled_timeout and self.last_status['phase'] == 'DONE':
                return len(encoded)
            rows = helpers.oled_metadata_v3(job_id=6 if self.started else 5,
                                            enabled=int(self.started), mode=self.config['mode'],
                                            priority=(16 if self.config['mode'] == 4 else 24) if self.started else 0,
                                            runtime_flags=4 if self.restore_fail and self.started else 0,
                                            restored=0 if self.restore_fail and self.started else 1)
        else:
            raise AssertionError('Unexpected fake command: ' + command)
        if rows[0].startswith('CBLONG v='):
            boundary = end.LongStatus()
            for row in rows:
                boundary.feed(row)
            self.last_status = boundary.values
        self.chunks.append(('\r\n'.join(rows) + '\r\n').encode('ascii'))
        return len(encoded)


class ProtocolTests(unittest.TestCase):
    def test_allowlist_and_request_bounds(self):
        self.assertEqual(end.request_config()['planned'], 30000)
        for op in (3, 4):
            for fixture in range(3):
                for seconds in (1, 600):
                    for mode in ('rate-10hz', 'split-low'):
                        self.assertTrue(end.command_allowed(end.long_command(end.request_config(op, fixture, seconds, mode))))
        for value in ('#CBENCH LONG 4,2,5,601,20000,4', '#CBENCH LONG 4,2,6,600,20000,4',
                      '#CBENCH LONG 4,2,5,600,21000,4', '#CBENCH LONG 4,2,5,600,20000,3',
                      '#CBENCH RUN 3,0,5,100,20000', '!START', '#CBENCH LONG\n!START'):
            self.assertFalse(end.command_allowed(value), value)

    def test_status_requires_ten_lines_and_explicit_same_id_end(self):
        rows = status_lines()
        boundary = end.LongStatus()
        self.assertEqual([boundary.feed(t) for t in rows], [False] * 9 + [True])
        for change in (lambda r: r[:-1] + ['CBLONG end id=7'],
                       lambda r: r[:1] + r[2:], lambda r: r[:3] + [r[3]] + r[3:],
                       lambda r: [t.replace('period_ms=20', 'period_ms=10') for t in r]):
            with self.assertRaises(end.ProtocolError):
                reader = end.LongStatus()
                for row in change(rows):
                    reader.feed(row)

    def test_full_600_seconds_stats_u64_and_bounded_history(self):
        status = parsed_status(end.request_config())
        result = parse_read(status)
        self.assertTrue(result.finished)
        self.assertEqual(len(result.values['trends']), 60)
        self.assertEqual(len(result.values['history']), 8)
        self.assertEqual(end.u64(*result.values['sum']), 30000 * 168000)
        self.assertGreater(end.u64(*result.values['dwt']), 2 ** 32)
        self.assertEqual(sum(result.values['histogram']), 30000)

    def test_empty_setup_failure_is_collectable_not_success(self):
        status = parsed_status(phase='ABORTED', n=0, elapsed=0, reason='SETUP')
        stats = parse_read(status, read_lines(status, error=10, error_sequence=0)).values
        boundary = end.OledBoundary(legacy, status['id'])
        for row in helpers.oled_metadata_v3(job_id=status['id']):
            boundary.feed(row)
        summary = end.summarize(status, stats, boundary.metadata.values, {'error': None})
        self.assertTrue(summary['collection_complete'])
        self.assertFalse(summary['job_success'])

    def test_each_missing_or_duplicate_read_record_rejected(self):
        status = parsed_status()
        rows = read_lines(status)
        for index in range(1, len(rows) - 1):
            for altered in (rows[:index] + rows[index + 1:], rows[:index] + [rows[index]] + rows[index:]):
                with self.subTest(index=index), self.assertRaises(end.ProtocolError):
                    parse_read(status, altered)

    def test_accounting_histogram_index_and_history_corruption_rejected(self):
        status = parsed_status()
        rows = read_lines(status)
        variants = [lambda t: t.replace('CBLONG H 6,1,50', 'CBLONG H 6,1,49'),
                    lambda t: t.replace('CBLONG P 6,0,0,8405000', 'CBLONG P 6,0,0,8405001'),
                    lambda t: t.replace('CBLONG O 6,1,4,0,8405000', 'CBLONG O 6,1,4,0,8405001'),
                    lambda t: t.replace('CBLONG F 6,0,42', 'CBLONG F 6,0,41'),
                    lambda t: t.replace('CBLONG counts 6,21,1,8', 'CBLONG counts 6,21,61,8')]
        for mutate in variants:
            with self.subTest(mutate=mutate), self.assertRaises(end.ProtocolError):
                parse_read(status, [mutate(t) for t in rows])

    def test_percentiles_are_bucket_intervals_not_exact_values(self):
        histogram = [0] * 21
        histogram[4] = 99
        histogram[20] = 1
        self.assertEqual(end.percentile_bucket(histogram, .95)['lower_us_inclusive'], 4000)
        self.assertEqual(end.percentile_bucket(histogram, 1)['upper_us_exclusive'], None)
        self.assertIsNone(end.percentile_bucket([0] * 21, .95))

    def test_production_cpp_formatter_both_math_modes(self):
        for mode in ('precise', 'cmsis512'):
            path = WIRE / mode / 'long-terminal.wire'
            if not path.exists():
                self.fail('Required production formatter evidence missing: ' + str(path))
            before = end.sha(path)
            rows = path.read_text(encoding='utf-8').splitlines()
            self.assertEqual(len(rows), 294)
            status = end.LongStatus()
            for row in rows[:10]:
                status.feed(row)
            stats = parse_read(status.values, rows[10:-18]).values
            oled = end.OledBoundary(legacy, status.values['id'])
            for row in rows[-18:]:
                oled.feed(row)
            summary = end.summarize(status.values, stats, oled.metadata.values, {'error': None})
            self.assertTrue(summary['collection_complete'])
            self.assertTrue(summary['job_success'])
            self.assertEqual(summary['status']['n'], 30000)
            self.assertFalse(summary['performance_acceptance'])
            self.assertEqual(end.sha(path), before)

    def test_production_clock_failure_exports_complete_failure_evidence(self):
        for mode in ('precise', 'cmsis512'):
            path = WIRE / mode / 'long-clock-error.wire'
            before = end.sha(path)
            rows = path.read_text(encoding='utf-8').splitlines()
            status = end.LongStatus()
            for row in rows[:10]:
                status.feed(row)
            stats = parse_read(status.values, rows[10:-18]).values
            oled = end.OledBoundary(legacy, status.values['id'])
            for row in rows[-18:]:
                oled.feed(row)
            summary = end.summarize(status.values, stats, oled.metadata.values, {'error': None})
            self.assertTrue(summary['collection_complete'])
            self.assertFalse(summary['job_success'])
            self.assertFalse(summary['kernel_success'])
            self.assertFalse(summary['timing_clock_usable'])
            self.assertIsNone(summary['mean_us'])
            self.assertEqual(stats['error'][0], 5)
            self.assertEqual(end.sha(path), before)

    def test_missing_worker_or_render_is_insufficient_activity_evidence(self):
        status = parsed_status()
        stats = parse_read(status).values
        boundary = end.OledBoundary(legacy, status['id'])
        for row in helpers.oled_metadata_v3(job_id=status['id']):
            boundary.feed(row)
        oled = boundary.metadata.values
        for variation in ('worker', 'render'):
            altered = dict(oled)
            for key in (('begun', 'ended') if variation == 'worker' else ('display_render_begun',)):
                altered[key] = 0
            result = end.summarize(status, stats, altered, {'error': None})
            self.assertTrue(result['kernel_success'])
            self.assertEqual(result['display_observation']['status'], 'evidence_insufficient')
            self.assertFalse(result['display_observation']['starvation_acceptance'])


class CaptureTests(unittest.TestCase):
    def prepare(self, root, seconds=1):
        return end.prepare(Path(root), fake_manifest(root), end.request_config(seconds=seconds), 10, 60)

    def collect_fake(self, out, plan, *, run=True, cancel=False, **options):
        clock = Clock()
        port = FakeUSB(plan['identity'], plan['request'], clock, **options)
        fake_serial = types.SimpleNamespace(Serial=lambda **kwargs: port, __version__='offline_fake_only')
        with mock.patch.dict('sys.modules', {'serial': fake_serial}), mock.patch('time.monotonic', side_effect=clock.monotonic), mock.patch('time.sleep', side_effect=clock.sleep), contextlib.redirect_stdout(io.StringIO()):
            result = end.collect(out, plan, 'OFFLINE_FAKE', run=run, cancel=cancel, timeout=.01)
        return result, port, clock

    def test_fake_600_second_run_and_offline_replay(self):
        with tempfile.TemporaryDirectory() as root:
            out, plan = self.prepare(root, 600)
            result, port, clock = self.collect_fake(out, plan)
            self.assertTrue(result['job_success'])
            self.assertEqual(result['status']['n'], 30000)
            self.assertEqual(clock.now, 700.0)
            self.assertFalse(port.opened)
            commands = [t.decode('ascii').strip() for t in port.writes]
            self.assertEqual(sum(t.startswith('#CBENCH LONG ') for t in commands), 1)
            self.assertEqual(commands.count('#CBENCH LONG'), 61)
            self.assertNotIn('#CBENCH CANCEL', commands)
            target, replay = end.analyze(out)
            self.assertTrue(target.exists())
            self.assertEqual(replay, result)

    def test_old_board_and_wrong_identity_rejected_before_long_transmit(self):
        for variation in ('old_board', 'wrong_identity'):
            with tempfile.TemporaryDirectory() as root, self.subTest(variation=variation):
                out, plan = self.prepare(root)
                result, port, _ = self.collect_fake(out, plan, **{variation: True})
                self.assertFalse(result['job_success'])
                self.assertFalse(result['session']['run_transmit_attempted'])
                self.assertFalse(any(t.startswith(b'#CBENCH LONG ') for t in port.writes))
                self.assertTrue((out / 'capture-index.json').exists())
                _, replay = end.analyze(out)
                self.assertEqual(replay, result)

    def test_planned_count_without_terminal_waits_for_duration_boundary(self):
        with tempfile.TemporaryDirectory() as root:
            out, plan = self.prepare(root, 600)
            result, port, clock = self.collect_fake(out, plan, boundary_pending=True)
            self.assertTrue(result['job_success'])
            self.assertEqual(clock.now, 710.0)
            self.assertEqual(port.writes.count(b'#CBENCH LONG\n'), 62)

    def test_run_ack_timeout_and_partial_write_preserve_attempt_not_acceptance(self):
        for variation in ('run_response_timeout', 'partial_write'):
            with tempfile.TemporaryDirectory() as root, self.subTest(variation=variation):
                out, plan = self.prepare(root)
                result, port, _ = self.collect_fake(out, plan, **{variation: True})
                self.assertTrue(result['session']['run_transmit_attempted'])
                self.assertFalse(result['session']['run_accepted'])
                self.assertFalse(result['job_success'])
                self.assertEqual(sum(t.startswith(b'#CBENCH LONG ') for t in port.writes), 1)
                self.assertNotIn(b'#CBENCH CANCEL\n', port.writes)
                self.assertEqual(end.analyze(out)[1], result)

    def test_final_oled_timeout_preserves_stats_without_complete_collection(self):
        with tempfile.TemporaryDirectory() as root:
            out, plan = self.prepare(root)
            result, _, _ = self.collect_fake(out, plan, final_oled_timeout=True)
            self.assertIsNotNone(result['aggregates'])
            self.assertFalse(result['collection_complete'])
            self.assertFalse(result['job_success'])
            self.assertEqual(end.analyze(out)[1], result)

    def test_poll_configuration_change_stops_and_failed_capture_replays(self):
        with tempfile.TemporaryDirectory() as root:
            out, plan = self.prepare(root)
            result, _, _ = self.collect_fake(out, plan, wrong_poll_config=True)
            self.assertIn('configuration', result['session']['error'])
            self.assertFalse(result['job_success'])
            self.assertEqual(end.analyze(out)[1], result)

    def test_explicit_cancel_only_identified_active_long(self):
        for prior_long in (False, True):
            with tempfile.TemporaryDirectory() as root, self.subTest(prior_long=prior_long):
                out, plan = self.prepare(root)
                result, port, _ = self.collect_fake(out, plan, run=False, cancel=True, prior_long=prior_long)
                self.assertEqual(b'#CBENCH CANCEL\n' in port.writes, prior_long)
                self.assertFalse(result['job_success'])
                if prior_long:
                    self.assertTrue(result['collection_complete'])
                    self.assertEqual(result['status']['phase'], 'CANCELLED')
                else:
                    self.assertIn('identified active LONG', result['session']['error'])
                self.assertEqual(end.analyze(out)[1], result)

    def test_deadline_stops_without_automatic_cancel_or_reconnect(self):
        with tempfile.TemporaryDirectory() as root:
            out, plan = self.prepare(root)
            result, port, clock = self.collect_fake(out, plan, never_terminal=True)
            self.assertFalse(result['job_success'])
            self.assertTrue(result['session']['run_transmit_attempted'])
            self.assertTrue(result['session']['run_accepted'])
            self.assertIn('overall deadline', result['session']['error'])
            self.assertNotIn(b'#CBENCH CANCEL\n', port.writes)
            self.assertLessEqual(clock.now, 161.0)

    def test_restore_failure_retains_full_capture_but_not_success(self):
        with tempfile.TemporaryDirectory() as root:
            out, plan = self.prepare(root)
            result, _, _ = self.collect_fake(out, plan, restore_fail=True)
            self.assertTrue(result['collection_complete'])
            self.assertFalse(result['job_success'])
            _, replay = end.analyze(out)
            self.assertEqual(replay, result)

    def test_readonly_screen_never_sends_start_command(self):
        with tempfile.TemporaryDirectory() as root:
            out, plan = self.prepare(root)
            result, port, _ = self.collect_fake(out, plan, run=False)
            self.assertFalse(result['session']['run_transmit_attempted'])
            self.assertEqual(result['status']['phase'], 'IDLE')
            self.assertFalse(any(t.startswith(b'#CBENCH LONG ') for t in port.writes))
            self.assertFalse(result['job_success'])
            _, replay = end.analyze(out)
            self.assertEqual(replay, result)

    def test_capture_cannot_be_resumed_or_overwritten(self):
        with tempfile.TemporaryDirectory() as root:
            out, plan = self.prepare(root)
            self.collect_fake(out, plan)
            original = (out / 'raw.jsonl').read_bytes()
            with self.assertRaises(end.ProtocolError):
                end.collect(out, plan, 'OFFLINE_FAKE', run=True)
            self.assertEqual((out / 'raw.jsonl').read_bytes(), original)

    def test_capture_frozen_source_and_raw_mutation_rejected(self):
        for name in ('raw.jsonl', 'collector_source.py', 'firmware.manifest.txt', 'session.json', 'summary.json'):
            with tempfile.TemporaryDirectory() as root, self.subTest(name=name):
                out, plan = self.prepare(root)
                self.collect_fake(out, plan)
                with (out / name).open('ab') as stream:
                    stream.write(b'\n')
                with self.assertRaises(end.ProtocolError):
                    end.analyze(out)

    def test_plan_only_and_analyze_do_not_import_serial(self):
        real_import = builtins.__import__
        def no_serial(name, *args, **kwargs):
            if name == 'serial':
                self.fail('Offline action imported serial')
            return real_import(name, *args, **kwargs)
        with tempfile.TemporaryDirectory() as root:
            manifest = fake_manifest(root)
            with mock.patch('builtins.__import__', side_effect=no_serial), contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(end.main(['--manifest', str(manifest), '--output-root', root]), 0)
            out, plan = self.prepare(root)
            self.collect_fake(out, plan)
            with mock.patch('builtins.__import__', side_effect=no_serial):
                self.assertTrue(end.analyze(out)[1]['job_success'])

    def test_invalid_cli_or_old_manifest_never_opens_serial(self):
        for argv in (['--run'], ['--cancel'], ['--analyze', 'unused', '--port', 'OFFLINE_FAKE']):
            with self.subTest(argv=argv), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                end.main(argv)
        with tempfile.TemporaryDirectory() as root:
            manifest = fake_manifest(root)
            manifest.write_text(manifest.read_text().replace('2026101701', '2026101606'))
            with self.assertRaises(end.ProtocolError):
                end.prepare(Path(root), manifest, end.request_config(), 10, 60)

    def test_all_6000_historical_samples_replay_legacy_unchanged(self):
        matrix = REPO / 'tools/tests/run_cartesian_benchmark_matrix.py'
        before_source = end.sha(matrix), end.sha(end.LEGACY)
        batch = REPO / 'firmware_artifacts/cartesian-benchmark-matrix-20261017T010354953565Z'
        state = end.read_json(batch / 'state.json')
        self.assertTrue(state['complete'])
        self.assertEqual(len(state['completed']), 60)
        samples = 0
        for item in state['completed']:
            capture = batch / item['capture']
            original = {name: end.sha(capture / name) for name in item['source_hashes']}
            parser = legacy.CaptureParser()
            for line in legacy.capture_lines(capture / 'raw.jsonl'):
                parser.feed(line)
            summary, rows = parser.summarize()
            self.assertTrue(summary['job_success'])
            self.assertEqual(len(rows), 100)
            samples += len(rows)
            self.assertEqual({name: end.sha(capture / name) for name in original}, original)
        self.assertEqual(samples, 6000)
        self.assertEqual((end.sha(matrix), end.sha(end.LEGACY)), before_source)
        self.assertEqual(before_source[1], end.LEGACY_SHA)
        self.assertEqual(before_source[0], 'da51fdb3422b4f6d00faa9c539f1674400e980eb55f7eec97d8ab10ac5ee572f')


if __name__ == '__main__':
    unittest.main(verbosity=2)
