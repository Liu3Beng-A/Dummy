"""Offline matrix-study safeguards, real-capture regressions and fake execution.

No real serial module or device is used. Historical captures are read-only;
intentional corruptions are made only in TemporaryDirectory copies.
"""
from __future__ import annotations

from collections import Counter
import builtins
import contextlib
import copy
import csv
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import shutil
import tempfile
import types
import unittest
from unittest import mock


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]


def module_at(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


matrix = module_at('cartesian_matrix_offline_test', HERE / 'run_cartesian_benchmark_matrix.py')
bench = matrix.load_collector(matrix.COLLECTOR)
fixtures = module_at('cartesian_matrix_protocol_fixtures', HERE / 'test_cartesian_benchmark_collector.py')
REAL = [
    ('20261016T093230981490Z', 'rate-10hz'),
    ('20261016T093251450928Z', 'split-low'),
    ('20261016T093305072967Z', 'split-same'),
    ('20261016T093317377991Z', 'rate-10hz'),
]
FAILED_PREFLIGHT = (REPO / 'firmware_artifacts' /
                    'cartesian-benchmark-matrix-20261017T004627401614Z')
FAILED_CAPTURE = (FAILED_PREFLIGHT / 'preflight' /
                  'cartesian-benchmark-20261017T004627597726Z')


def failed_motion_lines():
    """Read the original failed preflight; never rewrite its historical capture."""
    return [line for line in bench.capture_lines(FAILED_CAPTURE / 'raw.jsonl')
            if line.startswith('MOTION ')]


def real_path(index=1):
    return REPO / 'firmware_artifacts' / ('cartesian-benchmark-' + REAL[index][0])


def case_for(index=1):
    return {'ordinal': 1, 'round': 1, 'pair': 'r1-f0-op3', 'op': 3,
            'fixture': 0, 'mode': REAL[index][1], 'work': 5,
            'samples': 100, 'budget_us': 20000}


def identity_lines():
    identity = matrix.expected_identity()
    fields = [f"v={identity['v']} type={identity['type']}",
              f"epoch={identity['epoch']}", f"group={identity['group']}",
              'h0=' + identity['source_sha256'][:32].upper(),
              'h1=' + identity['source_sha256'][32:].upper(), 'end']
    return ['ok FW_VERSION [0] req=123 pending'] + [f'FW_VERSION [0] req=123 {t}' for t in fields]


def hashes(path):
    return {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
            for p in path.iterdir() if p.is_file()}


def clone_capture(root, index=1):
    out = Path(root) / ('capture-' + str(index))
    shutil.copytree(real_path(index), out)
    return out


def mutate_raw(path, transform):
    rows = [json.loads(t) for t in (path / 'raw.jsonl').read_text(encoding='utf-8').splitlines()]
    for row in rows:
        if row.get('direction') == 'rx':
            row['line'] = transform(row['line'])
    (path / 'raw.jsonl').write_text(''.join(json.dumps(t) + '\n' for t in rows), encoding='utf-8')


def regenerate_exports(path, case):
    parsed = bench.CaptureParser()
    for line in bench.capture_lines(path / 'raw.jsonl'):
        parsed.feed(line)
    parsed.jobs[parsed.current_id]['oled_request_mode'] = case['mode']
    bench.export(path, parsed, parsed.current_id, matrix.read_json(path / 'session.json'))


class MatrixPlanTests(unittest.TestCase):
    def test_default_plan_capacity_and_cell_counts(self):
        plan = matrix.make_plan()
        self.assertEqual(len(plan), 60)
        self.assertEqual(sum(c['samples'] for c in plan), 6000)
        self.assertEqual(sum(c['samples'] * c['work'] for c in plan), 30000)
        self.assertEqual([c['ordinal'] for c in plan], list(range(1, 61)))
        self.assertEqual(Counter((c['fixture'], c['op'], c['mode']) for c in plan),
                         Counter({(f, op, mode): 5 for f in range(3) for op in (3, 4)
                                  for mode in ('rate-10hz', 'split-low')}))
        self.assertTrue(all(c['work'] == 5 and c['samples'] == 100 and c['budget_us'] == 20000 for c in plan))
        self.assertEqual(matrix.make_plan(), plan)

    def test_pairs_adjacent_and_orders_balanced(self):
        plan = matrix.make_plan()
        orders = Counter()
        for a, b in zip(plan[::2], plan[1::2]):
            self.assertEqual(a['pair'], b['pair'])
            self.assertEqual((a['round'], a['fixture'], a['op']), (b['round'], b['fixture'], b['op']))
            self.assertEqual({a['mode'], b['mode']}, {'rate-10hz', 'split-low'})
            orders[a['mode']] += 1
        self.assertEqual(orders, {'rate-10hz': 15, 'split-low': 15})
        self.assertEqual(Counter(c['round'] for c in plan), {r: 12 for r in range(1, 6)})
        for f in range(3):
            for op in (3, 4):
                first = [a['mode'] for a in plan[::2] if (a['fixture'], a['op']) == (f, op)]
                self.assertTrue(all(a != b for a, b in zip(first, first[1:])))

    def test_invalid_capacity_rejected(self):
        for repeats, samples in [(0, 100), (-1, 100), (11, 100), (1, 15), (1, 129)]:
            with self.subTest(repeats=repeats, samples=samples), self.assertRaises(matrix.StudyError):
                matrix.make_plan(repeats, samples)

    def test_expected_identity_copy_is_not_mutable_global(self):
        value = matrix.expected_identity()
        value['v'] = 'wrong'
        self.assertEqual(matrix.expected_identity()['v'], '20261016.06')


class IdentityAndMotionTests(unittest.TestCase):
    def test_complete_identity_and_fields_before_pending(self):
        lines = identity_lines()
        for rows in (lines, lines[1:4] + lines[:1] + lines[4:]):
            self.assertEqual(matrix.parse_identity(rows), matrix.expected_identity())

    def test_missing_duplicate_wrong_request_and_wrong_identity_rejected(self):
        lines = identity_lines()
        variants = [lines[1:], lines[:-1], lines + [lines[2]],
                    [t.replace('req=123 h0', 'req=124 h0') for t in lines]]
        for old, new in [('20261016.06', '20261016.05'), ('1792142213', '1792142214'),
                         ('937E3D839C914097', '0000000000000000'), ('BB13A6D2', 'AB13A6D2')]:
            variants.append([t.replace(old, new) for t in lines])
        for rows in variants:
            with self.subTest(rows=rows), self.assertRaises((matrix.StudyError, ValueError)):
                matrix.parse_identity(rows)

    def test_static_motion_does_not_claim_disabled(self):
        result = matrix.check_motion_snapshot(fixtures.native_motion())
        self.assertFalse(result['disabled_proven_by_host'])
        self.assertEqual(result['phase'], 0)

    def test_terminal_phases_accept_all_legal_historical_axis_masks(self):
        for phase in (0, 3, 4, 5):
            for mask in range(0, 0xFF, 2):
                with self.subTest(phase=phase, mask=mask):
                    lines = [line.replace('phase=0', f'phase={phase}')
                             .replace('moving=000', f'moving={mask:03X}')
                             for line in fixtures.native_motion()]
                    result = matrix.check_motion_snapshot(lines)
                    self.assertEqual(result['phase'], phase)
                    self.assertEqual(result['moving'], mask)
                    self.assertEqual(result['pending'], 0)
                    self.assertFalse(result['disabled_proven_by_host'])

    def test_active_phases_rejected_even_with_legal_historical_masks(self):
        for phase in (1, 2):
            for mask in (0, 0x02, 0xFE):
                lines = [line.replace('phase=0', f'phase={phase}')
                         .replace('moving=000', f'moving={mask:03X}')
                         for line in fixtures.native_motion()]
                with self.subTest(phase=phase, mask=mask), self.assertRaises(matrix.StudyError):
                    matrix.check_motion_snapshot(lines)

    def test_invalid_axis_pending_and_incomplete_motion_rejected(self):
        lines = fixtures.native_motion()
        variants = [[], [t for t in lines if not t.startswith('MOTION id=')],
                    [t for t in lines if not t.startswith('MOTION plan_ms=')],
                    lines + [lines[0]], lines + [lines[3]]]
        for old, new in [('phase=0', 'phase=6'), ('phase=0', 'phase=-1'),
                         ('moving=000', 'moving=001'), ('moving=000', 'moving=100'),
                         ('moving=000', 'moving=1FE'), ('moving=000', 'moving=GGG'),
                         ('pending=000', 'pending=040'), ('pending=000', 'pending=100')]:
            variants.append([t.replace(old, new) for t in lines])
        for rows in variants:
            with self.subTest(rows=rows), self.assertRaises(matrix.StudyError):
                matrix.check_motion_snapshot(rows)

    def test_real_failed_preflight_completed_movej_is_historical_not_live_motion(self):
        before = hashes(FAILED_CAPTURE)
        rows = failed_motion_lines()
        self.assertIn('MOTION id=15 kind=MOVEJ phase=3 mode=2 ms=2802', rows)
        self.assertIn('MOTION plan_ms=2650.0 moving=0FE txok=0FE pending=000', rows)
        result = matrix.check_motion_snapshot(rows)
        self.assertEqual((result['phase'], result['moving'], result['pending']), (3, 0xFE, 0))
        self.assertFalse(result['disabled_proven_by_host'])
        session = matrix.read_json(FAILED_CAPTURE / 'session.json')
        self.assertFalse(session['compute_requested'])
        self.assertEqual(session['commands'], ['#FW_VERSION 0', '#KIN_CONFIG', '#TOOL', '#MOTION_STATUS'])
        self.assertEqual(matrix.read_json(FAILED_PREFLIGHT / 'state.json')['completed'], [])
        self.assertEqual(hashes(FAILED_CAPTURE), before)


class FakeGuardTests(unittest.TestCase):
    def make_session(self, root, *, fw=None, motion=None, profile_version=2, oled_version=3, cold_map=False):
        class Port(fixtures.FakePort):
            def __init__(self):
                super().__init__([])

            def write(self, value):
                self.writes.append(value)
                command = value.decode('ascii').strip()
                if command == '#FW_VERSION 0':
                    rows = identity_lines() if fw is None else fw
                elif command == '#MOTION_STATUS':
                    rows = fixtures.native_motion() if motion is None else motion
                elif command == '#CBENCH':
                    rows = fixtures.native_status(job_id=0, phase='IDLE', n=0, hz=0)
                elif command == '#CBENCH PROFILE':
                    factory = fixtures.split_profile_metadata if profile_version == 2 else fixtures.profile_metadata
                    rows = factory(job_id=0, mode=0, n=0)
                    if cold_map:
                        rows = [t.replace('mapmask=63', 'mapmask=0') for t in rows]
                elif command == '#CBENCH OLED':
                    if oled_version == 3:
                        rows = fixtures.oled_metadata_v3(job_id=0, enabled=0, priority=0)
                    else:
                        rows = fixtures.oled_metadata_v2(job_id=0, enabled=0)
                elif command.startswith('#CBENCH RUN '):
                    rows = fixtures.native_status(job_id=1, phase='PENDING', n=0, hz=0)
                    rows = [t.replace('op=2 fixture=0 work=3 samples=2', 'op=3 fixture=0 work=5 samples=100') for t in rows]
                else:
                    raise AssertionError(command)
                self.chunks.append(('\r\n'.join(rows) + '\r\n').encode('ascii'))
                return len(value)

        port, events = Port(), []
        recorder = types.SimpleNamespace(record=lambda *args, **kwargs: None)
        session = matrix.guarded_session_class(bench, case_for(), events)(port, recorder, .2, bench.CaptureParser())
        return session, port, events

    def screen(self, session):
        for command in ('#FW_VERSION 0', '#MOTION_STATUS', '#CBENCH', '#CBENCH PROFILE', '#CBENCH OLED'):
            session.request(command)

    def test_valid_guard_admits_one_exact_run_without_serial(self):
        with tempfile.TemporaryDirectory() as root:
            session, port, events = self.make_session(root)
            self.screen(session)
            session.request(matrix.run_command(case_for()))
            self.assertEqual(sum(e['kind'] == 'run_admitted' for e in events), 1)
            self.assertFalse(next(e for e in events if e['kind'] == 'run_admitted')['motion']['disabled_proven_by_host'])
            self.assertEqual(sum(t.startswith(b'#CBENCH RUN ') for t in port.writes), 1)

    def test_run_without_same_session_screen_not_written(self):
        with tempfile.TemporaryDirectory() as root:
            session, port, _ = self.make_session(root)
            with self.assertRaises(matrix.StudyError):
                session.request(matrix.run_command(case_for()))
            self.assertEqual(port.writes, [])

    def test_cold_idle_unmapped_profile_is_admitted(self):
        with tempfile.TemporaryDirectory() as root:
            session, port, events = self.make_session(root, cold_map=True)
            self.screen(session)
            session.request(matrix.run_command(case_for()))
            self.assertEqual(sum(e['kind'] == 'run_admitted' for e in events), 1)
            self.assertEqual(sum(t.startswith(b'#CBENCH RUN ') for t in port.writes), 1)

    def test_wrong_firmware_stops_before_motion_or_run(self):
        with tempfile.TemporaryDirectory() as root:
            fw = [t.replace('20261016.06', '20261016.05') for t in identity_lines()]
            session, port, _ = self.make_session(root, fw=fw)
            with self.assertRaises(matrix.StudyError):
                self.screen(session)
            self.assertEqual(port.writes, [b'#FW_VERSION 0\n'])

    def test_old_capability_and_active_prior_job_do_not_write_run(self):
        for variation in ('profile', 'oled', 'active', 'unexpected'):
            with tempfile.TemporaryDirectory() as root, self.subTest(variation=variation):
                session, port, _ = self.make_session(root, profile_version=1 if variation == 'profile' else 2,
                                                    oled_version=2 if variation == 'oled' else 3)
                self.screen(session)
                if variation == 'active':
                    session.parser.jobs[0]['phase'] = 'RUNNING'
                command = matrix.run_command(case_for())
                if variation == 'unexpected':
                    command = command.replace('3,0,5', '4,0,5')
                with self.assertRaises(matrix.StudyError):
                    session.request(command)
                self.assertFalse(any(t.startswith(b'#CBENCH RUN ') for t in port.writes))

    def test_invalid_axis_bit_snapshot_stops_before_run(self):
        with tempfile.TemporaryDirectory() as root:
            motion = [t.replace('moving=000', 'moving=001') for t in fixtures.native_motion()]
            session, port, _ = self.make_session(root, motion=motion)
            with self.assertRaises(matrix.StudyError):
                self.screen(session)
            self.assertFalse(any(t.startswith(b'#CBENCH RUN ') for t in port.writes))

    def test_real_completed_snapshot_admits_fake_run_without_claiming_disabled(self):
        before = hashes(FAILED_CAPTURE)
        with tempfile.TemporaryDirectory() as root:
            session, port, events = self.make_session(root, motion=failed_motion_lines())
            self.screen(session)
            session.request(matrix.run_command(case_for()))
            admitted = [e for e in events if e['kind'] == 'run_admitted']
            self.assertEqual(len(admitted), 1)
            self.assertEqual(admitted[0]['motion']['moving'], 0xFE)
            self.assertEqual(admitted[0]['motion']['phase'], 3)
            self.assertFalse(admitted[0]['motion']['disabled_proven_by_host'])
            self.assertEqual(sum(t.startswith(b'#CBENCH RUN ') for t in port.writes), 1)
            self.assertFalse(any(t.startswith((b'!START', b'!DISABLE', b'!STOP', b'@', b'!CALIBRATION'))
                                 for t in port.writes))
        self.assertEqual(hashes(FAILED_CAPTURE), before)

    def test_active_and_pending_snapshots_stop_fake_usb_before_run(self):
        original = failed_motion_lines()
        for old, new in [('phase=3', 'phase=1'), ('phase=3', 'phase=2'),
                         ('pending=000', 'pending=040')]:
            with tempfile.TemporaryDirectory() as root, self.subTest(old=old, new=new):
                motion = [t.replace(old, new) for t in original]
                session, port, _ = self.make_session(root, motion=motion)
                with self.assertRaises(matrix.StudyError):
                    self.screen(session)
                self.assertFalse(any(t.startswith(b'#CBENCH RUN ') for t in port.writes))


class RealCaptureTests(unittest.TestCase):
    def test_four_real_captures_raw_replay_and_hashes_unchanged(self):
        references = {}
        for index in range(4):
            with self.subTest(index=index):
                before = hashes(real_path(index))
                report = matrix.verify_capture(real_path(index), case_for(index), references)
                self.assertEqual(report['samples'], 100)
                self.assertTrue(report['capture_checks_passed'])
                self.assertFalse(report['performance_acceptance'])
                self.assertEqual(report['numeric_signature'], [10, 5, 40, 5, 0, 3141804433])
                self.assertEqual(hashes(real_path(index)), before)
        self.assertEqual(len(references), 1)

    def test_wrong_case_and_budget_rejected(self):
        for key, value in [('fixture', 1), ('op', 4), ('mode', 'rate-10hz'), ('samples', 99), ('budget_us', 19000)]:
            case = case_for()
            case[key] = value
            with self.subTest(key=key), self.assertRaises(matrix.StudyError):
                matrix.verify_capture(real_path(), case, {})

    def test_wrong_raw_identity_stops(self):
        with tempfile.TemporaryDirectory() as root:
            path = clone_capture(root)
            mutate_raw(path, lambda t: t.replace('20261016.06', '20261016.05'))
            with self.assertRaises(matrix.StudyError):
                matrix.verify_capture(path, case_for(), {})

    def test_full_identity_arriving_only_after_run_is_rejected(self):
        with tempfile.TemporaryDirectory() as root:
            path = clone_capture(root)
            target = path / 'raw.jsonl'
            events = [json.loads(t) for t in target.read_text(encoding='utf-8').splitlines()]
            fields = [e for e in events if e.get('direction') == 'rx' and 'FW_VERSION [0]' in e.get('line', '')]
            events = [e for e in events if not (e.get('direction') == 'rx' and 'FW_VERSION [0]' in e.get('line', ''))]
            position = next(i for i, e in enumerate(events) if e.get('direction') == 'tx' and e['command'].startswith('#CBENCH RUN '))
            events[position + 1:position + 1] = fields
            target.write_text(''.join(json.dumps(e) + '\n' for e in events), encoding='utf-8')
            with self.assertRaises(matrix.StudyError):
                matrix.verify_capture(path, case_for(), {})

    def test_budget_exit0_like_complete_capture_is_not_success(self):
        with tempfile.TemporaryDirectory() as root:
            path = clone_capture(root)
            mutate_raw(path, lambda t: t.replace('phase=DONE', 'phase=BUDGET'))
            regenerate_exports(path, case_for())
            self.assertTrue(matrix.read_json(path / 'summary.json')['collection_complete'])
            with self.assertRaises(matrix.StudyError):
                matrix.verify_capture(path, case_for(), {})

    def test_numerical_failure_despite_done_rejected(self):
        with tempfile.TemporaryDirectory() as root:
            path = clone_capture(root)
            def failed(line):
                if line.startswith('CBENCH RESULT '):
                    prefix, values = line.rsplit(' ', 1)
                    fields = values.split(',')
                    fields[6] = '1'
                    return prefix + ' ' + ','.join(fields)
                return line
            mutate_raw(path, failed)
            regenerate_exports(path, case_for())
            self.assertTrue(matrix.read_json(path / 'summary.json')['job_success'])
            with self.assertRaises(matrix.StudyError):
                matrix.verify_capture(path, case_for(), {})

    def test_impossible_valid_branch_count_rejected(self):
        for invalid in (0, 41):
            with tempfile.TemporaryDirectory() as root, self.subTest(valid=invalid):
                path = clone_capture(root)
                def bad_count(line):
                    if line.startswith('CBENCH RESULT '):
                        prefix, values = line.rsplit(' ', 1)
                        fields = values.split(',')
                        fields[4] = str(invalid)
                        return prefix + ' ' + ','.join(fields)
                    return line
                mutate_raw(path, bad_count)
                regenerate_exports(path, case_for())
                with self.assertRaises(matrix.StudyError):
                    matrix.verify_capture(path, case_for(), {})

    def test_same_cell_changed_signature_rejected_and_reference_not_poisoned(self):
        references = {(3, 0, 5): (10, 5, 40, 5, 0, 1)}
        before = copy.deepcopy(references)
        with self.assertRaises(matrix.StudyError):
            matrix.verify_capture(real_path(), case_for(), references)
        self.assertEqual(references, before)

    def test_saved_derived_summary_cannot_disagree_with_raw(self):
        with tempfile.TemporaryDirectory() as root:
            path = clone_capture(root)
            summary = matrix.read_json(path / 'summary.json')
            summary['oled']['accounting_valid'] = False
            matrix.write_json(path / 'summary.json', summary)
            with self.assertRaises(matrix.StudyError):
                matrix.verify_capture(path, case_for(), {})

    def test_csv_value_missing_column_and_missing_file_rejected(self):
        for variation in ('value', 'column', 'missing'):
            with tempfile.TemporaryDirectory() as root, self.subTest(variation=variation):
                path = clone_capture(root)
                target = path / 'samples.csv'
                if variation == 'missing':
                    target.unlink()
                else:
                    with target.open(encoding='utf-8', newline='') as stream:
                        reader = csv.DictReader(stream)
                        fields, rows = reader.fieldnames, list(reader)
                    if variation == 'column':
                        fields = ['id']
                        rows = [{'id': t['id']} for t in rows]
                    else:
                        rows[0]['checksum'] = '0'
                    with target.open('w', encoding='utf-8', newline='') as stream:
                        writer = csv.DictWriter(stream, fieldnames=fields)
                        writer.writeheader()
                        writer.writerows(rows)
                with self.assertRaises((matrix.StudyError, FileNotFoundError)):
                    matrix.verify_capture(path, case_for(), {})


class BatchLifecycleTests(unittest.TestCase):
    def prepare(self, root, repeats=1):
        return matrix.prepare_batch(Path(root), matrix.make_plan(repeats=repeats, samples=100))

    def test_prepare_freezes_sources_and_never_imports_serial(self):
        original_import = builtins.__import__
        def guarded(name, *args, **kwargs):
            if name == 'serial':
                self.fail('Offline preparation imported serial')
            return original_import(name, *args, **kwargs)
        with tempfile.TemporaryDirectory() as root, mock.patch('builtins.__import__', side_effect=guarded):
            out = self.prepare(root)
            study = matrix.check_frozen(out)
            self.assertFalse(study['execute_requested'])
            self.assertEqual(study['planned_runs'], 12)
            self.assertEqual(study['planned_samples'], 1200)
            self.assertEqual(matrix.digest(out / 'source/tools/tests/run_cartesian_benchmark.py'), matrix.COLLECTOR_SHA)

    def test_tampered_frozen_source_or_plan_rejected(self):
        for variation in ('collector', 'plan'):
            with tempfile.TemporaryDirectory() as root, self.subTest(variation=variation):
                out = self.prepare(root)
                if variation == 'collector':
                    path = out / 'source/tools/tests/run_cartesian_benchmark.py'
                    path.write_bytes(path.read_bytes() + b'\n# deliberately corrupted fixture\n')
                else:
                    study = matrix.read_json(out / 'plan.json')
                    study['plan'][0]['budget_us'] = 19000
                    matrix.write_json(out / 'plan.json', study)
                with self.assertRaises(matrix.StudyError):
                    matrix.check_frozen(out)

    def test_default_main_plan_only_and_no_serial(self):
        original_import = builtins.__import__
        def guarded(name, *args, **kwargs):
            if name == 'serial':
                self.fail('Default plan imported serial')
            return original_import(name, *args, **kwargs)
        with tempfile.TemporaryDirectory() as root, mock.patch('builtins.__import__', side_effect=guarded), contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(matrix.main(['--output-root', root]), 0)
            self.assertEqual(len(list(Path(root).glob('cartesian-benchmark-matrix-*'))), 1)

    def test_frozen_collector_entry_with_replayed_usb_exports_guarded_capture(self):
        """Actual collector and guard, with prerecorded response packets only."""
        source_events = [json.loads(t) for t in (real_path() / 'raw.jsonl').read_text(encoding='utf-8').splitlines()]
        responses = []
        for event in source_events:
            if event.get('direction') == 'tx':
                responses.append([event['command'], []])
            elif event.get('direction') == 'rx':
                responses[-1][1].append(event['line'])

        class ReplayUSB(fixtures.FakePort):
            def __init__(self):
                super().__init__([])
                self.remaining = copy.deepcopy(responses)
                self.opened = False

            def open(self):
                self.opened = True

            def close(self):
                self.opened = False

            def write(self, value):
                self.writes.append(value)
                command, rows = self.remaining.pop(0)
                if value.decode('ascii').strip() != command:
                    raise AssertionError('Replay command differs: ' + value.decode('ascii').strip() + ' != ' + command)
                self.chunks.append(('\r\n'.join(rows) + '\r\n').encode('ascii'))
                return len(value)

        with tempfile.TemporaryDirectory() as root:
            out = self.prepare(root)
            port = ReplayUSB()
            fake_serial = types.SimpleNamespace(Serial=lambda **kwargs: port, __version__='offline_replay_only')
            with mock.patch.dict('sys.modules', {'serial': fake_serial}), mock.patch('time.sleep', return_value=None):
                capture = matrix.run_collector(out, out / 'replay', 'OFFLINE_FAKE', case_for())
            report = matrix.verify_capture(capture, case_for(), {})
            self.assertEqual(report['samples'], 100)
            self.assertEqual(port.remaining, [])
            self.assertFalse(port.opened)
            admission = matrix.read_json(out / 'replay/admission.json')
            self.assertEqual(sum(e['kind'] == 'run_admitted' for e in admission), 1)
            session = matrix.read_json(capture / 'session.json')
            self.assertEqual(session['pyserial_version'], 'offline_replay_only')
            self.assertFalse(session['motion_commands_permitted'])

    def test_explicit_port_and_run_required_together(self):
        for argv in (['--port', 'OFFLINE_FAKE'], ['--run'], ['--analyze', 'unused', '--port', 'OFFLINE_FAKE']):
            with self.subTest(argv=argv), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                matrix.main(argv)

    def test_fake_batch_success_exact_order(self):
        with tempfile.TemporaryDirectory() as root:
            out = self.prepare(root, repeats=5)
            calls = []
            def fake_run(out_, destination, port, case):
                calls.append(copy.deepcopy(case))
                return destination / 'fake-capture'
            report = {'mean_us': 4500, 'timing': {'max_us': 5000},
                      'source_hashes': {name: '0' * 64 for name in matrix.ORIGINAL_FILES}}
            with mock.patch.object(matrix, 'run_collector', side_effect=fake_run), mock.patch.object(matrix, 'verify_capture', return_value=report), contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(matrix.execute_batch(out, 'OFFLINE_FAKE'), 0)
            study, state = matrix.read_json(out / 'plan.json'), matrix.read_json(out / 'state.json')
            self.assertEqual(calls, [None] + study['plan'])
            self.assertEqual([e['case'] for e in state['completed']], study['plan'])
            self.assertEqual(len(state['completed']), 60)
            self.assertEqual(study['planned_samples'], 6000)
            self.assertTrue(state['complete'])
            self.assertIsNone(state['stopped'])

    def test_fake_preflight_failure_stops_before_first_run(self):
        with tempfile.TemporaryDirectory() as root:
            out = self.prepare(root)
            with mock.patch.object(matrix, 'run_collector', side_effect=matrix.StudyError('FW pending timeout')) as run, mock.patch.object(matrix, 'verify_capture') as verify, contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(matrix.execute_batch(out, 'OFFLINE_FAKE'), 2)
            self.assertEqual(run.call_count, 1)
            self.assertIsNone(run.call_args.args[-1])
            verify.assert_not_called()
            self.assertEqual(matrix.read_json(out / 'state.json')['completed'], [])

    def test_fake_capture_failure_stops_without_retry_or_next_cell(self):
        with tempfile.TemporaryDirectory() as root:
            out = self.prepare(root)
            calls = []
            def fake_run(out_, destination, port, case):
                calls.append(copy.deepcopy(case))
                return destination / 'fake-capture'
            report = {'mean_us': 4500, 'timing': {'max_us': 5000},
                      'source_hashes': {name: '0' * 64 for name in matrix.ORIGINAL_FILES}}
            with mock.patch.object(matrix, 'run_collector', side_effect=fake_run), mock.patch.object(matrix, 'verify_capture', side_effect=[report, matrix.StudyError('BUDGET retained')]), contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(matrix.execute_batch(out, 'OFFLINE_FAKE'), 2)
            state, study = matrix.read_json(out / 'state.json'), matrix.read_json(out / 'plan.json')
            self.assertEqual(calls, [None] + study['plan'][:2])
            self.assertEqual(len(state['completed']), 1)
            self.assertEqual(state['attempting'], study['plan'][1])
            self.assertIn('BUDGET retained', state['stopped'])
            self.assertFalse(state['complete'])

    def test_analyze_does_not_import_serial_or_claim_incomplete_batch_passed(self):
        original_import = builtins.__import__
        def guarded(name, *args, **kwargs):
            if name == 'serial':
                self.fail('Offline analyze imported serial')
            return original_import(name, *args, **kwargs)
        with tempfile.TemporaryDirectory() as root:
            out = self.prepare(root)
            matrix.write_json(out / 'state.json', {'completed': [], 'complete': False, 'stopped': 'test intentionally incomplete'})
            with mock.patch('builtins.__import__', side_effect=guarded), contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(matrix.main(['--analyze', str(out)]), 2)
            reports = list(out.glob('analysis-*/report.json'))
            self.assertEqual(len(reports), 1)
            self.assertFalse(matrix.read_json(reports[0])['all_planned_capture_checks_passed'])

    def test_partial_analysis_replays_real_capture_and_rejects_later_mutation(self):
        with tempfile.TemporaryDirectory() as root:
            out = self.prepare(root)
            case = matrix.read_json(out / 'plan.json')['plan'][0]
            path = out / 'runs' / 'first-real-copy'
            shutil.copytree(real_path(0), path)
            verified = matrix.verify_capture(path, case, {})
            state = {'completed': [{'case': case, 'capture': str(path.relative_to(out)),
                                    'source_hashes': verified['source_hashes']}],
                     'complete': False, 'stopped': 'intentional partial study'}
            matrix.write_json(out / 'state.json', state)
            _, report = matrix.analyze_batch(out)
            self.assertEqual(report['verified_samples'], 100)
            self.assertEqual(report['verified_runs'], 1)
            self.assertFalse(report['all_planned_capture_checks_passed'])
            with (path / 'raw.jsonl').open('a', encoding='utf-8') as stream:
                stream.write('\n')
            _, changed = matrix.analyze_batch(out)
            self.assertEqual(changed['verified_runs'], 0)
            self.assertTrue(changed['errors'])
            self.assertIn('changed since initial verification', changed['errors'][0])

    def test_reexecuting_batch_rejected_without_overwriting_journal(self):
        with tempfile.TemporaryDirectory() as root:
            out = self.prepare(root)
            state = {'completed': [], 'complete': False, 'stopped': 'must preserve this evidence'}
            matrix.write_json(out / 'state.json', state)
            before = (out / 'state.json').read_bytes()
            with mock.patch.object(matrix, 'run_collector') as run, self.assertRaises(matrix.StudyError):
                matrix.execute_batch(out, 'OFFLINE_FAKE')
            run.assert_not_called()
            self.assertEqual((out / 'state.json').read_bytes(), before)


if __name__ == '__main__':
    unittest.main(verbosity=2)
