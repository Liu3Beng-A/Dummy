"""16.06 repeated short-batch study. Default only prepares a frozen plan.

Only explicit --run --port opens USB for no-motion CBENCH computation. No
flash, motion, automatic retry, cancellation, reconnection or resume. Offline
--analyze replays raw data; neither preparation nor analysis imports serial.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import contextlib
import csv
from datetime import datetime, timezone
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import platform
import re
import shutil
import statistics
import sys

REPO = Path(__file__).resolve().parents[2]
COLLECTOR = REPO / 'tools/tests/run_cartesian_benchmark.py'
COLLECTOR_SHA = '27cc58c80642cada8edcb38add081456cf7ed272653a6be5c6ff4be446fc9a23'
EXPECTED = {'v': '20261016.06', 'type': 'CORE', 'epoch': '1792142213',
            'group': '937E3D839C914097',
            'source_sha256': 'bb13a6d20b93106fa8a4ebb658982f7ec5ec6fddbf264af3a1c1a81763671652'}
ORIGINAL_FILES = ['raw.jsonl', 'session.json', 'summary.json', 'samples.csv',
                  'profile.csv', 'oled.csv', 'collector_source.py']


class StudyError(RuntimeError):
    pass


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def utc_stamp():
    return datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')


def read_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def write_json(path, value):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def require(condition, message):
    if not condition:
        raise StudyError(message)


def expected_identity():
    return dict(EXPECTED)


def make_plan(repeats=5, samples=100):
    require(1 <= repeats <= 10 and 16 <= samples <= 128, 'repeats1..10, samples16..128 required')
    cells = [(f, op) for f in range(3) for op in (3, 4)]
    plan = []
    for round_index in range(repeats):
        order = cells[round_index % 6:] + cells[:round_index % 6]
        if round_index % 2:
            order = list(reversed(order))
        for fixture, op in order:
            index = cells.index((fixture, op))
            modes = ['rate-10hz', 'split-low']
            if (round_index + index) % 2:
                modes.reverse()
            pair = f'r{round_index + 1}-f{fixture}-op{op}'
            for mode in modes:
                plan.append({'ordinal': len(plan) + 1, 'round': round_index + 1,
                             'pair': pair, 'fixture': fixture, 'op': op, 'mode': mode,
                             'work': 5, 'samples': samples, 'budget_us': 20000})
    return plan


def parse_identity(lines):
    pending = [re.fullmatch(r'ok FW_VERSION \[0\] req=(\d+) pending', t) for t in lines]
    requests = [int(m[1]) for m in pending if m]
    require(len(requests) == 1 and requests[0] > 0, 'Missing/ambiguous FW pending request')
    request = requests[0]
    fields, ended = {}, False
    for line in lines:
        m = re.fullmatch(r'FW_VERSION \[0\] req=(\d+) (.+)', line)
        if not m:
            continue
        require(int(m[1]) == request, 'FW request id mismatch')
        if m[2] == 'end':
            require(not ended, 'Duplicate FW end')
            ended = True
            continue
        require(not ended, 'FW field after end')
        for item in m[2].split():
            key, value = item.split('=', 1)
            require(key not in fields, 'Duplicate FW field')
            fields[key] = value
    require(ended and set(fields) == {'v', 'type', 'epoch', 'group', 'h0', 'h1'}, 'Incomplete FW identity')
    result = {k: fields[k] for k in ('v', 'type', 'epoch', 'group')}
    result['source_sha256'] = (fields['h0'] + fields['h1']).lower()
    require(result == EXPECTED, 'Board identity is not the frozen16.06 release; no RUN admitted')
    return result


def check_motion_snapshot(lines):
    headers = [re.fullmatch(r'MOTION id=\d+ kind=\S+ phase=(\d+) mode=\d+ ms=\d+', t) for t in lines]
    phases = [int(m[1]) for m in headers if m]
    states = [re.fullmatch(r'MOTION plan_ms=\S+ moving=([0-9A-Fa-f]+) txok=[0-9A-Fa-f]+ pending=([0-9A-Fa-f]+)', t) for t in lines]
    states = [tuple(int(v, 16) for v in m.groups()) for m in states if m]
    # "moving" is motionMovingMask: axes with nonzero planned displacement in
    # the last motion task. Firmware retains it after completion/cancellation;
    # it is not a live IsMoving() or StopState indication.
    require(len(phases) == 1 and phases[0] in (0, 3, 4, 5) and len(states) == 1
            and (states[0][0] & ~0xFE) == 0 and states[0][1] == 0,
            'MOTION snapshot is active/incomplete; no state-changing command will be sent')
    return {'phase': phases[0], 'moving': states[0][0], 'pending': 0,
            'moving_semantics': 'historical planned-displacement axis mask; not live motion',
            'disabled_proven_by_host': False,
            'note': 'MOTION phase/mask do not prove physical rest or StopState. Firmware RUN atomically enforces idle DISABLED.'}


def load_collector(path):
    require(digest(path) == COLLECTOR_SHA, 'Collector source differs from reviewed16.06 collector')
    name = 'matrix_collector_' + utc_stamp()
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def run_command(case):
    mode = {'rate-10hz': 2, 'split-same': 3, 'split-low': 4}[case['mode']]
    return f"#CBENCH RUN {case['op']},{case['fixture']},{case['work']},{case['samples']},{case['budget_us']},1,{mode}"


def guarded_session_class(bench, case, events):
    base = bench.SafeSession

    class GuardedSession(base):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self.matrix_identity = None
            self.matrix_motion = None

        def request(self, command):
            if command.startswith('#CBENCH RUN'):
                require(case is not None and command == run_command(case), 'Unexpected RUN command')
                require(self.matrix_identity == EXPECTED and self.matrix_motion is not None,
                        'Same-session identity/motion screening required before RUN')
                job = self.parser.jobs.get(self.parser.current_id, {})
                profile, oled = job.get('profile_meta', {}), job.get('oled_meta', {})
                require(job.get('phase') in bench.TERMINAL and job.get('contract') == {'no_motion': 1, 'scope': 'components'},
                        'Prior benchmark active or no-motion contract missing')
                cold_mapping = (job.get('phase') == 'IDLE' and profile.get('mode') == 0
                                and profile.get('n') == 0 and profile.get('mapmask') == 0)
                require(profile.get('version') == 2 and profile.get('tasks') == 7
                        and profile.get('samples') == 128 and profile.get('work') == 5
                        and profile.get('irq_included') == 1 and (profile.get('mapmask') == 63 or cold_mapping),
                        'Seven-task profile capability required')
                require(oled.get('version') == 3 and oled.get('lanes') == 2 and oled.get('stages') == 5
                        and oled.get('samples') == 128,
                        'Two-lane OLED capability required')
                event = {'kind': 'run_admitted', 'command': command, 'identity': self.matrix_identity,
                         'motion': self.matrix_motion, 'firmware_idle_disabled_gate': 'native RUN atomic guard'}
                events.append(event)
                self.recorder.record('matrix_guard', **event)
            result = super().request(command)
            if command == '#FW_VERSION 0':
                self.matrix_identity = parse_identity(result)
                events.append({'kind': 'identity_verified', 'identity': self.matrix_identity})
            elif command == '#MOTION_STATUS':
                self.matrix_motion = check_motion_snapshot(result)
                events.append({'kind': 'motion_snapshot', **self.matrix_motion})
            return result

    return GuardedSession


def csv_matches(path, expected):
    with path.open(encoding='utf-8', newline='') as stream:
        reader = csv.DictReader(stream)
        fields = reader.fieldnames
        actual = list(reader)
    require(bool(expected), f'Expected populated CSV: {path.name}')
    required = set().union(*(r.keys() for r in expected))
    if path.name == 'profile.csv':
        required.add('task_oled_cycles')  # Deliberately retained empty legacy column in v2 exports.
    require(fields is not None and len(fields) == len(set(fields)) and set(fields) == required,
            f'CSV header incomplete/duplicated: {path.name}')
    require(len(actual) == len(expected), f'CSV row count mismatch: {path.name}')
    for row, source in zip(actual, expected):
        calculated = {k: '' if source.get(k) is None else str(source[k]) for k in fields}
        require(row == calculated, f'Raw/CSV mismatch: {path.name}')


def verify_capture(path, case, references):
    path = Path(path)
    before = {name: digest(path / name) for name in ORIGINAL_FILES}
    bench = load_collector(path / 'collector_source.py')
    events = [json.loads(t) for t in (path / 'raw.jsonl').read_text(encoding='utf-8').splitlines()]
    rx = [e['line'] for e in events if e.get('direction') == 'rx']
    tx = [e['command'] for e in events if e.get('direction') == 'tx']
    identity = parse_identity(rx)
    require([t for t in tx if t.startswith('#CBENCH RUN')] == [run_command(case)], 'RUN count/config mismatch')
    run_position = next(i for i, e in enumerate(events)
                        if e.get('direction') == 'tx' and e.get('command') == run_command(case))
    require(parse_identity([e['line'] for e in events[:run_position] if e.get('direction') == 'rx']) == identity,
            'Full identity must be received before RUN')
    require(all(bench.command_allowed(t) for t in tx) and '#CBENCH CANCEL' not in tx, 'Unexpected command')
    session, saved = read_json(path / 'session.json'), read_json(path / 'summary.json')
    require(session['error'] is None and session['hardware_opened'] and session['compute_requested']
            and not session['motion_commands_permitted'], 'Capture session failed or not explicit compute')
    require(session['commands'] == tx and session['script_sha256'] == COLLECTOR_SHA, 'Session/raw provenance mismatch')
    require(session['oled_requested_mode'] == case['mode'] and session['profile_requested'], 'Request mode mismatch')
    parser = bench.CaptureParser()
    for line in rx:
        parser.feed(line)
    require(saved.get('id') in parser.jobs, 'Final job missing from raw')
    job = parser.jobs[saved['id']]
    job['oled_request_mode'] = case['mode']
    result, rows = parser.summarize(saved['id'])
    require(saved == {**result, 'session': session}, 'Raw/summary full export mismatch')
    for key in ('id', 'phase', 'config', 'clock', 'dwt', 'contract', 'collection_complete', 'job_success',
                'kernel_success', 'partial_work_samples', 'errors', 'timing_full_work'):
        require(result.get(key) == saved.get(key), f'Raw/summary mismatch: {key}')
    require(result['phase'] == 'DONE' and result['job_success'] and result['kernel_success']
            and result['collection_complete'] and result['partial_work_samples'] == 0 and not result['errors'],
            'Job did not finish all work; exit0 alone is not success')
    require(result['config'] == {k: case[k] for k in ('op', 'fixture', 'work', 'samples')}, 'Configuration mismatch')
    require(result['clock'] == {'clock_hz': 168000000, 'budget_us': 20000, 'n': case['samples'], 'period_ms': 20},
            'Clock/budget/count mismatch')
    require(result['contract'] == {'no_motion': 1, 'scope': 'components'}, 'No-motion contract missing')
    require(result['dwt']['dwt_ctrl'] & 1 and result['dwt']['demcr'] & (1 << 24), 'DWT not enabled')
    profile, oled = result['profile'], result['oled']
    require(profile['metadata']['version'] == 2 and profile['metadata']['tasks'] == 7
            and profile['metadata']['mapmask'] == 63 and profile['metadata']['irq_included'] == 1,
            'Profile capability/accounting changed')
    for lane in (profile, oled):
        require(lane['sample_count'] == case['samples'] and lane['records_complete'] and lane['accounting_valid']
                and not lane['errors'] and not lane['nonzero_flags'], 'Profile/OLED records invalid')
    meta = oled['metadata']
    require(meta['version'] == 3 and meta['lanes'] == 2 and meta['stages'] == 5
            and meta['mode'] == bench.OLED_MODES[case['mode']], 'OLED mode/schema mismatch')
    require(meta['runtime_flags'] == 0 and meta['restored'] == 1
            and meta['applied_priority'] == (16 if case['mode'] == 'split-low' else 24), 'Handoff/restore/priority failed')
    require(oled['policy_status'] == 'observed' and oled['runtime_valid'], 'OLED policy evidence insufficient/invalid')
    if case['mode'] == 'split-low':
        require(oled['split_test_passed'], 'Split diagnostic evidence incomplete')
    require(profile['metadata'] == saved['profile']['metadata'] and meta == saved['oled']['metadata'],
            'Raw/summary metadata mismatch')
    require(len(rows) == case['samples'] and [r['i'] for r in rows] == list(range(case['samples'])), 'Samples missing/duplicated')
    vectors = set()
    for row in rows:
        require(row['full_work'] and row['pair_complete'] and row['flag'] == 0 and row['work'] == 5,
                'Partial/failed component work')
        require(row['fk'] == 10 and row['ik'] == 5 and row['selected'] == 5 and row['failed'] == 0,
                'Numerical solve failed despite job completion')
        require(5 <= row['valid'] <= 40, 'Valid-branch count inconsistent with five successful solves')
        vectors.add(tuple(row[k] for k in ('fk', 'ik', 'valid', 'selected', 'failed', 'checksum')))
    require(len(vectors) == 1, 'Repeated fixed input produced different numerical results')
    signature = next(iter(vectors))
    key = (case['op'], case['fixture'], case['work'])
    if key in references:
        require(tuple(references[key]) == signature, 'Numerical result changed between modes/repetitions')
    csv_matches(path / 'samples.csv', rows)
    csv_matches(path / 'profile.csv', parser.profile_summary(job)[1])
    csv_matches(path / 'oled.csv', parser.oled_summary(job)[1])
    require(all(digest(path / name) == value for name, value in before.items()), 'Original capture changed during verification')
    references[key] = signature
    timings = [r['us'] for r in rows]
    return {'case': case, 'capture': str(path), 'identity': identity, 'job': saved['id'],
            'numeric_signature': list(signature), 'samples': len(rows),
            'mean_us': statistics.mean(timings), 'timing': result['timing_full_work'],
            'late_ticks': profile['late_ticks'], 'oled': meta, 'source_hashes': before,
            'timings_us': timings, 'capture_checks_passed': True,
            'performance_acceptance': False, 'stack_lines': [t for t in rx if t.startswith('ok STACK ')]}


def prepare_batch(output_root, plan):
    require(digest(COLLECTOR) == COLLECTOR_SHA, 'Current collector changed; review before preparing a16.06 study')
    out = Path(output_root) / ('cartesian-benchmark-matrix-' + utc_stamp())
    out.mkdir(parents=True, exist_ok=False)
    frozen = out / 'source/tools/tests'
    frozen.mkdir(parents=True)
    for path in (COLLECTOR, Path(__file__).resolve()):
        shutil.copy2(path, frozen / path.name)
    identity_files = ['Core-STM32F4-fw.manifest.txt', 'CURRENT_RELEASE.txt', 'verification.json']
    for name in identity_files:
        dest = out / 'firmware-reference' / name
        dest.parent.mkdir(exist_ok=True)
        shutil.copy2(REPO / 'firmware_artifacts' / name, dest)
    verification = read_json(out / 'firmware-reference/verification.json')
    require(verification['release'] == '2026101606' and verification['buildset'] == EXPECTED['group']
            and verification['source_sha256'] == EXPECTED['source_sha256'], 'Current products not reviewed16.06')
    study = {'schema': 1, 'kind': 'repeated_short_batches', 'identity': EXPECTED, 'plan': plan,
             'planned_runs': len(plan), 'planned_samples': sum(c['samples'] for c in plan),
             'planned_work': sum(c['samples'] * c['work'] for c in plan),
             'order': 'Deterministic rotating cells and alternatingAB/BA; no claim of randomization.',
             'collector_sha256': COLLECTOR_SHA, 'matrix_sha256': digest(Path(__file__)),
             'source_files': {p.relative_to(out).as_posix(): digest(p) for p in out.rglob('*') if p.is_file()},
             'python': sys.version, 'platform': platform.platform(), 'execute_requested': False,
             'limits': '100samples about2s; repeated CLI is not continuous long-run testing. Firmware RUN enforces idle DISABLED.'}
    write_json(out / 'plan.json', study)
    return out


def check_frozen(out):
    study = read_json(out / 'plan.json')
    require(study['identity'] == EXPECTED, 'Plan identity changed')
    require(digest(Path(__file__)) == study['matrix_sha256'], 'Use the matrix source frozen for this batch')
    require(study['plan'] == make_plan(max(c['round'] for c in study['plan']), study['plan'][0]['samples']), 'Plan/order changed')
    require(study['schema'] == 1 and study['kind'] == 'repeated_short_batches'
            and study['planned_runs'] == len(study['plan'])
            and study['planned_samples'] == sum(c['samples'] for c in study['plan'])
            and study['planned_work'] == sum(c['samples'] * c['work'] for c in study['plan']), 'Plan totals/schema changed')
    for name, value in study['source_files'].items():
        require(digest(out / name) == value, f'Frozen source/reference changed: {name}')
    return study


def run_collector(out, destination, port, case):
    destination.mkdir(parents=True, exist_ok=False)
    bench = load_collector(out / 'source/tools/tests/run_cartesian_benchmark.py')
    events = []
    bench.SafeSession = guarded_session_class(bench, case, events)
    args = ['--port', port, '--output-root', str(destination)]
    if case is not None:
        args += ['--run', '--profile', '--oled-mode', case['mode'], '--op', str(case['op']),
                 '--fixture', str(case['fixture']), '--work', '5', '--samples', str(case['samples']), '--budget-us', '20000']
    stdout, stderr = io.StringIO(), io.StringIO()
    try:
        with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
            code = bench.main(args)
    finally:
        (destination / 'collector.stdout.txt').write_text(stdout.getvalue(), encoding='utf-8')
        (destination / 'collector.stderr.txt').write_text(stderr.getvalue(), encoding='utf-8')
        write_json(destination / 'admission.json', events)
    captures = list(destination.glob('cartesian-benchmark-*'))
    require(code == 0 and len(captures) == 1, f'Collector stopped (exit{code}); preserve {destination}')
    require(any(e['kind'] == 'identity_verified' for e in events), 'Same-session identity guard did not run')
    if case is not None:
        require(sum(e['kind'] == 'run_admitted' for e in events) == 1, 'Expected exactly one preRUN admission')
    else:
        require(not any(e['kind'] == 'run_admitted' for e in events), 'Preflight must not run computation')
    return captures[0]


def execute_batch(out, port):
    study = check_frozen(out)
    require(not any((out / name).exists() for name in ('state.json', 'preflight', 'runs')),
            'Batch already attempted; no resume or overwrite. Keep this evidence and prepare a fresh batch.')
    state = {'schema': 1, 'execute_requested': True, 'planned_runs': len(study['plan']),
             'completed': [], 'stopped': None, 'complete': False,
             'no_automatic_retry_cancel_reconnect_resume': True}
    references = {}
    write_json(out / 'state.json', state)
    try:
        print('Read-only preflight; idle DISABLED required. No START/motion/flash.', flush=True)
        state['preflight'] = str(run_collector(out, out / 'preflight', port, None))
        write_json(out / 'state.json', state)
        for case in study['plan']:
            check_frozen(out)
            state['attempting'] = case
            write_json(out / 'state.json', state)
            name = f"{case['ordinal']:03d}-r{case['round']}-f{case['fixture']}-op{case['op']}-{case['mode']}"
            print(f"[{case['ordinal']}/{len(study['plan'])}] {name}", flush=True)
            capture = run_collector(out, out / 'runs' / name, port, case)
            report = verify_capture(capture, case, references)
            write_json(out / 'runs' / name / 'verified.json', report)
            state['completed'].append({'case': case, 'capture': str(capture.relative_to(out)),
                                       'source_hashes': report['source_hashes']})
            write_json(out / 'state.json', state)
            print(f"  DONE: mean{report['mean_us']/1000:.3f}ms max{report['timing']['max_us']/1000:.3f}ms", flush=True)
        state['complete'] = True
        state.pop('attempting', None)
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        state['stopped'] = f'{type(exc).__name__}: {exc}'
        print('STOP: ' + state['stopped'], file=sys.stderr, flush=True)
        return 2
    finally:
        write_json(out / 'state.json', state)


def analyze_batch(out):
    study = check_frozen(out)
    state = read_json(out / 'state.json')
    references, reports, errors = {}, [], []
    try:
        for entry in state['completed']:
            require(entry['case'] == study['plan'][len(reports)], 'Completed order differs from frozen plan')
            require(set(entry['source_hashes']) == set(ORIGINAL_FILES), 'Capture hash journal incomplete')
            for name, value in entry['source_hashes'].items():
                require(digest(out / entry['capture'] / name) == value, 'Capture changed since initial verification')
            reports.append(verify_capture(out / entry['capture'], entry['case'], references))
    except Exception as exc:
        errors.append(f'{type(exc).__name__}: {exc}')
    grouped = defaultdict(list)
    for report in reports:
        c = report['case']
        grouped[(c['fixture'], c['op'], c['mode'])].append(report)
    groups = []
    for (fixture, op, mode), group in sorted(grouped.items()):
        values = [x for r in group for x in r['timings_us']]
        groups.append({'fixture': fixture, 'op': op, 'mode': mode, 'runs': len(group), 'samples': len(values),
                       'mean_us': statistics.mean(values), 'observed_max_us': max(values),
                       'per_run_p95_us': [r['timing']['p95_us'] for r in group],
                       'compute_late_samples': sum(r['late_ticks']['late_samples'] for r in group),
                       'max_worker_loop_gap_ticks': max(r['oled']['worker_max_loop_gap_ticks'] for r in group),
                       'max_send_gap_ticks': max(r['oled']['max_send_gap_ticks'] for r in group),
                       'max_snapshot_age_ticks': max(r['oled']['max_snapshot_age_ticks'] for r in group),
                       'display_stack_min_free': min(r['oled']['display_stack_free'] for r in group)})
    complete = state['complete'] and not state['stopped'] and not errors and len(reports) == len(study['plan'])
    report = {'all_planned_capture_checks_passed': complete, 'performance_acceptance': False,
              'study_kind': 'repeated_short_batches', 'planned_runs': len(study['plan']), 'verified_runs': len(reports),
              'verified_samples': sum(r['samples'] for r in reports), 'state_stop': state['stopped'],
              'errors': errors, 'groups': groups,
              'limits': 'Observed component timing only. Cumulative stack watermarks. No continuous long-run, full7axis or motion acceptance.'}
    target = out / ('analysis-' + utc_stamp())
    target.mkdir()
    write_json(target / 'report.json', report)
    write_json(target / 'verified-captures.json', reports)
    write_json(target / 'evidence-index.json', {p.relative_to(out).as_posix(): digest(p)
               for p in out.rglob('*') if p.is_file() and target not in p.parents})
    return target, report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', action='store_true', help='Explicit no-motion USB study; never flashes or sends motor commands')
    parser.add_argument('--port', help='Required only with --run; no automatic port discovery')
    parser.add_argument('--analyze', type=Path, help='Offline replay of an existing matrix batch; never opens a device')
    parser.add_argument('--repeats', type=int, default=5)
    parser.add_argument('--samples', type=int, default=100)
    parser.add_argument('--output-root', type=Path, default=REPO / 'firmware_artifacts')
    args = parser.parse_args(argv)
    if args.analyze:
        if args.run or args.port:
            parser.error('--analyze cannot open hardware')
        target, report = analyze_batch(args.analyze.resolve())
        print(target)
        return 0 if report['all_planned_capture_checks_passed'] else 2
    if bool(args.port) != args.run:
        parser.error('Explicit --run and --port must be supplied together')
    out = prepare_batch(args.output_root.resolve(), make_plan(args.repeats, args.samples))
    print(out, flush=True)
    if not args.run:
        print('PLAN ONLY: no device opened. Full default study60runs/6000samples, not continuous long-run.')
        return 0
    code = execute_batch(out, args.port)
    target, report = analyze_batch(out)
    print(target, flush=True)
    return code if code else (0 if report['all_planned_capture_checks_passed'] else 2)


if __name__ == '__main__':
    raise SystemExit(main())
