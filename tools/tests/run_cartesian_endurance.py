"""Bounded continuous F405 component diagnostic, never a motion controller.

Default writes a frozen plan without importing serial. --port alone reads
identity/capabilities. Only --run --port requests LONG computation. No flash,
motion, reset, automatic cancellation, retry, reconnect or resume is permitted.
--analyze DIR replays saved raw responses without opening a device.
Exit0 indicates a completed read/action, never performance acceptance. Exit2
preserves a collection error; exit3 retains an incomplete/failed compute result.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import platform
import re
import shutil
import sys
import time


REPO = Path(__file__).resolve().parents[2]
LEGACY = REPO / 'tools/tests/run_cartesian_benchmark.py'
LEGACY_SHA = '27cc58c80642cada8edcb38add081456cf7ed272653a6be5c6ff4be446fc9a23'
RELEASE = '2026101701'
TERMINAL = {'IDLE', 'DONE', 'CANCELLED', 'ABORTED', 'BUDGET', 'CLOCK_ERROR'}
REASONS = ('NONE', 'NUMERIC', 'SIGNATURE', 'ACCOUNTING', 'DEADLINE', 'CLOCK',
           'BUDGET', 'CANCELLED', 'ENVIRONMENT', 'RESTORE', 'SETUP')
OLD_QUERIES = ('#FW_VERSION 0', '#KIN_CONFIG', '#TOOL', '#MOTION_STATUS', '#STACK', '#CBENCH')
UINT32_MAX = (1 << 32) - 1


class ProtocolError(RuntimeError):
    pass


def require(condition, text):
    if not condition:
        raise ProtocolError(text)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def stamp():
    return datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')


def write_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def read_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def load_legacy(path=LEGACY):
    require(sha(path) == LEGACY_SHA, 'Reviewed legacy collector source hash differs')
    spec = importlib.util.spec_from_file_location('endurance_legacy_' + stamp(), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def manifest_identity(path):
    values = {}
    for line in Path(path).read_text(encoding='utf-8').splitlines():
        if line.startswith('['):
            break
        if '=' in line:
            key, value = line.split('=', 1)
            require(key not in values, 'Duplicate manifest field')
            values[key] = value
    require(values.get('release') == RELEASE and values.get('project') == 'Core-STM32F4-fw',
            'Manifest must identify reviewed Core release2026101701')
    require(re.fullmatch('[0-9A-Fa-f]{16}', values.get('buildset', '')) is not None
            and re.fullmatch('[0-9a-fA-F]{64}', values.get('source_sha256', '')) is not None
            and values.get('utc_epoch', '').isdigit(), 'Incomplete manifest identity')
    return {'v': RELEASE[:8] + '.' + RELEASE[8:], 'type': 'CORE',
            'epoch': values['utc_epoch'], 'group': values['buildset'].upper(),
            'source_sha256': values['source_sha256'].lower()}


def parse_identity(lines, expected):
    pending = [re.fullmatch(r'ok FW_VERSION \[0\] req=(\d+) pending', t) for t in lines]
    ids = [int(m[1]) for m in pending if m]
    require(len(ids) == 1 and ids[0] > 0, 'Missing/ambiguous FW identity request')
    fields, ended = {}, False
    for line in lines:
        match = re.fullmatch(r'FW_VERSION \[0\] req=(\d+) (.+)', line)
        if match is None:
            continue
        require(int(match[1]) == ids[0], 'FW identity request changed')
        if match[2] == 'end':
            require(not ended, 'Duplicate FW end')
            ended = True
            continue
        require(not ended, 'FW field after end')
        for item in match[2].split():
            require('=' in item, 'Malformed FW identity field')
            key, value = item.split('=', 1)
            require(key not in fields, 'Duplicate FW identity field')
            fields[key] = value
    require(ended and set(fields) == {'v', 'type', 'epoch', 'group', 'h0', 'h1'}, 'Incomplete FW identity')
    actual = {key: fields[key] for key in ('v', 'type', 'epoch')}
    actual.update(group=fields['group'].upper(), source_sha256=(fields['h0'] + fields['h1']).lower())
    require(actual == expected, 'Board does not match frozen17.01 manifest; LONG was not admitted')
    return actual


def motion_snapshot(lines):
    headers = [re.fullmatch(r'MOTION id=\d+ kind=\S+ phase=(\d+) mode=\d+ ms=\d+', t) for t in lines]
    phases = [int(m[1]) for m in headers if m]
    masks = [re.fullmatch(r'MOTION plan_ms=\S+ moving=([0-9A-Fa-f]+) txok=[0-9A-Fa-f]+ pending=([0-9A-Fa-f]+)', t) for t in lines]
    masks = [tuple(int(v, 16) for v in m.groups()) for m in masks if m]
    require(len(phases) == 1 and phases[0] in (0, 3, 4, 5) and len(masks) == 1
            and masks[0][0] & ~0xFE == 0 and masks[0][1] == 0, 'MOTION history is active/incomplete')
    return {'phase': phases[0], 'historical_displacement_mask': masks[0][0], 'pending': 0,
            'disabled_proven_by_host': False,
            'note': 'Masks are historical task data. Firmware LONG atomically enforces idle DISABLED.'}


def uints(value, size):
    items = value.split(',')
    require(len(items) == size and all(re.fullmatch(r'[0-9]{1,10}', t) for t in items), 'Malformed numeric record')
    result = tuple(map(int, items))
    require(all(n <= UINT32_MAX for n in result), 'Numeric record exceeds uint32')
    return result


def u64(hi, lo):
    return (hi << 32) | lo


def request_config(op=4, fixture=2, seconds=600, mode='split-low'):
    require(op in (3, 4) and fixture in range(3) and 1 <= seconds <= 600
            and mode in ('rate-10hz', 'split-low'), 'op3/4, fixture0..2, seconds1..600, OLED mode2/4 required')
    return {'op': op, 'fixture': fixture, 'work': 5, 'seconds': seconds,
            'budget_us': 20000, 'mode': 4 if mode == 'split-low' else 2, 'planned': seconds * 50}


def long_command(config):
    return '#CBENCH LONG ' + ','.join(str(config[k]) for k in ('op', 'fixture', 'work', 'seconds', 'budget_us', 'mode'))


def command_allowed(command):
    if command in OLD_QUERIES or command in ('#CBENCH LONG', '#CBENCH LREAD', '#CBENCH OLED', '#CBENCH CANCEL'):
        return True
    match = re.fullmatch(r'#CBENCH LONG (\d+),(\d+),5,(\d+),20000,([24])', command)
    return bool(match and int(match[1]) in (3, 4) and int(match[2]) in range(3) and 1 <= int(match[3]) <= 600)


class Boundary:
    def __init__(self, command, prefix):
        self.command, self.prefix = command, prefix

    def related(self, line):
        return line.startswith(self.prefix)

    def trailing(self, line):
        require(not line.startswith('error ') and not self.related(line), 'Unexpected trailing response: ' + line)


class LongStatus(Boundary):
    PATTERNS = [
        r'CBLONG v=1 id=(\d+) phase=(IDLE|PENDING|RUNNING|DONE|CANCELLED|ABORTED|BUDGET|CLOCK_ERROR)',
        r'CBLONG caps seconds=600 period_ms=20 work=5',
        r'CBLONG caps bins=21 trends=60 history=8',
        r'CBLONG cfg op=(\d+) fixture=(\d+) mode=(\d+) seconds=(\d+)',
        r'CBLONG cfg work=5 budget_us=20000 planned=(\d+)',
        r'CBLONG clock hz=(\d+) tick_hz=1000',
        r'CBLONG run n=(\d+) elapsed_ms=(\d+)',
        r'CBLONG stop reason=([A-Z]+)',
        r'CBLONG no_motion=1 scope=components',
        r'CBLONG end id=(\d+)',
    ]

    def __init__(self, command='#CBENCH LONG', expected_id=None, config=None, previous_id=None):
        super().__init__(command, 'CBLONG ')
        self.index, self.values = 0, {}
        self.expected_id, self.config, self.previous_id = expected_id, config, previous_id

    def feed(self, line):
        require(not line.startswith('error '), line)
        if not self.related(line):
            return False
        require(self.index < len(self.PATTERNS), 'Status data after end')
        match = re.fullmatch(self.PATTERNS[self.index], line)
        require(match is not None, f'Incomplete/out-of-order LONG status line{self.index}: {line}')
        if self.index == 0:
            self.values.update(id=int(match[1]), phase=match[2])
            require(self.values['id'] <= UINT32_MAX, 'LONG id exceeds uint32')
            if self.expected_id is not None:
                require(self.values['id'] == self.expected_id, 'LONG job id changed')
            if self.previous_id is not None:
                require(self.values['id'] != self.previous_id, 'LONG did not return a new id')
        elif self.index == 3:
            self.values.update(zip(('op', 'fixture', 'mode', 'seconds'), map(int, match.groups())))
        elif self.index == 4:
            self.values.update(work=5, budget_us=20000, planned=int(match[1]))
        elif self.index == 5:
            self.values.update(hz=int(match[1]), tick_hz=1000)
        elif self.index == 6:
            self.values.update(zip(('n', 'elapsed_ms'), map(int, match.groups())))
        elif self.index == 7:
            require(match[1] in REASONS, 'Unknown LONG stop reason')
            self.values['reason'] = match[1]
        elif self.index == 9:
            require(int(match[1]) == self.values['id'], 'LONG end id mismatch')
        self.index += 1
        if self.index != len(self.PATTERNS):
            return False
        if self.values['phase'] != 'IDLE':
            config = request_config(self.values['op'], self.values['fixture'], self.values['seconds'],
                                    'split-low' if self.values['mode'] == 4 else 'rate-10hz')
            require(self.values['mode'] in (2, 4) and all(self.values[k] == v for k, v in config.items()), 'LONG configuration invalid')
            require(self.values['n'] <= self.values['planned'], 'LONG count exceeds plan')
        else:
            require(all(self.values[k] == 0 for k in ('id', 'op', 'fixture', 'mode', 'seconds', 'planned', 'hz', 'n', 'elapsed_ms'))
                    and self.values['reason'] == 'NONE', 'Malformed cold LONG status')
        if self.config is not None:
            require(all(self.values[k] == value for k, value in self.config.items()), 'LONG differs from requested configuration')
        require(all(not isinstance(value, int) or 0 <= value <= UINT32_MAX for value in self.values.values()),
                'LONG numeric metadata exceeds uint32')
        return True


class LongRead(Boundary):
    HEADERS = [('total', 3), ('sum', 3), ('window', 4), ('dwt', 3), ('minmax', 3),
               ('late', 4), ('error', 3), ('sig', 6), ('hash', 2), ('counts', 4),
               ('overhead', 4), ('scan', 3)]

    def __init__(self, status):
        super().__init__('#CBENCH LREAD', 'CBLONG ')
        self.status, self.index = status, 0
        self.values = {'histogram': [], 'tasks': [], 'oled': [], 'trends': [], 'history': []}
        self.finished = False

    def feed(self, line):
        require(not line.startswith('error '), line)
        if not self.related(line):
            return False
        require(not self.finished, 'LREAD record after end')
        if self.index == 0:
            require(line == f"CBLONG read v=1 id={self.status['id']}", 'LREAD header id/version mismatch')
        elif self.index <= len(self.HEADERS):
            name, count = self.HEADERS[self.index - 1]
            match = re.fullmatch(r'CBLONG ' + name + r' ([0-9,]+)', line)
            require(match is not None, 'Missing/out-of-order LREAD ' + name)
            values = uints(match[1], count)
            require(values[0] == self.status['id'], 'LREAD record id mismatch')
            self.values[name] = list(values[1:])
            if name == 'counts':
                require(values[1] == 21 and values[2] <= 60 and values[3] <= 8, 'LREAD capacities exceed bounded storage')
        else:
            bins, trends, history = self.values['counts']
            position = self.index - 1 - len(self.HEADERS)
            schedule = [('H', [i], 3) for i in range(bins)]
            schedule += [('P', [i], 4) for i in range(7)]
            schedule += [('O', [lane, stage], 5) for lane in range(4) for stage in range(5)]
            schedule += [(kind, [i], count) for i in range(trends) for kind, count in [('T', 5), ('U', 5), ('K', 5)]]
            schedule += [(kind, [i], count) for i in range(history) for kind, count in [('F', 5), ('G', 5), ('J', 4)]]
            if position == len(schedule):
                require(line == f"CBLONG read_end id={self.status['id']}", 'LREAD end missing/wrong id')
                self.finished = True
                self.validate()
                return True
            require(position < len(schedule), 'Extra LREAD record')
            kind, prefix, count = schedule[position]
            match = re.fullmatch(r'CBLONG ' + kind + r' ([0-9,]+)', line)
            require(match is not None, 'Missing/out-of-order LREAD ' + kind)
            values = uints(match[1], count)
            require(values[0] == self.status['id'] and list(values[1:1 + len(prefix)]) == prefix, 'LREAD index/id mismatch')
            payload = values[1 + len(prefix):]
            if kind == 'H':
                self.values['histogram'].append(payload[0])
            elif kind == 'P':
                self.values['tasks'].append(u64(*payload))
            elif kind == 'O':
                self.values['oled'].append(u64(*payload))
            else:
                key = 'trends' if kind in 'TUK' else 'history'
                if kind in ('T', 'F'):
                    self.values[key].append({'index': prefix[0]})
                self.values[key][-1][kind] = list(payload)
        self.index += 1
        return False

    def validate(self):
        value, status = self.values, self.status
        n, elapsed = value['total']
        require(n == status['n'] and elapsed == status['elapsed_ms'], 'LREAD total differs from terminal status')
        total, window = u64(*value['sum']), u64(*value['window'][:2])
        low, high = value['minmax']
        require(sum(value['histogram']) == n and sum(value['tasks']) == window, 'Histogram/task accounting mismatch')
        require(total <= window and low <= high and low * n <= total <= high * n, 'Duration aggregates inconsistent')
        require(high <= value['window'][2] and (n != 0 or not any((low, high, total, window))), 'Invalid extrema/empty aggregate')
        lanes = [value['oled'][i:i + 5] for i in range(0, 20, 5)]
        require(sum(lanes[0]) == value['tasks'][2] and sum(lanes[2]) == value['tasks'][5]
                and sum(lanes[1]) == window and sum(lanes[3]) == window, 'OLED/task accounting mismatch')
        require(all(a <= b for resident, full in ((lanes[0], lanes[1]), (lanes[2], lanes[3]))
                    for a, b in zip(resident, full)), 'OLED stage residence exceeds elapsed')
        require(len(value['trends']) == math.ceil(n / 500) and len(value['history']) == min(n, 8), 'Trend/history coverage mismatch')
        require(sum(t['T'][0] for t in value['trends']) == n
                and sum(u64(*t['T'][1:]) for t in value['trends']) == total, 'Trend totals inconsistent')
        for i, trend in enumerate(value['trends']):
            require(trend['T'][0] == min(500, n - i * 500) and trend['U'][0] <= high,
                    'Trend count/extrema mismatch')
        if n:
            require(max(t['U'][0] for t in value['trends']) == high, 'Trend/sample maximum mismatch')
        require(value['late'][0] <= n and value['error'][0] < len(REASONS), 'Invalid late/error count')
        require((value['error'][0] == 0 and value['error'][1] == UINT32_MAX)
                or (value['error'][0] != 0 and value['error'][1] <= n), 'Invalid first-error sequence')
        require((value['late'][0] == 0) == (value['late'][1] == 0), 'Late count/maximum mismatch')
        if n:
            require(max(t['U'][1] for t in value['trends']) == value['late'][1]
                    and max(t['U'][2] for t in value['trends']) == value['late'][2], 'Trend late/gap maximum mismatch')
            require(all(0 <= free <= size for trend in value['trends'] for free, size in zip(trend['K'], (8192, 2000, 2048))),
                    'Stack watermark exceeds configured task stack')
        if n and status['hz']:
            occupied = [i for i, count in enumerate(value['histogram']) if count]
            require(occupied[0] == min(20, low * 1000 // status['hz'])
                    and occupied[-1] == min(20, high * 1000 // status['hz']), 'Histogram extrema mismatch')
            if status['reason'] != 'CLOCK' and value['error'][0] != 5:
                require(u64(*value['dwt']) >= window, 'Observed DWT span cannot contain profile windows')
        previous_elapsed = -1
        for i, frame in enumerate(value['history']):
            require(frame['F'][0] == n - len(value['history']) + i and frame['F'][1] <= elapsed
                    and frame['F'][1] >= previous_elapsed and frame['G'][1] <= frame['F'][2], 'History sequence/time/window mismatch')
            previous_elapsed = frame['F'][1]


class OledBoundary(Boundary):
    def __init__(self, bench, job_id):
        super().__init__('#CBENCH OLED', 'CBOLED ')
        self.metadata = bench.OledMetadata(job_id)

    def feed(self, line):
        require(not line.startswith('error '), line)
        return self.metadata.feed(line) if self.related(line) else False


def session_class(bench):
    class Session(bench.SafeSession):
        def __init__(self, port, recorder, timeout, expected, config, metadata):
            super().__init__(port, recorder, timeout, bench.CaptureParser())
            self.expected, self.config, self.metadata = expected, config, metadata
            self.identity, self.motion, self.status, self.oled, self.stats = None, None, None, None, None

        def request(self, command):
            require(command_allowed(command), 'Command denied by endurance allowlist')
            if command in OLD_QUERIES:
                self.metadata['commands'].append(command)
                rows = super().request(command)
                if command == '#FW_VERSION 0':
                    self.identity = parse_identity(rows, self.expected)
                elif command == '#MOTION_STATUS':
                    self.motion = motion_snapshot(rows)
                return rows
            if command.startswith('#CBENCH LONG '):
                require(command == long_command(self.config), 'Unexpected LONG configuration')
                require(self.identity == self.expected and self.motion is not None and self.status is not None
                        and self.status['phase'] in TERMINAL and self.oled is not None
                        and self.oled['version'] == 3 and self.oled['lanes'] == 2, 'Same-session identity/motion/LONG/OLED capability required')
                require(self.parser.current_id is not None, 'Shared CBENCH id not screened')
                boundary = LongStatus(command, config=self.config, previous_id=self.parser.current_id)
            elif command == '#CBENCH LREAD':
                require(self.status is not None and self.status['phase'] in TERMINAL - {'IDLE'}, 'LREAD requires terminal LONG')
                boundary = LongRead(self.status)
            elif command == '#CBENCH OLED':
                require(self.status is not None, 'OLED requires a known LONG id')
                oled_id = self.parser.current_id if self.status['phase'] == 'IDLE' else self.status['id']
                require(oled_id is not None, 'Shared CBENCH id not known for OLED capabilities')
                boundary = OledBoundary(bench, oled_id)
            else:
                if command == '#CBENCH CANCEL':
                    require(self.identity == self.expected and self.status is not None and self.status['id'] != 0
                            and self.status['phase'] in ('PENDING', 'RUNNING')
                            and self.parser.current_id == self.status['id'],
                            'Cancellation requires an identified active LONG; old short jobs are not cancelled')
                boundary = LongStatus(command, expected_id=self.status['id'] if self.status is not None else None,
                                      config=self.config if self.metadata.get('run_accepted') else None)
            if self.pending or self.buffer:
                require(self.last_boundary is not None, 'Unconsumed response before request')
                deadline = time.monotonic() + self.timeout
                while self.pending or self.buffer:
                    self._background(self._line(deadline), self.last_boundary)
            encoded = (command + '\n').encode('ascii')
            if command.startswith('#CBENCH LONG '):
                self.metadata['run_transmit_attempted'] = True
            self.metadata['commands'].append(command)
            self.recorder.record('tx', command=command, hex=encoded.hex(), stage='attempt')
            written = self.port.write(encoded)
            self.recorder.record('tx_result', command=command, bytes_written=written, bytes_expected=len(encoded))
            require(written == len(encoded), 'Partial USB write; no retry performed')
            rows, deadline = [], time.monotonic() + self.timeout
            while True:
                line = self._line(deadline)
                rows.append(line)
                require(len(rows) <= 512, 'Response exceeds bounded LONG line count')
                if boundary.feed(line):
                    self.last_boundary = boundary
                    while self.pending:
                        self._background(self.pending.pop(0), boundary)
                    if isinstance(boundary, LongStatus):
                        if self.status is not None and boundary.values['id'] == self.status['id']:
                            require(boundary.values['n'] >= self.status['n']
                                    and boundary.values['elapsed_ms'] >= self.status['elapsed_ms'], 'LONG progress regressed')
                        self.status = dict(boundary.values)
                        if command.startswith('#CBENCH LONG '):
                            self.metadata['run_accepted'] = True
                    elif isinstance(boundary, LongRead):
                        self.stats = boundary.values
                    else:
                        self.oled = dict(boundary.metadata.values)
                    return rows

    return Session


def percentile_bucket(histogram, fraction):
    count = sum(histogram)
    if not count:
        return None
    rank, observed = math.ceil(count * fraction), 0
    for index, value in enumerate(histogram):
        observed += value
        if observed >= rank:
            return {'lower_us_inclusive': index * 1000, 'upper_us_exclusive': (index + 1) * 1000 if index < 20 else None,
                    'method': 'Nearest-rank observation lies in this1ms histogram bin; no exact percentile retained.'}


def summarize(status, stats, oled, metadata):
    complete = bool(stats is not None and oled is not None and status is not None
                    and status['phase'] in TERMINAL - {'IDLE'} and oled.get('id') == status['id']
                    and oled.get('version') == 3 and oled.get('lanes') == 2)
    kernel_checks, checks = [], []
    if stats is not None and status is not None:
        kernel_checks = [status['phase'] == 'DONE', status['reason'] == 'NONE', status['n'] == status['planned'],
                         status['elapsed_ms'] >= status['seconds'] * 1000, status['hz'] == 168000000,
                         stats['error'] == [0, UINT32_MAX], stats['sig'][0:2] == [10, 5],
                         5 <= stats['sig'][2] <= 40, stats['sig'][3:] == [5, 0]]
    if complete:
        checks = [status['phase'] == 'DONE', status['reason'] == 'NONE', status['n'] == status['planned'],
                  status['elapsed_ms'] >= status['seconds'] * 1000, status['hz'] == 168000000,
                  stats['error'] == [0, UINT32_MAX], stats['sig'][0:2] == [10, 5],
                  5 <= stats['sig'][2] <= 40, stats['sig'][3:] == [5, 0],
                  oled['enabled'] == 1, oled['mode'] == status['mode'], oled['runtime_flags'] == 0,
                  oled['restored'] == 1, oled['applied_priority'] == (16 if status['mode'] == 4 else 24),
                  all(frame['J'] == [stats['hash'][0], 0] and frame['G'][2] == 0 for frame in stats['history']),
                  metadata.get('error') is None]
    total = u64(*stats['sum']) if stats is not None else 0
    clock_usable = bool(status and stats and status['reason'] != 'CLOCK' and stats['error'][0] != 5
                        and status['phase'] != 'CLOCK_ERROR' and status.get('hz') == 168000000)
    hz = status['hz'] if clock_usable else 0
    activity = None
    if oled is not None:
        split = oled.get('mode', 0) >= 3
        prefix = 'display_' if split else ''
        worker_observed = bool(oled.get('begun', 0) and oled.get('ended', 0))
        render_completed = oled.get(prefix + 'ended', 0) - oled.get(prefix + 'skipped', 0)
        render_observed = bool(oled.get(prefix + 'render_begun', 0) and render_completed > 0)
        activity = {'worker_loop_observed': worker_observed, 'render_observed': render_observed,
                    'status': 'observed' if worker_observed and render_observed else 'evidence_insufficient',
                    'worker_begun': oled.get('begun'), 'worker_ended': oled.get('ended'),
                    'render_begun': oled.get(prefix + 'render_begun'), 'render_completed': render_completed,
                    'worker_max_loop_gap_ticks': oled.get('worker_max_loop_gap_ticks'),
                    'display_max_loop_gap_ticks': oled.get('display_max_loop_gap_ticks'),
                    'max_snapshot_age_ticks': oled.get('max_snapshot_age_ticks'),
                    'overwritten': oled.get('overwritten'), 'max_send_gap_ticks': oled.get('max_send_gap_ticks'),
                    'display_stack_free': oled.get('display_stack_free'),
                    'individual_service_calls_counted': False, 'physical_panel_ack_measured': False,
                    'starvation_acceptance': False,
                    'note': 'These counters establish observed worker/render activity only, not absence of starvation or physical panel FPS.'}
    return {'collection_complete': complete, 'job_success': bool(complete and all(checks)),
            'kernel_success': bool(kernel_checks and all(kernel_checks)), 'timing_clock_usable': clock_usable,
            'status': status, 'aggregates': stats, 'oled': oled, 'session': metadata,
            'display_observation': activity,
            'stack_watermark_trends': ([{'index': t['index'], 'command_free': t['K'][0],
                                        'worker_free': t['K'][1], 'display_free': t['K'][2]} for t in stats['trends']] if stats else []),
            'mean_us': total * 1e6 / hz / status['n'] if hz and stats and status['n'] else None,
            'window_mean_us': u64(*stats['window'][:2]) * 1e6 / hz / status['n'] if hz and stats and status['n'] else None,
            'observed_component_max_us': stats['minmax'][1] * 1e6 / hz if hz and stats else None,
            'overhead_mean_us': u64(*stats['overhead'][:2]) * 1e6 / hz / status['n'] if hz and stats and status['n'] else None,
            'overhead_max_us': stats['overhead'][2] * 1e6 / hz if hz and stats else None,
            'scan_max_us': stats['scan'][1] * 1e6 / hz if hz and stats else None,
            'p95_bucket': percentile_bucket(stats['histogram'], .95) if clock_usable else None,
            'p99_bucket': percentile_bucket(stats['histogram'], .99) if clock_usable else None,
            'hardware_evidence': False, 'performance_acceptance': False,
            'limits': 'Component-only continuous diagnostic. Histograms retain1ms intervals; last8 frames are not a full trajectory. Resident cycles include IRQ time; elapsed OLED lanes overlap. No full seven-axis controller or motion acceptance.'}


def prepare(output_root, manifest, config, poll_seconds, guard_seconds, argv=None):
    out = Path(output_root) / ('cartesian-endurance-' + stamp())
    out.mkdir(parents=True, exist_ok=False)
    for source, name in ((Path(__file__), 'collector_source.py'), (LEGACY, 'legacy_collector_source.py'),
                         (Path(manifest), 'firmware.manifest.txt')):
        shutil.copy2(source, out / name)
    require(sha(out / 'legacy_collector_source.py') == LEGACY_SHA, 'Legacy collector changed')
    identity = manifest_identity(out / 'firmware.manifest.txt')
    plan = {'schema': 1, 'kind': 'bounded_continuous_component_diagnostic', 'identity': identity,
            'request': config, 'poll_seconds': poll_seconds, 'guard_seconds': guard_seconds,
            'overall_deadline_seconds': config['seconds'] + guard_seconds,
            'source_hashes': {name: sha(out / name) for name in ('collector_source.py', 'legacy_collector_source.py', 'firmware.manifest.txt')},
            'python': sys.version, 'platform': platform.platform(), 'argv': sys.argv if argv is None else list(argv),
            'preparation_opened_serial': False, 'automatic_cancel_retry_reconnect_resume': False}
    write_json(out / 'plan.json', plan)
    return out, plan


def check_plan(out, plan):
    require(plan.get('schema') == 1 and plan.get('kind') == 'bounded_continuous_component_diagnostic', 'Unknown endurance plan')
    require(set(plan['source_hashes']) == {'collector_source.py', 'legacy_collector_source.py', 'firmware.manifest.txt'}, 'Frozen dependency list changed')
    for name, value in plan['source_hashes'].items():
        require(sha(out / name) == value, 'Frozen dependency/manifest changed: ' + name)
    require(sha(Path(__file__)) == plan['source_hashes']['collector_source.py'], 'Replay/collection source differs from frozen script')
    require(manifest_identity(out / 'firmware.manifest.txt') == plan['identity'], 'Frozen identity changed')
    config = plan['request']
    require(config == request_config(config['op'], config['fixture'], config['seconds'],
                                    'split-low' if config['mode'] == 4 else 'rate-10hz'), 'Frozen request changed')
    require(plan['poll_seconds'] in (5, 10) and plan['guard_seconds'] == 60
            and plan['overall_deadline_seconds'] == config['seconds'] + 60, 'Frozen deadline/poll policy changed')


def capture_index(out):
    names = ('collector_source.py', 'legacy_collector_source.py', 'firmware.manifest.txt',
             'plan.json', 'raw.jsonl', 'session.json', 'summary.json')
    write_json(out / 'capture-index.json', {name: sha(out / name) for name in names})


def collect(out, plan, port_name, run=False, cancel=False, timeout=8.0):
    check_plan(out, plan)
    require(not any((out / name).exists() for name in ('raw.jsonl', 'session.json', 'summary.json', 'capture-index.json')),
            'Capture already started; no resume/overwrite/reconnect')
    bench = load_legacy(out / 'legacy_collector_source.py')
    metadata = {'started_utc': datetime.now(timezone.utc).isoformat(), 'commands': [], 'port': port_name,
                'hardware_opened': False, 'motion_commands_permitted': False, 'run_transmit_attempted': False,
                'run_accepted': False, 'error': None, 'timeout_policy': 'No automatic cancellation/retry/reconnect/reset'}
    metadata['requested_action'] = 'run' if run else 'cancel' if cancel else 'read_only'
    recorder = bench.Recorder(out / 'raw.jsonl')
    port, session = None, None
    try:
        import serial
        metadata['pyserial_version'] = getattr(serial, '__version__', 'unknown')
        port = serial.Serial(port=None, baudrate=115200, timeout=.05, write_timeout=2)
        port.dtr = False
        port.rts = False
        port.port = port_name
        port.open()
        metadata['hardware_opened'] = True
        session = session_class(bench)(port, recorder, timeout, plan['identity'], plan['request'], metadata)
        for command in OLD_QUERIES:
            session.request(command)
        metadata['board_identity'] = session.identity
        metadata['motion_snapshot'] = session.motion
        session.request('#CBENCH LONG')
        session.request('#CBENCH OLED')
        if run:
            session.request(long_command(plan['request']))
        elif cancel:
            session.request('#CBENCH CANCEL')
        if run or cancel:
            deadline = time.monotonic() + plan['overall_deadline_seconds']
            while session.status['phase'] not in TERMINAL:
                require(time.monotonic() < deadline, 'LONG overall deadline expired; task was not automatically cancelled')
                print(f"{session.status['phase']}: {session.status['n']}/{session.status['planned']} samples, elapsed{session.status['elapsed_ms']/1000:.1f}s", flush=True)
                time.sleep(min(plan['poll_seconds'], max(0, deadline - time.monotonic())))
                session.request('#CBENCH LONG')
        if session.status['phase'] in TERMINAL - {'IDLE'}:
            session.request('#CBENCH LREAD')
            session.request('#CBENCH OLED')
        session.request('#STACK')
    except (Exception, KeyboardInterrupt) as exc:
        metadata['error'] = f'{type(exc).__name__}: {exc}'
        recorder.record('error', detail=metadata['error'])
    finally:
        if port is not None:
            try:
                port.close()
            except Exception as exc:
                metadata['close_error'] = str(exc)
        metadata['finished_utc'] = datetime.now(timezone.utc).isoformat()
        write_json(out / 'session.json', metadata)
        recorder.close()
    result = summarize(session.status if session else None, session.stats if session else None,
                       session.oled if session else None, metadata)
    write_json(out / 'summary.json', result)
    capture_index(out)
    return result


def analyze(out):
    out = Path(out)
    plan, metadata = read_json(out / 'plan.json'), read_json(out / 'session.json')
    check_plan(out, plan)
    index = read_json(out / 'capture-index.json')
    require(set(index) == {'collector_source.py', 'legacy_collector_source.py', 'firmware.manifest.txt',
                           'plan.json', 'raw.jsonl', 'session.json', 'summary.json'}, 'Capture index incomplete')
    require(all(sha(out / name) == value for name, value in index.items()), 'Original capture changed since collection')
    bench = load_legacy(out / 'legacy_collector_source.py')
    events = [json.loads(t) for t in (out / 'raw.jsonl').read_text(encoding='utf-8').splitlines()]
    blocks = []
    for event in events:
        if event.get('direction') == 'tx':
            require(command_allowed(event['command']), 'Raw contains forbidden command')
            blocks.append([event['command'], []])
        elif event.get('direction') == 'rx':
            require(bool(blocks), 'RX before any command')
            blocks[-1][1].append(event['line'])
    require([c for c, _ in blocks] == metadata['commands'], 'Session/raw command mismatch')
    identity, motion, status, stats, oled = None, None, None, None, None
    shared_id = None
    run_count = 0
    try:
        for block_index, (command, lines) in enumerate(blocks):
            if command in OLD_QUERIES:
                boundary = bench.ResponseBoundary(command)
            elif command == '#CBENCH OLED':
                require(status is not None, 'OLED without LONG status')
                oled_id = shared_id if status['phase'] == 'IDLE' else status['id']
                require(oled_id is not None, 'Shared id missing before OLED query')
                boundary = OledBoundary(bench, oled_id)
            elif command == '#CBENCH LREAD':
                require(status is not None and status['phase'] in TERMINAL - {'IDLE'}, 'LREAD without terminal LONG')
                boundary = LongRead(status)
            else:
                started = command.startswith('#CBENCH LONG ')
                if started:
                    require(identity == plan['identity'] and motion is not None and status is not None and oled is not None
                            and status['phase'] in TERMINAL and oled['version'] == 3 and oled['lanes'] == 2,
                            'LONG before same-session identity/capability/idle screening')
                    require(command == long_command(plan['request']), 'Raw LONG request differs from plan')
                    run_count += 1
                    require(run_count == 1, 'More than one LONG run in capture')
                if command == '#CBENCH CANCEL':
                    require(identity == plan['identity'] and status is not None and status['id'] != 0
                            and status['phase'] in ('PENDING', 'RUNNING') and shared_id == status['id'],
                            'Raw cancellation does not identify an active LONG')
                boundary = LongStatus(command, config=plan['request'] if started or run_count else None,
                                      previous_id=shared_id if started else None,
                                      expected_id=status['id'] if status and not started else None)
            ended = False
            for line in lines:
                if ended:
                    boundary.trailing(line)
                else:
                    ended = boundary.feed(line)
            if not ended:
                require(metadata['error'] is not None and block_index == len(blocks) - 1, 'Truncated response without recorded collection error')
                break
            if command == '#FW_VERSION 0':
                identity = parse_identity(lines, plan['identity'])
            elif command == '#MOTION_STATUS':
                motion = motion_snapshot(lines)
            elif command == '#CBENCH':
                shared = [bench.STATUS_RE.fullmatch(line) for line in lines]
                shared = [int(match[1]) for match in shared if match]
                require(len(shared) == 1, 'Shared CBENCH id missing')
                shared_id = shared[0]
            elif isinstance(boundary, LongStatus):
                status = boundary.values
            elif isinstance(boundary, LongRead):
                stats = boundary.values
            elif isinstance(boundary, OledBoundary):
                oled = boundary.metadata.values
    except Exception as exc:
        recorded = metadata.get('error')
        require(block_index == len(blocks) - 1 and recorded == f'{type(exc).__name__}: {exc}',
                'Raw protocol failure differs from recorded collection error')
    result = summarize(status, stats, oled, metadata)
    saved = read_json(out / 'summary.json')
    require(result == saved, 'Raw replay differs from saved summary')
    target = out / ('replay-' + stamp() + '.json')
    write_json(target, result)
    return target, result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    actions = parser.add_mutually_exclusive_group()
    actions.add_argument('--run', action='store_true')
    actions.add_argument('--cancel', action='store_true')
    parser.add_argument('--port')
    parser.add_argument('--analyze', type=Path)
    parser.add_argument('--manifest', type=Path, default=REPO / 'firmware_artifacts/Core-STM32F4-fw.manifest.txt')
    parser.add_argument('--op', type=int, choices=(3, 4), default=4)
    parser.add_argument('--fixture', type=int, choices=range(3), default=2)
    parser.add_argument('--seconds', type=int, default=600)
    parser.add_argument('--oled-mode', choices=('rate-10hz', 'split-low'), default='split-low')
    parser.add_argument('--poll-seconds', type=int, choices=(5, 10), default=10)
    parser.add_argument('--timeout', type=float, default=8.0)
    parser.add_argument('--output-root', type=Path, default=REPO / 'firmware_artifacts')
    args = parser.parse_args(argv)
    if args.analyze:
        if args.port or args.run or args.cancel:
            parser.error('--analyze never opens hardware')
        target, result = analyze(args.analyze)
        print(target)
        return 0 if result['job_success'] else 3
    if (args.run or args.cancel) and not args.port:
        parser.error('--run/--cancel requires explicit --port')
    if args.timeout <= 0:
        parser.error('Positive response timeout required')
    config = request_config(args.op, args.fixture, args.seconds, args.oled_mode)
    out, plan = prepare(args.output_root, args.manifest, config, args.poll_seconds, 60,
                        sys.argv if argv is None else [str(Path(__file__))] + list(argv))
    print(out, flush=True)
    if not args.port:
        print('PLAN ONLY: no serial import/device operation; component diagnostic only.')
        return 0
    result = collect(out, plan, args.port, args.run, args.cancel, args.timeout)
    print(f"phase={result['status']['phase'] if result['status'] else 'UNKNOWN'} job_success={result['job_success']}")
    if result['session']['error']:
        print(result['session']['error'], file=sys.stderr)
        return 2
    cancelled = bool(args.cancel and result['collection_complete'] and result['status']['phase'] == 'CANCELLED')
    return 0 if result['job_success'] or cancelled or not (args.run or args.cancel) else 3


if __name__ == '__main__':
    raise SystemExit(main())
