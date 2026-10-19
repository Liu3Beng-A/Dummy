"""Collect the release 17.02 full-cycle COMPUTE-ONLY diagnostic.

No arguments: print a plan without importing serial or touching hardware.
--port COM20: read-only screening and export of a completed FULL job.
--run --port COM20: explicitly run the bounded full calculation chain.
--cancel --port COM20: explicitly cancel only the active FULL diagnostic.
--run --cancel-after-ms 400 --port COM20: explicitly cancel this run on the
same port after the requested host interval, if the FULL job is still active.
--analyze CAPTURE: verify and replay a saved capture without importing serial.

Never sends START, STOP, DISABLE, motion, flash, reset, retries or reconnects.
Collection success and algorithm success are reported separately. A diagnostic
success is not a hardware motion or hard real-time acceptance certificate.
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
import platform
import re
import shutil
import struct
import sys
import time

REPO = Path(__file__).resolve().parents[2]
RELEASE = '2026101702'
MANIFESTS = ('Core-STM32F4-fw', 'motor-35', 'motor-42', 'motor-57', 'motor-gripper')
LEGACY = REPO / 'tools/tests/run_cartesian_benchmark.py'
LEGACY_SHA = '27cc58c80642cada8edcb38add081456cf7ed272653a6be5c6ff4be446fc9a23'
TERMINAL = frozenset(('IDLE', 'DONE', 'CANCELLED', 'ABORTED', 'BUDGET', 'CLOCK_ERROR'))
PHASES = TERMINAL | {'PENDING', 'RUNNING'}
U32 = (1 << 32) - 1
PREQUERIES = ('#FW_VERSION 0', '#MOTION_STATUS', '#STACK', '#CBENCH FULL')
CASES = ('hold', 'line-x', 'line-xyz', 'slider-burst', 'slider-smooth',
         'joystick-reverse', 'joystick-xyz', 'rail-line', 'rail-recovery',
         'injected-ik-loss', 'injected-segment-reject', 'unreachable', 'invalid-input')


class ProtocolError(ValueError):
    pass


def require(condition, message):
    if not condition:
        raise ProtocolError(message)


def stamp():
    return datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    with Path(path).open('x', encoding='utf-8', newline='\n') as stream:
        stream.write(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')


def read_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def load_legacy(path=LEGACY):
    require(sha(path) == LEGACY_SHA, 'Reviewed legacy collector hash differs')
    spec = importlib.util.spec_from_file_location('cycle_legacy_' + stamp(), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def configuration(scenario=1, fixture=0, variant=1, samples=160):
    require(type(scenario) is int and 0 <= scenario < 13, 'Scenario must be 0..12')
    require(type(fixture) is int and 0 <= fixture < 6, 'Fixture must be 0..5')
    require(type(variant) is int and 1 <= variant <= 3, 'Variant must be 1..3')
    require(type(samples) is int and 1 <= samples <= 160, 'Samples must be 1..160')
    return {'case': scenario, 'fixture': fixture, 'variant': variant, 'samples': samples, 'budget': 20000}


def run_command(config):
    require(config == configuration(config['case'], config['fixture'], config['variant'], config['samples']), 'Unsupported request')
    return '#CBENCH FULL_RUN ' + ','.join(str(config[k]) for k in ('case', 'fixture', 'variant', 'samples', 'budget'))


def command_allowed(command):
    if command in PREQUERIES + ('#CBENCH FULL_CANCEL', '#CBENCH OLED'):
        return True
    if match := re.fullmatch(r'#CBENCH FREAD ([0-9]{1,3}),([1-4])', command):
        return int(match[1]) + int(match[2]) <= 160
    if match := re.fullmatch(r'#CBENCH FULL_RUN ([0-9]+),([0-9]+),([0-9]+),([0-9]+),20000', command):
        try:
            return command == run_command(configuration(*map(int, match.groups())))
        except (ValueError, KeyError):
            return False
    return False


def manifests_identity(directory):
    identities = []
    for name in MANIFESTS:
        values = {}
        for line in (Path(directory) / (name + '.manifest.txt')).read_text(encoding='utf-8').splitlines():
            if line.startswith('['):
                break
            if '=' in line:
                key, value = line.split('=', 1)
                require(key not in values, 'Duplicate manifest field')
                values[key] = value
        require(values.get('release') == RELEASE, 'All five manifests must identify release2026101702')
        require(values.get('project') == ('Core-STM32F4-fw' if name == MANIFESTS[0] else 'Ctrl-Step-STM32-fw'), 'Manifest target mismatch')
        require(re.fullmatch(r'[0-9A-Fa-f]{16}', values.get('buildset', '')) is not None
                and re.fullmatch(r'[0-9A-Fa-f]{64}', values.get('source_sha256', '')) is not None
                and re.fullmatch(r'[0-9]{1,10}', values.get('utc_epoch', '')) is not None,
                'Incomplete manifest identity')
        identities.append({'v': RELEASE[:8] + '.' + RELEASE[8:], 'type': 'CORE',
                           'epoch': values['utc_epoch'], 'group': values['buildset'].upper(),
                           'source_sha256': values['source_sha256'].lower()})
    require(all(item == identities[0] for item in identities), 'Five firmware manifests are from different batches')
    return identities[0]


def identity_from_lines(lines, expected):
    pending = [re.fullmatch(r'ok FW_VERSION \[0\] req=([0-9]+) pending', line) for line in lines]
    ids = [int(m[1]) for m in pending if m]
    require(len(ids) == 1 and ids[0] > 0, 'Missing/ambiguous identity request')
    fields, ended = {}, False
    for line in lines:
        match = re.fullmatch(r'FW_VERSION \[0\] req=([0-9]+) (.+)', line)
        if not match:
            continue
        require(int(match[1]) == ids[0], 'Identity request id changed')
        if match[2] == 'end':
            require(not ended, 'Duplicate identity end')
            ended = True
            continue
        require(not ended, 'Identity field after end')
        for pair in match[2].split():
            require('=' in pair, 'Malformed identity field')
            key, value = pair.split('=', 1)
            require(key not in fields, 'Duplicate identity field')
            fields[key] = value
    require(ended and set(fields) == {'v', 'type', 'epoch', 'group', 'h0', 'h1'}, 'Incomplete identity')
    actual = {key: fields[key] for key in ('v', 'type', 'epoch')}
    actual.update(group=fields['group'].upper(), source_sha256=(fields['h0'] + fields['h1']).lower())
    require(actual == expected, 'Board differs from frozen release17.02 manifests; FULL_RUN was not admitted')
    return actual


def motion_snapshot(lines):
    headers = [re.fullmatch(r'MOTION id=[0-9]+ kind=\S+ phase=([0-9]+) mode=[0-9]+ ms=[0-9]+', line) for line in lines]
    phases = [int(m[1]) for m in headers if m]
    masks = [re.fullmatch(r'MOTION plan_ms=\S+ moving=([0-9A-Fa-f]+) txok=[0-9A-Fa-f]+ pending=([0-9A-Fa-f]+)', line) for line in lines]
    masks = [tuple(int(t, 16) for t in m.groups()) for m in masks if m]
    require(len(phases) == 1 and phases[0] in (0, 3, 4, 5) and len(masks) == 1 and masks[0][0] & ~0xFE == 0
            and masks[0][1] == 0, 'MOTION history is active/incomplete')
    return {'phase': phases[0], 'historical_displacement_mask': masks[0][0], 'pending': 0,
            'disabled_proven_by_host': False, 'note': 'Firmware admission atomically enforces idle DISABLED; masks are historical.'}


def numbers(text, count, signed=False):
    fields = text.split(',')
    pattern = r'-?[0-9]{1,10}' if signed else r'[0-9]{1,10}'
    require(len(fields) == count and all(re.fullmatch(pattern, t) for t in fields), 'Malformed bounded numeric record')
    values = tuple(map(int, fields))
    require(all((-(1 << 31) if signed else 0) <= n <= ((1 << 31) - 1 if signed else U32) for n in values), 'Numeric record out of range')
    return values


def record_checksum(frame):
    """FNV-1a of encoded92bytes including CCITT CRC16; emitted records only."""
    payload = bytearray(struct.pack('<BBHH', 2, 0, 1, frame['index'] + 1))
    for axis in range(7):
        payload.extend(struct.pack('<iii', *frame['q'][axis], *frame['va'][axis]))
    crc = 0xFFFF
    for byte in payload:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
    payload.extend(struct.pack('<H', crc))
    require(len(payload) == 92, 'Reconstructed record size differs')
    value = 2166136261
    for byte in payload:
        value = ((value ^ byte) * 16777619) & U32
    return value


class FullStatus:
    command = '#CBENCH FULL'
    patterns = (
        ('header', r'CFULL v=1 id=([0-9]+) phase=([A-Z_]+)'),
        ('caps', r'CFULL caps samples=([0-9]+) scenarios=([0-9]+) variants=([0-9]+)'),
        ('cfg', r'CFULL cfg case=([0-9]+) fixture=([0-9]+) variant=([0-9]+) samples=([0-9]+)'),
        ('clock', r'CFULL clock hz=([0-9]+) tick_hz=([0-9]+)'),
        ('run', r'CFULL run n=([0-9]+) planned=([0-9]+) budget=([0-9]+)'),
        ('fault', r'CFULL fault reason=([0-9]+) source=([0-9]+) would_stop=([01])'),
        ('first', r'CFULL first seq=([0-9]+)'),
        ('result', r'CFULL result emitted=([0-9]+) accepted=([0-9]+)'),
        ('work', r'CFULL work ik=([0-9]+) fk=([0-9]+) peak=([0-9]+)'),
        ('setup', r'CFULL setup cycles=([0-9]+)'),
        ('contract', r'CFULL no_motion=([01]) scope=(\S+)'),
        ('end', r'CFULL end id=([0-9]+)'),
    )

    def __init__(self, command='#CBENCH FULL', expected_id=None, config=None, previous_id=None):
        self.command, self.expected_id, self.config, self.previous_id = command, expected_id, config, previous_id
        self.seen, self.values = set(), {}

    def related(self, line):
        return line.startswith('CFULL ')

    def trailing(self, line):
        require(not line.startswith(('error ', 'CFULL ')), 'Unexpected trailing FULL response: ' + line)

    def feed(self, line):
        require(not line.startswith('error '), line)
        if not self.related(line):
            return False
        require(len(line.encode('utf-8')) <= 61, 'FULL payload exceeds firmware response capacity')
        for key, pattern in self.patterns:
            match = re.fullmatch(pattern, line)
            if not match:
                continue
            require(key not in self.seen, 'Duplicate FULL metadata ' + key)
            require(key == 'header' or 'header' in self.seen, 'FULL metadata arrived before header')
            if key == 'header':
                job_id, phase = int(match[1]), match[2]
                require(0 <= job_id <= U32 and phase in PHASES, 'Unsupported FULL phase/id')
                require(self.expected_id is None or job_id == self.expected_id, 'FULL job id changed')
                require(self.previous_id is None or job_id != self.previous_id, 'FULL_RUN did not allocate a new id')
                self.values.update(id=job_id, phase=phase)
            elif key == 'contract':
                require(match.groups() == ('1', 'full_cycle'), 'Unexpected full-cycle/no-motion contract')
            else:
                value = numbers(','.join(match.groups()), len(match.groups()))
                self.values[key] = value
            self.seen.add(key)
            if key == 'end':
                require(self.seen == {key for key, _ in self.patterns}, 'Incomplete FULL status at end')
                self.validate()
                return True
            return False
        raise ProtocolError('Malformed FULL metadata: ' + line)

    def validate(self):
        v = self.values
        require(v['caps'] == (160, 13, 3) and v['clock'][1] == 1000, 'Unsupported FULL capability/clock')
        require(v['end'] == (v['id'],), 'FULL end id mismatch')
        config = configuration(*v['cfg'])
        require(self.config is None or self.config == config, 'FULL configuration differs from request')
        require(v['run'][1:] == (config['samples'], 20000) and v['run'][0] <= config['samples'], 'FULL planned/count/budget mismatch')
        require(v['fault'][0] in set(range(18)) | set(range(32, 38)) and v['fault'][1] <= 4
                and max(v['result']) <= v['run'][0], 'FULL fault/result bounds')
        require(v['work'][2] <= 5 and v['work'][2] <= v['work'][0], 'FULL IK bound exceeded')
        if v['phase'] != 'IDLE':
            require(v['id'] > 0, 'Non-IDLE FULL missing job id')
        if v['phase'] == 'RUNNING' or v['run'][0]:
            require(v['clock'][0] > 0, 'Measured FULL missing clock frequency')
        if v['phase'] == 'DONE':
            require(v['run'][0] == config['samples'] and v['fault'] == (0, 0, 0), 'DONE FULL incomplete or failed')


class FullRead:
    def __init__(self, status, first, amount):
        require(status['phase'] in TERMINAL - {'IDLE'} and 1 <= amount <= 4 and first + amount <= status['run'][0], 'FREAD requires bounded terminal result range')
        self.status, self.first, self.amount = status, first, amount
        self.command = f'#CBENCH FREAD {first},{amount}'
        self.header, self.ended, self.frames = False, False, {}

    def related(self, line):
        return line.startswith('CFULL ')

    def trailing(self, line):
        require(not line.startswith(('error ', 'CFULL ')), 'Unexpected trailing FREAD response: ' + line)

    def feed(self, line):
        require(not line.startswith('error '), line)
        if not self.related(line):
            return False
        require(len(line.encode('utf-8')) <= 61, 'FREAD payload exceeds firmware response capacity')
        for kind, pattern in (('header', r'CFULL read id=([0-9]+) first=([0-9]+) n=([0-9]+)'),
                              ('end', r'CFULL read_end id=([0-9]+) first=([0-9]+) n=([0-9]+)')):
            if match := re.fullmatch(pattern, line):
                require(numbers(','.join(match.groups()), 3) == (self.status['id'], self.first, self.amount), 'FREAD header/end context differs')
                if kind == 'header':
                    require(not self.header and not self.frames, 'Duplicate FREAD header')
                    self.header = True
                    return False
                require(self.header and not self.ended and set(self.frames) == set(range(self.first, self.first + self.amount))
                        and all(row.get('ended') for row in self.frames.values()), 'Incomplete FREAD at read_end')
                self.ended = True
                return True
        match = re.fullmatch(r'CFULL ([TCSPIRQVE]) (.+)', line)
        require(match is not None and self.header and not self.ended, 'Malformed/unframed FREAD data')
        kind, text = match.groups()
        lengths = {'T': 5, 'C': 5, 'S': 6, 'P': 5, 'I': 5, 'R': 4, 'Q': 4, 'V': 5, 'E': 2}
        fields = text.split(',')
        require(len(fields) == lengths[kind], 'FREAD field count differs')
        job_id, index = numbers(','.join(fields[:2]), 2)
        require(job_id == self.status['id'] and self.first <= index < self.first + self.amount, 'FREAD job/index differs')
        payload = numbers(','.join(fields[2:]), len(fields) - 2, kind in 'PIRQV') if len(fields) > 2 else ()
        row = self.frames.setdefault(index, {'index': index, 'q': {}, 'va': {}})
        require(not row.get('ended'), 'FREAD data after frame end')
        if kind in 'QV':
            axis, *values = payload
            bucket = row['q' if kind == 'Q' else 'va']
            require(0 <= axis < 7 and axis not in bucket, 'Duplicate/missing/out-of-range axis')
            bucket[axis] = values
        else:
            require(kind not in row, 'Duplicate FREAD field ' + kind)
            row[kind] = payload
        if kind == 'E':
            require(all(k in row for k in 'TCSPIR') and set(row['q']) == set(range(7)) and set(row['va']) == set(range(7)), 'Incomplete seven-axis frame at E')
            plan, start, end = row['T']
            cycles, window, checksum = row['C']
            require(((start - plan) & U32) <= 10000 and ((end - start) & U32) <= 10000, 'FREAD timing window exceeds bounds')
            require(cycles <= window and window * 1000 <= self.status['clock'][0] * 10000, 'FREAD kernel/window cycles inconsistent')
            require(row['S'][0] <= 5 and row['S'][1] <= 65535 and row['S'][3] < 4096
                    and row['S'][2] in set(range(18)) | set(range(32, 38)), 'FREAD work/reason/flags out of range')
            require(0 <= row['R'][1] <= 1000000, 'FREAD margin outside0..1')
            if row['S'][3] & 1:
                require(record_checksum(row) == checksum, 'Emitted record FNV/CRC reconstruction mismatch')
            row['ended'] = True
        return False


class OledBoundary:
    command = '#CBENCH OLED'
    def __init__(self, legacy, job_id):
        self.legacy, self.job_id = legacy, job_id
        self.metadata = legacy.OledMetadata(job_id) if job_id is not None else None

    def related(self, line):
        return line.startswith('CBOLED ')

    def trailing(self, line):
        require(not line.startswith(('error ', 'CBOLED ')), 'Unexpected trailing OLED record')

    def feed(self, line):
        require(not line.startswith('error '), line)
        if self.related(line) and self.metadata is None:
            match = self.legacy.OLED_HEADER_RE.fullmatch(line)
            require(match is not None, 'OLED capability metadata before header')
            self.metadata = self.legacy.OledMetadata(int(match[2]))
        return self.metadata.feed(line) if self.related(line) else False


class Session:
    def __init__(self, legacy, port, recorder, timeout, plan, metadata):
        self.legacy, self.port, self.recorder, self.timeout = legacy, port, recorder, timeout
        self.plan, self.metadata = plan, metadata
        self.buffer, self.pending, self.last_boundary = bytearray(), [], None
        self.identity, self.motion, self.status, self.oled = None, None, None, None
        self.frames, self.stacks = {}, []

    def _line(self, deadline):
        while time.monotonic() < deadline:
            if self.pending:
                return self.pending.pop(0)
            chunk = self.port.read(4096)
            if not chunk:
                continue
            self.recorder.record('rx_chunk', hex=chunk.hex())
            self.buffer.extend(chunk)
            require(len(self.buffer) <= 65536, 'Receive fragment exceeds64KiB')
            while b'\n' in self.buffer:
                raw, _, tail = self.buffer.partition(b'\n')
                self.buffer = bytearray(tail)
                line = raw.rstrip(b'\r').decode('utf-8', errors='strict')
                self.recorder.record('rx', line=line)
                self.pending.append(line)
        raise TimeoutError('Response deadline expired; diagnostic was not automatically cancelled')

    def _background(self, line):
        require(self.last_boundary is not None, 'Unconsumed response fragment')
        self.last_boundary.trailing(line)
        self.recorder.record('background', line=line, after=self.last_boundary.command)

    def boundary(self, command):
        require(command_allowed(command), 'Command denied by strict compute-only allowlist')
        if command in ('#FW_VERSION 0', '#MOTION_STATUS', '#STACK'):
            return self.legacy.ResponseBoundary(command)
        if command == '#CBENCH OLED':
            require(self.status is not None, 'OLED requires same-session FULL status')
            return OledBoundary(self.legacy, None if self.status['phase'] == 'IDLE' and self.status['id'] == 0 else self.status['id'])
        if command.startswith('#CBENCH FREAD '):
            first, amount = map(int, command.split()[-1].split(','))
            return FullRead(self.status, first, amount)
        if command.startswith('#CBENCH FULL_RUN '):
            require(command == run_command(self.plan['request']), 'RUN differs from frozen request')
            require(self.identity == self.plan['identity'] and self.motion is not None and self.status is not None
                    and self.status['phase'] in TERMINAL and self.oled is not None
                    and self.oled.get('version') == 3 and self.oled.get('lanes') == 2,
                    'Same-session identity, idle history, FULL and split OLED capability required')
            return FullStatus(command, config=self.plan['request'], previous_id=self.status['id'])
        if command == '#CBENCH FULL_CANCEL':
            require(self.identity == self.plan['identity'] and self.status is not None
                    and self.status['id'] > 0 and self.status['phase'] in ('PENDING', 'RUNNING'), 'Cancel requires identified active FULL job')
        return FullStatus(command, expected_id=self.status['id'] if self.status else None,
                          config=self.plan['request'] if self.metadata.get('run_accepted') else None)

    def apply(self, command, boundary, lines):
        if command == '#FW_VERSION 0':
            self.identity = identity_from_lines(lines, self.plan['identity'])
        elif command == '#MOTION_STATUS':
            self.motion = motion_snapshot(lines)
        elif command == '#STACK':
            self.stacks.append(list(lines))
        elif isinstance(boundary, FullStatus):
            value = dict(boundary.values)
            if self.status and value['id'] == self.status['id']:
                require(value['run'][0] >= self.status['run'][0], 'FULL progress regressed')
                require(value['cfg'] == self.status['cfg'], 'FULL configuration changed')
                require(value['clock'][0] == self.status['clock'][0] or self.status['clock'][0] == 0, 'FULL clock changed')
                require(self.status['phase'] not in TERMINAL or value['phase'] == self.status['phase'], 'Terminal FULL phase changed')
            self.status = value
            if command.startswith('#CBENCH FULL_RUN '):
                self.metadata['run_accepted'] = True
        elif isinstance(boundary, FullRead):
            require(not (set(self.frames) & set(boundary.frames)), 'Duplicate FREAD range')
            self.frames.update(boundary.frames)
        elif isinstance(boundary, OledBoundary):
            self.oled = dict(boundary.metadata.values)

    def request(self, command):
        while self.pending or self.buffer:
            self._background(self._line(time.monotonic() + self.timeout))
        boundary = self.boundary(command)
        encoded = (command + '\n').encode('ascii')
        self.metadata['commands'].append(command)
        if command.startswith('#CBENCH FULL_RUN '):
            self.metadata['run_transmit_attempted'] = True
        self.recorder.record('tx', command=command, hex=encoded.hex(), stage='attempt')
        written = self.port.write(encoded)
        self.recorder.record('tx_result', command=command, bytes_written=written, bytes_expected=len(encoded))
        require(written == len(encoded), 'Partial write; no retry performed')
        if command == '#CBENCH FULL_CANCEL':
            self.metadata['cancel_transmitted'] = True
            if self.plan['explicit_cancel_after_ms'] is not None:
                self.metadata['explicit_cancel_sent'] = True
                self.metadata['explicit_cancel_outcome'] = 'sent'
        rows, deadline = [], time.monotonic() + self.timeout
        while True:
            line = self._line(deadline)
            rows.append(line)
            require(len(rows) <= 4096, 'Response exceeds4096line bound')
            if boundary.feed(line):
                self.last_boundary = boundary
                while self.pending:
                    self._background(self.pending.pop(0))
                # Explicitly retain the transaction point: a malformed trailing
                # line must fail before accepted job/context state is changed.
                self.recorder.record('response_commit', command=command)
                self.apply(command, boundary, rows)
                return rows


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    point = (len(ordered) - 1) * fraction
    low = int(point)
    return ordered[low] + (ordered[min(low + 1, len(ordered) - 1)] - ordered[low]) * (point - low)


def metric(values):
    return {'count': len(values), 'min': min(values) if values else None, 'mean': sum(values) / len(values) if values else None,
            'p50': percentile(values, .5), 'p95': percentile(values, .95), 'p99': percentile(values, .99),
            'max': max(values) if values else None}


def summarize(session, metadata):
    status = session.status if session else None
    frames = session.frames if session else {}
    errors = [metadata['error']] if metadata.get('error') else []
    if status is None:
        errors.append('Missing FULL status')
    elif status['phase'] not in TERMINAL:
        errors.append('FULL job remains active')
    elif status['phase'] != 'IDLE' and set(frames) != set(range(status['run'][0])):
        errors.append('Missing complete terminal frames')
    if status and frames:
        ordered = [frames[i] for i in sorted(frames)]
        for previous, current in zip(ordered, ordered[1:]):
            if (current['T'][0] - previous['T'][0]) & U32 != 20:
                errors.append('Planned frame interval differs from20ticks')
                break
        if sum(r['S'][0] for r in ordered) != status['work'][0] or sum(r['S'][1] for r in ordered) != status['work'][1]:
            errors.append('Terminal work counters differ from frames')
        if sum(bool(r['S'][3] & 1) for r in ordered) != status['result'][0] or sum(bool(r['S'][3] & 4) for r in ordered) != status['result'][1]:
            errors.append('Terminal emitted/accepted counters differ from frames')
    complete = bool(status and status['phase'] in TERMINAL and not errors)
    hz = status['clock'][0] if status else 0
    timing = lambda slot: [r['C'][slot] * 1e6 / hz for r in frames.values()] if hz else []
    oled = session.oled if session else None
    valid_tcp = [r for r in frames.values() if r['S'][3] & 2048]
    geometry = [sum(((p - goal) / 1000) ** 2 for p, goal in zip(r['P'], r['I'])) ** .5 for r in valid_tcp]
    return {'schema': 1, 'kind': 'full_cycle_compute_only', 'collection_complete': complete,
            'job_success': bool(complete and status['phase'] == 'DONE'), 'status': status, 'errors': errors,
            'kernel_us': metric(timing(0)), 'window_us': metric(timing(1)),
            'over20ms': sum(v > 20000 for v in timing(1)), 'lateness_ticks': metric([((r['T'][1] - r['T'][0]) & U32) for r in frames.values()]),
            'reasons': {str(reason): sum(r['S'][2] == reason for r in frames.values()) for reason in sorted({r['S'][2] for r in frames.values()})},
            'tcp_intent_error_mm': metric(geometry), 'tcp_valid_frames': len(valid_tcp),
            'accepted_without_output_frames': sum(bool(r['S'][3] & 4) and not bool(r['S'][3] & 1) for r in frames.values()),
            'oled_runtime_valid': bool(oled and oled.get('restored') == 1 and oled.get('runtime_flags') == 0),
            'oled': oled, 'stacks': session.stacks if session else [], 'session': metadata,
            'hardware_evidence': bool(metadata.get('hardware_opened') and not metadata.get('test_transport')),
            'evidence_note': 'Compute-only fixed scenarios; no motor outputs, actual CAN execution, mechanical tracking or general joystick acceptance. IRQ-inclusive window measurements; no exclusive CPU attribution. Accepted means input accepted by planner, including a subsequently rolled-back frame; emitted means committed virtual record. TCP errors use only TCP_VALID frames. Emitted checksum reconstructs v2/session1/sequence(index+1)/flags0 with CRC16 and FNV1a32; nonemitted last-endpoint checksum retained without reconstruction. Timed cancellation occurs only with explicit --run --cancel-after-ms intent, once on this same port; its host interval includes200ms polling and USB latency and is not an exact controller deadline.'}


def prepare(output_root, manifest_directory, config, argv=None, explicit_cancel_after_ms=None):
    require(explicit_cancel_after_ms is None or type(explicit_cancel_after_ms) is int and 0 <= explicit_cancel_after_ms <= 3000,
            'Explicit cancellation interval must be0..3000ms')
    out = Path(output_root) / ('cartesian-cycle-diagnostic-' + stamp())
    out.mkdir(parents=True, exist_ok=False)
    shutil.copy2(Path(__file__), out / 'collector_source.py')
    shutil.copy2(LEGACY, out / 'legacy_collector_source.py')
    for name in MANIFESTS:
        shutil.copy2(Path(manifest_directory) / (name + '.manifest.txt'), out / (name + '.manifest.txt'))
    identity = manifests_identity(out)
    names = ['collector_source.py', 'legacy_collector_source.py'] + [name + '.manifest.txt' for name in MANIFESTS]
    require(sha(out / 'legacy_collector_source.py') == LEGACY_SHA, 'Reviewed legacy collector changed')
    plan = {'schema': 1, 'kind': 'full_cycle_compute_only', 'identity': identity, 'request': config,
            'source_hashes': {name: sha(out / name) for name in names}, 'python': sys.version, 'platform': platform.platform(),
            'argv': list(sys.argv if argv is None else argv), 'period_ms': 20, 'poll_seconds': .2,
            'explicit_cancel_after_ms': explicit_cancel_after_ms,
            'overall_deadline_seconds': 15, 'preparation_opened_serial': False,
            'automatic_cancel_retry_reconnect_resume': False}
    write_json(out / 'plan.json', plan)
    return out, plan


def check_plan(out, plan):
    require(plan.get('schema') == 1 and plan.get('kind') == 'full_cycle_compute_only', 'Unknown capture plan')
    names = {'collector_source.py', 'legacy_collector_source.py'} | {name + '.manifest.txt' for name in MANIFESTS}
    require(set(plan['source_hashes']) == names, 'Frozen dependency list differs')
    require(all(sha(Path(out) / name) == value for name, value in plan['source_hashes'].items()), 'Frozen dependency/manifest changed')
    require(sha(Path(__file__)) == plan['source_hashes']['collector_source.py'], 'Use the collector frozen for this capture')
    require(manifests_identity(out) == plan['identity'] and sha(Path(out) / 'legacy_collector_source.py') == LEGACY_SHA, 'Frozen identity/dependency changed')
    config = plan['request']
    require(config == configuration(config['case'], config['fixture'], config['variant'], config['samples']), 'Frozen request changed')
    require(plan['period_ms'] == 20 and plan['poll_seconds'] == .2 and plan['overall_deadline_seconds'] == 15
            and not plan['automatic_cancel_retry_reconnect_resume'], 'Frozen timing/action policy changed')
    interval = plan['explicit_cancel_after_ms']
    require(interval is None or type(interval) is int and 0 <= interval <= 3000, 'Frozen explicit cancellation interval changed')


def export_frames(path, frames):
    fields = ['index', 'planned_rtos', 'start_rtos', 'end_rtos', 'cycles', 'window_cycles', 'checksum', 'ik', 'fk', 'reason', 'flags',
              'x_um', 'y_um', 'z_um', 'intent_x_um', 'intent_y_um', 'intent_z_um', 'rail_um', 'margin_ppm']
    fields += [f'{field}{axis}' for axis in range(7) for field in ('q', 'v', 'a')]
    with Path(path).open('x', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fields)
        writer.writeheader()
        for i, frame in sorted(frames.items()):
            row = dict(zip(fields[:19], (i, *frame['T'], *frame['C'], *frame['S'], *frame['P'], *frame['I'], *frame['R'])))
            for axis in range(7):
                row.update(zip((f'q{axis}', f'v{axis}', f'a{axis}'), (*frame['q'][axis], *frame['va'][axis])))
            writer.writerow(row)


def capture_index(out):
    names = ('collector_source.py', 'legacy_collector_source.py', 'plan.json', 'raw.jsonl', 'session.json', 'summary.json', 'frames.csv')
    names += tuple(name + '.manifest.txt' for name in MANIFESTS)
    write_json(Path(out) / 'capture-index.json', {name: sha(Path(out) / name) for name in names})


def collect(out, plan, port_name, run=False, cancel=False, timeout=8., port_factory=None):
    check_plan(out, plan)
    require(not run or not cancel, 'RUN and CANCEL are mutually exclusive')
    require(plan['explicit_cancel_after_ms'] is None or run and not cancel, 'Timed cancellation requires explicit RUN')
    require(not any((Path(out) / name).exists() for name in ('raw.jsonl', 'session.json', 'summary.json', 'capture-index.json', 'frames.csv')), 'Capture already started; no overwrite/resume/reconnect')
    legacy = load_legacy(Path(out) / 'legacy_collector_source.py')
    metadata = {'commands': [], 'port': port_name, 'started_utc': datetime.now(timezone.utc).isoformat(),
                'requested_action': 'run' if run else 'cancel' if cancel else 'read_only',
                'hardware_opened': False, 'test_transport': port_factory is not None, 'motion_commands_permitted': False,
                'run_transmit_attempted': False, 'run_accepted': False, 'error': None,
                'cancel_transmitted': False, 'explicit_cancel_after_ms': plan['explicit_cancel_after_ms'],
                'explicit_cancel_sent': False, 'explicit_cancel_elapsed_ms': None,
                'explicit_cancel_outcome': 'not_requested' if plan['explicit_cancel_after_ms'] is None else 'requested_not_sent',
                'timeout_policy': 'No automatic cancellation, retry, reset or reconnect'}
    recorder, port, session = legacy.Recorder(Path(out) / 'raw.jsonl'), None, None
    try:
        if port_factory is None:
            import serial
            metadata['pyserial_version'] = getattr(serial, '__version__', 'unknown')
            port = serial.Serial(port=None, baudrate=115200, timeout=.05, write_timeout=2)
            port.dtr = False
            port.rts = False
            port.port = port_name
            port.open()
        else:
            port = port_factory()
        metadata['hardware_opened'] = True
        session = Session(legacy, port, recorder, timeout, plan, metadata)
        for command in PREQUERIES:
            session.request(command)
        session.request('#CBENCH OLED')
        metadata.update(board_identity=session.identity, motion_snapshot=session.motion)
        run_started = None
        if run:
            run_started = time.monotonic()
            session.request(run_command(plan['request']))
        elif cancel:
            session.request('#CBENCH FULL_CANCEL')
        if run or cancel:
            deadline = time.monotonic() + plan['overall_deadline_seconds']
            while session.status['phase'] not in TERMINAL:
                require(time.monotonic() < deadline, 'Overall deadline expired; diagnostic was not automatically cancelled')
                elapsed_ms = (time.monotonic() - run_started) * 1000 if run_started is not None else None
                if (plan['explicit_cancel_after_ms'] is not None and not metadata['explicit_cancel_sent']
                        and elapsed_ms >= plan['explicit_cancel_after_ms']):
                    metadata['explicit_cancel_elapsed_ms'] = elapsed_ms
                    recorder.record('explicit_cancel_timer', requested_ms=plan['explicit_cancel_after_ms'], observed_ms=elapsed_ms,
                                    origin='FULL_RUN host transmit attempt; polling and USB latency included')
                    session.request('#CBENCH FULL_CANCEL')
                    continue
                time.sleep(min(plan['poll_seconds'], max(0, deadline - time.monotonic())))
                session.request('#CBENCH FULL')
        if plan['explicit_cancel_after_ms'] is not None and not metadata['explicit_cancel_sent'] and session.status['phase'] in TERMINAL:
            metadata['explicit_cancel_outcome'] = 'terminal_before_cancel'
        if session.status['phase'] in TERMINAL - {'IDLE'}:
            for first in range(0, session.status['run'][0], 4):
                session.request(f'#CBENCH FREAD {first},{min(4, session.status["run"][0] - first)}')
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
        write_json(Path(out) / 'session.json', metadata)
        recorder.close()
    result = summarize(session, metadata)
    write_json(Path(out) / 'summary.json', result)
    export_frames(Path(out) / 'frames.csv', session.frames if session else {})
    capture_index(out)
    return result


def analyze(out):
    out = Path(out)
    plan, metadata = read_json(out / 'plan.json'), read_json(out / 'session.json')
    check_plan(out, plan)
    index = read_json(out / 'capture-index.json')
    names = {'collector_source.py', 'legacy_collector_source.py', 'plan.json', 'raw.jsonl', 'session.json', 'summary.json', 'frames.csv'} | {name + '.manifest.txt' for name in MANIFESTS}
    require(set(index) == names and all(sha(out / name) == value for name, value in index.items()), 'Original capture/index changed')
    require(metadata['explicit_cancel_after_ms'] == plan['explicit_cancel_after_ms'], 'Saved explicit cancellation intent differs from plan')
    legacy = load_legacy(out / 'legacy_collector_source.py')
    session = Session(legacy, None, None, 0, plan, {'commands': [], 'run_accepted': False})
    blocks, block = [], None
    events = [json.loads(text) for text in (out / 'raw.jsonl').read_text(encoding='utf-8').splitlines()]
    require([event['command'] for event in events if event['direction'] == 'tx'] == metadata['commands'], 'Raw commands differ from saved session')
    cancels = metadata['commands'].count('#CBENCH FULL_CANCEL')
    require(cancels <= 1 and (not cancels or metadata['requested_action'] == 'cancel'
                            or metadata['requested_action'] == 'run' and plan['explicit_cancel_after_ms'] is not None),
            'Raw cancellation lacks explicit intent or was repeated')
    for number, event in enumerate(events):
        try:
            if event['direction'] == 'tx':
                require(block is None or block['committed'], f'Unfinished response before tx at line{number+1}')
                command = event['command']
                require(event.get('hex') == (command + '\n').encode('ascii').hex(), 'Transmitted bytes differ from allowlisted command')
                block = {'command': command, 'boundary': session.boundary(command), 'rows': [], 'ended': False, 'committed': False}
                blocks.append(block)
            elif event['direction'] == 'rx':
                require(block is not None, 'RX before first command')
                if block['ended']:
                    block['boundary'].trailing(event['line'])
                    continue
                block['rows'].append(event['line'])
                if block['boundary'].feed(event['line']):
                    block['ended'] = True
            elif event['direction'] == 'response_commit':
                require(block is not None and block['ended'] and not block['committed'] and event['command'] == block['command'], 'Malformed response transaction commit')
                session.apply(block['command'], block['boundary'], block['rows'])
                block['committed'] = True
        except (ValueError, RuntimeError) as exc:
            require(metadata.get('error') == f'{type(exc).__name__}: {exc}', 'Raw validation failure differs from saved failure')
            require(not any(later['direction'] == 'tx' for later in events[number+1:]), 'Commands transmitted after a saved failure')
            break
    # An incomplete last block is retained as failure evidence, never silently completed.
    if block is not None and not block['committed']:
        require(metadata.get('error') is not None, 'Incomplete response without saved failure')
    result = summarize(session, metadata)
    expected = read_json(out / 'summary.json')
    require(json.loads(json.dumps(result)) == expected, 'Raw replay differs from original summary')
    target = out / ('analysis-' + stamp())
    target.mkdir(exist_ok=False)
    write_json(target / 'summary.json', result)
    write_json(target / 'source-index.json', index)
    return target, result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port')
    action = parser.add_mutually_exclusive_group()
    action.add_argument('--run', action='store_true')
    action.add_argument('--cancel', action='store_true')
    parser.add_argument('--analyze', type=Path)
    parser.add_argument('--scenario', type=int, choices=range(13), default=1)
    parser.add_argument('--fixture', type=int, choices=range(6), default=0)
    parser.add_argument('--variant', type=int, choices=range(1, 4), default=1)
    parser.add_argument('--samples', type=int, choices=range(1, 161), default=160)
    parser.add_argument('--timeout', type=float, default=8.)
    parser.add_argument('--cancel-after-ms', type=int, help='With --run and --port, explicitly send one same-port FULL_CANCEL after0..3000ms if still active; host poll/USB precision only')
    parser.add_argument('--manifest-directory', type=Path, default=REPO / 'firmware_artifacts')
    parser.add_argument('--output-root', type=Path, default=REPO / 'firmware_artifacts')
    args = parser.parse_args(argv)
    if args.cancel_after_ms is not None:
        if not args.run or args.cancel or args.analyze or not args.port:
            parser.error('--cancel-after-ms requires --run and --port and excludes --cancel/--analyze')
        if not 0 <= args.cancel_after_ms <= 3000:
            parser.error('--cancel-after-ms must be0..3000')
    if args.analyze:
        if args.port or args.run or args.cancel:
            parser.error('--analyze excludes all hardware arguments')
        try:
            target, result = analyze(args.analyze)
            print(target)
            print(f'collection_complete={result["collection_complete"]} job_success={result["job_success"]}')
            return 0 if result['collection_complete'] else 2
        except (OSError, ValueError, KeyError) as exc:
            print(type(exc).__name__ + ': ' + str(exc), file=sys.stderr)
            return 2
    if (args.run or args.cancel) and not args.port:
        parser.error('--run/--cancel require explicit --port')
    if not 0 < args.timeout <= 60:
        parser.error('--timeout must be greater than0 and at most60seconds')
    config = configuration(args.scenario, args.fixture, args.variant, args.samples)
    if not args.port:
        print('PLAN ONLY: no serial import, port open, device operation or files written.')
        print('Release17.02; idle DISABLED; OLED split-low; fixed20ms; no motor output.')
        print(run_command(config))
        print('Scenarios: ' + ', '.join(f'{i}={name}' for i, name in enumerate(CASES)))
        return 0
    out = None
    try:
        out, plan = prepare(args.output_root, args.manifest_directory, config, argv, args.cancel_after_ms)
        print(out, flush=True)
        result = collect(out, plan, args.port, args.run, args.cancel, args.timeout)
        print(f'phase={result["status"]["phase"] if result["status"] else "UNKNOWN"} collection_complete={result["collection_complete"]} job_success={result["job_success"]}')
        for error in result['errors']:
            print(error, file=sys.stderr)
        return 0 if result['collection_complete'] else 2
    except (OSError, ValueError, KeyError) as exc:
        print(type(exc).__name__ + ': ' + str(exc), file=sys.stderr)
        if out is not None:
            print('Preserve incomplete preparation: ' + str(out), file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
