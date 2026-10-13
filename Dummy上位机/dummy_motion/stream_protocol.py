"""STREAM_JOINT v2 wire ABI; seed is the confirmed motor held reference. No Qt, transport, or live device access.

50 Hz knots contain motor subdivision positions relative to calibrated home,
and their first/second time derivatives. Firmware_new's axis signs and gearing
are part of this versioned ABI; this is not the display/URDF convention.
"""
from dataclasses import dataclass
import binascii
import re
import struct
import numpy as np
from .types import vector

STREAM_VERSION = 2
STREAM_MODE = 7
STREAM_HZ = 50
STREAM_PERIOD = 1 / STREAM_HZ
STREAM_MASK = 127
STREAM_SEED = 1
STREAM_END = 2
STREAM_MAX_SEQUENCE = 65534
STEPS_PER_REV = 51200
MOTOR_SCALE = np.r_[np.array([50., -50., -50., -50., -50., -30.]) * STEPS_PER_REV / (2*np.pi), -200.*STEPS_PER_REV]
MOTOR_ZERO = np.r_[np.deg2rad([0., -75., 180., 0., 0., 0.]), 0.]
MOTOR_SCALE.setflags(write=False)
MOTOR_ZERO.setflags(write=False)
_PACKET = struct.Struct('<BBHH21iH')


def motor_coordinates(q, velocity=None, acceleration=None):
    """Return seven (q, qdot, qddot) integer triples, without clipping."""
    q = vector(q, 7)
    velocity = vector(np.zeros(7) if velocity is None else velocity, 7)
    acceleration = vector(np.zeros(7) if acceleration is None else acceleration, 7)
    values = np.column_stack(((q-MOTOR_ZERO)*MOTOR_SCALE, velocity*MOTOR_SCALE, acceleration*MOTOR_SCALE))
    values = np.rint(values)
    if np.any(values < -2147483648) or np.any(values > 2147483647):
        raise ValueError('Stream coordinate exceeds signed int32')
    values = values.astype(np.int64)
    values.setflags(write=False)
    return values


def robot_coordinates(motor):
    values = np.asarray(motor)
    if values.shape != (7, 3) or not np.isfinite(values).all():
        raise ValueError('Expected seven motor q/v/a triples')
    return (values[:, 0]/MOTOR_SCALE+MOTOR_ZERO,
            values[:, 1]/MOTOR_SCALE, values[:, 2]/MOTOR_SCALE)


@dataclass(frozen=True, eq=False)
class StreamKnot:
    sequence: int
    motor: np.ndarray
    flags: int = 0

    def __post_init__(self):
        if type(self.sequence) is not int or not 0 <= self.sequence <= STREAM_MAX_SEQUENCE:
            raise ValueError('Invalid stream sequence')
        if type(self.flags) is not int or self.flags & ~(STREAM_SEED | STREAM_END):
            raise ValueError('Invalid stream flags')
        values = np.asarray(self.motor)
        if (values.shape != (7, 3) or not np.isfinite(values).all() or
                np.any(values != np.rint(values)) or np.any(values < -2147483648) or
                np.any(values > 2147483647)):
            raise ValueError('Expected seven int32 q/v/a triples')
        if not ((self.sequence == 0 and self.flags == STREAM_SEED) or
                (self.sequence > 0 and self.flags in (0, STREAM_END))):
            raise ValueError('Only sequence zero is the required seed')
        if self.flags & (STREAM_SEED | STREAM_END) and np.any(values[:, 1:] != 0):
            raise ValueError('Seed/end velocity and acceleration must be zero')
        values = values.astype(np.int64, copy=True)
        values.setflags(write=False)
        object.__setattr__(self, 'motor', values)

    def encode(self, session):
        if type(session) is not int or not 1 <= session <= 65535:
            raise ValueError('Invalid stream session')
        payload = _PACKET.pack(STREAM_VERSION, self.flags, session, self.sequence,
                               *self.motor.flat, 0)[:-2]
        crc = binascii.crc_hqx(payload, 0xffff)
        return '~S' + (payload+struct.pack('<H', crc)).hex()


def decode_knot(command):
    """Strict host-side decoder used for replay and cross-language goldens."""
    if re.fullmatch(r'~S[0-9a-fA-F]{184}', command) is None:
        raise ValueError('Invalid stream line length or hexadecimal data')
    raw = bytes.fromhex(command[2:])
    version, flags, session, sequence, *values = _PACKET.unpack(raw)
    if version != STREAM_VERSION or session == 0 or binascii.crc_hqx(raw[:-2], 0xffff) != values.pop():
        raise ValueError('Invalid stream version, session, or CRC')
    return session, StreamKnot(sequence, np.array(values).reshape(7, 3), flags)


def parse_stream_line(line):
    """Return validated structured replies; unrelated text returns None."""
    if line.startswith('STREAM_FAULT_'):
        return {'kind': 'fault_context', 'message': line}
    if line.startswith('error STREAM'):
        return {'kind': 'rejected', 'busy': 'busy' in line.lower() or 'code=5' in line,
                'message': '主控拒绝透传请求：' + line}
    match = re.fullmatch(r'STREAM_TEST s=(\d+) node=([0-7]) st=(IDLE|CAPS|OPENING|PROBING|CLOSING|DONE|FAULT) open=([01]) closed=([01]) code=(\d+)', line)
    if match:
        session, node, state, opened, closed, code = match.groups()
        if int(session) > 65535 or int(code) > 255:
            raise ValueError('OPEN test reply outside protocol bounds')
        return dict(kind='test', session=int(session), node=int(node), state=state,
                    opened=int(opened), closed=int(closed), code=int(code))
    patterns = (
        ('axis_cap', r'STREAM_AXES v=(\d+) hz=(\d+) mask=(\d+) targeted=([01])', ('version', 'hz', 'mask', 'targeted')),
        ('cap', r'STREAM_CAP v=(\d+) hz=(\d+) mode=(\d+) mask=(\d+)', ('version', 'hz', 'mode', 'mask')),
        ('features', r'STREAM_FEATURES s=(\d+) steady=(\d+) start=(\d+)', ('session', 'steady', 'start')),
        ('state', r'STREAM_STATE s=(\d+) st=([A-Z_]+) next=(\d+) free=(\d+)', ('session', 'state', 'next', 'free')),
        ('ack', r'STREAM_ACK s=(\d+) n=(\d+) free=(\d+)', ('session', 'sequence', 'free')),
        ('fault', r'STREAM_FAULT s=(\d+) code=([A-Z_0-9]+)', ('session', 'code')),
        ('done', r'STREAM_DONE s=(\d+) n=(\d+)', ('session', 'sequence')),
        ('diag', r'STREAM_DIAG commit=(\d+) exec=(\d+) gap_ms=(\d+) poll_us=(\d+)', ('committed', 'executed', 'gap_ms', 'poll_us')),
        ('diag', r'STREAM_DIAG c=(\d+) e=(\d+) gap=(\d+) us=(\d+)', ('committed', 'executed', 'gap_ms', 'poll_us')),
        ('node', r'STREAM_NODE node=([1-7]) n=(\d+) field=(0|[1-9]|[1-4][0-9]|50) value=(-?\d+)', ('node', 'nonce', 'field', 'value')),
    )
    for kind, pattern, keys in patterns:
        match = re.fullmatch(pattern, line)
        if match:
            result = {'kind': kind}
            for key, value in zip(keys, match.groups()):
                result[key] = value if key in ('state', 'code') else int(value)
            if result.get('session', 0) > 65535 or result.get('free', 0) > 16 or result.get('next', 0) > 65535:
                raise ValueError('Stream reply is outside protocol bounds')
            if kind == 'features' and (result['steady'] > STREAM_MASK or result['start'] > STREAM_MASK):
                raise ValueError('Motor feature mask outside protocol bounds')
            if kind == 'node' and result['field'] >= 34:
                value, field = result['value'], result['field']
                if field in (37, 38, 39, 40, 41, 42, 47, 50):
                    valid = -2147483648 <= value <= 2147483647
                else:
                    valid = 0 <= value <= (65535 if field == 34 else 0xffffffff)
                if not valid:
                    raise ValueError('Entry diagnostic value outside protocol bounds')
            return result
    if line.startswith(('STREAM_', 'error STREAM')):
        raise ValueError('Malformed or rejected stream reply: ' + line)
    return None


def fault_description(code):
    reasons = ('OK', 'BAD_SESSION', 'BAD_SEQUENCE', 'BAD_CRC', 'BAD_LIMIT', 'BUSY',
               'UNDERRUN', 'SYNC_TIMEOUT', 'SYNC_DRIFT', 'BLOCKED', 'INCOMPLETE',
               'BAD_STATE', 'ABORTED', 'BAD_VERSION', 'BAD_FLAGS', 'BAD_SEED',
               'BAD_TYPE', 'DEADLINE', 'RX_OVERFLOW')
    token = str(code)
    if token.isdigit() and int(token) < len(reasons):
        token += ' (' + reasons[int(token)] + ')'
    return token + '；阶段/节点/序号见完整串口日志'
