"""Bounded memory firmware for native-mode recovery and explicit diagnostics."""
from collections import deque
import threading
import time
import numpy as np
from dummy_motion.frames import from_wire_joints, to_wire_joints
from dummy_motion.stream_protocol import decode_knot, robot_coordinates, motor_coordinates


class StreamMemoryPort:
    def __init__(self):
        self.lock = threading.RLock()
        self.rx, self.writes = bytearray(), []
        self.closed = False
        self.mode = 2
        self.busy = False
        self.session = 0
        self.next = 0
        self.state = 'IDLE'
        self.mask = 127
        self.buffer = deque()
        self.running_at = None
        self.next_tick = None
        self.joints = [0., 10., 80., 0., 90., 0., 0.]
        self.held = motor_coordinates(from_wire_joints(self.joints))[:, 0].copy()
        self.actual_offset = np.zeros(7)
        self.version = 2
        self.drop_feedback = self.old_firmware = self.wrong_done = False
        self.drop_ack = set()
        self.bogus_ack = False
        self.executed = 0
        self.end = False
        self.diag_nonce = 0
        self.motor_velocity = 30.
        self.motor_acceleration = 30.
        self.maximum_buffer = 0
        self.clock_thread = threading.Thread(target=self._clock, daemon=True)
        self.clock_thread.start()

    @property
    def in_waiting(self):
        with self.lock:
            return len(self.rx)

    def read(self, n):
        with self.lock:
            data = bytes(self.rx[:n]); del self.rx[:n]
            return data

    def inject(self, *lines):
        with self.lock:
            self.rx.extend(('\n'.join(lines)+'\n').encode('ascii'))

    def commands(self):
        with self.lock:
            return list(self.writes)

    def snapshot(self):
        return f'STREAM_STATE s={self.session} st={self.state} next={self.next} free={16-len(self.buffer)}'

    def write(self, data):
        command = data.decode('ascii').strip()
        with self.lock:
            self.writes.append(command)
            if command == '#RGB_GET':
                self.inject('RGB_GET end')
            elif command == '!START':
                self.inject('Start pending', 'Started ok')
            elif command.startswith('#GET_CMDMODE '):
                self.inject(f'ok CMDMODE mode={self.mode} busy={int(self.busy)} issued=0 req={command.split()[1]}')
            elif command.startswith('#CMDMODE '):
                self.mode = int(command.split()[1]); self.inject(f'ok CMDMODE mode={self.mode}')
            elif command.startswith(('>', '&', '@')):
                self.motion_id = getattr(self, 'motion_id', 0) + 1
                kind = 'MOVEL' if command.startswith('@') else 'MOVEJ'
                self.inject(f'queued {kind} id={self.motion_id} mode=INT',
                            f'accepted {kind} id={self.motion_id}')
            elif command == '!DISABLE':
                self.buffer.clear(); self.running_at = self.next_tick = None
                self.state = 'FAULT'; self.inject('Disable pending', 'Disabled ok')
            elif command == '#GETJPOS':
                if not self.drop_feedback:
                    self.inject('ok '+' '.join(map(str, self.joints)))
            elif command.startswith('#GETJACC '):
                self.inject(f'ok ACC [{command.split()[1]}] = 30 r/s^2')
            elif command.startswith('#STREAM_DIAG '):
                node, field = map(int, command.split()[1:])
                self.diag_nonce += 1
                value = int(self.held[node-1]) if field == 7 else round((self.motor_velocity if field == 5 else self.motor_acceleration)*51200)
                self.inject(f'STREAM_NODE node={node} n={self.diag_nonce} field={field} value={value}')
            elif command == '#STREAM':
                if not self.old_firmware:
                    self.inject(f'STREAM_CAP v={self.version} hz=50 mode=7 mask={self.mask}', self.snapshot(),
                                f'STREAM_DIAG c={self.next} e={self.executed} gap=5 us=100')
            elif command.startswith('!STREAM_OPEN '):
                if self.session:
                    self.inject(f'error STREAM s={self.session} code=5')
                    return len(data)
                self.session = int(command.split()[1]); self.next = 0
                self.buffer.clear(); self.end = False; self.executed = 0
                self.running_at = self.next_tick = None
                self.state = 'OPEN'; self.inject(self.snapshot())
            elif command.startswith('~S'):
                session, knot = decode_knot(command)
                if session != self.session or knot.sequence != self.next or len(self.buffer) >= 16 or self.end:
                    self.inject(f'STREAM_FAULT s={self.session} code=BAD_SEQUENCE')
                else:
                    self.next += 1; self.buffer.append(knot)
                    self.maximum_buffer = max(self.maximum_buffer, len(self.buffer))
                    self.end = bool(knot.flags & 2)
                    if self.running_at is None:
                        self.state = 'ARMED' if self.next >= 6 or self.end else 'BUFFERING'
                    if knot.sequence not in self.drop_ack:
                        ack = knot.sequence+20 if self.bogus_ack else knot.sequence
                        self.inject(f'STREAM_ACK s={session} n={ack} free={16-len(self.buffer)}')
            elif command.startswith('!STREAM_START '):
                if self.state != 'ARMED':
                    self.inject(f'STREAM_FAULT s={self.session} code=BAD_STATE')
                else:
                    self.running_at = time.perf_counter()+.1
                    self.next_tick = self.running_at
            elif command.startswith('!STREAM_CLOSE '):
                self.session = 0; self.state = 'IDLE'; self.buffer.clear()
                self.running_at = self.next_tick = None
                self.inject(self.snapshot())
            elif command == '!STOP':
                self.buffer.clear(); self.running_at = self.next_tick = None
                self.state = 'FAULT'; self.inject('Stopping ok', 'STOP locked')
            else:
                raise AssertionError('unexpected command: '+command)
        return len(data)

    def _clock(self):
        while not self.closed:
            with self.lock:
                now = time.perf_counter()
                if self.next_tick is not None and now >= self.next_tick:
                    if not self.buffer:
                        self.inject(f'STREAM_FAULT s={self.session} code=UNDERRUN')
                        self.next_tick = None
                    else:
                        knot = self.buffer.popleft()
                        self.held = knot.motor[:, 0].copy()
                        self.joints = to_wire_joints(robot_coordinates(knot.motor)[0])
                        self.joints = list(np.asarray(self.joints) + self.actual_offset)
                        self.executed = knot.sequence
                        self.state = 'RUNNING'
                        self.next_tick += .02
                        if knot.flags & 2:
                            self.state = 'IDLE'; self.next_tick = None
                            self.inject(f'STREAM_DONE s={self.session} n={knot.sequence+int(self.wrong_done)}')
            time.sleep(.002)

    def cancel_read(self):
        pass

    def cancel_write(self):
        pass

    def close(self):
        self.closed = True
        if self.clock_thread is not threading.current_thread():
            self.clock_thread.join(timeout=.2)


