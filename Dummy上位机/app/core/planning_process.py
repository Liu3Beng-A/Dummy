"""Disposable spawn process for pure offline planning; no Qt or serial ownership."""
import multiprocessing
import threading


def _calculate(connection, cancelled, request):
    try:
        import numpy as np
        from dummy_motion import StreamPlanner, Tool
        from dummy_motion.frames import from_wire_joints, from_xyzabc
        from dummy_motion.types import MotionLimits
        model, point, speed, tool, seed, acceleration, velocity = request
        planner = StreamPlanner(model, acceleration, velocity)
        if point['tool'] is not None:
            tool = Tool(from_xyzabc(point['tool']), 'point-snapshot')
        if point['kind'] == 'MOVEJ':
            plan = planner.plan_joint(seed, from_wire_joints(point['target']), speed, cancelled.is_set)
        else:
            target = from_xyzabc(point['target'])
            if point['frame'] == 'base' and point['target_kind'] == 'flange':
                target = model.legacy_target(target, seed[6], tool)
            elif point['frame'] != 'world' or point['target_kind'] != 'tcp':
                raise ValueError('Unsupported Cartesian frame/target combination')
            limits = MotionLimits(linear_speed=.02*speed/100, angular_speed=np.deg2rad(10)*speed/100)
            plan = planner.plan_line(seed, target, limits, tool, cancelled.is_set)
        if not cancelled.is_set():
            connection.send((True, plan))
    except BaseException as exc:
        if not cancelled.is_set():
            connection.send((False, f'{type(exc).__name__}: {exc}'))
    finally:
        connection.close()


class PlanningProcess:
    """Future-like bounded lifetime, with a separate process for every plan.

    terminate is safe here: the child owns no device, file, or shared model.
    It also bounds cancellation when a numeric library does not yield.
    """
    def __init__(self, request):
        context = multiprocessing.get_context('spawn')
        self.cancelled = context.Event()
        self.receiver, sender = context.Pipe(duplex=False)
        self.process = context.Process(target=_calculate, args=(sender, self.cancelled, request),
                                       name='Dummy offline curve planner', daemon=True)
        self.lock = threading.RLock()
        self.outcome = None
        self.closed = False
        self.process.start()
        sender.close()

    def done(self):
        with self.lock:
            if self.outcome is not None:
                return True
            if self.closed:
                raise InterruptedError('Planning cancelled')
            if self.receiver.poll():
                try:
                    self.outcome = self.receiver.recv()
                except EOFError:
                    self.outcome = (False, 'Planning process exited without result')
            elif not self.process.is_alive():
                self.outcome = (False, f'Planning process exited: {self.process.exitcode}')
            return self.outcome is not None

    def result(self):
        if not self.done():
            raise RuntimeError('Planning result is not ready')
        ok, value = self.outcome
        self.close()
        if not ok:
            raise RuntimeError(value)
        return value

    def cancel(self):
        self.close()

    def close(self):
        with self.lock:
            if self.closed:
                return
            self.closed = True
            self.cancelled.set()
            if self.process.is_alive():
                self.process.terminate()
            self.process.join(timeout=.2)
            self.receiver.close()
