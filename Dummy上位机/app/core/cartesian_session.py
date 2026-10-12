"""Qt-owned host executor. Workers only calculate; every write uses controller gates.

Finite paths are preflighted, jog requests replace intent, and one unwritten slot
is kept. Receipts have a separate bounded ledger. Only matching terminal status
plus stable fresh joint feedback completes a point.
"""
from collections import deque
from dataclasses import replace
from concurrent.futures import ThreadPoolExecutor
import re
import threading
import time
import numpy as np
from PySide6.QtCore import QObject, QTimer, Qt
from dummy_motion import (RobotModel, LinePlanner, MotionSession, FeedbackSample,
                          MotionLimits, Tool, Pose, Status, Sample)
from dummy_motion.feedback import FeedbackTracker
from dummy_motion.frames import (from_xyzabc, from_wire_joints, to_xyzabc, error,
                                  compose, inverse)
from dummy_motion.validation import movej_command
from app.core.cartesian_backend import (CartesianBackend, configured_tool,
                                        point_tool, world_target)


class CartesianSession(QObject):
    def __init__(self, controller, clock=time.monotonic):
        super().__init__(controller)
        self.controller, self.clock = controller, clock
        self.backend = CartesianBackend.FIRMWARE
        self.model = RobotModel()
        self.feedback = FeedbackTracker(self.model)
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="dummy-motion")
        self.future = None
        self.cancel_event = threading.Event()
        self.epoch = 0
        self.active = False
        self.stage, self.message = "idle", "主控逆解"
        self.pending_backend = None
        self.job = None
        self.receipts = deque()
        self.last_id = 0
        self.last_written_q = None
        self._status_at = 0.
        self._timer = QTimer(self)
        self._timer.setTimerType(Qt.TimerType.PreciseTimer)
        self._timer.setInterval(20)
        self._timer.timeout.connect(self.tick)
        self._timer.start()

    @property
    def busy(self):
        return self.active or self.pending_backend is not None

    def _state(self, stage, message):
        self.stage, self.message = stage, message
        self.controller._emit_state()

    def switch(self, backend):
        backend = CartesianBackend(backend)
        c = self.controller
        if backend == self.backend and not self.pending_backend:
            return True
        if (self.busy or c.gamepad.claimed or c._seq_active or c._mode_pending is not None or
                c._mode_request is not None or c._parameter_request is not None or
                c._calibration_busy or c._stop_locked or c._acc_sync_waiting):
            c.log("请结束当前控制或任务后切换逆解后端", "WARN")
            return False
        if not c.is_connected:
            self.backend = backend
            self._state("idle", "上位机七轴逆解" if backend == CartesianBackend.HOST else "主控逆解")
            return True
        if self._has_queued_control():
            c.log("发送队列尚有控制任务，请稍后切换逆解后端", "WARN")
            return False
        self.pending_backend = backend
        self._state("switching", "核对主控空闲状态后切换逆解后端")
        return self._query_mode("switch")

    def _send_job(self, command, purpose, **values):
        c = self.controller
        job = dict(command=command, purpose=purpose, written=False, events=[],
                   generation=c._tx_generation, connection=c._connection_id,
                   deadline=self.clock()+3., **values)
        self.job = job
        if purpose == "motion":
            self.receipts.append(job)
        if not c.send_cmd(command, sequence=True, source="cartesian"):
            self.fail("指令未通过发送门禁")
            return False
        return True

    def _query_mode(self, purpose):
        request = self.controller.allocate_command_mode_request()
        return self._send_job(f"#GET_CMDMODE {request}", purpose, request=request)

    def _available(self):
        c = self.controller
        if (not c.is_connected or not c._motor_state_known or not c._motor_enabled or
                c._stop_locked or c._calibration_busy or c._acc_sync_waiting or
                c._parameter_request is not None or not c._poll_enabled):
            c.log("上位机逆解需要已连接、已使能、无保护任务，并开启关节回读", "WARN")
            return False
        if self.feedback.estimate(self.clock())[0] != Status.OK:
            c.log("上位机逆解需要 300 ms 内的新关节回读", "WARN")
            c._poll_position()
            return False
        return True

    def _has_queued_control(self):
        # GETJPOS is read-only even if RX already retired its request before
        # the queued command_written callback clears the admitted ledger.
        # Keep GETJPOS out of the global diagnostic whitelist: STOP/calibration
        # and parameter transactions must retain their existing polling gates.
        from app.protocol import capabilities
        c = self.controller
        for pending in (c._tx_queue, c._wire_admitted):
            with pending.mutex:
                items = [item[:2] for item in pending.queue if item is not None]
            if any(not capabilities.diagnostic(command) and command != "#GETJPOS"
                   for generation, command in items):
                return True
        return False

    def start_points(self, points, speed=None, sequence=False):
        c = self.controller
        if (self.busy or c.gamepad.claimed or c._seq_active or c._mode_pending is not None or
                c._mode_request is not None or not self._available()):
            c.log("当前不能启动上位机笛卡尔任务", "WARN")
            return False
        if self._has_queued_control():
            c.log("发送队列尚有写入任务，请稍后启动上位机轨迹", "WARN")
            return False
        self._begin("points")
        self.points, self.speed = tuple(points), speed
        self.sequence = sequence
        self.tool = configured_tool(c)
        self.seed = self.feedback.sample.q
        self.deadline = self.clock()+300.
        self._state("checking", "核对空闲状态，准备上位机轨迹")
        return self._query_mode("start")

    def _begin(self, kind):
        self.cancel_event.set()
        self.cancel_event = threading.Event()
        self.epoch += 1
        self.kind, self.active = kind, True
        self.generation = self.controller._tx_generation
        self.connection = self.controller._connection_id
        self.receipts.clear()
        self.issued_ids = deque(maxlen=128)
        self.job = None
        self.last_written_q = None
        self.terminal_id = None
        self.terminal_complete = False
        self.stable_since = None
        self.last_stable_q = None
        self.last_feedback_stamp = None
        self.feedback_wait_started = None
        self.feedback_wait_message = None
        self.plan_waiting_for_feedback = False
        self.last_write_at = self.clock()
        self.future = None
        self.retarget = None
        self._status_at = 0.

    def _feedback_ready(self, now):
        if self.feedback.estimate(now)[0] == Status.OK:
            self.feedback_wait_started = None
            if self.feedback_wait_message is not None:
                message, self.feedback_wait_message = self.feedback_wait_message, None
                self._state(self.stage, message)
            return True
        sample = self.feedback.sample
        age = "无有效回读" if sample is None else f"有效回读年龄 {(now-sample.received_at)*1000:.0f} ms"
        if self.kind == "points" and self.last_written_q is None and not self.receipts:
            # A startup query/UI scheduling gap is not a moving-robot timeout.
            # No plan result or MoveJ may pass this gate until feedback is fresh.
            if self.feedback_wait_started is None:
                self.feedback_wait_started = now
                self.feedback_wait_message = self.message
                self._state(self.stage, "启动前等待新关节回读；尚未发送运动")
                self.controller.log(f"启动前等待新关节回读（{age}，阶段 {self.stage}），最多等待 3 s", "INFO")
            elif now-self.feedback_wait_started >= 3.:
                self.fail(f"启动前等待新关节回读超时（{age}）；未发送运动，请重新发起", stop=False)
            return False
        self.fail(f"关节反馈超过 300 ms，停止上位机轨迹（{age}，阶段 {self.stage}）")
        return False

    def _launch_plan(self):
        self.plan_waiting_for_feedback = True
        if not self._feedback_ready(self.clock()):
            return
        self.plan_waiting_for_feedback = False
        self.seed = self.feedback.sample.q
        seed, points, tool = self.seed, self.points, self.tool
        speed, event = self.speed, self.cancel_event
        model = self.model
        def calculate():
            q, plans = seed, []
            for point in points:
                if event.is_set():
                    return None
                current_tool = point_tool(point, tool)
                current_speed = point.speed if speed is None else speed
                if point.kind == "MOVEJ":
                    q = from_wire_joints(point.target)
                    if not model.valid(q):
                        raise ValueError("MoveJ 点位超限")
                    pose = model.fk(q, current_tool)
                    samples = (Sample(0., q, pose),)
                else:
                    target = world_target(model, point, q[6], current_tool)
                    plan = LinePlanner(model).plan_line(q, target, tool=current_tool, cancelled=event.is_set)
                    if not plan.ok:
                        raise ValueError(f"{point.name}: {plan.status.value} / {plan.reason}")
                    samples = plan.samples[1:]
                    q = plan.samples[-1].q
                plans.append((samples, current_tool, current_speed))
            return plans
        self.future = self.executor.submit(calculate)
        self._state("planning", "离线预检全部点位与路径；尚未发送运动")

    def _use_plan(self, plans):
        if not plans:
            self.fail("轨迹已取消", stop=False)
            return
        # Planning may be expensive, but no motion can run concurrently. Reject
        # an externally moved robot instead of executing a stale starting path.
        if np.max(np.abs(self.feedback.sample.q - self.seed) / np.r_[np.deg2rad([.3]*6), .0005]) > 1:
            self.fail("规划期间实际位置变化，请重新发起", stop=False)
            return
        self.plans, self.point_index = plans, 0
        self.deadline = self.clock()+max(60., 2*sum(samples[-1].time for samples, _, _ in plans)+30.)
        self.controller._seq_active = self.sequence
        self.controller._seq_pending = self.points if self.sequence else ()
        self._next_point()

    def _next_point(self):
        self.samples, self.tool, self.speed_value = self.plans[self.point_index]
        self.sample_index = 0
        self.next_due = self.clock()
        self.terminal_id = None
        self.terminal_complete = False
        self.stable_since = None
        self.last_stable_q = None
        self.controller._seq_idx = self.point_index
        self._state("running", f"上位机轨迹 {self.point_index+1}/{len(self.plans)}")

    def cancel_remaining(self):
        """Preserve the current point, matching the teaching UI's cancel-tail action."""
        if not self.active or self.kind != "points" or not self.sequence:
            return
        self.retarget = None
        if self.stage == "running":
            self.plans = self.plans[:self.point_index+1]
            self.points = self.points[:self.point_index+1]
        else:
            self.cancel()  # No motion has started during checking/preflight.

    def jog(self, tcp):
        c, now = self.controller, self.clock()
        if not c.gamepad.ready or not self._available():
            return False
        if not self.active:
            if self.pending_backend is not None or c.current_cmdmode != 2:
                return False
            self._begin("jog")
            self.tool = configured_tool(c)
            self.seed = self.feedback.sample.q
            self.jog_base = self.model.base_pose(self.seed[6])
            self.session = MotionSession(self.model, self.seed, self.tool)
            self.last_plan_at = now
            self.jog_candidate = None
            self.budget_misses = 0
            self.jog_revision = 0
            self.speed_value = c.gamepad.robot["cmd_speed"]
            self.sequence = False
            self.last_id = c.gamepad._issued
            self.deadline = now+300.
            self._state("jog", "上位机七轴手柄控制")
        if self.kind != "jog":
            return False
        # Gamepad targets keep the entry base frame for the entire session;
        # never add the changing measured rail to an already integrated target.
        self.jog_target = compose(self.jog_base, from_xyzabc(tcp))
        return True

    def jog_idle(self):
        if not self.active or self.kind != "jog":
            return
        # No idle posture/rail optimization or continued pursuit of software lead.
        reference = self.job["q"] if self.job and self.job["purpose"] == "motion" else self.last_written_q
        if reference is None:
            reference = self.seed
        self.session.reference_q = reference
        self.session.velocity[:] = 0.
        self.jog_target = self.model.fk(reference, self.tool)
        self.jog_candidate = None
        self.jog_revision += 1

    def update_feedback(self, values, received_at=None):
        try:
            sample = FeedbackSample(from_wire_joints(values), self.clock() if received_at is None else received_at)
            return self.feedback.update(sample)
        except ValueError:
            return False

    def can_write(self, generation, command):
        job = self.job
        c = self.controller
        if job and job["purpose"] == "motion":
            if (not c._motor_enabled or c.current_cmdmode != 2 or
                    self.feedback.estimate(self.clock())[0] != Status.OK):
                return False
            if self.kind == "jog" and not c.gamepad.ready:
                return False
        return bool(job and not job["written"] and job["command"] == command and
                    job["generation"] == generation and
                    job["connection"] == self.controller._connection_id and self.busy)

    def command_written(self, command, generation, connection):
        job = self.job
        if (job is None or job["written"] or job["command"] != command or
                job["generation"] != generation or job["connection"] != connection):
            return
        job["written"], job["deadline"] = True, self.clock()+3.
        if job["purpose"] == "motion":
            self.last_written_q = job["q"]
            self.last_write_at = self.clock()
            self.job = None
            if self.kind == "jog":
                tcp = compose(inverse(self.jog_base), self.model.fk(job["q"], self.tool))
                self.controller.gamepad.written_reference_tcp = to_xyzabc(tcp)
        for event in job["events"]:
            self.handle_line(event)

    def _defer(self, job, line):
        if not job["written"]:
            if len(job["events"]) >= 16:
                self.fail("回执缓冲异常")
            else:
                job["events"].append(line)
            return True
        return False

    def handle_line(self, line):
        if not self.busy:
            return
        job = self.job
        match = re.fullmatch(r"ok CMDMODE mode=([1-4]) busy=([01]) issued=([0-9]+) req=([0-9]+)", line)
        if match and job and int(match[4]) == job.get("request"):
            if self._defer(job, line):
                return
            mode, busy, issued = map(int, match.groups()[:3])
            self.job = None
            if busy or issued > 0xFFFFFFFF:
                self.fail("主控仍在运动，任务或后端切换未执行", stop=False)
                return
            self.controller.current_cmdmode, self.last_id = mode, issued
            if job["purpose"] == "switch":
                self.backend, self.pending_backend = self.pending_backend, None
                self.controller._request_position("j", False, source="telemetry")
                self._state("idle", "上位机七轴逆解" if self.backend == CartesianBackend.HOST else "主控逆解")
            elif mode != 2:
                self._send_job("#CMDMODE 2", "set_mode")
            else:
                self._launch_plan()
            return
        if line == "ok CMDMODE mode=2" and job and job["purpose"] == "set_mode":
            if not self._defer(job, line):
                self.job = None
                self._query_mode("start")
            return
        match = re.fullmatch(r"queued MOVEJ id=([0-9]+) mode=(SEQ|INT|TRJ|TUN)", line)
        if match and self.active:
            motion_id = int(match[1])
            if not 1 <= motion_id <= 0xFFFFFFFF or not 0 < ((motion_id-self.last_id)&0xFFFFFFFF) < 0x80000000:
                return
            job = next((item for item in self.receipts if item.get("id") is None), None)
            if job is None:
                return
            if self._defer(job, line):
                return
            if match[2] != "INT":
                self.fail("主控运动模式不是 INT")
                return
            job["id"] = self.last_id = motion_id
            self.issued_ids.append(motion_id)
            if self.kind == "jog":
                self.controller.gamepad._issued = motion_id
            if job["terminal"]:
                self.terminal_id = motion_id
            return
        match = re.fullmatch(r"(accepted|error) MOVEJ id=([0-9]+)(?: reason=([A-Z_0-9]+))?", line)
        if match and self.active:
            job = next((item for item in self.receipts if item.get("id") == int(match[2])), None)
            if job is None:
                # RX can precede the Qt command_written callback. Defer to the
                # reserved unnumbered slot; queued will be replayed before this.
                job = next((item for item in self.receipts if not item["written"]), None)
                if job is not None:
                    self._defer(job, line)
                return
            if match[1] == "error":
                self.fail(line)
                return
            self.receipts.remove(job)
            return
        match = re.fullmatch(r"MOTION id=([0-9]+) kind=MOVEJ phase=([0-5]) mode=2 ms=([0-9]+)", line)
        if match and self.active and int(match[1]) == self.terminal_id and self.job and self.job["purpose"] == "status":
            if self._defer(self.job, line):
                return
            self.job = None
            phase = int(match[2])
            if phase == 3:
                self.terminal_complete = True
            elif phase in (0, 4, 5):
                self.fail("主控报告轨迹取消或失败", stop=phase != 5)
            return
        failure = re.fullmatch(r"MOTION fail id=([0-9]+) reason=[A-Z_0-9]+", line)
        if failure:
            if self.active and int(failure[1]) in self.issued_ids:
                self.fail(line, stop=False)
            return
        if line.startswith(("error CMDMODE", "error motion queue full")) or line in ("error DISABLED", "error STOP_LOCKED"):
            self.fail(line)

    def _complete_when_stable(self):
        if not self.terminal_complete or self.receipts or self.last_written_q is None:
            return
        sample = self.feedback.sample
        if sample.received_at == self.last_feedback_stamp:
            return
        self.last_feedback_stamp = sample.received_at
        residual = error(self.model.fk(sample.q, self.tool), self.model.fk(self.last_written_q, self.tool))
        close = (np.all(np.abs(sample.q-self.last_written_q) <= np.r_[np.deg2rad([.3]*6), .0005]) and
                 np.linalg.norm(residual[:3]) <= .001 and np.linalg.norm(residual[3:]) <= np.deg2rad(.5))
        stable = self.last_stable_q is None or np.all(np.abs(sample.q-self.last_stable_q) <= np.r_[np.deg2rad([.05]*6), .0001])
        self.last_stable_q = sample.q
        if not close or not stable:
            self.stable_since = None
            return
        if self.stable_since is None:
            self.stable_since = sample.received_at
        if sample.received_at-self.stable_since < .15:
            return
        self.point_index += 1
        if self.point_index < len(self.plans):
            self._next_point()
        else:
            retarget = self.retarget
            self.cancel()
            if retarget is not None:
                point, speed = retarget
                self.start_points((point,), speed)
                return
            self._state("complete", "上位机轨迹完成：终点编号完成且实际反馈稳定")

    def tick(self):
        if not self.busy:
            return
        now = self.clock()
        if self.job and now > self.job["deadline"] or any(now > item["deadline"] for item in self.receipts):
            self.fail("写出或编号回执超时")
            return
        if not self.active:
            return
        c = self.controller
        if self.generation != c._tx_generation or self.connection != c._connection_id or not c.is_connected:
            self.cancel()
            return
        if not c._motor_enabled or (self.stage in ("running", "jog") and c.current_cmdmode != 2):
            self.fail("电机失能或模式变化，上位机轨迹已取消")
            return
        if now > self.deadline:
            self.fail("轨迹执行超时")
            return
        if not self._feedback_ready(now):
            return
        if self.plan_waiting_for_feedback:
            self._launch_plan()
            return
        if self.future and self.future.done():
            future, self.future = self.future, None
            try:
                result = future.result()
                if self.kind == "points":
                    self._use_plan(result)
                else:
                    status, sample = result
                    if self._future_revision != getattr(self, "jog_revision", 0):
                        return  # Input was centred while this worker calculated.
                    if status == Status.BUDGET_EXCEEDED:
                        self.budget_misses += 1
                        if self.budget_misses >= 3:
                            self.fail("在线逆解连续三次超时")
                    elif status not in (Status.OK, Status.SLOWED):
                        self.fail(f"在线逆解：{status.value}")
                    elif sample is not None and np.linalg.norm(sample.q-self.session.reference_q) > 1e-8:
                        self.budget_misses = 0
                        written = self.last_written_q if self.last_written_q is not None else self.seed
                        lead = error(self.model.fk(written, self.tool), self.model.fk(sample.q, self.tool))
                        # Include reversal/braking results in the same packet
                        # bound. Retain the previous candidate if this one fails.
                        if (np.linalg.norm(lead[:3]) <= .002 and
                                np.linalg.norm(lead[3:]) <= np.deg2rad(1)):
                            self.session.commit(sample.q, self.step_dt)
                            self.jog_candidate = sample
            except Exception as exc:
                self.fail(f"轨迹计算失败：{exc}", stop=self.last_written_q is not None)
            if not self.active or self.kind == "points":
                return
        if self.future or self.job or len(self.receipts) >= 32:
            return
        if self.kind == "points" and self.stage == "running":
            if self.sample_index < len(self.samples) and now >= self.next_due:
                sample = self.samples[self.sample_index]
                speed_scale = self.feedback.speed_scale(self.model.fk(self.last_written_q, self.tool), self.tool, now) if self.last_written_q is not None else 1.
                if speed_scale <= 0:
                    return  # Bound following error without rewinding the reference.
                if self._write_sample(sample, terminal=self.sample_index == len(self.samples)-1):
                    previous_time = self.samples[self.sample_index-1].time if self.sample_index else 0.
                    self.next_due = now+max(.1, sample.time-previous_time)/max(.1, speed_scale)
                    self.sample_index += 1
            elif self.sample_index == len(self.samples):
                if self.terminal_id is not None and not self.terminal_complete and now-self._status_at >= .2:
                    self._status_at = now
                    self._send_job("#MOTION_STATUS", "status")
                self._complete_when_stable()
        elif self.kind == "jog":
            if self.jog_candidate is not None and now-self.last_write_at >= .1:
                sample, self.jog_candidate = self.jog_candidate, None
                self._write_sample(sample, terminal=False, dt=now-self.last_write_at)
                return
            if now-self.last_plan_at < .019:
                return
            # 50 Hz planner, 10 Hz initial wire cadence. Bound the unsent lead
            # to one 2 mm / 1 degree packet; missed frames never cause a burst.
            written = self.last_written_q if self.last_written_q is not None else self.seed
            lead = error(self.model.fk(written, self.tool), self.model.fk(self.session.reference_q, self.tool))
            if np.linalg.norm(lead[:3]) >= .0016 or np.linalg.norm(lead[3:]) >= np.deg2rad(.8):
                return
            self.step_dt = min(.05, max(.001, now-self.last_plan_at))
            self.last_plan_at = now
            # Workers receive snapshots, so centring input cannot mutate a
            # reference halfway through a Jacobian/IK calculation.
            limits = replace(self.session.limits,
                             max_translation_step=max(1e-7, .002-np.linalg.norm(lead[:3])),
                             max_rotation_step=max(1e-7, np.deg2rad(1)-np.linalg.norm(lead[3:])))
            snapshot = MotionSession(self.model, self.session.reference_q, self.tool, limits)
            snapshot.velocity = self.session.velocity.copy()
            snapshot.feedback.sample = self.feedback.sample
            snapshot.feedback.velocity = self.feedback.velocity.copy()
            target, dt, event = self.jog_target, self.step_dt, self.cancel_event
            self._future_revision = self.jog_revision
            self.future = self.executor.submit(snapshot.step, dt, target, now, event.is_set)

    def _write_sample(self, sample, terminal, dt=.1):
        try:
            command, q = movej_command(self.model, sample.q, sample.pose, self.tool, self.speed_value)
        except ValueError as exc:
            self.fail(str(exc))
            return False
        return self._send_job(command, "motion", q=q, terminal=terminal, dt=dt, id=None)

    def fail(self, message, stop=True):
        had_motion = self.active and (self.last_written_q is not None or bool(self.receipts))
        self.cancel()
        self._state("error", message)
        self.controller.log(message, "WARN")
        if had_motion and not stop:
            self.controller._cancel_motion_for_stop()
            self.controller._stop_locked = True
        if stop and had_motion and self.controller.is_connected:
            self.controller.send_cmd("!STOP", source="cartesian")

    def cancel(self):
        self.cancel_event.set()
        self.epoch += 1
        self.retarget = None
        self.future = None
        self.active = False
        self.pending_backend = None
        self.job = None
        self.receipts.clear()
        self.plan_waiting_for_feedback = False
        self.feedback_wait_started = None
        self.feedback_wait_message = None
        if getattr(self, "sequence", False):
            self.controller._seq_active = self.controller._seq_running = False
        self.stage = "idle"

    def close(self):
        self.cancel()
        self._timer.stop()
        self.executor.shutdown(wait=False, cancel_futures=True)
