"""Reproducible CPU-only benchmark. It does not measure serial/device latency."""
import argparse
from collections import Counter
import json
from pathlib import Path
import platform
import statistics
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
import scipy
from dummy_motion import RobotModel, IKSolver, MotionSession, FeedbackSample, Pose, LinePlanner
from dummy_motion.frames import from_wire_joints


def summarize(values):
    return {"samples": len(values), "median_ms": statistics.median(values),
            "p95_ms": float(np.percentile(values, 95)), "p99_ms": float(np.percentile(values, 99)),
            "max_ms": max(values)}


def run(count=500):
    model = RobotModel()
    seed = from_wire_joints([0, 10, 80, 0, 90, 0, 0])
    solver = IKSolver(model)
    rng = np.random.default_rng(20261012)
    times, statuses = [], Counter()
    for i in range(count+20):
        target_q = seed.copy()
        target_q[:6] += rng.uniform(-.003, .003, 6)
        target_q[6] += rng.uniform(-.0005, .0005)
        target = model.fk(target_q)
        start = time.perf_counter()
        result = solver.solve_pose(target, seed, max_iterations=8, budget_ms=10.)
        elapsed = (time.perf_counter()-start)*1000
        if i >= 20:
            times.append(elapsed)
            statuses[result.status.value] += 1
    online_times, online_status = [], Counter()
    for i in range(count):
        session = MotionSession(model, seed)
        session.update_feedback(FeedbackSample(seed, 1.))
        pose = model.fk(seed)
        target = Pose(pose.position+[0, .01, 0], pose.rotation)
        start = time.perf_counter()
        status, sample = session.step(.02, target, 1.01)
        online_times.append((time.perf_counter()-start)*1000)
        online_status[status.value] += 1
    line_times, line_status = [], Counter()
    for _ in range(20):
        pose = model.fk(seed)
        target = Pose(pose.position+[.008, .02, -.005], pose.rotation)
        start = time.perf_counter()
        plan = LinePlanner(model).plan_line(seed, target)
        line_times.append((time.perf_counter()-start)*1000)
        line_status[plan.status.value] += 1
    return {"environment": {"python": platform.python_version(), "platform": platform.platform(),
                            "processor": platform.processor(), "numpy": np.__version__, "scipy": scipy.__version__},
            "nearby_ik": dict(summarize(times), status=dict(statuses)),
            "online_20ms_start_step": dict(summarize(online_times), status=dict(online_status)),
            "line_22mm_preflight": dict(summarize(line_times), status=dict(line_status)),
            "note": "CPU-only; ordinary Windows scheduling. No USB, firmware, CAN, motor or physical accuracy measurement."}


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path)
    parser.add_argument('--count', type=int, default=500)
    args = parser.parse_args()
    result = run(args.count)
    text = json.dumps(result, ensure_ascii=False, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text+'\n', encoding='utf-8')
    print(text)
