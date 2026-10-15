"""Host-only CartBench component and boundary tests; never opens a device.

Freezes inputs and keeps logs/results under firmware_artifacts. Synthetic clock
values prove accounting and cutoffs only; they are not desktop or F405 timing.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys

REPO = Path(__file__).resolve().parents[2]
CORE = Path("firmware_new/dummy-ref-core-fw/Robot")
IK = CORE / "algorithms/kinematic"
INST = CORE / "instances"
INPUTS = [INST / "cartesian_benchmark.h", INST / "cartesian_benchmark.cpp",
          INST / "cartesian_planner.h", INST / "cartesian_planner.cpp",
          IK / "6dof_kinematic.h", IK / "6dof_kinematic.cpp",
          Path("firmware_new/stream_protocol.h"),
          Path("tools/tests/fixtures/arm_math_native_shim.h"),
          Path("tools/tests/cartesian_benchmark_test.cpp"),
          Path("tools/tests/run_cartesian_benchmark_test.py")]


def digest(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def run(out: Path, mode: str) -> dict:
    build = out / mode
    build.mkdir()
    snapshot = out / "source"
    shutil.copyfile(snapshot / "tools/tests/fixtures/arm_math_native_shim.h", build / "arm_math.h")
    sources = [snapshot / "tools/tests/cartesian_benchmark_test.cpp",
               snapshot / INST / "cartesian_benchmark.cpp",
               snapshot / INST / "cartesian_planner.cpp", snapshot / IK / "6dof_kinematic.cpp"]
    includes = [build, snapshot / INST, snapshot / IK]
    exe = build / ("cartesian_benchmark_test.exe" if os.name == "nt" else "cartesian_benchmark_test")
    compiler_info = {}
    if os.name == "nt":
        vswhere = Path(os.environ.get("ProgramFiles(x86)", "C:/Program Files (x86)")) / "Microsoft Visual Studio/Installer/vswhere.exe"
        raw = subprocess.check_output([str(vswhere), "-latest", "-products", "*", "-requires",
            "Microsoft.VisualStudio.Component.VC.Tools.x86.x64", "-property", "installationPath"])
        installation = raw.decode("utf-8-sig", errors="replace").strip()
        if not installation:
            raise RuntimeError("Visual Studio x64 C++ tools not found")
        vcvars = Path(installation) / "VC/Auxiliary/Build/vcvars64.bat"
        command = ["cl", "/nologo", "/utf-8", "/std:c++17", "/O2", "/W4", "/EHsc", "/fp:precise"]
        if mode == "cmsis512":
            command.append("/DCMSIS_INTERPOLATED")
        command += ["/I" + str(p) for p in includes] + [str(p) for p in sources] + ["/Fe:" + str(exe)]
        script = build / "build.cmd"
        script.write_text('@echo off\ncall "' + str(vcvars) + '" > nul\nif errorlevel 1 exit /b 1\n'
                          + 'where cl > compiler_path.log 2>&1\ncl /Bv > compiler_version.log 2>&1\n'
                          + subprocess.list2cmdline(command) + '\nexit /b %errorlevel%\n', encoding="utf-8")
        result = subprocess.run(["cmd.exe", "/d", "/c", str(script)], cwd=build, capture_output=True)
        compiler_info = {"vcvars": str(vcvars), "vcvars_sha256": digest(vcvars), "vswhere": str(vswhere)}
    else:
        compiler = shutil.which("g++") or shutil.which("clang++")
        if not compiler:
            raise RuntimeError("C++17 compiler not found")
        command = [compiler, "-std=c++17", "-O2", "-Wall", "-Wextra"]
        if mode == "cmsis512":
            command.append("-DCMSIS_INTERPOLATED")
        command += ["-I" + str(p) for p in includes] + [str(p) for p in sources] + ["-o", str(exe)]
        version = subprocess.run([compiler, "--version"], capture_output=True)
        (build / "compiler_version.log").write_bytes(version.stdout + version.stderr)
        result = subprocess.run(command, cwd=build, capture_output=True)
    (build / "compile.log").write_bytes(result.stdout + result.stderr)
    metadata = {"compile_command": command, "mode": mode, "mcu_timing": False, **compiler_info}
    (build / "build.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    if result.returncode:
        raise RuntimeError("Host compile failed: " + str(build / "compile.log"))
    tested = subprocess.run([str(exe)], cwd=build, capture_output=True)
    (build / "results.json").write_bytes(tested.stdout)
    (build / "run.stderr.log").write_bytes(tested.stderr)
    report = json.loads(tested.stdout)
    if tested.returncode or report["failures"]:
        raise RuntimeError("Host tests failed: " + str(build))
    return {"mode": mode, "assertions": report["assertions"], "rows": len(report["rows"]),
            "failures": report["failures"], "executable_sha256": digest(exe)}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--math", choices=("precise", "cmsis512", "both"), default="both")
    args = parser.parse_args()
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    out = REPO / "firmware_artifacts" / ("cartesian-benchmark-host-" + stamp)
    out.mkdir(parents=True)
    before = {p.as_posix(): digest(REPO / p) for p in INPUTS}
    for p in INPUTS:
        dest = out / "source" / p
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(REPO / p, dest)
    (out / "environment.json").write_text(json.dumps({"utc": stamp, "platform": platform.platform(),
        "python": sys.version, "python_executable": sys.executable, "argv": sys.argv,
        "mcu_timing": False, "clock": "synthetic deterministic hooks, not performance measurement"}, indent=2), encoding="utf-8")
    reports, failure = [], None
    try:
        for mode in (("precise", "cmsis512") if args.math == "both" else (args.math,)):
            reports.append(run(out, mode))
    except Exception as error:
        failure = repr(error)
        (out / "failure.txt").write_text(failure + "\n", encoding="utf-8")
    after = {p.as_posix(): digest(REPO / p) for p in INPUTS}
    identity = {"before": before, "after": after, "source_unchanged": before == after}
    (out / "source_identity.json").write_text(json.dumps(identity, indent=2), encoding="utf-8")
    summary = {"passed": failure is None and before == after, "reports": reports, "failure": failure,
               "mcu_timing": False, "output": str(out)}
    (out / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    files = {p.relative_to(out).as_posix(): digest(p) for p in out.rglob("*") if p.is_file()}
    (out / "evidence_index.json").write_text(json.dumps({"files": files, "file_count": len(files)}, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
