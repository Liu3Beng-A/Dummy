"""Host logic tests of extracted, unmodified production CBENCH adapter.

Never opens USB/ST-Link, builds target firmware, flashes, or sends motor data.
Fake RTOS/DWT time tests control flow only; it is not F405 timing evidence.
Each invocation freezes inputs and retains generated extraction/build/run logs.
"""
from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
INST = Path("firmware_new/dummy-ref-core-fw/Robot/instances")
KIN = Path("firmware_new/dummy-ref-core-fw/Robot/algorithms/kinematic")
INPUTS = [INST / p for p in ("cartesian_benchmark.h", "cartesian_benchmark.cpp",
          "native_cartesian_robot.cpp", "dummy_robot.cpp", "motion_values.h",
          "motion_values.cpp", "cartesian_planner.h", "cartesian_planner.cpp")]
INPUTS += [KIN / "6dof_kinematic.h", KIN / "6dof_kinematic.cpp",
           Path("firmware_new/stream_protocol.h"),
           Path("firmware_new/dummy-ref-core-fw/UserApp/main.cpp"),
           Path("tools/tests/fixtures/arm_math_native_shim.h"),
           Path("tools/tests/cartesian_benchmark_adapter_test.cpp"),
           Path("tools/tests/cartesian_benchmark_adapter_test.py")]


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def function(source: str, signature: str) -> str:
    start = source.index(signature)
    brace = source.index("{", start)
    level, end = 1, brace + 1
    while level:
        level += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


def build(out: Path, snap: Path, mode: str) -> tuple[Path, list[str]]:
    dest = out / mode
    dest.mkdir()
    shutil.copyfile(snap / "tools/tests/fixtures/arm_math_native_shim.h", dest / "arm_math.h")
    exe = dest / ("adapter_test.exe" if os.name == "nt" else "adapter_test")
    sources = [snap / "tools/tests/cartesian_benchmark_adapter_test.cpp",
               snap / INST / "cartesian_benchmark.cpp", snap / INST / "motion_values.cpp",
               snap / INST / "cartesian_planner.cpp", snap / KIN / "6dof_kinematic.cpp"]
    includes = [out, dest, snap / INST, snap / KIN]
    if os.name == "nt":
        where = Path(os.environ.get("ProgramFiles(x86)", "C:/Program Files (x86)")) / "Microsoft Visual Studio/Installer/vswhere.exe"
        installation = subprocess.check_output([str(where), "-latest", "-products", "*",
            "-requires", "Microsoft.VisualStudio.Component.VC.Tools.x86.x64", "-property",
            "installationPath"]).decode("utf-8", errors="replace").strip()
        if not installation:
            raise RuntimeError("Visual Studio x64 C++ tools missing")
        vcvars = Path(installation) / "VC/Auxiliary/Build/vcvars64.bat"
        cmd = ["cl", "/nologo", "/utf-8", "/std:c++17", "/O2", "/W4", "/EHsc", "/fp:precise"]
        if mode == "cmsis512":
            cmd.append("/DCMSIS_INTERPOLATED")
        cmd += ["/I" + str(p) for p in includes] + list(map(str, sources)) + ["/Fe:" + str(exe)]
        script = dest / "build.cmd"
        script.write_text('@echo off\ncall "' + str(vcvars) + '" > nul\nif errorlevel 1 exit /b 1\n'
                          + subprocess.list2cmdline(cmd) + '\nexit /b %errorlevel%\n', encoding="utf-8")
        run = subprocess.run(["cmd.exe", "/d", "/c", str(script)], cwd=dest, capture_output=True)
    else:
        compiler = shutil.which("g++") or shutil.which("clang++")
        if not compiler:
            raise RuntimeError("C++17 compiler missing")
        cmd = [compiler, "-std=c++17", "-O2", "-Wall", "-Wextra"]
        if mode == "cmsis512":
            cmd.append("-DCMSIS_INTERPOLATED")
        cmd += ["-I" + str(p) for p in includes] + list(map(str, sources)) + ["-o", str(exe)]
        run = subprocess.run(cmd, cwd=dest, capture_output=True)
    (dest / "compile.log").write_bytes(run.stdout + run.stderr)
    if run.returncode:
        raise RuntimeError(f"Compilation failed: {dest / 'compile.log'}")
    return exe, cmd


def main() -> int:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    out = ROOT / "firmware_artifacts" / ("cartesian-benchmark-adapter-" + stamp)
    snap = out / "source"
    snap.mkdir(parents=True)
    before = {p.as_posix(): sha(ROOT / p) for p in INPUTS}
    for p in INPUTS:
        (snap / p).parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / p, snap / p)
    native = (snap / INST / "native_cartesian_robot.cpp").read_text(encoding="utf-8-sig")
    robot = (snap / INST / "dummy_robot.cpp").read_text(encoding="utf-8-sig")
    start = native.index("enum class BenchPhase")
    end = native.index('    if(Equal(cmd,n,"#KIN_CONFIG")){', start)
    # Prefix is genuine production code; the only synthetic tail closes the
    # partially extracted handler after its diagnostic and busy gate branches.
    fragment = "namespace {\nvolatile bool plannerLease=false;\n"
    fragment += function(native, "bool Equal(") + "\n" + function(native, "bool Prefix(") + "\n"
    fragment += native[start:end] + "    return false;\n}\n"
    fragment += function(robot, "bool DummyRobot::RequestStart()") + "\n"
    fragment += function(robot, "bool DummyRobot::HandleProtectionCommand(") + "\n"
    (out / "adapter_fragment.inc").write_text(fragment, encoding="utf-8")
    # This compilation has no motor/CAN/Flash API declarations. Adding such a
    # dependency to the extracted diagnostic must fail compilation.
    contract = {
        "extract_start": "enum class BenchPhase",
        "extract_end": 'if(Equal(cmd,n,"#KIN_CONFIG")){',
        "production_fragment_sha256": hashlib.sha256(fragment.encode()).hexdigest(),
        "actual_start_and_calibration_methods_extracted": True,
        "command_task_calls_poll": "CartesianBenchmarkPoll()" in (snap / "firmware_new/dummy-ref-core-fw/UserApp/main.cpp").read_text(encoding="utf-8-sig"),
        "no_hardware": True,
        "fake_timing_is_not_f405_timing": True,
    }
    if not contract["command_task_calls_poll"]:
        raise RuntimeError("Command-task integration missing")
    results = []
    try:
        for mode in ("precise", "cmsis512"):
            exe, cmd = build(out, snap, mode)
            run = subprocess.run([str(exe)], cwd=exe.parent, capture_output=True)
            (exe.parent / "run.log").write_bytes(run.stdout + run.stderr)
            results.append({"mode": mode, "command": cmd, "exit_code": run.returncode,
                            "executable_sha256": sha(exe)})
            if run.returncode:
                raise RuntimeError(f"Adapter tests failed: {exe.parent / 'run.log'}")
    finally:
        after = {p.as_posix(): sha(ROOT / p) for p in INPUTS}
        identity = {"before": before, "after": after, "unchanged": before == after}
        (out / "source_identity.json").write_text(json.dumps(identity, indent=2), encoding="utf-8")
        (out / "results.json").write_text(json.dumps({"contract": contract, "runs": results,
            "python": sys.version, "platform": platform.platform(),
            "command": sys.argv, "utc": stamp}, indent=2), encoding="utf-8")
        files = {p.relative_to(out).as_posix(): sha(p) for p in out.rglob("*") if p.is_file()}
        (out / "evidence_manifest.json").write_text(json.dumps(files, indent=2), encoding="utf-8")
    if before != after:
        raise RuntimeError("Source changed during tests; preserve results but rerun")
    print(str(out))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
