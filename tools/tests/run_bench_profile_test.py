"""Host-only task/OLED-stage accounting tests; never opens a device.

Freezes sources and records compiler commands, logs and hashes. Deterministic
timestamps test accounting only, not F405 timing, instrumentation overhead or
exclusive CPU use. OLED stages separately account elapsed time and selected
OLED-task residency. Each invocation creates a new firmware_artifacts batch.
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
CONFIG = Path("firmware_new/dummy-ref-core-fw/Core/Inc/FreeRTOSConfig.h")
INPUTS = [INST / "bench_profile.h", INST / "bench_profile.cpp", CONFIG,
          Path("tools/tests/bench_profile_test.cpp"), Path("tools/tests/run_bench_profile_test.py")]


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(out: Path, mode: str) -> dict:
    build = out / mode
    build.mkdir()
    snapshot = out / "source"
    exe = build / ("bench_profile_test.exe" if os.name == "nt" else "bench_profile_test")
    # A C translation unit includes the actual FreeRTOS config, expands its
    # switch macro, and references the C ABI. The C++ test separately includes
    # that config FIRST inside extern "C", mirroring the firmware include order;
    # its calls must link to the normally included backend's C++ namespace API.
    c_source = build / "header_check.c"
    c_source.write_text('#include "FreeRTOSConfig.h"\n'
                        'void BenchProfileHeaderCheck(void* task) {\n'
                        '    void* pxCurrentTCB = task;\n'
                        '    (void)benchOledSessionActive;\n'
                        '    traceTASK_SWITCHED_IN();\n}\n', encoding="utf-8")
    sources = [snapshot / "tools/tests/bench_profile_test.cpp",
               snapshot / INST / "bench_profile.cpp"]
    includes = [snapshot / INST, snapshot / CONFIG.parent]
    injected = mode == "injected-clock"
    if os.name == "nt":
        vswhere = Path(os.environ.get("ProgramFiles(x86)", "C:/Program Files (x86)")) / "Microsoft Visual Studio/Installer/vswhere.exe"
        raw = subprocess.check_output([str(vswhere), "-latest", "-products", "*", "-requires",
            "Microsoft.VisualStudio.Component.VC.Tools.x86.x64", "-property", "installationPath"])
        installation = raw.decode("utf-8-sig", errors="replace").strip()
        if not installation:
            raise RuntimeError("Visual Studio x64 C++ tools not found")
        vcvars = Path(installation) / "VC/Auxiliary/Build/vcvars64.bat"
        common = ["cl", "/nologo", "/utf-8", "/O2", "/W4", "/WX"]
        common += ["/I" + str(path) for path in includes]
        if injected:
            common.append("/DBENCH_PROFILE_TEST_CLOCK")
        c_object = build / "header_check.obj"
        c_command = common + ["/TC", "/c", str(c_source), "/Fo:" + str(c_object)]
        command = common + ["/std:c++17", "/EHsc"] + list(map(str, sources)) + [str(c_object), "/Fe:" + str(exe)]
        script = build / "build.cmd"
        script.write_text('@echo off\ncall "' + str(vcvars) + '" > nul\nif errorlevel 1 exit /b 1\n'
                          + 'where cl > compiler_path.log 2>&1\ncl /Bv > compiler_version.log 2>&1\n'
                          + subprocess.list2cmdline(c_command) + '\nif errorlevel 1 exit /b 1\n'
                          + subprocess.list2cmdline(command) + '\nexit /b %errorlevel%\n', encoding="utf-8")
        result = subprocess.run(["cmd.exe", "/d", "/c", str(script)], cwd=build, capture_output=True)
        compiler = {"vcvars": str(vcvars), "vcvars_sha256": sha(vcvars), "vswhere": str(vswhere)}
    else:
        cpp = shutil.which("g++") or shutil.which("clang++")
        cc = shutil.which("gcc") or shutil.which("clang")
        if not cpp or not cc:
            raise RuntimeError("Host C and C++17 compilers not found")
        common = ["-O2", "-Wall", "-Wextra", "-Werror"] + ["-I" + str(path) for path in includes]
        if injected:
            common.append("-DBENCH_PROFILE_TEST_CLOCK")
        c_object = build / "header_check.o"
        c_command = [cc, "-std=c11"] + common + ["-c", str(c_source), "-o", str(c_object)]
        command = [cpp, "-std=c++17"] + common + list(map(str, sources)) + [str(c_object), "-o", str(exe)]
        version = subprocess.run([cpp, "--version"], capture_output=True)
        (build / "compiler_version.log").write_bytes(version.stdout + version.stderr)
        c_result = subprocess.run(c_command, cwd=build, capture_output=True)
        (build / "c_compile.log").write_bytes(c_result.stdout + c_result.stderr)
        if c_result.returncode:
            raise RuntimeError("C header compile failed: " + str(build / "c_compile.log"))
        result = subprocess.run(command, cwd=build, capture_output=True)
        compiler = {"c_compiler": cc, "cpp_compiler": cpp}
    (build / "compile.log").write_bytes(result.stdout + result.stderr)
    (build / "commands.json").write_text(json.dumps({"c_command": c_command, "cpp_link_command": command,
        "mode": mode, "compiler": compiler, "mcu_timing": False,
        "header_contexts": ["C configuration macro expansion", "C++ config-first nested extern C",
                            "C++ backend direct header inclusion"]}, indent=2), encoding="utf-8")
    if result.returncode:
        raise RuntimeError("Host compile failed: " + str(build / "compile.log"))
    tested = subprocess.run([str(exe)], cwd=build, capture_output=True)
    (build / "results.json").write_bytes(tested.stdout)
    (build / "run.stderr.log").write_bytes(tested.stderr)
    report = json.loads(tested.stdout)
    if tested.returncode or report["failures"]:
        raise RuntimeError("Host tests failed: " + str(build))
    return {"mode": mode, **report, "executable_sha256": sha(exe)}


def main() -> int:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    out = ROOT / "firmware_artifacts" / ("bench-profile-host-" + stamp)
    out.mkdir(parents=True)
    before = {str(path): sha(ROOT / path) for path in INPUTS}
    for path in INPUTS:
        target = out / "source" / path
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / path, target)
    (out / "environment.json").write_text(json.dumps({"utc": stamp, "platform": platform.platform(),
        "python": sys.version, "python_executable": sys.executable, "argv": sys.argv,
        "mcu_timing": False, "devices_opened": False, "firmware_built": False,
        "note": "Residency includes IRQ and hook overhead; OLED-stage elapsed includes preemption; host clocks are synthetic."}, indent=2), encoding="utf-8")
    reports, failure = [], None
    try:
        for mode in ("injected-clock", "portable-no-clock"):
            reports.append(run(out, mode))
    except Exception as error:
        failure = repr(error)
        (out / "failure.txt").write_text(failure + "\n", encoding="utf-8")
    after = {str(path): sha(ROOT / path) for path in INPUTS}
    identity = {"before": before, "after": after, "source_unchanged": before == after}
    (out / "source_identity.json").write_text(json.dumps(identity, indent=2), encoding="utf-8")
    summary = {"passed": failure is None and before == after, "reports": reports, "failure": failure,
               "mcu_timing": False, "output": str(out)}
    (out / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    files = {path.relative_to(out).as_posix(): sha(path) for path in out.rglob("*") if path.is_file()}
    (out / "evidence_index.json").write_text(json.dumps({"files": files, "file_count": len(files)}, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
