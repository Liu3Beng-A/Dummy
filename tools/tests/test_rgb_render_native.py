"""Compile actual RGB::Render/Phase against rgb.hpp and compare host preview.

Requires Windows Visual Studio C++ tools. Only the constructor and tim.h are
stubs: no HAL, DMA, RTOS, board timing or physical light output is exercised.
Outputs stay in a temporary directory unless --output is supplied.
"""
from array import array
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile


def function(source, signature):
    start = source.index(signature)
    pos = source.index("{", start) + 1
    depth = 1
    while depth:
        depth += (source[pos] == "{") - (source[pos] == "}")
        pos += 1
    return source[start:pos]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--baseline", type=Path, help="Optional pre-change rgb.cpp")
    parser.add_argument("--changed-mode", type=int, default=8, choices=range(13),
                        help="Mode excluded from optional baseline equality check")
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[2]
    core = repo / "firmware_new/dummy-ref-core-fw/Bsp/gpio"
    preview = repo / "Dummy上位机/app/core/rgb_simulator.py"
    inputs = [core / "rgb.cpp", core / "rgb.hpp", core / "rgb_rainbow.hpp", preview,
              preview.with_name("rgb_rainbow.py")]
    hashes = lambda: {str(p.relative_to(repo)): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
    before = hashes()
    out = (args.output or Path(tempfile.mkdtemp(prefix="rgb-render-"))).resolve()
    out.mkdir(parents=True, exist_ok=True)
    (out / "rgb.hpp").write_bytes((core / "rgb.hpp").read_bytes())
    (out / "rgb_rainbow.hpp").write_bytes((core / "rgb_rainbow.hpp").read_bytes())
    (out / "tim.h").write_text("// No hardware declarations needed by RGB class.\n")
    sys.path.insert(0, str(preview.parents[2]))
    from app.core.rgb_simulator import _Effect, _Scene, _render

    # Valid wire settings, both directions, all modes, custom colors and state
    # scales; spin includes every legal tail value. Dense default tests below
    # cover every millisecond across an entire revolution and its wrap.
    cases = []
    for mode in range(13):
        for reverse in (0, 1):
            for tail in (range(1, 13) if mode == 8 else (3,)):
                for period in (250, 1400, 6000, 7000, 30000):
                    times = sorted(set(range(0, period, max(1, period // 48))) |
                                   {period - 1, period, period + 1, 0xFFFFFFFF})
                    for scale in (0.15, 0.25, 1.0):
                        for elapsed in times:
                            cases.append((mode, reverse, tail, period, elapsed, scale))
    dense = {}
    for reverse in (0, 1):
        dense[reverse] = len(cases)
        cases.extend((8, reverse, 3, 1400, t, 1.0) for t in range(1401))
    rainbow_dense = {}
    for reverse in (0, 1):
        rainbow_dense[reverse] = len(cases)
        cases.extend((3, reverse, 3, 6000, t, 1.0) for t in range(6001))
    payload = "".join("%d %d %d %d %d %.8g\n" % c for c in cases).encode("ascii")

    vswhere = Path(os.environ.get("ProgramFiles(x86)", "C:/Program Files (x86)")) / "Microsoft Visual Studio/Installer/vswhere.exe"
    vs = subprocess.check_output([str(vswhere), "-latest", "-products", "*", "-requires",
                                  "Microsoft.VisualStudio.Component.VC.Tools.x86.x64", "-property", "installationPath"], text=True).strip()
    vcvars = Path(vs) / "VC/Auxiliary/Build/vcvars64.bat"

    def compile_run(source, name):
        parts = "\n".join(function(source, s) for s in ("float RGB::Smooth(", "float RGB::Phase(", "void RGB::Render("))
        harness = '''#include <cmath>
#include <cstdio>
#include <io.h>
#include <fcntl.h>
#define private public
#include "rgb.hpp"
#undef private
#include "rgb_rainbow.hpp"
constexpr float PI_F = 3.14159265358979323846f;
RGB::RGB(uint8_t) {} // Constructor alone is stubbed, renderer is production.
'''+parts+'''
int main() {
    _setmode(_fileno(stdout), _O_BINARY);
    RGB rgb;
    unsigned mode, reverse, tail, period, elapsed; float scale;
    while (scanf("%u %u %u %u %u %f", &mode, &reverse, &tail, &period, &elapsed, &scale)==6) {
        RGB::Scene s{}; s.mode=mode; s.scale=scale; s.solid={255,91,37};
        s.effect.first=s.solid; s.effect.second={17,180,255};
        s.effect.periodMs=period; s.effect.reverse=reverse; s.effect.tail=tail;
        s.effect.onMs=200; s.effect.offMs=1000;
        float frame[RGB::LED_COUNT][3]; rgb.Render(s, elapsed, frame);
        if (fwrite(frame, sizeof(frame), 1, stdout)!=1) return 2;
    }
    return ferror(stdin) ? 3 : 0;
}
'''
        cpp = out / (name + ".cpp")
        cpp.write_text(harness, encoding="utf-8")
        cmd = out / (name + ".cmd")
        cmd.write_text('@echo off\ncall "'+str(vcvars)+'" > nul\nif errorlevel 1 exit /b 1\n'
                       + f'cl /nologo /utf-8 /std:c++17 /O2 /EHsc /fp:precise /I"{out}" "{cpp}" /Fe:{name}.exe\nexit /b %errorlevel%\n')
        built = subprocess.run(["cmd.exe", "/d", "/c", str(cmd)], cwd=out, capture_output=True)
        (out / (name + "-build.log")).write_bytes(built.stdout + built.stderr)
        built.check_returncode()
        output = subprocess.run([str(out / (name + ".exe"))], input=payload, capture_output=True, check=True)
        frames = array("f")
        frames.frombytes(output.stdout)
        assert len(frames) == len(cases) * 72
        return frames

    source = (core / "rgb.cpp").read_text(encoding="utf-8")
    actual = compile_run(source, "rgb_render")
    error = 0.0
    for n, (mode, reverse, tail, period, elapsed, scale) in enumerate(cases):
        effect = _Effect((255, 91, 37), (17, 180, 255), period, bool(reverse), tail)
        scene = _Scene(mode, "MANUAL", effect.first, effect, scale)
        expected = [v for pixel in _render(scene, elapsed) for v in pixel]
        delta = max(abs(x-y) for x, y in zip(actual[n*72:(n+1)*72], expected))
        assert delta < 0.005, (cases[n], delta)
        error = max(error, delta)
    unchanged = 0
    if args.baseline:
        old = compile_run(args.baseline.read_text(encoding="utf-8"), "rgb_baseline")
        for n, case in enumerate(cases):
            if case[0] != args.changed_mode:
                assert actual[n*72:(n+1)*72] == old[n*72:(n+1)*72], case
                unchanged += 1
    max_step = 0.0
    for start in dense.values():
        assert actual[start*72:(start+1)*72] == actual[(start+1400)*72:(start+1401)*72]
        for n in range(start+1, start+1401):
            max_step = max(max_step, max(abs(actual[n*72+i]-actual[(n-1)*72+i]) for i in range(72)))
    assert max_step < 5.0, max_step
    rainbow_step = 0.0
    for start in rainbow_dense.values():
        assert actual[start*72:(start+1)*72] == actual[(start+6000)*72:(start+6001)*72]
        for n in range(start+1, start+6001):
            rainbow_step = max(rainbow_step, max(abs(actual[n*72+i]-actual[(n-1)*72+i]) for i in range(72)))
    assert rainbow_step < 1.5, rainbow_step
    assert before == hashes(), "Concurrent production edit; rerun against final sources"
    result = dict(frames=len(cases), channels=len(actual), max_host_difference=error,
                  other_modes_bit_identical_frames=unchanged, unchanged_except_mode=args.changed_mode,
                  spin_max_step_per_ms=max_step, rainbow_max_step_per_ms=rainbow_step,
                  input_sha256=before, hardware=False, arm_timing=False)
    (out / "result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
