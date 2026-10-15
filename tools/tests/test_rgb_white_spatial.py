"""Exercise the production MCU wire quantizer without HAL/RTOS or hardware.

MSVC compiles the actual QuantizeFrame/Gain function bodies. Check average
error, white balance, the narrowly scoped gate, legacy GRB output, host parity,
and unchanged Phase/Gain. Legacy 0..12 render parity and independent automatic
13/legacy-8 parity are checked by test_rgb_motion_firmware.py --baseline.
This does not measure MCU timing or physical mixing.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import random
import re
import struct
import subprocess
import sys

from test_rgb_render_native import function


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--baseline', required=True, type=Path)
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[2]
    core = repo / 'firmware_new/dummy-ref-core-fw/Bsp/gpio'
    preview = repo / 'Dummy上位机/app/core/rgb_simulator.py'
    inputs = [core / 'rgb.cpp', core / 'rgb.hpp', preview, args.baseline]
    hashes = lambda: {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
    before = hashes()
    source = inputs[0].read_text(encoding='utf-8')
    baseline = args.baseline.read_text(encoding='utf-8')
    for signature in ('float RGB::Phase(',):
        assert function(source, signature) == function(baseline, signature), signature
    assert function(source, 'float Gain(') == function(baseline, 'float Gain(')
    # Compile the real SendFrame prefix too, so scene gating and production
    # wiring cannot silently diverge from this standalone quantizer test.
    prefix = function(source, 'void RGB::SendFrame(').split('    if (wireValid')[0]
    prefix += '    std::memcpy(lastWire, wire, sizeof(wire));\n}\n'
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    (out / 'rgb.hpp').write_bytes((core / 'rgb.hpp').read_bytes())
    (out / 'tim.h').write_text('// No HAL needed for wire quantization.\n')
    gamma = re.search(r'const uint8_t gammaPercent\[101\][^;]+;', source)[0]
    cpp = '''#include <cmath>
#include <cstdio>
#include <cstring>
#include <io.h>
#include <fcntl.h>
#define private public
#include "rgb.hpp"
#undef private
RGB::RGB(uint8_t) {}
''' + gamma + '\n' + function(source, 'float Gain(') + '\n' + function(source, 'void QuantizeFrame(') + '\n' + prefix + '''
int main() {
    _setmode(_fileno(stdout), _O_BINARY);
    RGB rgb; unsigned src, mode, r, g, b; float brightness;
    while (scanf("%u %u %u %u %u %f", &src, &mode, &r, &g, &b, &brightness)==6) {
        rgb.scene.source=src; rgb.scene.mode=mode;
        rgb.scene.solid={uint8_t(r),uint8_t(g),uint8_t(b)}; rgb.brightness=brightness;
        for (auto& pixel:rgb.frame) for (float& channel:pixel)
            if (scanf("%f", &channel)!=1) return 2;
        rgb.SendFrame(0);
        if (fwrite(rgb.lastWire, sizeof(rgb.lastWire), 1, stdout)!=1) return 3;
    }
    return ferror(stdin) ? 4 : 0;
}
'''
    (out / 'wire.cpp').write_text(cpp, encoding='utf-8')
    old_prefix = function(baseline, 'void RGB::SendFrame(').split('    if (wireValid')[0]
    old_prefix += '    std::memcpy(lastWire, wire, sizeof(wire));\n}\n'
    (out / 'wire_baseline.cpp').write_text(cpp.replace(prefix, old_prefix), encoding='utf-8')
    vswhere = Path(os.environ.get('ProgramFiles(x86)', 'C:/Program Files (x86)')) / 'Microsoft Visual Studio/Installer/vswhere.exe'
    vs = subprocess.check_output([str(vswhere), '-latest', '-products', '*', '-requires',
                                  'Microsoft.VisualStudio.Component.VC.Tools.x86.x64', '-property', 'installationPath'], text=True).strip()
    vcvars = Path(vs) / 'VC/Auxiliary/Build/vcvars64.bat'
    cmd = out / 'build.cmd'
    cmd.write_text('@echo off\ncall "' + str(vcvars) + '" > nul\nif errorlevel 1 exit /b 1\n'
                   + 'cl /nologo /utf-8 /std:c++17 /O2 /EHsc /fp:precise /I. wire.cpp /Fe:wire.exe\n'
                   + 'if errorlevel 1 exit /b 1\n'
                   + 'cl /nologo /utf-8 /std:c++17 /O2 /EHsc /fp:precise /I. wire_baseline.cpp /Fe:wire_baseline.exe\nexit /b %errorlevel%\n')
    compiled = subprocess.run(['cmd.exe', '/d', '/c', str(cmd)], cwd=out, capture_output=True)
    (out / 'compile.log').write_bytes(compiled.stdout + compiled.stderr)
    compiled.check_returncode()
    sys.path.insert(0, str(preview.parents[2]))
    from app.core.rgb_simulator import (_Effect, _Scene, STATE_NAMES, RgbSimulator,
                                        _gain, _quantize_frame, default_config)
    f32 = lambda x: struct.unpack('<f', struct.pack('<f', x))[0]
    cases = []
    white = (255, 255, 255)
    # All levels, including exact quarter fractions and 0/255 saturation.
    for n in range(1021):
        cases.append((1, 5, white, 1.0, ((n / 4,) * 3,) * 24, True))
    # Rounding residue after a completed color blend is not a colored effect.
    residue = ((1.25, f32(1.25 + 1e-5), f32(1.25 - 1e-5)),) * 24
    cases.append((1, 5, white, 1.0, residue, True))
    visible = ((1.25, f32(1.25 + .001), 1.25),) * 24
    cases.append((1, 5, white, 1.0, visible, False))
    rng = random.Random(2026101503)
    for source_id in range(15):  # Includes MANUAL.
        for mode in range(13):
            for color in (white, (255, 254, 255), (0, 80, 255)):
                frame = ((1.25,) * 3,) * 24
                eligible = source_id == 1 and mode == 5 and color == white
                cases.append((source_id, mode, color, 1.0, frame, eligible))
                # Unequal pixels/channels model transitions from any old scene.
                frame = tuple(tuple(f32(rng.uniform(0, 255)) for _ in range(3)) for _ in range(24))
                cases.append((source_id, mode, color, .5, frame, False))
    cycle_indices = {}
    for brightness in (0, 1, 5, 10, 20, 50, 100):
        cycle_indices[brightness] = []
        for ms in range(0, 3800, 20):
            a = .02 + .98 * (.5 - .5 * math.cos(2 * math.pi * ms / 3800))
            frame = ((f32(255 * .15 * a),) * 3,) * 24
            cycle_indices[brightness].append(len(cases))
            cases.append((1, 5, white, brightness / 100, frame, True))
    payload = ''.join(' '.join(str(x) for x in (s, m, *color, b,
                       *(channel for pixel in frame for channel in pixel))) + '\n'
                      for s, m, color, b, frame, eligible in cases).encode('ascii')
    ran = subprocess.run([str(out / 'wire.exe')], input=payload, capture_output=True, check=True)
    old = subprocess.run([str(out / 'wire_baseline.exe')], input=payload, capture_output=True, check=True)
    assert len(ran.stdout) == 72 * len(cases)
    assert len(old.stdout) == len(ran.stdout)
    exact = 0
    unchanged_frames = 0
    max_host_mean_delta = 0
    max_mean_error = 0
    outputs = []
    for i, (source_id, mode, color, b, frame, eligible) in enumerate(cases):
        wire = ran.stdout[72*i:72*(i+1)]
        actual = [tuple(wire[j+k] for k in (1, 0, 2)) for j in range(0, 72, 3)]
        outputs.append(actual)
        src = STATE_NAMES[source_id] if source_id < len(STATE_NAMES) else 'MANUAL'
        scene = _Scene(mode, src, color, _Effect(color, (0, 0, 0), 3800, False, 3), .15)
        expected = _quantize_frame(frame, _gain(b), scene)
        if actual == expected:
            exact += 1
        else:
            # Host uses double, MCU float; at an exact threshold a single LED
            # may differ by one code, never a full-ring step or a color shift.
            assert max(abs(a-e) for ap, ep in zip(actual, expected) for a, e in zip(ap, ep)) <= 1, i
            if eligible:
                assert sum(a != e for a, e in zip(actual, expected)) <= 1, i
        mean_delta = abs(sum(p[0] for p in actual) - sum(p[0] for p in expected)) / 24
        max_host_mean_delta = max(max_host_mean_delta, mean_delta)
        if eligible:
            assert all(p[0] == p[1] == p[2] for p in actual), i
            target = f32(frame[0][0] * f32(_gain(f32(b))))
            error = abs(sum(p[0] for p in actual) / 24 - target)
            max_mean_error = max(max_mean_error, error)
            assert error <= 1/48 + 2e-5, (i, error)
        else:
            assert wire == old.stdout[72*i:72*(i+1)], (i, 'changed unapproved scene')
            unchanged_frames += 1
    cycles = {}
    def longest_hold(values):
        best = run = 1
        for previous, value in zip((values * 2), (values * 2)[1:]):
            run = run + 1 if previous == value else 1
            best = max(best, run)
        return min(best, len(values)) * 20
    for b, indices in cycle_indices.items():
        means = [sum(p[0] for p in outputs[i]) for i in indices]
        old_means = [sum(old.stdout[72*i:72*(i+1):3]) for i in indices]
        config = default_config()
        config.update(brightness=b, transition_ms=0, fade_on_ms=0,
                      fade_off_ms=0, brightness_ms=0)
        sim = RgbSimulator()
        sim.reset(initial_brightness=b)
        full_host_means = [sum(p[0] for p in sim.frame(config, 'DISABLED', ms, source='DISABLED'))
                           for ms in range(0, 3800, 20)]
        assert max(abs(a-h) for a, h in zip(means, full_host_means)) <= 1
        cycles[b] = {'distinct_mean_levels': len(set(means)), 'min': min(means)/24,
                     'max': max(means)/24, 'longest_hold_ms': longest_hold(means),
                     'old_distinct_mean_levels': len(set(old_means)),
                     'old_longest_hold_ms': longest_hold(old_means)}
    assert cycles[0]['max'] == 0 and cycles[10]['distinct_mean_levels'] == 15
    assert before == hashes(), 'Concurrent source edit; rerun on final files'
    result = {'wire_frames': len(cases), 'host_exact_frames': exact,
              'legacy_wire_bit_identical_frames': unchanged_frames,
              'max_host_ring_mean_difference': max_host_mean_delta,
              'max_ring_mean_error': max_mean_error, 'cycles': cycles,
              'unchanged': ['Phase', 'Gain'],
              'render_parity_test': 'test_rgb_motion_firmware.py --baseline',
              'input_sha256': before, 'hardware': False, 'mcu_timing': False}
    (out / 'result.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
