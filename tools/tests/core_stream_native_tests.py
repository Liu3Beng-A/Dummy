"""Build real portable core coordinator/parser tests without connecting hardware.

Run from a Visual Studio x64 developer shell, or with g++/clang++ on PATH:
  python tools/tests/core_stream_native_tests.py --output <scratch directory>
"""
from pathlib import Path
import argparse
import shutil
import subprocess
import tempfile

parser = argparse.ArgumentParser()
parser.add_argument('--output', type=Path, default=Path(tempfile.gettempdir())/'dummy-core-stream-tests')
args = parser.parse_args()
repo = Path(__file__).resolve().parents[2]
core = repo/'firmware_new/dummy-ref-core-fw'
out = args.output.resolve()
out.mkdir(parents=True, exist_ok=True)
compiler = shutil.which('cl') or shutil.which('g++') or shutil.which('clang++')
if not compiler:
    raise SystemExit('Run in a Visual Studio developer shell, or install a C++17 g++/clang++ compiler.')
msvc = Path(compiler).stem.lower() == 'cl'
production = (core/'Bsp/communication/ascii_processor.cpp').read_text(encoding='utf-8-sig')
# Stub only transport includes; compile the actual parser functions unchanged.
(out/'ascii_processor_under_test.inc').write_text('\n'.join(line for line in production.splitlines()
    if not line.lstrip().startswith('#include')), encoding='utf-8')
adapter=(core/'Robot/instances/core_stream.cpp').read_text(encoding='utf-8-sig')
(out/'core_stream_adapter_under_test.inc').write_text('\n'.join(line for line in adapter.splitlines() if not line.lstrip().startswith('#include')),encoding='utf-8')
start=adapter.index('    bool Validate(')
brace=adapter.index('{',start)
depth=1; end=brace+1
while depth:
    depth += (adapter[end]=='{') - (adapter[end]=='}'); end+=1
(out/'core_stream_geometry_under_test.inc').write_text(
    'static bool ProductionValidate(const Record& record,const Record* previous,uint8_t mask) '+adapter[brace:end],encoding='utf-8')
def function_text(source,signature):
    start=source.index(signature);brace=source.index('{',start);depth=1;end=brace+1
    while depth:
        depth+=(source[end]=='{')-(source[end]=='}');end+=1
    return source[start:end]
robot=(core/'Robot/instances/dummy_robot.cpp').read_text(encoding='utf-8-sig')
ascii=(core/'UserApp/protocols/ascii_protocol.cpp').read_text(encoding='utf-8-sig')
# Find the definition after the forward declaration (source remains unchanged).
motion=ascii[ascii.index('static bool HandleMotionCommand',ascii.index('static bool HandleStopCommand')):]
(out/'core_stream_restore_under_test.inc').write_text(function_text(robot,'bool DummyRobot::TrySetCommandMode')+'\n'+function_text(motion,'static bool HandleMotionCommand')+'\n'+function_text(ascii,'static bool HandleStopCommand'),encoding='utf-8')
# Reuse only the hardware-free state/transport scaffold; compile the actual
# adapter unchanged. Dedicated lifecycle cases cover its internal native owner.
prefix=(repo/'tools/tests/core_stream_adapter_test.cpp').read_text(encoding='utf-8').split('static StreamSink* usbStreamOutputPtr')[0]
prefix=prefix.replace('#include "../../firmware_new/', '#include "'+str(repo/'firmware_new').replace('\\','/')+'/')
prefix=prefix.replace('#include <vector>','#include <vector>\n#include <algorithm>')
# A deterministic reentry barrier inside the REAL validator verifies that the
# publication guard rechecks the token after validation outside IRQ masking.
prefix=prefix.replace('#include "core_stream_adapter_under_test.inc"', 'static void(*validationHook)()=nullptr;\nstatic double ValidationFabs(double value){if(validationHook){auto callback=validationHook;validationHook=nullptr;callback();}return std::fabs(value);}\n#define fabs ValidationFabs\n#include "core_stream_adapter_under_test.inc"\n#undef fabs')
(out/'core_native_lifecycle_test.cpp').write_text(prefix+'\n'+(repo/'tools/tests/core_native_lifecycle_test.cpp').read_text(encoding='utf-8'),encoding='utf-8')
tests = [('core_stream_engine', [core/'Robot/instances/core_stream_engine.cpp', repo/'tools/tests/core_stream_engine_test.cpp']),
         ('core_stream_parser', [repo/'tools/tests/core_stream_parser_test.cpp']),
         ('core_stream_geometry', [repo/'tools/tests/core_stream_geometry_test.cpp']),
         ('core_stream_adapter', [core/'Robot/instances/core_stream_engine.cpp',repo/'tools/tests/core_stream_adapter_test.cpp']),
         ('core_native_lifecycle',[core/'Robot/instances/core_stream_engine.cpp',out/'core_native_lifecycle_test.cpp'])]
for name, sources in tests:
    executable = out/(name + ('.exe' if msvc else ''))
    if msvc:
        command = [compiler, '/nologo', '/std:c++17', '/O2', '/W4', '/EHsc', '/I'+str(out), *map(str,sources), '/Fe:'+str(executable)]
    else:
        command = [compiler, '-std=c++17', '-O2', '-Wall', '-Wextra', '-I'+str(out), *map(str,sources), '-o', str(executable)]
    build = subprocess.run(command, cwd=out, capture_output=True)
    (out/(name+'-build.log')).write_bytes(build.stdout+build.stderr)
    if build.returncode:
        print((build.stdout+build.stderr).decode(errors='replace')); raise SystemExit(build.returncode)
    result = subprocess.run([str(executable)], cwd=out, capture_output=True)
    (out/(name+'-test.log')).write_bytes(result.stdout+result.stderr)
    print((result.stdout+result.stderr).decode(errors='replace').strip())
    if result.returncode: raise SystemExit(result.returncode)
