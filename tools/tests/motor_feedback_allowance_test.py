"""Extract production allowance block and test old/new exact math and gates.
No USB/CAN/STLink or firmware builds; compile only a native test in --build-dir.
"""
from pathlib import Path
import argparse,hashlib,json,os,shutil,subprocess,tempfile
p=argparse.ArgumentParser();p.add_argument('--build-dir');args=p.parse_args()
here=Path(__file__).resolve().parent;repo=here.parents[1]
source_path=repo/'firmware_new/dummy-motor-fw/Ctrl/Motor/motor.cpp'
source=source_path.read_text(encoding='utf-8-sig')
start=source.index('        int64_t allowed;')
end=source.index('        // A parity-valid discontinuity',start)
body=source[start:end]
assert 'elapsedTicks <= 20001U' in body and 'motionPlanner.CONTROL_FREQUENCY == 20000' in body
assert '(controller->transportMissingTicks + 1)' not in body
guard_end=source.index('        controller->transportMissingTicks = 0;',end)
assert 'candidateDelta > allowed || candidateDelta < -allowed' in source[end:guard_end]
w=Path(args.build_dir).resolve() if args.build_dir else Path(tempfile.mkdtemp(prefix='motor-allowance-'))
w.mkdir(parents=True,exist_ok=True)
generated='''#include <cstdint>
int64_t actualAllowed(int32_t velocity,uint32_t missing,int32_t frequency=20000) {
 struct {uint32_t transportMissingTicks;}state{missing};auto*controller=&state;
 struct {int32_t velocityLimit;}boardConfig{velocity};
 struct {int32_t CONTROL_FREQUENCY;}motionPlanner{frequency};
'''+body+'\n return allowed;\n}\n'
(w/'motor_feedback_allowance_under_test.inc').write_text(generated,encoding='utf-8')
shutil.copyfile(here/'motor_feedback_allowance_test.cpp',w/'motor_feedback_allowance_test.cpp')
identity={str(f.relative_to(repo)):hashlib.sha256(f.read_bytes()).hexdigest() for f in (source_path,here/'motor_feedback_allowance_test.cpp',here/'motor_feedback_allowance_test.py')}
(w/'source-identity.json').write_text(json.dumps(identity,indent=2),encoding='utf-8')
if os.name=='nt':
 setup=''
 if not shutil.which('cl'):
  vswhere=Path(os.environ.get('ProgramFiles(x86)','C:/Program Files (x86)'))/'Microsoft Visual Studio/Installer/vswhere.exe'
  install=subprocess.check_output([str(vswhere),'-latest','-products','*','-requires','Microsoft.VisualStudio.Component.VC.Tools.x86.x64','-property','installationPath'],text=True).strip()
  setup=f'call "{Path(install)/"VC/Auxiliary/Build/vcvars64.bat"}" > nul\nif errorlevel 1 exit /b 1\n'
 cmd='@echo off\n'+setup+'cl /nologo /utf-8 /std:c++17 /O2 /EHsc motor_feedback_allowance_test.cpp /Fe:motor_feedback_allowance_test.exe\nif errorlevel 1 exit /b 1\nmotor_feedback_allowance_test.exe\nexit /b %errorlevel%\n'
 (w/'run.cmd').write_text(cmd,encoding='utf-8');r=subprocess.run(['cmd.exe','/d','/c',str(w/'run.cmd')],cwd=w,capture_output=True)
else:
 r=subprocess.run(['c++','-std=c++17','-O2','motor_feedback_allowance_test.cpp','-o','motor_feedback_allowance_test'],cwd=w,capture_output=True)
 if r.returncode==0:r=subprocess.run([str(w/'motor_feedback_allowance_test')],cwd=w,capture_output=True)
(w/'result.log').write_bytes(r.stdout+r.stderr);print((r.stdout+r.stderr).decode(errors='replace'));raise SystemExit(r.returncode)
