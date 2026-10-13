"""Native production motor stream tests. No USB/CAN/STLink or hardware access.
Usage: python tools/tests/motor_stream_test.py [--build-dir path]
MSVC developer prompt or installed Visual Studio; Linux also supports c++.
"""
from pathlib import Path
from motor_entry_test_helpers import completion_source,feedback_source
import argparse,os,shutil,subprocess,tempfile,sys
p=argparse.ArgumentParser();p.add_argument('--build-dir');args=p.parse_args()
here=Path(__file__).resolve().parent;repo=here.parents[1]
root=repo/'firmware_new/dummy-motor-fw/Ctrl/Motor'
w=Path(args.build_dir).resolve() if args.build_dir else Path(tempfile.mkdtemp(prefix='motor-stream-test-'))
w.mkdir(parents=True,exist_ok=True)
for name in ['motor_stream_test.cpp','motor_stream_test_shim.h','motor_stream_channel_test.cpp','motor_stream_encoder_test.cpp','motor_entry_evidence_test.cpp']:
 shutil.copyfile(here/name,w/name)
source=(root/'stream_controller.cpp').read_text(encoding='utf-8')
source=source.replace('#include "motor.h"','#include "motor_stream_test_shim.h"').replace('#include "configurations.h"','').replace('#include <can.h>','')
(w/'stream_controller_under_test.cpp').write_text(source,encoding='utf-8')
(w/'motor_position_completion_under_test.inc').write_text(completion_source(repo),encoding='utf-8')
common=(here/'motor_stream_channel_test.cpp').read_text(encoding='utf-8').split('int main(){')[0]
(w/'motor_entry_test_common.inc').write_text(common,encoding='utf-8')
(w/'motor_entry_feedback_under_test.inc').write_text(feedback_source(repo),encoding='utf-8')
# Compile the production feedback-update and immediate output-disable branch.
# The wrapper supplies state only; it does not reimplement the 100-tick guard.
control=(root/'motor.cpp').read_text(encoding='utf-8')
start=control.index('    bool validSample =');end=control.index('    /********************************* Estimate Data',start)
guard_start=control.index('    if (controller->isStalled ||',end)
guard_end=control.index(' else if (controller->softBrake)',guard_start)
wrapper='void UpdateAndGuard() { int32_t deltaLapPosition;\n'+control[start:end]+control[guard_start:guard_end]+'\n}\n'
(w/'motor_feedback_under_test.h').write_text(wrapper,encoding='utf-8')

if os.name=='nt':
 setup=''
 if not shutil.which('cl'):
  vswhere=Path(os.environ.get('ProgramFiles(x86)','C:/Program Files (x86)'))/'Microsoft Visual Studio/Installer/vswhere.exe'
  found=subprocess.check_output([str(vswhere),'-latest','-products','*','-requires','Microsoft.VisualStudio.Component.VC.Tools.x86.x64','-property','installationPath'],text=True).strip()
  setup=f'call "{Path(found)/"VC/Auxiliary/Build/vcvars64.bat"}" > nul\nif errorlevel 1 exit /b 1\n'
 commands='@echo off\n'+setup
 for stem in ['motor_stream_test','motor_stream_channel_test','motor_stream_encoder_test','motor_entry_evidence_test']:
  encoder=root.parent/'Sensor/Encoder'
  sources=[encoder/'mt6816_base.cpp'] if stem.endswith('encoder_test') else [root/'stream_curve.cpp',root/'stream_runtime.cpp']
  source_args=' '.join('"'+str(s)+'"' for s in sources)
  commands+=f'cl /nologo /std:c++17 /O2 /EHsc /I"{repo/"firmware_new"}" /I"{root}" /I"{encoder}" /I"{w}" "{w/(stem+".cpp")}" {source_args} /Fe:"{w/(stem+".exe")}"\nif errorlevel 1 exit /b 1\n"{w/(stem+".exe")}"\nif errorlevel 1 exit /b 1\n'
 commands+='exit /b 0\n';(w/'build.cmd').write_text(commands,encoding='utf-8')
 result=subprocess.run(['cmd.exe','/d','/c',str(w/'build.cmd')],cwd=w,capture_output=True)
 (w/'result.log').write_bytes(result.stdout+result.stderr);print((result.stdout+result.stderr).decode(errors='replace'))
 if result.returncode==0:subprocess.run([sys.executable,str(here/'motor_stream_hold_test.py'),'--build-dir',str(w/'hold_oracle')],check=True)
 raise SystemExit(result.returncode)
else:
 for stem in ['motor_stream_test','motor_stream_channel_test','motor_stream_encoder_test','motor_entry_evidence_test']:
  encoder=root.parent/'Sensor/Encoder'
  sources=[encoder/'mt6816_base.cpp'] if stem.endswith('encoder_test') else [root/'stream_curve.cpp',root/'stream_runtime.cpp']
  subprocess.run(['c++','-std=c++17','-O2','-I'+str(repo/'firmware_new'),'-I'+str(root),'-I'+str(encoder),'-I'+str(w),str(w/(stem+'.cpp'))]+list(map(str,sources))+['-o',str(w/stem)],check=True,cwd=w)
  subprocess.run([str(w/stem)],check=True,cwd=w)

 if os.name!='nt':subprocess.run([sys.executable,str(here/'motor_stream_hold_test.py'),'--build-dir',str(w/'hold_oracle')],check=True)
