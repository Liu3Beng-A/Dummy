"""Compare actual d50efb1 baseline/current control output and limit evaluation.
Native compile only, no HAL/USB/CAN/STLink; results are not cycle measurements.
"""
from pathlib import Path
from motor_entry_test_helpers import function_text
import argparse,hashlib,json,os,re,shutil,subprocess,tempfile
p=argparse.ArgumentParser();p.add_argument('--build-dir');args=p.parse_args()
here=Path(__file__).resolve().parent;repo=here.parents[1]
motor=repo/'firmware_new/dummy-motor-fw/Ctrl/Motor'
w=Path(args.build_dir).resolve() if args.build_dir else Path(tempfile.mkdtemp(prefix='motor-current-limit-'))
w.mkdir(parents=True,exist_ok=True)
source=(motor/'motor.cpp').read_text(encoding='utf-8-sig')
stall=(motor/'stall_protection.cpp').read_text(encoding='utf-8-sig')
header=(motor/'motor.h').read_text(encoding='utf-8-sig')
defs=header[header.index('        typedef struct'):header.index('        explicit Controller')]
sig=('void Motor::Controller::CalcCurrentToOutput(int32_t current)',
     'void Motor::Controller::ApplyCurrentToOutput(int32_t current, int32_t limit)',
     'void Motor::Controller::CalcPidToOutput(int32_t _speed)',
     'void Motor::Controller::CalcDceToOutput(int32_t _location, int32_t _speed)')
current='\n\n'.join(function_text(source,s) for s in sig)+'\n'+function_text(stall,'int32_t Motor::Controller::EffectiveCurrentLimit() const')
old=(here/'fixtures/motor_current_limit_baseline.inc').read_text(encoding='utf-8-sig')
moving=function_text(stall,'bool MovingPhase(uint8_t phase)')
common='''#include <cstdint>
#include "protection_protocol.h"
using namespace ProtectionProtocol;
struct Controls {
'''+defs+'''PID_t pid{};DCE_t dce{};bool stallProtectSwitch=false;
};
struct Driver {unsigned calls=0;int32_t position=0,current=0;
 void SetFocCurrentVector(int32_t p,int32_t c){++calls;position=p;current=c;}
};
struct Context {struct {struct {int32_t ratedCurrent=2500;}motionParams;}config;Driver*driver=nullptr;};
int32_t Min(int32_t a,int32_t b){return a<b?a:b;}int32_t Max(int32_t a,int32_t b){return a>b?a:b;}
'''+moving+'\n'
for namespace,body in (('baseline',old),('candidate',current)):
 common+='namespace '+namespace+' {\nunsigned limitCalls=0;\n'
 common+='''class Motor {public:static constexpr int32_t SOFT_DIVIDE_NUM=256;
 class Controller {public:
 Context*context=nullptr;Controls*config=nullptr;
 int32_t focCurrent=0,focPosition=0,estPosition=0,estVelocity=0;
 uint8_t protectionPhase=0;bool calibrationTask=false,localStall=false;
 int32_t calibrationCurrent=0,retreatCurrent=0,holdingCurrent=0;
 int32_t EffectiveCurrentLimit()const;
 void CalcCurrentToOutput(int32_t);void CalcPidToOutput(int32_t);void CalcDceToOutput(int32_t,int32_t);
 void ApplyCurrentToOutput(int32_t,int32_t);
 };};
'''
 # Count entry into the real query only; no changes to its decision/arithmetic.
 body=body.replace('int32_t Motor::Controller::EffectiveCurrentLimit() const {','int32_t Motor::Controller::EffectiveCurrentLimit() const {\n    ++limitCalls;')
 assert '++limitCalls;' in body
 common+=body+'\n}\n'
(w/'motor_current_limit_common.inc').write_text(common,encoding='utf-8')
shutil.copyfile(here/'motor_current_limit_test.cpp',w/'motor_current_limit_test.cpp')
identity={str(f.relative_to(repo)):hashlib.sha256(f.read_bytes()).hexdigest() for f in (motor/'motor.cpp',motor/'motor.h',motor/'stall_protection.cpp',here/'motor_current_limit_test.cpp',here/'fixtures/motor_current_limit_baseline.inc')}
(w/'source-identity.json').write_text(json.dumps(identity,indent=2),encoding='utf-8')
if os.name=='nt':
 setup=''
 if not shutil.which('cl'):
  vswhere=Path(os.environ.get('ProgramFiles(x86)','C:/Program Files (x86)'))/'Microsoft Visual Studio/Installer/vswhere.exe'
  install=subprocess.check_output([str(vswhere),'-latest','-products','*','-requires','Microsoft.VisualStudio.Component.VC.Tools.x86.x64','-property','installationPath'],text=True).strip()
  setup=f'call "{Path(install)/"VC/Auxiliary/Build/vcvars64.bat"}" > nul\nif errorlevel 1 exit /b 1\n'
 cmd='@echo off\n'+setup+f'cl /nologo /utf-8 /std:c++17 /O2 /EHsc /I"{repo/"firmware_new"}" motor_current_limit_test.cpp /Fe:motor_current_limit_test.exe\nif errorlevel 1 exit /b 1\nmotor_current_limit_test.exe\nexit /b %errorlevel%\n'
 (w/'run.cmd').write_text(cmd,encoding='utf-8');r=subprocess.run(['cmd.exe','/d','/c',str(w/'run.cmd')],cwd=w,capture_output=True)
else:
 r=subprocess.run(['c++','-std=c++17','-O2','-I'+str(repo/'firmware_new'),'motor_current_limit_test.cpp','-o','motor_current_limit_test'],cwd=w,capture_output=True)
 if r.returncode==0:r=subprocess.run([str(w/'motor_current_limit_test')],cwd=w,capture_output=True)
(w/'result.log').write_bytes(r.stdout+r.stderr);print((r.stdout+r.stderr).decode(errors='replace'));raise SystemExit(r.returncode)
