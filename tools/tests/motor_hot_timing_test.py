"""Native tests of actual completion/classification/timing code; no devices.
Extract the production TIM4 callback and verify its recorder scheduling with
deterministic DWT values. This is behavior evidence, never ARM cycle evidence.
"""
from pathlib import Path
from motor_entry_test_helpers import completion_source,function_text
import argparse,hashlib,json,os,shutil,subprocess,tempfile

p=argparse.ArgumentParser();p.add_argument('--build-dir');args=p.parse_args()
here=Path(__file__).resolve().parent;repo=here.parents[1]
motor=repo/'firmware_new/dummy-motor-fw/Ctrl/Motor'
w=Path(args.build_dir).resolve() if args.build_dir else Path(tempfile.mkdtemp(prefix='motor-hot-timing-'))
w.mkdir(parents=True,exist_ok=True)
for name in ('motor_stream_test_shim.h','motor_hot_timing_test.cpp'):shutil.copyfile(here/name,w/name)
source=(motor/'stream_controller.cpp').read_text(encoding='utf-8-sig')
source=source.replace('#include "motor.h"','#include "motor_stream_test_shim.h"').replace('#include "configurations.h"','').replace('#include <can.h>','')
(w/'stream_controller_under_test.cpp').write_text(source,encoding='utf-8')
(w/'motor_position_completion_under_test.inc').write_text(completion_source(repo),encoding='utf-8')
common=(here/'motor_stream_channel_test.cpp').read_text(encoding='utf-8-sig').split('int main(){')[0]
(w/'motor_entry_test_common.inc').write_text(common,encoding='utf-8')
baseline=(here/'fixtures/motor_completion_reset_baseline.inc').read_text(encoding='utf-8-sig')
baseline='struct BaselineCompletionController:Motor::Controller {void ResetPositionCompletion(bool);};\n'+baseline.replace('Motor::Controller::ResetPositionCompletion','BaselineCompletionController::ResetPositionCompletion')
(w/'motor_completion_reset_baseline.inc').write_text(baseline,encoding='utf-8')

main=(repo/'firmware_new/dummy-motor-fw/UserApp/main.cpp').read_text(encoding='utf-8-sig')
callback=function_text(main,'extern "C" void Tim4Callback20kHz()')
assert '++motorCanDiagnostic[MC_CONTROL_LATE]; ++motorControlTiming[phase+3];' in callback
assert '(++timingDivider & 63U)==0' in callback
callback='''#include <cstdint>
#include <cstdio>
#include <cstdlib>
enum {MC_CONTROL_GAP,MC_CONTROL_MAX,MC_CONTROL_LATE};
volatile uint32_t motorCanDiagnostic[3]{},motorControlTiming[11]{},motorStreamTiming[11]{};
bool motorTimingSampling;uint32_t motorTimingCheckpointCycle;
struct {uint32_t CYCCNT=0;}dwt;
#define DWT (&dwt)
struct {uint32_t flag=0;}htim4;
#define TIM_IT_UPDATE 1
#define TIM_FLAG_UPDATE 1
#define __HAL_TIM_CLEAR_IT(timer,flag) ((timer)->flag=0)
#define __HAL_TIM_GET_FLAG(timer,flag) ((timer)->flag)
uint32_t SystemCoreClock=72000000;
class Motor {public:
 enum{MODE_STREAM_JOINT=9};
 struct Controller{int modeRunning=0,requestMode=0;uint8_t timingPhase=0;unsigned clocks=0;
  uint8_t StreamTimingPhase(){return timingPhase;}void TickStreamClock(){++clocks;}
 }ctrl;Controller*controller=&ctrl;
 uint32_t cost=100;unsigned sampled=0;
 void Tick20kHz(){if(motorTimingSampling)++sampled;dwt.CYCCNT+=cost;}
}motor;
struct {bool isTriggered=false;void Tick20kHz(){dwt.CYCCNT+=200;}}encoderCalibrator;
'''+callback+'''
static unsigned checks;
#define CHECK(v)do{++checks;if(!(v)){printf("FAIL line%u %s\\n",__LINE__,#v);exit(1);}}while(0)
void tick(uint8_t phase,uint32_t cost=100){motor.ctrl.timingPhase=phase;motor.cost=cost;dwt.CYCCNT+=4000;Tim4Callback20kHz();}
int main(){
 for(unsigned n=0;n<100;++n)tick(0);CHECK(!motor.sampled && !motorTimingSampling);
 for(unsigned n=0;n<64;++n)tick(1);CHECK(motorStreamTiming[8]==1 && motor.sampled==1 && motorStreamTiming[5]==100);
 for(unsigned n=0;n<128;++n)tick(2);CHECK(motorStreamTiming[9]==2 && motor.sampled==3 && motorStreamTiming[6]==100);
 for(unsigned n=0;n<64;++n)tick(3);CHECK(motorStreamTiming[10]==1 && motor.sampled==4 && motorStreamTiming[7]==100);
 motor.ctrl.modeRunning=motor.ctrl.requestMode=Motor::MODE_STREAM_JOINT;
 tick(2,3600);CHECK(motorCanDiagnostic[MC_CONTROL_LATE]==1 && motorControlTiming[4]==1);
 CHECK(motorStreamTiming[6]==3600 && motorControlTiming[1]==3600);
 tick(2,3599);CHECK(motorCanDiagnostic[MC_CONTROL_LATE]==1);
 for(unsigned n=0;n<63;++n)tick(3);CHECK(motorStreamTiming[10]==2);
 encoderCalibrator.isTriggered=true;for(unsigned n=0;n<128;++n)tick(3);
 CHECK(!motorTimingSampling && motorStreamTiming[10]==2 && motorControlTiming[2]==200);
 CHECK(motor.ctrl.clocks==549);printf("motor_actual_callback_scheduling_checks=%u\\n",checks);
}
'''
# Macro arguments cannot also use the member name `flag`.
callback=callback.replace('__HAL_TIM_CLEAR_IT(timer,flag) ((timer)->flag=0)','__HAL_TIM_CLEAR_IT(timer,it) ((timer)->flag=0)').replace('__HAL_TIM_GET_FLAG(timer,flag) ((timer)->flag)','__HAL_TIM_GET_FLAG(timer,it) ((timer)->flag)')
(w/'actual_callback.cpp').write_text(callback,encoding='utf-8')
identity={str(f.relative_to(repo)):hashlib.sha256(f.read_bytes()).hexdigest() for f in (motor/'motor.cpp',motor/'motor.h',motor/'stream_controller.cpp',repo/'firmware_new/dummy-motor-fw/UserApp/main.cpp',here/'motor_hot_timing_test.cpp',here/'motor_hot_timing_test.py')}
(w/'source-identity.json').write_text(json.dumps(identity,indent=2),encoding='utf-8')
if os.name=='nt':
 setup=''
 if not shutil.which('cl'):
  vswhere=Path(os.environ.get('ProgramFiles(x86)','C:/Program Files (x86)'))/'Microsoft Visual Studio/Installer/vswhere.exe'
  install=subprocess.check_output([str(vswhere),'-latest','-products','*','-requires','Microsoft.VisualStudio.Component.VC.Tools.x86.x64','-property','installationPath'],text=True).strip()
  setup=f'call "{Path(install)/"VC/Auxiliary/Build/vcvars64.bat"}" > nul\nif errorlevel 1 exit /b 1\n'
 cmd='@echo off\n'+setup
 cmd+=f'cl /nologo /utf-8 /std:c++17 /O2 /EHsc /I"{repo/"firmware_new"}" /I"{motor}" /I"{w}" motor_hot_timing_test.cpp "{motor/"stream_curve.cpp"}" "{motor/"stream_runtime.cpp"}" /Fe:motor_hot_timing_test.exe\nif errorlevel 1 exit /b 1\nmotor_hot_timing_test.exe\nif errorlevel 1 exit /b 1\n'
 cmd+='cl /nologo /std:c++17 /O2 /EHsc actual_callback.cpp /Fe:actual_callback.exe\nif errorlevel 1 exit /b 1\nactual_callback.exe\nexit /b %errorlevel%\n'
 (w/'run.cmd').write_text(cmd,encoding='utf-8');result=subprocess.run(['cmd.exe','/d','/c',str(w/'run.cmd')],cwd=w,capture_output=True)
else:
 result=subprocess.run(['c++','-std=c++17','-O2','-I'+str(repo/'firmware_new'),'-I'+str(motor),'-I'+str(w),'motor_hot_timing_test.cpp',str(motor/'stream_curve.cpp'),str(motor/'stream_runtime.cpp'),'-o','motor_hot_timing_test'],cwd=w,capture_output=True)
 if result.returncode==0:result=subprocess.run([str(w/'motor_hot_timing_test')],cwd=w,capture_output=True)
 if result.returncode==0:
  result=subprocess.run(['c++','-std=c++17','-O2','actual_callback.cpp','-o','actual_callback'],cwd=w,capture_output=True)
  if result.returncode==0:result=subprocess.run([str(w/'actual_callback')],cwd=w,capture_output=True)
(w/'result.log').write_bytes(result.stdout+result.stderr)
print((result.stdout+result.stderr).decode(errors='replace'));raise SystemExit(result.returncode)
