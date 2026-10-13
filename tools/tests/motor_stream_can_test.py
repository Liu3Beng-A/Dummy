"""Exercise production can.c queue/ownership/filter functions with a bounded HAL.
No device access. Do not replace CAN_QueueStreamReply with a reliable fake.
"""
from pathlib import Path
import argparse, subprocess, os, re, shutil, tempfile
p=argparse.ArgumentParser();p.add_argument('--build-dir');args=p.parse_args()
here=Path(__file__).resolve().parent
repo=here.parents[1]
w=Path(args.build_dir).resolve() if args.build_dir else Path(tempfile.mkdtemp(prefix='motor-stream-can-'))
w.mkdir(parents=True,exist_ok=True)
source=(repo/'firmware_new/dummy-motor-fw/Core/Src/can.c').read_text(encoding='utf-8')
header=(repo/'firmware_new/dummy-motor-fw/Core/Inc/can.h').read_text(encoding='utf-8')
# Integration guards: CAN init precedes DIP/EEPROM selection at boot, and 0x11
# can change the node at runtime. Both must update the hardware acceptance bank.
main=(repo/'firmware_new/dummy-motor-fw/UserApp/main.cpp').read_text(encoding='utf-8')
init=main[:main.index('/*---------------- Init Motor')]
assert init.index('CAN_UpdateFilters();') > init.rindex('boardConfig.canNodeId =')
runtime=(repo/'firmware_new/dummy-motor-fw/UserApp/protocols/interface_can.cpp').read_text(encoding='utf-8')
setter=runtime[runtime.index('case 0x11:'):runtime.index('case 0x12:')]
assert setter.index('CAN_UpdateFilters();') > setter.index('boardConfig.canNodeId =')

def function(name):
    start=source.index('void '+name+'('); a=source.index('{',start); depth=1;b=a+1
    while depth:
        depth+=(source[b]=='{')-(source[b]=='}');b+=1
    return source[start:b]
enum=header[header.index('enum MotorCanDiagnosticIndex'):header.index('};',header.index('enum MotorCanDiagnosticIndex'))+2]
common=source[source.index('CAN_TxHeaderTypeDef TxHeader;'):source.index('/* USER CODE END 0 */')]
body=source[source.index('// Caller owns PRIMASK.'):source.index('void HAL_CAN_RxFifo0MsgPendingCallback')]
text='''#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
struct CAN_TxHeaderTypeDef {uint32_t StdId,ExtId,IDE,RTR,DLC,TransmitGlobalTime;};
struct CAN_RxHeaderTypeDef {uint32_t StdId;};
struct CAN_HandleTypeDef {uint32_t ErrorCode;};
struct CAN_FilterTypeDef {uint32_t FilterBank,FilterMode,FilterScale,FilterFIFOAssignment,FilterActivation,FilterMaskIdLow,SlaveStartFilterBank,FilterIdHigh,FilterMaskIdHigh;};
constexpr uint32_t CAN_FILTERMODE_IDMASK=0,CAN_FILTERSCALE_32BIT=1,CAN_RX_FIFO0=0,ENABLE=1;
constexpr uint32_t HAL_CAN_ERROR_TX_ALST0=1,HAL_CAN_ERROR_TX_TERR0=2,HAL_CAN_ERROR_TX_ALST1=4,HAL_CAN_ERROR_TX_TERR1=8,HAL_CAN_ERROR_TX_ALST2=16,HAL_CAN_ERROR_TX_TERR2=32;
constexpr uint32_t HAL_CAN_ERROR_RX_FOV0=512,HAL_CAN_ERROR_RX_FOV1=1024,HAL_OK=0;
CAN_HandleTypeDef hcan{};struct {uint32_t canNodeId=1;}boardConfig;
struct {uint32_t CYCCNT;}dwt;
#define DWT (&dwt)
uint32_t tick,irq,hardwareBusy,aborts;uint8_t hardwareData[8];uint32_t hardwareId;
uint32_t __get_PRIMASK(){return irq;}void __disable_irq(){irq=1;}void __set_PRIMASK(uint32_t v){irq=v;}
uint32_t HAL_GetTick(){return tick;}
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef*){return hardwareBusy?2:3;}
uint32_t HAL_CAN_AddTxMessage(CAN_HandleTypeDef*,CAN_TxHeaderTypeDef*h,uint8_t*d,uint32_t*m){if(hardwareBusy)abort();hardwareBusy=1;hardwareId=h->StdId;memcpy(hardwareData,d,8);*m=1;return HAL_OK;}
void HAL_CAN_AbortTxRequest(CAN_HandleTypeDef*,uint32_t){++aborts;}
std::vector<CAN_FilterTypeDef> filters;
uint32_t HAL_CAN_ConfigFilter(CAN_HandleTypeDef*,CAN_FilterTypeDef*f){filters.push_back(*f);return HAL_OK;}
void Error_Handler(){abort();}
bool CAN_TrySend(CAN_TxHeaderTypeDef*,const uint8_t*);
'''+enum+'\n'+common+'\n'+function('CAN_UpdateFilters')+'\n'+body
if os.name=='nt': text=text.replace('__attribute__((noinline))','__declspec(noinline)')
text+='''
static unsigned checks;
#define CHECK(v) do{++checks;if(!(v)){printf("FAIL line %u %s\\n",__LINE__,#v);exit(1);}}while(0)
void Reset(){tick=0;irq=0;hardwareBusy=0;aborts=0;motorCanRead=motorCanWrite=motorCanProtectionRead=motorCanProtectionWrite=0;motorCanTransmitMailbox=motorCanProtectionMailbox=0;motorCanProtectionAborting=false;motorCanStreamEpoch=0;motorCanStreamFailed=false;motorCanStreamRetries=motorCanStreamDrops=0;for(auto& word:motorCanDiagnostic)word=0;}
bool Queue(uint8_t state,uint16_t seq=1,uint8_t cmd=0x47){CAN_TxHeaderTypeDef h{};h.StdId=128|cmd;h.DLC=8;uint8_t b[8]={7,0,uint8_t(seq),uint8_t(seq>>8),state,7,0,2};return CAN_QueueStreamReply(&h,b);}
void Complete(bool success,bool busError=false){hardwareBusy=0;if(success)HAL_CAN_TxMailbox0CompleteCallback(&hcan);else{hcan.ErrorCode=busError?HAL_CAN_ERROR_TX_TERR0:HAL_CAN_ERROR_TX_ALST0;HAL_CAN_ErrorCallback(&hcan);}}
int main(){
 Reset();CHECK(Queue(2));CHECK(Queue(3));CHECK(Queue(2,2));CHECK(motorCanProtectionWrite==3);
 for(uint8_t expected:{2,3,2}){CAN_PollTransmit();CHECK(hardwareBusy && hardwareData[4]==expected);Complete(true);}CHECK(motorCanProtectionRead==motorCanProtectionWrite);
 Reset();Queue(4);CAN_PollTransmit();Complete(false,true);CHECK(motorCanProtectionRead==0 && motorCanStreamRetries==1);CAN_PollTransmit();CHECK(!hardwareBusy);tick=5;CAN_PollTransmit();CHECK(hardwareBusy && hardwareData[4]==4);Complete(true);CHECK(motorCanProtectionRead==motorCanProtectionWrite);
 Reset();Queue(4);CAN_PollTransmit();hardwareBusy=0;CHECK(!CAN_TrySend(&motorCanProtectionQueue[0].frame.header,motorCanProtectionQueue[0].frame.data));CHECK(motorCanTransmitMailbox==1);HAL_CAN_TxMailbox0CompleteCallback(&hcan);CHECK(motorCanTransmitMailbox==0);
 Reset();Queue(2);Queue(3);CAN_PollTransmit();CAN_CancelStreamReplies();CHECK(aborts==1 && motorCanProtectionMailbox==1);CHECK(Queue(7));hardwareBusy=0;HAL_CAN_TxMailbox0AbortCallback(&hcan);CAN_PollTransmit();CHECK(hardwareBusy && hardwareData[4]==7 && !CAN_StreamReplyFault());Complete(true);
 Reset();Queue(5,2);Queue(5,3);CHECK(motorCanProtectionWrite==1);CAN_PollTransmit();CHECK(hardwareData[2]==3);Complete(true);
 Reset();for(uint16_t n=1;n<=8;++n)CHECK(Queue(n==1?5:2,n));CHECK(motorCanProtectionWrite==8);CHECK(Queue(5,9));CHECK(motorCanProtectionWrite==8);CHECK(!Queue(3,10));CHECK(CAN_StreamReplyFault());
 // Normal NART arbitration losses do not spend the physical error budget.
 // More than 16 contenders/retries is valid; total age still remains bounded.
 Reset();CHECK(Queue(4));
 for(unsigned n=0;n<32;++n){tick=n;CAN_PollTransmit();CHECK(hardwareBusy);Complete(false);CHECK(!CAN_StreamReplyFault() && motorCanProtectionQueue[0].attempts==0);}
 CAN_PollTransmit();CHECK(hardwareBusy);Complete(true);CHECK(motorCanStreamRetries==32 && motorCanStreamDrops==0);
 Reset();CHECK(Queue(4));CAN_PollTransmit();Complete(false);tick=41;CAN_PollTransmit();CHECK(CAN_StreamReplyFault() && motorCanStreamDrops==1);
 // Reserved slots and actual priority: new STOP must pass old retrying DIAG.
 Reset();Queue(0,1,0x4B);CAN_PollTransmit();Complete(false,true);CAN_TxHeaderTypeDef stop{};stop.StdId=128|0x0A;stop.DLC=8;uint8_t b[8]{};CAN_Send(&stop,b);CAN_PollTransmit();CHECK(hardwareBusy && (hardwareId&127)==0x0A);Complete(true);
 Reset();Queue(0,1,0x4B);CAN_PollTransmit();CAN_CancelStreamReplies();CHECK(aborts==1);hardwareBusy=0;HAL_CAN_TxMailbox0AbortCallback(&hcan);CHECK(motorCanProtectionRead==motorCanProtectionWrite);
 Reset();Queue(2);tick=41;CAN_PollTransmit();CHECK(!hardwareBusy && CAN_StreamReplyFault());CHECK(motorCanProtectionRead==motorCanProtectionWrite);
 // Exhaustive standard-ID filter membership, plus runtime node-ID changes.
 for(unsigned node:{0U,1U,7U,8U,15U}){
  boardConfig.canNodeId=node;filters.clear();CAN_UpdateFilters();CHECK(filters.size()==3);
  for(unsigned id=0;id<2048;++id){bool accepted=false;for(const auto& f:filters)accepted|=((id<<5)&f.FilterMaskIdHigh)==(f.FilterIdHigh&f.FilterMaskIdHigh);CHECK(accepted==((id>>7)==node || (id>>7)==0 || (id&127)==0x0B));}
  for(const auto& f:filters){CHECK(f.FilterMaskIdLow==6 && f.FilterFIFOAssignment==CAN_RX_FIFO0);for(unsigned low:{2U,4U,6U})CHECK((low&f.FilterMaskIdLow)!=0);}
 }
 CHECK(irq==0);printf("motor_actual_can_checks=%u queue_slots=%u pending_bytes=%zu\\n",checks,unsigned(MOTOR_CAN_PROTECTION_SLOTS),sizeof(MotorCanProtectionPending));
}
'''
(w/'actual_motor_can.cpp').write_text(text,encoding='utf-8')
if os.name=='nt':
 setup=''
 if not shutil.which('cl'):
  vswhere=Path(os.environ.get('ProgramFiles(x86)','C:/Program Files (x86)'))/'Microsoft Visual Studio/Installer/vswhere.exe'
  found=subprocess.check_output([str(vswhere),'-latest','-products','*','-requires','Microsoft.VisualStudio.Component.VC.Tools.x86.x64','-property','installationPath'],text=True).strip()
  setup=f'call "{Path(found)/"VC/Auxiliary/Build/vcvars64.bat"}" > nul\nif errorlevel 1 exit /b 1\n'
 cmd='@echo off\n'+setup+'cl /nologo /utf-8 /std:c++17 /O2 /EHsc actual_motor_can.cpp /Fe:actual_motor_can.exe\nif errorlevel 1 exit /b 1\nactual_motor_can.exe\n'
 (w/'run.cmd').write_text(cmd,encoding='utf-8')
 r=subprocess.run(['cmd.exe','/d','/c',str(w/'run.cmd')],cwd=w,capture_output=True)
else:
 subprocess.run(['c++','-std=c++17','-O2','actual_motor_can.cpp','-o','actual_motor_can'],cwd=w,check=True)
 r=subprocess.run([str(w/'actual_motor_can')],cwd=w,capture_output=True)
(w/'result.txt').write_bytes(r.stdout+r.stderr)
print((r.stdout+r.stderr).decode(errors='replace'));raise SystemExit(r.returncode)
