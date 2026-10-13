"""Compose actual core + motor CAN queues and actual seven-axis coordinators.
Only registers, IRQ scheduling, CAN wire time and physical tracking are modeled.
"""
from pathlib import Path
from motor_entry_test_helpers import completion_source
import re, subprocess, argparse, tempfile, shutil, os
local=Path(__file__).resolve().parent
repo=local.parents[1]
args=argparse.ArgumentParser(description='Actual core/motor CAN queues with seven-axis offline replay. No hardware access.')
args.add_argument('--build-dir')
opt=args.parse_args()
out=Path(opt.build_dir).resolve() if opt.build_dir else Path(tempfile.mkdtemp(prefix='stream-link-'))
out.mkdir(parents=True,exist_ok=True)
core=repo/'firmware_new/dummy-ref-core-fw'
motor=repo/'firmware_new/dummy-motor-fw'
def read(p):return Path(p).read_text(encoding='utf-8-sig')
def includes(s):return '\n'.join(l for l in s.splitlines() if not l.lstrip().startswith(('#include','#pragma once')))
prefix=read(repo/'tools/tests/core_can_reliability_test.cpp').split('static unsigned checks=0;')[0]
prefix=prefix.replace('#include "interface_can_header_under_test.inc"',includes(read(core/'Bsp/communication/interface_can.hpp')))
prefix=prefix.replace('#include "interface_can_under_test.inc"',includes(read(core/'Bsp/communication/interface_can.cpp')))
prefix=includes(prefix)
prefix=prefix.replace('return active&&session==activeSession;','return ::engineAllows(session);')
prefix=prefix.replace('faults.push_back({node,command,sequence,age});active=false;', 'faults.push_back({node,command,sequence,age});::transportFault(node,command,sequence,age);')
undef='\n'.join('#undef '+m for m in re.findall(r'^#define\s+(\w+)',prefix,re.M))
code='''#include <cstdint>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <deque>
#include <algorithm>
#include <fstream>
#include <string>
#include "core_stream_engine.h"
#include "stream_runtime.h"
#include "motion_completion_protocol.h"
struct Frame {uint32_t id; uint8_t bytes[8]; unsigned owner,slot;};
static uint32_t simTick=0;
static CoreStreamEngine* engine=nullptr;
static bool engineAllows(uint16_t s){return engine&&engine->CanTransmit(s);}
static void transportFault(uint8_t n,uint8_t c,uint16_t q,uint32_t age){if(engine)engine->TransportFailure(engine->Session(),StreamProtocol::DEADLINE,n,c,q,age,simTick/20);}
extern "C" uint32_t _estack=0;
namespace corehw {
'''+prefix+'''
uint64_t order[3]{},ordinal=0;
void Reset(){can1Ctx=CAN_context{};can1Ctx.handle=&hcan1;can2Ctx=CAN_context{};can2Ctx.handle=&hcan2;bus1=CAN_TypeDef{};bus2=CAN_TypeDef{};dummy=Dummy{};nowMs=clockUs=abortMask=0;sem1.count=sem2.count=1;faults.clear();streamHead=streamCount=0;for(auto&p:streamPending)p={};for(auto&o:order)o=0;ordinal=0;}
void Complete(unsigned i,bool ok){order[i]=0;bus1.sTxMailBox[i].TIR&=~1U;bus1.TSR|=1U<<(26+i);FinishCanTransmit(&hcan1,uint8_t(i),ok);}
void Poll(){nowMs=simTick/20;PollCanMotionTransmit(&can1Ctx);PollCanStreamTransmit(&can1Ctx);auto pending=abortMask;abortMask=0;for(unsigned i=0;i<3;++i)if(pending&(1U<<i))Complete(i,false);}
bool Peek(Frame& f){unsigned chosen=3;for(unsigned i=0;i<3;++i)if(bus1.sTxMailBox[i].TIR&1U){bus1.TSR&=~(1U<<(26+i));if(!order[i])order[i]=++ordinal;if(chosen==3||order[i]<order[chosen])chosen=i;}if(chosen==3)return false;auto& mb=bus1.sTxMailBox[chosen];f={mb.TIR>>21,{},0,chosen};memcpy(f.bytes,&mb.TDLR,4);memcpy(f.bytes+4,&mb.TDHR,4);return true;}
bool Send(uint8_t node,uint8_t cmd,const uint8_t* bytes){nowMs=simTick/20;CAN_TxHeaderTypeDef h;h.StdId=(uint32_t(node)<<7)|cmd;return TryCanSendStreamMessage(&can1Ctx,const_cast<uint8_t*>(bytes),&h,StreamProtocol::Read16(bytes));}
void Feedback(unsigned n){CAN_TxHeaderTypeDef h;h.StdId=(n<<7)|0x23;uint8_t b[8]{};TryCanSendMessage(&can1Ctx,b,&h);}
}
'''+undef+'\n'

can=read(motor/'Core/Src/can.c'); hdr=read(motor/'Core/Inc/can.h')
enum=hdr[hdr.index('enum MotorCanDiagnosticIndex'):hdr.index('};',hdr.index('enum MotorCanDiagnosticIndex'))+2]
canbody=can[can.index('CAN_TxHeaderTypeDef TxHeader;'):can.index('/* USER CODE END 0 */')]
canbody+=can[can.index('// Caller owns PRIMASK.'):can.index('void HAL_CAN_RxFifo0MsgPendingCallback')]
canbody=canbody.replace('__attribute__((noinline))','__declspec(noinline)')
shim=includes(read(repo/'tools/tests/motor_stream_test_shim.h'))
shim=shim.replace('struct CAN_TxHeaderTypeDef {uint32_t StdId{},IDE{},RTR{},DLC{};};','struct CAN_TxHeaderTypeDef {uint32_t StdId{},ExtId{},IDE{},RTR{},DLC{},TransmitGlobalTime{};};')
production=completion_source(repo)+'\n'+includes(read(motor/'Ctrl/Motor/stream_controller.cpp'))
for node in range(1,8):
    code+=f'namespace node{node} {{\n'+shim+'''
std::vector<ReplyFrame> replies;BoardConfig boardConfig;uint32_t irqMask;
struct CAN_RxHeaderTypeDef {uint32_t StdId;};
struct CAN_HandleTypeDef {uint32_t ErrorCode;};CAN_HandleTypeDef hcan{};
constexpr uint32_t HAL_CAN_ERROR_TX_ALST0=1,HAL_CAN_ERROR_TX_TERR0=2,HAL_CAN_ERROR_TX_ALST1=4,HAL_CAN_ERROR_TX_TERR1=8,HAL_CAN_ERROR_TX_ALST2=16,HAL_CAN_ERROR_TX_TERR2=32;
constexpr uint32_t HAL_CAN_ERROR_RX_FOV0=512,HAL_CAN_ERROR_RX_FOV1=1024,HAL_OK=0;
uint32_t hardwareBusy=0,aborts=0,hardwareId=0;uint8_t hardwareData[8];
uint32_t HAL_GetTick(){return ::simTick/20;}
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef*){return hardwareBusy?2:3;}
uint32_t HAL_CAN_AddTxMessage(CAN_HandleTypeDef*,CAN_TxHeaderTypeDef*h,uint8_t*d,uint32_t*m){if(hardwareBusy)std::abort();hardwareBusy=1;hardwareId=h->StdId;memcpy(hardwareData,d,8);*m=1;return HAL_OK;}
void HAL_CAN_AbortTxRequest(CAN_HandleTypeDef*,uint32_t){++aborts;}
'''+enum+'\n'+canbody+'\n'+production+f'''
Motor device;Motor::Controller controller;unsigned stableTicks;
void Reset(int32_t q){{stream=MotorStream::Runtime{{}};packetRead=packetWrite=0;packetOverflow=false;generation=0;irqMask=0;memset(entrySnapshot,0,sizeof(entrySnapshot));calibrationBusy=false;controller=Motor::Controller{{}};controller.context=&device;controller.realPosition=controller.goalPosition=controller.softPosition=q;boardConfig.canNodeId={node};stableTicks=0;hardwareBusy=aborts=0;motorCanRead=motorCanWrite=motorCanProtectionRead=motorCanProtectionWrite=0;motorCanTransmitMailbox=motorCanProtectionMailbox=0;motorCanProtectionAborting=false;motorCanStreamEpoch=0;motorCanStreamFailed=false;motorCanStreamRetries=motorCanStreamDrops=0;controller.ResetPositionCompletion();for(unsigned i=0;i<1000;++i){{controller.TickStreamClock();controller.TickPositionCompletion();}}}}
void Receive(uint8_t cmd,const uint8_t*p){{if(cmd==0x23){{CAN_TxHeaderTypeDef h{{}};h.StdId=({node}<<7)|cmd;h.DLC=8;uint8_t b[8]{{}};CAN_Send(&h,b);}}else controller.ProcessStreamCan(cmd,p,8);}}
void Poll(){{dwt.CYCCNT=::simTick*3600;controller.PollStreamMain();CAN_PollTransmit();if(aborts){{aborts=0;hardwareBusy=0;HAL_CAN_TxMailbox0AbortCallback(&hcan);}}}}
void Tick(){{controller.TickStreamClock();controller.modeRunning=controller.requestMode;if(controller.modeRunning==Motor::MODE_STREAM_JOINT)controller.TickStream();controller.realPosition=controller.softPosition;controller.estVelocity=controller.softVelocity;controller.TickPositionCompletion();}}
bool Peek(Frame&f){{if(!hardwareBusy)return false;f={{hardwareId,{{}},{node},0}};memcpy(f.bytes,hardwareData,8);return true;}}
void Complete(bool ok){{hardwareBusy=0;if(ok)HAL_CAN_TxMailbox0CompleteCallback(&hcan);else{{hcan.ErrorCode=HAL_CAN_ERROR_TX_ALST0;HAL_CAN_ErrorCallback(&hcan);}}}}
void Stop(){{controller.RequestSoftStop(SoftStopProtocol::STOP,123);}}
uint32_t Anchor(){{return stream.armed?stream.anchor:0;}}uint16_t Completed(){{return stream.completed;}}
uint8_t State(){{return stream.state;}}uint8_t Reason(){{return stream.reason;}}int32_t Position(){{return controller.goalPosition;}}bool Locked(){{return controller.locked;}}
uint32_t Retries(){{return motorCanStreamRetries;}}uint32_t Drops(){{return motorCanStreamDrops;}}
}}
'''
    code+='\n'.join('#undef '+m for m in set(re.findall(r'^#define\s+(\w+)',shim+'\n'+canbody,re.M)))+'\n'

code+=read(local/'stream_link_test.cpp')
(out/'actual_stream_link.cpp').write_text(code,encoding='utf-8')
setup=''
if not shutil.which('cl'):
    vswhere=Path(os.environ.get('ProgramFiles(x86)','C:/Program Files (x86)'))/'Microsoft Visual Studio/Installer/vswhere.exe'
    found=subprocess.check_output([str(vswhere),'-latest','-products','*','-requires','Microsoft.VisualStudio.Component.VC.Tools.x86.x64','-property','installationPath'],text=True).strip()
    setup='call "'+str(Path(found)/'VC/Auxiliary/Build/vcvars64.bat')+'" > nul\nif errorlevel 1 exit /b 1'
cmd=f'''@echo off
{setup}
cl /nologo /utf-8 /std:c++20 /O2 /EHsc /I"{repo/'firmware_new'}" /I"{core/'Robot/instances'}" /I"{motor/'Ctrl/Motor'}" actual_stream_link.cpp "{core/'Robot/instances/core_stream_engine.cpp'}" "{motor/'Ctrl/Motor/stream_curve.cpp'}" "{motor/'Ctrl/Motor/stream_runtime.cpp'}" /Fe:actual_stream_link.exe > build.log 2>&1
if errorlevel 1 exit /b 1
actual_stream_link.exe "{repo/'tools/tests/fixtures/seven_axis_25mm_35pct.txt'}" > result.txt 2>&1
exit /b %errorlevel%
'''
(out/'run.cmd').write_text(cmd,encoding='utf-8')
r=subprocess.run(['cmd.exe','/d','/c',str(out/'run.cmd')],cwd=out)
for name in ('build.log','result.txt'):
    p=out/name
    if p.exists():print(p.read_text(encoding='utf-8',errors='replace')[-15000:])
raise SystemExit(r.returncode)
