#include "../../firmware_new/dummy-ref-core-fw/Robot/instances/core_stream_engine.h"
#include "../../firmware_new/dummy-ref-core-fw/Robot/instances/core_stream.h"
#include "../../firmware_new/motion_completion_protocol.h"
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <cstdarg>
#include <string>
#include <vector>
static unsigned checks=0,criticalNesting=0,softStops=0;
#define CHECK(x) do{++checks;if(!(x)){fprintf(stderr,"FAIL %d %s\n",__LINE__,#x);exit(1);}}while(0)
#define taskENTER_CRITICAL() (++criticalNesting)
#define taskEXIT_CRITICAL() (--criticalNesting)
constexpr float RAIL_MM_PER_CIRCLE=5.0f;
struct Motor { bool inverseDirection=false; int reduction=50; float angleLimitMin=-175,angleLimitMax=175,angle=0; };
struct Pose { float a[6]{}; };
class StreamSink;
struct DummyRobot {
 enum CommandMode {COMMAND_TARGET_POINT_SEQUENTIAL=1,COMMAND_DEFAULT=2,COMMAND_MOTOR_TUNING=4,COMMAND_STREAM_JOINT=7};
 static constexpr unsigned DEFAULT_COMMAND_MODE=2;
 enum StopState {NORMAL,STOPPING,STOPPED,STARTING,DISABLING,DISABLED};StopState stopState=NORMAL;
 CommandMode commandMode=COMMAND_STREAM_JOINT;bool locked=false,motionActive=false,startPending=false;
 bool calibration=false,parameter=false;
 struct Handler {bool busy=false;bool IsBusy(){return busy;}}commandHandler;
 struct Tuning {bool active=false;bool IsActive(){return active;}}tuningHelper;
 bool TrySetCommandMode(uint32_t mode);
 bool IsMoving(){return motionActive;}bool IsMotionBusy(){return motionActive;}unsigned LastIssuedMotionId(){return 0;}
 bool IsRGBSaveBusy(){return false;}bool RequestStart(){return false;}void RequestDisable(){}
 void PrintMotionStatus(StreamSink&){}bool HandleProtectionCommand(const char*,size_t,StreamSink&){return false;}
 Motor values[7]; Motor* motorJ[8]={nullptr,&values[0],&values[1],&values[2],&values[3],&values[4],&values[5],&values[6]};
 Pose initPose;float jointAccRuntime[8]={0,30,30,30,30,30,30,30};
 bool IsEnabled(){return true;}bool IsStopLocked(){return locked;}bool IsCalibrationBusy(){return calibration;}bool IsProtectionParameterBusy(){return parameter;}
 void RequestSoftStop(){CHECK(criticalNesting==0);++softStops;locked=true;stopState=STOPPING;CoreStreamCancel();}
 void MotorAccelerationCallback(unsigned node,float value,unsigned){jointAccRuntime[node]=value;}
} dummy;
struct CAN_TxHeaderTypeDef {uint32_t StdId=0,IDE=0,RTR=0,DLC=8;};
constexpr unsigned CAN_ID_STD=0,CAN_RTR_DATA=0;
static unsigned hcan1;
static unsigned*get_can_ctx(unsigned*p){return p;}
static bool canBusy=true;
struct Frame {unsigned node,command;uint8_t data[8];};
static std::vector<Frame> frames;
static bool TryCanSendStreamMessage(unsigned*,uint8_t*d,CAN_TxHeaderTypeDef*h,uint16_t){if(canBusy)return false;Frame f{h->StdId>>7,h->StdId&127,{}};memcpy(f.data,d,8);frames.push_back(f);return true;}
static void PollCanMotionTransmit(unsigned*){}
static void PollCanStreamTransmit(unsigned*){}
static uint32_t nowMs=1;
static uint32_t HAL_GetTick(){return nowMs;}
static uint32_t micros(){return nowMs*1000;}
namespace SoftStopProtocol {constexpr unsigned COMMAND=10;}
class StreamSink {public: enum {CHANNEL_TYPE_USB=1};unsigned channelType=CHANNEL_TYPE_USB;std::vector<std::string> lines;};
static void Respond(StreamSink&s,const char*fmt,...){CHECK(!criticalNesting);char line[256];va_list ap;va_start(ap,fmt);vsnprintf(line,sizeof(line),fmt,ap);va_end(ap);CHECK(strlen(line)<=62);s.lines.emplace_back(line);}
#include "core_stream_adapter_under_test.inc"
static StreamSink* usbStreamOutputPtr=nullptr;
static bool HandleRGBCommand(const char*,size_t,StreamSink&){return false;}
#include "core_stream_restore_under_test.inc"
static bool RunCommand(StreamSink&s,const char*c){return HandleCoreStreamCommand(c,strlen(c),s);}
static void FinishOpenTest(uint16_t token,uint8_t node) {
 ++nowMs;CoreStreamPoll(); // Emit the directed OPEN after CAPS.
 uint8_t reply[8]{};Write16(reply,token);reply[4]=StreamProtocol::STATE_OPEN;
 reply[5]=StreamProtocol::CAPACITY;reply[7]=StreamProtocol::VERSION;
 CoreStreamReceive(node,StreamProtocol::STATUS,reply);
 ++nowMs;CoreStreamPoll(); // Probe confirms OPEN.
 ++nowMs;CoreStreamPoll(); // Firmware owns automatic CLOSE.
 reply[4]=StreamProtocol::STATE_IDLE;CoreStreamReceive(node,StreamProtocol::STATUS,reply);
 ++nowMs;CoreStreamPoll();CHECK(!engine.Active());
}

int main(){using namespace StreamProtocol;StreamSink output;
 CHECK(RunCommand(output,"#STREAM_AXES"));
 CHECK(output.lines.back()=="STREAM_AXES v=1 hz=50 mask=127 targeted=1");
 CHECK(!engine.Active()&&frames.empty());

 CHECK(RunCommand(output,"#STREAM_DIAG 1 51"));CHECK(output.lines.back().find("range")!=std::string::npos);
 for(unsigned field=0;field<=DIAG_LAST;++field){char command[40];snprintf(command,sizeof(command),"#STREAM_DIAG 7 %u",field);const auto before=output.lines.size();frames.clear();canBusy=true;
  CHECK(RunCommand(output,command));CHECK(output.lines.size()==before);for(unsigned i=0;i<10;++i){++nowMs;CoreStreamPoll();}CHECK(frames.empty());CHECK(output.lines.size()==before);
  canBusy=false;++nowMs;CoreStreamPoll();CHECK(frames.size()==1);CHECK(frames[0].node==7&&frames[0].command==DIAG);const auto nonce=Read16(frames[0].data);
  nowMs+=20;CoreStreamPoll();CHECK(frames.size()==2);CHECK(Read16(frames[1].data)==nonce);
  uint8_t reply[8]{};Write16(reply,uint16_t(nonce+1));reply[2]=uint8_t(field);Write32(reply+4,UINT32_MAX);CoreStreamReceive(7,DIAG,reply);CoreStreamPrint();CHECK(output.lines.size()==before);
  Write16(reply,nonce);CoreStreamReceive(7,DIAG,reply);CoreStreamPrint();CHECK(output.lines.size()==before+1);
  CHECK(output.lines.back().find(field==0||field==7||(field>=26&&field<=31)||(field>=37&&field<=42)||field==47||field==50 ? "value=-1" : "value=4294967295")!=std::string::npos);
 }
 canBusy=true;CHECK(RunCommand(output,"#STREAM_DIAG 1 0"));nowMs+=1000;CoreStreamPoll();CoreStreamPrint();CHECK(output.lines.back().find("timeout")!=std::string::npos);
 CHECK(RunCommand(output,"!STREAM_OPEN 42 127"));CoreStreamTransportFault(42,DEADLINE,7,Q,19,30);CoreStreamPoll();CHECK(softStops==1);CHECK(engine.Failure().source==CoreStreamEngine::SOURCE_CAN);CoreStreamPrint();
 bool at=false,mask=false,can=false,time=false;for(const auto&line:output.lines){at|=line.find("STREAM_FAULT_AT")!=std::string::npos;mask|=line.find("STREAM_FAULT_MASK")!=std::string::npos;can|=line.find("STREAM_FAULT_CAN")!=std::string::npos;time|=line.find("STREAM_FAULT_T ")!=std::string::npos;}
 CHECK(at&&mask&&can&&time);CHECK(RunCommand(output,"#STREAM"));CHECK(!criticalNesting);
 // Actual ASCII stop gate and mode setter: queries/cleanup pass, motion lock stays.
 CHECK(CoreStreamAuxAllowed("#GET_CMDMODE 3",14));const char* query="#GET_CMDMODE 3";
 CHECK(HandleStopCommand(query,strlen(query),output));CHECK(output.lines.back().find("ok CMDMODE mode=7")!=std::string::npos);
 const char* restore="#CMDMODE 2";CHECK(HandleStopCommand(restore,strlen(restore),output));CHECK(output.lines.back()=="error CMDMODE busy");
 dummy.stopState=DummyRobot::STOPPED;CHECK(!dummy.TrySetCommandMode(2)); // session still owned
 CHECK(RunCommand(output,"!STREAM_CLOSE 42"));CoreStreamPoll();CHECK(!engine.Active());
 dummy.stopState=DummyRobot::STOPPING;CHECK(!dummy.TrySetCommandMode(2));dummy.stopState=DummyRobot::STOPPED;
 CHECK(!dummy.TrySetCommandMode(1));CHECK(!dummy.TrySetCommandMode(7));dummy.startPending=true;CHECK(!dummy.TrySetCommandMode(2));dummy.startPending=false;
 dummy.calibration=true;CHECK(!dummy.TrySetCommandMode(2));dummy.calibration=false;dummy.parameter=true;CHECK(!dummy.TrySetCommandMode(2));dummy.parameter=false;
 CHECK(HandleStopCommand(restore,strlen(restore),output));CHECK(output.lines.back()=="ok CMDMODE mode=2");CHECK(dummy.IsStopLocked());CHECK(dummy.stopState==DummyRobot::STOPPED);
 CHECK(!dummy.TrySetCommandMode(7));CHECK(RunCommand(output,"!STREAM_OPEN 43 127"));CHECK(!engine.Active());CHECK(dummy.IsStopLocked());
 // Diagnostic entry preserves the native mode and excludes moving commands.
 dummy.locked=false;dummy.stopState=DummyRobot::NORMAL;dummy.motionActive=true;
 CHECK(RunCommand(output,"!STREAM_TEST 55 4"));CHECK(output.lines.back().find("gate=MOVING")!=std::string::npos);
 dummy.motionActive=false;canBusy=false;frames.clear();
 CHECK(RunCommand(output,"!STREAM_TEST 55 8"));CHECK(!engine.Active());
 CHECK(RunCommand(output,"!STREAM_TEST 55 4"));CHECK(engine.IsOpenTest()&&engine.Active());
 CHECK(dummy.commandMode==DummyRobot::COMMAND_DEFAULT);
 for(const char* cmd:{"!START","!HOME",">0,0,0,0,0,0,0,5","#CMDMODE 7","#SETJACC 4 30","!HAND_O"})CHECK(!CoreStreamAuxAllowed(cmd,strlen(cmd)));
 CHECK(CoreStreamAuxAllowed("#GETJPOS",8));CHECK(CoreStreamAuxAllowed("!STOP",5));
 CHECK(RunCommand(output,"!STREAM_START 55"));CHECK(!engine.TestOpened());
 for(unsigned i=0;i<3100;++i){++nowMs;CoreStreamPoll();CoreStreamPrint();}
 CHECK(!engine.Active());CHECK(RunCommand(output,"#STREAM_TEST"));
 CHECK(output.lines.back().find("st=DONE open=0 closed=1 code=17")!=std::string::npos);
 CHECK(softStops==1);CHECK(dummy.commandMode==DummyRobot::COMMAND_DEFAULT);
 for(const auto&frame:frames)CHECK(frame.node==4&&frame.command==CAPS);
 // Feature evidence is session-scoped, comes from real CAPS, and survives CLOSE.
 dummy.locked=false;dummy.stopState=DummyRobot::NORMAL;
 CHECK(RunCommand(output,"!STREAM_TEST 56 4"));
 CHECK(engine.FeatureSession()==56&&engine.StableEntryMask()==0&&engine.CapabilityMask()==0);
 uint8_t cap[8]{};Write16(cap,56);cap[2]=VERSION;cap[3]=CAP_TIMED|CAP_OPEN_SNAPSHOT|CAP_STABLE_ENTRY;
 Write16(cap+4,SEGMENT_TICKS);cap[6]=CAPACITY;cap[7]=42;
 CoreStreamReceive(4,CAPS,cap);CHECK(engine.StableEntryMask()==8);CoreStreamPrint();
 CHECK(output.lines[output.lines.size()-2]=="STREAM_FEATURES s=56 steady=8 start=8");
 FinishOpenTest(56,4);
 CHECK(RunCommand(output,"#STREAM_TEST"));CHECK(output.lines[output.lines.size()-2]=="STREAM_FEATURES s=56 steady=8 start=8");
 CHECK(RunCommand(output,"#STREAM"));
 bool featureSeen=false;for(const auto&line:output.lines)featureSeen|=line=="STREAM_FEATURES s=56 steady=8 start=8";
 CHECK(featureSeen);
 CHECK(RunCommand(output,"!STREAM_TEST 57 1"));CHECK(engine.StableEntryMask()==0&&engine.CapabilityMask()==0);
 Write16(cap,56);CoreStreamReceive(1,CAPS,cap);CHECK(engine.StableEntryMask()==0);
 Write16(cap,57);cap[3]=CAP_TIMED|CAP_OPEN_SNAPSHOT;CoreStreamReceive(1,CAPS,cap);CoreStreamPrint();
 CHECK(output.lines[output.lines.size()-2]=="STREAM_FEATURES s=57 steady=0 start=0");
 FinishOpenTest(57,1);
 printf("core_stream_adapter: %u checks passed; actual adapter, DIAG retries/fields/log bounds/STOP critical scope\n",checks);
}


