#include "core_stream.h"
#include "cartesian_planner.h"
#include "motion_values.h"
#include "motion_completion_protocol.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdarg>
#include <string>
#include <vector>
#include <deque>
#include <algorithm>
static unsigned checks=0,criticalNesting=0,softStops=0,ordinaryTargets=0,taggedQueries=0;
#define CHECK(x) do{++checks;if(!(x)){fprintf(stderr,"FAIL %d %s\n",__LINE__,#x);exit(1);}}while(0)
void EnterCritical();
#define taskENTER_CRITICAL() EnterCritical()
#define taskEXIT_CRITICAL() (--criticalNesting)
constexpr float RAIL_MM_PER_CIRCLE=5.0f;
using osThreadId_t=void*;static int taskMarker;osThreadId_t ControlLoopUpdateHandle=&taskMarker;
static bool onCommandTask=true;
static osThreadId_t osThreadGetId(){return onCommandTask?ControlLoopUpdateHandle:nullptr;}
static uint32_t nowMs=1;static uint32_t HAL_GetTick(){return nowMs;}static uint32_t micros(){return nowMs*1000;}
void osDelay(unsigned duration);
struct FakeQueue{unsigned size,capacity;std::deque<std::vector<unsigned char>> data;};
using osMessageQueueId_t=FakeQueue*;constexpr unsigned osOK=0;
using osStatus_t=unsigned;
static unsigned osMessageQueuePut(FakeQueue* q,const void* data,unsigned,unsigned){if(q->data.size()==q->capacity)return 1;auto p=static_cast<const unsigned char*>(data);q->data.emplace_back(p,p+q->size);return osOK;}
static unsigned osMessageQueueGet(FakeQueue* q,void* data,void*,unsigned){if(q->data.empty())return 1;memcpy(data,q->data.front().data(),q->size);q->data.pop_front();return osOK;}
static unsigned osMessageQueueGetSpace(FakeQueue* q){return q->capacity-unsigned(q->data.size());}
class StreamSink{public:enum {CHANNEL_TYPE_USB=1};unsigned channelType=1;std::vector<std::string> lines;};
static StreamSink output;static StreamSink* usbStreamOutputPtr=&output;static StreamSink* uart4StreamOutputPtr=nullptr;
static bool precheckSeen=false,stopAtPublish=false,loseClose=false,denyGoal=false,stopOnClose=false;
static bool attemptCloseInt=false,attemptHandoffInt=false,attemptSettlingInt=false;
static unsigned intRejections=0,completionCriticalSeen=0;
static bool stopInSettling=false,stopAtCompleteBoundary=false,inDelay=false;
static void TryIntDuringNative();
static void(*executionHook)()=nullptr;
static uint32_t settlingAt=0;
static void Respond(StreamSink&s,const char* fmt,...){CHECK(!criticalNesting);char line[256];va_list ap;va_start(ap,fmt);vsnprintf(line,sizeof(line),fmt,ap);va_end(ap);s.lines.emplace_back(line);if(s.lines.back().find("stage=PRECHECK")!=std::string::npos)precheckSeen=true;if(s.lines.back().find("stage=SETTLING")!=std::string::npos)settlingAt=nowMs;}
struct MotorProbe{
 struct {struct {int32_t encoderHomeOffset=0;}motionParams;} config;
 struct Context{decltype(config) config;};Context storage{};Context* context=&storage;
 static constexpr uint8_t MODE_COMMAND_POSITION=1,STATE_FINISH=2;
 uint8_t modeRunning=1,requestMode=1,state=2;
 int32_t realPosition=0,goalPosition=0,softPosition=0,softVelocity=0,positionCompletionTarget=0;
 bool feedback=true,positionSettled=true,overloadFlag=false,goalDisable=false,softDisable=false,goalBrake=false,softBrake=false,isStalled=false;
 bool IsFeedbackValid()const{return feedback;}bool IsSoftStopLocked()const{return false;}
 uint8_t Status(float expectedMotorR)const;
};
static struct {uint8_t canNodeId=1;} boardConfig;
#include "motor_completion.inc"
struct Motor{bool inverseDirection=false;int reduction=50;float angleLimitMin=-175,angleLimitMax=175,angle=0;unsigned node=0;MotorProbe motor;
 bool QueryPositionCompletion(float target,uint32_t nonce,uint32_t epoch,uint32_t generation);
 void UpdateAngle(){}
 bool SendMotionTarget(float target,float,uint32_t,uint32_t){++ordinaryTargets;motor.goalPosition=motor.softPosition=motor.positionCompletionTarget=int32_t(target*51200);motor.realPosition=motor.goalPosition;return true;}
};
static unsigned hcan1;static unsigned*get_can_ctx(unsigned*p){return p;}
static void PollCanMotionTransmit(unsigned*){}static void PollCanStreamTransmit(unsigned*){}
struct CAN_TxHeaderTypeDef{uint32_t StdId=0,IDE=0,RTR=0,DLC=8;};constexpr unsigned CAN_ID_STD=0,CAN_RTR_DATA=0;
struct Frame{unsigned node,command;uint8_t data[8];};static std::vector<Frame> frames;static size_t frameCursor=0;
static bool TryCanSendStreamMessage(unsigned*,uint8_t*d,CAN_TxHeaderTypeDef*h,uint16_t){Frame f{h->StdId>>7,h->StdId&127,{}};memcpy(f.data,d,8);frames.push_back(f);return true;}
namespace SoftStopProtocol {constexpr unsigned COMMAND=10;}
struct DummyRobot{
 enum CommandMode{COMMAND_TARGET_POINT_SEQUENTIAL=1,COMMAND_TARGET_POINT_INTERRUPTABLE=2,COMMAND_DEFAULT=2,COMMAND_CONTINUES_TRAJECTORY=3,COMMAND_MOTOR_TUNING=4,COMMAND_STREAM_JOINT=7};
 enum MotionPhase{MOTION_IDLE,MOTION_DISPATCHING,MOTION_RUNNING,MOTION_COMPLETED,MOTION_CANCELLED,MOTION_FAILED};
 enum StopState{NORMAL,STOPPING,STOPPED,STARTING,DISABLING,DISABLED};
 static constexpr CommandMode DEFAULT_COMMAND_MODE=COMMAND_DEFAULT;
 CommandMode commandMode=COMMAND_DEFAULT;StopState stopState=NORMAL;
 bool locked=false,isEnabled=true,calibration=false,parameter=false,rgbSaveBusy=false,startPending=false;
 uint32_t motionEpoch=71;uint8_t jointsStateFlag=0;
 float jointSpeed=35,currentRailPos=0,targetRailPos=0;
 DOF6Kinematic::Joint6D_t initPose{0,-75,180,0,0,0},currentJoints{},targetJoints{};
 DOF6Kinematic* dof6Solver=nullptr;Motor values[7];Motor* motorJ[8]={nullptr,&values[0],&values[1],&values[2],&values[3],&values[4],&values[5],&values[6]};
 float jointAccRuntime[8]={0,30,30,30,30,30,30,30};unsigned*hcan=&hcan1;
 uint32_t GetMotionEpoch()const{return motionEpoch;}bool IsEnabled()const{return isEnabled;}bool IsStopLocked()const{return locked;}bool IsCalibrationBusy()const{return calibration;}bool IsProtectionParameterBusy()const{return parameter;}
 bool CanSendMotion(uint32_t epoch,uint32_t generation)const{return motionActive&&!CoreNativeOwned()&&isEnabled&&!IsStopLocked()&&epoch==motionEpoch&&generation==motionCompletionGeneration;}
 uint32_t AllocateMotionTaskId(){if(!++issuedMotionId)++issuedMotionId;return issuedMotionId;}
 void RequestSoftStop(){++softStops;locked=true;++motionEpoch;CancelMotion();CoreStreamCancel();}
 void MotorAccelerationCallback(unsigned node,float value,unsigned){jointAccRuntime[node]=value;}
 bool IsMotionBusy()const{return motionActive||CoreNativeOwned()||CoreStreamBusy();}
 bool IsMoving()const{return motionActive;}
#include "fields.inc"
 void MoveJoints(DOF6Kinematic::Joint6D_t);void UpdateJointAngles();bool MotionCompletionCallback(uint8_t,const uint8_t*);void PollMotionCompletion();void CancelMotion();bool WaitMotionCompletion(uint32_t,uint32_t);
 bool RunCartesian(const DOF6Kinematic::Pose6D_t&,const DOF6Kinematic::Pose6D_t&,bool,uint32_t,uint32_t);
 bool PlanMoveJ(float,float,float,float,float,float,float,float,uint32_t,uint32_t,const char*){return false;}
 bool PlanMoveL(float x,float y,float z,float a,float b,float c,uint32_t epoch,uint32_t id){auto goal=NativeCartesian::FromXYZABC(x*.001f,y*.001f,z*.001f,a,b,c);goal.Y+=currentRailPos*.001f;return RunCartesian(goal,NativeCartesian::FromXYZABC(0,0,0,0,0,0),false,epoch,id);}
 void SetMotionSpeed(float s,uint32_t e){if(e==motionEpoch)jointSpeed=s;}bool HomingForGeneration(uint32_t,uint32_t){return false;}bool RestingForGeneration(uint32_t,uint32_t){return false;}void CalibrateForGeneration(uint32_t){}
 class CommandHandler{public:
  struct QueueItem{uint32_t epoch,id;CommandMode mode;StreamSink*response;char command[160];};DummyRobot*context;FakeQueue queue{sizeof(QueueItem),16,{}};FakeQueue*commandFifo=&queue;
  bool parsing=false;uint32_t activeEpoch=0,poppedId=0,poppedEpoch=0,announcedId[3]{};CommandMode poppedMode=COMMAND_DEFAULT;StreamSink*poppedResponse=nullptr;
  explicit CommandHandler(DummyRobot* c):context(c){}bool IsBusy()const{return parsing||osMessageQueueGetSpace(commandFifo)!=16;}
  static const char*Name(const std::string&cmd){return Name(cmd.c_str());}static const char*Name(const char*cmd){return cmd[0]=='@'?"MOVEL":"MOVEJ";}
  struct ExecutionScope{CommandHandler&h;ExecutionScope(CommandHandler&a,uint32_t e):h(a){h.activeEpoch=e;h.parsing=true;}~ExecutionScope(){h.parsing=false;}};
  uint32_t Push(const std::string&,StreamSink*);std::string Pop(uint32_t);uint32_t ParseCommand(const std::string&);uint32_t GetSpace();bool IsAnnounced(uint32_t,StreamSink*)const;void Announce(const QueueItem&);
 }commandHandler{this};
}dummy;
namespace TcpTool{DOF6Kinematic::Pose6D_t Get(){return NativeCartesian::FromXYZABC(0,0,0,0,0,0);}}
#include "adapter.inc"
#include "robot.inc"
void EnterCritical(){
 if(!criticalNesting&&stopAtPublish&&precheckSeen&&!strcmp(lastPlan,"OK")){stopAtPublish=false;dummy.RequestSoftStop();}
 // The first producer critical section checks the settled result; the second
 // publishes the final completion. Inject user STOP immediately before that
 // second lock, after physical completion but before terminal success.
 if(!criticalNesting&&!inDelay&&stopAtCompleteBoundary&&dummy.nativePhase==DummyRobot::NATIVE_SETTLING&&dummy.motionPhase==DummyRobot::MOTION_COMPLETED&&++completionCriticalSeen==2){stopAtCompleteBoundary=false;dummy.RequestSoftStop();}
 ++criticalNesting;
}
using namespace StreamProtocol;
static int32_t held[7]{};static Axis endpoint[7]{};static std::vector<Axis> motorRecords[7];static uint16_t committedMotor[7]{},finalMotor[7]{};static unsigned starts=0;
static void StatusReply(unsigned node,uint16_t token,uint16_t sequence,uint8_t state){uint8_t d[8]{};Write16(d,token);Write16(d+2,sequence);d[4]=state;d[5]=CAPACITY;d[7]=VERSION;CoreStreamReceive(uint8_t(node),STATUS,d);}
static void ConsumeFrames(){while(frameCursor<frames.size()){const auto f=frames[frameCursor++];const auto token=Read16(f.data),sequence=Read16(f.data+2);CHECK(!criticalNesting);CHECK(f.command>=0x20);
 if(f.command==CAPS){uint8_t d[8]{};Write16(d,token);d[2]=VERSION;d[3]=CAP_TIMED|CAP_OPEN_SNAPSHOT|CAP_STABLE_ENTRY;Write16(d+4,SEGMENT_TICKS);d[6]=CAPACITY;d[7]=42;CoreStreamReceive(uint8_t(f.node),CAPS,d);}
 else if(f.command==OPEN){motorRecords[f.node-1].clear();committedMotor[f.node-1]=finalMotor[f.node-1]=0;StatusReply(f.node,token,0,STATE_OPEN);}
 else if(f.command==DIAG){uint8_t d[8]{};memcpy(d,f.data,4);Write32(d+4,uint32_t(held[f.node-1]));CoreStreamReceive(uint8_t(f.node),DIAG,d);}
 else if(f.command>=Q&&f.command<=A){auto&p=endpoint[f.node-1];if(f.command==Q)p.q=ReadI32(f.data+4);else if(f.command==V)p.v=ReadI32(f.data+4);else p.a=ReadI32(f.data+4);}
 else if(f.command==PREPARE||f.command==COMMIT){const auto&p=endpoint[f.node-1];CHECK(Read16(f.data+4)==CrcEndpoint(token,sequence,f.data[6],p.q,p.v,p.a));if(f.command==COMMIT){committedMotor[f.node-1]=sequence;if(f.data[6]&END)finalMotor[f.node-1]=sequence;auto&all=motorRecords[f.node-1];if(all.size()<=sequence)all.resize(sequence+1);all[sequence]=p;}StatusReply(f.node,token,sequence,f.command==PREPARE?STATE_PREPARED:STATE_COMMITTED);}
 else if(f.command==START){++starts;for(unsigned n=1;n<=7;++n)StatusReply(n,token,1,STATE_STARTED);}
 else if(f.command==SYNC){auto elapsed=Read32(f.data+4)/20;for(unsigned n=1;n<=7;++n){auto executed=uint16_t(std::min<uint32_t>(elapsed/20,committedMotor[n-1]));bool final=finalMotor[n-1]&&executed==finalMotor[n-1];if(executed<motorRecords[n-1].size()){auto q=motorRecords[n-1][executed].q;auto&m=dummy.values[n-1];m.motor.goalPosition=m.motor.realPosition=m.motor.softPosition=m.motor.positionCompletionTarget=q;m.angle=q*360.f/(51200.f*m.reduction)*(m.inverseDirection?-1:1);held[n-1]=q;}StatusReply(n,token,executed,final?STATE_COMPLETED:STATE_RUNNING);}}
 else if(f.command==ABORT){if(attemptCloseInt&&!f.data[2]){attemptCloseInt=false;TryIntDuringNative();}if(stopOnClose&&!f.data[2]){stopOnClose=false;dummy.RequestSoftStop();}if(!loseClose&&!f.data[2])for(unsigned n=1;n<=7;++n)StatusReply(n,token,0,STATE_IDLE);}
 else CHECK(false);
}}
bool Motor::QueryPositionCompletion(float target,uint32_t nonce,uint32_t,uint32_t){if(stopInSettling){stopInSettling=false;CHECK(dummy.nativePhase==DummyRobot::NATIVE_SETTLING&&!CoreNativeOwned()&&!CoreStreamActive());dummy.RequestSoftStop();return false;}if(attemptSettlingInt){attemptSettlingInt=false;CHECK(dummy.nativePhase==DummyRobot::NATIVE_SETTLING);TryIntDuringNative();}++taggedQueries;boardConfig.canNodeId=uint8_t(node);motor.positionSettled=settlingAt&&nowMs-settlingAt>=150;uint8_t d[8]{};float actual=motor.realPosition/51200.f;memcpy(d,&actual,4);d[4]=motor.Status(target);if(denyGoal)d[4]&=uint8_t(~MotionCompletionProtocol::TARGET_READY);MotionCompletionProtocol::PutNonce(d,nonce);CHECK(dummy.MotionCompletionCallback(uint8_t(node),d));return true;}
static unsigned ordinaryTick=0,queryTick=0;
void osDelay(unsigned duration){CHECK(!criticalNesting);const bool oldInDelay=inDelay;inDelay=true;for(unsigned i=0;i<duration;++i){++nowMs;CoreStreamPoll();ConsumeFrames();if(attemptHandoffInt&&!CoreNativeOwned()&&dummy.nativePhase==DummyRobot::NATIVE_EXECUTING){attemptHandoffInt=false;TryIntDuringNative();}if(executionHook&&CoreNativeSnapshotRead().state==CoreStreamEngine::RUNNING){auto callback=executionHook;executionHook=nullptr;callback();}if(++ordinaryTick<5)continue;ordinaryTick=0;++queryTick;dummy.PollMotionCompletion();if(dummy.IsEnabled()){dummy.MoveJoints(dummy.currentJoints);if(dummy.IsMotionBusy()||(queryTick&3)==0)dummy.UpdateJointAngles();}}inDelay=oldInDelay;}
static DOF6Kinematic solver(.2073f,0,.17f,.117f,.0695f,.113f);
static void Reset(float firstJointStep=0){CHECK(!CoreNativeOwned()&&!CoreStreamActive());dummy.locked=false;dummy.isEnabled=true;dummy.motionActive=dummy.motionDispatchPending=false;dummy.motionPhase=DummyRobot::MOTION_IDLE;dummy.commandMode=DummyRobot::COMMAND_DEFAULT;dummy.motionTaskId=0;dummy.dof6Solver=&solver;dummy.commandHandler.queue.data.clear();dummy.motionTxOkMask=dummy.motionTxPendingMask=dummy.motionTargetReadyMask=0;dummy.motionQueryNode=1;frameCursor=frames.size();starts=ordinaryTargets=taggedQueries=0;precheckSeen=stopAtPublish=loseClose=denyGoal=false;settlingAt=0;lastPlan="NONE";plannerLease=false;
 const float initial[6]={firstJointStep*360.f/(50.f*51200.f),10,80,0,90,0};
 for(unsigned j=0;j<7;++j){auto&m=dummy.values[j];m.node=j+1;m.inverseDirection=j>0;m.reduction=j==5?30:j==6?1:50;m.angleLimitMin=j==1?-75:j==2?0:j==3?-270:j==4?-100:j==5?-180:j==6?-250:-175;m.angleLimitMax=j==1?90:j==2?180:j==3?270:j==4?100:j==5?180:j==6?250:175;m.angle=j==6?0:initial[j]-dummy.initPose.a[j];held[j]=int32_t(lround(m.angle/360.f*m.reduction*51200.f*(m.inverseDirection?-1:1)));m.motor.goalPosition=m.motor.realPosition=m.motor.softPosition=m.motor.positionCompletionTarget=held[j];if(j<6)dummy.currentJoints.a[j]=dummy.initPose.a[j]+held[j]*360.f/(m.reduction*51200.f)*(m.inverseDirection?-1:1);}
}
static DOF6Kinematic::Pose6D_t Goal(float dx=0){DOF6Kinematic::Pose6D_t p;CHECK(solver.SolveFK(dummy.currentJoints,p));p.X+=dx;return p;}
static const auto tool=NativeCartesian::FromXYZABC(0,0,0,0,0,0);
static bool Has(const char* s){return std::any_of(output.lines.begin(),output.lines.end(),[&](const std::string&t){return t.find(s)!=std::string::npos;});}
static std::string holdCommand;
static void IntReject(){CHECK(dummy.commandHandler.Push(holdCommand,&output)==0xFE);CHECK(dummy.commandHandler.queue.data.empty());}
static void SeqQueue(){CHECK(dummy.commandHandler.Push(holdCommand,&output)!=0xFF);CHECK(dummy.commandHandler.queue.data.size()==1);}
static void TryIntDuringNative(){CHECK(dummy.commandMode==DummyRobot::COMMAND_DEFAULT);CHECK(dummy.commandHandler.Push(">0,10,80,0,90,0,0,35",&output)==0xFE);CHECK(dummy.commandHandler.queue.data.empty());++intRejections;}
int main(){
 for(int32_t q:{53,-53,430001,-430001,850002,-850002,1650003,-1650003}){MotorProbe m;m.realPosition=m.goalPosition=m.softPosition=m.positionCompletionTarget=q;volatile float wire=q/51200.f;float r=wire;uint8_t before=m.Status(r);float fixed=std::nextafterf(r,q>0?INFINITY:-INFINITY);uint8_t after=m.Status(fixed);printf("actual_motor_STATUS q=%ld float_steps=%.9g decoded=%ld flags=%02X; one_ulp_steps=%.9g decoded=%ld flags=%02X\n",(long)q,double(r*51200.f),(long)int32_t(r*51200.f),before,double(fixed*51200.f),(long)int32_t(fixed*51200.f),after);CHECK(after&MotionCompletionProtocol::TARGET_READY);}
 for(int32_t q=-2560000;q<=2560000;++q){float value=0;CHECK(ExactMotorR(q,value));volatile float wire=value;volatile float motorSteps=wire*51200.f;CHECK(int32_t(motorSteps)==q);}
 printf("actual_ExactMotorR exhaustive signed5120001 physical endpoints PASS\n");
 Reset();onCommandTask=false;CHECK(!dummy.RunCartesian(Goal(),tool,false,dummy.motionEpoch,1));CHECK(!CoreNativeOwned());onCommandTask=true;
 Reset();CHECK(dummy.RunCartesian(Goal(),tool,false,dummy.motionEpoch,2));for(const auto&s:output.lines)printf("WIRE %s\n",s.c_str());CHECK(Has("stage=SETTLING")&&Has("stage=DONE"));CHECK(dummy.motionPhase==DummyRobot::MOTION_COMPLETED);CHECK(nowMs-settlingAt>=150);CHECK(starts==1&&ordinaryTargets==0&&taggedQueries>=7);printf("finite_hold PASS, tagged=%u ordinary=%u\n",taggedQueries,ordinaryTargets);
 Reset();CHECK(dummy.RunCartesian(Goal(.005f),tool,false,dummy.motionEpoch,3));CHECK(dummy.motionPhase==DummyRobot::MOTION_COMPLETED);printf("finite_5mm passed, tagged=%u ordinary=%u final_q1=%ld\n",taggedQueries,ordinaryTargets,(long)held[0]);
 Reset();auto goal=Goal();float p[6];NativeCartesian::ToXYZABC(goal,p);char cmd[180];snprintf(cmd,sizeof(cmd),"@%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,35",p[0],p[1],p[2],p[3],p[4],p[5]);CHECK(dummy.commandHandler.Push(cmd,&output)!=0xFF);auto line=dummy.commandHandler.Pop(0);dummy.commandHandler.ParseCommand(line);CHECK(dummy.motionPhase==DummyRobot::MOTION_COMPLETED);CHECK(!dummy.commandHandler.IsBusy());printf("actual_QueuePush_Pop_Parse_ExecutionScope PASS\n");
 holdCommand=cmd;Reset();executionHook=IntReject;CHECK(dummy.RunCartesian(Goal(),tool,false,dummy.motionEpoch,0));CHECK(!executionHook);CHECK(ordinaryTargets==0);printf("INT_native_busy_explicit_rejection PASS\n");
 Reset();dummy.commandMode=DummyRobot::COMMAND_TARGET_POINT_SEQUENTIAL;executionHook=SeqQueue;CHECK(dummy.commandHandler.Push(holdCommand,&output)!=0xFF);line=dummy.commandHandler.Pop(0);dummy.commandHandler.ParseCommand(line);CHECK(dummy.motionPhase==DummyRobot::MOTION_COMPLETED);CHECK(dummy.commandHandler.IsBusy());line=dummy.commandHandler.Pop(0);dummy.commandHandler.ParseCommand(line);CHECK(dummy.motionPhase==DummyRobot::MOTION_COMPLETED);CHECK(!dummy.commandHandler.IsBusy());CHECK(ordinaryTargets==0);printf("SEQ_native_queue_two_finite_commands PASS\n");
 Reset();attemptCloseInt=attemptHandoffInt=attemptSettlingInt=true;CHECK(dummy.RunCartesian(Goal(),tool,false,dummy.motionEpoch,0));CHECK(!attemptCloseInt&&!attemptHandoffInt&&!attemptSettlingInt&&intRejections==3);CHECK(ordinaryTargets==0);printf("EXT_INT_CLOSE_handoff_SETTLING_three_windows_explicit_BUSY PASS\n");
 Reset();auto impossible=Goal();impossible.X=3;CHECK(!dummy.RunCartesian(impossible,tool,false,dummy.motionEpoch,4));CHECK(starts==0&&!CoreNativeOwned()&&!dummy.locked);printf("preflight_rejection_no_START PASS\n");
 Reset();stopAtPublish=true;CHECK(!dummy.RunCartesian(Goal(),tool,false,dummy.motionEpoch,5));CHECK(starts==0&&!CoreNativeOwned()&&dummy.locked);CHECK(!dummy.motionActive);printf("STOP_at_publication_lock PASS\n");
 Reset();loseClose=true;CHECK(!dummy.RunCartesian(Goal(),tool,false,dummy.motionEpoch,6));CHECK(CoreNativeOwned()&&CoreStreamActive()&&dummy.locked);CHECK(CoreNativeBegin(dummy.motionEpoch,7)==BUSY);CHECK(ordinaryTargets==0);loseClose=false;for(unsigned i=0;i<60&&CoreNativeOwned();++i)osDelay(1);CHECK(!CoreNativeOwned()&&!CoreStreamActive()&&dummy.locked);printf("lost_CLOSE_then_late_ACK_keeps_BUSY PASS\n");
 Reset(53);const auto expected=held[0];CHECK(expected==53);CHECK(dummy.RunCartesian(Goal(),tool,false,dummy.motionEpoch,8));printf("roundtrip q=%ld final=%ld ordinary=%u tagged=%u\n",(long)expected,(long)held[0],ordinaryTargets,taggedQueries);CHECK(dummy.motionPhase==DummyRobot::MOTION_COMPLETED&&!dummy.locked);CHECK(taggedQueries>=7);
 Reset();denyGoal=true;CHECK(!dummy.RunCartesian(Goal(),tool,false,dummy.motionEpoch,9));CHECK(ordinaryTargets==0&&dummy.locked);printf("native_settling_missing_TARGET_READY_never_ordinary_retry PASS\n");
 Reset();stopOnClose=true;CHECK(!dummy.RunCartesian(Goal(),tool,false,dummy.motionEpoch,10));CHECK(dummy.locked&&ordinaryTargets==0&&taggedQueries==0);CHECK(!CoreNativeOwned());printf("user_STOP_during_CLOSE_no_false_DONE PASS\n");
 Reset();stopInSettling=true;CHECK(!dummy.RunCartesian(Goal(),tool,false,dummy.motionEpoch,11));CHECK(!stopInSettling&&dummy.locked&&!dummy.motionActive&&ordinaryTargets==0);CHECK(!CoreNativeOwned());CHECK(Has("id=11 stage=SETTLING"));CHECK(Has("id=11 reason=CANCELLED code=12"));CHECK(!Has("id=11 stage=DONE"));printf("user_STOP_inside_SETTLING_wait_CANCELLED12_no_DONE_no_ordinary PASS\n");
 Reset();stopAtCompleteBoundary=true;completionCriticalSeen=0;CHECK(!dummy.RunCartesian(Goal(),tool,false,dummy.motionEpoch,12));CHECK(!stopAtCompleteBoundary&&completionCriticalSeen==2&&dummy.locked&&ordinaryTargets==0);CHECK(taggedQueries>=7&&!CoreNativeOwned());CHECK(Has("id=12 reason=CANCELLED code=12"));CHECK(!Has("id=12 stage=DONE"));printf("user_STOP_at_final_completion_lock_CANCELLED12_no_DONE_no_ordinary PASS\n");
 for(const auto&s:output.lines)if(s.find("reason=")!=std::string::npos)printf("%s\n",s.c_str());
 CHECK(Has("id=5 reason=CANCELLED code=12"));CHECK(Has("id=6 reason=EXECUTOR_CLOSE code=17"));CHECK(Has("id=10 reason=CANCELLED code=12"));
 CHECK(Has("id=9 reason=TARGET_READY_TIMEOUT code=17"));CHECK(!Has("id=9 stage=DONE"));
 CHECK(ordinaryTargets==0);
 printf("root_integration %u checks PASS; actual RunCartesian+planner+IK+adapter+engine+tagged completion; no hardware\n",checks);
}
