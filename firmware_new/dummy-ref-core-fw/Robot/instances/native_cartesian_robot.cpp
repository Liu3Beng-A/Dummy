#include "common_inc.h"
#include "cartesian_planner.h"
#include "tcp_tool.h"
#include "motion_values.h"
#include <cstring>
#include <cmath>
extern DummyRobot dummy;
extern osThreadId_t ControlLoopUpdateHandle;
namespace {
NativeCartesian::Planner planner;
volatile bool plannerLease=false;
uint32_t maxPlanningUs=0,lastPlanningUs=0,lastWork=0;
uint32_t maxIKCheckUs=0;
float lastBound=0,lastRotation=0;
const char* lastPlan="NONE";
class RobotProgress final:public NativeCartesian::Progress {
public:
    explicit RobotProgress(uint32_t e):epoch(e),began(HAL_GetTick()){}
    bool budgetExpired=false;
    bool Checkpoint() override {
        if(dummy.GetMotionEpoch()!=epoch||dummy.IsStopLocked()||!dummy.IsEnabled())return false;
        if(HAL_GetTick()-began>30000){budgetExpired=true;return false;}
        if((++calls&31)==0){CoreNativeHeartbeat(epoch);osDelay(1);}
        return true;
    }
private:uint32_t epoch,began,calls=0;
};
bool Equal(const char* cmd,unsigned n,const char* literal){return std::strlen(literal)==n&&!std::memcmp(cmd,literal,n);}
bool Prefix(const char* cmd,unsigned n,const char* literal){unsigned k=std::strlen(literal);return n>=k&&!std::memcmp(cmd,literal,k);}
bool ExactMotorR(int32_t q,float& value){
    value=float(q)/51200.f;
    for(unsigned retry=0;retry<4;++retry){volatile float scaled=value*51200.f;
        if(scaled>=2147483648.f||scaled<-2147483648.f)return false;
        if(int32_t(scaled)==q)return true;
        value=std::nextafterf(value,q>=0?1.e30f:-1.e30f);
    }
    return false;
}
void PrintTool(StreamSink& response){auto p=TcpTool::Get();auto s=TcpTool::GetStatus();
    Respond(response,"TOOL v=1 rev=%lu dirty=%u storage=%u",(unsigned long)s.revision,unsigned(s.dirty),unsigned(s.storage));
    Respond(response,"TOOL XYZ %.3f %.3f %.3f",p.X*1000,p.Y*1000,p.Z*1000);
    Respond(response,"TOOL ABC %.3f %.3f %.3f",p.A,p.B,p.C);}
}
bool DummyRobot::HandleCartesianCommand(const char* cmd,unsigned n,StreamSink& response){
    while(n&&(*cmd==' '||*cmd=='\t'||*cmd=='\r'||*cmd=='\n')){++cmd;--n;}
    while(n&&(cmd[n-1]==' '||cmd[n-1]=='\t'||cmd[n-1]=='\r'||cmd[n-1]=='\n'))--n;
    if(Equal(cmd,n,"#KIN_CONFIG")){
        Respond(response,"KIN v=2 xyz=mm abc=deg joints=deg rail=mm");
        Respond(response,"KIN base=207.3 db=0 arm=170 forearm=117");
        Respond(response,"KIN elbow=69.5 wrist=113 tool_default=114");
        Respond(response,"KIN rail_axis=+Y rail_min=-250 rail_max=250");
        Respond(response,"KIN TCP=@T x,y,z,a,b,c,speed,rail_auto");return true;}
    if(Equal(cmd,n,"#CARTESIAN_STATUS")){
        auto s=CoreNativeSnapshotRead();
        Respond(response,"CARTESIAN id=%lu owner=%u state=%u code=%u",(unsigned long)s.id,unsigned(s.owned),unsigned(s.state),unsigned(s.reason));
        Respond(response,"CARTESIAN n=%u c=%u e=%u free=%u",unsigned(s.next),unsigned(s.committed),unsigned(s.executed),unsigned(s.free));
        Respond(response,"CARTESIAN fault src=%u st=%u node=%u n=%u",unsigned(s.failure.source),unsigned(s.failure.state),unsigned(s.failure.node),unsigned(s.failure.sequence));
        Respond(response,"CARTESIAN fault age=%lu cmd=%u gap=%lu",(unsigned long)s.failure.age,unsigned(s.failure.command),(unsigned long)s.failure.gap);
        Respond(response,"CARTESIAN plan=%s work=%lu",lastPlan,(unsigned long)lastWork);
        Respond(response,"CARTESIAN us=%lu max_us=%lu",(unsigned long)lastPlanningUs,(unsigned long)maxPlanningUs);
        Respond(response,"CARTESIAN bound_mm=%.4f bound_deg=%.4f",lastBound*1000,lastRotation*57.2957795f);return true;}
    if(Equal(cmd,n,"#TOOL")){PrintTool(response);return true;}
    if(Equal(cmd,n,"#TCPPOSE")){
        DOF6Kinematic::Joint6D_t joint{};float rail;
        taskENTER_CRITICAL();joint=currentJoints;rail=currentRailPos;taskEXIT_CRITICAL();
        DOF6Kinematic::Pose6D_t flange{};if(!dof6Solver->SolveFK(joint,flange)){Respond(response,"error TCPPOSE FK_INVALID");return true;}
        auto tcp=NativeCartesian::Compose(flange,TcpTool::Get());tcp.Y+=rail*.001f;float p[6];NativeCartesian::ToXYZABC(tcp,p);
        Respond(response,"TCPPOSE world XYZ %.3f %.3f %.3f",p[0],p[1],p[2]);Respond(response,"TCPPOSE ABC %.3f %.3f %.3f",p[3],p[4],p[5]);return true;}
    if(Prefix(cmd,n,"#IKCHECK ")){
        if(CoreNativeOwned()){Respond(response,"error IKCHECK native_busy");return true;}
        float q[6];if(!MotionValues::ParseCartesian(cmd+9,n-9,q,6,6)){Respond(response,"error IKCHECK syntax");return true;}
        DOF6Kinematic::Joint6D_t joint{};std::memcpy(joint.a,q,sizeof(q));DOF6Kinematic::Pose6D_t p{};DOF6Kinematic::IKSolves_t solutions{};uint32_t began=micros();
        if(!dof6Solver->SolveFK(joint,p)||!dof6Solver->SolveIKMeters(p,joint,solutions)){Respond(response,"error IKCHECK NO_IK");return true;}
        unsigned valid=0;for(unsigned i=0;i<8;++i)valid+=solutions.solFlag[i][0]&&solutions.solFlag[i][1]&&solutions.solFlag[i][2];
        uint32_t elapsed=micros()-began;if(elapsed>maxIKCheckUs)maxIKCheckUs=elapsed;
        Respond(response,"IKCHECK wall_us=%lu max=%lu",(unsigned long)elapsed,(unsigned long)maxIKCheckUs);
        Respond(response,"IKCHECK valid=%u no_motion=1",valid);Respond(response,"IKCHECK flange XYZ %.3f %.3f %.3f",p.X*1000,p.Y*1000,p.Z*1000);return true;}
    const bool set=Prefix(cmd,n,"#TOOL_SET "),reset=Equal(cmd,n,"#TOOL_RESET"),save=Equal(cmd,n,"#TOOL_SAVE");
    if(!set&&!reset&&!save){if(Prefix(cmd,n,"#TOOL")){Respond(response,"error TOOL syntax");return true;}return false;}
    float values[6]{};if(set&&!MotionValues::ParseCartesian(cmd+10,n-10,values,6,6)){Respond(response,"error TOOL syntax");return true;}
    if(set&&(std::fabs(values[0])>500||std::fabs(values[1])>500||std::fabs(values[2])>500)){Respond(response,"error TOOL XYZ range=+-500mm");return true;}
    taskENTER_CRITICAL();
    bool allowed=!rgbSaveBusy&&!motionActive&&!CoreStreamActive()&&!plannerLease&&!commandHandler.IsBusy()&&!IsCalibrationBusy()&&!IsProtectionParameterBusy()&&!startPending;
    if(save)allowed=allowed&&stopState==DISABLED&&!isEnabled&&!stopFaultMask;
    else allowed=allowed&&(stopState==NORMAL||stopState==DISABLED);
    bool ok=false;if(allowed){if(save)rgbSaveBusy=true;else if(set)ok=TcpTool::Set(values[0],values[1],values[2],values[3],values[4],values[5]);else{TcpTool::Reset();ok=true;}}
    taskEXIT_CRITICAL();
    if(!allowed){Respond(response,save?"error TOOL save_requires_idle_disabled":"error TOOL idle_required");return true;}
    if(save){ok=TcpTool::Save();taskENTER_CRITICAL();rgbSaveBusy=false;taskEXIT_CRITICAL();}
    Respond(response,ok?(save?"ok TOOL saved":"ok TOOL applied"):"error TOOL invalid_or_flash_failure");PrintTool(response);return true;
}
bool DummyRobot::RunCartesian(const DOF6Kinematic::Pose6D_t& goal,const DOF6Kinematic::Pose6D_t& tool,bool automaticRail,uint32_t epoch,uint32_t id){
    // The serialized command task is the sole producer; direct fibre callers
    // must not block the USB task that is also responsible for emergency STOP.
    if(osThreadGetId()!=ControlLoopUpdateHandle)return false;
    taskENTER_CRITICAL();
    bool allowed=!plannerLease&&!motionActive&&!CoreStreamActive()&&!IsStopLocked()&&isEnabled&&epoch==motionEpoch&&commandMode<=COMMAND_CONTINUES_TRAJECTORY&&!rgbSaveBusy;
    if(allowed)plannerLease=true;taskEXIT_CRITICAL();if(!allowed)return false;
    uint32_t ownId=id?id:AllocateMotionTaskId();uint8_t code=CoreNativeBegin(epoch,ownId);bool success=false,started=false,settlingFailure=false;
    const char* reason="EXECUTOR_BEGIN";uint32_t began=HAL_GetTick();
    auto valid=[&](){return epoch==motionEpoch&&!IsStopLocked()&&isEnabled;};
    auto wait=[&](){CoreNativeHeartbeat(epoch);osDelay(1);return valid();};
    auto stage=[&](const char* name){if(usbStreamOutputPtr)Respond(*usbStreamOutputPtr,"CARTESIAN id=%lu stage=%s",(unsigned long)ownId,name);};
    NativeCartesian::Request request{};NativeCartesian::Limits limits{};StreamProtocol::Record record{};uint32_t trajectoryStartedAt=0;
    if(code==StreamProtocol::OK){
        stage("OPEN");
        while(valid()&&HAL_GetTick()-began<4000){auto s=CoreNativeSnapshotRead();if(s.reason){code=s.reason;break;}if(s.state==CoreStreamEngine::OPEN)break;wait();}
        auto s=CoreNativeSnapshotRead();if(s.state!=CoreStreamEngine::OPEN||!valid())reason="EXECUTOR_OPEN";
        else{
            bool held=true;for(uint8_t node=1;node<=7&&valid();++node){code=CoreNativeReadHeld(node,epoch);uint32_t at=HAL_GetTick();
                while(valid()&&HAL_GetTick()-at<1000&&!CoreNativeHeld(node,request.seed[node-1],epoch))wait();
                if(code!=StreamProtocol::OK||!CoreNativeHeld(node,request.seed[node-1],epoch)){held=false;break;}}
            if(!held||!valid())reason="EXECUTOR_HELD";
            else{
                request.goal=goal;request.tool=tool;request.automaticRail=automaticRail;request.exactSeed=true;request.speed=jointSpeed;
                for(unsigned j=0;j<7;++j){const auto* m=motorJ[j+1];float direction=m->inverseDirection?-1.f:1.f;
                    limits.origin[j]=j==6?0:initPose.a[j];limits.stepsPerUnit[j]=direction*(j==6?51200.f/RAIL_MM_PER_CIRCLE:51200.f*m->reduction/360.f);
                    request.start[j]=limits.origin[j]+request.seed[j]/limits.stepsPerUnit[j];limits.lower[j]=m->angleLimitMin;limits.upper[j]=m->angleLimitMax;
                    limits.velocity[j]=j==6?20.f:40.f;float physical=jointAccRuntime[j+1]*(j==6?RAIL_MM_PER_CIRCLE:360.f/m->reduction);
                    limits.acceleration[j]=std::fmin(j==6?40.f:80.f,physical>0?physical:40.f);limits.jerk[j]=j==6?3000.f:20000.f;}
                stage("PRECHECK");RobotProgress progress(epoch);uint32_t at=micros();auto result=planner.Build(*dof6Solver,request,limits,progress);
                if(progress.budgetExpired&&result==NativeCartesian::CANCELLED)result=NativeCartesian::BUDGET;
                lastPlanningUs=micros()-at;if(lastPlanningUs>maxPlanningUs)maxPlanningUs=lastPlanningUs;lastWork=planner.Work();lastPlan=NativeCartesian::Name(result);
                lastBound=planner.PositionBound();lastRotation=planner.RotationBound();reason=lastPlan;
                if(result==NativeCartesian::OK&&valid()){
                    taskENTER_CRITICAL();bool publishing=valid()&&CoreNativeOwned()&&CoreNativeSnapshotRead().id==ownId;
                    if(publishing){++motionCompletionGeneration;motionTaskId=ownId;motionTaskEpoch=epoch;motionMode=commandMode;motionKind="MOVEL";motionPhase=MOTION_DISPATCHING;
                    motionFailureReason=nullptr;nativePhase=NATIVE_EXECUTING;motionActive=true;motionDispatchPending=false;motionStartedAt=trajectoryStartedAt=HAL_GetTick();motionCompletedAt=0;motionProgressMask=jointsStateFlag=0;
                    targetRailPos=planner.RailTarget();std::memcpy(targetJoints.a,planner.GoalJoints(),sizeof(targetJoints.a));}taskEXIT_CRITICAL();
                    if(publishing){
                    if(usbStreamOutputPtr)Respond(*usbStreamOutputPtr,"CARTESIAN id=%lu n=%u rail=%.2f",(unsigned long)ownId,planner.Segments(),planner.RailTarget());
                    unsigned sequence=0;bool runningPublished=false;stage("BUFFER");uint32_t executionAt=HAL_GetTick();reason="EXECUTOR_FAULT";
                    while(valid()&&HAL_GetTick()-executionAt<uint32_t(planner.Duration()*1000)+10000){s=CoreNativeSnapshotRead();if(s.reason){code=s.reason;break;}
                        if(s.free&&sequence<=planner.Segments()){if(!planner.Record(sequence,s.session,record)){reason="RECORD_INVALID";break;}
                            code=CoreNativeSubmit(record,epoch);if(code==StreamProtocol::OK)++sequence;else if(code!=StreamProtocol::BUSY)break;}
                        if(!started&&s.state==CoreStreamEngine::ARMED){code=CoreNativeStart(epoch);if(code!=StreamProtocol::OK)break;started=true;stage("START_PENDING");}
                        if(started&&!runningPublished&&(s.state==CoreStreamEngine::RUNNING||s.state==CoreStreamEngine::DRAINING)){taskENTER_CRITICAL();runningPublished=valid()&&motionTaskId==ownId;if(runningPublished)motionPhase=MOTION_RUNNING;taskEXIT_CRITICAL();if(runningPublished)stage("RUNNING");}
                        if(started&&s.state==CoreStreamEngine::IDLE){success=true;break;}wait();
                    }
                    }else reason="CANCELLED";
                }
            }
        }
    }
    auto primary=CoreNativeSnapshotRead();
    if(primary.id==ownId&&primary.epoch==epoch&&primary.reason&&primary.reason!=StreamProtocol::ABORTED){code=primary.reason;reason="EXECUTOR_FAULT";}
    else if(!valid())reason="CANCELLED";
    // Retire the exact session before ordinary completion queries or another
    // command can take ownership. STOP changes epoch but cleanup uses its lease.
    if(CoreNativeOwned()){
        if(!success&&started&&valid())RequestSoftStop();
        if(!success&&started)CoreStreamCancel();
        CoreNativeClose(epoch);uint32_t closeAt=HAL_GetTick();
        while(CoreStreamActive()&&HAL_GetTick()-closeAt<1800){CoreNativeClose(epoch);osDelay(1);}
        if(CoreStreamActive()){success=false;reason="EXECUTOR_CLOSE";if(valid())RequestSoftStop();}
        CoreNativeRelease(epoch);
    }
    auto retired=CoreNativeSnapshotRead();
    if(retired.id==ownId&&retired.epoch==epoch&&retired.reason&&retired.reason!=StreamProtocol::ABORTED){code=retired.reason;if(retired.failure.source==CoreStreamEngine::SOURCE_CLOSE)reason="EXECUTOR_CLOSE";}
    else if(!valid()){reason="CANCELLED";if(!code)code=StreamProtocol::ABORTED;}
    if(success&&valid()){
        // END/COMPLETED proves reference execution. Existing tagged completion
        // rounds still prove physical settling without resending ordinary 0x07.
        float expected[7];for(unsigned j=0;j<7;++j)if(!ExactMotorR(record.axes[j].q,expected[j])){success=false;reason="ENDPOINT_FLOAT";break;}
        taskENTER_CRITICAL();success=success&&valid()&&motionTaskId==ownId&&!CoreNativeOwned();if(success){motionTxOkMask=motionTxHistoryMask=0xFE;motionTxPendingMask=0;motionTargetReadyMask=0;motionConfirmedMask=0;motionFailureReason=nullptr;
        for(unsigned j=0;j<7;++j){motionMotorTargets[j+1]=expected[j];motionTxAttempts[j+1]=0;motionFirstReadyMs[j+1]=MOTION_TIME_MISSING;motionReplyFlags[j+1]=0;motionRetryAt[j+1]=HAL_GetTick()+10000;}
        motionStartedAt=HAL_GetTick();nativePhase=NATIVE_SETTLING;BeginMotionRound();}taskEXIT_CRITICAL();if(success)stage("SETTLING");uint32_t at=HAL_GetTick();
        while(success&&valid()&&motionActive&&HAL_GetTick()-at<3000)osDelay(2);
        taskENTER_CRITICAL();success=success&&valid()&&motionTaskId==ownId&&motionPhase==MOTION_COMPLETED;
        if(success)motionStartedAt=trajectoryStartedAt;
        else if(motionTaskId==ownId&&motionPhase==MOTION_FAILED&&motionFailureReason){
            settlingFailure=true;reason=motionFailureReason;code=StreamProtocol::DEADLINE;
        }
        taskEXIT_CRITICAL();
        if(!success&&!settlingFailure){
            if(!valid()){reason="CANCELLED";if(!code)code=StreamProtocol::ABORTED;}
            else{reason="TCP_SETTLE_TIMEOUT";RequestSoftStop();}
        }
    }
    taskENTER_CRITICAL();plannerLease=false;success=success&&valid()&&motionTaskId==ownId&&motionPhase==MOTION_COMPLETED;
    if(!settlingFailure&&!valid()&&(code==StreamProtocol::OK||code==StreamProtocol::ABORTED)&&
       std::strcmp(reason,"TCP_SETTLE_TIMEOUT")&&std::strcmp(reason,"EXECUTOR_CLOSE")){
        reason="CANCELLED";code=StreamProtocol::ABORTED;
    }
    if(motionTaskId==ownId)nativePhase=NATIVE_NONE;
    if(!success&&epoch==motionEpoch){motionActive=motionDispatchPending=false;motionTaskId=ownId;motionTaskEpoch=epoch;motionKind="MOVEL";motionMode=commandMode;motionPhase=MOTION_FAILED;motionCompletedAt=HAL_GetTick();motionFailureReason=reason;}
    taskEXIT_CRITICAL();
    if(!success&&usbStreamOutputPtr){Respond(*usbStreamOutputPtr,"CARTESIAN id=%lu reason=%s code=%u",(unsigned long)ownId,reason,unsigned(code));}
    if(success)stage("DONE");return success;
}
