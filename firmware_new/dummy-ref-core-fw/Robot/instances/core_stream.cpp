#include "common_inc.h"
#include "core_stream.h"
#include "core_stream_engine.h"
#include <math.h>
#include <string.h>
extern DummyRobot dummy;
using namespace StreamProtocol;

namespace {
bool stopRequested=false,openTestRequest=false;
class RobotStreamPort final : public CoreStreamPort {
public:
    bool Send(uint8_t node,uint8_t command,const uint8_t data[8]) override {
        CAN_TxHeaderTypeDef header{};
        header.StdId=(uint32_t(node)<<7)|command; header.IDE=CAN_ID_STD;
        header.RTR=CAN_RTR_DATA; header.DLC=8;
        return TryCanSendStreamMessage(get_can_ctx(&hcan1),const_cast<uint8_t*>(data),&header,Read16(data));
    }
    bool Allowed() const override {
        return (openTestRequest || dummy.commandMode==DummyRobot::COMMAND_STREAM_JOINT) && dummy.IsEnabled() &&
            !dummy.IsStopLocked() && !dummy.IsCalibrationBusy() && !dummy.IsProtectionParameterBusy();
    }
    bool Validate(const Record& record,const Record* previous,uint8_t mask) const override {
        for(unsigned i=0;i<AXES;++i) {
            if(!(mask&(1U<<i))) continue;
            const auto* motor=dummy.motorJ[i+1];
            const double direction=motor->inverseDirection ? -1.0 : 1.0;
            const double factor=i==6 ? 51200.0/RAIL_MM_PER_CIRCLE : 51200.0*motor->reduction/360.0;
            const double origin=i==6 ? 0 : dummy.initPose.a[i];
            double minimum=(motor->angleLimitMin-origin)*factor*direction;
            double maximum=(motor->angleLimitMax-origin)*factor*direction;
            if(minimum>maximum) { const double tmp=minimum; minimum=maximum; maximum=tmp; }
            const auto& end=record.axes[i];
            const double velocity=30.0*51200.0;
            double acceleration=double(dummy.jointAccRuntime[i+1])*51200.0;
            if(acceleration<1 || acceleration>51200.0*3000.0) acceleration=30.0*51200.0;
            if(end.q<minimum-1 || end.q>maximum+1 || fabs(double(end.v))>velocity ||
                fabs(double(end.a))>acceleration) return false;
            if(!previous) {
                if(record.sequence || record.flags!=SEED || end.v || end.a) return false;
                const double actual=double(motor->angle)/360.0*motor->reduction*51200.0*direction;
                // The motor OPEN locks its already-held target. The cached float
                // feedback may differ by the existing physical arrival tolerance
                // plus one encoded integer step and its IEEE float roundoff.
                const double rounding=fabs(actual)*0x1p-22+1.0;
                if(fabs(double(end.q)-actual)>MotionCompletionProtocol::PositionToleranceSteps(uint8_t(i+1))+rounding) return false;
                continue;
            }
            const auto& begin=previous->axes[i];
            // Bernstein control hull bounds position, speed and acceleration
            // throughout the quintic; endpoint checks alone miss overshoots.
            constexpr double T=0.020;
            const double p[6]={double(begin.q),begin.q+begin.v*T/5,
                begin.q+2*begin.v*T/5+begin.a*T*T/20,
                end.q-2*end.v*T/5+end.a*T*T/20,end.q-end.v*T/5,double(end.q)};
            for(unsigned k=0;k<6;++k) if(p[k]<minimum-1 || p[k]>maximum+1) return false;
            for(unsigned k=0;k<5;++k) if(fabs(5*(p[k+1]-p[k])/T)>velocity+1) return false;
            for(unsigned k=0;k<4;++k) if(fabs(20*(p[k+2]-2*p[k+1]+p[k])/(T*T))>acceleration+1) return false;
        }
        return true;
    }
    void Stop() override { stopRequested=true; }
};
RobotStreamPort streamPort;
CoreStreamEngine engine(streamPort);
StreamSink* sink=nullptr;
uint32_t printedRevision=0, maxPollUs=0;
StreamSink* diagnosticSink=nullptr;
volatile bool diagnosticPending=false, diagnosticReady=false;
uint16_t diagnosticNonce=0;
uint8_t diagnosticNode=0, diagnosticField=0;
uint32_t diagnosticValue=0, diagnosticAt=0, diagnosticAttemptAt=0;

bool Equal(const char* value,size_t n,const char* literal) { return strlen(literal)==n && !memcmp(value,literal,n); }
bool Prefix(const char* value,size_t n,const char* literal) { const size_t m=strlen(literal); return n>=m && !memcmp(value,literal,m); }
bool Numbers(const char* p,size_t n,uint16_t* values,unsigned count) {
    size_t at=0;
    for(unsigned index=0;index<count;++index) {
        while(at<n && (p[at]==' ' || p[at]=='\t')) ++at;
        if(at==n || p[at]<'0' || p[at]>'9') return false;
        uint32_t number=0;
        while(at<n && p[at]>='0' && p[at]<='9') {
            number=number*10+unsigned(p[at++]-'0'); if(number>65535) return false;
        }
        values[index]=uint16_t(number);
        if(at<n && p[at]!=' ' && p[at]!='\t') return false;
    }
    while(at<n && (p[at]==' ' || p[at]=='\t')) ++at;
    return at==n;
}
struct Snapshot {
    CoreStreamEngine::State state; uint16_t session,next,finalSeq,committed,executed;
    uint8_t free,mask,reason; uint32_t revision,gap; CoreStreamEngine::FaultSnapshot failure;
    uint16_t featureSession; uint8_t stableEntryMask;
};
Snapshot ReadSnapshot() {
    taskENTER_CRITICAL();
    Snapshot s{engine.GetState(),engine.Session(),engine.Next(),engine.FinalSequence(),engine.LastCommitted(),
        engine.LastExecuted(),engine.Free(),engine.CapabilityMask(),engine.Reason(),engine.Revision(),engine.MaxPollGap(),engine.Failure(),engine.FeatureSession(),engine.StableEntryMask()};
    taskEXIT_CRITICAL(); return s;
}
void PrintFeatures(StreamSink& output,const Snapshot& s) {
    if(s.featureSession && s.mask)
        Respond(output,"STREAM_FEATURES s=%u steady=%u start=%u",unsigned(s.featureSession),
            unsigned(s.stableEntryMask),unsigned(s.stableEntryMask));
}
void Status(StreamSink& output,const Snapshot& s) {
    Respond(output,"STREAM_STATE s=%u st=%s next=%u free=%u",unsigned(s.session),
        CoreStreamEngine::Name(s.state),unsigned(s.next),unsigned(s.free));
}
void PrintFailure(StreamSink& output,const Snapshot& s) {
    const auto& f=s.failure;
    Respond(output,"STREAM_FAULT_AT s=%u src=%u st=%u n=%u age=%lu",unsigned(s.session),unsigned(f.source),unsigned(f.state),unsigned(f.sequence),(unsigned long)f.age);
    Respond(output,"STREAM_FAULT_MASK s=%u prep=%02X commit=%02X start=%02X",unsigned(s.session),unsigned(f.prepared),unsigned(f.committed),unsigned(f.started));
    Respond(output,"STREAM_FAULT_CAN s=%u node=%u cmd=%02X gap=%lu",unsigned(s.session),unsigned(f.node),unsigned(f.command),(unsigned long)f.gap);
    Respond(output,"STREAM_FAULT_T s=%u t=%lu ph=%u",unsigned(s.session),(unsigned long)f.at,unsigned(f.phase));
}

}

namespace {
void PrintOpenTest(StreamSink& response) {
    PrintFeatures(response,ReadSnapshot());
    uint16_t token;uint8_t node,reason;bool active,closing,failed,opened,capable;
    taskENTER_CRITICAL();
    token=engine.TestSession();node=token?engine.TestNode():0;reason=engine.Reason();
    capable=engine.CapabilityMask()!=0;active=engine.Active();closing=engine.TestClosing();failed=engine.TestCloseFailed();opened=engine.TestOpened();
    taskEXIT_CRITICAL();
    const char* stage=!token?"IDLE":failed?"FAULT":closing?"CLOSING":active?(capable?"OPENING":"CAPS"):"DONE";
    Respond(response,"STREAM_TEST s=%u node=%u st=%s open=%u closed=%u code=%u",unsigned(token),unsigned(node),stage,unsigned(opened),unsigned(token&&!active),unsigned(token?reason:0));
}
}

bool HandleCoreStreamCommand(const char* command,size_t length,StreamSink& response) {
    while(length && (*command==' ' || *command=='\t')) { ++command; --length; }
    while(length && (command[length-1]==' ' || command[length-1]=='\t' || command[length-1]=='\r' || command[length-1]=='\n')) --length;
    const bool test=Prefix(command,length,"!STREAM_TEST ");
    const bool testQuery=Equal(command,length,"#STREAM_TEST");
    if(test || testQuery) {
        if(response.channelType!=StreamSink::CHANNEL_TYPE_USB) { Respond(response,"error STREAM_TEST USB_only");return true; }
        if(testQuery) { PrintOpenTest(response);return true; }
        uint16_t values[2]{};
        if(!Numbers(command+13,length-13,values,2) || !values[0] || values[1]<1 || values[1]>7) { Respond(response,"error STREAM_TEST syntax");return true; }
        taskENTER_CRITICAL();
        uint8_t result=BUSY;
        const char* gate=engine.Active()?"SESSION_BUSY":!dummy.IsEnabled()?"DISABLED":
            dummy.IsStopLocked()?"STOP_LOCKED":dummy.IsCalibrationBusy()?"CALIBRATION":
            dummy.IsProtectionParameterBusy()?"PARAMETER":dummy.commandHandler.IsBusy()?"COMMAND_BUSY":
            (dummy.IsMoving() || dummy.IsMotionBusy())?"MOVING":nullptr;
        if(gate && strcmp(gate,"SESSION_BUSY") && strcmp(gate,"COMMAND_BUSY") && strcmp(gate,"MOVING"))result=BLOCKED;
        if(!gate) {
            openTestRequest=true;
            result=engine.Begin(values[0],uint8_t(1U<<(values[1]-1)),HAL_GetTick(),true);
            if(result!=OK)openTestRequest=false;
        }
        if(result==OK)sink=&response;
        taskEXIT_CRITICAL();
        if(result!=OK)Respond(response,"error STREAM_TEST code=%u gate=%s",unsigned(result),gate?gate:"BEGIN");
        else PrintOpenTest(response);
        return true;
    }
    const bool packet=Prefix(command,length,"~S");
    const bool axesQuery=Equal(command,length,"#STREAM_AXES");
    const bool query=Equal(command,length,"#STREAM");
    const bool diagnostic=Prefix(command,length,"#STREAM_DIAG ");
    const bool open=Prefix(command,length,"!STREAM_OPEN ");
    const bool start=Prefix(command,length,"!STREAM_START ");
    const bool close=Prefix(command,length,"!STREAM_CLOSE ");
    if(!packet && !axesQuery && !query && !diagnostic && !open && !start && !close && !Prefix(command,length,"!STREAM")) return false;
    if(response.channelType!=StreamSink::CHANNEL_TYPE_USB) { Respond(response,"error STREAM USB_only"); return true; }
    if(axesQuery) {
        Respond(response,"STREAM_AXES v=1 hz=50 mask=127 targeted=1");return true;
    }
    if(diagnostic) {
        uint16_t values[2]{};
        if(!Numbers(command+13,length-13,values,2) || values[0]<1 || values[0]>7 || values[1]>DIAG_LAST) {
            Respond(response,"error STREAM_DIAG range node=1..7 field=0..50"); return true;
        }
        taskENTER_CRITICAL();
        if(diagnosticPending || diagnosticReady) { taskEXIT_CRITICAL(); Respond(response,"error STREAM_DIAG busy"); return true; }
        if(!++diagnosticNonce) ++diagnosticNonce;
        diagnosticNode=uint8_t(values[0]); diagnosticField=uint8_t(values[1]); diagnosticSink=&response;
        diagnosticPending=true;diagnosticAt=HAL_GetTick();diagnosticAttemptAt=diagnosticAt-20;
        taskEXIT_CRITICAL();
        // The 1kHz task owns retries. A busy mailbox is neither an error nor an ACK.
        return true;
    }
    if(query) {
        taskENTER_CRITICAL(); engine.Heartbeat(HAL_GetTick()); taskEXIT_CRITICAL();
        const Snapshot s=ReadSnapshot();
        Respond(response,"STREAM_CAP v=%u hz=50 mode=%u mask=%u",unsigned(VERSION),unsigned(MODE),unsigned(s.mask));
        PrintFeatures(response,s);
        Status(response,s);
        Respond(response,"STREAM_DIAG c=%u e=%u gap=%lu us=%lu",unsigned(s.committed),unsigned(s.executed),
            (unsigned long)s.gap,(unsigned long)maxPollUs);
        if(s.state==CoreStreamEngine::FAULT) PrintFailure(response,s);
        return true;
    }
    if(packet) {
        if(engine.IsOpenTest() && engine.Active()) { Respond(response,"error STREAM code=%u",unsigned(BAD_STATE));return true; }
        Record record{};
        if(!DecodeLine(command,length,record)) {
            taskENTER_CRITICAL(); engine.ProtocolFault(BAD_CRC); taskEXIT_CRITICAL();
            Respond(response,"error STREAM code=%u",unsigned(BAD_CRC)); return true;
        }
        // Double-precision hull validation stays outside the interrupt mask.
        // Only this USB task produces records; Poll consumes but never changes
        // the last accepted endpoint. Submit rechecks session/state afterwards.
        Record previous{}; bool havePrevious; uint8_t selected;
        taskENTER_CRITICAL();
        havePrevious=engine.Previous()!=nullptr;
        if(havePrevious) previous=*engine.Previous();
        selected=engine.SelectedMask();
        taskEXIT_CRITICAL();
        const bool valid=streamPort.Validate(record,havePrevious ? &previous : nullptr,selected);
        taskENTER_CRITICAL();
        const uint8_t result=valid ? engine.Submit(record,HAL_GetTick(),true) : uint8_t(BAD_LIMIT);
        if(!valid) engine.ProtocolFault(BAD_LIMIT);
        const unsigned free=engine.Free();
        taskEXIT_CRITICAL();
        if(result==OK) Respond(response,"STREAM_ACK s=%u n=%u free=%u",unsigned(record.session),unsigned(record.sequence),free);
        else Respond(response,"error STREAM s=%u n=%u code=%u",unsigned(record.session),unsigned(record.sequence),unsigned(result));
        return true;
    }
    uint16_t values[2]{};
    const size_t offset=open ? 13 : start ? 14 : 14;
    if((!open && !start && !close) || !Numbers(command+offset,length-offset,values,open ? 2 : 1) ||
        (open && values[1]>127)) { Respond(response,"error STREAM syntax"); return true; }
    taskENTER_CRITICAL();
    uint8_t result;
    if(open) { result=engine.Begin(values[0],uint8_t(values[1]),HAL_GetTick()); if(result==OK) sink=&response; }
    else if(start) result=engine.Start(values[0],HAL_GetTick());
    else result=engine.Close(values[0],HAL_GetTick());
    taskEXIT_CRITICAL();
    if(result!=OK) Respond(response,"error STREAM s=%u code=%u",unsigned(values[0]),unsigned(result));
    else Status(response,ReadSnapshot());
    return true;
}
bool CoreStreamAuxAllowed(const char* command,size_t length) {
    if(!CoreStreamActive()) return true;
    while(length && (*command==' ' || *command=='\t')) { ++command; --length; }
    while(length && (command[length-1]==' ' || command[length-1]=='\t' || command[length-1]=='\r' || command[length-1]=='\n')) --length;
    if(engine.IsOpenTest()) return Equal(command,length,"!STOP") || Equal(command,length,"!DISABLE") ||
        Equal(command,length,"#GETJPOS") || Prefix(command,length,"#GET_CMDMODE ") ||
        Equal(command,length,"#STACK") || Equal(command,length,"#MOTION_STATUS") ||
        Equal(command,length,"#RGB_STATUS") || Prefix(command,length,"#FW_VERSION ") ||
        Prefix(command,length,"#CAN_DIAG ") || Prefix(command,length,"#STALL_STATUS ");
    return Equal(command,length,"!STOP") || Equal(command,length,"!DISABLE") || Equal(command,length,"!START") ||
        Equal(command,length,"#GETJPOS") || Equal(command,length,"#GETRAILPOS") || Equal(command,length,"#STACK") ||
        Prefix(command,length,"#GET_CMDMODE") || Equal(command,length,"#MOTION_STATUS") ||
        Equal(command,length,"#RGB_STATUS") || Prefix(command,length,"#FW_VERSION") ||
        Prefix(command,length,"#CAN_DIAG") || Prefix(command,length,"#STALL_STATUS") ||
        Prefix(command,length,"!HAND_O") || Prefix(command,length,"!HAND_C") ||
        Prefix(command,length,"!HAND_POS") || Prefix(command,length,"!HAND_I");
}
bool CoreStreamBusy() { return engine.Busy(); }
bool CoreStreamActive() { return engine.Active(); }
bool CoreStreamCanTransmit(uint16_t session) { return engine.CanTransmit(session); }
bool CoreStreamAllowsCan(uint8_t node,uint8_t command) {
    if(!engine.Active() || node==8) return true;
    // Native/fibre callers cannot silently replace a streamed reference or
    // persist altered motor parameters while a timed session owns J1..J7.
    if(command<0x20 && command!=SoftStopProtocol::COMMAND) return false;
    if(command==0x7F) return false;
    return true;
}
void CoreStreamCancel() { taskENTER_CRITICAL(); engine.Cancel(); taskEXIT_CRITICAL(); }
void CoreStreamPoll() {
    const uint32_t began=micros();
    PollCanMotionTransmit(get_can_ctx(&hcan1));
    PollCanStreamTransmit(get_can_ctx(&hcan1));
    bool stop;
    taskENTER_CRITICAL();
    engine.Poll(HAL_GetTick());stop=stopRequested;stopRequested=false;
    if(!engine.Active())openTestRequest=false;
    if(diagnosticPending && HAL_GetTick()-diagnosticAttemptAt>=20 && HAL_GetTick()-diagnosticAt<1000) {
        uint8_t data[8]{};Write16(data,diagnosticNonce);data[2]=diagnosticField;
        if(streamPort.Send(diagnosticNode,DIAG,data)) diagnosticAttemptAt=HAL_GetTick();
    }
    taskEXIT_CRITICAL();
    // Queue cancellation/soft-stop bookkeeping may touch RTOS queues. Never
    // keep the coordinator's outer CAN interrupt mask around that work.
    if(stop) dummy.RequestSoftStop();
    PollCanStreamTransmit(get_can_ctx(&hcan1));
    const uint32_t duration=micros()-began;if(duration>maxPollUs)maxPollUs=duration;
}
void CoreStreamTransportFault(uint16_t session,uint8_t reason,uint8_t node,uint8_t command,uint16_t sequence,uint32_t age) {
    // Called under the transport's task/ISR-safe mask; no RTOS I/O here.
    engine.TransportFailure(session,reason,node,command,sequence,age,HAL_GetTick());
}

void CoreStreamPrint() {
    bool ready,timeout; uint16_t nonce; uint8_t node,field; uint32_t value;
    taskENTER_CRITICAL();
    ready=diagnosticReady; timeout=diagnosticPending && HAL_GetTick()-diagnosticAt>=1000;
    nonce=diagnosticNonce; node=diagnosticNode; field=diagnosticField; value=diagnosticValue;
    if(ready || timeout) { diagnosticPending=false; diagnosticReady=false; }
    taskEXIT_CRITICAL();
    if(diagnosticSink && ready) {
        if(field==0 || field==7 || (field>=26 && field<=31) || (field>=37 && field<=42) || field==47 || field==50) Respond(*diagnosticSink,"STREAM_NODE node=%u n=%u field=%u value=%ld",unsigned(node),unsigned(nonce),unsigned(field),(long)int32_t(value));
        else Respond(*diagnosticSink,"STREAM_NODE node=%u n=%u field=%u value=%lu",unsigned(node),unsigned(nonce),unsigned(field),(unsigned long)value);
    } else if(diagnosticSink && timeout) Respond(*diagnosticSink,"error STREAM_DIAG node=%u timeout",unsigned(node));
    const Snapshot s=ReadSnapshot();
    if(!sink || s.revision==printedRevision) return;
    printedRevision=s.revision;
    if(engine.IsOpenTest()) { PrintOpenTest(*sink);return; }
    PrintFeatures(*sink,s);
    if(s.state==CoreStreamEngine::FAULT) { Respond(*sink,"STREAM_FAULT s=%u code=%u",unsigned(s.session),unsigned(s.reason)); PrintFailure(*sink,s); }
    else if(s.state==CoreStreamEngine::IDLE && s.session) Respond(*sink,"STREAM_DONE s=%u n=%u",unsigned(s.session),unsigned(s.finalSeq));
    else Status(*sink,s);
}
void CoreStreamReceive(uint8_t node,uint8_t command,const uint8_t data[8]) {
    if(command==DIAG) {
        if(diagnosticPending && node==diagnosticNode && Read16(data)==diagnosticNonce &&
            data[2]==diagnosticField && !data[3]) {
            diagnosticValue=Read32(data+4); diagnosticPending=false; diagnosticReady=true;
            // Keep the core's range validator on the same measured acceleration
            // used by the PC, even if its older startup 0x2C query was lost.
            if(diagnosticField==6 && diagnosticValue>0 && diagnosticValue<=153600000UL)
                dummy.MotorAccelerationCallback(node,float(diagnosticValue)/51200.0f,0);
        }
        return;
    }
    engine.Receive(node,command,data,HAL_GetTick());
}
