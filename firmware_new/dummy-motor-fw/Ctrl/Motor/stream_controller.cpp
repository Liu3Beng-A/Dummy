#include "motor.h"
#include "stream_runtime.h"
#include "configurations.h"
#include <can.h>
#include <cstring>

extern "C" uint32_t _estack;
extern bool MotorStreamCalibrationBusy();

namespace {
using namespace StreamProtocol;
MotorStream::Runtime stream;
// CAN interrupt only copies packets. Software-double curve preparation and
// CRC validation are preemptible main-loop operations, never 20kHz/ CAN ISR.
struct Packet { uint32_t tick; uint8_t command, bytes[8]; };
Packet packets[24];
volatile uint8_t packetRead, packetWrite;
volatile bool packetOverflow, stopDispatching;
uint32_t generation;
volatile uint32_t maxStreamTickCycles, maxPreparationCycles, minStackHeadroom=4096, rxOverflowCount;
uint32_t maxRxAgeTicks, maxStartAgeTicks;
// Fields23..50 are contiguous: OPEN11, START11, OPEN evidence3, START evidence3.
uint32_t entrySnapshot[28]{};
static_assert(int64_t(MotionCompletionProtocol::STABLE_SPEED_STEPS) * MotionCompletionProtocol::STABLE_TICKS <= INT32_MAX,
              "Published evidence sum must fit the cold signed diagnostic division");

void SampleStack() {
    const uint32_t floor=static_cast<uint32_t>(reinterpret_cast<uintptr_t>(&_estack))-4096U;
    const uint32_t msp=__get_MSP();
    const uint32_t remaining=msp>floor ? msp-floor : 0;
    const uint32_t mask=__get_PRIMASK();__disable_irq();
    if (remaining<minStackHeadroom) minStackHeadroom=remaining;
    __set_PRIMASK(mask);
}
struct Guard {
    uint32_t mask;
    Guard():mask(__get_PRIMASK()) { __disable_irq(); }
    ~Guard() { __set_PRIMASK(mask); }
};
bool Reply(uint8_t command,const uint8_t* bytes) {
    CAN_TxHeaderTypeDef header{};
    header.StdId=(boardConfig.canNodeId<<7)|command;
    header.IDE=CAN_ID_STD; header.RTR=CAN_RTR_DATA; header.DLC=8;
    return CAN_QueueStreamReply(&header,bytes); // Owned until TXOK; bounded NART retries.
}
bool Status(uint16_t sequence,uint8_t state,uint8_t reason=OK,uint16_t replySession=0) {
    uint8_t bytes[8];
    { Guard guard;
      Write16(bytes,replySession ? replySession : stream.session); Write16(bytes+2,sequence);
      bytes[4]=state;bytes[5]=stream.Free();bytes[6]=reason;bytes[7]=VERSION; }
    return Reply(STATUS,bytes);
}
bool Active() {
    return stream.state!=STATE_IDLE && stream.state!=STATE_FAULT && stream.state!=STATE_COMPLETED;
}
void ServiceStop(Motor::Controller& controller) {
    { Guard guard;
      if (!stream.stopRequired || stopDispatching) return;
      stopDispatching=true;stream.stopRequired=false;
    }
    // One claimant only. Keep motion/release gates closed without masking the
    // entire soft-stop request against the 20kHz interrupt.
    controller.RequestSoftStop(SoftStopProtocol::STOP,static_cast<uint16_t>(controller.GetSoftStopToken()+1));
    { Guard guard;
      if (!controller.IsSoftStopLocked()) stream.stopRequired=true;
      stopDispatching=false;
    }
}
}

// Only main calls this bounded snapshot. The decision uses these same values;
// retries of a rejected session cannot overwrite the first refusal evidence.
#if defined(__GNUC__)
__attribute__((optimize("Os")))
#endif
uint32_t Motor::Controller::StreamClock() const { return stream.ticks; }

#if defined(__GNUC__)
__attribute__((optimize("Os")))
#endif
uint32_t Motor::Controller::PositionEntryEvidenceAge() const {
    if (!positionEvidenceValid || positionCompletionTarget!=goalPosition ||
        positionCompletionHome!=context->config.motionParams.encoderHomeOffset ||
        positionCompletionMode!=uint8_t(modeRunning) || requestMode!=modeRunning)
        return UINT32_MAX;
    return stream.ticks-positionEvidenceTick;
}

#if defined(__GNUC__)
__attribute__((optimize("Os")))
#endif
uint32_t Motor::Controller::CaptureStreamOpen(uint16_t token,bool start) {
    Guard guard;
    uint32_t* snapshot=entrySnapshot+(start ? 11 : 0);
    if(snapshot[0]==token) return snapshot[1];
    // A new OPEN owns the next START evidence, even if its16bit token matches
    // a much older START after bounded retirement history has rolled over.
    if(!start) entrySnapshot[11]=0;
    const bool settled=positionSettled;
    const uint32_t age=PositionEntryEvidenceAge();
    uint32_t bits=0;
    bits|=uint32_t(!start && Active())<<0; bits|=uint32_t(stream.stopRequired)<<1;
    bits|=uint32_t(IsSoftStopLocked())<<2; bits|=uint32_t(!IsFeedbackValid())<<3;
    bits|=uint32_t(goalDisable)<<4; bits|=uint32_t(softDisable)<<5;
    bits|=uint32_t(goalBrake)<<6; bits|=uint32_t(softBrake)<<7;
    bits|=uint32_t(isStalled)<<8; bits|=uint32_t(calibrationTask)<<9;
    bits|=uint32_t(MotorStreamCalibrationBusy())<<10;
    bits|=uint32_t(protectionPhase!=ProtectionProtocol::IDLE)<<11;
    bits|=uint32_t(age>MotionCompletionProtocol::STABLE_TICKS)<<12;
    bits|=uint32_t(softVelocity!=0)<<13;
    bits|=uint32_t(requestMode!=modeRunning)<<14;
    bits|=uint32_t(softPosition!=goalPosition)<<15;
    const int64_t error=int64_t(goalPosition)-realPosition;
    const int32_t tolerance=MotionCompletionProtocol::PositionToleranceSteps(boardConfig.canNodeId);
    bits|=uint32_t(error < -tolerance || error > tolerance)<<16;
    const int64_t held=int64_t(goalPosition)-context->config.motionParams.encoderHomeOffset;
    bits|=uint32_t(held<INT32_MIN || held>INT32_MAX)<<17;
    bits|=uint32_t(modeRunning!=MODE_COMMAND_POSITION && modeRunning!=MODE_STREAM_JOINT)<<18;
    bits|=uint32_t(start && (int64_t(goalPosition)!=int64_t(stream.position)+stream.home ||
                          context->config.motionParams.encoderHomeOffset!=stream.home))<<19;
    // A hard condition observed between control ticks also retires the old
    // certificate; recovery must accumulate a complete new window.
    if(bits & ~(1U<<0)) InvalidatePositionEntry();
    snapshot[0]=token; snapshot[1]=bits;
    snapshot[2]=uint32_t(modeRunning)|(uint32_t(requestMode)<<8)|(uint32_t(protectionPhase)<<16);
    snapshot[3]=uint32_t(estVelocity); snapshot[4]=uint32_t(softVelocity);
    snapshot[5]=uint32_t(goalPosition); snapshot[6]=uint32_t(softPosition);
    snapshot[7]=uint32_t(realPosition); snapshot[8]=uint32_t(context->config.motionParams.encoderHomeOffset);
    snapshot[9]=stream.ticks; snapshot[10]=uint32_t(settled);
    uint32_t* evidence=entrySnapshot+(start ? 25 : 22);
    evidence[0]=age;
    evidence[1]=age==UINT32_MAX ? UINT32_MAX : positionEvidenceRange;
    evidence[2]=age==UINT32_MAX ? 0 : uint32_t(int32_t(positionEvidenceVelocitySum) / int32_t(MotionCompletionProtocol::STABLE_TICKS));
    return bits;
}

bool Motor::Controller::StreamActive() const { return Active() || stream.stopRequired || stopDispatching; }
bool Motor::Controller::StreamTerminal() const { return stream.terminal && stream.armed; }

// STOP includes all locked ramp/hold/disabled/fault states. MODE_STOP at boot
// alone is not a stream STOP sample. Entry also includes END settlement and
// request/actual-mode handover; RUNNING requires the actual executing runtime.
uint8_t Motor::Controller::StreamTimingPhase() const {
    if (IsSoftStopLocked()) return 3;
    if (modeRunning!=MODE_STREAM_JOINT && requestMode!=MODE_STREAM_JOINT) return 0;
    return stream.armed && stream.state==StreamProtocol::STATE_RUNNING && !stream.terminal ? 2 : 1;
}

// Only one in every64 control ticks calls this cold recorder. It uses no
// division/allocation; recorder overhead is included in whole callback timing.
#if defined(__GNUC__)
__attribute__((noinline,optimize("Os")))
#endif
void MotorTimingCheckpoint(uint8_t segment) {
    const uint32_t now=DWT->CYCCNT;
    const uint32_t elapsed=now-motorTimingCheckpointCycle;
    motorTimingCheckpointCycle=now;
    if (elapsed>motorStreamTiming[segment]) motorStreamTiming[segment]=elapsed;
}

void Motor::Controller::CancelStream(uint8_t reason) {
    Guard guard;
    ++generation; CAN_CancelStreamReplies();
    packetRead=packetWrite; packetOverflow=false;
    const bool pendingStop=stream.stopRequired;
    stream.Clear(stream.state==StreamProtocol::STATE_FAULT ? stream.reason : reason);
    stream.stopRequired=pendingStop; // Cancellation never revokes a fault before STOP latches.
}

bool Motor::Controller::ProcessStreamCan(uint8_t command,const uint8_t* data,uint32_t length) {
    if (command<OPEN || command>DIAG) return false;
    if (command==STATUS) return true; // Never accept a motor reply as a request.
    if (length!=8) return true;
    OnMasterCommunication();
    Guard guard;
    const uint8_t next=(packetWrite+1)%24;
    if (next==packetRead) { packetOverflow=true;++rxOverflowCount; return true; }
    packets[packetWrite].tick=stream.ticks;
    packets[packetWrite].command=command;
    memcpy(packets[packetWrite].bytes,data,8);
    packetWrite=next;
    return true;
}

// Always tick the stream clock, including encoder-calibration branches.
void Motor::Controller::TickStreamClock() { ++stream.ticks; }

void Motor::Controller::TickStream() {
    SampleStack();
    if (packetOverflow) stream.Fault(RX_OVERFLOW);
    if (CAN_StreamReplyFault()) stream.Fault(DEADLINE);
    if (!stream.armed && !stream.stopRequired) return;
    if (stream.home!=context->config.motionParams.encoderHomeOffset ||
        stream.velocityLimit!=boardConfig.velocityLimit ||
        stream.accelerationLimit!=boardConfig.velocityAcc || !IsFeedbackValid() ||
        IsSoftStopLocked() || goalDisable || softDisable || goalBrake || softBrake ||
        isStalled || calibrationTask || protectionPhase!=ProtectionProtocol::IDLE)
        stream.Fault(BLOCKED);
    const uint32_t began=DWT->CYCCNT;
    stream.Tick();
    const uint32_t elapsed=DWT->CYCCNT-began;
    if (elapsed>maxStreamTickCycles) maxStreamTickCycles=elapsed;
    if (stream.stopRequired) {
        ServiceStop(*this);
        return;
    }
    if (stream.state==STATE_FAULT) return;
    // Curves were bounded together with this immutable home offset in main.
    softPosition=goalPosition=stream.position+stream.home;
    softVelocity=stream.velocity;
    if (!stream.terminal && !motionIntent) { ResetDetection(true);motionIntent=true; }
}

#if defined(__GNUC__)
__attribute__((optimize("Os")))
#endif
void Motor::Controller::PollStreamMain() {
    Packet packet{};
    bool havePacket=false;
    uint32_t epoch;
    { Guard guard;
      epoch=generation;
      if (packetRead!=packetWrite) {
          packet=packets[packetRead];packetRead=(packetRead+1)%24;havePacket=true;
      }
      if (packetOverflow && Active()) stream.Fault(RX_OVERFLOW);
      if (CAN_StreamReplyFault() && Active()) stream.Fault(DEADLINE);
    }
    if (havePacket) {
        const uint32_t age=stream.ticks-packet.tick;
        if (age>maxRxAgeTicks) maxRxAgeTicks=age;
        if (packet.command==START && age>maxStartAgeTicks) maxStartAgeTicks=age;
        const auto* b=packet.bytes;
        const uint16_t ss=Read16(b),sequence=Read16(b+2);
        const uint8_t command=packet.command;
        if (command==DIAG) {
            uint32_t value=0;
            switch(b[2]) {
                case 0:value=static_cast<uint32_t>(stream.phaseError);break;
                case 1:value=maxStreamTickCycles;break;
                case 2:value=maxPreparationCycles;break;
                case 3:value=minStackHeadroom;break;
                case 4:value=rxOverflowCount;break;
                case 5:value=static_cast<uint32_t>(boardConfig.velocityLimit);break;
                case 6:value=static_cast<uint32_t>(boardConfig.velocityAcc);break;
                case 7:value=static_cast<uint32_t>(int64_t(goalPosition)-context->config.motionParams.encoderHomeOffset);break;
                case 17:value=CAN_StreamReplyDiagnostic(0);break;
                case 18:value=CAN_StreamReplyDiagnostic(1);break;
                case 19:value=maxRxAgeTicks;break;
                case 20:value=maxStartAgeTicks;break;
                case 21:value=motorControlTiming[9];break;
                case 22:value=motorControlTiming[10];break;
                case 51:value=0x544d0001U;break;
                case 63:value=64;break;
                default:if(b[2]>=8 && b[2]<=16)value=motorControlTiming[b[2]-8];else if(b[2]>=DIAG_OPEN_SESSION && b[2]<=DIAG_START_EVIDENCE_MEAN)value=entrySnapshot[b[2]-DIAG_OPEN_SESSION];else if(b[2]>=52 && b[2]<=62)value=motorStreamTiming[b[2]-52];else return;
            }
            uint8_t reply[8]={};Write16(reply,ss);reply[2]=b[2];Write32(reply+4,value);Reply(DIAG,reply);
        } else if (command==CAPS) {
            uint8_t reply[8]={};Write16(reply,ss);reply[2]=VERSION;
            reply[3]=(boardConfig.canNodeId>=1 && boardConfig.canNodeId<=7 && MOTOR_TYPE!=8) ? (CAP_TIMED|CAP_OPEN_SNAPSHOT|CAP_STABLE_ENTRY):0;
            Write16(reply+4,SEGMENT_TICKS);reply[6]=CAPACITY;reply[7]=MOTOR_TYPE;
            Reply(CAPS,reply);
        } else if (command==OPEN) {
            if (Active() && ss!=stream.session) {
                // Old OPEN cannot fault or replace the healthy current session.
            } else if (!Active() && (stream.Retired(ss) || (ss && ss==stream.session))) {
                Status(0,STATE_FAULT,BAD_SESSION,ss);
            } else if (!ss || b[2]!=VERSION || !ValidMask(b[3]) || Read32(b+4)) {
                Status(sequence,STATE_FAULT,BAD_VERSION,ss);
            } else if (MOTOR_TYPE==8 || boardConfig.canNodeId<1 || boardConfig.canNodeId>7 || !(b[3]&(1U<<(boardConfig.canNodeId-1)))) {
                // Broadcast masks intentionally exclude the gripper/other nodes.
            } else if (ss==stream.session && Active()) {
                Status(0,STATE_OPEN);
            } else {
                uint32_t blocks;
                { Guard guard;
                  if (generation!=epoch) return;
                  blocks=CaptureStreamOpen(ss);
                  if(!blocks) {
                    stream.Clear();CAN_CancelStreamReplies();stream.session=ss;stream.mask=b[3];
                    stream.home=context->config.motionParams.encoderHomeOffset;
                    stream.position=goalPosition-stream.home; // v2 seed is the already-held reference.
                    stream.velocityLimit=boardConfig.velocityLimit;
                    stream.accelerationLimit=boardConfig.velocityAcc;
                    stream.state=STATE_OPEN;
                  }
                }
                Status(0,blocks?STATE_FAULT:STATE_OPEN,blocks?BLOCKED:OK,ss);
            }
        } else if (command==ABORT && b[2]==0 && !Active() && !stream.stopRequired &&
                   ss==entrySnapshot[0] && ss!=stream.session) {
            Status(0,STATE_IDLE,OK,ss);
        } else if (command==ABORT && ss==stream.session) {
            uint8_t replyState=STATE_IDLE,replyReason=OK;
            { Guard guard;
              if (stream.stopRequired) { replyState=STATE_FAULT;replyReason=stream.reason; }
              else if (b[2]==0 && (!stream.armed || stream.state==STATE_COMPLETED || stream.state==STATE_FAULT || stream.state==STATE_IDLE)) {
                  stream.Clear();++generation;CAN_CancelStreamReplies();packetRead=packetWrite;
                  if (modeRunning==MODE_STREAM_JOINT && !IsSoftStopLocked()) requestMode=MODE_COMMAND_POSITION;
              } else { stream.Fault(ABORTED);replyState=STATE_FAULT;replyReason=ABORTED; }
            }
            Status(stream.completed,replyState,replyReason);
        } else if (ss!=stream.session) {
            // Ignore stale-session packets; never label their error with current identity.
        } else if (!Active()) {
            // Delayed packets from a cancelled/old session cannot move the motor.
            if (ss==stream.session && stream.state==STATE_COMPLETED) Status(stream.completed,STATE_COMPLETED);
            else Status(sequence,STATE_FAULT,BAD_SESSION);
        } else if (command==ABORT) {
            { Guard guard; stream.Fault(ABORTED); }
        } else if (stream.seeded && sequence < stream.prepared &&
                   (command==Q || command==V || command==A || command==PREPARE || command==COMMIT)) {
            // A CAN retransmission can lose arbitration to the next endpoint.
            // Never let an already superseded frame destroy the new staging.
        } else if (command==Q || command==V || command==A) {
            if (stream.seeded && sequence==stream.prepared) return;
            if (sequence!=stream.stagedSequence) {
                stream.stagedSequence=sequence;stream.stagedMask=0;
            }
            const uint8_t bit=static_cast<uint8_t>(1U<<(command-Q));
            if (command==Q) stream.staged.q=ReadI32(b+4);
            else if (command==V) stream.staged.v=ReadI32(b+4);
            else stream.staged.a=ReadI32(b+4);
            stream.stagedMask|=bit;
        } else if (command==PREPARE || command==COMMIT) {
            const uint16_t crc=Read16(b+4);const uint8_t flags=b[6];
            uint8_t error=OK;
            if (!ValidFlags(sequence,flags) || b[7]) error=BAD_FLAGS;
            if (command==COMMIT && !error) {
                Guard guard;
                if (generation!=epoch || !stream.seeded || sequence!=stream.prepared || crc!=stream.preparedCrc || flags!=stream.preparedFlags)
                    error=BAD_SEQUENCE;
                else if (!sequence) {
                    stream.seedCommitted=true;stream.committedCrc=crc;
                } else {
                    auto& slot=stream.slots[sequence%CAPACITY];
                    if (stream.committed==sequence && stream.committedCrc==crc) {}
                    else if (slot.sequence!=sequence || slot.state!=1 || slot.crc!=crc || sequence!=stream.committed+1)
                        error=BAD_SEQUENCE;
                    else { slot.state=2;stream.committed=sequence;stream.committedCrc=crc; }
                }
            } else if (!error) {
                const MotorStream::Point point=stream.staged;
                if (stream.seeded && sequence==stream.prepared) {
                    if (crc!=stream.preparedCrc || flags!=stream.preparedFlags) error=BAD_CRC;
                }
                else if (stream.stagedMask!=7 || stream.stagedSequence!=sequence) error=INCOMPLETE;
                else if (CrcEndpoint(ss,sequence,flags,point.q,point.v,point.a)!=crc) error=BAD_CRC;
                else if ((flags&(SEED|END)) && (point.v || point.a)) error=BAD_LIMIT;
                else if (!sequence) {
                    if (stream.seeded || point.q!=stream.position ||
                        int64_t(point.q)+stream.home < INT32_MIN || int64_t(point.q)+stream.home > INT32_MAX) error=BAD_SEED;
                    else { Guard guard;
                        if (generation!=epoch) return;
                        stream.seeded=true;stream.seed=stream.previous=point;
                        stream.position=point.q;stream.velocity=0;
                        stream.prepared=0;stream.preparedCrc=crc;stream.preparedFlags=flags;
                    }
                } else if (!stream.seedCommitted || sequence!=stream.prepared+1 ||
                           (stream.preparedFlags&END)) error=BAD_SEQUENCE;
                else if (stream.slots[sequence%CAPACITY].state) error=BUSY;
                else {
                    MotorStream::Curve curve;
                    SampleStack();
                    const uint32_t began=DWT->CYCCNT;
                    const bool built=curve.Build(stream.previous,point,stream.velocityLimit,stream.accelerationLimit,stream.home);
                    const uint32_t elapsed=DWT->CYCCNT-began;
                    if (elapsed>maxPreparationCycles) maxPreparationCycles=elapsed;
                    if (!built) error=BAD_LIMIT;
                    else { Guard guard;
                        if (generation!=epoch || !Active()) error=BAD_SESSION;
                        else {
                            auto& slot=stream.slots[sequence%CAPACITY];
                            slot.curve=curve;slot.sequence=sequence;slot.crc=crc;slot.flags=flags;slot.state=1;
                            stream.previous=point;stream.prepared=sequence;
                            stream.preparedCrc=crc;stream.preparedFlags=flags;
                        }
                    }
                }
            }
            Status(sequence,error?STATE_FAULT:(command==PREPARE?STATE_PREPARED:STATE_COMMITTED),error);
            if (error && error!=BUSY) { Guard guard; stream.Fault(error); }
        } else if (command==START) {
            const uint16_t delay=Read16(b+4);
            uint8_t error=OK;
            { Guard guard;
              if (generation!=epoch || !Active()) error=BAD_SESSION;
              else {
              // Capture the first matching, unarmed decision, including syntax
              // or buffer rejection; an armed duplicate cannot overwrite it.
              const uint32_t blocks=stream.armed ? 0 : CaptureStreamOpen(ss,true);
              if (sequence!=1 || (delay<START_GUARD_TICKS || delay>START_DELAY_TICKS) || b[6] || b[7]) error=BAD_STATE;
              else if (stream.armed) {} // Duplicate START must never restart its clock.
              else if (blocks) error=BAD_SEED;
              else if (!stream.seedCommitted || !stream.committed ||
                       (stream.committed<PREFILL && !(stream.preparedFlags&END))) error=INCOMPLETE;
              else if (static_cast<uint32_t>(stream.ticks-packet.tick)>=delay-START_GUARD_TICKS) error=DEADLINE;
              else {
                  stream.anchor=packet.tick+delay;stream.lastSyncTick=stream.anchor;
                  stream.armed=true;stream.state=STATE_STARTED;stream.changed=true;
                  softPosition=goalPosition=stream.position+stream.home;softVelocity=0;
                  requestMode=MODE_STREAM_JOINT;ResetPositionCompletion(true);state=STATE_RUNNING;
              }
              }
            }
            Status(1,error?STATE_FAULT:STATE_STARTED,error);
            if (error) { Guard guard;stream.Fault(error); }
        } else if (command==SYNC) {
            Guard guard;
            if (generation==epoch && stream.armed && (!stream.syncSeen || static_cast<int16_t>(sequence-stream.syncCounter)>0)) {
                stream.syncSeen=true;stream.syncCounter=sequence;
                stream.lastSyncTick=packet.tick;
                const int32_t local=static_cast<int32_t>(packet.tick-stream.anchor);
                if (local>=0) {
                    const int64_t difference=int64_t(local)-Read32(b+4);
                    stream.phaseError=difference < INT32_MIN ? INT32_MIN : difference > INT32_MAX ? INT32_MAX : static_cast<int32_t>(difference);
                    if (difference < -MAX_PHASE_ERROR || difference > MAX_PHASE_ERROR) stream.Fault(SYNC_DRIFT);
                }
                stream.changed=true;
            }
        }
    }
    bool report=false,needStop=false;uint16_t sequence=0;uint8_t stateValue=0,reasonValue=0;
    { Guard guard;
      if (stream.state!=STATE_COMPLETED && stream.terminal && positionSettled && modeRunning==MODE_STREAM_JOINT && !IsSoftStopLocked()) {
          stream.state=STATE_COMPLETED;stream.settled=true;stream.changed=true;
      }
      needStop=stream.stopRequired;
      if (stream.changed) {
          stream.changed=false;sequence=stream.state==STATE_STARTED ? 1 : stream.completed;stateValue=stream.state;
          reasonValue=stream.reason;report=true;
      }
    }
    if (needStop) ServiceStop(*this);
    if (report && !Status(sequence,stateValue,reasonValue)) { Guard guard;stream.changed=true; }
}
