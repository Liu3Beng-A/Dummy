#pragma once
#include <stdint.h>
#include "../../../stream_protocol.h"

// Portable transaction coordinator. The adapter serializes callers using short
// critical sections; no allocation, blocking calls or printing in Poll/Receive.
class CoreStreamPort {
public:
    virtual bool Send(uint8_t node, uint8_t command, const uint8_t data[8]) = 0;
    virtual bool Allowed() const = 0;
    virtual bool Validate(const StreamProtocol::Record& record,
                          const StreamProtocol::Record* previous, uint8_t mask) const = 0;
    virtual void Stop() = 0;
};

class CoreStreamEngine {
public:
    enum State : uint8_t { IDLE, PROBING, OPEN, BUFFERING, ARMED, RUNNING, DRAINING, FAULT };
    explicit CoreStreamEngine(CoreStreamPort& io) : port(io) {}
    uint8_t Begin(uint16_t session, uint8_t mask, uint32_t now, bool openTest=false);
    uint8_t Submit(const StreamProtocol::Record& record, uint32_t now, bool prevalidated=false);
    const StreamProtocol::Record* Previous() const { return havePrevious ? &previous : nullptr; }
    uint8_t Start(uint16_t session, uint32_t now);
    uint8_t Close(uint16_t session, uint32_t now);
    void Cancel(uint8_t reason=StreamProtocol::ABORTED);
    void ProtocolFault(uint8_t reason);
    enum FaultSource : uint8_t { SOURCE_NONE,SOURCE_PROTOCOL,SOURCE_PROBE,SOURCE_TRANSFER,
        SOURCE_POLL,SOURCE_CLOSE,SOURCE_START,SOURCE_SYNC,SOURCE_COMPLETE,SOURCE_MOTOR,SOURCE_CAN };
    struct FaultSnapshot {
        uint32_t age=0,at=0,gap=0;
        uint16_t sequence=0;
        uint8_t source=0,state=0,phase=0,node=0,command=0,prepared=0,committed=0,started=0;
    };
    const FaultSnapshot& Failure() const { return failure; }
    void TransportFailure(uint16_t token,uint8_t reason,uint8_t node,uint8_t command,uint16_t sequence,uint32_t age,uint32_t now);
    void Heartbeat(uint32_t now) { hostAt=now; }
    void Poll(uint32_t now);
    void Receive(uint8_t node, uint8_t command, const uint8_t data[8], uint32_t now);
    bool Busy() const { return state!=IDLE && state!=FAULT; }
    bool Active() const { return session!=0; }
    bool IsOpenTest() const { return testOnly; }
    uint16_t TestSession() const { return testSession; }
    uint8_t TestNode() const { return testNode; }
    bool TestOpened() const { return testOpened; }
    bool TestClosing() const { return closing; }
    bool TestCloseFailed() const { return testCloseFailed; }
    bool CanTransmit(uint16_t token) const { return session==token && Busy(); }
    State GetState() const { return state; }
    uint16_t Session() const { return session; }
    uint16_t Next() const { return next; }
    uint16_t LastCommitted() const { return committed; }
    uint16_t LastExecuted() const;
    uint8_t Free() const { return StreamProtocol::CORE_CAPACITY-count; }
    uint8_t CapabilityMask() const { return capabilities; }
    // Actual CAPS evidence survives CLOSE for this token only.
    uint16_t FeatureSession() const { return featureSession; }
    uint8_t StableEntryMask() const { return stableEntryMask; }
    uint8_t SelectedMask() const { return selected; }
    uint8_t Reason() const { return reason; }
    uint16_t FinalSequence() const { return finalSequence; }
    uint32_t Revision() const { return revision; }
    uint32_t MaxPollGap() const { return maxPollGap; }
    static const char* Name(State value);
private:
    CoreStreamPort& port;
    StreamProtocol::Record queue[StreamProtocol::CORE_CAPACITY]{};
    StreamProtocol::Record previous{};
    uint32_t nodeAt[7]{}, executedAt[7]{};
    uint16_t executed[7]{}, nodeCommitted[7]{};
    uint8_t motorFree[7]{};
    uint16_t session=0, retiredSession=0, next=0, committed=0, finalSequence=0;
    uint16_t syncCounter=0;
    uint32_t openedAt=0, hostAt=0, transferAt=0, retryAt=0, startAt=0, lastStart=0, closeAt=0, closeRetryAt=0;
    uint32_t lastSync=0, lastPoll=0, maxPollGap=0, revision=0;
    uint8_t selected=0, capabilities=0, opened=0, prepared=0, committedMask=0;
    uint8_t startedMask=0, completedMask=0, closeMask=0, head=0, count=0, txCursor=0, txPart=0;
    uint8_t transferPhase=0, probeCursor=0, reason=0;
    bool havePrevious=false, haveCommitted=false, endAccepted=false, endCommitted=false;
    bool startRequested=false, startSent=false, havePoll=false, stopPending=false, closing=false;
    bool testOnly=false,testOpened=false,testCloseFailed=false;
    uint16_t testSession=0; uint8_t testNode=0,testContacted=0;
    uint16_t featureSession=0; uint8_t stableEntryMask=0;
    State state=IDLE;
    FaultSnapshot failure{};
    void SetState(State value) { if(state!=value) { state=value; ++revision; } }
    void Fail(uint8_t value,FaultSource source=SOURCE_PROTOCOL,uint32_t now=0,uint32_t age=0,uint8_t node=0,uint8_t command=0);
    bool SendControl(uint8_t node, uint8_t command, uint16_t sequence,
                     uint16_t value=0, uint8_t flag=0);
    bool SendSelected(uint8_t command,const uint8_t data[8],bool* accepted=nullptr,uint8_t confirmed=0);
    void Probe(uint32_t now);
    void Transfer(uint32_t now);
    void TransferFinished();
    bool RoomInMotors() const;
};
