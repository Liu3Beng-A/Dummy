#include "core_stream_engine.h"
#include <string.h>
using namespace StreamProtocol;

const char* CoreStreamEngine::Name(State value) {
    static const char* const names[]={"IDLE","PROBING","OPEN","BUFFERING","ARMED","RUNNING","DRAINING","FAULT"};
    return names[unsigned(value)<=unsigned(FAULT) ? unsigned(value) : unsigned(FAULT)];
}
uint8_t CoreStreamEngine::Begin(uint16_t token,uint8_t mask,uint32_t now,bool openTest) {
    if(!token || token==retiredSession) return BAD_SESSION;
    // Only the J1 + rail diagnostic pair extends the existing single/all set.
    // Individual motor OPEN packets retain their established one-bit masks.
    if((!ValidMask(mask) && mask!=0x41) || (openTest && (mask&(mask-1)))) return BAD_FLAGS;
    if(Active() || Busy()) return BUSY;
    if(!port.Allowed()) return BLOCKED;
    testOnly=openTest;testSession=openTest?token:0;testOpened=testCloseFailed=false;testContacted=0;
    testNode=1;while(!(mask&(1U<<(testNode-1))))++testNode;
    session=token; selected=diagnosticMask=mask; capabilities=opened=prepared=committedMask=0;
    featureSession=token; stableEntryMask=0;
    startedMask=completedMask=head=count=txCursor=txPart=transferPhase=probeCursor=reason=0;
    next=committed=finalSequence=syncCounter=0;
    havePrevious=haveCommitted=endAccepted=endCommitted=startRequested=startSent=stopPending=closing=false;
    closeMask=0;
    openedAt=hostAt=transferAt=closeAt=now; failure={}; retryAt=now-20; lastStart=lastSync=0;
    memset(nodeAt,0,sizeof(nodeAt)); memset(executedAt,0,sizeof(executedAt));
    memset(executed,0,sizeof(executed)); memset(motorFree,0,sizeof(motorFree));
    memset(nodeCommitted,0,sizeof(nodeCommitted));
    SetState(PROBING); return OK;
}
uint8_t CoreStreamEngine::Submit(const Record& r,uint32_t now,bool prevalidated) {
    if(testOnly) return BAD_STATE;
    if(r.session!=session || !session) return BAD_SESSION;
    if(state!=OPEN && state!=BUFFERING && state!=ARMED && state!=RUNNING && state!=DRAINING) return BAD_STATE;
    if(!port.Allowed()) return BLOCKED;
    if(r.sequence!=next || !ValidFlags(r.sequence,r.flags) || endAccepted) return BAD_SEQUENCE;
    if(count==CORE_CAPACITY) return BUSY;
    if(!prevalidated && !port.Validate(r,havePrevious ? &previous : nullptr,selected)) { Fail(BAD_LIMIT); return BAD_LIMIT; }
    for(unsigned i=0;i<AXES;++i) if((r.flags&(SEED|END)) && (r.axes[i].v || r.axes[i].a)) {
        Fail(BAD_FLAGS); return BAD_FLAGS;
    }
    queue[(head+count)%CORE_CAPACITY]=r; ++count; ++next;
    previous=r; havePrevious=true; hostAt=now;
    if(r.flags&END) { endAccepted=true; finalSequence=r.sequence; }
    if(state==OPEN) SetState(BUFFERING);
    return OK;
}
uint8_t CoreStreamEngine::Start(uint16_t token,uint32_t now) {
    if(testOnly) return BAD_STATE;
    if(token!=session || !session) return BAD_SESSION;
    if(startRequested) return OK; // Retrying the ASCII request never shifts time.
    if(state!=ARMED || !port.Allowed()) return BAD_STATE;
    startRequested=true; startAt=now+START_DELAY_TICKS/20;
    lastStart=now-5; hostAt=now;
    return OK;
}
uint8_t CoreStreamEngine::Close(uint16_t token,uint32_t now) {
    if(!token || token!=session) return BAD_SESSION;
    if(stopPending || (startRequested && state!=IDLE && state!=FAULT)) return BUSY;
    if(count || (state!=IDLE && state!=FAULT && state!=OPEN)) return BUSY;
    if(!closing) { closing=true; testCloseFailed=false; closeMask=0; closeAt=now; closeRetryAt=now-5; }
    hostAt=now; return OK;
}
void CoreStreamEngine::Cancel(uint8_t value) {
    if(!session) return;
    count=0; transferPhase=0; startRequested=false; stopPending=false; closing=false;
    if(state!=FAULT) { reason=value; SetState(FAULT); }
}
void CoreStreamEngine::ProtocolFault(uint8_t value) { if(Busy()) Fail(value); }
void CoreStreamEngine::TransportFailure(uint16_t token,uint8_t value,uint8_t node,uint8_t command,uint16_t sequence,uint32_t age,uint32_t now) {
    if(token!=session || !Busy()) return;
    Fail(value,SOURCE_CAN,now,age,node,command); failure.sequence=sequence;
}
void CoreStreamEngine::Fail(uint8_t value,FaultSource source,uint32_t now,uint32_t age,uint8_t node,uint8_t command) {
    if(state==FAULT) return;
    failure={age,now,maxPollGap,count?queue[head].sequence:committed,uint8_t(source),uint8_t(state),
        transferPhase,node,command,prepared,committedMask,startedMask};
    reason=value; count=0; transferPhase=0; startRequested=false; closing=false;
    SetState(FAULT);
    stopPending=!testOnly; // An OPEN-only test never changes a reference or arms motion.
}
bool CoreStreamEngine::SendControl(uint8_t node,uint8_t command,uint16_t sequence,uint16_t value,uint8_t flag) {
    uint8_t data[8]{}; Write16(data,session); Write16(data+2,sequence);
    Write16(data+4,value); data[6]=flag;
    return port.Send(node,command,data);
}
bool CoreStreamEngine::SendSelected(uint8_t command,const uint8_t data[8],bool* accepted,uint8_t confirmed) {
    bool all=true,any=false;
    if(selected==ALL_AXES) all=any=port.Send(0,command,data);
    else {
        // A token may collide with an old session on an unselected motor. Never
        // broadcast group controls for partial sessions, including graceful CLOSE.
        for(unsigned i=0;i<AXES;++i) if((selected&~confirmed)&(1U<<i)) {
            const bool sent=port.Send(uint8_t(i+1),command,data);
            any|=sent;all&=sent;
        }
    }
    if(accepted)*accepted=any;
    return all;
}
void CoreStreamEngine::Probe(uint32_t now) {
    if(now-openedAt>=3000) { Fail(DEADLINE,SOURCE_PROBE,now,now-openedAt); return; }
    const bool capsDone=(capabilities&selected)==selected;
    if(capsDone && (opened&selected)==selected) { SetState(OPEN); return; }
    const uint8_t missing=selected&~(capsDone ? opened : capabilities);
    for(unsigned attempt=0;attempt<7;++attempt) {
        const uint8_t axis=probeCursor; probeCursor=(probeCursor+1)%7;
        if(!(missing&(1U<<axis))) continue;
        if(nodeAt[axis] && now-nodeAt[axis]<20) continue;
        uint8_t data[8]{}; Write16(data,session); data[2]=VERSION;
        if(capsDone) data[3]=selected==ALL_AXES ? selected : uint8_t(1U<<axis);
        if(port.Send(axis+1,capsDone ? StreamProtocol::OPEN : CAPS,data)) { nodeAt[axis]=now; if(testOnly && capsDone)testContacted|=1U<<axis; }
        break;
    }
}
bool CoreStreamEngine::RoomInMotors() const {
    for(unsigned i=0;i<7;++i) if((selected&(1U<<i)) && !motorFree[i]) return false;
    return true;
}
void CoreStreamEngine::TransferFinished() {
    const Record& record=queue[head];
    committed=record.sequence; haveCommitted=true;
    if(record.flags&END) endCommitted=true;
    head=(head+1)%CORE_CAPACITY; --count; transferPhase=0;
    if(!startRequested && (committed>=PREFILL || (endCommitted && committed>=1))) SetState(ARMED);
    else if(startRequested && endCommitted && state==RUNNING) SetState(DRAINING);
}
void CoreStreamEngine::Transfer(uint32_t now) {
    if(!count) return;
    if(transferPhase==0) {
        if(!RoomInMotors()) return;
        prepared=committedMask=0; txCursor=txPart=0; transferPhase=1;
        transferAt=retryAt=now;
    }
    if(now-transferAt >= (startSent ? 60U : 500U)) { Fail(DEADLINE,SOURCE_TRANSFER,now,now-transferAt); return; }
    const Record& r=queue[head];
    for(unsigned budget=0;budget<3;++budget) {
        if(transferPhase==1) {
            while(txCursor<7 && (!(selected&(1U<<txCursor)) || (prepared&(1U<<txCursor)))) { ++txCursor; txPart=0; }
            if(txCursor==7) { transferPhase=2; retryAt=now; break; }
            const Axis& a=r.axes[txCursor]; uint8_t data[8]{};
            Write16(data,session); Write16(data+2,r.sequence);
            uint8_t command=Q+txPart;
            if(txPart<3) Write32(data+4,uint32_t(txPart==0 ? a.q : txPart==1 ? a.v : a.a));
            else { command=PREPARE; Write16(data+4,CrcEndpoint(session,r.sequence,r.flags,a.q,a.v,a.a)); data[6]=r.flags; }
            if(!port.Send(txCursor+1,command,data)) break;
            if(++txPart==4) { txPart=0; ++txCursor; }
        } else if(transferPhase==2) {
            if((prepared&selected)==selected) { transferPhase=3; txCursor=0; retryAt=now; continue; }
            if(now-retryAt>=15) { transferPhase=1; txCursor=txPart=0; }
            break;
        } else {
            if((committedMask&selected)==selected) { TransferFinished(); break; }
            while(txCursor<7 && (!(selected&(1U<<txCursor)) || (committedMask&(1U<<txCursor)))) ++txCursor;
            if(txCursor==7) {
                if(now-retryAt>=5) { txCursor=0; retryAt=now; }
                break;
            }
            const Axis& a=r.axes[txCursor];
            if(!SendControl(txCursor+1,COMMIT,r.sequence,CrcEndpoint(session,r.sequence,r.flags,a.q,a.v,a.a),r.flags)) break;
            ++txCursor;
        }
    }
}
uint16_t CoreStreamEngine::LastExecuted() const {
    uint16_t value=0xFFFF;
    for(unsigned i=0;i<7;++i) if((diagnosticMask&(1U<<i)) && executed[i]<value) value=executed[i];
    return value==0xFFFF ? 0 : value;
}
void CoreStreamEngine::Poll(uint32_t now) {
    if(havePoll && Busy() && now-lastPoll>maxPollGap) maxPollGap=now-lastPoll;
    if(havePoll && startSent && Busy() && now-lastPoll>10) { Fail(DEADLINE,SOURCE_POLL,now,now-lastPoll); }
    lastPoll=now; havePoll=true;
    if(stopPending) {
        stopPending=false;
        uint8_t data[8]{}; Write16(data,session); data[2]=reason;
        SendSelected(ABORT,data);
        port.Stop(); // Existing coordinated soft-stop remains the final authority.
    }
    // The firmware owns cleanup, even if the host disappears after OPEN.
    if(testOnly && session && !closing && !testCloseFailed && (state==OPEN || state==FAULT)) {
        testOpened|=state==OPEN;
        closing=true;closeMask=0;closeAt=now;closeRetryAt=now-5;++revision;
    }
    if(closing) {
        const uint8_t required=testOnly?testContacted:opened;
        if((closeMask&required)==required) {
            closing=false; retiredSession=session; session=0; selected=0;
            SetState(IDLE); ++revision; return;
        }
        if(now-closeAt>=1000) {
            closing=false;
            if(testOnly) { testCloseFailed=true;if(!reason)reason=DEADLINE;SetState(FAULT);++revision; }
            else Fail(DEADLINE,SOURCE_CLOSE,now,now-closeAt);
            return;
        }
        if(now-closeRetryAt>=5) {
            uint8_t data[8]{}; Write16(data,session); // reason=OK: graceful release.
            if(testOnly ? port.Send(testNode,ABORT,data) : SendSelected(ABORT,data,nullptr,closeMask)) closeRetryAt=now;
        }
        return;
    }
    if(!Busy()) return;
    if(!port.Allowed()) { Fail(BLOCKED); return; }
    // Partial sessions are the staged communication test path. A disconnected
    // host must not leave a motor reserved indefinitely before it sends START.
    // The established seven-axis workflow retains its existing timing contract.
    if(selected!=ALL_AXES && !testOnly && !startRequested && now-hostAt>=1000) {
        Fail(SYNC_TIMEOUT,SOURCE_POLL,now,now-hostAt);return;
    }
    if(state==PROBING) { Probe(now); return; }
    if(startRequested) {
        if(now-hostAt>=300) { Fail(SYNC_TIMEOUT); return; }
        if(!startSent || ((startedMask&selected)!=selected && now-lastStart>=5)) {
            // All selected nodes share one absolute anchor. Motors ignore a
            // duplicate START; retries carry only the remaining delay, including
            // a partially admitted pair of individually addressed controls.
            if(int32_t(startAt-now)<=20) { Fail(DEADLINE,SOURCE_START,now,uint32_t(now-(startAt-100))); return; }
            uint8_t data[8]{};Write16(data,session);Write16(data+2,1);
            Write16(data+4,uint16_t((startAt-now)*20));
            bool accepted=false;SendSelected(START,data,&accepted,startedMask);
            // A partial group admission can already arm one motor. Enable the
            // strict execution deadlines from that first accepted START.
            if(accepted) { startSent=true; lastStart=now; }
        }
        if(int32_t(startAt-now)<=20 && (startedMask&selected)!=selected) { Fail(INCOMPLETE); return; }
        if(int32_t(now-startAt)>=0) {
            if(state==ARMED) SetState(endCommitted ? DRAINING : RUNNING);
            if(!lastSync || now-lastSync>=20) {
                uint8_t data[8]{}; Write16(data,session); Write16(data+2,uint16_t(syncCounter+1));
                Write32(data+4,(now-startAt)*20);
                if(SendSelected(SYNC,data)) { ++syncCounter; lastSync=now; }
                else if(lastSync && now-lastSync>25) { Fail(DEADLINE,SOURCE_SYNC,now,now-lastSync); return; }
            }
            // Every selected motor already confirmed this endpoint. Publish
            // that fact before checking lookahead; this sends no new frames.
            if(count && transferPhase==3 && (committedMask&selected)==selected) {
                // An ACK deferred until this Poll still obeys the existing
                // transfer deadline, including one already in progress at START.
                if(now-transferAt >= (startSent ? 60U : 500U)) {
                    Fail(DEADLINE,SOURCE_TRANSFER,now,now-transferAt); return;
                }
                TransferFinished();
            }
            const uint32_t elapsed=now-startAt;
            if(!endCommitted && committed<=elapsed/20+1) { Fail(UNDERRUN,SOURCE_PROTOCOL,now); return; }
            if(endCommitted && elapsed>uint32_t(finalSequence)*20+3000) { Fail(DEADLINE,SOURCE_COMPLETE,now,elapsed-uint32_t(finalSequence)*20); return; }
            for(unsigned i=0;i<7;++i) if((selected&(1U<<i)) &&
                ((elapsed>=50 && now-nodeAt[i]>=100) || (elapsed>=100 && now-executedAt[i]>=100))) { Fail(SYNC_TIMEOUT); return; }
            if((completedMask&selected)==selected && endCommitted) { SetState(IDLE); startRequested=false; return; }
        }
    }
    Transfer(now);
}
void CoreStreamEngine::Receive(uint8_t node,uint8_t command,const uint8_t d[8],uint32_t now) {
    if(node<1 || node>7 || !session || !(selected&(1U<<(node-1)))) return;
    const unsigned axis=node-1; const uint8_t bit=1U<<axis;
    if(command==CAPS) {
        if(state!=PROBING || Read16(d)!=session) return;
        if(d[2]!=VERSION || !(d[3]&CAP_TIMED) || (testOnly && !(d[3]&CAP_OPEN_SNAPSHOT)) || Read16(d+4)!=SEGMENT_TICKS || d[6]!=CAPACITY || d[7]==8) { Fail(BAD_VERSION); return; }
        const uint8_t oldCaps=capabilities,oldEntry=stableEntryMask;
        capabilities|=bit;
        if(d[3]&CAP_STABLE_ENTRY) stableEntryMask|=bit; else stableEntryMask&=uint8_t(~bit);
        if(oldCaps!=capabilities || oldEntry!=stableEntryMask) ++revision;
        motorFree[axis]=d[6]; nodeAt[axis]=0; return;
    }
    if(command!=STATUS || Read16(d)!=session || (!Busy() && !closing)) return;
    const uint16_t sequence=Read16(d+2); const uint8_t status=d[4];
    if(d[7]!=VERSION) { Fail(BAD_VERSION); return; }
    if(closing && status==STATE_IDLE && !d[6]) { closeMask|=bit; return; }
    if(closing && testOnly && status==STATE_FAULT) return;
    if(status==STATE_FAULT || d[6]) {
        // Cleanup may receive another node's delayed fault. Preserve the first
        // code/source/node/sequence together, including during CLOSE.
        if(state!=FAULT) { Fail(d[6] ? d[6] : BLOCKED,SOURCE_MOTOR,now,0,node,command); failure.sequence=sequence; }
        return;
    }
    if(d[5]>CAPACITY) { Fail(BAD_STATE); return; }
    bool current=false;
    if(status==STATE_OPEN && state==PROBING && (capabilities&bit)) { opened|=bit; current=true; }
    if(count && sequence==queue[head].sequence) {
        if(status==STATE_PREPARED) { prepared|=bit; current=true; }
        if(status==STATE_COMMITTED) { prepared|=bit; committedMask|=bit; nodeCommitted[axis]=sequence; current=true; }
    }
    if(status==STATE_STARTED && startRequested && sequence==1) { startedMask|=bit; current=true; }
    if((status==STATE_RUNNING || status==STATE_COMPLETED) && startSent) {
        if(sequence>committed) { Fail(BAD_SEQUENCE); return; }
        // CAN mailboxes/retries can deliver an older observation after a newer
        // one. Ignore it without renewing health or granting buffer credits.
        if(sequence<executed[axis]) return;
        if(status==STATE_COMPLETED) {
            if(!endCommitted || sequence!=finalSequence) { Fail(BAD_SEQUENCE); return; }
            completedMask|=bit;
        }
        executed[axis]=sequence; executedAt[axis]=now; current=true;
    }
    if(!current) return;
    nodeAt[axis]=now;
    // Derive credits from monotonically observed sequence numbers. A delayed
    // duplicate ACK's old free count must not invent room in a now-full queue.
    const uint16_t outstanding=nodeCommitted[axis]-executed[axis];
    if(outstanding>CAPACITY) { Fail(BAD_SEQUENCE); return; }
    motorFree[axis]=uint8_t(CAPACITY-outstanding);
}
