#pragma once
#include <stddef.h>
#include <stdint.h>
class StreamSink;
bool HandleCoreStreamCommand(const char* command,size_t length,StreamSink& response);
bool CoreStreamAuxAllowed(const char* command,size_t length);
bool CoreStreamBusy();
bool CoreStreamActive();
bool CoreStreamCanTransmit(uint16_t session);
bool CoreStreamAllowsCan(uint8_t node,uint8_t command);
void CoreStreamCancel();
void CoreStreamPoll();
void CoreStreamPrint();
void CoreStreamReceive(uint8_t node,uint8_t command,const uint8_t data[8]);

void CoreStreamTransportFault(uint16_t session,uint8_t reason,uint8_t node,uint8_t command,uint16_t sequence,uint32_t age);

// Internal finite native trajectory owner. Uses the existing motor v2 executor
// without changing the public command mode. Only its low-priority producer
// renews Heartbeat; the 1kHz coordinator never manufactures producer liveness.
#include "core_stream_engine.h"
struct CoreNativeSnapshot {
    uint32_t epoch=0,id=0,revision=0;
    uint16_t session=0,next=0,committed=0,executed=0,finalSequence=0;
    uint8_t state=CoreStreamEngine::IDLE,reason=0,free=0,capabilities=0;
    bool owned=false,closing=false;
    CoreStreamEngine::FaultSnapshot failure{};
};
bool CoreNativeOwned();
uint8_t CoreNativeBegin(uint32_t epoch,uint32_t id);
uint8_t CoreNativeReadHeld(uint8_t node,uint32_t epoch);
bool CoreNativeHeld(uint8_t node,int32_t& value,uint32_t epoch);
// Record must carry the exact current session from SnapshotRead. Validation
// outside IRQ masking may never rebound a retired token to a fresh owner.
uint8_t CoreNativeSubmit(const StreamProtocol::Record& record,uint32_t epoch);
uint8_t CoreNativeStart(uint32_t epoch);
bool CoreNativeHeartbeat(uint32_t epoch);
CoreNativeSnapshot CoreNativeSnapshotRead();
// CLOSE is also available to the original owner after epoch cancellation.
// Release requires an observed closed engine; it never enables the motors.
uint8_t CoreNativeClose(uint32_t epoch);
bool CoreNativeRelease(uint32_t epoch);
