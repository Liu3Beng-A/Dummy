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
