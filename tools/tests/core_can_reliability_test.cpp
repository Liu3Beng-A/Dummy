#include <cstdint>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <algorithm>
#include "stream_protocol_under_test.h"
using UBaseType_t=uint32_t;
static unsigned maskValue=0,criticalNesting=0;
static UBaseType_t portSET_INTERRUPT_MASK_FROM_ISR(){auto old=maskValue;maskValue=5;return old;}
static void portCLEAR_INTERRUPT_MASK_FROM_ISR(UBaseType_t old){maskValue=old;}
#define taskENTER_CRITICAL() (++criticalNesting)
#define taskEXIT_CRITICAL() (--criticalNesting)
#define __DMB() ((void)0)
struct Mailbox {uint32_t TIR=0,TDTR=0,TDLR=0,TDHR=0;};
struct CAN_TypeDef {uint32_t TSR=7U<<26;Mailbox sTxMailBox[3];};
CAN_TypeDef bus1,bus2;
#define CAN1 (&bus1)
#define CAN2 (&bus2)
enum HAL_StatusTypeDef {HAL_OK,HAL_ERROR,HAL_BUSY};
enum {HAL_CAN_STATE_READY=1,HAL_CAN_STATE_LISTENING=2};
struct CAN_HandleTypeDef {CAN_TypeDef* Instance;uint32_t ErrorCode=0,State=HAL_CAN_STATE_LISTENING;};
struct CAN_TxHeaderTypeDef {uint32_t StdId=0,ExtId=0,IDE=0,RTR=0,DLC=8,TransmitGlobalTime=0;};
using CAN_RxHeaderTypeDef=CAN_TxHeaderTypeDef;
struct CAN_FilterTypeDef {uint32_t FilterIdHigh,FilterIdLow,FilterMaskIdHigh,FilterMaskIdLow,FilterFIFOAssignment,FilterBank,FilterMode,FilterScale,FilterActivation,SlaveStartFilterBank;};
#define CAN_TI0R_STID_Pos 21
#define CAN_TI0R_EXID_Pos 3
#define CAN_TDT0R_TGT (1U<<8)
#define CAN_TI0R_TXRQ 1U
#define CAN_TX_MAILBOX0 1U
#define CAN_TX_MAILBOX1 2U
#define CAN_TX_MAILBOX2 4U
#define CAN_ID_STD 0U
#define CAN_RTR_DATA 0U
#define CAN_FILTERMODE_IDMASK 0U
#define CAN_FILTERSCALE_16BIT 0U
#define CAN_RX_FIFO0 0U
#define ENABLE 1U
#define CAN_IT_TX_MAILBOX_EMPTY 1U
#define CAN_IT_RX_FIFO0_MSG_PENDING 2U
#define CAN_IT_RX_FIFO1_MSG_PENDING 4U
#define CAN_IT_RX_FIFO0_FULL 8U
#define CAN_IT_RX_FIFO1_FULL 16U
#define CAN_IT_RX_FIFO0_OVERRUN 32U
#define CAN_IT_RX_FIFO1_OVERRUN 64U
#define CAN_IT_WAKEUP 128U
#define CAN_IT_SLEEP_ACK 256U
#define CAN_IT_ERROR_WARNING 512U
#define CAN_IT_ERROR_PASSIVE 1024U
#define CAN_IT_BUSOFF 2048U
#define CAN_IT_LAST_ERROR_CODE 4096U
#define CAN_IT_ERROR 8192U
#define HAL_CAN_ERROR_TX_ALST0 1U
#define HAL_CAN_ERROR_TX_ALST1 2U
#define HAL_CAN_ERROR_TX_ALST2 4U
#define HAL_CAN_ERROR_TX_TERR0 8U
#define HAL_CAN_ERROR_TX_TERR1 16U
#define HAL_CAN_ERROR_TX_TERR2 32U
#define HAL_CAN_ERROR_EWG 64U
#define HAL_CAN_ERROR_ACK 128U
struct Semaphore {int count=1;};
Semaphore sem1,sem2,otherSem;
using osSemaphoreId_t=Semaphore*;using osStatus=int;using osThreadId_t=void*;
osSemaphoreId_t sem_can1_tx=&sem1,sem_can2_tx=&sem2;
constexpr int osOK=0;
#define osSemaphoreDef(x)
#define osSemaphore(x) nullptr
static osSemaphoreId_t osSemaphoreNew(unsigned,unsigned,void*){return &otherSem;}
static void(*acquireHook)()=nullptr;
static int osSemaphoreAcquire(Semaphore*s,unsigned){if(!s->count)return 1;--s->count;if(acquireHook){auto hook=acquireHook;acquireHook=nullptr;hook();}return 0;}
static int osSemaphoreRelease(Semaphore*s){if(s->count>=1){std::fprintf(stderr,"duplicate token release\n");std::abort();}++s->count;return 0;}
static void*osThreadGetId(){return nullptr;}
osThreadId_t ControlLoopUpdateHandle=reinterpret_cast<void*>(1);
static uint32_t nowMs=0,clockUs=0,abortMask=0;
static uint32_t HAL_GetTick(){return nowMs;}
static void(*microsHook)()=nullptr;
static uint32_t micros(){if(microsHook){auto hook=microsHook;microsHook=nullptr;hook();}return ++clockUs;}
static HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef*,CAN_FilterTypeDef*){return HAL_OK;}
static HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef*){return HAL_OK;}
static HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef*,uint32_t){return HAL_OK;}
static HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef*,uint32_t bits){abortMask|=bits;return HAL_OK;}
static unsigned rxQueued=0,rxDelivered=0;
static HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef*,unsigned,CAN_RxHeaderTypeDef*,uint8_t*){if(!rxQueued)return HAL_ERROR;--rxQueued;return HAL_OK;}
namespace SoftStopProtocol {constexpr unsigned COMMAND=10;}
namespace ProtectionProtocol {constexpr unsigned CONTEXT=0x35,QUERY=1,EVENT_ACK=0x32,PARAM_SET=0x30,DIAG_REPLY=0x36;}
struct Dummy {
 struct Handler {bool IsTaskCancelled(){return false;}}commandHandler;
 uint32_t epoch=1;bool locked=false;
 uint32_t GetMotionEpoch(){return epoch;}bool IsStopLocked(){return locked;}
 bool CanSendMotion(uint32_t e,uint32_t){return e==epoch&&!locked;}
 void MotionTargetQueued(uint8_t,uint32_t,uint32_t){}
 void MotionTargetResult(uint8_t,uint32_t,uint32_t,bool){}
}dummy;
using DummyRobot=Dummy;
static uint16_t activeSession=42;static bool active=true;
static bool CoreStreamCanTransmit(uint16_t session){return active&&session==activeSession;}
static bool allowNative=true;
static bool CoreStreamAllowsCan(uint8_t,uint8_t){return allowNative;}
struct Fault {uint8_t node,command;uint16_t sequence;uint32_t age;};
std::vector<Fault> faults;
static void CoreStreamTransportFault(uint16_t,uint8_t,uint8_t node,uint8_t command,uint16_t sequence,uint32_t age){faults.push_back({node,command,sequence,age});active=false;}
uint64_t serialNumber=0;
CAN_HandleTypeDef hcan1{CAN1},hcan2{CAN2};
#include "interface_can_header_under_test.inc"
void OnCanMessage(CAN_context*,CAN_RxHeaderTypeDef*,uint8_t*){++rxDelivered;}
#include "interface_can_under_test.inc"
static unsigned checks=0;
#define CHECK(x) do{++checks;if(!(x)){std::fprintf(stderr,"FAIL %u %s\n",__LINE__,#x);std::exit(1);}}while(0)
static void Reset(){can1Ctx=CAN_context{};can1Ctx.handle=&hcan1;can2Ctx=CAN_context{};can2Ctx.handle=&hcan2;bus1=CAN_TypeDef{};bus2=CAN_TypeDef{};dummy=Dummy{};nowMs=clockUs=abortMask=0;active=true;activeSession=42;allowNative=true;acquireHook=microsHook=nullptr;sem1.count=sem2.count=1;faults.clear();streamHead=streamCount=0;for(auto&p:streamPending)p={};}
static bool Send(uint8_t node,uint8_t command,uint16_t sequence=0,uint32_t value=0){CAN_TxHeaderTypeDef h;h.StdId=(unsigned(node)<<7)|command;uint8_t d[8]{};StreamProtocol::Write16(d,activeSession);StreamProtocol::Write16(d+2,sequence);StreamProtocol::Write32(d+4,value);return TryCanSendStreamMessage(&can1Ctx,d,&h,activeSession);}
static uint8_t ReadCommand(unsigned i){return uint8_t((bus1.sTxMailBox[i].TIR>>21)&127);}
static uint8_t Node(unsigned i){return uint8_t((bus1.sTxMailBox[i].TIR>>21)>>7);}
static uint32_t Value(unsigned i){return bus1.sTxMailBox[i].TDHR;}
static void Complete(unsigned i,bool delivered=true){bus1.sTxMailBox[i].TIR&=~1U;bus1.TSR|=1U<<(26+i);FinishCanTransmit(&hcan1,uint8_t(i),delivered);}
int main(){using namespace StreamProtocol;
 Reset();CHECK(Send(1,Q));CHECK(Send(2,Q));CHECK(Send(3,Q));PollCanStreamTransmit(&can1Ctx);CHECK(can1Ctx.inFlightMailboxes==7);CHECK(sem1.count==1);
 // Hardware-free mb0 with pending old completion cannot be reused by a native writer.
 bus1.sTxMailBox[0].TIR&=~1U;CAN_TxHeaderTypeDef h;h.StdId=0x23;uint8_t bytes[8]{};
 CHECK(!TryCanSendMessage(&can1Ctx,bytes,&h));CHECK(can1Ctx.streamPendingSlot[0]!=0);Complete(0);CHECK(TryCanSendMessage(&can1Ctx,bytes,&h));CHECK(sem1.count==0);Complete(0);CHECK(sem1.count==1);Complete(1,false);Complete(2,false);
 Reset();CHECK(Send(1,Q));CHECK(Send(1,V));CHECK(Send(1,A));CHECK(Send(1,PREPARE));CHECK(Send(2,Q));PollCanStreamTransmit(&can1Ctx);CHECK(ReadCommand(0)==Q&&Node(0)==1);CHECK(ReadCommand(1)==Q&&Node(1)==2);CHECK(can1Ctx.inFlightMailboxes==3);Complete(0,false);CHECK(!(can1Ctx.inFlightMailboxes&1));nowMs=3;PollCanStreamTransmit(&can1Ctx);CHECK(ReadCommand(0)==Q);Complete(0);CHECK(ReadCommand(0)==V);Complete(0);CHECK(ReadCommand(0)==A);Complete(0);CHECK(ReadCommand(0)==PREPARE);Complete(0);Complete(1);CHECK(streamCount==0&&faults.empty());
 Reset();CHECK(Send(0,START,1,2000));nowMs=10;PollCanStreamTransmit(&can1Ctx);CHECK((Value(0)&65535)==1800);Complete(0,false);nowMs=13;PollCanStreamTransmit(&can1Ctx);CHECK((Value(0)&65535)==1740);Complete(0);CHECK(Send(0,SYNC,1,300));nowMs=18;PollCanStreamTransmit(&can1Ctx);CHECK(Value(0)==400);Complete(0);
 Reset();CHECK(Send(0,START,1,420));nowMs=2;PollCanStreamTransmit(&can1Ctx);CHECK(!faults.empty()&&can1Ctx.inFlightMailboxes==0);
 Reset();for(unsigned i=0;i<16;++i)CHECK(Send(1,Q,uint16_t(i)));CHECK(!Send(1,Q,17));PollCanStreamTransmit(&can1Ctx);dummy.locked=true;++dummy.epoch;active=false;PollCanMotionTransmit(&can1Ctx);PollCanStreamTransmit(&can1Ctx);CHECK(abortMask==1&&can1Ctx.inFlightMailboxes==1);Complete(0,false);CHECK(!streamCount&&!can1Ctx.inFlightMailboxes&&faults.empty());
 Reset();CHECK(Send(1,Q));PollCanStreamTransmit(&can1Ctx);for(unsigned attempt=0;attempt<8;++attempt){Complete(0,false);nowMs+=3;PollCanStreamTransmit(&can1Ctx);}CHECK(faults.size()==1&&faults[0].command==Q);CHECK(!active);
 // Mixed ALST/TERR completion must retire captured owners before IRQ bits
 // can be re-used. A fresh owner loaded by the pump survives a second callback.
 Reset();for(unsigned n=1;n<=4;++n)CHECK(Send(uint8_t(n),Q));PollCanStreamTransmit(&can1Ctx);
 hcan1.ErrorCode=HAL_CAN_ERROR_TX_ALST0|HAL_CAN_ERROR_TX_TERR0|HAL_CAN_ERROR_TX_TERR1;
 HAL_CAN_ErrorCallback(&hcan1);CHECK(hcan1.ErrorCode==0);CHECK(Node(0)==4);CHECK(can1Ctx.inFlightMailboxes==5);
 const auto retained=can1Ctx.streamPendingSlot[0];HAL_CAN_ErrorCallback(&hcan1);CHECK(can1Ctx.streamPendingSlot[0]==retained);CHECK(sem1.count==1);
 Reset();CHECK(Send(1,Q));PollCanStreamTransmit(&can1Ctx);HAL_CAN_TxMailbox0AbortCallback(&hcan1);CHECK(can1Ctx.inFlightMailboxes==0);CHECK(streamCount==1);CHECK(sem1.count==1);
 nowMs=3;PollCanStreamTransmit(&can1Ctx);CHECK(can1Ctx.inFlightMailboxes==1);HAL_CAN_TxMailbox0CompleteCallback(&hcan1);CHECK(streamCount==0);CHECK(sem1.count==1);
 Reset();CHECK(TryCanSendMessage(&can1Ctx,bytes,&h));CHECK(sem1.count==0);HAL_CAN_TxMailbox0AbortCallback(&hcan1);CHECK(sem1.count==1);HAL_CAN_TxMailbox0AbortCallback(&hcan1);CHECK(sem1.count==1);
 rxQueued=3;rxDelivered=0;HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);HAL_CAN_RxFifo0FullCallback(&hcan1);HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);CHECK(rxQueued==0&&rxDelivered==3);
 // A task may lose motion/mode permission while acquiring its token or
 // waiting for a mailbox. The actual final TXRQ admission must recheck it.
 Reset();h.StdId=(1U<<7)|7U;acquireHook=[](){allowNative=false;};CHECK(!TryCanSendMessage(&can1Ctx,bytes,&h));CHECK(!can1Ctx.inFlightMailboxes&&sem1.count==1);
 Reset();for(unsigned n=1;n<=3;++n)CHECK(Send(uint8_t(n),Q));PollCanStreamTransmit(&can1Ctx);
 microsHook=[](){dummy.locked=true;++dummy.epoch;Complete(0);};CanSendMessage(&can1Ctx,bytes,&h);CHECK(!(can1Ctx.nativeTokenMask&1));CHECK(sem1.count==1);
 Reset();for(unsigned n=1;n<=3;++n)CHECK(Send(uint8_t(n),Q));PollCanStreamTransmit(&can1Ctx);
 microsHook=[](){dummy.locked=true;++dummy.epoch;Complete(0);};CHECK(!CanSendMotionMessage(&can1Ctx,bytes,&h,1,1));CHECK(sem1.count==1);
 // Actual queue, allocation, callbacks, retry/order logic under a 1kHz producer.
 // 100 seven-axis endpoints at50Hz; per-endpoint order is checked at TXOK.
 Reset();unsigned generated=0,accepted=0,delivered=0,native=0;std::vector<std::pair<uint8_t,uint8_t>> input;
 unsigned phase[8]{};uint32_t tick=0;
 for(;tick<44000;++tick){nowMs=tick/20;
  if(tick%400==0&&generated<100){for(uint8_t n=1;n<=7;++n)for(uint8_t c:{uint8_t(Q),uint8_t(V),uint8_t(A),uint8_t(PREPARE),uint8_t(COMMIT)})input.push_back({n,c});++generated;}
  if(tick%20==0){for(unsigned b=0;b<3&&accepted<input.size();++b){auto f=input[accepted];if(!Send(f.first,f.second,uint16_t(accepted/35)))break;++accepted;}PollCanStreamTransmit(&can1Ctx);if(nowMs%5==0){h.StdId=((1+(nowMs/5)%7)<<7)|0x23;if(TryCanSendMessage(&can1Ctx,bytes,&h))++native;}PollCanMotionTransmit(&can1Ctx);}
  if(tick%3==0&&can1Ctx.inFlightMailboxes){unsigned i=0;while(!(can1Ctx.inFlightMailboxes&(1U<<i)))++i;const auto slot=can1Ctx.streamPendingSlot[i];
   if(slot){auto p=streamPending[slot-1];bool fail=(delivered%47==0&&p.attempts==1);if(!fail){const uint8_t n=Node(i),c=ReadCommand(i);const uint8_t expected[5]={Q,V,A,PREPARE,COMMIT};CHECK(c==expected[phase[n]%5]);++phase[n];++delivered;}Complete(i,!fail);}else Complete(i);
  }
  if(delivered==3500&&streamCount==0)break;
 }
 CHECK(delivered==3500&&faults.empty()&&tick<41000);for(unsigned n=1;n<=7;++n)CHECK(phase[n]==500);CHECK(native>250);
 std::printf("PASS %u actual-driver checks; 3500 stream TXOK + %u native requests in %.2fms; sizeofPending=%zu total=%zu\n",checks,native,tick/20.0,sizeof(StreamPending),sizeof(streamPending));
}
