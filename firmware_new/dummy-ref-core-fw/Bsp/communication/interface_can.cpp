/*
*
* Zero-config node ID negotiation
* -------------------------------
*
* A heartbeat message is a message with a 8 byte unique serial number as payload.
* A regular message is any message that is not a heartbeat message.
*
* All nodes MUST obey these four rules:
*
* a) At a given point in time, a node MUST consider a node ID taken (by others)
*   if any of the following is true:
*     - the node received a (not self-emitted) heartbeat message with that node ID
*       within the last second
*     - the node attempted and failed at sending a heartbeat message with that
*       node ID within the last second (failed in the sense of not ACK'd)
*
* b) At a given point in time, a node MUST NOT consider a node ID self-assigned
*   if, within the last second, it did not succeed in sending a heartbeat
*   message with that node ID.
*
* c) At a given point in time, a node MUST NOT send any heartbeat message with
*   a node ID that is taken.
*
* d) At a given point in time, a node MUST NOT send any regular message with
*   a node ID that is not self-assigned.
*
* Hardware allocation
* -------------------
*   RX FIFO0:
*       - filter bank 0: heartbeat messages
*/

#include "common_inc.h"
#include "instances/protection_coordinator.h"
#include "instances/core_stream.h"
#include "../../../stream_protocol.h"
#include <stm32f4xx_hal.h>
#include <cmsis_os.h>

// defined in can.c
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern DummyRobot dummy;
extern osThreadId_t ControlLoopUpdateHandle;

CAN_context can1Ctx;
CAN_context can2Ctx;
static CAN_context* ctxs = nullptr;
static CAN_RxHeaderTypeDef headerRx;
static uint8_t data[8];

namespace {
enum class ProbeTxResult : uint8_t { TXOK, ALST, TERR, ABORTED };

uint8_t ProbeMailboxIndex(uint32_t mailbox)
{
    return mailbox == CAN_TX_MAILBOX0 ? 0 : mailbox == CAN_TX_MAILBOX1 ? 1 :
        mailbox == CAN_TX_MAILBOX2 ? 2 : 3;
}

void ClearProbeMailbox(CAN_context* ctx, uint32_t mailbox)
{
    const uint8_t index = ProbeMailboxIndex(mailbox);
    if (index < 3) ctx->probeTx[index] = {};
}

void RecordProbeAccepted(CAN_context* ctx, const CAN_TxHeaderTypeDef* header,
                         const uint8_t* payload, uint32_t mailbox)
{
    const uint8_t index = ProbeMailboxIndex(mailbox);
    if (index >= 3) return;
    // Every successful mailbox assignment clears the former diagnostic owner.
    ctx->probeTx[index] = {};
    const uint32_t node = header->StdId >> 7;
    if (header->IDE != CAN_ID_STD || header->RTR != CAN_RTR_DATA || header->DLC != 8 ||
        (header->StdId & 0x7F) != ProtectionProtocol::CONTEXT ||
        payload[0] != ProtectionProtocol::QUERY || node < 1 || node > 8) return;
    ctx->probeTx[index] = {uint16_t(payload[2] | (uint16_t(payload[3]) << 8)), uint8_t(node)};
    ++ctx->probeTxCounters[node].accepted;
}

void RecordProbeResult(CAN_context* ctx, uint8_t index, ProbeTxResult result)
{
    if (!ctx || index >= 3) return;
    const CAN_context::ProbeTx probe = ctx->probeTx[index];
    ctx->probeTx[index] = {}; // One hardware result, even if HAL reports it twice.
    if (probe.node < 1 || probe.node > 8) return;
    CAN_context::ProbeTxCounters& counters = ctx->probeTxCounters[probe.node];
    if (result == ProbeTxResult::TXOK) { ++counters.txok; counters.lastTxOk = probe.nonce; }
    else if (result == ProbeTxResult::ALST) ++counters.alst;
    else if (result == ProbeTxResult::TERR) ++counters.terr;
    else ++counters.aborted;
}
}


struct CAN_context* get_can_ctx(CAN_HandleTypeDef* hcan)
{
    if (hcan->Instance == CAN1)
        return &can1Ctx;
    else if (hcan->Instance == CAN2)
        return &can2Ctx;
    else
        return nullptr;
}

bool StartCanServer(CAN_TypeDef* hcan)
{
    if (hcan == CAN1)
    {
        ctxs = &can1Ctx;
        ctxs->handle = &hcan1;
    } else if (hcan == CAN2)
    {
        ctxs = &can2Ctx;
        ctxs->handle = &hcan2;
    } else
        return false; // fail if none of the above checks matched

    HAL_StatusTypeDef status;

    ctxs->node_id = 0;
    ctxs->serial_number = serialNumber;
    osSemaphoreDef(sem_send_heartbeat);
    ctxs->sem_send_heartbeat = osSemaphoreNew(1, 0, osSemaphore(sem_send_heartbeat));

    //// Set up filter
    CAN_FilterTypeDef sFilterConfig = {
        .FilterIdHigh = 0x0000,
        .FilterIdLow = 0x0000,
        .FilterMaskIdHigh = 0x0000,
        .FilterMaskIdLow = 0x0000,
        .FilterFIFOAssignment = CAN_RX_FIFO0,
        .FilterBank = 0,
        .FilterMode = CAN_FILTERMODE_IDMASK,
        .FilterScale = CAN_FILTERSCALE_16BIT, // two 16-bit filters
        .FilterActivation = ENABLE,
        .SlaveStartFilterBank = 0
    };
    status = HAL_CAN_ConfigFilter(ctxs->handle, &sFilterConfig);
    if (status != HAL_OK)
        return false;

    status = HAL_CAN_Start(ctxs->handle);
    if (status != HAL_OK)
        return false;

    status = HAL_CAN_ActivateNotification(ctxs->handle,
                                          CAN_IT_TX_MAILBOX_EMPTY |
                                          CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING |
                                          /* we probably only want this */
                                          CAN_IT_RX_FIFO0_FULL | CAN_IT_RX_FIFO1_FULL |
                                          CAN_IT_RX_FIFO0_OVERRUN | CAN_IT_RX_FIFO1_OVERRUN |
                                          CAN_IT_WAKEUP | CAN_IT_SLEEP_ACK |
                                          CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE |
                                          CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE |
                                          CAN_IT_ERROR);
    if (status != HAL_OK)
        return false;

    return true;
}

namespace {
// This guard works in tasks and CAN callbacks, and preserves an outer mask.
struct CanGuard {
    UBaseType_t mask;
    CanGuard():mask(portSET_INTERRUPT_MASK_FROM_ISR()) {}
    ~CanGuard(){portCLEAR_INTERRUPT_MASK_FROM_ISR(mask);}
};
constexpr unsigned STREAM_PENDING_CAPACITY=16;
constexpr uint32_t STREAM_TX_MAX_AGE_MS=30;
constexpr uint8_t STREAM_TX_MAX_ATTEMPTS=8;
enum : uint8_t { PENDING_READY=1,PENDING_HARDWARE=2,PENDING_DONE=3 };
struct StreamPending {
    CAN_TxHeaderTypeDef header{};
    uint8_t data[8]{};
    uint32_t epoch=0,created=0,retryAt=0,timeValue=0;
    uint16_t session=0;
    uint8_t attempts=0,state=0;
};
StreamPending streamPending[STREAM_PENDING_CAPACITY];
uint8_t streamHead=0,streamCount=0;

// HAL selects by hardware TME/CODE alone. Explicitly select an unowned mailbox
// so a completed transmission with a pending RQCP cannot lose its software owner.
// The register encoding follows HAL_CAN_AddTxMessage; all callers hold CanGuard
// or a task critical section until their metadata is committed.
bool QueueHardware(CAN_context* ctx,const CAN_TxHeaderTypeDef* h,const uint8_t* d,bool native) {
    if(!ctx || !ctx->handle || h->DLC>8 ||
       (ctx->handle->State!=HAL_CAN_STATE_READY && ctx->handle->State!=HAL_CAN_STATE_LISTENING)) return false;
    const uint32_t available=((ctx->handle->Instance->TSR>>26)&7U)&~ctx->inFlightMailboxes;
    if(!available) return false;
    const unsigned i=(available&1U)?0:(available&2U)?1:2;
    const uint32_t bit=1U<<i;
    auto& mailbox=ctx->handle->Instance->sTxMailBox[i];
    ctx->motionTx[i]={};ctx->streamTx[i]={};ctx->probeTx[i]={};ctx->streamPendingSlot[i]=0;
    if(native) ctx->nativeTokenMask|=bit; else ctx->nativeTokenMask&=~bit;
    ctx->inFlightMailboxes|=bit;ctx->last_heartbeat_mailbox=bit;
    mailbox.TIR=h->IDE==CAN_ID_STD ? (h->StdId<<CAN_TI0R_STID_Pos)|h->RTR :
        (h->ExtId<<CAN_TI0R_EXID_Pos)|h->IDE|h->RTR;
    mailbox.TDTR=h->DLC | (h->TransmitGlobalTime==ENABLE ? CAN_TDT0R_TGT:0U);
    mailbox.TDLR=uint32_t(d[0])|(uint32_t(d[1])<<8)|(uint32_t(d[2])<<16)|(uint32_t(d[3])<<24);
    mailbox.TDHR=uint32_t(d[4])|(uint32_t(d[5])<<8)|(uint32_t(d[6])<<16)|(uint32_t(d[7])<<24);
    __DMB();mailbox.TIR|=CAN_TI0R_TXRQ;
    return true;
}
void ReclaimStreamQueue() {
    while(streamCount && streamPending[streamHead].state==PENDING_DONE) {
        streamPending[streamHead]={};streamHead=(streamHead+1)%STREAM_PENDING_CAPACITY;--streamCount;
    }
}
bool PendingCurrent(const StreamPending& p) {
    return p.epoch==dummy.GetMotionEpoch() && !dummy.IsStopLocked() && CoreStreamCanTransmit(p.session);
}
void ReportPendingFailure(const StreamPending& p,uint32_t now) {
    CoreStreamTransportFault(p.session,StreamProtocol::DEADLINE,uint8_t(p.header.StdId>>7),
        uint8_t(p.header.StdId&127),StreamProtocol::Read16(p.data+2),now-p.created);
}
void StreamTransmitResult(unsigned index,bool delivered) {
    CanGuard guard;
    auto& p=streamPending[index];
    if(p.state!=PENDING_HARDWARE) return;
    const uint32_t now=HAL_GetTick();
    if(delivered || !PendingCurrent(p)) p.state=PENDING_DONE;
    else if(p.attempts>=STREAM_TX_MAX_ATTEMPTS || now-p.created>=STREAM_TX_MAX_AGE_MS) {
        ReportPendingFailure(p,now);p.state=PENDING_DONE;
    } else {
        p.state=PENDING_READY;
        p.retryAt=now+1U+((p.attempts+(p.header.StdId>>7))&1U);
    }
    ReclaimStreamQueue();
}
}

void PollCanStreamTransmit(CAN_context* ctx) {
    if(!ctx || ctx->handle!=&hcan1) return;
    CanGuard guard;
    const uint32_t now=HAL_GetTick();
    // Three submissions maximum. Software owners remain occupied until their
    // hardware result, including abort, has actually been consumed.
    for(unsigned budget=0;budget<3;++budget) {
        int candidate=-1;
        uint16_t blockedNodes=0;
        for(unsigned k=0;k<streamCount;++k) {
            const unsigned index=(streamHead+k)%STREAM_PENDING_CAPACITY;
            auto& p=streamPending[index];
            if(p.state==PENDING_DONE) continue;
            const uint16_t bit=1U<<(p.header.StdId>>7);
            if(!PendingCurrent(p)) {
                if(p.state==PENDING_READY) p.state=PENDING_DONE;
                else blockedNodes|=bit;
                continue;
            }
            if(p.state==PENDING_READY && now-p.created>=STREAM_TX_MAX_AGE_MS) {
                ReportPendingFailure(p,now);p.state=PENDING_DONE;continue;
            }
            const bool blocked=(blockedNodes&bit)!=0;blockedNodes|=bit;
            if(!blocked && p.state==PENDING_READY && int32_t(now-p.retryAt)>=0) {
                // START/SYNC go before bulk endpoints once their own preceding
                // broadcast has completed; no endpoint ordering is changed.
                if(candidate<0 || (p.header.StdId>>7)==0) candidate=int(index);
                if((p.header.StdId>>7)==0) break;
            }
        }
        if(candidate<0) break;
        auto& p=streamPending[candidate];
        uint8_t payload[8];memcpy(payload,p.data,8);
        const uint8_t command=p.header.StdId&127;
        if(command==StreamProtocol::START) {
            const int32_t remaining=int32_t(p.timeValue-now*20U);
            if(remaining<=StreamProtocol::START_GUARD_TICKS) {
                ReportPendingFailure(p,now);p.state=PENDING_DONE;continue;
            }
            StreamProtocol::Write16(payload+4,uint16_t(remaining));
        } else if(command==StreamProtocol::SYNC) {
            StreamProtocol::Write32(payload+4,p.timeValue+(now-p.created)*20U);
        }
        if(!QueueHardware(ctx,&p.header,payload,false)) break;
        const unsigned mailbox=ProbeMailboxIndex(ctx->last_heartbeat_mailbox);
        ctx->streamTx[mailbox]={p.epoch,now,p.session,command,false};
        ctx->streamPendingSlot[mailbox]=uint8_t(candidate+1);
        ++p.attempts;p.state=PENDING_HARDWARE;
    }
    ReclaimStreamQueue();
}

static void FinishCanTransmit(CAN_HandleTypeDef* hcan, uint8_t mailbox_idx, bool delivered)
{
    CAN_context* ctx = get_can_ctx(hcan);
    if (!ctx || mailbox_idx > 2 || !(ctx->inFlightMailboxes & (1U << mailbox_idx))) return;
    const uint32_t ownedBit=1U<<mailbox_idx;
    const bool nativeToken=(ctx->nativeTokenMask&ownedBit)!=0;
    const uint8_t pendingSlot=ctx->streamPendingSlot[mailbox_idx];
    ctx->nativeTokenMask &= ~ownedBit;
    ctx->streamPendingSlot[mailbox_idx]=0;
    const CAN_context::MotionTx target = ctx->motionTx[mailbox_idx];
    ctx->motionTx[mailbox_idx] = {};
    ctx->streamTx[mailbox_idx] = {};
    ctx->inFlightMailboxes &= ~(1U << mailbox_idx);
    if (delivered) ++ctx->TxMailboxCompleteCallbackCnt;
    if (target.node) dummy.MotionTargetResult(target.node, target.epoch, target.generation, delivered);
    if (nativeToken && hcan->Instance == CAN1) osSemaphoreRelease(sem_can1_tx);
    else if (nativeToken && hcan->Instance == CAN2) osSemaphoreRelease(sem_can2_tx);
    if(pendingSlot) StreamTransmitResult(pendingSlot-1,delivered);
    if(hcan==&hcan1) PollCanStreamTransmit(ctx);
}

void tx_complete_callback(CAN_HandleTypeDef* hcan, uint8_t mailbox_idx)
{
    RecordProbeResult(get_can_ctx(hcan), mailbox_idx, ProbeTxResult::TXOK);
    FinishCanTransmit(hcan, mailbox_idx, true);
}

void tx_aborted_callback(CAN_HandleTypeDef* hcan, uint8_t mailbox_idx)
{
    if (!get_can_ctx(hcan))
        return;
    get_can_ctx(hcan)->TxMailboxAbortCallbackCnt++;
    RecordProbeResult(get_can_ctx(hcan), mailbox_idx, ProbeTxResult::ABORTED);
    // An aborted protection transmission must not consume the sole send token.
    FinishCanTransmit(hcan, mailbox_idx, false);
}

void tx_error(CAN_context* ctx, uint8_t mailbox_idx)
{
    // HAL TERR calls only ErrorCallback, without complete/abort notification.
    if (ctx) FinishCanTransmit(ctx->handle, mailbox_idx, false);
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef* hcan)
{ tx_complete_callback(hcan, 0); }

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef* hcan)
{ tx_complete_callback(hcan, 1); }

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef* hcan)
{ tx_complete_callback(hcan, 2); }

void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef* hcan)
{ tx_aborted_callback(hcan, 0); }

void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef* hcan)
{ tx_aborted_callback(hcan, 1); }

void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef* hcan)
{ tx_aborted_callback(hcan, 2); }

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    CAN_context* ctx = get_can_ctx(hcan);
    if (!ctx) return;
    ctx->received_msg_cnt++;

    HAL_StatusTypeDef status = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &headerRx, data);
    if (status != HAL_OK)
    {
        ctx->unexpected_errors++;
        return;
    }

    OnCanMessage(ctx, &headerRx, data);
}

void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef* hcan)
{
    CAN_context* ctx = get_can_ctx(hcan);
    if (!ctx) return;
    ctx->RxFifo0FullCallbackCnt++;
    // The HAL IRQ can report FULL and MSG_PENDING together. Every consumed
    // frame must still reach the protocol, especially the seven-node ACK burst.
    HAL_CAN_RxFifo0MsgPendingCallback(hcan);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan)
{ if (get_can_ctx(hcan)) get_can_ctx(hcan)->RxFifo1MsgPendingCallbackCnt++; }

void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef* hcan)
{ if (get_can_ctx(hcan)) get_can_ctx(hcan)->RxFifo1FullCallbackCnt++; }

void HAL_CAN_SleepCallback(CAN_HandleTypeDef* hcan)
{ if (get_can_ctx(hcan)) get_can_ctx(hcan)->SleepCallbackCnt++; }

void HAL_CAN_WakeUpFromRxMsgCallback(CAN_HandleTypeDef* hcan)
{ if (get_can_ctx(hcan)) get_can_ctx(hcan)->WakeUpFromRxMsgCallbackCnt++; }

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef* hcan)
{
    //__asm volatile ("bkpt");
    CAN_context* ctx = get_can_ctx(hcan);
    if (!ctx) return;
    volatile uint32_t original_error = hcan->ErrorCode;
    if (original_error & (HAL_CAN_ERROR_TX_ALST0 | HAL_CAN_ERROR_TX_ALST1 | HAL_CAN_ERROR_TX_ALST2))
        ++ctx->tx_arbitration_losses;
    if (original_error & (HAL_CAN_ERROR_TX_TERR0 | HAL_CAN_ERROR_TX_TERR1 | HAL_CAN_ERROR_TX_TERR2))
        ++ctx->tx_transmission_errors;

    // Consume each captured result exactly once, before any completion pump
    // can assign a new owner. ALST/TERR bits must never leak into a later IRQ.
    const uint32_t alst[3]={HAL_CAN_ERROR_TX_ALST0,HAL_CAN_ERROR_TX_ALST1,HAL_CAN_ERROR_TX_ALST2};
    const uint32_t terr[3]={HAL_CAN_ERROR_TX_TERR0,HAL_CAN_ERROR_TX_TERR1,HAL_CAN_ERROR_TX_TERR2};
    const uint32_t txErrors=alst[0]|alst[1]|alst[2]|terr[0]|terr[1]|terr[2];
    hcan->ErrorCode &= ~txErrors;
    if(original_error&(terr[0]|terr[1]|terr[2])) hcan->ErrorCode &= ~(HAL_CAN_ERROR_EWG|HAL_CAN_ERROR_ACK);
    for(uint8_t i=0;i<3;++i) if(original_error&(alst[i]|terr[i])) {
        RecordProbeResult(ctx,i,(original_error&terr[i]) ? ProbeTxResult::TERR : ProbeTxResult::ALST);
        tx_error(ctx,i);
    }

    if (hcan->ErrorCode)
        ctx->unexpected_errors++;
}

bool TryCanSendMessage(CAN_context* canCtx, uint8_t* txData, CAN_TxHeaderTypeDef* txHeader)
{
    // Deferred parameter queries must never wait for CAN from the OLED task.
    if (!canCtx) return false;
    const uint8_t command = txHeader->StdId & 0x7F;
    if (!CoreStreamAllowsCan(txHeader->StdId >> 7, command)) return false;
    const bool protectionTraffic = command == SoftStopProtocol::COMMAND ||
        command == ProtectionProtocol::EVENT_ACK ||
        (command >= ProtectionProtocol::PARAM_SET && command <= ProtectionProtocol::DIAG_REPLY);
    if (!protectionTraffic && dummy.IsStopLocked()) return false;
    const uint32_t epoch = dummy.GetMotionEpoch();
    osSemaphoreId_t semaphore;
    if (canCtx->handle->Instance == CAN1) semaphore = sem_can1_tx;
    else if (canCtx->handle->Instance == CAN2) semaphore = sem_can2_tx;
    else return false;
    if (osSemaphoreAcquire(semaphore, 0) != osOK) return false;
    taskENTER_CRITICAL();
    const bool stale = !protectionTraffic && (dummy.IsStopLocked() || epoch != dummy.GetMotionEpoch());
    const bool sent = !stale && CoreStreamAllowsCan(txHeader->StdId >> 7,command) && QueueHardware(canCtx,txHeader,txData,true);
    if (sent) {
        RecordProbeAccepted(canCtx, txHeader, txData, canCtx->last_heartbeat_mailbox);
        canCtx->inFlightMailboxes |= canCtx->last_heartbeat_mailbox;
    }
    taskEXIT_CRITICAL();
    if (!sent)
    {
        osSemaphoreRelease(semaphore);
        return false;
    }
    // The normal TX-complete callback releases the semaphore on success.
    return true;
}

void CanSendMessage(CAN_context* canCtx, uint8_t* txData, CAN_TxHeaderTypeDef* txHeader)
{
    if (!CoreStreamAllowsCan(txHeader->StdId >> 7, txHeader->StdId & 0x7F)) return;
    const uint32_t epoch = dummy.GetMotionEpoch();
    osStatus semaphore_status;
    if (canCtx->handle->Instance == CAN1)
        semaphore_status = osSemaphoreAcquire(sem_can1_tx, 1);
    else if (canCtx->handle->Instance == CAN2)
        semaphore_status = osSemaphoreAcquire(sem_can2_tx, 1);
    else
        return;

    if (semaphore_status == osOK)
    {
        const uint8_t command = txHeader->StdId & 0x7F;
        const bool stopTraffic = command == SoftStopProtocol::COMMAND || command == 0x23;
        if (!stopTraffic && (dummy.IsStopLocked() || epoch != dummy.GetMotionEpoch() ||
            (osThreadGetId() == ControlLoopUpdateHandle && dummy.commandHandler.IsTaskCancelled())))
        {
            if (canCtx->handle->Instance == CAN1) osSemaphoreRelease(sem_can1_tx);
            else osSemaphoreRelease(sem_can2_tx);
            return;
        }
        // Native traffic retains its existing bounded wait. Recheck ownership
        // after any wait/preemption and immediately before committing TXRQ.
        uint32_t start = micros();
        HAL_StatusTypeDef status;
        for (;;)
        {
            taskENTER_CRITICAL();
            const bool current=CoreStreamAllowsCan(txHeader->StdId>>7,command) &&
                (stopTraffic || (!dummy.IsStopLocked() && epoch==dummy.GetMotionEpoch() &&
                 !(osThreadGetId()==ControlLoopUpdateHandle && dummy.commandHandler.IsTaskCancelled())));
            status = current && QueueHardware(canCtx,txHeader,txData,true) ? HAL_OK : HAL_BUSY;
            if (status == HAL_OK) {
                ClearProbeMailbox(canCtx, canCtx->last_heartbeat_mailbox);
                canCtx->inFlightMailboxes |= canCtx->last_heartbeat_mailbox;
            }
            taskEXIT_CRITICAL();
            if (status == HAL_OK || !current) break;
            if ((micros() - start) > 1000)   // 1ms 超时
            {
                canCtx->tx_busy_errors++;   // 用于诊断（健康时应 ≈ 0）
                break;
            }
        }

        if (status != HAL_OK)
        {
            if (canCtx->handle->Instance == CAN1)
                osSemaphoreRelease(sem_can1_tx);
            else if (canCtx->handle->Instance == CAN2)
                osSemaphoreRelease(sem_can2_tx);
        }
        // 成功时信号量由 tx_complete_callback 在发送完成后释放
    }
}


bool CanSendMotionMessage(CAN_context* ctx, uint8_t* data, CAN_TxHeaderTypeDef* header,
                          uint32_t epoch, uint32_t generation)
{
    if (!ctx || !CoreStreamAllowsCan(header->StdId >> 7, header->StdId & 0x7F) || !dummy.CanSendMotion(epoch, generation)) return false;
    osSemaphoreId_t sem;
    if (ctx->handle->Instance == CAN1) sem = sem_can1_tx;
    else if (ctx->handle->Instance == CAN2) sem = sem_can2_tx;
    else return false;
    if (osSemaphoreAcquire(sem, 1) != osOK) return false;
    bool sent = false;
    const uint32_t start = micros();
    do {
        taskENTER_CRITICAL();
        const bool current = dummy.CanSendMotion(epoch, generation) && CoreStreamAllowsCan(header->StdId>>7,header->StdId&127);
        sent = current && QueueHardware(ctx,header,data,true);
        if (sent) {
            const uint32_t mailbox = ctx->last_heartbeat_mailbox;
            ClearProbeMailbox(ctx, mailbox);
            const uint8_t index = mailbox == CAN_TX_MAILBOX0 ? 0 : mailbox == CAN_TX_MAILBOX1 ? 1 : 2;
            const uint8_t node = header->StdId >> 7;
            if ((header->StdId & 0x7F) == 0x07 && node >= 1 && node <= 7) {
                ctx->motionTx[index] = {epoch, generation, HAL_GetTick(), node, false};
                dummy.MotionTargetQueued(node, epoch, generation);
            } else ctx->motionTx[index] = {};
            ctx->inFlightMailboxes |= mailbox;
        }
        taskEXIT_CRITICAL();
        if (sent || !current) break;
    } while (micros() - start <= 1000);
    if (!sent) { ++ctx->tx_busy_errors; osSemaphoreRelease(sem); }
    return sent;
}

void PollCanMotionTransmit(CAN_context* ctx)
{
    if (!ctx || !ctx->handle) return;
    taskENTER_CRITICAL();
    for (uint8_t index = 0; index < 3; ++index) {
        auto& stream=ctx->streamTx[index];
        if(stream.session && !stream.aborting && (ctx->inFlightMailboxes&(1U<<index))) {
            const bool stale=dummy.GetMotionEpoch()!=stream.epoch || !CoreStreamCanTransmit(stream.session);
            // Timed frames never sit in hardware indefinitely. An abort keeps
            // the pending payload and its owner until the result callback.
            const uint32_t lifetime=(stream.command==StreamProtocol::START || stream.command==StreamProtocol::SYNC)?1U:5U;
            if(stale || HAL_GetTick()-stream.queuedAt>=lifetime)
                if(HAL_CAN_AbortTxRequest(ctx->handle,1U<<index)==HAL_OK) stream.aborting=true;
        }
        CAN_context::MotionTx& target = ctx->motionTx[index];
        if (!target.node || target.aborting || !(ctx->inFlightMailboxes & (1U << index))) continue;
        if (!dummy.CanSendMotion(target.epoch, target.generation) || HAL_GetTick() - target.queuedAt >= 20) {
            // Nonblocking hardware abort. The result callback remains the sole
            // owner of semaphore release; never reuse a mailbox on a timeout.
            if (HAL_CAN_AbortTxRequest(ctx->handle, 1U << index) == HAL_OK) target.aborting = true;
        }
    }
    taskEXIT_CRITICAL();
}

// Admission is bounded queue ownership, not delivery. The pump preserves
// per-node TXOK ordering; it can fill three independently owned hardware slots.
bool TryCanSendStreamMessage(CAN_context* ctx,uint8_t* data,
                             CAN_TxHeaderTypeDef* header,uint16_t session) {
    if(!ctx || ctx->handle!=&hcan1) return false;
    const uint8_t command=header->StdId&0x7F;
    if(command==StreamProtocol::ABORT || command==StreamProtocol::DIAG) {
        // These queries/stop notices have a caller-owned retry state and are
        // permitted after a session faults. Preserve native token accounting.
        if(osSemaphoreAcquire(sem_can1_tx,0)!=osOK) return false;
        bool sent;
        { CanGuard guard; sent=QueueHardware(ctx,header,data,true); }
        if(!sent) osSemaphoreRelease(sem_can1_tx);
        return sent;
    }
    CanGuard guard;
    if(!CoreStreamCanTransmit(session)||dummy.IsStopLocked()) return false;
    ReclaimStreamQueue();
    // A protocol-level retry need not duplicate an already owned CAN frame.
    for(unsigned k=0;k<streamCount;++k) {
        const auto& p=streamPending[(streamHead+k)%STREAM_PENDING_CAPACITY];
        if(p.state!=PENDING_DONE && p.session==session && p.header.StdId==header->StdId &&
            !memcmp(p.data,data,8)) return true;
    }
    if(streamCount==STREAM_PENDING_CAPACITY) return false;
    auto& p=streamPending[(streamHead+streamCount)%STREAM_PENDING_CAPACITY];
    p={};p.header=*header;memcpy(p.data,data,8);p.session=session;
    p.epoch=dummy.GetMotionEpoch();p.created=p.retryAt=HAL_GetTick();p.state=PENDING_READY;
    // START stores its absolute anchor; SYNC stores elapsed at admission.
    p.timeValue=command==StreamProtocol::START ? p.created*20U+StreamProtocol::Read16(data+4) : StreamProtocol::Read32(data+4);
    ++streamCount;
    return true;
}
