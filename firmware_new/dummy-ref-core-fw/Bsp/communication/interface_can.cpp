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

static void FinishCanTransmit(CAN_HandleTypeDef* hcan, uint8_t mailbox_idx, bool delivered)
{
    CAN_context* ctx = get_can_ctx(hcan);
    if (!ctx || mailbox_idx > 2 || !(ctx->inFlightMailboxes & (1U << mailbox_idx))) return;
    const CAN_context::MotionTx target = ctx->motionTx[mailbox_idx];
    ctx->motionTx[mailbox_idx] = {};
    ctx->inFlightMailboxes &= ~(1U << mailbox_idx);
    if (delivered) ++ctx->TxMailboxCompleteCallbackCnt;
    if (target.node) dummy.MotionTargetResult(target.node, target.epoch, target.generation, delivered);
    if (hcan->Instance == CAN1)
        osSemaphoreRelease(sem_can1_tx);
    else if (hcan->Instance == CAN2)
        osSemaphoreRelease(sem_can2_tx);
}

void tx_complete_callback(CAN_HandleTypeDef* hcan, uint8_t mailbox_idx)
{ FinishCanTransmit(hcan, mailbox_idx, true); }

void tx_aborted_callback(CAN_HandleTypeDef* hcan, uint8_t mailbox_idx)
{
    if (!get_can_ctx(hcan))
        return;
    get_can_ctx(hcan)->TxMailboxAbortCallbackCnt++;
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
    // §8.7 — FIFO 满 = 主控来不及读。释放 1 个槽位让后续帧进来。
    // HAL_CAN_GetRxMessage 调用后该帧已被消费（不再上 CAN 协议层）。
    // 不调 OnCanMessage — 这一帧已被放弃，由 HAL_CAN_RxFifo0MsgPendingCallback
    // 在它自己的入口正常处理后续到达的帧。
    CAN_RxHeaderTypeDef tmpHeader;
    uint8_t tmpBuf[8];
    (void) HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &tmpHeader, tmpBuf);
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

    // handle transmit errors in all three mailboxes
    if (hcan->ErrorCode & HAL_CAN_ERROR_TX_ALST0)
    {
        tx_error(ctx, 0);
        hcan->ErrorCode &= ~HAL_CAN_ERROR_TX_ALST0;
    } else if (hcan->ErrorCode & HAL_CAN_ERROR_TX_TERR0)
    {
        tx_error(ctx, 0);
        hcan->ErrorCode &= ~HAL_CAN_ERROR_EWG;
        hcan->ErrorCode &= ~HAL_CAN_ERROR_ACK;
        hcan->ErrorCode &= ~HAL_CAN_ERROR_TX_TERR0;
    }

    if (hcan->ErrorCode & HAL_CAN_ERROR_TX_ALST1)
    {
        tx_error(ctx, 1);
        hcan->ErrorCode &= ~HAL_CAN_ERROR_TX_ALST1;
    } else if (hcan->ErrorCode & HAL_CAN_ERROR_TX_TERR1)
    {
        tx_error(ctx, 1);
        hcan->ErrorCode &= ~HAL_CAN_ERROR_EWG;
        hcan->ErrorCode &= ~HAL_CAN_ERROR_ACK;
        hcan->ErrorCode &= ~HAL_CAN_ERROR_TX_TERR1;
    }

    if (hcan->ErrorCode & HAL_CAN_ERROR_TX_ALST2)
    {
        tx_error(ctx, 2);
        hcan->ErrorCode &= ~HAL_CAN_ERROR_TX_ALST2;
    } else if (hcan->ErrorCode & HAL_CAN_ERROR_TX_TERR2)
    {
        tx_error(ctx, 2);
        hcan->ErrorCode &= ~HAL_CAN_ERROR_EWG;
        hcan->ErrorCode &= ~HAL_CAN_ERROR_ACK;
        hcan->ErrorCode &= ~HAL_CAN_ERROR_TX_TERR2;
    }

    if (hcan->ErrorCode)
        ctx->unexpected_errors++;
}

bool TryCanSendMessage(CAN_context* canCtx, uint8_t* txData, CAN_TxHeaderTypeDef* txHeader)
{
    // Deferred parameter queries must never wait for CAN from the OLED task.
    if (!canCtx) return false;
    const uint8_t command = txHeader->StdId & 0x7F;
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
    const bool sent = !stale && HAL_CAN_AddTxMessage(canCtx->handle, txHeader, txData,
                            &canCtx->last_heartbeat_mailbox) == HAL_OK;
    if (sent) canCtx->inFlightMailboxes |= canCtx->last_heartbeat_mailbox;
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
        // §8.6+ — bxCAN 3 mb 限制：等 mb 出现空位再入
        // 5ms 控制周期内 6 条 0x07：J1-J3 入 mb0/1/2 立即成功，J4-J6 HAL_BUSY
        // 等 mb 空后 J4-J6 也能入；总等待 ~340µs（占 5ms 周期 7%）
        // 1ms 超时兜底，防止总线故障永久阻塞信号量
        uint32_t start = micros();
        HAL_StatusTypeDef status;
        for (;;)
        {
            taskENTER_CRITICAL();
            status = HAL_CAN_AddTxMessage(canCtx->handle, txHeader, txData, &canCtx->last_heartbeat_mailbox);
            if (status == HAL_OK) canCtx->inFlightMailboxes |= canCtx->last_heartbeat_mailbox;
            taskEXIT_CRITICAL();
            if (status == HAL_OK) break;
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
    if (!ctx || !dummy.CanSendMotion(epoch, generation)) return false;
    osSemaphoreId_t sem;
    if (ctx->handle->Instance == CAN1) sem = sem_can1_tx;
    else if (ctx->handle->Instance == CAN2) sem = sem_can2_tx;
    else return false;
    if (osSemaphoreAcquire(sem, 1) != osOK) return false;
    bool sent = false;
    const uint32_t start = micros();
    do {
        taskENTER_CRITICAL();
        const bool current = dummy.CanSendMotion(epoch, generation);
        sent = current && HAL_CAN_AddTxMessage(ctx->handle, header, data, &ctx->last_heartbeat_mailbox) == HAL_OK;
        if (sent) {
            const uint32_t mailbox = ctx->last_heartbeat_mailbox;
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
