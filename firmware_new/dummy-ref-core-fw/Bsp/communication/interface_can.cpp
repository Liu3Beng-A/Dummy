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
#include <stm32f4xx_hal.h>
#include <cmsis_os.h>

// defined in can.c
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

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

void tx_complete_callback(CAN_HandleTypeDef* hcan, uint8_t mailbox_idx)
{
//    CAN_context* ctx = get_can_ctx(hcan);
//    if (!ctx) return;
//    ctx->tx_msg_cnt++;

    if (hcan->Instance == CAN1)
        osSemaphoreRelease(sem_can1_tx);
    else if (hcan->Instance == CAN2)
        osSemaphoreRelease(sem_can2_tx);
}

void tx_aborted_callback(CAN_HandleTypeDef* hcan, uint8_t mailbox_idx)
{
    if (!get_can_ctx(hcan))
        return;
    get_can_ctx(hcan)->TxMailboxAbortCallbackCnt++;
}

void tx_error(CAN_context* ctx, uint8_t mailbox_idx)
{
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
    (void) original_error;

    // handle transmit errors in all three mailboxes
    if (hcan->ErrorCode & HAL_CAN_ERROR_TX_ALST0)
    {
        SET_BIT(hcan->Instance->sTxMailBox[0].TIR, CAN_TI0R_TXRQ);
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
        SET_BIT(hcan->Instance->sTxMailBox[1].TIR, CAN_TI1R_TXRQ);
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
        SET_BIT(hcan->Instance->sTxMailBox[2].TIR, CAN_TI2R_TXRQ);
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

void CanSendMessage(CAN_context* canCtx, uint8_t* txData, CAN_TxHeaderTypeDef* txHeader)
{
    osStatus semaphore_status;
    if (canCtx->handle->Instance == CAN1)
        semaphore_status = osSemaphoreAcquire(sem_can1_tx, osWaitForever);
    else if (canCtx->handle->Instance == CAN2)
        semaphore_status = osSemaphoreAcquire(sem_can2_tx, osWaitForever);
    else
        return;

    if (semaphore_status == osOK)
    {
        // §8.6+ — bxCAN 3 mb 限制：等 mb 出现空位再入
        // 5ms 控制周期内 6 条 0x07：J1-J3 入 mb0/1/2 立即成功，J4-J6 HAL_BUSY
        // 等 mb 空后 J4-J6 也能入；总等待 ~340µs（占 5ms 周期 7%）
        // 1ms 超时兜底，防止总线故障永久阻塞信号量
        uint32_t start = micros();
        HAL_StatusTypeDef status;
        while ((status = HAL_CAN_AddTxMessage(
                    canCtx->handle, txHeader, txData, &canCtx->last_heartbeat_mailbox)) != HAL_OK)
        {
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

