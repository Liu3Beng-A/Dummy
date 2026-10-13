/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include "common_inc.h"
#include "configurations.h"

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;
volatile uint32_t motorCanDiagnostic[MOTOR_CAN_DIAG_WORDS];
// Mailbox acceptance is not delivery when NART is enabled. Protection replies
// remain owned here until TXOK, with bounded retries independent of the master's
// 5ms motion cycle. Ordinary position feedback keeps its best-effort semantics.
typedef struct {
    CAN_TxHeaderTypeDef header;
    uint8_t data[8];
} MotorCanPending;
static MotorCanPending motorCanQueue[24];
static volatile uint8_t motorCanRead, motorCanWrite;

#define MOTOR_CAN_PROTECTION_SLOTS 12U
#define MOTOR_CAN_PROTECTION_MAX_AGE_MS 100U
#define MOTOR_CAN_PROTECTION_MAX_ATTEMPTS 16U
typedef struct {
    MotorCanPending frame;
    uint32_t createdMs, retryAtMs;
    uint8_t attempts;
    uint16_t streamEpoch;
} MotorCanProtectionPending;
static MotorCanProtectionPending motorCanProtectionQueue[MOTOR_CAN_PROTECTION_SLOTS];
static volatile uint8_t motorCanProtectionRead, motorCanProtectionWrite;
static volatile uint32_t motorCanProtectionMailbox;
// One owned transmission also prevents reuse of a hardware-free mailbox whose
// previous RQCP interrupt has not yet been handled (normal or protected frame).
static volatile uint32_t motorCanTransmitMailbox;
static volatile bool motorCanProtectionAborting;
static volatile uint16_t motorCanStreamEpoch;
static volatile bool motorCanStreamFailed;
static uint32_t motorCanStreamRetries, motorCanStreamDrops;

static bool MotorCanIsStream(uint32_t id) {
    const uint8_t c=(uint8_t)(id & 0x7F);
    return c==0x47 || c==0x4A || c==0x4B;
}
static bool MotorCanStreamStale(const MotorCanProtectionPending* p) {
    return MotorCanIsStream(p->frame.header.StdId) && p->streamEpoch!=motorCanStreamEpoch;
}
static uint32_t MotorCanMaxAge(const MotorCanProtectionPending* p) {
    return MotorCanIsStream(p->frame.header.StdId) ? 40U : MOTOR_CAN_PROTECTION_MAX_AGE_MS;
}

static bool MotorCanIsProtection(uint32_t id)
{
    const uint8_t command = (uint8_t)(id & 0x7F);
    // stop_protocol.h / protection_protocol.h: motor-to-master replies only.
    return command == 0x0A || command == 0x0B || command == 0x32 ||
           command == 0x35 || command == 0x36;
}

static void MotorCanCopy(MotorCanPending* frame, CAN_TxHeaderTypeDef* header, const uint8_t* data)
{
    frame->header = *header;
    for (uint8_t i = 0; i < 8; ++i) frame->data[i] = data[i];
}

static bool MotorCanSameReply(const MotorCanPending* frame, CAN_TxHeaderTypeDef* header, const uint8_t* data)
{
    if (frame->header.StdId != header->StdId) return false;
    const uint8_t command = (uint8_t)(header->StdId & 0x7F);
    if (command == 0x47) {
        if (frame->data[0]!=data[0] || frame->data[1]!=data[1] || frame->data[4]!=data[4]) return false;
        // Only RUNNING is replaceable telemetry. Transaction phases must each
        // retain their own sequence and state until actual TXOK.
        if (data[4]==5) return true;
        return frame->data[2]==data[2] && frame->data[3]==data[3] && frame->data[6]==data[6];
    }
    if (command == 0x4A || command == 0x4B)
        return frame->data[0]==data[0] && frame->data[1]==data[1] &&
               (command==0x4A || frame->data[2]==data[2]);
    if (command == 0x32) {
        // Keep distinct parameters/transactions and BEGIN/COMMIT/SAVE replies.
        return frame->data[0] == data[0] && frame->data[2] == data[2] &&
               frame->data[3] == data[3] && (data[0] || frame->data[4] == data[4]);
    }
    // A lease ACK must never replace a motion-completion context, or vice versa.
    if (command == 0x35) return (frame->data[0] & 0xC0) == (data[0] & 0xC0);
    if (command == 0x23) return frame->data[5] == data[5] &&
        frame->data[6] == data[6] && frame->data[7] == data[7];
    return true;
}

static void MotorCanPopProtection(void)
{
    motorCanProtectionRead = (uint8_t)((motorCanProtectionRead + 1U) % MOTOR_CAN_PROTECTION_SLOTS);
    motorCanProtectionMailbox = 0;
    motorCanProtectionAborting = false;
}

static void MotorCanProbeDrop(const MotorCanProtectionPending* pending, uint32_t now)
{
    if ((pending->frame.header.StdId & 0x7F) != 0x36) return;
    ++motorCanDiagnostic[now - pending->createdMs >= MOTOR_CAN_PROTECTION_MAX_AGE_MS ?
                         MC_PROBE_AGE : MC_PROBE_TRIES];
}

// Keep the added counters in one copy instead of duplicating them in six ISR
// wrappers; the delivery/retry decisions below retain their existing behavior.
static __attribute__((noinline)) void MotorCanProtectionResult(CAN_HandleTypeDef* handle, uint8_t mailbox, bool success)
{
    if (handle != &hcan) return;
    const uint32_t irq = __get_PRIMASK(); __disable_irq();
    if (motorCanTransmitMailbox == (1UL << mailbox)) motorCanTransmitMailbox = 0;
    if (motorCanProtectionMailbox == (1UL << mailbox)) {
        MotorCanProtectionPending* pending = &motorCanProtectionQueue[motorCanProtectionRead];
        const uint32_t now = HAL_GetTick();
        if ((pending->frame.header.StdId & 0x7F) == 0x36) {
            if (success) {
                ++motorCanDiagnostic[MC_PROBE_TXOK];
                const uint32_t nonce = pending->frame.data[2] | ((uint32_t)pending->frame.data[3] << 8);
                motorCanDiagnostic[MC_NONCES] = (motorCanDiagnostic[MC_NONCES] & 0xFFFFU) | (nonce << 16);
                motorCanDiagnostic[MC_LAST_TX_MS] = now;
            } else {
                const uint32_t shift = mailbox * 2U;
                if (handle->ErrorCode & (HAL_CAN_ERROR_TX_ALST0 << shift)) ++motorCanDiagnostic[MC_PROBE_ALST];
                if (handle->ErrorCode & (HAL_CAN_ERROR_TX_TERR0 << shift)) ++motorCanDiagnostic[MC_PROBE_TERR];
            }
        }
        // Arbitration loss is normal CAN scheduling, not a failed bus transfer.
        // STREAM retries it on the next main-loop poll, with the same 40ms age
        // bound. Counting it as one of 16 bus errors starves high-node STARTED.
        const uint32_t shift = mailbox * 2U;
        const bool streamArbitration = !success && MotorCanIsStream(pending->frame.header.StdId) &&
            (handle->ErrorCode & (HAL_CAN_ERROR_TX_ALST0 << shift)) &&
            !(handle->ErrorCode & (HAL_CAN_ERROR_TX_TERR0 << shift));
        if (streamArbitration && pending->attempts) --pending->attempts;
        if (success || motorCanProtectionAborting || MotorCanStreamStale(pending) ||
            pending->attempts >= MOTOR_CAN_PROTECTION_MAX_ATTEMPTS ||
            now - pending->createdMs >= MotorCanMaxAge(pending)) {
            if (!success) {
                MotorCanProbeDrop(pending, now);
                if (MotorCanIsStream(pending->frame.header.StdId) && !MotorCanStreamStale(pending)) {
                    ++motorCanStreamDrops;
                    if ((pending->frame.header.StdId & 0x7F)==0x47) motorCanStreamFailed=true;
                }
            }
            MotorCanPopProtection();
        } else {
            motorCanProtectionMailbox = 0;
            // Genuine errors retain bounded 1..4ms backoff; arbitration can retry now.
            pending->retryAtMs = streamArbitration ? now : now + 1U + ((pending->attempts + boardConfig.canNodeId) & 3U);
            if (MotorCanIsStream(pending->frame.header.StdId)) ++motorCanStreamRetries;
        }
    }
    __set_PRIMASK(irq);
}

/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

// Preserve exactly the software receive routing: own-node commands,
// broadcast commands, and peer 0x0B protection events. Reject unrelated traffic
// in hardware before it can overflow FIFO0 while TIM4 has CPU priority.
void CAN_UpdateFilters(void) {
    CAN_FilterTypeDef f={0};
    f.FilterMode=CAN_FILTERMODE_IDMASK; f.FilterScale=CAN_FILTERSCALE_32BIT;
    f.FilterFIFOAssignment=CAN_RX_FIFO0; f.FilterActivation=ENABLE;
    f.FilterMaskIdLow=6U; // Standard data only (IDE/RTR).
    f.SlaveStartFilterBank=14;
    for (uint32_t bank=0;bank<3;++bank) {
        f.FilterBank=bank;
        f.FilterIdHigh=bank==0 ? ((boardConfig.canNodeId & 15U)<<12) : bank==1 ? 0 : (0x0BU<<5);
        f.FilterMaskIdHigh=bank==2 ? (0x7FU<<5) : (0x0FU<<12);
        if (HAL_CAN_ConfigFilter(&hcan,&f)!=HAL_OK) Error_Handler();
    }
}

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = ENABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
    CAN_UpdateFilters();

    HAL_CAN_Start(&hcan); //start CAN

    HAL_CAN_ActivateNotification(&hcan,
                                 CAN_IT_TX_MAILBOX_EMPTY |
                                 CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING |
                                 /* we probably only want this */
                                 CAN_IT_RX_FIFO0_FULL | CAN_IT_RX_FIFO1_FULL |
                                 CAN_IT_RX_FIFO0_OVERRUN | CAN_IT_RX_FIFO1_OVERRUN |
                                 CAN_IT_WAKEUP | CAN_IT_SLEEP_ACK |
                                 CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE |
                                 CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE |
                                 CAN_IT_ERROR);

/* Configure Transmission process */
    TxHeader.StdId = boardConfig.canNodeId;
    TxHeader.ExtId = 0x00;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8;
    TxHeader.TransmitGlobalTime = DISABLE;

  /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN GPIO Configuration
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_AFIO_REMAP_CAN1_2();

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(USB_HP_CAN1_TX_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USB_HP_CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
// Caller owns PRIMASK. No waiting and no unbounded allocation.
static bool MotorCanEnqueueReliable(CAN_TxHeaderTypeDef* pHeader,const uint8_t* data) {
        uint8_t index = motorCanProtectionRead;
        while (index != motorCanProtectionWrite) {
            // Never overwrite the frame whose hardware completion is pending.
            if (!(index == motorCanProtectionRead && motorCanProtectionMailbox) &&
                !MotorCanStreamStale(&motorCanProtectionQueue[index]) &&
                MotorCanSameReply(&motorCanProtectionQueue[index].frame, pHeader, data)) {
                MotorCanCopy(&motorCanProtectionQueue[index].frame, pHeader, data);
                if ((pHeader->StdId & 0x7F) == 0x36) ++motorCanDiagnostic[MC_PROBE_MERGED];
                return true;
            }
            index = (uint8_t)((index + 1U) % MOTOR_CAN_PROTECTION_SLOTS);
        }
        const uint8_t depthNow=(motorCanProtectionWrite+MOTOR_CAN_PROTECTION_SLOTS-motorCanProtectionRead)%MOTOR_CAN_PROTECTION_SLOTS;
        if (MotorCanIsStream(pHeader->StdId) && depthNow>=MOTOR_CAN_PROTECTION_SLOTS-4U) return false;
        const uint8_t next = (uint8_t)((motorCanProtectionWrite + 1U) % MOTOR_CAN_PROTECTION_SLOTS);
        if (next != motorCanProtectionRead) {
            MotorCanProtectionPending* pending = &motorCanProtectionQueue[motorCanProtectionWrite];
            MotorCanCopy(&pending->frame, pHeader, data);
            pending->createdMs = pending->retryAtMs = HAL_GetTick(); pending->attempts = 0;
            pending->streamEpoch=motorCanStreamEpoch;
            motorCanProtectionWrite = next;
            if ((pHeader->StdId & 0x7F) == 0x36) ++motorCanDiagnostic[MC_PROBE_QUEUED];
            const uint32_t depth = (next + MOTOR_CAN_PROTECTION_SLOTS - motorCanProtectionRead) % MOTOR_CAN_PROTECTION_SLOTS;
            if (depth > (motorCanDiagnostic[MC_QUEUE] >> 8)) motorCanDiagnostic[MC_QUEUE] = depth << 8;
            return true;
        } else if ((pHeader->StdId & 0x7F) == 0x36) {
            ++motorCanDiagnostic[MC_PROBE_FULL];
        }
    return false;
}

bool CAN_QueueStreamReply(CAN_TxHeaderTypeDef* header,const uint8_t* data) {
    const uint32_t irq=__get_PRIMASK();__disable_irq();
    // Keep three slots available for STOP/protection even during stream retries.
    const bool ok=MotorCanEnqueueReliable(header,data);
    if (!ok) { ++motorCanStreamDrops; if ((header->StdId & 0x7F)==0x47) motorCanStreamFailed=true; }
    __set_PRIMASK(irq);return ok;
}
void CAN_CancelStreamReplies(void) {
    const uint32_t irq=__get_PRIMASK();__disable_irq();
    ++motorCanStreamEpoch; motorCanStreamFailed=false;
    if (motorCanProtectionMailbox && MotorCanStreamStale(&motorCanProtectionQueue[motorCanProtectionRead])) {
        motorCanProtectionAborting=true;
        HAL_CAN_AbortTxRequest(&hcan,motorCanProtectionMailbox);
    }
    __set_PRIMASK(irq);
}
bool CAN_StreamReplyFault(void) { return motorCanStreamFailed; }
uint32_t CAN_StreamReplyDiagnostic(uint8_t field) { return field==0 ? motorCanStreamRetries : motorCanStreamDrops; }

void CAN_Send(CAN_TxHeaderTypeDef* pHeader, uint8_t* data)
{
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    // motion_completion_protocol.h: tagged 0x23 replies must not be replaced
    // by legacy position samples. Reuse the bounded TXOK/retry path.
    if (MotorCanIsProtection(pHeader->StdId) ||
        ((pHeader->StdId & 0x7F) == 0x23 && (data[4] & 0x80))) {
        MotorCanEnqueueReliable(pHeader,data);
        __set_PRIMASK(irq); return;
    }
    // Periodic 0x23 samples supersede unsent older samples instead of filling
    // the normal queue while a protection reply is being retried.
    if ((pHeader->StdId & 0x7F) == 0x23) {
        uint8_t index = motorCanRead;
        while (index != motorCanWrite) {
            if (motorCanQueue[index].header.StdId == pHeader->StdId) {
                MotorCanCopy(&motorCanQueue[index], pHeader, data);
                __set_PRIMASK(irq); return;
            }
            index = (uint8_t)((index + 1U) % 24U);
        }
    }
    uint8_t next = (uint8_t)((motorCanWrite + 1) % 24);
    if (next != motorCanRead) {
        MotorCanCopy(&motorCanQueue[motorCanWrite], pHeader, data);
        motorCanWrite = next;
    }
    __set_PRIMASK(irq);
}

bool CAN_TrySend(CAN_TxHeaderTypeDef* pHeader, const uint8_t* data)
{
    const uint32_t irq = __get_PRIMASK(); __disable_irq();
    if (motorCanTransmitMailbox || !HAL_CAN_GetTxMailboxesFreeLevel(&hcan)) {
        __set_PRIMASK(irq); return false;
    }
    uint32_t mailbox;
    const bool sent = HAL_CAN_AddTxMessage(&hcan, pHeader, (uint8_t*)data, &mailbox) == HAL_OK;
    if (sent) motorCanTransmitMailbox = mailbox;
    __set_PRIMASK(irq); return sent;
}

// Safety acknowledgements must not wait behind telemetry retry backoff.
static void MotorCanPrioritizeSafety(void) {
    if (motorCanProtectionMailbox) return;
    for (uint8_t i=motorCanProtectionRead;i!=motorCanProtectionWrite;i=(uint8_t)((i+1U)%MOTOR_CAN_PROTECTION_SLOTS)) {
        const MotorCanPending* f=&motorCanProtectionQueue[i].frame;
        const uint8_t command=(uint8_t)(f->header.StdId&0x7F);
        if (command==0x0A || (command==0x0B && f->data[4]!=0)) {
            if (i!=motorCanProtectionRead) {
                const MotorCanProtectionPending saved=motorCanProtectionQueue[motorCanProtectionRead];
                motorCanProtectionQueue[motorCanProtectionRead]=motorCanProtectionQueue[i];
                motorCanProtectionQueue[i]=saved;
            }
            break;
        }
    }
}

void CAN_PollTransmit(void)
{
    static uint32_t previousCycle;
    const uint32_t cycle = DWT->CYCCNT;
    const uint32_t gap = cycle - previousCycle;
    if (previousCycle && gap > motorCanDiagnostic[MC_POLL_GAP]) motorCanDiagnostic[MC_POLL_GAP] = gap;
    previousCycle = cycle;
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    const uint32_t now = HAL_GetTick();
    MotorCanPrioritizeSafety();
    while (motorCanProtectionRead != motorCanProtectionWrite) {
        MotorCanProtectionPending* pending = &motorCanProtectionQueue[motorCanProtectionRead];
        if (motorCanProtectionMailbox) {
            if (!motorCanProtectionAborting && (MotorCanStreamStale(pending) || now - pending->createdMs >= MotorCanMaxAge(pending))) {
                // An absent bus cannot retain a stale proof indefinitely.
                motorCanProtectionAborting = true;
                HAL_CAN_AbortTxRequest(&hcan, motorCanProtectionMailbox);
            }
            __set_PRIMASK(irq); return;
        }
        if (MotorCanStreamStale(pending) || now - pending->createdMs >= MotorCanMaxAge(pending) ||
            pending->attempts >= MOTOR_CAN_PROTECTION_MAX_ATTEMPTS) {
            MotorCanProbeDrop(pending, now);
            if (MotorCanIsStream(pending->frame.header.StdId) && !MotorCanStreamStale(pending)) {
                ++motorCanStreamDrops;
                if ((pending->frame.header.StdId & 0x7F)==0x47) motorCanStreamFailed=true;
            }
            MotorCanPopProtection(); continue;
        }
        if ((int32_t)(now - pending->retryAtMs) >= 0) {
            uint32_t mailbox;
            if (!motorCanTransmitMailbox && HAL_CAN_GetTxMailboxesFreeLevel(&hcan) &&
                HAL_CAN_AddTxMessage(&hcan, &pending->frame.header, pending->frame.data, &mailbox) == HAL_OK) {
                motorCanTransmitMailbox = motorCanProtectionMailbox = mailbox; ++pending->attempts;
            }
            __set_PRIMASK(irq); return;
        }
        break; // A deferred retry does not block ordinary position samples.
    }
    if (motorCanRead != motorCanWrite &&
        CAN_TrySend(&motorCanQueue[motorCanRead].header, motorCanQueue[motorCanRead].data))
        motorCanRead = (uint8_t)((motorCanRead + 1) % 24);
    __set_PRIMASK(irq);
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef* handle) { MotorCanProtectionResult(handle, 0, true); }
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef* handle) { MotorCanProtectionResult(handle, 1, true); }
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef* handle) { MotorCanProtectionResult(handle, 2, true); }
void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef* handle) { MotorCanProtectionResult(handle, 0, false); }
void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef* handle) { MotorCanProtectionResult(handle, 1, false); }
void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef* handle) { MotorCanProtectionResult(handle, 2, false); }

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef* handle)
{
    if (handle != &hcan) return;
    const uint32_t errors = handle->ErrorCode;
    motorCanDiagnostic[MC_RX_OVERRUN] |= errors & (HAL_CAN_ERROR_RX_FOV0 | HAL_CAN_ERROR_RX_FOV1);
    if (errors & (HAL_CAN_ERROR_TX_ALST0 | HAL_CAN_ERROR_TX_TERR0)) MotorCanProtectionResult(handle, 0, false);
    if (errors & (HAL_CAN_ERROR_TX_ALST1 | HAL_CAN_ERROR_TX_TERR1)) MotorCanProtectionResult(handle, 1, false);
    if (errors & (HAL_CAN_ERROR_TX_ALST2 | HAL_CAN_ERROR_TX_TERR2)) MotorCanProtectionResult(handle, 2, false);
    handle->ErrorCode &= ~(HAL_CAN_ERROR_TX_ALST0 | HAL_CAN_ERROR_TX_TERR0 |
                           HAL_CAN_ERROR_TX_ALST1 | HAL_CAN_ERROR_TX_TERR1 |
                           HAL_CAN_ERROR_TX_ALST2 | HAL_CAN_ERROR_TX_TERR2);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* CanHandle)
{
    /* Get RX message */
    if (HAL_CAN_GetRxMessage(CanHandle, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
        /* Reception Error */
        return;
    }

    if (RxHeader.IDE != CAN_ID_STD || RxHeader.RTR != CAN_RTR_DATA || RxHeader.DLC > 8) return;
    uint8_t id = (RxHeader.StdId >> 7); // 4Bits ID & 7Bits Msg
    uint8_t cmd = RxHeader.StdId & 0x7F; // 4Bits ID & 7Bits Msg
    if (id == 0 || id == boardConfig.canNodeId)
    {
        if (cmd == 0x34 && RxHeader.DLC == 8 && RxData[0] == 0) {
            ++motorCanDiagnostic[MC_QUERY_RX];
            motorCanDiagnostic[MC_NONCES] = (motorCanDiagnostic[MC_NONCES] & 0xFFFF0000U) |
                RxData[2] | ((uint32_t)RxData[3] << 8);
            motorCanDiagnostic[MC_LAST_RX_MS] = HAL_GetTick();
        }
        OnCanCmd(cmd, RxData, RxHeader.DLC);
    }
    else if (cmd == 0x0B && id >= 1 && id <= 7)
        OnProtectionPeerFrame(id, cmd, RxData, RxHeader.DLC);
}

// Only capture immutable evidence; never clear counters or change a queue here.
void CAN_CaptureDiagnostic(uint32_t* values)
{
    const uint32_t irq = __get_PRIMASK(); __disable_irq();
    for (uint32_t i = 0; i < MOTOR_CAN_DIAG_WORDS; ++i) values[i] = motorCanDiagnostic[i];
    values[MC_UPTIME] = HAL_GetTick(); values[MC_CPU_HZ] = SystemCoreClock;
    const uint32_t depth = (motorCanProtectionWrite + MOTOR_CAN_PROTECTION_SLOTS - motorCanProtectionRead) % MOTOR_CAN_PROTECTION_SLOTS;
    uint32_t command = 0, age = 0;
    if (depth) {
        const MotorCanProtectionPending* head = &motorCanProtectionQueue[motorCanProtectionRead];
        command = head->frame.header.StdId & 0x7F;
        age = values[MC_UPTIME] - head->createdMs;
    }
    values[MC_QUEUE] |= depth | (command << 16) |
        (((motorCanProtectionMailbox ? 1U : 0U) | (motorCanProtectionAborting ? 2U : 0U)) << 24);
    values[MC_HEAD_AGE] = age;
    __set_PRIMASK(irq);
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
