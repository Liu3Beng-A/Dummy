/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdbool.h>

/* USER CODE BEGIN Includes */
extern CAN_HandleTypeDef hcan;
extern CAN_TxHeaderTypeDef TxHeader;
extern CAN_RxHeaderTypeDef RxHeader;
extern uint8_t TxData[8];
extern uint8_t RxData[8];
extern uint32_t TxMailbox;
extern float motor_temperature;
/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_CAN_Init(void);
void CAN_UpdateFilters(void);

/* USER CODE BEGIN Prototypes */
void CAN_Send(CAN_TxHeaderTypeDef* pHeader, uint8_t* data);
bool CAN_TrySend(CAN_TxHeaderTypeDef* pHeader, const uint8_t* data);
void CAN_PollTransmit(void);
bool CAN_QueueStreamReply(CAN_TxHeaderTypeDef* header,const uint8_t* data);
void CAN_CancelStreamReplies(void);
bool CAN_StreamReplyFault(void);
uint32_t CAN_StreamReplyDiagnostic(uint8_t field);
// max[normal,stream,cal], late[...], gap[...], encoder max, control max.
extern volatile uint32_t motorControlTiming[11];
// RAM-only diagnostics. The numbering mirrors ProtectionProtocol::CD_*;
// this C header is shared by the interrupt/main code and the C++ transport.
enum MotorCanDiagnosticIndex {
    MC_UPTIME = 2, MC_CPU_HZ, MC_CONTROL_MAX, MC_CONTROL_GAP,
    MC_CONTROL_LATE, MC_POLL_GAP, MC_RX_OVERRUN, MC_QUERY_RX,
    MC_PROBE_QUEUED, MC_PROBE_MERGED, MC_PROBE_TXOK, MC_PROBE_ALST,
    MC_PROBE_TERR, MC_PROBE_FULL, MC_PROBE_AGE, MC_PROBE_TRIES,
    MC_NONCES, MC_LAST_TX_MS, MC_LAST_RX_MS, MC_QUEUE, MC_HEAD_AGE,
    MC_BOOT_SESSION, MOTOR_CAN_DIAG_WORDS
};
extern volatile uint32_t motorCanDiagnostic[MOTOR_CAN_DIAG_WORDS];
void CAN_CaptureDiagnostic(uint32_t* values);
void OnProtectionPeerFrame(uint8_t node, uint8_t command, uint8_t* data, uint32_t length);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
