// #include "common_inc.h"


// // Used for response CAN message.
// static CAN_TxHeaderTypeDef txHeader =
//     {
//         .StdId = 0,
//         .ExtId = 0,
//         .IDE = CAN_ID_STD,
//         .RTR = CAN_RTR_DATA,
//         .DLC = 8,
//         .TransmitGlobalTime = DISABLE
//     };

// extern DummyRobot dummy;

// void OnCanMessage(CAN_context* canCtx, CAN_RxHeaderTypeDef* rxHeader, uint8_t* data)
// {
//     // Common CAN message callback, uses ID 32~0x7FF.
//     if (canCtx->handle->Instance == CAN1)
//     {
//         uint8_t id = rxHeader->StdId >> 7; // 4Bits ID & 7Bits Msg
//         uint8_t cmd = rxHeader->StdId & 0x7F; // 4Bits ID & 7Bits Msg

//         /*----------------------- ↓ Add Your CAN1 Packet Protocol Here ↓ ------------------------*/
//         switch (cmd)
//         {
//             case 0x23:
//                 dummy.motorJ[id]->UpdateAngleCallback(*(float*) (data), data[4]);
//                 break;
//             case 0x25:
//                  memcpy(&dummy.motorJ[id]->temperature, data, sizeof(uint32_t));//(uint32_t) (data);
//                 break;
//             default:
//                 break;
//         }

//         dummy.UpdateJointAnglesCallback();

//     } else if (canCtx->handle->Instance == CAN2)
//     {
//         /*----------------------- ↓ Add Your CAN2 Packet Protocol Here ↓ ------------------------*/
//     }
//     /*----------------------- ↑ Add Your Packet Protocol Here ↑ ------------------------*/
// }

#include "common_inc.h"


// Used for response CAN message.
static CAN_TxHeaderTypeDef txHeader =
    {
        .StdId = 0,
        .ExtId = 0,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = 8,
        .TransmitGlobalTime = DISABLE
    };

extern DummyRobot dummy;

void OnCanMessage(CAN_context* canCtx, CAN_RxHeaderTypeDef* rxHeader, uint8_t* data)
{
    // Common CAN message callback, uses ID 32~0x7FF.
    if (canCtx->handle->Instance == CAN1)
    {
        uint8_t id = rxHeader->StdId >> 7; // 4Bits ID & 7Bits Msg
        uint8_t cmd = rxHeader->StdId & 0x7F; // 4Bits ID & 7Bits Msg

        /* ── 夹爪电机（nodeID=7，id>6）的 CAN 回包处理 ──
         * 0x23：角度回传（data[0-3]=位置, data[4-5]=状态+errorCode, data[6]=flag1, data[7]=flag2）
         * 0x25：温度回传
         * hand 由原 hand2 重命名而来（StepHand，继承 CtrlStepMotor） */
        if (id == 7)
        {
            switch (cmd)
            {
                case 0x23:
                    dummy.hand->UpdateAngleCallback(
                        *(float*)(data),
                        (int16_t)((data[5] << 8) | data[4]),
                        data[6],
                        data[7]
                    );
                    break;
                case 0x25:
                    memcpy(&dummy.hand->temperature, data, sizeof(uint32_t));
                    break;
                default:
                    break;
            }
            return;
        }
        else if (id >= 1 && id <= 6)
        {
            /*----------------------- ↓ Add Your CAN1 Packet Protocol Here ↓ ------------------------*/
            switch (cmd)
            {
                case 0x23:
                    dummy.motorJ[id]->UpdateAngleCallback(
                        *(float*) (data),
                        (int16_t)((data[5] << 8) | data[4]),
                        data[6],
                        data[7]
                    );
                    break;
                case 0x25:
                     memcpy(&dummy.motorJ[id]->temperature, data, sizeof(uint32_t));//(uint32_t) (data);
                    break;
                default:
                    break;
            }
        }

        dummy.UpdateJointAnglesCallback();

    } else if (canCtx->handle->Instance == CAN2)
    {
        /*----------------------- ↓ Add Your CAN2 Packet Protocol Here ↓ ------------------------*/
    }
    /*----------------------- ↑ Add Your Packet Protocol Here ↑ ------------------------*/
}