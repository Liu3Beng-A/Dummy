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
        uint8_t id = rxHeader->StdId >> 7; // 7Bits ID (0~127)
        uint8_t cmd = rxHeader->StdId & 0x7F; // 7Bits CMD (0x00~0x7F普通, 0x80~0xBF广播)

        // ── 地轨电机 (nodeID=9) 的 CAN 回包处理 ──
        if (id == 9)
        {
            switch (cmd)
            {
                case 0x23:
                    dummy.motorJ[0]->UpdateAngleCallback(*(float*)(data), data[4]);
                    // 更新地轨位置（角度 → mm）
                    // angle = 圈数 × 360°，丝杆 1605 = 5mm/圈
                    dummy.currentRailPos = dummy.motorJ[0]->angle / 360.0f * 5.0f;
                    break;
                case 0x25:
                    memcpy(&dummy.motorJ[0]->temperature, data, sizeof(uint32_t));
                    break;
                case 0x28:
                    memcpy(&dummy.motorDceKps[0], data, sizeof(int32_t));
                    printf("PID_RAIL Kp=%ld\r\n", (long)dummy.motorDceKps[0]);
                    break;
                case 0x29:
                    memcpy(&dummy.motorDceKvs[0], data, sizeof(int32_t));
                    printf("PID_RAIL Kv=%ld\r\n", (long)dummy.motorDceKvs[0]);
                    break;
                case 0x2A:
                    memcpy(&dummy.motorDceKis[0], data, sizeof(int32_t));
                    printf("PID_RAIL Ki=%ld\r\n", (long)dummy.motorDceKis[0]);
                    break;
                case 0x2B:
                    memcpy(&dummy.motorDceKds[0], data, sizeof(int32_t));
                    printf("PID_RAIL Kd=%ld\r\n", (long)dummy.motorDceKds[0]);
                    break;
                case 0x7C:
                    // 电机主动上报堵转
                    if (data[1] == 1)
                        dummy.SetStallMode(9);
                    break;
                default:
                    break;
            }
            dummy.UpdateJointAnglesCallback();
            return;
        }
        // ── 臂关节电机 (nodeID=1~6) 的 CAN 回包处理 ──
        else if (id >= 1 && id <= 6)
        {
            switch (cmd)
            {
                case 0x23:
                    dummy.motorJ[id]->UpdateAngleCallback(*(float*) (data), data[4]);
                    break;
                case 0x25:
                     memcpy(&dummy.motorJ[id]->temperature, data, sizeof(uint32_t));
                    break;
                case 0x28:
                    memcpy(&dummy.motorDceKps[id], data, sizeof(int32_t));
                    printf("PID_J%d Kp=%ld\r\n", id, (long)dummy.motorDceKps[id]);
                    break;
                case 0x29:
                    memcpy(&dummy.motorDceKvs[id], data, sizeof(int32_t));
                    printf("PID_J%d Kv=%ld\r\n", id, (long)dummy.motorDceKvs[id]);
                    break;
                case 0x2A:
                    memcpy(&dummy.motorDceKis[id], data, sizeof(int32_t));
                    printf("PID_J%d Ki=%ld\r\n", id, (long)dummy.motorDceKis[id]);
                    break;
                case 0x2B:
                    memcpy(&dummy.motorDceKds[id], data, sizeof(int32_t));
                    printf("PID_J%d Kd=%ld\r\n", id, (long)dummy.motorDceKds[id]);
                    break;
                case 0x7C:
                    // 电机主动上报堵转
                    if (data[1] == 1)
                        dummy.SetStallMode((int)id);
                    break;
                default:
                    break;
            }
            dummy.UpdateJointAnglesCallback();
        }
        // ── 夹爪电机 (nodeID=8) 的 CAN 回包处理 ──
        else if (id == 8)
        {
            switch (cmd)
            {
                case 0x23:
                    dummy.hand->UpdateAngleCallback(*(float*)(data), data[4]);
                    break;
                case 0x25:
                    memcpy(&dummy.hand->temperature, data, sizeof(uint32_t));
                    break;
                default:
                    break;
            }
        }

    } else if (canCtx->handle->Instance == CAN2)
    {
        /*----------------------- ↓ Add Your CAN2 Packet Protocol Here ↓ ------------------------*/
    }
    /*----------------------- ↑ Add Your Packet Protocol Here ↑ ------------------------*/
}