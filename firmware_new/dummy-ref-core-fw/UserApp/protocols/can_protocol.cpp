#include "common_inc.h"
#include "instances/protection_coordinator.h"


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

        /*----------------------- ↓ Add Your CAN1 Packet Protocol Here ↓ ------------------------*/
        switch (cmd)
        {
            case ProtectionProtocol::EVENT:
            case ProtectionProtocol::PARAM_REPLY:
            case ProtectionProtocol::CONTEXT_REPLY:
                if (rxHeader->DLC == 8) dummy.ProtectionCallback(id, cmd, data);
                return;
            case SoftStopProtocol::COMMAND:
                if (rxHeader->DLC == 8) dummy.SoftStopCallback(id, data);
                return;
            case 0x23:
            {
                if (id < 1 || id > 8 || rxHeader->DLC < 5) return;
                uint32_t bits; float position;
                memcpy(&bits, data, 4); memcpy(&position, data, 4);
                // Invalid encoder snapshots must not poison pose/IK caches.
                if ((bits & 0x7f800000U) == 0x7f800000U) return;
                if (id == 8)
                    dummy.hand->UpdateAngleCallback(position, data[4]);
                else
                    dummy.motorJ[id]->UpdateAngleCallback(position, data[4]);
                break;
            }
            case 0x25:
                if (id == 8)
                    memcpy(&dummy.hand->temperature, data, sizeof(uint32_t));
                else
                    memcpy(&dummy.motorJ[id]->temperature, data, sizeof(uint32_t));
                break;
            // [v2026-10-03] 查询端补齐：0x28~0x2D 回包路由
            //   0x28~0x2B: 4 帧 PID 合并 → motorPidRcvdCount[idx]，到 4 置 _pid_print_pending
            //   0x2C: jointAccRuntime[idx] + 时间戳
            //   0x2D: jointCurrentLimitRuntime[idx] + 时间戳
            //   节点 id 1~8 全部覆盖（含地轨 7 + 夹爪 8）；id==0 走电机端广播过滤，主控不响应
            case 0x28:  // QueryDceKp
            case 0x29:  // QueryDceKv
            case 0x2A:  // QueryDceKi
            case 0x2B:  // QueryDceKd
                if (id >= 1 && id <= 8) {
                    int idx = (id == 7) ? 7 : (id == 8) ? 8 : (int)id;
                    int32_t val = *(int32_t*)data;
                    if      (cmd == 0x28) dummy.motorDceKps[idx] = val;
                    else if (cmd == 0x29) dummy.motorDceKvs[idx] = val;
                    else if (cmd == 0x2A) dummy.motorDceKis[idx] = val;
                    else                  dummy.motorDceKds[idx] = val;
                    dummy.motorPidRcvdCount[idx]++;
                    if (dummy.motorPidRcvdCount[idx] >= 4) {
                        dummy.motorPidRcvdCount[idx] = 0;  // 重置合并
                        dummy.motorPidLastUpdateMs[idx] = HAL_GetTick();   // [v2026-10-04] 给同步等待用的时间戳
                        // [v2026-10-04] 不再在 ISR 置 _pid_print_pending / _pid_pending_node：
                        //   单节点路径由 ASCII #GET_PID <n> 时间戳同步打印（1 行）
                        //   ALL 模式由 ASCII #GET_PID 0 主动置 flag（避免单节点双发）
                    }
                }
                break;
            case 0x2C:  // QueryAcceleration
                if (id >= 1 && id <= 8 && rxHeader->DLC >= 4) {
                    float value;
                    uint32_t requestId = 0;
                    memcpy(&value, data, sizeof(value));
                    if (rxHeader->DLC == 8) memcpy(&requestId, data + 4, sizeof(requestId));
                    dummy.MotorAccelerationCallback(id, value, requestId);
                }
                break;
            case 0x2D:  // QueryCurrentLimit
                if (id >= 1 && id <= 8) {
                    int idx = (id == 7) ? 7 : (id == 8) ? 8 : (int)id;
                    dummy.jointCurrentLimitRuntime[idx] = *(float*)data;
                    dummy.jointCurrentLimitLastUpdateMs[idx] = HAL_GetTick();
                }
                break;
            default:
                break;
        }

        dummy.UpdateJointAnglesCallback();

    } else if (canCtx->handle->Instance == CAN2)
    {
        /*----------------------- ↓ Add Your CAN2 Packet Protocol Here ↓ ------------------------*/
    }
    /*----------------------- ↑ Add Your Packet Protocol Here ↑ ------------------------*/
}
