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

// ── PID 合并回包状态（修复 P2） ──
// motorDceKps[8] 索引: [0]=地轨(node=9), [1~6]=关节(node=1~6), [7]=夹爪(node=8)
// 为每个节点维护一个计数器，收到 0x28(Kp)/0x29(Kv)/0x2A(Ki)/0x2B(Kd) 之一就+1，
// 4个都到后 printf() 合并回包（走 _write 到 USB+UART4）。
static int32_t _pid_tmp_kp[8] = {0}, _pid_tmp_kv[8] = {0};
static int32_t _pid_tmp_ki[8] = {0}, _pid_tmp_kd[8] = {0};
static uint8_t _pid_rcv_cnt[8] = {0};

// 辅助：将 CAN nodeID 转为 motorDceKps[] 数组索引
static inline int _pid_node_to_idx(uint8_t nodeId) {
    if (nodeId == 9) return 0;      // 地轨
    if (nodeId == 8) return 7;      // 夹爪
    return (int)nodeId;              // 关节 1~6 → 索引 1~6
}

// CAN nodeID → 打印时显示的节点号（用户可见）
static inline int _pid_node_to_disp(uint8_t nodeId) {
    return (int)nodeId;
}

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
                    {
                        int idx = _pid_node_to_idx(id);
                        _pid_tmp_kp[idx] = *(int32_t*)data;
                        _pid_rcv_cnt[idx]++;
                        if (_pid_rcv_cnt[idx] == 4) {
                            printf("ok PID %d kp=%ld kv=%ld ki=%ld kd=%ld\r\n",
                                   _pid_node_to_disp(id),
                                   (long)_pid_tmp_kp[idx], (long)_pid_tmp_kv[idx],
                                   (long)_pid_tmp_ki[idx], (long)_pid_tmp_kd[idx]);
                            _pid_rcv_cnt[idx] = 0;
                        }
                    }
                    break;
                case 0x29:
                    {
                        int idx = _pid_node_to_idx(id);
                        _pid_tmp_kv[idx] = *(int32_t*)data;
                        _pid_rcv_cnt[idx]++;
                        if (_pid_rcv_cnt[idx] == 4) {
                            printf("ok PID %d kp=%ld kv=%ld ki=%ld kd=%ld\r\n",
                                   _pid_node_to_disp(id),
                                   (long)_pid_tmp_kp[idx], (long)_pid_tmp_kv[idx],
                                   (long)_pid_tmp_ki[idx], (long)_pid_tmp_kd[idx]);
                            _pid_rcv_cnt[idx] = 0;
                        }
                    }
                    break;
                case 0x2A:
                    {
                        int idx = _pid_node_to_idx(id);
                        _pid_tmp_ki[idx] = *(int32_t*)data;
                        _pid_rcv_cnt[idx]++;
                        if (_pid_rcv_cnt[idx] == 4) {
                            printf("ok PID %d kp=%ld kv=%ld ki=%ld kd=%ld\r\n",
                                   _pid_node_to_disp(id),
                                   (long)_pid_tmp_kp[idx], (long)_pid_tmp_kv[idx],
                                   (long)_pid_tmp_ki[idx], (long)_pid_tmp_kd[idx]);
                            _pid_rcv_cnt[idx] = 0;
                        }
                    }
                    break;
                case 0x2B:
                    {
                        int idx = _pid_node_to_idx(id);
                        _pid_tmp_kd[idx] = *(int32_t*)data;
                        _pid_rcv_cnt[idx]++;
                        if (_pid_rcv_cnt[idx] == 4) {
                            printf("ok PID %d kp=%ld kv=%ld ki=%ld kd=%ld\r\n",
                                   _pid_node_to_disp(id),
                                   (long)_pid_tmp_kp[idx], (long)_pid_tmp_kv[idx],
                                   (long)_pid_tmp_ki[idx], (long)_pid_tmp_kd[idx]);
                            _pid_rcv_cnt[idx] = 0;
                        }
                    }
                    break;
                case 0x2C:
                    printf("[ACC] MOTOR [9] = %.2f\r\n", *(float*)data);
                    break;
                case 0x2D:
                    printf("[I_LIMIT] MOTOR [9] = %.2f\r\n", *(float*)data);
                    break;
                case 0x7C:
                    // 重构阶段3+4 (2026-08-23): 0x7C 完整状态机上报
                    // data[0]=nodeID, data[1]=stallMode(0=IDLE/1=RETREATING/2=LOCKED), data[2~3]=i_q(mA), data[4]=enabled
                    if (data[1] == 2 /* STALL_LOCKED */) {
                        dummy.SetStallMode(0);  // 地轨 motorJ[0]
                    } else if (data[1] == 0 /* STALL_IDLE */) {
                        dummy.ClearStallMode(0);
                    }
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
                    {
                        int idx = _pid_node_to_idx(id);
                        _pid_tmp_kp[idx] = *(int32_t*)data;
                        _pid_rcv_cnt[idx]++;
                        if (_pid_rcv_cnt[idx] == 4) {
                            printf("ok PID %d kp=%ld kv=%ld ki=%ld kd=%ld\r\n",
                                   _pid_node_to_disp(id),
                                   (long)_pid_tmp_kp[idx], (long)_pid_tmp_kv[idx],
                                   (long)_pid_tmp_ki[idx], (long)_pid_tmp_kd[idx]);
                            _pid_rcv_cnt[idx] = 0;
                        }
                    }
                    break;
                case 0x29:
                    {
                        int idx = _pid_node_to_idx(id);
                        _pid_tmp_kv[idx] = *(int32_t*)data;
                        _pid_rcv_cnt[idx]++;
                        if (_pid_rcv_cnt[idx] == 4) {
                            printf("ok PID %d kp=%ld kv=%ld ki=%ld kd=%ld\r\n",
                                   _pid_node_to_disp(id),
                                   (long)_pid_tmp_kp[idx], (long)_pid_tmp_kv[idx],
                                   (long)_pid_tmp_ki[idx], (long)_pid_tmp_kd[idx]);
                            _pid_rcv_cnt[idx] = 0;
                        }
                    }
                    break;
                case 0x2A:
                    {
                        int idx = _pid_node_to_idx(id);
                        _pid_tmp_ki[idx] = *(int32_t*)data;
                        _pid_rcv_cnt[idx]++;
                        if (_pid_rcv_cnt[idx] == 4) {
                            printf("ok PID %d kp=%ld kv=%ld ki=%ld kd=%ld\r\n",
                                   _pid_node_to_disp(id),
                                   (long)_pid_tmp_kp[idx], (long)_pid_tmp_kv[idx],
                                   (long)_pid_tmp_ki[idx], (long)_pid_tmp_kd[idx]);
                            _pid_rcv_cnt[idx] = 0;
                        }
                    }
                    break;
                case 0x2B:
                    {
                        int idx = _pid_node_to_idx(id);
                        _pid_tmp_kd[idx] = *(int32_t*)data;
                        _pid_rcv_cnt[idx]++;
                        if (_pid_rcv_cnt[idx] == 4) {
                            printf("ok PID %d kp=%ld kv=%ld ki=%ld kd=%ld\r\n",
                                   _pid_node_to_disp(id),
                                   (long)_pid_tmp_kp[idx], (long)_pid_tmp_kv[idx],
                                   (long)_pid_tmp_ki[idx], (long)_pid_tmp_kd[idx]);
                            _pid_rcv_cnt[idx] = 0;
                        }
                    }
                    break;
                case 0x2C:
                    printf("[ACC] MOTOR [%d] = %.2f\r\n", id, *(float*)data);
                    break;
                case 0x2D:
                    printf("[I_LIMIT] MOTOR [%d] = %.2f\r\n", id, *(float*)data);
                    break;
                case 0x7C:
                    // 重构阶段3+4 (2026-08-23): 0x7C 完整状态机上报
                    // data[0]=nodeID, data[1]=stallMode(0=IDLE/1=RETREATING/2=LOCKED), data[2~3]=i_q(mA), data[4]=enabled
                    if (data[1] == 2 /* STALL_LOCKED */) {
                        dummy.SetStallMode((int)id);  // id 1~6 → motorJ[1~6]
                    } else if (data[1] == 0 /* STALL_IDLE */) {
                        dummy.ClearStallMode((int)id);
                    }
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
                case 0x28:
                    {
                        int idx = _pid_node_to_idx(id);
                        _pid_tmp_kp[idx] = *(int32_t*)data;
                        _pid_rcv_cnt[idx]++;
                        if (_pid_rcv_cnt[idx] == 4) {
                            printf("ok PID %d kp=%ld kv=%ld ki=%ld kd=%ld\r\n",
                                   _pid_node_to_disp(id),
                                   (long)_pid_tmp_kp[idx], (long)_pid_tmp_kv[idx],
                                   (long)_pid_tmp_ki[idx], (long)_pid_tmp_kd[idx]);
                            _pid_rcv_cnt[idx] = 0;
                        }
                    }
                    break;
                case 0x29:
                    {
                        int idx = _pid_node_to_idx(id);
                        _pid_tmp_kv[idx] = *(int32_t*)data;
                        _pid_rcv_cnt[idx]++;
                        if (_pid_rcv_cnt[idx] == 4) {
                            printf("ok PID %d kp=%ld kv=%ld ki=%ld kd=%ld\r\n",
                                   _pid_node_to_disp(id),
                                   (long)_pid_tmp_kp[idx], (long)_pid_tmp_kv[idx],
                                   (long)_pid_tmp_ki[idx], (long)_pid_tmp_kd[idx]);
                            _pid_rcv_cnt[idx] = 0;
                        }
                    }
                    break;
                case 0x2A:
                    {
                        int idx = _pid_node_to_idx(id);
                        _pid_tmp_ki[idx] = *(int32_t*)data;
                        _pid_rcv_cnt[idx]++;
                        if (_pid_rcv_cnt[idx] == 4) {
                            printf("ok PID %d kp=%ld kv=%ld ki=%ld kd=%ld\r\n",
                                   _pid_node_to_disp(id),
                                   (long)_pid_tmp_kp[idx], (long)_pid_tmp_kv[idx],
                                   (long)_pid_tmp_ki[idx], (long)_pid_tmp_kd[idx]);
                            _pid_rcv_cnt[idx] = 0;
                        }
                    }
                    break;
                case 0x2B:
                    {
                        int idx = _pid_node_to_idx(id);
                        _pid_tmp_kd[idx] = *(int32_t*)data;
                        _pid_rcv_cnt[idx]++;
                        if (_pid_rcv_cnt[idx] == 4) {
                            printf("ok PID %d kp=%ld kv=%ld ki=%ld kd=%ld\r\n",
                                   _pid_node_to_disp(id),
                                   (long)_pid_tmp_kp[idx], (long)_pid_tmp_kv[idx],
                                   (long)_pid_tmp_ki[idx], (long)_pid_tmp_kd[idx]);
                            _pid_rcv_cnt[idx] = 0;
                        }
                    }
                    break;
                case 0x2C:
                    printf("[ACC] MOTOR [8] = %.2f\r\n", *(float*)data);
                    break;
                case 0x2D:
                    printf("[I_LIMIT] MOTOR [8] = %.2f\r\n", *(float*)data);
                    break;
                case 0x7C:
                    // 夹爪不参与堵转（重构阶段3, 2026-08-23 决策 #19）
                    // 保留 0x7C 处理以便调试，但 SetStallMode(8) 无效（mask 不含夹爪）
                    (void)data;  // 静默忽略
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