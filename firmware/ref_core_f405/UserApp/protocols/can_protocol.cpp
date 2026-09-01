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
                    // v3.0: 缓存加速度值 + 写入时间戳，移除 ISR 内 printf（消除与 ASCII 任务的 UART TX 竞争）
                    dummy.jointAccRuntime[0] = *(float*)data;
                    dummy.jointAccLastUpdateMs[0] = HAL_GetTick();
                    break;
                case 0x2D:
                    // v3.0: 缓存电流限制值 + 时间戳，移除 ISR 内 printf
                    dummy.jointCurrentLimitRuntime[0] = *(float*)data;
                    dummy.jointCurrentLimitLastUpdateMs[0] = HAL_GetTick();
                    break;
                case 0x5A:
                    // 电机堵转广播: data[0]=nodeID, data[1]=1=TRIGGER/2=DONE/3=TIMEOUT/4=HEARTBEAT
                    {
                        uint8_t stallNodeId = data[0];
                        uint8_t stallCmd = data[1];
                        if (stallCmd == 1) {
                            // TRIGGER: 堵转刚触发
                            dummy.SetStallMode((int)stallNodeId);
                            printf("[STALL] node=%d TRIGGER\r\n", stallNodeId);
                        } else if (stallCmd == 2) {
                            // DONE: 回退完成
                            dummy.SetStallMode((int)stallNodeId);
                            printf("[STALL] node=%d done\r\n", stallNodeId);
                        } else if (stallCmd == 3) {
                            // TIMEOUT: 回退超时
                            dummy.SetStallMode((int)stallNodeId);
                            printf("[STALL] node=%d timeout\r\n", stallNodeId);
                        }
                        // state==4 (HEARTBEAT): RETREATING 期间心跳，忽略
                    }
                    break;
                case 0x5C:
                    // 0x5C 查询响应: data[0]=queryType(1=en/2=lock), data[1]=value(0/1)
                    {
                        uint8_t qtype = data[0];
                        uint8_t qval = data[1];
                        if (qtype == 1) {
                            printf("[STALL_STATUS] node=%d en=%d\r\n", id, qval);
                        } else if (qtype == 2) {
                            printf("[STALL_STATUS] node=%d lock=%d\r\n", id, qval);
                        }
                    }
                    break;
                case 0x7C:
                    // 旧堵转上报 (兼容旧固件)
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
                    // v3.0: 缓存加速度值 + 时间戳，移除 ISR 内 printf
                    dummy.jointAccRuntime[id] = *(float*)data;
                    dummy.jointAccLastUpdateMs[id] = HAL_GetTick();
                    break;
                case 0x2D:
                    // v3.0: 缓存电流限制值 + 时间戳，移除 ISR 内 printf
                    dummy.jointCurrentLimitRuntime[id] = *(float*)data;
                    dummy.jointCurrentLimitLastUpdateMs[id] = HAL_GetTick();
                    break;
                case 0x5A:
                    // 电机堵转广播: data[0]=nodeID, data[1]=1=TRIGGER/2=DONE/3=TIMEOUT/4=HEARTBEAT
                    {
                        uint8_t stallNodeId = data[0];
                        uint8_t stallCmd = data[1];
                        if (stallCmd == 1) {
                            dummy.SetStallMode((int)stallNodeId);
                            printf("[STALL] node=%d TRIGGER\r\n", stallNodeId);
                        } else if (stallCmd == 2) {
                            dummy.SetStallMode((int)stallNodeId);
                            printf("[STALL] node=%d done\r\n", stallNodeId);
                        } else if (stallCmd == 3) {
                            dummy.SetStallMode((int)stallNodeId);
                            printf("[STALL] node=%d timeout\r\n", stallNodeId);
                        }
                        // state==4 (HEARTBEAT): RETREATING 期间心跳，忽略
                    }
                    break;
                case 0x5C:
                    // 0x5C 查询响应: data[0]=queryType(1=en/2=lock), data[1]=value(0/1)
                    {
                        uint8_t qtype = data[0];
                        uint8_t qval = data[1];
                        if (qtype == 1) {
                            printf("[STALL_STATUS] node=%d en=%d\r\n", id, qval);
                        } else if (qtype == 2) {
                            printf("[STALL_STATUS] node=%d lock=%d\r\n", id, qval);
                        }
                    }
                    break;
                case 0x7C:
                    // 旧堵转上报 (兼容旧固件)
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
                    // v3.0: 缓存夹爪实际加速度 + 时间戳，移除 ISR 内 printf
                    dummy.jointAccRuntime[7] = *(float*)data;
                    dummy.jointAccLastUpdateMs[7] = HAL_GetTick();
                    break;
                case 0x2D:
                    // v3.0: 缓存夹爪电流限制 + 时间戳，移除 ISR 内 printf
                    dummy.jointCurrentLimitRuntime[7] = *(float*)data;
                    dummy.jointCurrentLimitLastUpdateMs[7] = HAL_GetTick();
                    break;
                case 0x5A:
                    // 夹爪固件不支持堵转检测，忽略
                    break;
                case 0x7C:
                    // 旧堵转上报 (兼容旧固件)
                    if (data[1] == 1)
                        dummy.SetStallMode(8);
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