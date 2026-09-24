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

// ── PID 合并回包状态 ──
// [v3.1] PID 缓存索引: [0]=空闲(ALL代理, 无电机), [1..6]=J1~J6(node=1~6), [7]=J7/地轨(node=7), [8]=夹爪(node=8)
// 为每个节点维护一个计数器，收到 0x28(Kp)/0x29(Kv)/0x2A(Ki)/0x2B(Kd) 之一就+1，
// 4个都到后 ISR 仅设 print_pending=1，主线程 (OledTask) poll 后清 0 + printf。
// 避免 ISR 内 printf 阻塞 UART DMA / 与 ASCII 线程产生 UART TX 竞争。
static int32_t _pid_tmp_kp[9] = {0}, _pid_tmp_kv[9] = {0};
static int32_t _pid_tmp_ki[9] = {0}, _pid_tmp_kd[9] = {0};
static uint8_t _pid_rcv_cnt[9] = {0};
// [PID 修复] 合并完成待打印标志（单字节读写在 ARM Cortex-M4 上原子）
static volatile uint8_t _pid_print_pending[9] = {0};
static volatile uint8_t _pid_pending_node[9] = {0};  // 触发合并的节点 ID（用于显示）

// 辅助：将 CAN nodeID 转为 motorDceKps[] 数组索引
static inline int _pid_node_to_idx(uint8_t nodeId) {
    if (nodeId == 7) return 7;      // [v3.1] J7/地轨（nodeID 9→7）
    if (nodeId == 8) return 8;      // [v3.1] 夹爪（避免与 J7 共享 idx=7）
    return (int)nodeId;              // 关节 1~6 → 索引 1~6
}

// CAN nodeID → 打印时显示的节点号（用户可见）
static inline int _pid_node_to_disp(uint8_t nodeId) {
    return (int)nodeId;
}

// [PID 修复] 主线程 poll PID 合并回包（由 OledTask 调用，详见 main.cpp）
// 遍历 9 个 slot；先清 pending 再读 tmp（防止主线程读期间 ISR 又写一轮）
extern "C" void PollPidPrint() {
    for (int idx = 1; idx <= 8; idx++) {
        if (_pid_print_pending[idx]) {
            _pid_print_pending[idx] = 0;  // 先清，再读（原子单字节写）
            printf("ok PID %d kp=%ld kv=%ld ki=%ld kd=%ld\r\n",
                   _pid_node_to_disp(_pid_pending_node[idx]),
                   (long)_pid_tmp_kp[idx], (long)_pid_tmp_kv[idx],
                   (long)_pid_tmp_ki[idx], (long)_pid_tmp_kd[idx]);
        }
    }
}

void OnCanMessage(CAN_context* canCtx, CAN_RxHeaderTypeDef* rxHeader, uint8_t* data)
{
    if (canCtx->handle->Instance == CAN1)
    {
        uint8_t id = rxHeader->StdId >> 7;
        uint8_t cmd = rxHeader->StdId & 0x7F; // 7Bits CMD (0x00~0x7F普通, 0x80~0xBF广播)

        // ── J7/地轨电机 (nodeID=7) 的 CAN 回包处理 ──
        if (id == 7)
        {
            switch (cmd)
            {
                case 0x23:
                    dummy.motorJ[7]->UpdateAngleCallback(*(float*)(data), data[4]);
                    // angle = 圈数 × 360°，丝杆 1605 = 5mm/圈
                    dummy.currentRailPos = dummy.motorJ[7]->angle / 360.0f * 5.0f;
                    break;
                case 0x25:
                    memcpy(&dummy.motorJ[7]->temperature, data, sizeof(uint32_t));
                    break;
                case 0x28:
                    {
                        int idx = _pid_node_to_idx(id);
                        _pid_tmp_kp[idx] = *(int32_t*)data;
                        _pid_rcv_cnt[idx]++;
                        if (_pid_rcv_cnt[idx] == 4) {
                            // [PID 修复] ISR 只设标志，主线程 poll 后 printf
                            _pid_print_pending[idx] = 1;
                            _pid_pending_node[idx] = id;
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
                            _pid_print_pending[idx] = 1;
                            _pid_pending_node[idx] = id;
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
                            _pid_print_pending[idx] = 1;
                            _pid_pending_node[idx] = id;
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
                            _pid_print_pending[idx] = 1;
                            _pid_pending_node[idx] = id;
                            _pid_rcv_cnt[idx] = 0;
                        }
                    }
                    break;
                case 0x2C:
                    // v3.0: 缓存加速度值 + 写入时间戳，移除 ISR 内 printf（消除与 ASCII 任务的 UART TX 竞争）
                    // [v3.1] J7/地轨 → runtime[7]（runtime[0] 留给 ALL 代理空闲槽）
                    dummy.jointAccRuntime[7] = *(float*)data;
                    dummy.jointAccLastUpdateMs[7] = HAL_GetTick();
                    break;
                case 0x2D:
                    // v3.0: 缓存电流限制值 + 时间戳，移除 ISR 内 printf
                    dummy.jointCurrentLimitRuntime[7] = *(float*)data;
                    dummy.jointCurrentLimitLastUpdateMs[7] = HAL_GetTick();
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
                            // [PID 修复] ISR 只设标志，主线程 poll 后 printf
                            _pid_print_pending[idx] = 1;
                            _pid_pending_node[idx] = id;
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
                            _pid_print_pending[idx] = 1;
                            _pid_pending_node[idx] = id;
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
                            _pid_print_pending[idx] = 1;
                            _pid_pending_node[idx] = id;
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
                            _pid_print_pending[idx] = 1;
                            _pid_pending_node[idx] = id;
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
                            // [PID 修复] ISR 只设标志，主线程 poll 后 printf
                            _pid_print_pending[idx] = 1;
                            _pid_pending_node[idx] = id;
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
                            _pid_print_pending[idx] = 1;
                            _pid_pending_node[idx] = id;
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
                            _pid_print_pending[idx] = 1;
                            _pid_pending_node[idx] = id;
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
                            _pid_print_pending[idx] = 1;
                            _pid_pending_node[idx] = id;
                            _pid_rcv_cnt[idx] = 0;
                        }
                    }
                    break;
                case 0x2C:
                    // v3.0: 缓存夹爪实际加速度 + 时间戳，移除 ISR 内 printf
                    // [v3.1] 夹爪迁移到 runtime[8]（runtime[7] 留给 J7/地轨）
                    dummy.jointAccRuntime[8] = *(float*)data;
                    dummy.jointAccLastUpdateMs[8] = HAL_GetTick();
                    break;
                case 0x2D:
                    // v3.0: 缓存夹爪电流限制 + 时间戳，移除 ISR 内 printf
                    dummy.jointCurrentLimitRuntime[8] = *(float*)data;
                    dummy.jointCurrentLimitLastUpdateMs[8] = HAL_GetTick();
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