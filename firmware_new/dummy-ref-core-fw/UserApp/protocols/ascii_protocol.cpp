#include "common_inc.h"

extern DummyRobot dummy;

static bool HandleStopCommand(const char* cmd, size_t len, StreamSink& response)
{
    std::string command(cmd, len);
    const size_t first = command.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return true;
    const size_t last = command.find_last_not_of(" \t\r\n");
    command = command.substr(first, last - first + 1);
    if (command == "!START")
    {
        if (dummy.IsCalibrationBusy()) { Respond(response, "error CALIBRATION busy"); return true; }
        if (dummy.IsProtectionParameterBusy()) { Respond(response, "error START busy"); return true; }
        dummy.RequestStart();
        Respond(response, dummy.IsEnabled() && !dummy.IsStopLocked() ? "Started ok" : "Start pending");
        return true;
    }
    if (command == "!DISABLE")
    {
        dummy.RequestDisable();
        Respond(response, "Disable pending");
        return true;
    }
    if (command == "!STOP")
    {
        dummy.RequestSoftStop();
        Respond(response, "Stopping ok");
        return true;
    }
    if (dummy.IsStopLocked())
    {
        Respond(response, dummy.IsCalibrationBusy() ? "error CALIBRATION busy" : "error STOP_LOCKED");
        return true;
    }
    if (dummy.HandleProtectionCommand(command.c_str(), command.size(), response)) return true;
    if (!dummy.IsEnabled() && (command[0] == '>' || command[0] == '@' || command[0] == '&'))
    {
        Respond(response, "error DISABLED");
        return true;
    }
    return false;
}

static void RespondStackStats(StreamSink& responseChannel)
{
    // Minimum unused stack since boot, in bytes (not current free stack).
    Respond(responseChannel, "ok STACK usb=%lu uart=%lu heap=%lu",
            (unsigned long) UsbServerStackFreeBytes(),
            (unsigned long) UartServerStackFreeBytes(),
            (unsigned long) xPortGetFreeHeapSize());
}

static bool HandleAccelerationSyncCommand(const char* cmd, size_t len, StreamSink& response)
{
    std::string command(cmd, len);
    const size_t first = command.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return false;
    const size_t last = command.find_last_not_of(" \t\r\n");
    command = command.substr(first, last - first + 1);
    if (command.rfind("#SYNC_ACC", 0) != 0) return false;
    if (command != "#SYNC_ACC") Respond(response, "error SYNC_ACC syntax: #SYNC_ACC");
    else if (!dummy.RequestMotorAccelerationSync(response)) Respond(response, "error SYNC_ACC busy");
    return true;
}

void OnUsbAsciiCmd(const char* _cmd, size_t _len, StreamSink &_responseChannel)
{
    if (HandleStopCommand(_cmd, _len, _responseChannel)) return;
    if (HandleAccelerationSyncCommand(_cmd, _len, _responseChannel)) return;
    uint8_t  i;
    /*---------------------------- ↓ Add Your CMDs Here ↓ -----------------------------*/
    if (_cmd[0] == '!' )
    {
        std::string s(_cmd);
        if (s.find("HOME") != std::string::npos)
        {
            Respond(_responseChannel, dummy.commandHandler.PushSystem("!HOME") == 0xFF ?
                    "error DISABLED" : "Home pending");
        } else if (s.find("CALIBRATION") != std::string::npos)
        {
            Respond(_responseChannel, dummy.commandHandler.PushSystem("!CALIBRATION") == 0xFF ?
                    "error STOP_LOCKED" : "Calibration pending");
        } else if (s.find("RESET") != std::string::npos)
        {
            Respond(_responseChannel, dummy.commandHandler.PushSystem("!RESET") == 0xFF ?
                    "error DISABLED" : "Reset pending");
        }
        // [Step 3] !HAND_* 12 条命令（运行时 4 条 + 配置 8 条；UART4 仅允许运行时 4 条）
        // 运行时 4 条：!HAND_O / !HAND_C / !HAND_POS / !HAND_I
        // 严格匹配：!HAND_O 是 !HAND_ONBOOT 的前缀，用 "== "||" rfind(... , 0) == 0" 避免 !HAND_ONBOOT 被抢先匹配
        else if (s == "!HAND_O" || s.rfind("!HAND_O ", 0) == 0)        // 张开方向电流（取负）
        {
            float A = 0;
            sscanf(_cmd, "!HAND_O %f", &A);
            if (A <= 0) A = dummy.hand->current;
            if (A > 2.5f) A = 2.5f;
            dummy.hand->SetCurrentSetPoint(-A);
            Respond(_responseChannel, "ok hand open I=%.2f", A);
        }
        else if (s == "!HAND_C" || s.rfind("!HAND_C ", 0) == 0)        // 闭合方向电流（取正）
        {
            float A = 0;
            sscanf(_cmd, "!HAND_C %f", &A);
            if (A <= 0) A = dummy.hand->current;
            if (A > 2.5f) A = 2.5f;
            dummy.hand->SetCurrentSetPoint(+A);
            Respond(_responseChannel, "ok hand close I=%.2f", A);
        }
        else if (s.rfind("!HAND_POS ", 0) == 0)                         // 位置命令 0~100
        {
            uint32_t pos;
            sscanf(_cmd, "!HAND_POS %lu", &pos);
            if (pos > 100) pos = 100;
            dummy.hand->SetAngleWithSpeedLimit(static_cast<float>(pos));
            Respond(_responseChannel, "ok hand pos=%lu", pos);
        }
        else if (s.rfind("!HAND_I ", 0) == 0)                          // 运行时电流幅值（不发 CAN 帧）
        {
            float cu;
            sscanf(_cmd, "!HAND_I %f", &cu);
            if (cu > 0 && cu <= 2.5f)
            {
                dummy.hand->current = cu;
                Respond(_responseChannel, "ok hand I=%.2f", cu);
            }
            // 上限 2.5A / ≤0：静默拒绝，不响应（与决策 §2.1 一致）
        }
        // 配置 8 条：UART4 禁止；USB 调基类方法（EEPROM 持久化由基类 canBuf[4] 决定）
        else if (s.rfind("!HAND_VEL ", 0) == 0)
        {
            float v;
            sscanf(_cmd, "!HAND_VEL %f", &v);
            dummy.hand->SetVelocityLimit(v);
            Respond(_responseChannel, "ok hand set VEL=%.2f", v);
        }
        else if (s.rfind("!HAND_ACC ", 0) == 0)
        {
            float a;
            sscanf(_cmd, "!HAND_ACC %f", &a);
            dummy.hand->SetAcceleration(a);
            Respond(_responseChannel, "ok hand set ACC=%.2f", a);
        }
        else if (s.rfind("!HAND_KP ", 0) == 0)
        {
            int32_t p;
            sscanf(_cmd, "!HAND_KP %ld", &p);
            dummy.hand->SetDceKp(p);
            Respond(_responseChannel, "ok hand set KP=%ld", p);
        }
        else if (s.rfind("!HAND_KV ", 0) == 0)
        {
            int32_t v;
            sscanf(_cmd, "!HAND_KV %ld", &v);
            dummy.hand->SetDceKv(v);
            Respond(_responseChannel, "ok hand set KV=%ld", v);
        }
        else if (s.rfind("!HAND_KI ", 0) == 0)
        {
            int32_t i;
            sscanf(_cmd, "!HAND_KI %ld", &i);
            dummy.hand->SetDceKi(i);
            Respond(_responseChannel, "ok hand set KI=%ld", i);
        }
        else if (s.rfind("!HAND_KD ", 0) == 0)
        {
            int32_t d;
            sscanf(_cmd, "!HAND_KD %ld", &d);
            dummy.hand->SetDceKd(d);
            Respond(_responseChannel, "ok hand set KD=%ld", d);
        }
        else if (s == "!HAND_ONBOOT" || s.rfind("!HAND_ONBOOT ", 0) == 0)
        {
            uint32_t val = 1;  // 缺省 = 1（使能上电自启）；<0/1> 显式传参
            sscanf(_cmd, "!HAND_ONBOOT %lu", &val);
            dummy.hand->SetEnableOnBoot(val != 0);
            Respond(_responseChannel, "ok hand set ONBOOT=%lu", val);
        }
        else if (s == "!HAND_ERASE")
        {
            dummy.hand->EraseConfigs();
            Respond(_responseChannel, "ok hand set ERASE=1");
        }
    } else if (_cmd[0] == '#')
    {
        std::string s(_cmd);
        if (s == "#STACK")
        {
            RespondStackStats(_responseChannel);
        } else if (s.find("GETJPOS") != std::string::npos)
        {
            Respond(_responseChannel, "ok %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
                    dummy.currentJoints.a[0], dummy.currentJoints.a[1],
                    dummy.currentJoints.a[2], dummy.currentJoints.a[3],
                    dummy.currentJoints.a[4], dummy.currentJoints.a[5],
                    dummy.currentRailPos);  // [NEW] 第 7 位，Rail (mm)
        } else if (s.find("GETRAILPOS") != std::string::npos)
        {
            Respond(_responseChannel, "ok %.2f", dummy.currentRailPos);
        } else if (s.find("GETLPOS") != std::string::npos)
        {
            dummy.UpdateJointPose6D();
            Respond(_responseChannel, "ok %.2f %.2f %.2f %.2f %.2f %.2f",
                    dummy.currentPose6D.X, dummy.currentPose6D.Y,
                    dummy.currentPose6D.Z, dummy.currentPose6D.A,
                    dummy.currentPose6D.B, dummy.currentPose6D.C);
        }
        // [v2026-10-03] 查询端补齐：#GET_PID 1~8 + 0=ALL
        //   1~8 单节点 30ms 同步等（osDelay(1) 轮询 motorPidRcvdCount[idx] >= 4）
        //   0=ALL 立即 Respond "ALL dispatched"，32 帧回包由 OledTask PollPidPrint 分行打印
        else if (s.find("GET_PID") != std::string::npos)
        {
            uint32_t node;
            sscanf(_cmd, "#GET_PID %lu", &node);
            if (node == 0) {
                // ALL 模式：8 节点 × 4 帧 = 32 帧，立即返回不阻塞
                // [v2026-10-04] ASCII 主动设 _pid_print_pending（ISR 已不再设）：让 OledTask PollPidPrint 异步打印
                for (int i = 1; i <= 7; i++) {
                    dummy.motorJ[i]->QueryDceKp();
                    dummy.motorJ[i]->QueryDceKv();
                    dummy.motorJ[i]->QueryDceKi();
                    dummy.motorJ[i]->QueryDceKd();
                    dummy._pid_print_pending[i] = 1;       // ASCII 主动置 flag → OledTask PollPidPrint
                    dummy._pid_pending_node[i] = i;        // 节点号用于 printf (idx → 实际 nodeID)
                }
                dummy.hand->QueryDceKp();
                dummy.hand->QueryDceKv();
                dummy.hand->QueryDceKi();
                dummy.hand->QueryDceKd();
                dummy._pid_print_pending[8] = 1;           // hand = idx 8
                dummy._pid_pending_node[8] = 8;
                Respond(_responseChannel, "ok GET_PID ALL dispatched (8 nodes x 4 = 32 frames; results printed by OledTask PollPidPrint in 1-2 s)");
            } else if (node >= 1 && node <= 8) {
                int idx = (node == 7) ? 7 : (node == 8) ? 8 : (int)node;
                if (node == 8) {
                    dummy.hand->QueryDceKp(); dummy.hand->QueryDceKv();
                    dummy.hand->QueryDceKi(); dummy.hand->QueryDceKd();
                } else {
                    dummy.motorJ[node]->QueryDceKp(); dummy.motorJ[node]->QueryDceKv();
                    dummy.motorJ[node]->QueryDceKi(); dummy.motorJ[node]->QueryDceKd();
                }
                // [v2026-10-04] 改用时间戳模式，与 ACC/I_LIMIT 同模式；不再共享 _pid_print_pending
                uint32_t old_ms = dummy.motorPidLastUpdateMs[idx];
                uint32_t waited = 0;
                while (dummy.motorPidLastUpdateMs[idx] == old_ms && waited < 10) {  // 30ms → 10ms
                    osDelay(1); waited++;
                }
                if (dummy.motorPidLastUpdateMs[idx] != old_ms) {
                    Respond(_responseChannel, "ok PID [%lu] kp=%ld kv=%ld ki=%ld kd=%ld%s",
                            node, (long)dummy.motorDceKps[idx], (long)dummy.motorDceKvs[idx],
                            (long)dummy.motorDceKis[idx], (long)dummy.motorDceKds[idx],
                            node == 8 ? " (HAND)" : (node == 7 ? " (rail)" : ""));
                } else {
                    uint8_t got = dummy.motorPidRcvdCount[idx];
                    Respond(_responseChannel, "error GET_PID [%lu] timeout (got %u/4, %lu ms)", node, got, waited);
                }
            } else {
                Respond(_responseChannel, "error GET_PID [%lu] wrong (use 7=rail, 1~6=joints, 8=hand, 0=ALL)", node);
            }
        } else if (s.find("SET_DCE_KP") != std::string::npos)
        {
            uint32_t kp;
            uint32_t node;
            sscanf(_cmd, "#SET_DCE_KP %lu %lu", &node, &kp);
            if (node >= 1 && node <= 7){  // [Step 4] 范围 1~6 → 1~7（8 在下面单独处理）
                dummy.motorJ[node]->SetDceKp(kp);
                Respond(_responseChannel, "ok SET MOTOR [%lu] DCE_KP [%lu]", node, kp);
            }
            else if (node == 8){           // [Step 4] 夹爪走 dummy.hand
                dummy.hand->SetDceKp(kp);
                Respond(_responseChannel, "ok SET MOTOR [%lu] DCE_KP [%lu] (HAND)", node, kp);
            }
            else {
                Respond(_responseChannel, "error SET MOTOR [%lu] DCE_KP [%lu] is wrong (range 1~8, reject 0)", node, kp);
            }
        } else if (s.find("SET_DCE_KI") != std::string::npos)
        {
            uint32_t ki;
            uint32_t node;
            sscanf(_cmd, "#SET_DCE_KI %lu %lu", &node, &ki);
            if (node >= 1 && node <= 7){  // [Step 4] 范围 1~6 → 1~7（8 在下面单独处理）
                dummy.motorJ[node]->SetDceKi(ki);
                Respond(_responseChannel, "ok SET MOTOR [%lu] DCE_KI [%lu]", node, ki);
            }
            else if (node == 8){           // [Step 4] 夹爪走 dummy.hand
                dummy.hand->SetDceKi(ki);
                Respond(_responseChannel, "ok SET MOTOR [%lu] DCE_KI [%lu] (HAND)", node, ki);
            }
            else {
                Respond(_responseChannel, "error SET MOTOR [%lu] DCE_KI [%lu] is wrong (range 1~8, reject 0)", node, ki);
            }
        } else if (s.find("SET_DCE_KD") != std::string::npos)
        {
            uint32_t kd;
            uint32_t node;
            sscanf(_cmd, "#SET_DCE_KD %lu %lu", &node, &kd);
            if (node >= 1 && node <= 7){  // [Step 4] 范围 1~6 → 1~7（8 在下面单独处理）
                dummy.motorJ[node]->SetDceKd(kd);
                Respond(_responseChannel, "ok SET MOTOR [%lu] DCE_KD [%lu]", node, kd);
            }
            else if (node == 8){           // [Step 4] 夹爪走 dummy.hand
                dummy.hand->SetDceKd(kd);
                Respond(_responseChannel, "ok SET MOTOR [%lu] DCE_KD [%lu] (HAND)", node, kd);
            }
            else {
                Respond(_responseChannel, "error SET MOTOR [%lu] DCE_KD [%lu] is wrong (range 1~8, reject 0)", node, kd);
            }
        }
        // [Step 4] 新增 #SET_DCE_KV / #I_LIMIT_J / #ACC_J 3 条 # 配置类命令
        else if (s.find("SET_DCE_KV") != std::string::npos)
        {
            int32_t kv;
            uint32_t node;
            sscanf(_cmd, "#SET_DCE_KV %lu %ld", &node, &kv);
            if (node >= 1 && node <= 7){
                dummy.motorJ[node]->SetDceKv(kv);
                Respond(_responseChannel, "ok SET MOTOR [%lu] DCE_KV [%ld]", node, kv);
            }
            else if (node == 8){
                dummy.hand->SetDceKv(kv);
                Respond(_responseChannel, "ok SET MOTOR [%lu] DCE_KV [%ld] (HAND)", node, kv);
            }
            else {
                Respond(_responseChannel, "error SET MOTOR [%lu] DCE_KV [%ld] is wrong (range 1~8, reject 0)", node, kv);
            }
        }
        // [v2026-10-03] 查询端补齐：#SET_PID 1~8（写 EEPROM 4 个 PID 值）
        //   放在 SET_DCE_KV 之后、I_LIMIT_J 之前（防 SET_DCE_K* 子串抢先匹配）
        else if (s.find("SET_PID") != std::string::npos)
        {
            uint32_t node, kp, kv, ki, kd;
            sscanf(_cmd, "#SET_PID %lu %lu %lu %lu %lu", &node, &kp, &kv, &ki, &kd);
            if (node >= 1 && node <= 7) {
                dummy.motorJ[node]->SetDceKp(kp);
                dummy.motorJ[node]->SetDceKv(kv);
                dummy.motorJ[node]->SetDceKi(ki);
                dummy.motorJ[node]->SetDceKd(kd);
                Respond(_responseChannel, "ok SET PID [%lu] kp=%lu kv=%lu ki=%lu kd=%lu%s",
                        node, kp, kv, ki, kd, node == 7 ? " (rail)" : "");
            } else if (node == 8) {
                dummy.hand->SetDceKp(kp);
                dummy.hand->SetDceKv(kv);
                dummy.hand->SetDceKi(ki);
                dummy.hand->SetDceKd(kd);
                Respond(_responseChannel, "ok SET PID [8] kp=%lu kv=%lu ki=%lu kd=%lu (HAND)", kp, kv, ki, kd);
            } else {
                Respond(_responseChannel, "error SET PID [%lu] wrong (use 7=rail, 1~6=joints, 8=hand, 0=ALL not allowed)", node);
            }
        }
        else if (s.find("I_LIMIT_J") != std::string::npos)
        {
            float current;
            uint32_t node;
            char saveFlag = 0;
            int ret = sscanf(_cmd, "#I_LIMIT_J %lu %f %c", &node, &current, &saveFlag);
            if (ret >= 2) {
                bool persist = (saveFlag == '&');   // [v2026-10-04] & 决定是否写 EEPROM（与 ref 对齐，默认不写）
                if (node >= 1 && node <= 7){
                    dummy.motorJ[node]->SetCurrentLimit_persist(current, persist);
                    Respond(_responseChannel, "ok SET MOTOR [%lu] I_LIMIT [%.2f]%s", node, current,
                            persist ? " (saved)" : "");
                }
                else if (node == 8){
                    dummy.hand->SetCurrentLimit_persist(current, persist);
                    Respond(_responseChannel, "ok SET MOTOR [%lu] I_LIMIT [%.2f] (HAND)%s", node, current,
                            persist ? " (saved)" : "");
                }
                else {
                    Respond(_responseChannel, "error SET MOTOR [%lu] I_LIMIT [%.2f] is wrong (range 1~8, reject 0)", node, current);
                }
            } else {
                Respond(_responseChannel, "error I_LIMIT_J parse failed. Use: #I_LIMIT_J <node> <A> [&]");
            }
        }
        // [v2026-10-03] 查询端补齐：#GETJACC 1~8 + 0=ALL
        //   1~8 单节点 30ms 同步等 jointAccLastUpdateMs[idx] 变化
        //   0=ALL 调 SyncAllMotorAcceleration() 异步触发 8 个 0x2C
        //   放在 I_LIMIT_J 之后、ACC_J 之前
        else if (s.find("GETJACC") != std::string::npos)
        {
            uint32_t node;
            sscanf(_cmd, "#GETJACC %lu", &node);
            if (node == 0) {
                dummy.SyncAllMotorAcceleration();
                Respond(_responseChannel, "ok GETJACC ALL dispatched (8 nodes; results printed by OledTask in <100 ms)");
            } else if (node >= 1 && node <= 8) {
                int idx = (node == 7) ? 7 : (node == 8) ? 8 : (int)node;
                uint32_t old_ms = dummy.jointAccLastUpdateMs[idx];
                if (node == 8) dummy.hand->QueryAcceleration();
                else           dummy.motorJ[node]->QueryAcceleration();
                uint32_t waited = 0;
                while (dummy.jointAccLastUpdateMs[idx] == old_ms && waited < 30) {
                    osDelay(1); waited++;
                }
                if (dummy.jointAccLastUpdateMs[idx] != old_ms) {
                    Respond(_responseChannel, "ok ACC [%lu] = %.2f r/s^2%s", node, dummy.jointAccRuntime[idx],
                            node == 8 ? " (HAND)" : (node == 7 ? " (rail)" : ""));
                } else {
                    Respond(_responseChannel, "error GETJACC [%lu] timeout (%lu ms)", node, waited);
                }
            } else {
                Respond(_responseChannel, "error GETJACC [%lu] wrong (use 7=rail, 1~6=joints, 8=hand, 0=ALL)", node);
            }
        }
        // [v2026-10-03] 查询端补齐：#GETI 1~8 + 0=ALL
        //   ⚠️ 必须放 ACC_J 分支之前（防 #GETI 1 被 ACC_J 抢先匹配 — 实际不会但按方案顺序约束）
        else if (s.find("GETI") != std::string::npos)
        {
            uint32_t node;
            sscanf(_cmd, "#GETI %lu", &node);
            if (node == 0) {
                dummy.SyncAllMotorCurrentLimit();
                Respond(_responseChannel, "ok GETI ALL dispatched (8 nodes, async; results printed by OledTask in <100 ms)");
            } else if (node >= 1 && node <= 8) {
                int idx = (node == 7) ? 7 : (node == 8) ? 8 : (int)node;
                uint32_t old_ms = dummy.jointCurrentLimitLastUpdateMs[idx];
                if (node == 8) dummy.hand->QueryCurrentLimit();
                else           dummy.motorJ[node]->QueryCurrentLimit();
                uint32_t waited = 0;
                while (dummy.jointCurrentLimitLastUpdateMs[idx] == old_ms && waited < 30) {
                    osDelay(1); waited++;
                }
                if (dummy.jointCurrentLimitLastUpdateMs[idx] != old_ms) {
                    Respond(_responseChannel, "ok I_LIMIT [%lu] = %.2f A%s", node, dummy.jointCurrentLimitRuntime[idx],
                            node == 8 ? " (HAND)" : (node == 7 ? " (rail)" : ""));
                } else {
                    Respond(_responseChannel, "error GETI [%lu] timeout (%lu ms)", node, waited);
                }
            } else {
                Respond(_responseChannel, "error GETI [%lu] wrong (use 7=rail, 1~6=joints, 8=hand, 0=ALL)", node);
            }
        }
        else if (s.find("ACC_J") != std::string::npos)
        {
            float acc;
            uint32_t node;
            char saveFlag = 0;
            int ret = sscanf(_cmd, "#ACC_J %lu %f %c", &node, &acc, &saveFlag);
            if (ret >= 2) {
                bool persist = (saveFlag == '&');   // [v2026-10-04] & 决定是否写 EEPROM（与 ref 对齐）
                if (node >= 1 && node <= 7){
                    dummy.motorJ[node]->SetAcceleration_persist(acc, persist);
                    Respond(_responseChannel, "ok SET MOTOR [%lu] ACC [%.2f]%s", node, acc,
                            persist ? " (saved)" : "");
                }
                else if (node == 8){
                    dummy.hand->SetAcceleration_persist(acc, persist);
                    Respond(_responseChannel, "ok SET MOTOR [%lu] ACC [%.2f] (HAND)%s", node, acc,
                            persist ? " (saved)" : "");
                }
                else {
                    Respond(_responseChannel, "error SET MOTOR [%lu] ACC [%.2f] is wrong (range 1~8, reject 0)", node, acc);
                }
            } else {
                Respond(_responseChannel, "error ACC_J parse failed. Use: #ACC_J <node> <r/s^2> [&]");
            }
        } else if (s.find("REBOOT") != std::string::npos)
        {
            uint32_t node;
            sscanf(_cmd, "#REBOOT %lu", &node);
            if (node >= 1 && node <= 7)                  // J1~J7（含地轨）
            {
                dummy.motorJ[node]->Reboot();
                Respond(_responseChannel, "ok REBOOT MOTOR [%lu]", node);
            }
            else if (node == 8)                          // [Step 4] 夹爪独立 reboot
            {
                dummy.hand->Reboot();
                Respond(_responseChannel, "ok REBOOT MOTOR [%lu] (HAND)", node);
            }
            else if (node == 0)                          // ALL 代理（J1~J7 + 夹爪 1 帧广播）
            {
                Respond(_responseChannel, "ok REBOOT ALL MOTORS + HAND");
                dummy.RebootAllMotors();
            }
            else
            {
                Respond(_responseChannel, "error REBOOT MOTOR [%lu] is wrong (range 1~8, 0=ALL)", node);
            }
        }else if (s.find("CMDMODE") != std::string::npos)
        {
            uint32_t mode;
            sscanf(_cmd, "#CMDMODE %lu", &mode);
            dummy.SetCommandMode(mode);
            Respond(_responseChannel, "ok Set command mode to [%lu]", mode);
        } else
            Respond(_responseChannel, "ok");
    } else if (_cmd[0] == '>' || _cmd[0] == '@' || _cmd[0] == '&')
    {
        uint32_t freeSize = dummy.commandHandler.Push(_cmd);
        Respond(_responseChannel, "%d", freeSize);
    }

/*---------------------------- ↑ Add Your CMDs Here ↑ -----------------------------*/
}


void OnUart4AsciiCmd(const char* _cmd, size_t _len, StreamSink &_responseChannel)
{
    if (HandleStopCommand(_cmd, _len, _responseChannel)) return;
    /*---------------------------- ↓ Add Your CMDs Here ↓ -----------------------------*/
    if (_cmd[0] == '!' || !dummy.IsEnabled())
    {
        std::string s(_cmd);
        if (s.find("HOME") != std::string::npos)
        {
            Respond(_responseChannel, dummy.commandHandler.PushSystem("!HOME") == 0xFF ?
                    "error DISABLED" : "Home pending");
        } else if (s.find("CALIBRATION") != std::string::npos)
        {
            Respond(_responseChannel, dummy.commandHandler.PushSystem("!CALIBRATION") == 0xFF ?
                    "error STOP_LOCKED" : "Calibration pending");
        } else if (s.find("RESET") != std::string::npos)
        {
            Respond(_responseChannel, dummy.commandHandler.PushSystem("!RESET") == 0xFF ?
                    "error DISABLED" : "Reset pending");
        }
        // [Step 3] UART4 仅允许 !HAND_O/C/POS/I 4 条运行时（顺序匹配避免 !HAND_ONBOOT 抢先匹配 !HAND_O）
        else if (s == "!HAND_O" || s.rfind("!HAND_O ", 0) == 0)
        {
            float A = 0;
            sscanf(_cmd, "!HAND_O %f", &A);
            if (A <= 0) A = dummy.hand->current;
            if (A > 2.5f) A = 2.5f;
            dummy.hand->SetCurrentSetPoint(-A);
            Respond(_responseChannel, "ok hand open I=%.2f", A);
        }
        else if (s == "!HAND_C" || s.rfind("!HAND_C ", 0) == 0)
        {
            float A = 0;
            sscanf(_cmd, "!HAND_C %f", &A);
            if (A <= 0) A = dummy.hand->current;
            if (A > 2.5f) A = 2.5f;
            dummy.hand->SetCurrentSetPoint(+A);
            Respond(_responseChannel, "ok hand close I=%.2f", A);
        }
        else if (s.rfind("!HAND_POS ", 0) == 0)
        {
            uint32_t pos;
            sscanf(_cmd, "!HAND_POS %lu", &pos);
            if (pos > 100) pos = 100;
            dummy.hand->SetAngleWithSpeedLimit(static_cast<float>(pos));
            Respond(_responseChannel, "ok hand pos=%lu", pos);
        }
        else if (s.rfind("!HAND_I ", 0) == 0)
        {
            float cu;
            sscanf(_cmd, "!HAND_I %f", &cu);
            if (cu > 0 && cu <= 2.5f)
            {
                dummy.hand->current = cu;
                Respond(_responseChannel, "ok hand I=%.2f", cu);
            }
            // 上限 2.5A / ≤0：静默拒绝，不响应
        }
        // [阻塞 3 关键] 其余 !HAND_*（8 条配置类）一律 forbidden——不让上位机超时卡死
        else if (s.substr(0, 5) == "!HAND")
            Respond(_responseChannel, "error forbidden on UART4");
        // 其它未知 ! 命令同样 forbidden
        else
            Respond(_responseChannel, "error forbidden on UART4");
    } else if (_cmd[0] == '#')
    {
        std::string s(_cmd);
        if (s == "#STACK")
        {
            RespondStackStats(_responseChannel);
        } else if (s.find("GETJPOS") != std::string::npos)
        {
            Respond(_responseChannel, "ok %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
                    dummy.currentJoints.a[0], dummy.currentJoints.a[1],
                    dummy.currentJoints.a[2], dummy.currentJoints.a[3],
                    dummy.currentJoints.a[4], dummy.currentJoints.a[5],
                    dummy.currentRailPos);  // [NEW] 第 7 位，Rail (mm)
        } else if (s.find("GETRAILPOS") != std::string::npos)
        {
            Respond(_responseChannel, "ok %.2f", dummy.currentRailPos);
        } else if (s.find("GETLPOS") != std::string::npos)
        {
            dummy.UpdateJointPose6D();
            Respond(_responseChannel, "ok %.2f %.2f %.2f %.2f %.2f %.2f",
                    dummy.currentPose6D.X, dummy.currentPose6D.Y,
                    dummy.currentPose6D.Z, dummy.currentPose6D.A,
                    dummy.currentPose6D.B, dummy.currentPose6D.C);
        } else if (s.find("CMDMODE") != std::string::npos)
        {
            uint32_t mode;
            sscanf(_cmd, "#CMDMODE %lu", &mode);
            dummy.SetCommandMode(mode);
            Respond(_responseChannel, "Set command mode to [%lu]", mode);
        } else
            Respond(_responseChannel, "ok");
    } else if (_cmd[0] == '>' || _cmd[0] == '@' || _cmd[0] == '&')
    {
        uint32_t freeSize = dummy.commandHandler.Push(_cmd);
        Respond(_responseChannel, "%d", freeSize);
    }
/*---------------------------- ↑ Add Your CMDs Here ↑ -----------------------------*/
}


void OnUart5AsciiCmd(const char* _cmd, size_t _len, StreamSink &_responseChannel)
{
    if (dummy.IsStopLocked()) return;
    /*---------------------------- ↓ Add Your CMDs Here ↓ -----------------------------*/

/*---------------------------- ↑ Add Your CMDs Here ↑ -----------------------------*/
}
