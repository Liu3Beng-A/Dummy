#include "common_inc.h"
#include "instances/dummy_robot.h"

extern DummyRobot dummy;
extern RGB rgb;

/* ======================================================================
 * OnUsbAsciiCmd —— USB 通道 ASCII 协议指令处理
 *
 * 指令前缀说明：
 *   '!'  → 系统控制类（使能、急停、回零、夹爪控制等）
 *   '#'  → 查询/配置类（获取位姿、设置电机参数等）
 *   '>'  → 关节空间运动 MoveJ（直接入队）
 *   '@'  → 笛卡尔直线运动 MoveL（直接入队）
 *   '&'  → 关节空间运动（另一前缀，与 '>' 等效）
 *   '$'  → 力矩透传（6轴电流，直接入队）
 * ====================================================================== */
void OnUsbAsciiCmd(const char* _cmd, size_t _len, StreamSink &_responseChannel)
{
    /*---------------------------- ↓ Add Your CMDs Here ↓ -----------------------------*/
    if (_cmd[0] == '!')
    {
        std::string s(_cmd);

        /* ── 系统控制指令 ── */
        if (s.find("STOP") != std::string::npos)
        {
            dummy.commandHandler.EmergencyStop();
            Respond(_responseChannel, "Stopped ok");
        }
        else if (s.find("START") != std::string::npos)
        {
            dummy.SetEnable(true);
            Respond(_responseChannel, "Started ok");
        }
        else if (s.find("HOME") != std::string::npos)
        {
            dummy.Homing();
            Respond(_responseChannel, "Started ok");
        }
        else if (s.find("RESET") != std::string::npos)
        {
            dummy.Resting();
            Respond(_responseChannel, "Started ok");
        }
        else if (s.find("DISABLE") != std::string::npos)
        {
            dummy.SetEnable(false);
            Respond(_responseChannel, "Disabled ok");
        }

        /* ── 夹爪控制指令（hand，节点ID=7）──
         *
         * 夹爪控制说明：
         *   !HAND_O          → 电流模式张开（-current 施加开夹力矩，注意方向已反转）
         *   !HAND_C          → 电流模式闭合（+current 施加合夹力矩，注意方向已反转）
         *   !HAND_EN         → 使能夹爪电机
         *   !HAND_DIS        → 失能夹爪电机
         *   !HAND_POS <0-100>→ 位置模式：0=完全张开，100=完全闭合
         *   !HAND_I  <0-2.0> → 设置电流幅值（A），影响 HAND_O/HAND_C 力度
         */
        else if (s.find("HAND_O") != std::string::npos)
        {
            /* 向张开方向施加电流（-1 × current，因为0是张开，100是闭合） */
            dummy.hand->SetAngleWithCurrentLimit(-1);
            Respond(_responseChannel, "ok hand open");
        }
        else if (s.find("HAND_C") != std::string::npos)
        {
            /* 向闭合方向施加电流（+1 × current） */
            dummy.hand->SetAngleWithCurrentLimit(1);
            Respond(_responseChannel, "ok hand close");
        }
        
        /* ── RGB 信仰灯控制指令 ──
         * !RGB_EN         → 开灯
         * !RGB_DIS        → 关灯
         * !RGB_MODE <0-9> → 切换当前灯效
         * !RGB_COLOR <idx> <r> <g> <b> → 设置静态纯色(0/1/2)的颜色，0-255
         * !RGB_SET_START <0-9>  → 设置开机默认灯效
         * !RGB_SET_ENABLE <0-9> → 设置使能时灯效
         * !RGB_SET_DISABLE <0-9>→ 设置失能时灯效
         */
        else if (s.find("RGB_EN") != std::string::npos)
        {
            dummy.SetRGBEnabled(true);
            Respond(_responseChannel, "ok rgb enable");
        }
        else if (s.find("RGB_DIS") != std::string::npos)
        {
            dummy.SetRGBEnabled(false);
            Respond(_responseChannel, "ok rgb disable");
        }
        else if (s.find("RGB_MODE") != std::string::npos)
        {
            uint32_t mode;
            if (sscanf(_cmd, "!RGB_MODE %lu", &mode) == 1 && mode <= 9)
            {
                dummy.SetRGBMode(mode);
                Respond(_responseChannel, "ok rgb mode %lu", mode);
            }
            else
            {
                Respond(_responseChannel, "error rgb mode format. Use !RGB_MODE <0-9>");
            }
        }
        else if (s.find("RGB_COLOR") != std::string::npos)
        {
            uint32_t idx, r, g, b;
            if (sscanf(_cmd, "!RGB_COLOR %lu %lu %lu %lu", &idx, &r, &g, &b) == 4)
            {
                if (idx <= 2 && r <= 255 && g <= 255 && b <= 255)
                {
                    rgb.static_r[idx] = r;
                    rgb.static_g[idx] = g;
                    rgb.static_b[idx] = b;
                    dummy.SaveConfig();
                    Respond(_responseChannel, "ok rgb color %lu %lu %lu %lu", idx, r, g, b);
                }
                else
                {
                    Respond(_responseChannel, "error rgb color args. idx(0-2) rgb(0-255)");
                }
            }
            else
            {
                Respond(_responseChannel, "error rgb color format. Use !RGB_COLOR <idx> <r> <g> <b>");
            }
        }
        else if (s.find("RGB_SET_ST") != std::string::npos)
        {
            uint32_t mode;
            if (sscanf(_cmd, "!RGB_SET_ST %lu", &mode) == 1 && mode <= 9)
            {
                dummy.rgbStateStart = mode;
                dummy.SaveConfig();
                Respond(_responseChannel, "ok rgb start mode set to %lu", mode);
            }
            else
            {
                Respond(_responseChannel, "error rgb start format. Use !RGB_SET_START <0-9>");
            }
        }
        else if (s.find("RGB_SET_EN") != std::string::npos)
        {
            uint32_t mode;
            if (sscanf(_cmd, "!RGB_SET_EN %lu", &mode) == 1 && mode <= 9)
            {
                dummy.rgbStateEnable = mode;
                dummy.SaveConfig();
                Respond(_responseChannel, "ok rgb enable mode set to %lu", mode);
            }
            else
            {
                Respond(_responseChannel, "error rgb enable format. Use !RGB_SET_ENABLE <0-9>");
            }
        }
        else if (s.find("RGB_SET_DI") != std::string::npos)
        {
            uint32_t mode;
            if (sscanf(_cmd, "!RGB_SET_DI %lu", &mode) == 1 && mode <= 9)
            {
                dummy.rgbStateDisable = mode;
                dummy.SaveConfig();
                Respond(_responseChannel, "ok rgb disable mode set to %lu", mode);
            }
            else
            {
                Respond(_responseChannel, "error rgb disable format. Use !RGB_SET_DISABLE <0-9>");
            }
        }
        
        else if (s.find("HAND_EN") != std::string::npos)
        {
            dummy.hand->SetEnable(true);
            Respond(_responseChannel, "ok hand enable");
            Respond(_responseChannel, "ok hand enable/disable is %lu", dummy.hand->isEnabled());
        }
        else if (s.find("HAND_DIS") != std::string::npos)
        {
            dummy.hand->SetEnable(false);
            Respond(_responseChannel, "ok hand disable");
            Respond(_responseChannel, "ok hand enable/disable is %lu", dummy.hand->isEnabled());
        }
        else if (s.find("HAND_POS") != std::string::npos)
        {
            /* 格式：!HAND_POS <0-100>，0=完全张开，100=完全闭合 */
            uint32_t pos;
            if (sscanf(_cmd, "!HAND_POS %lu", &pos) == 1)
            {
                /* pos 为 uint32_t，隐含 >= 0，只需检查上界 */
                if (pos <= 100)
                {
                    dummy.hand->SetAngleWithSpeedLimit(static_cast<float>(pos));
                    Respond(_responseChannel, "ok hand position %lu", pos);
                }
                else
                {
                    Respond(_responseChannel,
                            "error hand position %lu - Value exceeds maximum (100)", pos);
                }
            }
            else
            {
                Respond(_responseChannel,
                        "error hand position - Invalid format. Use !HAND_POS <0-100>");
            }
        }
        else if (s.find("HAND_I") != std::string::npos)
        {
            /* 格式：!HAND_I <0-2.0>，设置电流控制时的幅值（A） */
            float cu;
            if (sscanf(_cmd, "!HAND_I %f", &cu) == 1)
            {
                if (cu > 0 && cu <= 2.0f)
                {
                    dummy.hand->current = cu;
                    Respond(_responseChannel, "ok hand current %f", cu);
                }
                else
                {
                    Respond(_responseChannel,
                            "error hand current %f - Value must be in range (0, 2.0]", cu);
                }
            }
            else
            {
                Respond(_responseChannel,
                        "error hand current - Invalid format. Use !HAND_I <0-2.0>");
            }
        }
        else if (s.find("PRINTPOSE") != std::string::npos)
        {
            /* 打印当前关节角和末端位姿（调试用） */
            Respond(_responseChannel, "GETJPOS: %.2f %.2f %.2f %.2f %.2f %.2f",
                    dummy.currentJoints.a[0], dummy.currentJoints.a[1],
                    dummy.currentJoints.a[2], dummy.currentJoints.a[3],
                    dummy.currentJoints.a[4], dummy.currentJoints.a[5]);

            /* 直接读取控制环维护的 currentPose6D 缓存，无需重复触发 FK 计算 */
            Respond(_responseChannel, "GETLPOS: %.2f %.2f %.2f %.2f %.2f %.2f",
                    dummy.currentPose6D.X, dummy.currentPose6D.Y,
                    dummy.currentPose6D.Z, dummy.currentPose6D.A,
                    dummy.currentPose6D.B, dummy.currentPose6D.C);
        }

    }
    else if (_cmd[0] == '#')
    {
        std::string s(_cmd);

        if (s.find("GETJPOS") != std::string::npos)
        {
            Respond(_responseChannel, "ok %.2f %.2f %.2f %.2f %.2f %.2f",
                    dummy.currentJoints.a[0], dummy.currentJoints.a[1],
                    dummy.currentJoints.a[2], dummy.currentJoints.a[3],
                    dummy.currentJoints.a[4], dummy.currentJoints.a[5]);
        }
        else if (s.find("GETLPOS") != std::string::npos)
        {
            /* 直接读取缓存，避免在通信线程中重复做 FK 计算 */
            Respond(_responseChannel, "ok %.2f %.2f %.2f %.2f %.2f %.2f",
                    dummy.currentPose6D.X, dummy.currentPose6D.Y,
                    dummy.currentPose6D.Z, dummy.currentPose6D.A,
                    dummy.currentPose6D.B, dummy.currentPose6D.C);
        }
        else if (s.find("SET_DCE_KV") != std::string::npos)
        {
            uint32_t kv, node;
            sscanf(_cmd, "#SET_DCE_KV %lu %lu", &node, &kv);
            /* 修复：使用 && 替代 & （原代码误用位与运算符） */
            if (node >= 1 && node <= 6)
            {
                dummy.motorJ[node]->SetDceKv(kv);
                Respond(_responseChannel, "ok SET MOTOR [%lu] DCE_KV [%lu]", node, kv);
            }
            else
            {
                Respond(_responseChannel, "error SET MOTOR [%lu] DCE_KV [%lu] is wrong", node, kv);
            }
        }
        else if (s.find("SET_DCE_KP") != std::string::npos)
        {
            uint32_t kp, node;
            sscanf(_cmd, "#SET_DCE_KP %lu %lu", &node, &kp);
            if (node >= 1 && node <= 6)
            {
                dummy.motorJ[node]->SetDceKp(kp);
                Respond(_responseChannel, "ok SET MOTOR [%lu] DCE_KP [%lu]", node, kp);
            }
            else
            {
                Respond(_responseChannel, "error SET MOTOR [%lu] DCE_KP [%lu] is wrong", node, kp);
            }
        }
        else if (s.find("SET_DCE_KI") != std::string::npos)
        {
            uint32_t ki, node;
            sscanf(_cmd, "#SET_DCE_KI %lu %lu", &node, &ki);
            if (node >= 1 && node <= 6)
            {
                dummy.motorJ[node]->SetDceKi(ki);
                Respond(_responseChannel, "ok SET MOTOR [%lu] DCE_KI [%lu]", node, ki);
            }
            else
            {
                Respond(_responseChannel, "error SET MOTOR [%lu] DCE_KI [%lu] is wrong", node, ki);
            }
        }
        else if (s.find("SET_DCE_KD") != std::string::npos)
        {
            uint32_t kd, node;
            sscanf(_cmd, "#SET_DCE_KD %lu %lu", &node, &kd);
            if (node >= 1 && node <= 6)
            {
                dummy.motorJ[node]->SetDceKd(kd);
                Respond(_responseChannel, "ok SET MOTOR [%lu] DCE_KD [%lu]", node, kd);
            }
            else
            {
                Respond(_responseChannel, "error SET MOTOR [%lu] DCE_KD [%lu] is wrong", node, kd);
            }
        }
        else if (s.find("REBOOT") != std::string::npos)
        {
            uint32_t node;
            sscanf(_cmd, "#REBOOT %lu", &node);
            if (node >= 1 && node <= 6)
            {
                dummy.motorJ[node]->Reboot();
                Respond(_responseChannel, "ok REBOOT MOTOR [%lu]", node);
            }
            else
            {
                Respond(_responseChannel, "error REBOOT MOTOR [%lu] is wrong", node);
            }
        }
        else if (s.find("CMDMODE") != std::string::npos)
        {
            uint32_t mode;
            sscanf(_cmd, "#CMDMODE %lu", &mode);
            dummy.SetCommandMode(mode);
            Respond(_responseChannel, "ok Set command mode to [%lu]", mode);
        }
        else if (s.find("OFFSET_J") != std::string::npos)
        {
            uint32_t node;
            sscanf(_cmd, "#OFFSET_J %lu", &node);
            if (node >= 1 && node <= 6)
            {
                dummy.motorJ[node]->ApplyPositionAsHome();
                Respond(_responseChannel, "ok HOMEOFFSET MOTOR [%lu]", node);
            }
            else
            {
                Respond(_responseChannel, "error HOMEOFFSET MOTOR [%lu] is wrong", node);
            }
        }
        
        else if (s.find("ACC_J") != std::string::npos)
        {
            float S;
            uint32_t node;
            sscanf(_cmd, "#ACC_J %lu %f", &node, &S);
            if (node >= 1 && node <= 6)
            {
                dummy.motorJ[node]->SetAcceleration(S);
                Respond(_responseChannel, "ok SET MOTOR [%lu] ACCELERATION [%f]", node, S);
            }
            else
            {
                Respond(_responseChannel,
                        "error SET MOTOR [%lu] ACCELERATION [%f] is wrong", node, S);
            }
        }
        else if (s.find("SPEED_J") != std::string::npos)
        {
            float S;
            uint32_t node;
            sscanf(_cmd, "#SPEED_J %lu %f", &node, &S);
            if (node >= 1 && node <= 6)
            {
                dummy.motorJ[node]->SetVelocityLimit(S);
                Respond(_responseChannel, "ok SET MOTOR [%lu] SPEED [%f]", node, S);
            }
            else
            {
                Respond(_responseChannel, "error SET MOTOR [%lu] SPEED [%f] is wrong", node, S);
            }
        }
        else if (s.find("I_LIMIT_J") != std::string::npos)
        {
            float I;
            uint32_t node;
            sscanf(_cmd, "#I_LIMIT_J %lu %f", &node, &I);
            if (node >= 1 && node <= 6)
            {
                dummy.motorJ[node]->SetCurrentLimit(I);
                Respond(_responseChannel, "ok SET MOTOR [%lu] CURRENT_LIMIT [%f]", node, I);
            }
            else
            {
                Respond(_responseChannel,
                        "error SET MOTOR [%lu] CURRENT_LIMIT [%f] is wrong", node, I);
            }
        }
        else
        {
            /* 未识别的 '#' 指令转发到命令队列 */
            dummy.commandHandler.Push(_cmd);
        }
    }
    if (_cmd[0] == '>' || _cmd[0] == '@' || _cmd[0] == '&' || _cmd[0] == '$')
    {
        /* ────────────────────────────────────────────────────────────
         * 修复：USB通道也需要保护：未使能时拒绝运动指令并提示。
         * ──────────────────────────────────────────────────────────── */
        if (!dummy.IsEnabled())
        {
            Respond(_responseChannel, "error: robot not enabled, send !START first");
            return;
        }
        /* 运动/力矩指令直接入队，返回队列剩余空间供上位机流控 */
        uint32_t freeSize = dummy.commandHandler.Push(_cmd);
        Respond(_responseChannel, "%d", freeSize);
    }
    /*---------------------------- ↑ Add Your CMDs Here ↑ -----------------------------*/
}


/* ======================================================================
 * OnUart4AsciiCmd —— UART4 通道 ASCII 协议指令处理
 *
 * 注意：原版代码中 `if (_cmd[0] == '!' || !dummy.IsEnabled())` 有逻辑问题：
 * 未使能时所有指令（'#' '>' '@'）都会落入 '!' 分支而无法正确处理。
 * 修复：未使能时仅阻止运动指令（'>' '@' '&'），查询指令（'#'）仍然可用。
 * ====================================================================== */
void OnUart4AsciiCmd(const char* _cmd, size_t _len, StreamSink &_responseChannel)
{
    /*---------------------------- ↓ Add Your CMDs Here ↓ -----------------------------*/
    if (_cmd[0] == '!')
    {
        std::string s(_cmd);

        if (s.find("STOP") != std::string::npos)
        {
            dummy.commandHandler.EmergencyStop();
            Respond(_responseChannel, "Stopped ok");
        }
        else if (s.find("START") != std::string::npos)
        {
            dummy.SetEnable(true);
            /* 每次收到使能指令，强制切回默认位置控制模式，防止残留力矩模式 */
            dummy.SetCommandMode(dummy.DEFAULT_COMMAND_MODE);
            Respond(_responseChannel, "Started ok");
        }
        else if (s.find("HOME") != std::string::npos)
        {
            dummy.Homing();
            Respond(_responseChannel, "Started ok");
        }
        else if (s.find("RESET") != std::string::npos)
        {
            dummy.Resting();
            Respond(_responseChannel, "Started ok");
        }
        else if (s.find("DISABLE") != std::string::npos)
        {
            dummy.SetEnable(false);
            Respond(_responseChannel, "Disabled ok");
        }
        else if (s.find("PRINTPOSE") != std::string::npos)
        {
            Respond(_responseChannel, "GETJPOS: %.2f %.2f %.2f %.2f %.2f %.2f",
                    dummy.currentJoints.a[0], dummy.currentJoints.a[1],
                    dummy.currentJoints.a[2], dummy.currentJoints.a[3],
                    dummy.currentJoints.a[4], dummy.currentJoints.a[5]);

            Respond(_responseChannel, "GETLPOS: %.2f %.2f %.2f %.2f %.2f %.2f",
                    dummy.currentPose6D.X, dummy.currentPose6D.Y,
                    dummy.currentPose6D.Z, dummy.currentPose6D.A,
                    dummy.currentPose6D.B, dummy.currentPose6D.C);
        }
    }
    else if (_cmd[0] == '#')
    {
        std::string s(_cmd);

        if (s.find("GETJPOS") != std::string::npos)
        {
            Respond(_responseChannel, "ok %.2f %.2f %.2f %.2f %.2f %.2f",
                    dummy.currentJoints.a[0], dummy.currentJoints.a[1],
                    dummy.currentJoints.a[2], dummy.currentJoints.a[3],
                    dummy.currentJoints.a[4], dummy.currentJoints.a[5]);
        }
        else if (s.find("GETLPOS") != std::string::npos)
        {
            Respond(_responseChannel, "ok %.2f %.2f %.2f %.2f %.2f %.2f",
                    dummy.currentPose6D.X, dummy.currentPose6D.Y,
                    dummy.currentPose6D.Z, dummy.currentPose6D.A,
                    dummy.currentPose6D.B, dummy.currentPose6D.C);
        }
        else if (s.find("CMDMODE") != std::string::npos)
        {
            uint32_t mode;
            sscanf(_cmd, "#CMDMODE %lu", &mode);
            dummy.SetCommandMode(mode);
            Respond(_responseChannel, "Set command mode to [%lu]", mode);
        }
        
        else
        {
            Respond(_responseChannel, "ok");
        }
    }
    else if (_cmd[0] == '>' || _cmd[0] == '@' || _cmd[0] == '&' || _cmd[0] == '$')
    {
        /* ────────────────────────────────────────────────────────────
         * 修复：原版 `!dummy.IsEnabled()` 判断导致未使能时运动指令
         * 进入 '!' 分支被忽略。此处改为：未使能时拒绝运动指令并提示。
         * ──────────────────────────────────────────────────────────── */
        if (!dummy.IsEnabled())
        {
            Respond(_responseChannel, "error: robot not enabled, send !START first");
            return;
        }
        uint32_t freeSize = dummy.commandHandler.Push(_cmd);
        Respond(_responseChannel, "%d", freeSize);
    }
    /*---------------------------- ↑ Add Your CMDs Here ↑ -----------------------------*/
}


void OnUart5AsciiCmd(const char* _cmd, size_t _len, StreamSink &_responseChannel)
{
    /*---------------------------- ↓ Add Your CMDs Here ↓ -----------------------------*/

    /*---------------------------- ↑ Add Your CMDs Here ↑ -----------------------------*/
}
