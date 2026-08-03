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
        if (s == "!STOP")
        {
            dummy.commandHandler.EmergencyStop();
            Respond(_responseChannel, "Stopped ok");
        }
        else if (s.find("!RGB_SET_START") == 0)
        {
            uint32_t mode;
            if (sscanf(_cmd, "!RGB_SET_START %lu", &mode) == 1 && mode <= 9)
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
        else if (s.find("!RGB_SET_ENABLE") == 0)
        {
            uint32_t mode;
            if (sscanf(_cmd, "!RGB_SET_ENABLE %lu", &mode) == 1 && mode <= 9)
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
        else if (s.find("!RGB_SET_DISABLE") == 0)
        {
            uint32_t mode;
            if (sscanf(_cmd, "!RGB_SET_DISABLE %lu", &mode) == 1 && mode <= 9)
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
        else if (s == "!START")
        {
            dummy.SetEnable(true);
            Respond(_responseChannel, "Started ok");
        }
        else if (s == "!HOME")
        {
            dummy.Homing();
            Respond(_responseChannel, "Started ok");
        }
        else if (s == "!RESET")
        {
            dummy.Resting();
            Respond(_responseChannel, "Started ok");
        }
        else if (s == "!DISABLE")
        {
            dummy.SetEnable(false);
            Respond(_responseChannel, "Disabled ok");
        }
        else if (s.find("!RAIL_L") == 0)
        {
            float delta;
            if (sscanf(_cmd, "!RAIL_L %f", &delta) == 1)
            {
                dummy.MoveRailRelative(-fabsf(delta));
                Respond(_responseChannel, "ok rail left %.1f mm, target %.1f mm", fabsf(delta), dummy.targetRailPos);
            }
            else
            {
                Respond(_responseChannel, "error rail left - Use !RAIL_L <delta(mm)>");
            }
        }
        else if (s.find("RAIL_R") != std::string::npos)
        {
            float delta;
            if (sscanf(_cmd, "!RAIL_R %f", &delta) == 1)
            {
                dummy.MoveRailRelative(fabsf(delta));
                Respond(_responseChannel, "ok rail right %.1f mm, target %.1f mm", fabsf(delta), dummy.targetRailPos);
            }
            else
            {
                Respond(_responseChannel, "error rail right - Use !RAIL_R <delta(mm)>");
            }
        }

        /* ── 夹爪控制指令（hand，节点ID=8）──
         *
         * 夹爪控制说明：
         *   !CALIBRATION      → 关节零点标定（6轴同时应用零点）
         *   !HAND_ZERO        → 夹爪标定（标定夹爪当前位置为零点）
         *   !HAND_O           → 电流模式张开（-current 施加开夹力矩，注意方向已反转）
         *   !HAND_C           → 电流模式闭合（+current 施加合夹力矩，注意方向已反转）
         *   !HAND_EN          → 使能夹爪电机
         *   !HAND_DIS         → 失能夹爪电机
         *   !HAND_POS <0-100> → 位置模式：0=完全张开，100=完全闭合
         */
        else if (s.find("HAND_ZERO") != std::string::npos)
        {
            /* 夹爪标定：将当前位置设为夹爪零点 */
            dummy.hand->ApplyPositionAsHome();
            Respond(_responseChannel, "ok hand zero calibrated");
        }
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

        /* ── 堵转检测控制指令 ──
         * !STALL_EN  → 开启所有电机堵转检测（发 CAN 0x1B 到 J0~J6）
         * !STALL_DIS → 关闭所有电机堵转检测
         */
        else if (s.find("STALL_EN") != std::string::npos)
        {
            for (int i = 0; i < 7; i++)
                dummy.motorJ[i]->SetEnableStallProtect(true);
            Respond(_responseChannel, "ok stall protect enabled");
        }
        else if (s.find("STALL_DIS") != std::string::npos)
        {
            for (int i = 0; i < 7; i++)
                dummy.motorJ[i]->SetEnableStallProtect(false);
            Respond(_responseChannel, "ok stall protect disabled");
        }

        /* ── RGB 信仰灯控制指令 ──
         * !RGB_BRIGHT [<0-100>] [&] → 查询亮度 / 设置亮度 / 设置并保存亮度
         * !RGB_MODE <0-9>      → 切换灯效模式
         * !RGB_COLOR <idx> <r> <g> <b> → 设置静态纯色的颜色，0-255
         * !RGB_SET_START <0-9>  → 设置开机默认灯效
         * !RGB_SET_ENABLE <0-9> → 设置使能时灯效
         * !RGB_SET_DISABLE <0-9>→ 设置失能时灯效
         */
        else if (s.find("RGB_BRIGHT") != std::string::npos)
        {
            uint32_t val;
            char saveFlag;
            if (sscanf(_cmd, "!RGB_BRIGHT %lu %c", &val, &saveFlag) == 2 && val <= 100)
            {
                rgb.targetBrightness = (float)val / 100.0f;
                if (saveFlag == '&')
                    dummy.SaveConfig();
                Respond(_responseChannel, "ok rgb bright %lu", val);
            }
            else if (sscanf(_cmd, "!RGB_BRIGHT %lu", &val) == 1 && val <= 100)
            {
                rgb.targetBrightness = (float)val / 100.0f;
                Respond(_responseChannel, "ok rgb bright %lu", val);
            }
            else
            {
                Respond(_responseChannel, "%.0f", rgb.targetBrightness * 100.0f);
            }
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
        else if (s.find("!RGB_SET_START") == 0)
        {
            uint32_t mode;
            if (sscanf(_cmd, "!RGB_SET_START %lu", &mode) == 1 && mode <= 9)
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
        else if (s.find("!RGB_SET_ENABLE") == 0)
        {
            uint32_t mode;
            if (sscanf(_cmd, "!RGB_SET_ENABLE %lu", &mode) == 1 && mode <= 9)
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
        else if (s.find("!RGB_SET_DISABLE") == 0)
        {
            uint32_t mode;
            if (sscanf(_cmd, "!RGB_SET_DISABLE %lu", &mode) == 1 && mode <= 9)
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
            /* 2026-06-25: 放开 node=0 限制，允许通过 ASCII 命令调整地轨电机 DCE 参数 */
            if (node >= 0 && node <= 6)
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
            if (node >= 0 && node <= 6)
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
            if (node >= 0 && node <= 6)
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
            if (node >= 0 && node <= 6)
            {
                dummy.motorJ[node]->SetDceKd(kd);
                Respond(_responseChannel, "ok SET MOTOR [%lu] DCE_KD [%lu]", node, kd);
            }
            else
            {
                Respond(_responseChannel, "error SET MOTOR [%lu] DCE_KD [%lu] is wrong", node, kd);
            }
        }
        else if (s.find("GET_PID") != std::string::npos)
        {
            uint32_t node;
            sscanf(_cmd, "#GET_PID %lu", &node);
            if (node == 9)
            {
                dummy.motorJ[0]->QueryDceKp();
                dummy.motorJ[0]->QueryDceKv();
                dummy.motorJ[0]->QueryDceKi();
                dummy.motorJ[0]->QueryDceKd();
                Respond(_responseChannel, "ok QUERY PID RAIL [9]");
            }
            else if (node >= 1 && node <= 6)
            {
                dummy.motorJ[node]->QueryDceKp();
                dummy.motorJ[node]->QueryDceKv();
                dummy.motorJ[node]->QueryDceKi();
                dummy.motorJ[node]->QueryDceKd();
                Respond(_responseChannel, "ok QUERY PID MOTOR [%lu]", node);
            }
            else
            {
                Respond(_responseChannel, "error GET_PID [%lu] wrong (use 9 for rail, 1~6 for joints)", node);
            }
        }
        else if (s.find("REBOOT") != std::string::npos)
        {
            uint32_t node;
            sscanf(_cmd, "#REBOOT %lu", &node);
            if (node == 9)
            {
                dummy.motorJ[0]->Reboot();
                Respond(_responseChannel, "ok REBOOT RAIL [9]");
            }
            else if (node >= 1 && node <= 6)
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
        else if (s.find("ACC_BASE_J") != std::string::npos)
        {
            float S;
            uint32_t node;
            sscanf(_cmd, "#ACC_BASE_J %lu %f", &node, &S);
            if (node >= 1 && node <= 6)
            {
                if (S >= 1.0f && S <= 2000.0f)
                {
                    dummy.jointAccBases.a[node - 1] = S;
                    dummy.SaveConfig();
                    
                    // 立即按当前模式重新应用加速度
                    uint32_t currentMode = static_cast<uint32_t>(dummy.commandMode);
                    dummy.SetCommandMode(currentMode); 

                    Respond(_responseChannel, "ok SET MOTOR [%lu] BASE ACCELERATION [%f] AND SAVED", node, S);
                }
                else
                {
                    Respond(_responseChannel, "error ACC_BASE_J value must be in [1.0, 2000.0]");
                }
            }
            else
            {
                Respond(_responseChannel,
                        "error SET MOTOR [%lu] BASE ACCELERATION [%f] is wrong", node, S);
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
        else if (s.find("SPEED_RAIL") != std::string::npos)
        {
            float speed;
            char saveFlag;
            if (sscanf(_cmd, "#SPEED_RAIL %f %c", &speed, &saveFlag) >= 1)
            {
                dummy.SetRailSpeed(speed);
                Respond(_responseChannel, "ok rail speed set to %.1f mm/s", speed);
                if (saveFlag == '&')
                {
                    dummy.SaveConfig();
                    Respond(_responseChannel, " [saved to EEPROM]");
                }
            }
            else
            {
                Respond(_responseChannel, "%.1f", dummy.railSpeed_mm_s);
            }
        }
        else if (s.find("ACC_RAIL") != std::string::npos)
        {
            float acc;
            char saveFlag;
            if (sscanf(_cmd, "#ACC_RAIL %f %c", &acc, &saveFlag) >= 1)
            {
                dummy.SetRailAcc(acc);
                Respond(_responseChannel, "ok rail acc set to %.1f mm/s2", acc);
                if (saveFlag == '&')
                {
                    dummy.SaveConfig();
                    Respond(_responseChannel, " [saved to EEPROM]");
                }
            }
            else
            {
                Respond(_responseChannel, "%.1f", dummy.railAcc_mm_s2);
            }
        }
        else if (s.find("I_LIMIT_J") != std::string::npos)
        {
            float I;
            uint32_t node;
            sscanf(_cmd, "#I_LIMIT_J %lu %f", &node, &I);
            if (node == 9)
            {
                // 节点 9: 地轨电机（motorJ[0] 对应 CAN ID=9）
                dummy.motorJ[0]->SetCurrentLimit(I);
                Respond(_responseChannel, "ok SET MOTOR [9] CURRENT_LIMIT [%f] (地轨)", I);
            }
            else if (node >= 1 && node <= 6)
            {
                dummy.motorJ[node]->SetCurrentLimit(I);
                Respond(_responseChannel, "ok SET MOTOR [%lu] CURRENT_LIMIT [%f]", node, I);
            }
            else if (node == 8)
            {
                dummy.hand->SetCurrentLimit(I);
                Respond(_responseChannel, "ok SET MOTOR [8] CURRENT_LIMIT [%f] (夹爪)", I);
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

        if (s == "!STOP")
        {
            dummy.commandHandler.EmergencyStop();
            Respond(_responseChannel, "Stopped ok");
        }
        else if (s.find("!RGB_SET_START") == 0)
        {
            uint32_t mode;
            if (sscanf(_cmd, "!RGB_SET_START %lu", &mode) == 1 && mode <= 9)
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
        else if (s.find("!RGB_SET_ENABLE") == 0)
        {
            uint32_t mode;
            if (sscanf(_cmd, "!RGB_SET_ENABLE %lu", &mode) == 1 && mode <= 9)
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
        else if (s.find("!RGB_SET_DISABLE") == 0)
        {
            uint32_t mode;
            if (sscanf(_cmd, "!RGB_SET_DISABLE %lu", &mode) == 1 && mode <= 9)
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
        else if (s == "!START")
        {
            dummy.SetEnable(true);
            /* 每次收到使能指令，强制切回默认位置控制模式，防止残留力矩模式 */
            dummy.SetCommandMode(dummy.DEFAULT_COMMAND_MODE);
            Respond(_responseChannel, "Started ok");
        }
        else if (s == "!HOME")
        {
            dummy.Homing();
            Respond(_responseChannel, "Started ok");
        }
        else if (s == "!RESET")
        {
            dummy.Resting();
            Respond(_responseChannel, "Started ok");
        }
        else if (s == "!DISABLE")
        {
            dummy.SetEnable(false);
            Respond(_responseChannel, "Disabled ok");
        }
        else if (s.find("!CALIBRATION") == 0)
        {
            /* 关节零点标定：对所有6个关节应用当前电机位置作为零点偏移 */
            for (int i = 1; i <= 6; i++)
            {
                dummy.motorJ[i]->ApplyPositionAsHome();
            }
            Respond(_responseChannel, "ok calibration done for all joints");
        }
        else if (s.find("RAIL_L") != std::string::npos)
        {
            /* !RAIL_L <delta>  地轨向左（负方向）移动 delta mm */
            float delta;
            if (sscanf(_cmd, "!RAIL_L %f", &delta) == 1)
            {
                dummy.MoveRailRelative(-fabsf(delta));
                Respond(_responseChannel, "ok rail left %.1f mm, target %.1f mm", fabsf(delta), dummy.targetRailPos);
            }
            else
            {
                Respond(_responseChannel, "error rail left - Use !RAIL_L <delta(mm)>");
            }
        }
        else if (s.find("RAIL_R") != std::string::npos)
        {
            /* !RAIL_R <delta>  地轨向右（正方向）移动 delta mm */
            float delta;
            if (sscanf(_cmd, "!RAIL_R %f", &delta) == 1)
            {
                dummy.MoveRailRelative(fabsf(delta));
                Respond(_responseChannel, "ok rail right %.1f mm, target %.1f mm", fabsf(delta), dummy.targetRailPos);
            }
            else
            {
                Respond(_responseChannel, "error rail right - Use !RAIL_R <delta(mm)>");
            }
        }
        else if (s.find("HAND_ZERO") != std::string::npos)
        {
            /* 夹爪标定：将当前位置设为夹爪零点 */
            dummy.hand->ApplyPositionAsHome();
            Respond(_responseChannel, "ok hand zero calibrated");
        }
        else if (s.find("HAND_EN") != std::string::npos)
        {
            dummy.hand->SetEnable(true);
            Respond(_responseChannel, "ok hand enable");
        }
        else if (s.find("HAND_DIS") != std::string::npos)
        {
            dummy.hand->SetEnable(false);
            Respond(_responseChannel, "ok hand disable");
        }
        else if (s.find("HAND_POS") != std::string::npos)
        {
            uint32_t pos;
            if (sscanf(_cmd, "!HAND_POS %lu", &pos) == 1)
            {
                if (pos <= 100)
                {
                    dummy.hand->SetAngleWithSpeedLimit(static_cast<float>(pos));
                    Respond(_responseChannel, "ok hand position %lu", pos);
                }
                else
                {
                    Respond(_responseChannel, "error hand position %lu - Value exceeds maximum (100)", pos);
                }
            }
            else
            {
                Respond(_responseChannel, "error hand position - Invalid format. Use !HAND_POS <0-100>");
            }
        }
        else if (s.find("HAND_O") != std::string::npos)
        {
            dummy.hand->SetAngleWithCurrentLimit(-1);
            Respond(_responseChannel, "ok hand open");
        }
        else if (s.find("HAND_C") != std::string::npos)
        {
            dummy.hand->SetAngleWithCurrentLimit(1);
            Respond(_responseChannel, "ok hand close");
        }
        else if (s.find("STALL_EN") != std::string::npos)
        {
            for (int i = 0; i < 7; i++)
                dummy.motorJ[i]->SetEnableStallProtect(true);
            Respond(_responseChannel, "ok stall protect enabled");
        }
        else if (s.find("STALL_DIS") != std::string::npos)
        {
            for (int i = 0; i < 7; i++)
                dummy.motorJ[i]->SetEnableStallProtect(false);
            Respond(_responseChannel, "ok stall protect disabled");
        }
        else if (s.find("RGB_BRIGHT") != std::string::npos)
        {
            uint32_t val;
            char saveFlag;
            if (sscanf(_cmd, "!RGB_BRIGHT %lu %c", &val, &saveFlag) == 2 && val <= 100)
            {
                rgb.targetBrightness = (float)val / 100.0f;
                if (saveFlag == '&')
                    dummy.SaveConfig();
                Respond(_responseChannel, "ok rgb bright %lu", val);
            }
            else if (sscanf(_cmd, "!RGB_BRIGHT %lu", &val) == 1 && val <= 100)
            {
                rgb.targetBrightness = (float)val / 100.0f;
                Respond(_responseChannel, "ok rgb bright %lu", val);
            }
            else
            {
                Respond(_responseChannel, "%.0f", rgb.targetBrightness * 100.0f);
            }
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
        else if (s.find("!RGB_SET_START") == 0)
        {
            uint32_t mode;
            if (sscanf(_cmd, "!RGB_SET_START %lu", &mode) == 1 && mode <= 9)
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
        else if (s.find("!RGB_SET_ENABLE") == 0)
        {
            uint32_t mode;
            if (sscanf(_cmd, "!RGB_SET_ENABLE %lu", &mode) == 1 && mode <= 9)
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
        else if (s.find("!RGB_SET_DISABLE") == 0)
        {
            uint32_t mode;
            if (sscanf(_cmd, "!RGB_SET_DISABLE %lu", &mode) == 1 && mode <= 9)
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
        else if (s.find("ACC_BASE_J") != std::string::npos)
        {
            float S;
            uint32_t node;
            sscanf(_cmd, "#ACC_BASE_J %lu %f", &node, &S);
            if (node >= 1 && node <= 6)
            {
                if (S >= 1.0f && S <= 2000.0f)
                {
                    dummy.jointAccBases.a[node - 1] = S;
                    dummy.SaveConfig();

                    uint32_t currentMode = static_cast<uint32_t>(dummy.commandMode);
                    dummy.SetCommandMode(currentMode);

                    Respond(_responseChannel, "ok SET MOTOR [%lu] BASE ACCELERATION [%f] AND SAVED", node, S);
                }
                else
                {
                    Respond(_responseChannel, "error ACC_BASE_J value must be in [1.0, 2000.0]");
                }
            }
            else
            {
                Respond(_responseChannel,
                        "error SET MOTOR [%lu] BASE ACCELERATION [%f] is wrong", node, S);
            }
        }
        else if (s.find("SPEED_RAIL") != std::string::npos)
        {
            float speed;
            char saveFlag;
            if (sscanf(_cmd, "#SPEED_RAIL %f %c", &speed, &saveFlag) >= 1)
            {
                dummy.SetRailSpeed(speed);
                Respond(_responseChannel, "ok rail speed set to %.1f mm/s", speed);
                if (saveFlag == '&')
                {
                    dummy.SaveConfig();
                    Respond(_responseChannel, " [saved to EEPROM]");
                }
            }
            else
            {
                Respond(_responseChannel, "%.1f", dummy.railSpeed_mm_s);
            }
        }
        else if (s.find("ACC_RAIL") != std::string::npos)
        {
            float acc;
            char saveFlag;
            if (sscanf(_cmd, "#ACC_RAIL %f %c", &acc, &saveFlag) >= 1)
            {
                dummy.SetRailAcc(acc);
                Respond(_responseChannel, "ok rail acc set to %.1f mm/s2", acc);
                if (saveFlag == '&')
                {
                    dummy.SaveConfig();
                    Respond(_responseChannel, " [saved to EEPROM]");
                }
            }
            else
            {
                Respond(_responseChannel, "%.1f", dummy.railAcc_mm_s2);
            }
        }
        else if (s.find("SET_DCE_KV") != std::string::npos)
        {
            uint32_t kv, node;
            sscanf(_cmd, "#SET_DCE_KV %lu %lu", &node, &kv);
            /* 2026-06-25: 放开 node=0 限制，允许通过 ASCII 命令调整地轨电机 DCE 参数 */
            if (node >= 0 && node <= 6)
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
            if (node >= 0 && node <= 6)
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
            if (node >= 0 && node <= 6)
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
            if (node >= 0 && node <= 6)
            {
                dummy.motorJ[node]->SetDceKd(kd);
                Respond(_responseChannel, "ok SET MOTOR [%lu] DCE_KD [%lu]", node, kd);
            }
            else
            {
                Respond(_responseChannel, "error SET MOTOR [%lu] DCE_KD [%lu] is wrong", node, kd);
            }
        }
        else if (s.find("GET_PID") != std::string::npos)
        {
            uint32_t node;
            sscanf(_cmd, "#GET_PID %lu", &node);
            if (node == 9)
            {
                dummy.motorJ[0]->QueryDceKp();
                dummy.motorJ[0]->QueryDceKv();
                dummy.motorJ[0]->QueryDceKi();
                dummy.motorJ[0]->QueryDceKd();
                Respond(_responseChannel, "ok QUERY PID RAIL [9]");
            }
            else if (node >= 1 && node <= 6)
            {
                dummy.motorJ[node]->QueryDceKp();
                dummy.motorJ[node]->QueryDceKv();
                dummy.motorJ[node]->QueryDceKi();
                dummy.motorJ[node]->QueryDceKd();
                Respond(_responseChannel, "ok QUERY PID MOTOR [%lu]", node);
            }
            else
            {
                Respond(_responseChannel, "error GET_PID [%lu] wrong (use 9 for rail, 1~6 for joints)", node);
            }
        }
        else if (s.find("REBOOT") != std::string::npos)
        {
            uint32_t node;
            sscanf(_cmd, "#REBOOT %lu", &node);
            if (node == 9)
            {
                dummy.motorJ[0]->Reboot();
                Respond(_responseChannel, "ok REBOOT RAIL [9]");
            }
            else if (node >= 1 && node <= 6)
            {
                dummy.motorJ[node]->Reboot();
                Respond(_responseChannel, "ok REBOOT MOTOR [%lu]", node);
            }
            else
            {
                Respond(_responseChannel, "error REBOOT MOTOR [%lu] is wrong", node);
            }
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
                Respond(_responseChannel, "error SET MOTOR [%lu] ACCELERATION [%f] is wrong", node, S);
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
            if (node == 9)
            {
                // 节点 9: 地轨电机（motorJ[0] 对应 CAN ID=9）
                dummy.motorJ[0]->SetCurrentLimit(I);
                Respond(_responseChannel, "ok SET MOTOR [9] CURRENT_LIMIT [%f] (地轨)", I);
            }
            else if (node >= 1 && node <= 6)
            {
                dummy.motorJ[node]->SetCurrentLimit(I);
                Respond(_responseChannel, "ok SET MOTOR [%lu] CURRENT_LIMIT [%f]", node, I);
            }
            else if (node == 8)
            {
                dummy.hand->SetCurrentLimit(I);
                Respond(_responseChannel, "ok SET MOTOR [8] CURRENT_LIMIT [%f] (夹爪)", I);
            }
            else
            {
                Respond(_responseChannel, "error SET MOTOR [%lu] CURRENT_LIMIT [%f] is wrong", node, I);
            }
        }
        else
        {
            Respond(_responseChannel, "ok");
        }
    }
    else if (_cmd[0] == '>' || _cmd[0] == '@' || _cmd[0] == '&' || _cmd[0] == '$')
    {
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
