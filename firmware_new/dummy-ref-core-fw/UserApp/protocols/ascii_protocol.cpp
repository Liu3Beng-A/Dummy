#include "common_inc.h"

extern DummyRobot dummy;

void OnUsbAsciiCmd(const char* _cmd, size_t _len, StreamSink &_responseChannel)
{
    uint8_t  i;
    /*---------------------------- ↓ Add Your CMDs Here ↓ -----------------------------*/
    if (_cmd[0] == '!' )
    {
        std::string s(_cmd);
        if (s.find("STOP") != std::string::npos)
        {
            dummy.commandHandler.EmergencyStop();
            Respond(_responseChannel, "Stopped ok");
        } else if (s.find("START") != std::string::npos)
        {
            dummy.SetEnable(true);
            Respond(_responseChannel, "Started ok");
        } else if (s.find("HOME") != std::string::npos)
        {
            dummy.Homing();
            Respond(_responseChannel, "Started ok");
        } else if (s.find("CALIBRATION") != std::string::npos)
        {
            dummy.CalibrateHomeOffset();
            Respond(_responseChannel, "calibration ok");
        } else if (s.find("RESET") != std::string::npos)
        {
            dummy.Resting();
            Respond(_responseChannel, "Started ok");
        } else if (s.find("DISABLE") != std::string::npos)
        {
            dummy.SetEnable(false);
            Respond(_responseChannel, "Disabled ok");
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
            dummy.hand->SetAngle(static_cast<float>(pos));
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
        if (s.find("GETJPOS") != std::string::npos)
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
        } else if (s.find("SET_DCE_KP") != std::string::npos)
        {
            uint32_t kp;
            uint32_t node;
            sscanf(_cmd, "#SET_DCE_KP %lu %lu", &node, &kp);
            if (node >= 1 & node <= 6){
                dummy.motorJ[node]->SetDceKp(kp);
                Respond(_responseChannel, "ok SET MOTOR [%lu] DCE_KP [%lu]", node, kp);
            }
            else {
                Respond(_responseChannel, "error SET MOTOR [%lu] DCE_KP [%lu] is wrong", node, kp);
            }
        } else if (s.find("SET_DCE_KI") != std::string::npos)
        {
            uint32_t kp;
            uint32_t node;
            sscanf(_cmd, "#SET_DCE_KI %lu %lu", &node, &kp);
            if (node >= 1 & node <= 6){
                dummy.motorJ[node]->SetDceKi(kp);
                Respond(_responseChannel, "ok SET MOTOR [%lu] DCE_KI [%lu]", node, kp);
            }
            else {
                Respond(_responseChannel, "error SET MOTOR [%lu] DCE_KI [%lu] is wrong", node, kp);
            }
        } else if (s.find("SET_DCE_KD") != std::string::npos)
        {
            uint32_t kp;
            uint32_t node;
            sscanf(_cmd, "#SET_DCE_KD %lu %lu", &node, &kp);
            if (node >= 1 & node <= 6){
                dummy.motorJ[node]->SetDceKd(kp);
                Respond(_responseChannel, "ok SET MOTOR [%lu] DCE_KD [%lu]", node, kp);
            }
            else {
                Respond(_responseChannel, "error SET MOTOR [%lu] DCE_KD [%lu] is wrong", node, kp);
            }
        } else if (s.find("REBOOT") != std::string::npos)
        {
            uint32_t node;
            sscanf(_cmd, "#REBOOT %lu", &node);
            if (node >= 1 && node <= 7)                  // [FIX] && 写法 + 范围到 7
            {
                dummy.motorJ[node]->Reboot();
                Respond(_responseChannel, "ok REBOOT MOTOR [%lu]", node);
            }
            else if (node == 0)                          // [NEW] ALL 代理
            {
                Respond(_responseChannel, "ok REBOOT ALL MOTORS + HAND");  // [FIX] 先回包
                dummy.RebootAllMotors();                                   // [FIX] 再发重启帧
            }
            else
            {
                Respond(_responseChannel, "error REBOOT MOTOR [%lu] is wrong", node);
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
    /*---------------------------- ↓ Add Your CMDs Here ↓ -----------------------------*/
    if (_cmd[0] == '!' || !dummy.IsEnabled())
    {
        std::string s(_cmd);
        if (s.find("STOP") != std::string::npos)
        {
            dummy.commandHandler.EmergencyStop();
            Respond(_responseChannel, "Stopped ok");
        } else if (s.find("START") != std::string::npos)
        {
            dummy.SetEnable(true);
            Respond(_responseChannel, "Started ok");
        } else if (s.find("HOME") != std::string::npos)
        {
            dummy.Homing();
            Respond(_responseChannel, "Started ok");
        } else if (s.find("CALIBRATION") != std::string::npos)
        {
            dummy.CalibrateHomeOffset();
            Respond(_responseChannel, "calibration ok");
        } else if (s.find("RESET") != std::string::npos)
        {
            dummy.Resting();
            Respond(_responseChannel, "Started ok");
        } else if (s.find("DISABLE") != std::string::npos)
        {
            dummy.SetEnable(false);
            Respond(_responseChannel, "Disabled ok");
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
            dummy.hand->SetAngle(static_cast<float>(pos));
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
        if (s.find("GETJPOS") != std::string::npos)
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
    /*---------------------------- ↓ Add Your CMDs Here ↓ -----------------------------*/

/*---------------------------- ↑ Add Your CMDs Here ↑ -----------------------------*/
}
