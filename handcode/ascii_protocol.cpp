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

        //新增
        else if (s.find("HAND_O") != std::string::npos)
        {
            dummy.hand2->SetAngleWithCurrentLimit(1);
            Respond(_responseChannel, "ok hand open");
        }
        else if (s.find("HAND_C") != std::string::npos)
        {
            dummy.hand2->SetAngleWithCurrentLimit(-1);
            Respond(_responseChannel, "ok hand close");
        }
        else if (s.find("HAND_EN") != std::string::npos)
        {
            dummy.hand2->SetEnable(true);
            Respond(_responseChannel, "ok hand enable");
            Respond(_responseChannel, "ok hand enable/disable is %lu", dummy.hand2->isEnabled());
        }
        else if (s.find("HAND_DIS") != std::string::npos)
        {
            dummy.hand2->SetEnable(false);
            Respond(_responseChannel, "ok hand disable");
            Respond(_responseChannel, "ok hand enable/disable is %lu", dummy.hand2->isEnabled());
        }
        else if (s.find("HAND_ZERO") != std::string::npos)
        {
            Respond(_responseChannel, "hand offset start");
            dummy.hand2->HandCalibration();
            Respond(_responseChannel, "ok hand offset MIN:%f max:%f",dummy.hand2->ClosedAngle, dummy.hand2->OpenedAngle);
        }
        else if (s.find("HAND_POS") != std::string::npos)
        {
            uint32_t pos;
            if (sscanf(_cmd, "!HAND_POS %lu", &pos) == 1)  // Check if parsing succeeded
            {
                if (pos <= 100 && pos > 0)  // Removed redundant pos>=0 check since uint32_t is always >=0
                {
                    dummy.hand2->SetAngleWithSpeedLimit(static_cast<float>(pos));
                    Respond(_responseChannel, "ok hand position %lu", pos);

                }
                else
                {
                    Respond(_responseChannel, "error hand position %lu - Value exceeds maximum (100)", pos);
                }
            }
            else
            {
                Respond(_responseChannel, "error hand position - Invalid format. Use #HAND_POS <0-100>");
            }
        }
        else if (s.find("HAND_I") != std::string::npos)
        {
            float cu;
            if (sscanf(_cmd, "!HAND_I %f", &cu) == 1)  // Check if parsing succeeded
            {

                if (cu <= 2 && cu>0)  // Removed redundant pos>=0 check since uint32_t is always >=0
                {
                    dummy.hand2->current=static_cast<float>(cu);

                    Respond(_responseChannel, "ok hand current %f", cu);
                }
                else
                {
                    Respond(_responseChannel, "error hand current %f - Value exceeds maximum (2.0)", cu);
                }
            }
            else
            {
                Respond(_responseChannel, "error hand current - Invalid format. Use #value <0-2.0> ");
            }
        }

    } else if (_cmd[0] == '#')
    {
        std::string s(_cmd);
        if (s.find("GETJPOS") != std::string::npos)
        {
            Respond(_responseChannel, "ok %.2f %.2f %.2f %.2f %.2f %.2f",
                    dummy.currentJoints.a[0], dummy.currentJoints.a[1],
                    dummy.currentJoints.a[2], dummy.currentJoints.a[3],
                    dummy.currentJoints.a[4], dummy.currentJoints.a[5]);
        } else if (s.find("GETLPOS") != std::string::npos)
        {
            dummy.UpdateJointPose6D();
            Respond(_responseChannel, "ok %.2f %.2f %.2f %.2f %.2f %.2f",
                    dummy.currentPose6D.X, dummy.currentPose6D.Y,
                    dummy.currentPose6D.Z, dummy.currentPose6D.A,
                    dummy.currentPose6D.B, dummy.currentPose6D.C);
        }

        else if (s.find("SET_DCE_KV") != std::string::npos)
        {
            uint32_t kp;
            uint32_t node;
            sscanf(_cmd, "#SET_DCE_KV %lu %lu", &node, &kp);
            if (node >= 1 & node <= 6){
                dummy.motorJ[node]->SetDceKv(kp);
                Respond(_responseChannel, "ok SET MOTOR [%lu] DCE_KV [%lu]", node, kp);
            }
            else {
                Respond(_responseChannel, "error SET MOTOR [%lu] DCE_KV [%lu] is wrong", node, kp);
            }
        }

        else if (s.find("SET_DCE_KP") != std::string::npos)
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
            if (node >= 1 & node <= 6){
                dummy.motorJ[node]->Reboot();
                Respond(_responseChannel, "ok REBOOT MOTOR [%lu]", node);
            }
            else {
                Respond(_responseChannel, "error REBOOT MOTOR [%lu] is wrong", node);
            }
        }else if (s.find("CMDMODE") != std::string::npos)
        {
            uint32_t mode;
            sscanf(_cmd, "#CMDMODE %lu", &mode);
            dummy.SetCommandMode(mode);
            Respond(_responseChannel, "ok Set command mode to [%lu]", mode);
        }
        //////////////////////////////
        else if (s.find("OFFSET_J") != std::string::npos)
        {
            uint32_t node;
            sscanf(_cmd, "#OFFSET_J %lu", &node);
            if (node >= 1 && node <= 7){
                dummy.motorJ[node]->ApplyPositionAsHome();
                Respond(_responseChannel, "ok HOMEOFFSET MOTOR [%lu]", node);
            }
            else {
                Respond(_responseChannel, "error HOMEOFFSET MOTOR [%lu] is wrong", node);
            }
        }

        else if (s.find("ACC_J") != std::string::npos)
        {
            float S;
            uint32_t node;
            sscanf(_cmd, "#ACC_J %lu %f", &node, &S);
            if (node >= 1 & node <= 7){
                dummy.motorJ[node]->SetAcceleration(S);
                Respond(_responseChannel, "ok SET MOTOR [%lu] ACCELERATION [%f]", node, S);
            }
            else {
                Respond(_responseChannel, "error SET MOTOR [%lu] ACCELERATION [%f] is wrong", node, S);
            }
        }
        else if (s.find("SPEED_J") != std::string::npos)
        {
            float S;
            uint32_t node;
            sscanf(_cmd, "#SPEED_J %lu %f", &node, &S);
            if (node >= 1 & node <= 7){
                dummy.motorJ[node]->SetVelocityLimit(S);
                Respond(_responseChannel, "ok SET MOTOR [%lu] SPEED [%f]", node, S);
            }
            else {
                Respond(_responseChannel, "error SET MOTOR [%lu] SPEED [%f] is wrong", node, S);
            }
        }
        else if (s.find("I_LIMIT_J") != std::string::npos)
        {
            float I;
            uint32_t node;
            sscanf(_cmd, "#I_LIMIT_J %lu %f", &node, &I);
            if (node >= 1 & node <= 7){
                dummy.motorJ[node]->SetCurrentLimit(I);
                Respond(_responseChannel, "ok SET MOTOR [%lu] CURRENT_LIMIT [%f]", node, I);
            }
            else {
                Respond(_responseChannel, "error SET MOTOR [%lu] CURRENT_LIMIT [%f] is wrong", node, I);
            }
        }
            ////////////////////////////
        else
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
    } else if (_cmd[0] == '#')
    {
        std::string s(_cmd);
        if (s.find("GETJPOS") != std::string::npos)
        {
            Respond(_responseChannel, "ok %.2f %.2f %.2f %.2f %.2f %.2f",
                    dummy.currentJoints.a[0], dummy.currentJoints.a[1],
                    dummy.currentJoints.a[2], dummy.currentJoints.a[3],
                    dummy.currentJoints.a[4], dummy.currentJoints.a[5]);
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
