#include "common_inc.h"
#include "configurations.h"
#include <can.h>
#include <cstring>


extern Motor motor;
extern EncoderCalibrator encoderCalibrator;

CAN_TxHeaderTypeDef txHeader =
    {
        .StdId = 0x00,
        .ExtId = 0x00,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = 8,
        .TransmitGlobalTime = DISABLE
    };


void OnCanCmd(uint8_t _cmd, uint8_t* _data, uint32_t _len)
{
    float tmpF;
    int32_t tmpI;

    if (motor.controller->ProcessProtectionCan(_cmd, _data, _len)) return;
    if (_len < 4) return;
    motor.controller->OnMasterCommunication();

    if (motor.controller->IsSoftStopLocked() &&
        _cmd != SoftStopProtocol::COMMAND && _cmd != 0x23 &&
        !(_cmd == 0x01 && _len >= 4 && *(uint32_t*)_data == 0))
        return;

    switch (_cmd)
    {
        case SoftStopProtocol::COMMAND:
        {
            if (_len < 4) break;
#if MOTOR_TYPE == 8
            // Gripper only participates in START from disabled, never STOP/RELEASE.
            if (_data[0] != SoftStopProtocol::ENABLE_HOLD &&
                _data[0] != SoftStopProtocol::DISABLE &&
                _data[0] != SoftStopProtocol::QUERY &&
                !(_data[0] == SoftStopProtocol::STOP && motor.controller->IsCalibrationContext())) break;
#endif
            const uint16_t token = static_cast<uint16_t>(_data[2] | (_data[3] << 8));
            motor.controller->RequestSoftStop(_data[0], token);
            uint8_t reply[8] = {};
            const float position = motor.controller->GetSoftStopPosition();
            memcpy(reply, &position, sizeof(position));
            reply[4] = motor.controller->GetSoftStopPhase();
            const uint16_t replyToken = motor.controller->GetSoftStopToken();
            reply[6] = static_cast<uint8_t>(replyToken);
            reply[7] = static_cast<uint8_t>(replyToken >> 8);
            CAN_TxHeaderTypeDef header = txHeader;
            header.StdId = (boardConfig.canNodeId << 7) | SoftStopProtocol::COMMAND;
            CAN_Send(&header, reply);
            break;
        }
        // 0x00~0x0F No Memory CMDs
        case 0x09:  // Broadcast EStop - Emergency stop all motors
            motor.controller->requestMode = Motor::MODE_STOP;
            motor.controller->SetBrake(true);
            motor.controller->SetCurrentSetPoint(0);
            motor.controller->SetVelocitySetPoint(0);
            // 不发 ACK，纯制动
            break;
        case 0x01:  // Enable Motor
            if (_len < 4) break;
            if (*(uint32_t*)_data == 0)
            {
                motor.controller->DisableForStop();
                break;
            }
            motor.controller->SetDisable(false);
            motor.controller->requestMode = (*(uint32_t*) (RxData) == 1) ?
                                            Motor::MODE_COMMAND_VELOCITY : Motor::MODE_STOP;
            break;
        case 0x02:  // Do Calibration
            encoderCalibrator.isTriggered = true;
            break;
        case 0x03:  // Set Current SetPoint
            if (motor.controller->modeRunning != Motor::MODE_COMMAND_CURRENT)
                motor.controller->SetCtrlMode(Motor::MODE_COMMAND_CURRENT);
            motor.controller->SetCurrentSetPoint((int32_t) (*(float*) RxData * 1000));
            break;
        case 0x04:  // Set Velocity SetPoint
            if (motor.controller->modeRunning != Motor::MODE_COMMAND_VELOCITY)
            {
                motor.config.motionParams.ratedVelocity = boardConfig.velocityLimit;
                motor.controller->SetCtrlMode(Motor::MODE_COMMAND_VELOCITY);
            }
            motor.controller->SetVelocitySetPoint(
                (int32_t) (*(float*) RxData *
                           (float) Motor::MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS));
            break;
        case 0x05:  // Set Position SetPoint
            if (motor.controller->modeRunning != Motor::MODE_COMMAND_POSITION)
            {
                motor.config.motionParams.ratedVelocity = boardConfig.velocityLimit;
                motor.controller->SetCtrlMode(Motor::MODE_COMMAND_POSITION);
            }
            motor.controller->SetPositionSetPoint(
                (int32_t) (*(float*) RxData * (float) Motor::MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS));
            printf("SET MOTOR[0x05] POSITION[]\r\n");
            if (_data[4]) // Need Position & Finished ACK
            {
                tmpF = motor.controller->GetPosition();
                auto* b = (unsigned char*) &tmpF;
                for (int i = 0; i < 4; i++)
                    _data[i] = *(b + i);
                _data[4] = motor.controller->state == Motor::STATE_FINISH ? 1 : 0;
                txHeader.StdId = (boardConfig.canNodeId << 7) | 0x23;
                CAN_Send(&txHeader, _data);
            }
            break;
        case 0x06:  // Set Position with Time
            if (motor.controller->modeRunning != Motor::MODE_COMMAND_POSITION)
                motor.controller->SetCtrlMode(Motor::MODE_COMMAND_POSITION);
            motor.controller->SetPositionSetPointWithTime(
                (int32_t) (*(float*) RxData * (float) Motor::MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS),
                *(float*) (RxData + 4));
            if (_data[4]) // Need Position & Finished ACK
            {
                tmpF = motor.controller->GetPosition();
                auto* b = (unsigned char*) &tmpF;
                for (int i = 0; i < 4; i++)
                    _data[i] = *(b + i);
                _data[4] = motor.controller->state == Motor::STATE_FINISH ? 1 : 0;
                txHeader.StdId = (boardConfig.canNodeId << 7) | 0x23;
                CAN_Send(&txHeader, _data);
            }
            break;
        case 0x07:  // Set Position with Velocity-Limit
        {
            if (motor.controller->modeRunning != Motor::MODE_COMMAND_POSITION)
            {
                motor.config.motionParams.ratedVelocity = boardConfig.velocityLimit;
                motor.controller->SetCtrlMode(Motor::MODE_COMMAND_POSITION);
            }
            motor.config.motionParams.ratedVelocity =
                (int32_t) (*(float*) (RxData + 4) * (float) Motor::MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);
            motor.controller->SetPositionSetPoint(
                (int32_t) (*(float*) RxData * (float) Motor::MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS));
            // Always Need Position & Finished ACK
            tmpF = motor.controller->GetPosition();
            auto* b = (unsigned char*) &tmpF;
            for (int i = 0; i < 4; i++)
                _data[i] = *(b + i);
            _data[4] = motor.controller->state == Motor::STATE_FINISH ? 1 : 0;
            txHeader.StdId = (boardConfig.canNodeId << 7) | 0x23;
            CAN_Send(&txHeader, _data);
        }
            break;

            // 0x10~0x1F CMDs with Memory
#if MOTOR_TYPE == 8
        case 0x11:  // Set Node-ID and Store to EEPROM (no-op for gripper)
            // gripper 硬编�?ID=8，启动时已被 main.cpp 强制覆盖
            break;
#else
        case 0x11:  // Set Node-ID and Store to EEPROM
            boardConfig.canNodeId = *(uint32_t*) (RxData);
            if (_data[4])
                boardConfig.configStatus = CONFIG_COMMIT;
            break;
#endif
        case 0x12:  // Set Current-Limit and Store to EEPROM
            if (!ProtectionSettings::Finite(*(float*)RxData) || *(float*)RxData < .1f || *(float*)RxData > 2.5f) break;
            motor.config.motionParams.ratedCurrent = (int32_t) (*(float*) RxData * 1000);
            boardConfig.currentLimit = motor.config.motionParams.ratedCurrent;
            motor.controller->OnMotionLimitChange();
            if (_data[4])
                boardConfig.configStatus = CONFIG_COMMIT;
            break;
        case 0x13:  // Set Velocity-Limit and Store to EEPROM
            if (!ProtectionSettings::Finite(*(float*)RxData) || *(float*)RxData < .001f || *(float*)RxData > 30) break;
            motor.config.motionParams.ratedVelocity =
                (int32_t) (*(float*) RxData *
                           (float) Motor::MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);
            boardConfig.velocityLimit = motor.config.motionParams.ratedVelocity;
            motor.controller->OnMotionLimitChange();
            if (_data[4])
                boardConfig.configStatus = CONFIG_COMMIT;
            break;
        case 0x14:  // Set Acceleration （and Store to EEPROM�?
            if (!ProtectionSettings::Finite(*(float*)RxData) || *(float*)RxData < .1f || *(float*)RxData > 100) break;
            tmpF = *(float*) RxData * (float) Motor::MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;

            motor.config.motionParams.ratedVelocityAcc = (int32_t) tmpF;
            motor.motionPlanner.velocityTracker.SetVelocityAcc((int32_t) tmpF);
            motor.motionPlanner.positionTracker.SetVelocityAcc((int32_t) tmpF);
            boardConfig.velocityAcc = motor.config.motionParams.ratedVelocityAcc;
            motor.controller->OnMotionLimitChange();
            if (_data[4])
                boardConfig.configStatus = CONFIG_COMMIT;
            break;
        case 0x15:  // Apply Home-Position and Store to EEPROM
            motor.controller->ApplyPosAsHomeOffset();
            boardConfig.encoderHomeOffset = motor.config.motionParams.encoderHomeOffset %
                                            Motor::MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
            boardConfig.configStatus = CONFIG_COMMIT;
            break;
        case 0x16:  // Set Auto-Enable and Store to EEPROM
            boardConfig.enableMotorOnBoot = (*(uint32_t*) (RxData) == 1);
            if (_data[4])
                boardConfig.configStatus = CONFIG_COMMIT;
            break;
        case 0x17:  // Set DCE Kp
            motor.config.ctrlParams.dce.kp = *(int32_t*) (RxData);
            boardConfig.dce_kp = motor.config.ctrlParams.dce.kp;
            if (_data[4])
                boardConfig.configStatus = CONFIG_COMMIT;
            break;
        case 0x18:  // Set DCE Kv
            motor.config.ctrlParams.dce.kv = *(int32_t*) (RxData);
            boardConfig.dce_kv = motor.config.ctrlParams.dce.kv;
            if (_data[4])
                boardConfig.configStatus = CONFIG_COMMIT;
            break;
        case 0x19:  // Set DCE Ki
            motor.config.ctrlParams.dce.ki = *(int32_t*) (RxData);
            boardConfig.dce_ki = motor.config.ctrlParams.dce.ki;
            if (_data[4])
                boardConfig.configStatus = CONFIG_COMMIT;
            break;
        case 0x1A:  // Set DCE Kd
            motor.config.ctrlParams.dce.kd = *(int32_t*) (RxData);
            boardConfig.dce_kd = motor.config.ctrlParams.dce.kd;
            if (_data[4])
                boardConfig.configStatus = CONFIG_COMMIT;
            break;
        #if MOTOR_TYPE == 8
        case 0x1B:  // Set Enable Stall-Protect (no-op for gripper)
            // gripper 没有 stall 保护；保�?EEPROM 字段不变
            break;
#else
        case 0x1B:  // Set Enable Stall-Protect
            motor.config.ctrlParams.stallProtectSwitch = (*(uint32_t*) (RxData) == 1);
            motor.controller->protectionSettings.value[ProtectionProtocol::ENABLE] = motor.config.ctrlParams.stallProtectSwitch ? 1 : 0;
            boardConfig.enableStallProtect = motor.config.ctrlParams.stallProtectSwitch;
            if (_data[4])
                boardConfig.configStatus = CONFIG_COMMIT;
            break;
#endif


            // 0x20~0x2F Inquiry CMDs
        case 0x21: // Get Current
        {
            tmpF = motor.controller->GetFocCurrent();
            auto* b = (unsigned char*) &tmpF;
            for (int i = 0; i < 4; i++)
                _data[i] = *(b + i);
            _data[4] = (motor.controller->state == Motor::STATE_FINISH ? 1 : 0);

            txHeader.StdId = (boardConfig.canNodeId << 7) | 0x21;
            CAN_Send(&txHeader, _data);
        }
            break;
        case 0x22: // Get Velocity
        {
            tmpF = motor.controller->GetVelocity();
            auto* b = (unsigned char*) &tmpF;
            for (int i = 0; i < 4; i++)
                _data[i] = *(b + i);
            _data[4] = (motor.controller->state == Motor::STATE_FINISH ? 1 : 0);

            txHeader.StdId = (boardConfig.canNodeId << 7) | 0x22;
            CAN_Send(&txHeader, _data);
        }
            break;
        case 0x23: // Get Position
        {
            if (_len == 8 && _data[4] == MotionCompletionProtocol::QUERY_MARKER)
            {
                float expected;
                memcpy(&expected, _data, sizeof(expected));
                const uint32_t irq = __get_PRIMASK(); __disable_irq();
                tmpF = motor.controller->GetPosition();
                const uint8_t flags = motor.controller->GetPositionCompletionStatus(expected);
                __set_PRIMASK(irq);
                memcpy(_data, &tmpF, sizeof(tmpF));
                _data[4] = flags; // Preserve the nonce; legacy ACKs have no marker.
                CAN_TxHeaderTypeDef header = txHeader;
                header.StdId = (boardConfig.canNodeId << 7) | MotionCompletionProtocol::COMMAND;
                CAN_Send(&header, _data);
                break;
            }
            tmpF = motor.controller->GetPosition();
            auto* b = (unsigned char*) &tmpF;
            for (int i = 0; i < 4; i++)
                _data[i] = *(b + i);
            // Finished ACK
            _data[4] = motor.controller->state == Motor::STATE_FINISH ? 1 : 0;
            txHeader.StdId = (boardConfig.canNodeId << 7) | 0x23;
            CAN_Send(&txHeader, _data);
//            printf("CAN SEND BACK to NODE[%d]\n", boardConfig.canNodeId );
        }
            break;
        case 0x24: // Get Offset
        {
            tmpI = motor.config.motionParams.encoderHomeOffset;
            auto* b = (unsigned char*) &tmpI;
            for (int i = 0; i < 4; i++)
                _data[i] = *(b + i);
            txHeader.StdId = (boardConfig.canNodeId << 7) | 0x24;
            CAN_Send(&txHeader, _data);
        }
            break;

        case 0x25: // Get temperature
        {
            auto* b = (unsigned char*) &boardConfig.motor_temperature;
            for (int i = 0; i < 4; i++)
                _data[i] = *(b + i);
            _data[4] = 0;
            _data[5] = 0;
            _data[6] = 0;
            _data[7] = 0;
            txHeader.StdId = (boardConfig.canNodeId << 7) | 0x25;
            CAN_Send(&txHeader, _data);
        }
            break;

        case 0x28: // Get DCE Kp
        {
            tmpI = boardConfig.dce_kp;
            auto* b = (unsigned char*) &tmpI;
            for (int i = 0; i < 4; i++)
                _data[i] = *(b + i);
            txHeader.StdId = (boardConfig.canNodeId << 7) | 0x28;
            CAN_Send(&txHeader, _data);
        }
            break;
        case 0x29: // Get DCE Kv
        {
            tmpI = boardConfig.dce_kv;
            auto* b = (unsigned char*) &tmpI;
            for (int i = 0; i < 4; i++)
                _data[i] = *(b + i);
            txHeader.StdId = (boardConfig.canNodeId << 7) | 0x29;
            CAN_Send(&txHeader, _data);
        }
            break;
        case 0x2A: // Get DCE Ki
        {
            tmpI = boardConfig.dce_ki;
            auto* b = (unsigned char*) &tmpI;
            for (int i = 0; i < 4; i++)
                _data[i] = *(b + i);
            txHeader.StdId = (boardConfig.canNodeId << 7) | 0x2A;
            CAN_Send(&txHeader, _data);
        }
            break;
        case 0x2B: // Get DCE Kd
        {
            tmpI = boardConfig.dce_kd;
            auto* b = (unsigned char*) &tmpI;
            for (int i = 0; i < 4; i++)
                _data[i] = *(b + i);
            txHeader.StdId = (boardConfig.canNodeId << 7) | 0x2B;
            CAN_Send(&txHeader, _data);
        }
            break;
        case 0x2C: // Get Acceleration (r/s²)
        {
            tmpF = (float) boardConfig.velocityAcc / (float) Motor::MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
            auto* b = (unsigned char*) &tmpF;
            for (int i = 0; i < 4; i++)
                _data[i] = *(b + i);
            txHeader.StdId = (boardConfig.canNodeId << 7) | 0x2C;
            CAN_Send(&txHeader, _data);
        }
            break;
        case 0x2D: // Get Current-Limit (A)
        {
            tmpF = (float) boardConfig.currentLimit / 1000.0f;
            auto* b = (unsigned char*) &tmpF;
            for (int i = 0; i < 4; i++)
                _data[i] = *(b + i);
            txHeader.StdId = (boardConfig.canNodeId << 7) | 0x2D;
            CAN_Send(&txHeader, _data);
        }
            break;

        case 0x7d:  // enable motor temperature watch
            boardConfig.enableTempWatch = true;
            break;
        case 0x7e:  // Erase Configs
            boardConfig.configStatus = CONFIG_RESTORE;
            break;
        case 0x7f:  // Reboot
            HAL_NVIC_SystemReset();
            break;
        default:
            break;
    }

}

extern "C" void OnProtectionPeerFrame(uint8_t node, uint8_t command, uint8_t* data, uint32_t length)
{
    if (command != ProtectionProtocol::EVENT || length != 8) return;
    uint32_t event; memcpy(&event, data, 4);
    motor.controller->OnPeerProtection(node, event, data[4], data[7]);
}

