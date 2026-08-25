#include "common_inc.h"
#include "configurations.h"
#include <can.h>


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

    switch (_cmd)
    {
        // 0x00~0x0F No Memory CMDs
        case 0x01:  // Enable Motor (重构: 支持 stallMode 状态机)
            if (*(uint32_t*) (RxData) == 1)
            {
                // enable: 清除 stallMode → IDLE，退出 LOCKED
                motor.controller->stallMode = Motor::STALL_IDLE;
                // 关键：调用 ResetGoalsToCurrentPosition() 把 goalPosition 重置为 estPosition，
                // 并清零 goalVelocity/goalCurrent/触发 softNewCurve，
                // 否则下个 20kHz 周期 CalcSoftGoal(goalPosition) 仍会用堵转前的目标位置,
                // 电机被再次驱动到同一目标 → 再次堵转 → 死循环。
                motor.controller->ResetGoalsToCurrentPosition();
                // 用当前位置作为 MotionPlanner 新起点 (estError = 0, 电机"已到位")
                motor.motionPlanner.positionTracker.NewTask(motor.controller->GetEstPosition(), motor.controller->GetEstVelocity());
                // 清堵转检测累积时间
                motor.controller->stallStartTick = 0;
                motor.controller->stallDetectRisingEdge = false;  // 重置上升沿标记
                motor.controller->positionModeStartCycles = 0;   // 重新进入启动豁免期
                // 恢复速度设置（回退期间可能改了 ratedVelocity）
                motor.config.motionParams.ratedVelocity = boardConfig.velocityLimit;
                motor.motionPlanner.positionTracker.SetVelocityAcc(boardConfig.velocityAcc);
                // 注意：不改 requestMode，保持原模式等待新命令
            }
            else
            {
                // disable: 类似处理，电机失能
                motor.controller->requestMode = Motor::MODE_STOP;
                motor.controller->stallMode = Motor::STALL_IDLE;
                // 同样重置 goalPosition，保持电机停在当前位置
                motor.controller->ResetGoalsToCurrentPosition();
                motor.motionPlanner.positionTracker.NewTask(motor.controller->GetEstPosition(), 0);
                motor.controller->stallStartTick = 0;
                motor.controller->stallDetectRisingEdge = false;
                motor.controller->positionModeStartCycles = 0;
                // 恢复速度设置（与 enable 路径一致: 回退期间可能改了 ratedVelocity）
                motor.config.motionParams.ratedVelocity = boardConfig.velocityLimit;
                motor.motionPlanner.positionTracker.SetVelocityAcc(boardConfig.velocityAcc);
            }
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
                           (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS));
            break;
        case 0x05:  // Set Position SetPoint
            // LOCKED/RETREATING 状态丢弃位置命令 (避免主控 stuck MoveJ 时反复下发覆盖 retreatTarget)
            if (motor.controller->stallMode == Motor::STALL_LOCKED ||
                motor.controller->stallMode == Motor::STALL_RETREATING)
                break;
            if (motor.controller->modeRunning != Motor::MODE_COMMAND_POSITION)
            {
                motor.config.motionParams.ratedVelocity = boardConfig.velocityLimit;
                motor.controller->SetCtrlMode(Motor::MODE_COMMAND_POSITION);
            }
            motor.controller->SetPositionSetPoint(
                (int32_t) (*(float*) RxData * (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS));
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
                (int32_t) (*(float*) RxData * (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS),
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
            // LOCKED/RETREATING 状态丢弃位置命令 (避免主控 stuck MoveJ 时反复下发覆盖 retreatTarget)
            if (motor.controller->stallMode == Motor::STALL_LOCKED ||
                motor.controller->stallMode == Motor::STALL_RETREATING)
                break;
            if (motor.controller->modeRunning != Motor::MODE_COMMAND_POSITION)
            {
                motor.config.motionParams.ratedVelocity = boardConfig.velocityLimit;
                motor.controller->SetCtrlMode(Motor::MODE_COMMAND_POSITION);
            }
            motor.config.motionParams.ratedVelocity =
                (int32_t) (*(float*) (RxData + 4) * (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);
            motor.controller->SetPositionSetPoint(
                (int32_t) (*(float*) RxData * (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS));
            // 不再立即应答 FINISH ACK：应答时 5kHz 控制循环尚未跑到，
            // controller->state 是上次的 stale 值，主控拿到后会立即误判到位、
            // SEQ 阻塞循环提前退出。状态由主控 100Hz 主动查 0x23 拿真实值。
            break;
        }

            // 0x10~0x1F CMDs with Memory
        case 0x11:  // Set Node-ID and Store to EEPROM
            boardConfig.canNodeId = *(uint32_t*) (RxData);
            if (_data[4])
                boardConfig.configStatus = CONFIG_COMMIT;
            break;
        case 0x12:  // Set Current-Limit and Store to EEPROM
            motor.config.motionParams.ratedCurrent = (int32_t) (*(float*) RxData * 1000);
            boardConfig.currentLimit = motor.config.motionParams.ratedCurrent;
            // 更新堵转阈值：按新电流的 75% 计算
            motor.controller->stallCurrentThreshold = motor.config.motionParams.ratedCurrent * 75 / 100;
            if (_data[4])
                boardConfig.configStatus = CONFIG_COMMIT;
            break;
        case 0x13:  // Set Velocity-Limit and Store to EEPROM
            motor.config.motionParams.ratedVelocity =
                (int32_t) (*(float*) RxData *
                           (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);
            boardConfig.velocityLimit = motor.config.motionParams.ratedVelocity;
            if (_data[4])
                boardConfig.configStatus = CONFIG_COMMIT;
            break;
        case 0x14:  // Set Acceleration （and Store to EEPROM）
            tmpF = *(float*) RxData * (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;

            motor.config.motionParams.ratedVelocityAcc = (int32_t) tmpF;
            motor.motionPlanner.velocityTracker.SetVelocityAcc((int32_t) tmpF);
            motor.motionPlanner.positionTracker.SetVelocityAcc((int32_t) tmpF);
            boardConfig.velocityAcc = motor.config.motionParams.ratedVelocityAcc;
            if (_data[4])
                boardConfig.configStatus = CONFIG_COMMIT;
            break;
        case 0x15:  // Apply Home-Position and Store to EEPROM
            motor.controller->ApplyPosAsHomeOffset();
            boardConfig.encoderHomeOffset = motor.config.motionParams.encoderHomeOffset %
                                            motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
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
        case 0x1B:  // Set Enable Stall-Protect (临时修改，不写 EEPROM)
            motor.config.ctrlParams.stallProtectSwitch = (*(uint32_t*) (RxData) == 1);
            // 不修改 boardConfig.enableStallProtect，保持默认 true
            // 不写入 EEPROM，重启后恢复默认开启
            break;


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

        case 0x2C: // Get Acceleration (Circle/s²)
        {
            tmpF = (float) boardConfig.velocityAcc / (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
            auto* b = (unsigned char*) &tmpF;
            for (int i = 0; i < 4; i++)
                _data[i] = *(b + i);
            _data[4] = 0;
            _data[5] = 0;
            _data[6] = 0;
            _data[7] = 0;
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
            _data[4] = 0;
            _data[5] = 0;
            _data[6] = 0;
            _data[7] = 0;
            txHeader.StdId = (boardConfig.canNodeId << 7) | 0x2D;
            CAN_Send(&txHeader, _data);
        }
            break;

        case 0x7d:  // enable motor temperature watch
            boardConfig.enableTempWatch = true;
            break;

        // ── 广播急停命令 (0x80~0xBF 区间): 所有电机节点无条件响应 ──
        case 0x89:  // Broadcast Emergency Stop
        {
            extern Motor motor;
            motor.controller->requestMode = Motor::MODE_STOP;
            motor.controller->SetBrake(true);  // P0-5: brake instead of coast
            motor.controller->SetVelocitySetPoint(0);
            motor.controller->SetCurrentSetPoint(0);
            // 急停不清 stallMode，保持 LOCKED 状态
            printf("[CAN BROADCAST] Emergency Stop Received!\r\n");
        }
            break;

        // ── 堵转检测重构新增命令 (0x50~0x7F 区间: 与广播阈值 0x50 对齐，避开位宽冲突) ──
        case 0x5A:  // 广播 STALL/STALL_DONE
            // 其他电机收到 STALL 广播 → 直接进 LOCKED（跳过 RETREATING）
            // 触发堵转的电机本身已在 RETREATING 状态，此处处理不影响
            {
                extern Motor motor;
                uint8_t stallNodeId = _data[0];  // 发起堵转的节点 ID
                uint8_t stallCmd = _data[1];     // 1=TRIGGER, 2=DONE, 3=TIMEOUT
                (void)stallNodeId;  // 本节点不检查（自己广播也可能收到回环）

                if (stallCmd == 1 && motor.controller->stallMode == Motor::STALL_IDLE)
                {
                    // 非堵转源电机：收到他人 STALL → 直接锁定
                    motor.controller->stallMode = Motor::STALL_LOCKED;
                    motor.controller->ClearIntegral();
                    printf("[STALL] node=%d locked by remote stall\r\n", boardConfig.canNodeId);
                }
                // TRIGGER 状态下 STALL_DONE/TIMEOUT 不做特殊处理（由电机端主循环的 RETREATING 处理器负责）
            }
            break;

        case 0x5B:  // 广播 UNLOCKED: 全局解除所有电机 LOCKED
        {
            extern Motor motor;
            if (motor.controller->stallMode == Motor::STALL_LOCKED)
            {
                motor.controller->stallMode = Motor::STALL_IDLE;
                // 关键: 调用 ResetGoalsToCurrentPosition() 把 goalPosition 重置为 estPosition，
                // 否则 CalcSoftGoal(goalPosition) 仍会驱动电机到堵转前的目标位置 → 再次堵转
                motor.controller->ResetGoalsToCurrentPosition();
                motor.motionPlanner.positionTracker.NewTask(motor.controller->GetEstPosition(), motor.controller->GetEstVelocity());
                motor.controller->stallStartTick = 0;
                motor.controller->stallDetectRisingEdge = false;
                motor.controller->positionModeStartCycles = 0;
                motor.config.motionParams.ratedVelocity = boardConfig.velocityLimit;
                motor.motionPlanner.positionTracker.SetVelocityAcc(boardConfig.velocityAcc);
                printf("[CAN BROADCAST] UNLOCKED Received, node=%d\r\n", boardConfig.canNodeId);
            }
            // 非 LOCKED 状态忽略
        }
            break;

        case 0x5C:  // 单播查询 stall 状态 (主控->单电机)
        {
            // Data[0] = 1: 查询 stallProtectSwitch(stall en)
            // Data[0] = 2: 查询 stallMode == LOCKED
            extern Motor motor;
            uint8_t queryType = _data[0];
            uint8_t respValue = 0;
            if (queryType == 1)
                respValue = motor.controller->config->stallProtectSwitch ? 1 : 0;
            else if (queryType == 2)
                respValue = (motor.controller->stallMode == Motor::STALL_LOCKED) ? 1 : 0;

            txHeader.StdId = (boardConfig.canNodeId << 7) | 0x5C;
            txHeader.IDE = CAN_ID_STD;
            txHeader.RTR = CAN_RTR_DATA;
            txHeader.DLC = 8;
            uint8_t txData[8] = { queryType, respValue, 0, 0, 0, 0, 0, 0 };
            CAN_Send(&txHeader, txData);
        }
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

