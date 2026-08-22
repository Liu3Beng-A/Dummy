#include "configurations.h"
#include "motor.h"
#include <can.h>

#include <cmath>


void Motor::Tick20kHz()
{
    // 1.Encoder data Update
    encoder->UpdateAngle();

    // 2.Motor Control Update
    CloseLoopControlTick();
}


void Motor::AttachEncoder(EncoderBase* _encoder)
{
    encoder = _encoder;
}


void Motor::AttachDriver(DriverBase* _driver)
{
    driver = _driver;
}


void Motor::CloseLoopControlTick()
{
    /************************************ First Called ************************************/
    static bool isFirstCalled = true;
    if (isFirstCalled)
    {
        int32_t angle;
        if (config.motionParams.encoderHomeOffset < MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS / 2)
        {
            angle =
                encoder->angleData.rectifiedAngle >
                config.motionParams.encoderHomeOffset + MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS / 2 ?
                encoder->angleData.rectifiedAngle - MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS :
                encoder->angleData.rectifiedAngle;
        } else
        {
            angle =
                encoder->angleData.rectifiedAngle <
                config.motionParams.encoderHomeOffset - MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS / 2 ?
                encoder->angleData.rectifiedAngle + MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS :
                encoder->angleData.rectifiedAngle;
        }

        controller->realLapPosition = angle;
        controller->realLapPositionLast = angle;
        controller->realPosition = angle;
        controller->realPositionLast = angle;

        isFirstCalled = false;
        return;
    }

    /********************************* Update Data *********************************/
    int32_t deltaLapPosition;

    // Read Encoder data
    controller->realLapPositionLast = controller->realLapPosition;
    controller->realLapPosition = encoder->angleData.rectifiedAngle;

    // Lap-Position calculate
    deltaLapPosition = controller->realLapPosition - controller->realLapPositionLast;
    if (deltaLapPosition > MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS >> 1)
        deltaLapPosition -= MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
    else if (deltaLapPosition < -MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS >> 1)
        deltaLapPosition += MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;

    // Naive-Position calculate
    controller->realPositionLast = controller->realPosition;
    controller->realPosition += deltaLapPosition;

    /********************************* Estimate Data *********************************/
    // Estimate Velocity
    controller->estVelocityIntegral += (
        (controller->realPosition - controller->realPositionLast) * motionPlanner.CONTROL_FREQUENCY
        + ((controller->estVelocity << 5) - controller->estVelocity)
    );
    controller->estVelocity = controller->estVelocityIntegral >> 5;
    controller->estVelocityIntegral -= (controller->estVelocity << 5);

    // Estimate Position
    controller->estLeadPosition = Controller::CompensateAdvancedAngle(controller->estVelocity);
    controller->estPosition = controller->realPosition + controller->estLeadPosition;

    // Estimate Error
    controller->estError = controller->softPosition - controller->estPosition;

    /************************************ Ctrl Loop ************************************/
    if (controller->softDisable ||
        !encoder->IsCalibrated() ||
        encoder->HasNoMagnet())
    {
        controller->ClearIntegral();    // clear integrals
        controller->focPosition = 0;    // clear outputs
        controller->focCurrent = 0;
        driver->Sleep();
    } else if (controller->isStalled)
    {
        // P1-32 fix: stall does NOT sleep - maintain minimum holding torque
        // to prevent arm free-fall under gravity. Fall through to normal
        // control loop which will output low holding current.
        controller->ClearIntegral();
    } else if (controller->softBrake)
    {
        controller->ClearIntegral();
        controller->focPosition = 0;
        controller->focCurrent = 0;
        driver->Brake();
    } else
    {
        switch (controller->modeRunning)
        {
            case MODE_STEP_DIR:
                controller->CalcDceToOutput(controller->softPosition, controller->softVelocity);
                break;
            case MODE_STOP:
                driver->Sleep();
                break;
            case MODE_COMMAND_Trajectory:
                controller->CalcDceToOutput(controller->softPosition, controller->softVelocity);
                break;
            case MODE_COMMAND_CURRENT:
                controller->CalcCurrentToOutput(controller->softCurrent);
                break;
            case MODE_COMMAND_VELOCITY:
                controller->CalcPidToOutput(controller->softVelocity);
                break;
            case MODE_COMMAND_POSITION:
                controller->CalcDceToOutput(controller->softPosition, controller->softVelocity);
                break;
            case MODE_PWM_CURRENT:
                controller->CalcCurrentToOutput(controller->softCurrent);
                break;
            case MODE_PWM_VELOCITY:
                controller->CalcPidToOutput(controller->softVelocity);
                break;
            case MODE_PWM_POSITION:
                controller->CalcDceToOutput(controller->softPosition, controller->softVelocity);
                break;
            default:
                break;
        }
    }

    /******************************* Mode Change Handle *******************************/
    if (controller->modeRunning != controller->requestMode)
    {
        controller->modeRunning = controller->requestMode;
        controller->softNewCurve = true;
    }

    /******************************* Update Hard-Goal *******************************/
    if (controller->goalVelocity > config.motionParams.ratedVelocity)
        controller->goalVelocity = config.motionParams.ratedVelocity;
    else if (controller->goalVelocity < -config.motionParams.ratedVelocity)
        controller->goalVelocity = -config.motionParams.ratedVelocity;
    if (controller->goalCurrent > config.motionParams.ratedCurrent)
        controller->goalCurrent = config.motionParams.ratedCurrent;
    else if (controller->goalCurrent < -config.motionParams.ratedCurrent)
        controller->goalCurrent = -config.motionParams.ratedCurrent;

    /******************************** Motion Plan *********************************/
    if ((controller->softDisable && !controller->goalDisable) ||
        (controller->softBrake && !controller->goalBrake))
    {
        controller->softNewCurve = true;
    }

    if (controller->softNewCurve)
    {
        controller->softNewCurve = false;
        controller->ClearIntegral();
        controller->ClearStallFlag();

        switch (controller->modeRunning)
        {
            case MODE_STOP:
                break;
            case MODE_COMMAND_POSITION:
                motionPlanner.positionTracker.NewTask(controller->estPosition, controller->estVelocity);
                break;
            case MODE_COMMAND_VELOCITY:
                motionPlanner.velocityTracker.NewTask(controller->estVelocity);
                break;
            case MODE_COMMAND_CURRENT:
                motionPlanner.currentTracker.NewTask(controller->focCurrent);
                break;
            case MODE_COMMAND_Trajectory:
                motionPlanner.trajectoryTracker.NewTask(controller->estPosition, controller->estVelocity);
                break;
            case MODE_PWM_POSITION:
                motionPlanner.positionTracker.NewTask(controller->estPosition, controller->estVelocity);
                break;
            case MODE_PWM_VELOCITY:
                motionPlanner.velocityTracker.NewTask(controller->estVelocity);
                break;
            case MODE_PWM_CURRENT:
                motionPlanner.currentTracker.NewTask(controller->focCurrent);
                break;
            case MODE_STEP_DIR:
                motionPlanner.positionInterpolator.NewTask(controller->estPosition, controller->estVelocity);
                // step/dir mode uses delta-position, so stay where we are
                controller->goalPosition = controller->estPosition;
                break;
            default:
                break;
        }
    }

    /******************************* Update Soft Goal *******************************/
    switch (controller->modeRunning)
    {
        case MODE_STOP:
            break;
        case MODE_COMMAND_POSITION:
            motionPlanner.positionTracker.CalcSoftGoal(controller->goalPosition);
            controller->softPosition = motionPlanner.positionTracker.go_location;
            controller->softVelocity = motionPlanner.positionTracker.go_velocity;
            break;
        case MODE_COMMAND_VELOCITY:
            motionPlanner.velocityTracker.CalcSoftGoal(controller->goalVelocity);
            controller->softVelocity = motionPlanner.velocityTracker.goVelocity;
            break;
        case MODE_COMMAND_CURRENT:
            motionPlanner.currentTracker.CalcSoftGoal(controller->goalCurrent);
            controller->softCurrent = motionPlanner.currentTracker.goCurrent;
            break;
        case MODE_COMMAND_Trajectory:
            motionPlanner.trajectoryTracker.CalcSoftGoal(controller->goalPosition, controller->goalVelocity);
            controller->softPosition = motionPlanner.trajectoryTracker.goPosition;
            controller->softVelocity = motionPlanner.trajectoryTracker.goVelocity;
            break;
        case MODE_PWM_POSITION:
            motionPlanner.positionTracker.CalcSoftGoal(controller->goalPosition);
            controller->softPosition = motionPlanner.positionTracker.go_location;
            controller->softVelocity = motionPlanner.positionTracker.go_velocity;
            break;
        case MODE_PWM_VELOCITY:
            motionPlanner.velocityTracker.CalcSoftGoal(controller->goalVelocity);
            controller->softVelocity = motionPlanner.velocityTracker.goVelocity;
            break;
        case MODE_PWM_CURRENT:
            motionPlanner.currentTracker.CalcSoftGoal(controller->goalCurrent);
            controller->softCurrent = motionPlanner.currentTracker.goCurrent;
            break;
        case MODE_STEP_DIR:
            motionPlanner.positionInterpolator.CalcSoftGoal(controller->goalPosition);
            controller->softPosition = motionPlanner.positionInterpolator.goPosition;
            controller->softVelocity = motionPlanner.positionInterpolator.goVelocity;
            break;
        default:
            break;
    }

    controller->softDisable = controller->goalDisable;
    controller->softBrake = controller->goalBrake;

    /******************************** State Check ********************************/
    int32_t current = abs(controller->focCurrent);

    // stallCurrentThreshold 需在状态机外层计算（overload 也要用）
    if (stallState.stallCurrentThreshold == 0) {
        // 第一次：按 ratedCurrent 95% 初始化
        stallState.stallCurrentThreshold = (int32_t)(config.motionParams.ratedCurrent * 95 / 100);
    }
    if (stallState.stallVelocityThreshold == 0) {
        stallState.stallVelocityThreshold = MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS / 5;
    }

    const int32_t stallThreshold = stallState.stallCurrentThreshold;

    // 重构阶段2.2 (2026-08-23): Stall 状态机（替换原 if-switch 段，#41 修复）
    if (stallState.enabled && !controller->softDisable)
    {
        switch (stallState.stallMode)
        {
            case STALL_IDLE:
            {
                // 触发条件：电流 >= 阈值 && |速度| < 阈值
                bool currentSpike = (current >= stallThreshold);
                bool lowVelocity = (abs(controller->estVelocity) < stallState.stallVelocityThreshold);
                if (currentSpike && lowVelocity)
                {
                    stallState.stallDetectTime += motionPlanner.CONTROL_PERIOD;
                    if (stallState.stallDetectTime >= stallState.detectThresholdTime)
                    {
                        // 200ms 持续低速度高电流 → 触发 RETREATING
                        stallState.stallMode = STALL_RETREATING;
                        stallState.stallRetreatTime = 0;
                        stallState.lastGoalPosition = controller->goalPosition;
                        // 记录运动方向（用最近 1ms 平均速度符号）
                        stallState.lastMoveDirection = (controller->estVelocity >= 0) ? +1 : -1;
                        // 主动上报 RETREATING（主控可显示进度）
                        CAN_TxHeaderTypeDef txHdr = {};
                        txHdr.StdId = (boardConfig.canNodeId << 7) | 0x7C;
                        txHdr.IDE = CAN_ID_STD;
                        txHdr.RTR = CAN_RTR_DATA;
                        txHdr.DLC = 8;
                        uint8_t txData[8] = {
                            (uint8_t)boardConfig.canNodeId,
                            (uint8_t)STALL_RETREATING,
                            (uint8_t)(current & 0xFF), (uint8_t)((current >> 8) & 0xFF),
                            (uint8_t)(stallState.enabled ? 1 : 0),
                            0, 0, 0
                        };
                        CAN_Send(&txHdr, txData);
                    }
                }
                else
                {
                    // 电流回落或速度恢复 → 重置检测计数
                    stallState.stallDetectTime = 0;
                }
                break;
            }
            case STALL_RETREATING:
            {
                // 计算回退目标（lastGoalPosition 减去 direction × retreatSteps）
                int32_t retreatTarget = stallState.lastGoalPosition
                                      - stallState.lastMoveDirection * stallState.retreatSteps;
                // 强制位置指令为回退目标（覆盖主控指令）
                controller->SetPositionSetPoint(retreatTarget);

                stallState.stallRetreatTime += motionPlanner.CONTROL_PERIOD;

                // 触发 LOCKED 条件：
                // 1. 回退超时（2s 内没回退完）
                // 2. 已回退到位（距离 < 1/10 圈）
                bool reachedRetreat =
                    (abs(controller->realPosition - retreatTarget) < MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS / 10);

                if (stallState.stallRetreatTime >= stallState.retreatTimeoutTime || reachedRetreat)
                {
                    stallState.stallMode = STALL_LOCKED;
                    controller->isStalled = true;
                    stallState.lockEntryCleared = false;  // 待下次 Ctrl Loop 入口清积分

                    // 主动上报 LOCKED（一次性）
                    CAN_TxHeaderTypeDef txHdr = {};
                    txHdr.StdId = (boardConfig.canNodeId << 7) | 0x7C;
                    txHdr.IDE = CAN_ID_STD;
                    txHdr.RTR = CAN_RTR_DATA;
                    txHdr.DLC = 8;
                    uint8_t txData[8] = {
                        (uint8_t)boardConfig.canNodeId,
                        (uint8_t)STALL_LOCKED,
                        (uint8_t)(current & 0xFF), (uint8_t)((current >> 8) & 0xFF),
                        (uint8_t)(stallState.enabled ? 1 : 0),
                        0, 0, 0
                    };
                    CAN_Send(&txHdr, txData);
                }
                break;
            }
            case STALL_LOCKED:
            {
                // LOCKED 状态：保持位置，不发新指令
                // ClearIntegral 在 Ctrl Loop 入口处理（第 105 行 isStalled 检查）
                // 这里无需操作
                break;
            }
        }
    }

    // Overload detect
    if ((controller->modeRunning != MODE_COMMAND_CURRENT) &&
        (controller->modeRunning != MODE_PWM_CURRENT) &&
        (current >= stallThreshold))
    {
        if (controller->overloadTime >= 1000 * 1000)
            controller->overloadFlag = true;
        else
            controller->overloadTime += motionPlanner.CONTROL_PERIOD;
    } else // auto clear overload flag when released
    {
        controller->overloadTime = 0;
        controller->overloadFlag = false;
    }

    /******************************** Update State ********************************/
    if (!encoder->IsCalibrated())
        controller->state = STATE_NO_CALIB;
    else if (controller->modeRunning == MODE_STOP)
        controller->state = STATE_STOP;
    else if (controller->isStalled)
        controller->state = STATE_STALL;
    else if (controller->overloadFlag)
        controller->state = STATE_OVERLOAD;
    else
    {
        if (controller->modeRunning == MODE_COMMAND_POSITION)
        {
            if ((controller->softPosition == controller->goalPosition)
                && (controller->softVelocity == 0))
                controller->state = STATE_FINISH;
            else
                controller->state = STATE_RUNNING;
        } else if (controller->modeRunning == MODE_COMMAND_VELOCITY)
        {
            if (controller->softVelocity == controller->goalVelocity)
                controller->state = STATE_FINISH;
            else
                controller->state = STATE_RUNNING;
        } else if (controller->modeRunning == MODE_COMMAND_CURRENT)
        {
            if (controller->softCurrent == controller->goalCurrent)
                controller->state = STATE_FINISH;
            else
                controller->state = STATE_RUNNING;
        } else
        {
            controller->state = STATE_FINISH;
        }
    }
}


void Motor::Controller::CalcCurrentToOutput(int32_t current)
{
    focCurrent = current;

    if (focCurrent > 0)
        focPosition = estPosition + context->SOFT_DIVIDE_NUM; // ahead phase of 90°
    else if (focCurrent < 0)
        focPosition = estPosition - context->SOFT_DIVIDE_NUM; // behind phase of 90°
    else focPosition = estPosition;

    context->driver->SetFocCurrentVector(focPosition, focCurrent);
}


void Motor::Controller::CalcPidToOutput(int32_t _speed)
{
    config->pid.vErrorLast = config->pid.vError;
    config->pid.vError = _speed - estVelocity;
    if (config->pid.vError > (1024 * 1024)) config->pid.vError = (1024 * 1024);
    if (config->pid.vError < (-1024 * 1024)) config->pid.vError = (-1024 * 1024);
    config->pid.outputKp = ((config->pid.kp) * (config->pid.vError));

    config->pid.integralRound += (config->pid.ki * config->pid.vError);
    config->pid.integralRemainder = config->pid.integralRound >> 10;
    config->pid.integralRound -= (config->pid.integralRemainder << 10);
    config->pid.outputKi += config->pid.integralRemainder;
    // integralRound limitation is  ratedCurrent*1024
    if (config->pid.outputKi > context->config.motionParams.ratedCurrent << 10)
        config->pid.outputKi = context->config.motionParams.ratedCurrent << 10;
    else if (config->pid.outputKi < -(context->config.motionParams.ratedCurrent << 10))
        config->pid.outputKi = -(context->config.motionParams.ratedCurrent << 10);

    config->pid.outputKd = config->pid.kd * (config->pid.vError - config->pid.vErrorLast);

    config->pid.output = (config->pid.outputKp + config->pid.outputKi + config->pid.outputKd) >> 10;
    if (config->pid.output > context->config.motionParams.ratedCurrent)
        config->pid.output = context->config.motionParams.ratedCurrent;
    else if (config->pid.output < -context->config.motionParams.ratedCurrent)
        config->pid.output = -context->config.motionParams.ratedCurrent;

    CalcCurrentToOutput(config->pid.output);
}


void Motor::Controller::CalcDceToOutput(int32_t _location, int32_t _speed)
{
    config->dce.pError = _location - estPosition;
    if (config->dce.pError > (3200)) config->dce.pError = (3200);   // limited pError to 1/16r (51200/16)
    if (config->dce.pError < (-3200)) config->dce.pError = (-3200);
    config->dce.vError = (_speed - estVelocity) >> 7;
    if (config->dce.vError > (4000)) config->dce.vError = (4000);   // limited vError
    if (config->dce.vError < (-4000)) config->dce.vError = (-4000);

    config->dce.outputKp = config->dce.kp * config->dce.pError;

    config->dce.integralRound += (config->dce.ki * config->dce.pError + config->dce.kv * config->dce.vError);
    config->dce.integralRemainder = config->dce.integralRound >> 7;
    config->dce.integralRound -= (config->dce.integralRemainder << 7);
    config->dce.outputKi += config->dce.integralRemainder;
    // limited to ratedCurrent * 1024, should be TUNED when use different scene
    if (config->dce.outputKi > context->config.motionParams.ratedCurrent << 10)
        config->dce.outputKi = context->config.motionParams.ratedCurrent << 10;
    else if (config->dce.outputKi < -(context->config.motionParams.ratedCurrent << 10))
        config->dce.outputKi = -(context->config.motionParams.ratedCurrent << 10);

    config->dce.outputKd = ((config->dce.kd) * (config->dce.vError));

    config->dce.output = (config->dce.outputKp + config->dce.outputKi + config->dce.outputKd) >> 10;
    if (config->dce.output > context->config.motionParams.ratedCurrent)
        config->dce.output = context->config.motionParams.ratedCurrent;
    else if (config->dce.output < -context->config.motionParams.ratedCurrent)
        config->dce.output = -context->config.motionParams.ratedCurrent;

    CalcCurrentToOutput(config->dce.output);
}


void Motor::Controller::SetCtrlMode(Motor::Mode_t _mode)
{
    requestMode = _mode;
}


void Motor::Controller::AddTrajectorySetPoint(int32_t _pos, int32_t _vel)
{
    SetPositionSetPoint(_pos);
    SetVelocitySetPoint(_vel);
}


void Motor::Controller::SetPositionSetPoint(int32_t _pos)
{
    goalPosition = _pos + context->config.motionParams.encoderHomeOffset;
}


bool Motor::Controller::SetPositionSetPointWithTime(int32_t _pos, float _time)
{
    int32_t deltaPos = abs(_pos - realPosition + context->config.motionParams.encoderHomeOffset);

    float pMax = (float) context->config.motionParams.ratedVelocityAcc * _time * _time / 4;
    if ((float) deltaPos > pMax)
    {
        context->config.motionParams.ratedVelocity = boardConfig.velocityLimit;
        SetPositionSetPoint(_pos);

        return false;
    } else
    {
        float discriminant = _time * _time - 4.0f * (float) deltaPos /
                                     (float) context->config.motionParams.ratedVelocityAcc;
        float vMax = _time * (float) context->config.motionParams.ratedVelocityAcc;
        vMax -= (float) context->config.motionParams.ratedVelocityAcc *
                sqrtf(fmaxf(0.0f, discriminant));
        vMax /= 2;

        context->config.motionParams.ratedVelocity = (int32_t) vMax;
        SetPositionSetPoint(_pos);

        return true;
    }
}


void Motor::Controller::SetVelocitySetPoint(int32_t _vel)
{
    if ((_vel >= -context->config.motionParams.ratedVelocity) &&
        (_vel <= context->config.motionParams.ratedVelocity))
    {
        goalVelocity = _vel;
    }
}


float Motor::Controller::GetPosition(bool _isLap)
{
    return _isLap ?
           (float) (realLapPosition - context->config.motionParams.encoderHomeOffset) /
           (float) (context->MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS)
                  :
           (float) (realPosition - context->config.motionParams.encoderHomeOffset) /
           (float) (context->MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);
}


float Motor::Controller::GetVelocity()
{
    return (float) estVelocity / (float) context->MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
}


float Motor::Controller::GetFocCurrent()
{
    return (float) focCurrent / 1000.f;
}


void Motor::Controller::SetCurrentSetPoint(int32_t _cur)
{
    if (_cur > context->config.motionParams.ratedCurrent)
        goalCurrent = context->config.motionParams.ratedCurrent;
    else if (_cur < -context->config.motionParams.ratedCurrent)
        goalCurrent = -context->config.motionParams.ratedCurrent;
    else
        goalCurrent = _cur;
}


void Motor::Controller::SetDisable(bool _disable)
{
    goalDisable = _disable;
}


void Motor::Controller::SetBrake(bool _brake)
{
    goalBrake = _brake;

}


void Motor::Controller::ClearStallFlag()
{
    // 重构阶段2.2 (2026-08-23): 完整复位状态机
    stalledTime = 0;
    isStalled = false;
    context->stallState.stallDetectTime = 0;
    context->stallState.stallRetreatTime = 0;
    context->stallState.stallMode = STALL_IDLE;
    context->stallState.lockEntryCleared = true;  // 已清，避免重复 ClearIntegral
}


int32_t Motor::Controller::CompensateAdvancedAngle(int32_t _vel)
{
    /*
     * The code is for DPS series sensors, need to measured and renew for TLE5012/MT6816.
     */

    int32_t compensate;

    if (_vel < 0)
    {
        if (_vel > -100000) compensate = 0;
        else if (_vel > -1300000) compensate = (((_vel + 100000) * 262) >> 20) - 0;
        else if (_vel > -2200000) compensate = (((_vel + 1300000) * 105) >> 20) - 300;
        else compensate = (((_vel + 2200000) * 52) >> 20) - 390;

        if (compensate < -430) compensate = -430;
    } else
    {
        if (_vel < 100000) compensate = 0;
        else if (_vel < 1300000) compensate = (((_vel - 100000) * 262) >> 20) + 0;
        else if (_vel < 2200000) compensate = (((_vel - 1300000) * 105) >> 20) + 300;
        else compensate = (((_vel - 2200000) * 52) >> 20) + 390;

        if (compensate > 430) compensate = 430;
    }

    return compensate;
}


void Motor::Controller::Init()
{
    requestMode = boardConfig.enableMotorOnBoot ? static_cast<Mode_t>(boardConfig.defaultMode) : MODE_STOP;

    modeRunning = MODE_STOP;
    state = STATE_STOP;

    realLapPosition = 0;
    realLapPositionLast = 0;
    realPosition = 0;
    realPositionLast = 0;

    estVelocityIntegral = 0;
    estVelocity = 0;
    estLeadPosition = 0;
    estPosition = 0;
    estError = 0;

    goalPosition = context->config.motionParams.encoderHomeOffset;
    goalVelocity = 0;
    goalCurrent = 0;
    goalDisable = false;
    goalBrake = false;

    softPosition = 0;
    softVelocity = 0;
    softCurrent = 0;
    softDisable = false;
    softBrake = false;
    softNewCurve = false;

    focPosition = 0;
    focCurrent = 0;

    stalledTime = 0;
    isStalled = false;

    overloadTime = 0;
    overloadFlag = false;

    config->pid.vError = 0;
    config->pid.vErrorLast = 0;
    config->pid.outputKp = 0;
    config->pid.outputKi = 0;
    config->pid.outputKd = 0;
    config->pid.integralRound = 0;
    config->pid.integralRemainder = 0;
    config->pid.output = 0;

    config->dce.pError = 0;
    config->dce.vError = 0;
    config->dce.outputKp = 0;
    config->dce.outputKi = 0;
    config->dce.outputKd = 0;
    config->dce.integralRound = 0;
    config->dce.integralRemainder = 0;
    config->dce.output = 0;
}


void Motor::Controller::ApplyPosAsHomeOffset()
{
    context->config.motionParams.encoderHomeOffset = realPosition %
                                                     context->MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
}


void Motor::Controller::AttachConfig(Motor::Controller::Config_t* _config)
{
    config = _config;
}


void Motor::Controller::ClearIntegral() const
{
    config->pid.integralRound = 0;
    config->pid.integralRemainder = 0;
    config->pid.outputKi = 0;

    config->dce.integralRound = 0;
    config->dce.integralRemainder = 0;
    config->dce.outputKi = 0;
}


