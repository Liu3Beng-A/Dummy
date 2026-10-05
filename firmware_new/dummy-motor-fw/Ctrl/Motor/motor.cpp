#include "configurations.h"
#include "motor.h"

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

    // Requests are submitted by CAN; all ramp/hold transitions run at 20kHz.
    const uint32_t stopIrqMask = __get_PRIMASK();
    __disable_irq();
    const bool softStopOwnsControl = controller->TickSoftStop();
    __set_PRIMASK(stopIrqMask);

    /************************************ Ctrl Loop ************************************/
    if (controller->isStalled ||
        controller->softDisable ||
        !encoder->IsCalibrated())
    {
        controller->ClearIntegral();    // clear integrals
        controller->focPosition = 0;    // clear outputs
        controller->focCurrent = 0;
        driver->Sleep();
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

    if (!softStopOwnsControl)
    {
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

    }
    controller->softDisable = controller->goalDisable;
    controller->softBrake = controller->goalBrake;

    /******************************** State Check ********************************/
    int32_t current = abs(controller->focCurrent);

    // Stall detect
    if (controller->config->stallProtectSwitch)
    {
        if (// Current Mode
            ((controller->modeRunning == MODE_COMMAND_CURRENT ||
              controller->modeRunning == MODE_PWM_CURRENT) &&
             (current != 0))
            || // Other Mode
            current == config.motionParams.ratedCurrent)
        {
            if (abs(controller->estVelocity) < MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS / 5)
            {
                if (controller->stalledTime >= 1000 * 1000)
                    controller->isStalled = true;
                else
                    controller->stalledTime += motionPlanner.CONTROL_PERIOD;
            }
        } else // can ONLY clear stall flag  MANUALLY
        {
            controller->stalledTime = 0;
        }
    }

    // Overload detect
    if ((controller->modeRunning != MODE_COMMAND_CURRENT) &&
        (controller->modeRunning != MODE_PWM_CURRENT) &&
        (current == config.motionParams.ratedCurrent))
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
    if (controller->IsSoftStopLocked())
        controller->state = controller->softStopPhase == SoftStopProtocol::HOLDING ?
                            STATE_FINISH : STATE_RUNNING;
    else if (!encoder->IsCalibrated())
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
        focPosition = estPosition + Motor::SOFT_DIVIDE_NUM; // ahead phase of 90°
    else if (focCurrent < 0)
        focPosition = estPosition - Motor::SOFT_DIVIDE_NUM; // behind phase of 90°
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
    if (IsSoftStopLocked()) return;
    requestMode = _mode;
}

void Motor::Controller::RequestSoftStop(uint8_t action, uint16_t token)
{
    using namespace SoftStopProtocol;
    if (action == SoftStopProtocol::DISABLE)
    {
        if (softStopTokenValid && token == softStopToken && softStopPhase == DISABLED) return;
        softStopToken = token;
        softStopTokenValid = true;
        softStopLocked = true;
        softStopPhase = DECELERATING;
        softStopRequest = SoftStopProtocol::DISABLE;
        return;
    }
    if (action == STOP)
    {
        // Retries (including a delayed STOP after RELEASE) never restart a ramp.
        if (softStopTokenValid && token == softStopToken) return;
        softStopToken = token;
        softStopTokenValid = true;
        softStopLocked = true;
        softStopPhase = DECELERATING;
        softStopRequest = STOP;
    }
    else if (action == RELEASE && token == softStopToken && softStopLocked &&
             softStopPhase == HOLDING)
        softStopRequest = RELEASE;
    else if (action == ENABLE_HOLD &&
             (!softStopLocked || softStopPhase == DISABLED || softStopPhase == HOLDING))
    {
        if (softStopTokenValid && token == softStopToken && softStopPhase == NORMAL) return;
        softStopKeepHold = softStopPhase == HOLDING;
        softStopToken = token;
        softStopTokenValid = true;
        softStopLocked = true;
        softStopPhase = DECELERATING;
        softStopRequest = ENABLE_HOLD;
    }
}

void Motor::Controller::DisableForStop()
{
    softStopRequest = 0;
    softStopLocked = false;
    softStopPhase = SoftStopProtocol::DISABLED;
    softStopTokenValid = false;
    goalDisable = true;
    goalVelocity = 0;
    goalCurrent = 0;
    requestMode = MODE_STOP;
}

void Motor::Controller::CaptureStopPosition()
{
    softStopPosition = realPosition;
    goalPosition = softPosition = softStopPosition;
    goalVelocity = softVelocity = 0;
    goalCurrent = softCurrent = 0;
    modeRunning = requestMode = MODE_COMMAND_POSITION;
    softNewCurve = false;
    context->motionPlanner.positionTracker.NewTask(softStopPosition, 0);
    ClearIntegral();
    // Carry the current holding torque across the velocity -> position transition.
    config->dce.outputKi = focCurrent * 1024;
}

float Motor::Controller::GetSoftStopPosition()
{
    const int32_t position = (softStopPhase == SoftStopProtocol::HOLDING ||
                              softStopPhase == SoftStopProtocol::NORMAL) ?
                             softStopPosition : realPosition;
    return static_cast<float>(position - context->config.motionParams.encoderHomeOffset) /
           MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
}

bool Motor::Controller::TickSoftStop()
{
    using namespace SoftStopProtocol;
    const uint8_t action = softStopRequest;
    softStopRequest = 0;
    if (action == SoftStopProtocol::DISABLE)
    {
        DisableForStop();
        softStopTokenValid = true;
        softDisable = true;
        goalBrake = softBrake = false;
        modeRunning = MODE_STOP;
        softVelocity = softCurrent = 0;
        return true;
    }
    if (action == ENABLE_HOLD)
    {
        if (isStalled || !context->encoder->IsCalibrated())
        {
            softStopPhase = FAULT;
            goalDisable = softDisable = true;
            requestMode = modeRunning = MODE_STOP;
            return true;
        }
        goalDisable = softDisable = false;
        goalBrake = softBrake = false;
        if (!softStopKeepHold) CaptureStopPosition();
        softStopKeepHold = false;
        softStopLocked = false;
        softStopPhase = NORMAL;
    }
    else if (action == RELEASE)
    {
        softStopLocked = false;
        softStopPhase = NORMAL;
        // Keep the captured position and existing holding torque.
    }
    else if (action == STOP)
    {
        softStopStableTicks = 0;
        softStopStableAnchor = softStopStableMax = realPosition;
        softStopVelocitySum = 0;
        if (modeRunning == MODE_STOP || goalDisable || softDisable)
            softStopPhase = DISABLED; // STOP never energizes a disabled motor.
        else if (context->config.motionParams.ratedVelocityAcc <= 0)
        {
            softStopPhase = FAULT;
            goalDisable = true;
            requestMode = modeRunning = MODE_STOP;
        }
        else
        {
            context->motionPlanner.velocityTracker.SetVelocityAcc(
                context->config.motionParams.ratedVelocityAcc);
            context->motionPlanner.velocityTracker.NewTask(estVelocity);
            softVelocity = estVelocity;
            goalVelocity = goalCurrent = 0;
            modeRunning = requestMode = MODE_COMMAND_VELOCITY;
            softNewCurve = false;
            ClearIntegral();
            config->pid.vError = config->pid.vErrorLast = 0;
            config->pid.outputKi = focCurrent * 1024;
        }
    }
    if (!softStopLocked) return false;
    if (isStalled || !context->encoder->IsCalibrated()) softStopPhase = FAULT;
    if (softStopPhase == DECELERATING)
    {
        context->motionPlanner.velocityTracker.CalcSoftGoal(0);
        softVelocity = context->motionPlanner.velocityTracker.goVelocity;
        // A one-step encoder change at 20kHz produces 625 steps/s in the
        // existing velocity estimator. Judge its mean over 50ms, while every
        // position sample must remain inside an eight-step peak-to-peak range.
        if (softVelocity == 0)
        {
            if (realPosition < softStopStableAnchor) softStopStableAnchor = realPosition;
            if (realPosition > softStopStableMax) softStopStableMax = realPosition;
            const int64_t range = static_cast<int64_t>(softStopStableMax) - softStopStableAnchor;
            if (range > 8)
            {
                softStopStableTicks = 0;
                softStopStableAnchor = softStopStableMax = realPosition;
                softStopVelocitySum = 0;
            }
            else
            {
                softStopVelocitySum += estVelocity;
                if (++softStopStableTicks >= 1000)
                {
                    const int64_t limit = static_cast<int64_t>(MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS / 100) *
                                          softStopStableTicks;
                    if (softStopVelocitySum >= -limit && softStopVelocitySum <= limit)
                    {
                        CaptureStopPosition();
                        softStopPhase = HOLDING;
                    }
                    else
                    {
                        softStopStableTicks = 0;
                        softStopStableAnchor = softStopStableMax = realPosition;
                        softStopVelocitySum = 0;
                    }
                }
            }
        }
        else
        {
            softStopStableTicks = 0;
            softStopStableAnchor = softStopStableMax = realPosition;
            softStopVelocitySum = 0;
        }
    }
    return true;
}


void Motor::Controller::AddTrajectorySetPoint(int32_t _pos, int32_t _vel)
{
    SetPositionSetPoint(_pos);
    SetVelocitySetPoint(_vel);
}


void Motor::Controller::SetPositionSetPoint(int32_t _pos)
{
    if (IsSoftStopLocked()) return;
    goalPosition = _pos + context->config.motionParams.encoderHomeOffset;
}


bool Motor::Controller::SetPositionSetPointWithTime(int32_t _pos, float _time)
{
    if (IsSoftStopLocked()) return false;
    int32_t deltaPos = abs(_pos - realPosition + context->config.motionParams.encoderHomeOffset);

    float pMax = (float) context->config.motionParams.ratedVelocityAcc * _time * _time / 4;
    if ((float) deltaPos > pMax)
    {
        context->config.motionParams.ratedVelocity = boardConfig.velocityLimit;
        SetPositionSetPoint(_pos);

        return false;
    } else
    {
        float discriminant = _time * _time - 4 * (float) deltaPos /
                                         (float) context->config.motionParams.ratedVelocityAcc;
        float vMax = _time * (float) context->config.motionParams.ratedVelocityAcc;
        vMax -= (float) context->config.motionParams.ratedVelocityAcc *
                (sqrtf(fmaxf(0, discriminant)));
        vMax /= 2;

        context->config.motionParams.ratedVelocity = (int32_t) vMax;
        SetPositionSetPoint(_pos);

        return true;
    }
}


void Motor::Controller::SetVelocitySetPoint(int32_t _vel)
{
    if (IsSoftStopLocked()) return;
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
           (float) (Motor::MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS)
                  :
           (float) (realPosition - context->config.motionParams.encoderHomeOffset) /
           (float) (Motor::MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);
}


float Motor::Controller::GetVelocity()
{
    return (float) estVelocity / (float) Motor::MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
}


float Motor::Controller::GetFocCurrent()
{
    return (float) focCurrent / 1000.f;
}


void Motor::Controller::SetCurrentSetPoint(int32_t _cur)
{
    if (IsSoftStopLocked()) return;
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
    stalledTime = 0;
    isStalled = false;
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
    // 计算新的 home offset（realPosition 单圈内余数）
    context->config.motionParams.encoderHomeOffset = realPosition %
                                                     Motor::MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
    // 关键 v3.2 修复：同步归零 realPosition 和 realLapPosition，
    // 让 GetPosition() 立即返回 0 / 设定偏移
    realPosition = context->config.motionParams.encoderHomeOffset;
    realLapPosition = realPosition;
    realLapPositionLast = realPosition;
    realPositionLast = realPosition;
    // 清空相关估算值，避免积分误差残留
    estPosition = 0;
    estVelocity = 0;
    estError = 0;
    estVelocityIntegral = 0;
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


