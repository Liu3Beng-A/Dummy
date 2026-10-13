#include "configurations.h"
#include "motor.h"

#include <cmath>
#include <can.h>


void Motor::Tick20kHz()
{
    // 1.Encoder data Update
    const uint32_t beforeEncoder=DWT->CYCCNT;
    // Every closed-loop tick has one bounded acquisition. Retry on the next
    // 50us tick, preserving the existing 100-invalid-tick protection window.
    // Electrical calibration has its own explicit three-attempt path.
    encoder->UpdateAngle(1);
    const uint32_t beforeControl=DWT->CYCCNT;
    const uint32_t encoderCycles=beforeControl-beforeEncoder;
    if (encoderCycles>motorControlTiming[9]) motorControlTiming[9]=encoderCycles;
    // 2.Motor Control Update
    CloseLoopControlTick();
    const uint32_t controlCycles=DWT->CYCCNT-beforeControl;
    if (controlCycles>motorControlTiming[10]) motorControlTiming[10]=controlCycles;
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
        if (!encoder->IsCalibrated() || !encoder->IsSampleValid())
        {
            // Even without a first usable sample, DISABLE/FAULT requests must
            // complete; do not make a missing encoder strand the CAN state.
            if (controller->invalidFeedbackTicks < 20000) ++controller->invalidFeedbackTicks;
            controller->TickProtection();
            const uint32_t irq = __get_PRIMASK(); __disable_irq();
            controller->TickSoftStop(); __set_PRIMASK(irq);
            driver->Sleep();
            return;
        }
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
        controller->feedbackInitialized = true;
#if MOTOR_TYPE == 8
        // HAND maps 100% to -100deg*16/360 motor revolutions. Establish the
        // assumed reference exactly once, after the first valid sample; no
        // movement and no Flash write occur at boot.
        config.motionParams.encoderHomeOffset = angle + static_cast<int32_t>(100.0f * 16 / 360 * MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);
#endif
        controller->goalPosition = controller->softPosition = controller->softStopPosition = angle;

        isFirstCalled = false;
        return;
    }

    /********************************* Update Data *********************************/
    int32_t deltaLapPosition;

    bool validSample = encoder->IsCalibrated() && encoder->IsSampleValid();
    if (validSample)
    {
        int32_t candidateDelta = static_cast<int32_t>(encoder->angleData.rectifiedAngle) - controller->realLapPosition;
        if (candidateDelta > MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS / 2) candidateDelta -= MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
        else if (candidateDelta < -MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS / 2) candidateDelta += MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
        const int64_t allowed = controller->transportMissingTicks == 0 ?
            boardConfig.velocityLimit / motionPlanner.CONTROL_FREQUENCY + 64 :
            static_cast<int64_t>(boardConfig.velocityLimit) *
                (controller->transportMissingTicks + 1) / motionPlanner.CONTROL_FREQUENCY + 64;
        // A parity-valid discontinuity is still not usable feedback. Genuine
        // SPI gaps get their elapsed-motion allowance; repeatedly implausible
        // valid packets do not grow that allowance until a jump is accepted.
        if (candidateDelta > allowed || candidateDelta < -allowed) validSample = false;
        controller->transportMissingTicks = 0;
    }
    else if (controller->transportMissingTicks < 20000) ++controller->transportMissingTicks;
    if (validSample) controller->invalidFeedbackTicks = 0;
    else if (controller->invalidFeedbackTicks < 20000) ++controller->invalidFeedbackTicks;
    if (!validSample) controller->InvalidatePositionEntry();
    // Invalid samples never advance the unwrap or the stability window.
    if (validSample)
    {
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
    }
    else controller->realPositionLast = controller->realPosition;

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

    controller->TickProtection();

    // Requests are submitted by CAN; all ramp/hold transitions run at 20kHz.
    const uint32_t stopIrqMask = __get_PRIMASK();
    __disable_irq();
    const bool softStopOwnsControl = controller->TickInternalProtection() || controller->TickSoftStop();
    __set_PRIMASK(stopIrqMask);

    /************************************ Ctrl Loop ************************************/
    if (controller->isStalled ||
        controller->softDisable ||
        !encoder->IsCalibrated() || controller->invalidFeedbackTicks >= 100)
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
            case MODE_STREAM_JOINT:
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
        if (controller->modeRunning == MODE_STREAM_JOINT && controller->requestMode != MODE_STREAM_JOINT) controller->CancelStream(StreamProtocol::OK);
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
        case MODE_STREAM_JOINT:
            controller->TickStream();
            break;
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

    // Recoverable protection is owned by TickProtection. isStalled remains
    // reserved for the existing electrical-encoder-calibration hard fault.

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
    const bool positionFinished = controller->TickPositionCompletion();
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
        if (controller->modeRunning == MODE_COMMAND_POSITION || controller->modeRunning == MODE_STREAM_JOINT)
        {
            if (positionFinished)
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
    const int32_t limit = EffectiveCurrentLimit();
    if (current > limit) current = limit;
    else if (current < -limit) current = -limit;
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
    const int32_t currentLimit = EffectiveCurrentLimit();
    if (config->pid.outputKi > currentLimit * 1024)
        config->pid.outputKi = currentLimit * 1024;
    else if (config->pid.outputKi < -currentLimit * 1024)
        config->pid.outputKi = -currentLimit * 1024;

    config->pid.outputKd = config->pid.kd * (config->pid.vError - config->pid.vErrorLast);

    config->pid.output = (config->pid.outputKp + config->pid.outputKi + config->pid.outputKd) >> 10;
    if (config->pid.output > currentLimit) config->pid.output = currentLimit;
    else if (config->pid.output < -currentLimit) config->pid.output = -currentLimit;

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
    const int32_t currentLimit = EffectiveCurrentLimit();
    if (config->dce.outputKi > currentLimit * 1024)
        config->dce.outputKi = currentLimit * 1024;
    else if (config->dce.outputKi < -currentLimit * 1024)
        config->dce.outputKi = -currentLimit * 1024;

    config->dce.outputKd = ((config->dce.kd) * (config->dce.vError));

    config->dce.output = (config->dce.outputKp + config->dce.outputKi + config->dce.outputKd) >> 10;
    if (config->dce.output > currentLimit) config->dce.output = currentLimit;
    else if (config->dce.output < -currentLimit) config->dce.output = -currentLimit;

    CalcCurrentToOutput(config->dce.output);
}


// Cold CAN/query/configuration entry only.
#if defined(__GNUC__)
__attribute__((optimize("Os")))
#endif
void Motor::Controller::SetCtrlMode(Motor::Mode_t _mode)
{
    if (IsSoftStopLocked()) return;
    if (requestMode != _mode) { InvalidatePositionEntry(); ResetDetection(); }
    requestMode = _mode;
}

// Low-frequency CAN entry; keep STOP compatibility within the 95KiB app area.
#if defined(__GNUC__)
__attribute__((optimize("Os")))
#endif
void Motor::Controller::RequestSoftStop(uint8_t action, uint16_t token)
{
    using namespace SoftStopProtocol;
    // START/release cannot steal ownership from an active curve or pending fault.
    if ((action == ENABLE_HOLD || action == RELEASE) && StreamActive()) return;
    if (action == STOP || action == SoftStopProtocol::DISABLE || action == ProtectionProtocol::STOP_PROTECTION) {
        InvalidatePositionEntry(); CancelStream();
    }
    if ((action == ENABLE_HOLD || action == RELEASE) &&
        (defaultsRestoreRequested || saveRequested || parameterSaveRequested || parameterSavingInProgress || coordinateSavingInProgress ||
         (calibrationTask && protectionPhase != ProtectionProtocol::CAL_FAILED) ||
         protectionPhase == ProtectionProtocol::LOCAL_STOPPING || protectionPhase == ProtectionProtocol::WAIT_GRANT ||
         protectionPhase == ProtectionProtocol::RETREATING || protectionPhase == ProtectionProtocol::PEER_STOPPING)) return;
    if (action == SoftStopProtocol::DISABLE)
    {
        if (protectionPhase != ProtectionProtocol::CAL_DONE && protectionPhase != ProtectionProtocol::SAVED &&
            protectionPhase != ProtectionProtocol::SAVE_PENDING)
            CancelInternal(ProtectionProtocol::CANCELLED_DISABLE);
        if (softStopTokenValid && token == softStopToken && softStopPhase == DISABLED) return;
        softStopToken = token;
        softStopTokenValid = true;
        softStopTokenAction = action;
        softStopLocked = true;
        softStopPhase = DECELERATING;
        softStopRequest = SoftStopProtocol::DISABLE;
        return;
    }
    const bool preservingProtection = action == ProtectionProtocol::STOP_PROTECTION;
    if (action == STOP || preservingProtection)
    {
        // Check before cancellation: the first user STOP cancels protection,
        // but its retransmission must not restart the 50ms stable window.
        // Action is part of the key: user STOP can reuse a protection token.
        if (softStopTokenValid && token == softStopToken && action == softStopTokenAction) return;
        // DISABLE already accepted is never overwritten by a later stop ramp.
        if (softStopRequest == SoftStopProtocol::DISABLE) return;
        const bool keepHold = preservingProtection && softStopLocked &&
            softStopPhase == HOLDING && !softStopRequest && !contextRequest && !internalStopping &&
            (protectionPhase == ProtectionProtocol::WAIT_GRANT || protectionPhase == ProtectionProtocol::PEER_HOLD) &&
            modeRunning == MODE_COMMAND_POSITION && !goalDisable && !softDisable &&
            IsFeedbackValid() && !isStalled && !configurationFault &&
            abs(static_cast<int64_t>(realPosition) - softStopPosition) <= 8 &&
            abs(estVelocity) <= MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS / 100;
        if (action == STOP) CancelInternal(ProtectionProtocol::CANCELLED_STOP);
        softStopToken = token;
        softStopTokenValid = true;
        softStopTokenAction = action;
        softStopLocked = true;
        // Bind a fresh master confirmation to an existing, fixed hold. This
        // never recaptures position, clears holding torque, or skips first stop.
        if (keepHold) return;
        softStopPhase = DECELERATING;
        softStopRequest = STOP;
    }
    else if (action == RELEASE && token == softStopToken && softStopLocked &&
             softStopPhase == HOLDING)
        softStopRequest = RELEASE;
    else if (action == ENABLE_HOLD &&
             (!softStopLocked || softStopPhase == DISABLED || softStopPhase == HOLDING || softStopPhase == FAULT))
    {
        if (!IsFeedbackValid() || configurationFault)
        {
            softStopToken = token; softStopTokenValid = true;
            softStopTokenAction = action;
            softStopLocked = true; softStopPhase = FAULT;
            goalDisable = softDisable = true; requestMode = modeRunning = MODE_STOP;
            return;
        }
        if (softStopTokenValid && token == softStopToken && softStopPhase == NORMAL) return;
        softStopKeepHold = softStopPhase == HOLDING;
        softStopToken = token;
        softStopTokenValid = true;
        softStopTokenAction = action;
        softStopLocked = true;
        softStopPhase = DECELERATING;
        softStopRequest = ENABLE_HOLD;
    }
}

void Motor::Controller::DisableForStop()
{
    InvalidatePositionEntry();
    CancelStream();
    const bool preserveCalibration = calibrationTask &&
        (protectionPhase == ProtectionProtocol::CAL_DONE || protectionPhase == ProtectionProtocol::SAVED ||
         protectionPhase == ProtectionProtocol::SAVE_PENDING);
    if (!preserveCalibration)
    {
        CancelInternal(ProtectionProtocol::CANCELLED_DISABLE);
        calibrationTask = false; protectionTask = 0; localStall = false;
        SetProtectionPhase(ProtectionProtocol::IDLE);
    }
    softStopRequest = 0;
    softStopLocked = false;
    softStopPhase = SoftStopProtocol::DISABLED;
    softStopTokenValid = false;
    goalDisable = softDisable = true;
    goalVelocity = 0;
    goalCurrent = 0;
    requestMode = modeRunning = MODE_STOP;
    ResetDetection();
}

void Motor::Controller::CaptureStopPosition()
{
    InvalidatePositionEntry();
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

// Cold CAN/query/configuration entry only.
#if defined(__GNUC__)
__attribute__((optimize("Os")))
#endif
float Motor::Controller::GetSoftStopPosition()
{
    if (!IsFeedbackValid()) return NAN;
    const int32_t position = (softStopPhase == SoftStopProtocol::HOLDING) ?
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
        if (isStalled || !IsFeedbackValid())
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
        calibrationTask = false; protectionTask = 0; localStall = false;
        SetProtectionPhase(ProtectionProtocol::IDLE);
        ResetDetection();
    }
    else if (action == RELEASE)
    {
        softStopLocked = false;
        softStopPhase = NORMAL;
        calibrationTask = false; protectionTask = 0; localStall = false;
        SetProtectionPhase(ProtectionProtocol::IDLE);
        ResetDetection();
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
    if (isStalled || !context->encoder->IsCalibrated() || invalidFeedbackTicks >= 100) softStopPhase = FAULT;
    if (softStopPhase == DECELERATING)
    {
        context->motionPlanner.velocityTracker.CalcSoftGoal(0);
        softVelocity = context->motionPlanner.velocityTracker.goVelocity;
        // A one-step encoder change at 20kHz produces 625 steps/s in the
        // existing velocity estimator. Judge its mean over 50ms, while every
        // position sample must remain inside an eight-step peak-to-peak range.
        if (softVelocity == 0 && IsFeedbackValid())
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


// CAN setpoint entry; preserve the 20kHz control/arrival optimization.
#if defined(__GNUC__)
__attribute__((optimize("Os")))
#endif
void Motor::Controller::SetPositionSetPoint(int32_t _pos)
{
    if (IsSoftStopLocked()) return;
    const int64_t target = static_cast<int64_t>(_pos) + context->config.motionParams.encoderHomeOffset;
    if (target > INT32_MAX || target < INT32_MIN) return;
    const uint32_t irq = __get_PRIMASK(); __disable_irq();
    if (goalPosition != target)
    {
        goalPosition = static_cast<int32_t>(target);
        ResetPositionCompletion(true);
        state = STATE_RUNNING; // The immediate 0x07 ACK must not carry old FINISH.
    }
    const int64_t error = static_cast<int64_t>(goalPosition) - realPosition;
    const int32_t tolerance = MotionCompletionProtocol::PositionToleranceSteps(boardConfig.canNodeId);
    if (error < -tolerance || error > tolerance)
    {
        if (!motionIntent) ResetDetection(true);
        motionIntent = true;
    }
    __set_PRIMASK(irq);
}

// Low-frequency setters use a short atomic invalidation; the 20kHz reset
// function itself retains its original optimization and constant-time work.
#if defined(__GNUC__)
__attribute__((optimize("Os")))
#endif
void Motor::Controller::InvalidatePositionEntry()
{
    const uint32_t irq = __get_PRIMASK(); __disable_irq();
    ResetPositionCompletion();
    __set_PRIMASK(irq);
}

void Motor::Controller::ResetPositionCompletion(bool newGoal)
{
    positionStableTicks = 0;
    positionStableMin = positionStableMax = realPosition;
    positionVelocitySum = 0;
    positionCompletionTarget = goalPosition;
    positionCompletionHome = context->config.motionParams.encoderHomeOffset;
    positionCompletionMode = uint8_t(modeRunning);
    positionEvidenceValid = false;
    positionSettled = false;
    if (newGoal) positionCompleted = false;
}

bool Motor::Controller::TickPositionCompletion()
{
    using namespace MotionCompletionProtocol;
    const bool streamMode = modeRunning == MODE_STREAM_JOINT && requestMode == MODE_STREAM_JOINT && StreamTerminal();
    if (positionCompletionTarget != goalPosition ||
        positionCompletionHome != context->config.motionParams.encoderHomeOffset ||
        positionCompletionMode != uint8_t(modeRunning) ||
        protectionPhase != ProtectionProtocol::IDLE || calibrationTask || invalidFeedbackTicks != 0 ||
        ((!streamMode) && (modeRunning != MODE_COMMAND_POSITION || requestMode != MODE_COMMAND_POSITION)) || IsSoftStopLocked() || !IsFeedbackValid() ||
        goalDisable || softDisable || goalBrake || softBrake || isStalled || overloadFlag)
    {
        ResetPositionCompletion(true);
        return false;
    }
    if (!IsOrdinaryPositionArrival())
    {
        ResetPositionCompletion();
        return boardConfig.canNodeId != 8 && positionCompleted;
    }
    if (!positionStableTicks && !positionSettled)
        positionStableMin = positionStableMax = realPosition;
    if (realPosition < positionStableMin) positionStableMin = realPosition;
    if (realPosition > positionStableMax) positionStableMax = realPosition;
    if (static_cast<int64_t>(positionStableMax) - positionStableMin > StableRangeSteps(boardConfig.canNodeId))
    {
        ResetPositionCompletion();
        return boardConfig.canNodeId != 8 && positionCompleted;
    }
    positionVelocitySum += estVelocity;
    if (++positionStableTicks >= STABLE_TICKS)
    {
        const int64_t limit = static_cast<int64_t>(STABLE_SPEED_STEPS) * positionStableTicks;
        if (positionVelocitySum < -limit || positionVelocitySum > limit)
        {
            ResetPositionCompletion();
            return boardConfig.canNodeId != 8 && positionCompleted;
        }
        // Publish only a completed 50ms window. Division belongs to the cold
        // diagnostic capture, not this 20kHz path. The global clock also advances
        // while encoder calibration temporarily owns the control callback.
        positionEvidenceTick = StreamClock();
        positionEvidenceRange = uint32_t(positionStableMax - positionStableMin);
        positionEvidenceVelocitySum = positionVelocitySum;
        positionEvidenceValid = true;
        positionSettled = true;
        positionCompleted = true;
        if (motionIntent)
        {
            ResetDetection();
            detectionReset = ProtectionProtocol::DR_COMPLETED;
            detectionResetDetail = 0;
            detectionGate = ProtectionProtocol::DG_NO_MOTION;
        }
        positionStableTicks = 0;
        positionStableMin = positionStableMax = realPosition;
        positionVelocitySum = 0;
    }
    return boardConfig.canNodeId == 8 ? positionSettled : positionCompleted;
}

bool Motor::Controller::IsOrdinaryPositionArrival(bool referenceFinished) const
{
    const int64_t error = static_cast<int64_t>(goalPosition) - realPosition;
    const int32_t tolerance = MotionCompletionProtocol::PositionToleranceSteps(boardConfig.canNodeId);
    const bool streamMode = modeRunning == MODE_STREAM_JOINT && requestMode == MODE_STREAM_JOINT && StreamTerminal();
    if ((!streamMode && (modeRunning != MODE_COMMAND_POSITION || requestMode != MODE_COMMAND_POSITION)) ||
        error < -tolerance || error > tolerance) return false;
    if (referenceFinished) return softPosition == goalPosition && softVelocity == 0;
    // Detection can suppress tolerated endpoint correction slightly before
    // the last reference tick; completion still requires the full reference.
    const int64_t referenceError = static_cast<int64_t>(goalPosition) - softPosition;
    return referenceError >= -tolerance && referenceError <= tolerance;
}

// Low-frequency CAN query only; keep the 20kHz controller optimization unchanged.
#if defined(__GNUC__)
__attribute__((optimize("Os")))
#endif
uint8_t Motor::Controller::GetPositionCompletionStatus(float expectedMotorR) const
{
    using namespace MotionCompletionProtocol;
    uint8_t flags = REPLY_MARKER;
    if (!MotionCompletionProtocol::Finite(expectedMotorR)) return flags;
    const float expectedSteps = expectedMotorR * STEPS_PER_REV;
    // Check before conversion, including float rounding at the int32 boundary.
    if (expectedSteps < -2147483648.0f || expectedSteps >= 2147483648.0f) return flags;
    const int64_t expected = static_cast<int32_t>(expectedSteps) +
                             static_cast<int64_t>(context->config.motionParams.encoderHomeOffset);
    const bool feedbackValid = IsFeedbackValid();
    if (feedbackValid) flags |= ENCODER_READY;
    if (softPosition == goalPosition && softVelocity == 0) flags |= PLANNER_READY;
    const int64_t error = static_cast<int64_t>(realPosition) - goalPosition;
    const int32_t tolerance = PositionToleranceSteps(static_cast<uint8_t>(boardConfig.canNodeId));
    if (error >= -tolerance && error <= tolerance) flags |= POSITION_READY;
    if (overloadFlag) flags |= CURRENT_CMD_LIMITED;
    if (modeRunning != MODE_COMMAND_POSITION || requestMode != MODE_COMMAND_POSITION ||
        IsSoftStopLocked() || !feedbackValid || goalDisable || softDisable ||
        goalBrake || softBrake || isStalled) flags |= DRIVE_BLOCKED;
    if (expected != goalPosition || requestMode != MODE_COMMAND_POSITION) return flags;
    flags |= TARGET_READY; // This goal was adopted, even if control is now blocked.
    if (!(flags & (DRIVE_BLOCKED | CURRENT_CMD_LIMITED)) &&
        positionSettled && positionCompletionTarget == goalPosition && state == STATE_FINISH &&
        softPosition == goalPosition && softVelocity == 0 && error >= -tolerance && error <= tolerance)
        flags |= SETTLED;
    return flags;
}


// Cold CAN/query/configuration entry only.
#if defined(__GNUC__)
__attribute__((optimize("Os")))
#endif
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


// Cold CAN/query/configuration entry only.
#if defined(__GNUC__)
__attribute__((optimize("Os")))
#endif
void Motor::Controller::SetVelocitySetPoint(int32_t _vel)
{
    if (IsSoftStopLocked()) return;
    if ((_vel >= -context->config.motionParams.ratedVelocity) &&
        (_vel <= context->config.motionParams.ratedVelocity))
    {
        goalVelocity = _vel;
        if (_vel && !motionIntent) ResetDetection(true);
        motionIntent = _vel != 0;
    }
}


// Cold CAN/query/configuration entry only.
#if defined(__GNUC__)
__attribute__((optimize("Os")))
#endif
float Motor::Controller::GetPosition(bool _isLap)
{
    if (!IsFeedbackValid()) return NAN;
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


// Cold CAN/query/configuration entry only.
#if defined(__GNUC__)
__attribute__((optimize("Os")))
#endif
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
    if (goalDisable != _disable) InvalidatePositionEntry();
    goalDisable = _disable;
}


void Motor::Controller::SetBrake(bool _brake)
{
    if (goalBrake != _brake) InvalidatePositionEntry();
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


// Startup-only setup; preserve the periodic controller's build options.
#if defined(__GNUC__)
__attribute__((optimize("Os")))
#endif
void Motor::Controller::Init()
{
    requestMode = boardConfig.enableMotorOnBoot ? static_cast<Mode_t>(boardConfig.defaultMode) : MODE_STOP;
    if (requestMode == MODE_STREAM_JOINT) requestMode = MODE_COMMAND_POSITION; // Stream always requires a fresh session.

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
    goalDisable = requestMode == MODE_STOP;
    goalBrake = false;

    softPosition = 0;
    softVelocity = 0;
    softCurrent = 0;
    softDisable = goalDisable;
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
    if (goalDisable) softStopPhase = SoftStopProtocol::DISABLED;
}


// Cold CAN/query/configuration entry only.
#if defined(__GNUC__)
__attribute__((optimize("Os")))
#endif
void Motor::Controller::ApplyPosAsHomeOffset()
{
    InvalidatePositionEntry();
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


