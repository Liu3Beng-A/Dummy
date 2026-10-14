// Frozen actual functions from pre-fix snapshot d50efb1.
// Motor source SHA256 7c373f09ee1cd01e3f263d55d5af770ca76b161a0310005c1ca6d8ba639a4a68
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

int32_t Motor::Controller::EffectiveCurrentLimit() const {
    int32_t limit = context->config.motionParams.ratedCurrent;
    if (MovingPhase(protectionPhase)) limit = Min(limit, calibrationTask ? calibrationCurrent : retreatCurrent);
    else if (localStall || protectionPhase == PEER_HOLD || protectionPhase == HOLD_FAILED || protectionPhase == HOLD_SUCCESS)
        limit = Min(limit, holdingCurrent);
    return Max(0, limit);
}
