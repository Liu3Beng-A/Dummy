// Frozen actual ResetPositionCompletion from pre-fix snapshot d50efb1.
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
