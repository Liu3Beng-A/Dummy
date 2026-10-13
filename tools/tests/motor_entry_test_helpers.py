"""Extract actual bounded feedback/completion implementations; no hardware."""
from pathlib import Path

def function_text(source,signature):
    start=source.index(signature);brace=source.index('{',start);depth=1;end=brace+1
    while depth:
        depth+=(source[end]=='{')-(source[end]=='}');end+=1
    return source[start:end]

def completion_source(repo):
    motor=repo/'firmware_new/dummy-motor-fw/Ctrl/Motor'
    source=(motor/'motor.cpp').read_text(encoding='utf-8-sig')
    protection=(motor/'stall_protection.cpp').read_text(encoding='utf-8-sig')
    signatures=('void Motor::Controller::ResetPositionCompletion(bool newGoal)',
                'void Motor::Controller::InvalidatePositionEntry()',
                'bool Motor::Controller::TickPositionCompletion()',
                'bool Motor::Controller::IsOrdinaryPositionArrival(bool referenceFinished)',
                'void Motor::Controller::SetCtrlMode(Motor::Mode_t _mode)',
                'void Motor::Controller::SetPositionSetPoint(int32_t _pos)',
                'void Motor::Controller::SetDisable(bool _disable)',
                'void Motor::Controller::SetBrake(bool _brake)',
                'void Motor::Controller::ApplyPosAsHomeOffset()')
    return '\n'.join(function_text(source,s) for s in signatures)+'\n'+function_text(protection,'void Motor::Controller::SetProtectionPhase(uint8_t phase, uint8_t reason)')

def feedback_source(repo):
    source=(repo/'firmware_new/dummy-motor-fw/Ctrl/Motor/motor.cpp').read_text(encoding='utf-8-sig')
    start=source.index('    bool validSample =');end=source.index('    // Estimate Position',start)
    return source[start:end]
