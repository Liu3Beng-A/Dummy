// Actual production completion, stream phase classifier and DWT recorder.
// Deterministic scheduling only: this never accesses a device or measures ARM.
#include "motor_entry_test_common.inc"
#include "motor_completion_reset_baseline.inc"

void clockCompletion(unsigned count) {
    for(unsigned tick=0;tick<count;++tick) {
        controller.TickStreamClock();
        controller.TickPositionCompletion();
    }
}
void running() {
    controller.modeRunning=controller.requestMode=Motor::MODE_STREAM_JOINT;
    stream.armed=true;stream.state=STATE_RUNNING;stream.terminal=false;
}
void freshEndWindow() {
    stream.terminal=true;
    controller.goalPosition=controller.softPosition=controller.realPosition;
    controller.softVelocity=controller.estVelocity=0;
    clockCompletion(999);
    CHECK(!controller.positionEvidenceValid && !controller.positionSettled && !controller.positionCompleted);
    CHECK(controller.positionStableTicks==999 && !controller.positionStreamSuspended);
    clockCompletion(1);
    CHECK(controller.positionEvidenceValid && controller.positionSettled && controller.positionCompleted);
}
int main() {
    // No-inline sharing must preserve the complete actual old reset in every
    // mode and STOP lock state, including the intentionally latched result.
    for(unsigned mode=0;mode<12;++mode)for(bool locked:{false,true})for(bool newGoal:{false,true})
      for(int32_t real:{INT32_MIN,-10000,0,10000,INT32_MAX}) {
        reset();controller.modeRunning=Motor::Mode_t(mode);controller.locked=locked;
        controller.realPosition=real;controller.goalPosition=-1234;
        device.config.motionParams.encoderHomeOffset=5678;
        controller.positionStableTicks=923;controller.positionVelocitySum=-512345;
        controller.positionStableMin=-100;controller.positionStableMax=100;
        controller.positionCompletionTarget=9;controller.positionCompletionHome=10;
        controller.positionCompletionMode=11;controller.positionEvidenceValid=true;
        controller.positionSettled=controller.positionCompleted=controller.positionStreamSuspended=true;
        BaselineCompletionController old;
        static_cast<Motor::Controller&>(old)=controller;
        old.ResetPositionCompletion(newGoal);controller.ResetPositionCompletion(newGoal);
        CHECK(old.positionStableTicks==controller.positionStableTicks);
        CHECK(old.positionStableMin==controller.positionStableMin && old.positionStableMax==controller.positionStableMax);
        CHECK(old.positionVelocitySum==controller.positionVelocitySum);
        CHECK(old.positionCompletionTarget==controller.positionCompletionTarget && old.positionCompletionHome==controller.positionCompletionHome);
        CHECK(old.positionCompletionMode==controller.positionCompletionMode);
        CHECK(old.positionEvidenceValid==controller.positionEvidenceValid && old.positionSettled==controller.positionSettled);
        CHECK(old.positionCompleted==controller.positionCompleted);
        CHECK(!controller.positionStreamSuspended && controller.locked==locked);
      }
    reset();running();
    controller.positionStableTicks=999;controller.positionVelocitySum=12000;
    CHECK(!controller.TickPositionCompletion());
    CHECK(controller.positionStreamSuspended && !controller.positionEvidenceValid);
    CHECK(!controller.positionSettled && !controller.positionCompleted && !controller.positionStableTicks);
    // A changing reference never rebuilds a certificate or redoes the full
    // reset; the saved binding is deliberately not a live motion reference.
    const auto retiredTarget=controller.positionCompletionTarget;
    for(int tick=1;tick<=20000;++tick) {
        controller.goalPosition=controller.softPosition=tick;
        controller.realPosition=tick;controller.estVelocity=800;
        controller.TickStreamClock();
        CHECK(!controller.TickPositionCompletion());
        CHECK(controller.positionCompletionTarget==retiredTarget);
        CHECK(controller.positionStreamSuspended && !controller.positionEvidenceValid && !controller.positionStableTicks);
    }
    freshEndWindow();

    // A bad feedback sample immediately invalidates even during nonterminal
    // execution. Recovery/END requires 1000 wholly new good samples.
    reset();running();clockCompletion(50);controller.invalidFeedbackTicks=1;
    controller.InvalidatePositionEntry();
    CHECK(!controller.positionStreamSuspended && !controller.positionEvidenceValid);
    clockCompletion(200);CHECK(!controller.positionEvidenceValid && controller.positionStableTicks==0);
    stream.terminal=true;clockCompletion(1200);
    CHECK(!controller.positionEvidenceValid && !controller.positionStreamSuspended);
    controller.invalidFeedbackTicks=0;freshEndWindow();

    // Actual low-frequency setters retire all partial evidence when values
    // change away and back between interrupts; none can reveal an old window.
    for(unsigned action=0;action<5;++action) {
        reset();running();clockCompletion(70);
        switch(action) {
          case 0:controller.SetPositionSetPoint(10);controller.SetPositionSetPoint(0);break;
          case 1:controller.SetDisable(true);controller.SetDisable(false);break;
          case 2:controller.SetBrake(true);controller.SetBrake(false);break;
          case 3:controller.realPosition=3;controller.ApplyPosAsHomeOffset();controller.realPosition=0;controller.ApplyPosAsHomeOffset();break;
          case 4:controller.SetCtrlMode(Motor::Mode_t(2));controller.SetCtrlMode(Motor::MODE_STREAM_JOINT);break;
        }
        CHECK(!controller.positionStreamSuspended && !controller.positionEvidenceValid);
        clockCompletion(100);CHECK(!controller.positionEvidenceValid && !controller.positionStableTicks);
        freshEndWindow();
    }
    reset();running();clockCompletion(100);controller.RequestSoftStop(SoftStopProtocol::STOP,1);
    CHECK(controller.locked && !controller.positionEvidenceValid);
    clockCompletion(1200);CHECK(!controller.positionEvidenceValid && !controller.positionCompleted);

    // Exit to ordinary position mode must clear suspension and retain its
    // usual 50ms completion contract; live settlement and latched completion
    // still differ after an out-of-tolerance measurement.
    reset();running();clockCompletion(100);
    controller.modeRunning=controller.requestMode=Motor::MODE_COMMAND_POSITION;
    clockCompletion(999);CHECK(!controller.positionEvidenceValid);
    clockCompletion(1);CHECK(controller.positionEvidenceValid);
    controller.realPosition=10000;clockCompletion(1);
    CHECK(!controller.positionEvidenceValid && controller.positionCompleted);
    controller.goalPosition=10000;clockCompletion(1);CHECK(!controller.positionCompleted);

    reset();CHECK(controller.StreamTimingPhase()==0);
    controller.requestMode=Motor::MODE_STREAM_JOINT;CHECK(controller.StreamTimingPhase()==1);
    running();CHECK(controller.StreamTimingPhase()==2);
    stream.terminal=true;CHECK(controller.StreamTimingPhase()==1);
    stream.terminal=false;stream.armed=false;CHECK(controller.StreamTimingPhase()==1);
    stream.armed=true;stream.state=STATE_STARTED;CHECK(controller.StreamTimingPhase()==1);
    stream.state=STATE_FAULT;CHECK(controller.StreamTimingPhase()==1);
    controller.locked=true;CHECK(controller.StreamTimingPhase()==3);
    controller.modeRunning=controller.requestMode=Motor::MODE_COMMAND_POSITION;
    CHECK(controller.StreamTimingPhase()==3);

    // Recorder subtraction remains correct at the unsigned DWT wrap. Reading
    // diagnostics must not alias past the 28-word OPEN/START snapshot.
    for(auto& word:motorStreamTiming)word=0;
    motorTimingCheckpointCycle=UINT32_MAX-20;dwt.CYCCNT=29;MotorTimingCheckpoint(0);
    CHECK(motorStreamTiming[0]==50 && motorTimingCheckpointCycle==29);
    dwt.CYCCNT=39;MotorTimingCheckpoint(0);CHECK(motorStreamTiming[0]==50);
    for(unsigned field=52;field<=62;++field)motorStreamTiming[field-52]=100+field;
    for(unsigned field=23;field<=50;++field)entrySnapshot[field-23]=200+field;
    for(unsigned field=23;field<=63;++field) {
        command(DIAG,123,uint16_t(field));
        const auto expected=field<=50 ? 200+field : field==51 ? 0x544d0001U : field==63 ? 64U : 100+field;
        CHECK(Read32(replies.back().bytes+4)==expected);
        CHECK(replies.back().bytes[2]==field && Read16(replies.back().bytes)==123);
    }
    CHECK(irqMask==0);
    printf("motor_hot_completion_timing_checks=%u\n",checks);
}
