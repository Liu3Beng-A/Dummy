// Actual production completion window, estimator update, entry snapshots and
// CAN controller. The encoder samples and timer scheduling are deterministic;
// no USB/CAN/STLink device or physical plant is accessed.
#include "motor_entry_test_common.inc"

struct FeedbackProbe {
 struct Encoder {
  bool calibrated=true,valid=true;
  struct {int32_t rectifiedAngle=0;} angleData;
  bool IsCalibrated()const{return calibrated;}
  bool IsSampleValid()const{return valid;}
 } sample;
 Encoder* encoder=&sample;
 Motor::Controller* controller=&::controller;
 struct {static constexpr int32_t CONTROL_FREQUENCY=20000;} motionPlanner;
 static constexpr int32_t MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS=51200;
 void Tick(int32_t absolute,bool valid=true) {
  int32_t angle=absolute%MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
  if(angle<0)angle+=MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
  sample.angleData.rectifiedAngle=angle;sample.valid=valid;
  controller->TickStreamClock();
  int32_t deltaLapPosition;
#include "motor_entry_feedback_under_test.inc"
  controller->TickPositionCompletion();
 }
};

void windowTicks(unsigned n,int32_t velocity=0) {
 for(unsigned i=0;i<n;++i){controller.TickStreamClock();controller.estVelocity=velocity;controller.TickPositionCompletion();}
}
void requireNewWindow() {
 CHECK(!controller.positionEvidenceValid);
 windowTicks(999);CHECK(!controller.positionEvidenceValid);
 // A home/mode binding change can consume its first tick rebinding the window.
 windowTicks(2);CHECK(controller.positionEvidenceValid);
}
void point(uint16_t token,uint16_t seq,int32_t q,uint8_t flags) {
 command(Q,token,seq,uint32_t(q));command(V,token,seq);command(A,token,seq);
 const uint32_t payload=CrcEndpoint(token,seq,flags,q,0,0)|(uint32_t(flags)<<16);
 command(PREPARE,token,seq,payload);command(COMMIT,token,seq,payload);
}
void begin(uint16_t token) {
 command(OPEN,token,uint16_t(VERSION)|uint16_t(1U<<(boardConfig.canNodeId-1))<<8);
 CHECK(stream.state==STATE_OPEN && stream.session==token);
 point(token,0,stream.position,SEED);point(token,1,stream.position,END);
 CHECK(stream.seedCommitted && stream.committed==1);
}
void closeSession(uint16_t token){command(ABORT,token);CHECK(stream.state==STATE_IDLE);}

int main() {
 reset();controller.InvalidatePositionEntry();windowTicks(999);
 CHECK(!controller.positionEvidenceValid && controller.PositionEntryEvidenceAge()==UINT32_MAX);
 windowTicks(1);CHECK(controller.positionEvidenceValid && controller.PositionEntryEvidenceAge()==0);
 CHECK(controller.positionEvidenceRange==0 && controller.positionEvidenceVelocitySum==0);

 // A single actual bad sample must not count toward a fresh 50ms certificate.
 reset();FeedbackProbe feedback;feedback.Tick(0,false);
 CHECK(controller.invalidFeedbackTicks==1 && !controller.positionEvidenceValid && controller.positionStableTicks==0);
 for(unsigned i=0;i<999;++i)feedback.Tick(0);
 CHECK(controller.invalidFeedbackTicks==0 && !controller.positionEvidenceValid && controller.positionStableTicks==999);
 feedback.Tick(0);CHECK(controller.positionEvidenceValid);
 controller.InvalidatePositionEntry();
 for(unsigned i=0;i<1100;++i)feedback.Tick(0,false);
 CHECK(!controller.positionEvidenceValid && controller.positionStableTicks==0);

 // Bounded 1..4 subdivision pulse noise traverses the real unwrap + IIR.
 // Instantaneous raw speed can exceed512 while the signed50ms mean is stable.
 for(int32_t amplitude=1;amplitude<=4;++amplitude)for(int direction:{-1,1}) {
  reset();controller.InvalidatePositionEntry();FeedbackProbe pulse;
  for(unsigned tick=1;tick<=1040;++tick)pulse.Tick(tick%40==0?amplitude*direction:0);
  CHECK(controller.positionEvidenceValid && controller.invalidFeedbackTicks==0);
  CHECK(controller.estVelocity>512 || controller.estVelocity<-512);
  CHECK(controller.positionEvidenceRange==uint32_t(amplitude));
  const auto raw=controller.estVelocity;
  begin(55);CHECK(entrySnapshot[1]==0 && int32_t(entrySnapshot[3])==raw);
  command(START,55,1,START_DELAY_TICKS);
  CHECK(stream.armed && entrySnapshot[11]==55 && entrySnapshot[12]==0 && int32_t(entrySnapshot[14])==raw);
  CHECK(entrySnapshot[0]==55 && entrySnapshot[1]==0);
 }
 reset();controller.estVelocity=857;begin(56);command(START,56,1,START_DELAY_TICKS);
 CHECK(stream.armed && entrySnapshot[3]==857 && entrySnapshot[14]==857);

 // Real encoder drift remains a refusal even if its positional range is small.
 for(int direction:{-1,1}) {
  reset();controller.InvalidatePositionEntry();FeedbackProbe drift;
  for(unsigned tick=1;tick<=3000;++tick)drift.Tick(direction*int32_t(tick/20));
  CHECK(controller.invalidFeedbackTicks==0 && !controller.positionEvidenceValid);
  CHECK(controller.realPosition==direction*150);
  CHECK(controller.CaptureStreamOpen(57)&(1U<<12));
 }

 // Production window boundary checks. Input injection here is explicit and
 // tests acceptance arithmetic; it does not fabricate a finished certificate.
 for(int32_t mean:{-513,-512,512,513}) {
  reset();controller.InvalidatePositionEntry();windowTicks(1000,mean);
  const bool good=mean>=-512 && mean<=512;
  CHECK(controller.positionEvidenceValid==good);
  if(good){CHECK(controller.CaptureStreamOpen(58)==0);CHECK(int32_t(entrySnapshot[24])==mean);}
 }
 for(unsigned node:{1U,6U,7U}) {
  reset();boardConfig.canNodeId=node;controller.InvalidatePositionEntry();windowTicks(1);
  controller.realPosition=MotionCompletionProtocol::StableRangeSteps(node);
  windowTicks(999);CHECK(controller.positionEvidenceValid);
  CHECK(controller.positionEvidenceRange==uint32_t(MotionCompletionProtocol::StableRangeSteps(node)));
  controller.InvalidatePositionEntry();controller.realPosition=0;windowTicks(1);
  controller.realPosition=MotionCompletionProtocol::StableRangeSteps(node)+1;windowTicks(1);
  CHECK(!controller.positionEvidenceValid && controller.positionStableTicks==0);
 }
 boardConfig.canNodeId=1;
 reset();for(unsigned i=0;i<1000;++i)controller.TickStreamClock();
 CHECK(controller.PositionEntryEvidenceAge()==1000 && controller.CaptureStreamOpen(59)==0);
 reset();for(unsigned i=0;i<1001;++i)controller.TickStreamClock();
 CHECK(controller.PositionEntryEvidenceAge()==1001 && (controller.CaptureStreamOpen(60)&(1U<<12)));
 CHECK(entrySnapshot[22]==1001 && entrySnapshot[10]==1 && !controller.positionEvidenceValid);

 // Changes away and back between control ticks cannot reuse previous evidence.
 reset();controller.SetPositionSetPoint(100);controller.SetPositionSetPoint(0);requireNewWindow();
 reset();controller.SetCtrlMode(Motor::Mode(2));controller.SetCtrlMode(Motor::MODE_COMMAND_POSITION);requireNewWindow();
 reset();controller.realPosition=3;controller.ApplyPosAsHomeOffset();controller.realPosition=0;controller.ApplyPosAsHomeOffset();requireNewWindow();
 reset();controller.SetDisable(true);controller.SetDisable(false);requireNewWindow();
 reset();controller.SetBrake(true);controller.SetBrake(false);requireNewWindow();
 reset();controller.SetProtectionPhase(1);controller.SetProtectionPhase(0);requireNewWindow();
 reset();controller.RequestSoftStop(SoftStopProtocol::STOP,1);controller.locked=false;requireNewWindow();
 reset();controller.calibrationTask=true;windowTicks(1);controller.calibrationTask=false;requireNewWindow();
 reset();controller.InvalidatePositionEntry();for(unsigned i=0;i<1001;++i)controller.TickStreamClock();
 CHECK(controller.PositionEntryEvidenceAge()==UINT32_MAX);requireNewWindow();

 // A refusal between callbacks restarts ALL of a partly accumulated window.
 reset();controller.InvalidatePositionEntry();windowTicks(900);controller.softVelocity=1;
 CHECK(controller.CaptureStreamOpen(61)&(1U<<13));CHECK(controller.positionStableTicks==0);
 controller.softVelocity=0;windowTicks(99);CHECK(!controller.positionEvidenceValid);
 windowTicks(901);CHECK(controller.positionEvidenceValid);

 // START preserves OPEN evidence and independently rejects changed hard gates.
 for(unsigned bit:{2U,3U,4U,6U,8U,9U,10U,11U,13U,15U,16U,19U}) {
  reset();begin(62);const uint32_t originalOpen=entrySnapshot[1];
  switch(bit){
   case 2:controller.locked=true;break;case 3:controller.feedback=false;break;
   case 4:controller.goalDisable=true;break;case 6:controller.goalBrake=true;break;
   case 8:controller.isStalled=true;break;case 9:controller.calibrationTask=true;break;
   case 10:calibrationBusy=true;break;case 11:controller.protectionPhase=1;break;
   case 13:controller.softVelocity=1;break;case 15:controller.softPosition=1;break;
   case 16:controller.realPosition=2000;break;case 19:controller.goalPosition=controller.softPosition=1;break;
  }
  command(START,62,1,START_DELAY_TICKS);
  CHECK(!stream.armed && stream.reason==BAD_SEED && entrySnapshot[11]==62 && (entrySnapshot[12]&(1U<<bit)));
  CHECK(entrySnapshot[0]==62 && entrySnapshot[1]==originalOpen);
 }
 reset();begin(65);controller.realPosition=3;controller.ApplyPosAsHomeOffset();windowTicks(1001);
 CHECK(controller.positionEvidenceValid && controller.PositionEntryEvidenceAge()==0 && controller.goalPosition==0);
 command(START,65,1,START_DELAY_TICKS);
 CHECK(!stream.armed && stream.reason==BAD_SEED && (entrySnapshot[12]&(1U<<19)) && !(entrySnapshot[12]&(1U<<12)));
 reset();begin(63);controller.softVelocity=1;command(START,63,1,START_DELAY_TICKS);
 const auto failedBits=entrySnapshot[12];controller.softVelocity=0;command(START,63,1,START_DELAY_TICKS);
 CHECK(entrySnapshot[12]==failedBits && entrySnapshot[15]==1);
 reset();begin(64);command(START,64,1,START_DELAY_TICKS);const auto anchor=stream.anchor;
 controller.estVelocity=9999;ticks(100);command(START,64,1,START_DELAY_TICKS-100);
 CHECK(stream.armed && stream.anchor==anchor && entrySnapshot[14]==0);
 for(unsigned field=23;field<=DIAG_LAST;++field){command(DIAG,123,uint16_t(field));CHECK(Read32(replies.back().bytes+4)==entrySnapshot[field-23]);}

 // After retirement history rolls over, a reused old START token belongs to
 // its NEW OPEN. Historical failure/success cannot bypass the new START gate.
 for(bool failed:{false,true}) {
  reset();begin(70);
  if(failed)controller.softVelocity=1;
  command(START,70,1,START_DELAY_TICKS);CHECK(entrySnapshot[11]==70);
  if(!failed){ticks(START_DELAY_TICKS);command(SYNC,70,1,0);ticks(1401);controller.PollStreamMain();CHECK(stream.state==STATE_COMPLETED);}
  closeSession(70);controller.locked=false;controller.softVelocity=0;
  for(uint16_t ss=71;ss<=80;++ss){certify();command(OPEN,ss,uint16_t(VERSION)|uint16_t(1)<<8);CHECK(stream.state==STATE_OPEN);closeSession(ss);}
  CHECK(!stream.Retired(70));certify();begin(70);
  CHECK(entrySnapshot[11]==0);command(DIAG,123,DIAG_START_SESSION);CHECK(Read32(replies.back().bytes+4)==0);
  controller.softVelocity=1;command(START,70,1,START_DELAY_TICKS);
  CHECK(entrySnapshot[11]==70 && (entrySnapshot[12]&(1U<<13)) && !stream.armed);
 }
 CHECK(irqMask==0);printf("production_entry_evidence_checks=%u\n",checks);
}
