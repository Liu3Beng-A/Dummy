#include "motor_stream_test_shim.h"
#include <cstring>
#include <cstdio>
#include <cstdlib>
std::vector<ReplyFrame> replies;
BoardConfig boardConfig;
uint32_t irqMask;
extern "C" uint32_t _estack=0;
bool CAN_TrySend(CAN_TxHeaderTypeDef*h,const uint8_t*b){ReplyFrame r{h->StdId,{}};memcpy(r.bytes,b,8);replies.push_back(r);return true;}
// Transport reliability is separately covered by actual can.c tests.
bool CAN_QueueStreamReply(CAN_TxHeaderTypeDef*h,const uint8_t*b){return CAN_TrySend(h,b);}
void CAN_CancelStreamReplies(){}
bool CAN_StreamReplyFault(){return false;}
uint32_t CAN_StreamReplyDiagnostic(uint8_t){return 0;}
// Production protocol/controller implementation; only the HAL/Motor surface is stubbed.
#include "stream_controller_under_test.cpp"
static unsigned checks=0;
#define CHECK(x) do {++checks;if(!(x)){fprintf(stderr,"FAIL line%u %s\n",__LINE__,#x);exit(1);}}while(0)
Motor device;Motor::Controller controller;
void certify(){controller.modeRunning=controller.requestMode;controller.ResetPositionCompletion();for(unsigned i=0;i<1000;++i){controller.TickStreamClock();controller.TickPositionCompletion();}}
void reset(){device=Motor{};calibrationBusy=false;stream=MotorStream::Runtime{};packetRead=packetWrite=0;packetOverflow=false;stopDispatching=false;memset(entrySnapshot,0,sizeof(entrySnapshot));irqReleaseHook=nullptr;generation=0;controller=Motor::Controller{};controller.context=&device;replies.clear();irqMask=0;certify();}
void command(uint8_t cmd,uint16_t ss,uint16_t seq=0,uint32_t value=0){uint8_t b[8]{};Write16(b,ss);Write16(b+2,seq);Write32(b+4,value);CHECK(controller.ProcessStreamCan(cmd,b,8));controller.PollStreamMain();}
void open(){certify();command(OPEN,7,uint16_t(VERSION)|uint16_t(ALL_AXES)<<8);CHECK(stream.state==STATE_OPEN && stream.session==7);}
uint16_t endpoint(uint16_t seq,MotorStream::Point p,uint8_t flags=0,bool commit=true){
 command(Q,7,seq,uint32_t(p.q));command(V,7,seq,uint32_t(p.v));command(A,7,seq,uint32_t(p.a));
 uint16_t crc=CrcEndpoint(7,seq,flags,p.q,p.v,p.a);
 command(PREPARE,7,seq,uint32_t(crc)|uint32_t(flags)<<16);
 CHECK(replies.back().bytes[4]==STATE_PREPARED || replies.back().bytes[4]==STATE_OPEN);
 if(commit)command(COMMIT,7,seq,uint32_t(crc)|uint32_t(flags)<<16);
 return crc;
}
void ticks(unsigned n){for(unsigned i=0;i<n;++i){controller.TickStreamClock();controller.modeRunning=controller.requestMode;if(controller.modeRunning==Motor::MODE_STREAM_JOINT)controller.TickStream();controller.TickPositionCompletion();}}
int main(){
 reset();command(OPEN,17,uint16_t(VERSION+1)|uint16_t(ALL_AXES)<<8);CHECK(replies.back().bytes[6]==BAD_VERSION && Read16(replies.back().bytes)==17);
 reset();command(CAPS,123);CHECK(replies.back().bytes[3]==(CAP_TIMED|CAP_OPEN_SNAPSHOT|CAP_STABLE_ENTRY) && Read16(replies.back().bytes)==123);
 open();endpoint(0,{0,0,0},SEED);CHECK(stream.seedCommitted);
 const auto crc=endpoint(1,{10,0,0},END);CHECK(stream.committed==1);
 command(COMMIT,7,1,uint32_t(crc)|(uint32_t(END)<<16));CHECK(stream.committed==1 && stream.Free()==7);
 command(START,7,1,2000);const auto firstAnchor=stream.ticks+2000;CHECK(stream.armed && stream.anchor==firstAnchor);
 ticks(100);command(START,7,1,1900);CHECK(stream.anchor==firstAnchor);
 ticks(1900);CHECK(stream.completed==0 && controller.softPosition==0);
 command(SYNC,7,1,0);ticks(400);CHECK(stream.completed==1 && stream.terminal && controller.goalPosition==10);
 ticks(1000);controller.PollStreamMain();CHECK(stream.state==STATE_COMPLETED && !controller.locked);
 // Reopen a new session cannot reuse previous seeded data.
 command(OPEN,8,uint16_t(VERSION)|uint16_t(ALL_AXES)<<8);CHECK(stream.session==8 && !stream.seeded);
 auto repliesBefore=replies.size();command(Q,7,1,0);command(START,7,1,2000);command(ABORT,7,0,0);command(OPEN,7,uint16_t(VERSION)|uint16_t(ALL_AXES)<<8);
 CHECK(stream.session==8 && stream.state==STATE_OPEN && !controller.locked && replies.size()==repliesBefore);
 command(ABORT,8,0,0);CHECK(stream.state==STATE_IDLE && !controller.locked);
 reset();open();endpoint(0,{0,0,0},SEED);endpoint(1,{0,0,0},END);
 command(START,7,1,1900);CHECK(stream.anchor==stream.ticks+1900);ticks(1800);
 command(SYNC,7,1,0);CHECK(stream.reason==OK); // pre-start elapsed zero accepted
 ticks(120);command(SYNC,7,2,500);CHECK(stream.reason==SYNC_DRIFT);controller.PollStreamMain();ticks(1);CHECK(controller.locked);
 reset();open();endpoint(0,{0,0,0},SEED);
 command(Q,7,1,0);command(V,7,1,0);command(A,7,1,0);command(PREPARE,7,1,123);
 CHECK(stream.state==STATE_FAULT && stream.reason==BAD_CRC && controller.locked);
 reset();open();endpoint(0,{0,0,0},SEED);endpoint(1,{0,0,0},END);
 controller.CancelStream();command(START,7,1,2000);CHECK(!stream.armed); // cancelled START cannot revive
 reset();open();endpoint(0,{0,0,0},SEED);
 command(START,7,1,2000);CHECK(stream.reason==INCOMPLETE && !stream.armed);
 reset();open();endpoint(0,{0,0,0},SEED);endpoint(1,{0,0,0},END);
 command(START,7,1,2000);ticks(2001);boardConfig.velocityAcc=1000000;ticks(1);CHECK(controller.locked && stream.reason==BLOCKED);boardConfig.velocityAcc=1536000;
 reset();uint8_t raw[8]{};Write16(raw,7);for(unsigned i=0;i<25;++i)controller.ProcessStreamCan(Q,raw,8);CHECK(packetOverflow);controller.CancelStream();CHECK(packetRead==packetWrite && !packetOverflow);
 reset();open();endpoint(0,{0,0,0},SEED);command(Q,7,1,0);command(PREPARE,7,1,0);CHECK(stream.reason==INCOMPLETE && controller.locked);
 reset();open();endpoint(0,{0,0,0},SEED);endpoint(1,{0,0,0},END);command(START,7,1,2000);command(ABORT,7,0,0);ticks(1);CHECK(controller.locked && stream.reason==ABORTED);
 reset();controller.goalDisable=true;command(OPEN,7,uint16_t(VERSION)|uint16_t(ALL_AXES)<<8);CHECK(!stream.armed && stream.state==STATE_IDLE && replies.back().bytes[6]==BLOCKED && Read16(replies.back().bytes)==7);
 reset();boardConfig.canNodeId=8;command(CAPS,44);CHECK(replies.back().bytes[3]==0);command(OPEN,7,uint16_t(VERSION)|uint16_t(ALL_AXES)<<8);CHECK(!stream.armed && stream.state==STATE_IDLE);boardConfig.canNodeId=1;
 reset();command(DIAG,99,0,0);CHECK((replies.back().id&127)==DIAG && Read16(replies.back().bytes)==99);
 command(DIAG,100,5,0);CHECK(Read32(replies.back().bytes+4)==uint32_t(boardConfig.velocityLimit));command(DIAG,101,6,0);CHECK(Read32(replies.back().bytes+4)==uint32_t(boardConfig.velocityAcc));
 reset();controller.goalPosition=10000;command(OPEN,7,uint16_t(VERSION)|uint16_t(ALL_AXES)<<8);CHECK(!stream.armed && stream.state==STATE_IDLE && replies.back().bytes[6]==BLOCKED && Read16(replies.back().bytes)==7);
 reset();open();endpoint(0,{0,0,0},SEED);endpoint(1,{0,0,0},END);command(START,7,1,2000);ticks(2001);command(SYNC,7,1,0x80000000U);CHECK(stream.reason==SYNC_DRIFT && stream.phaseError<0);
 reset();open();command(ABORT,7,0,0);command(OPEN,7,uint16_t(VERSION)|uint16_t(ALL_AXES)<<8);CHECK(stream.state==STATE_IDLE && !stream.armed && replies.back().bytes[6]==BAD_SESSION);
 command(OPEN,8,uint16_t(VERSION)|uint16_t(ALL_AXES)<<8);command(ABORT,8,0,0);command(OPEN,7,uint16_t(VERSION)|uint16_t(ALL_AXES)<<8);CHECK(stream.state==STATE_IDLE && !stream.armed && stream.session==8);
 reset();open();controller.CancelStream();controller.locked=false;command(OPEN,7,uint16_t(VERSION)|uint16_t(ALL_AXES)<<8);CHECK(!controller.StreamActive() && !stream.armed);command(OPEN,8,uint16_t(VERSION)|uint16_t(ALL_AXES)<<8);CHECK(stream.session==8 && stream.state==STATE_OPEN);
 reset();open();auto seedCrc=endpoint(0,{0,0,0},SEED);auto firstCrc=endpoint(1,{10,0,0});
 command(Q,7,2,20);command(Q,7,1,10);command(V,7,2,0);command(PREPARE,7,1,firstCrc);command(A,7,2,0);
 auto secondCrc=CrcEndpoint(7,2,END,20,0,0);command(PREPARE,7,2,uint32_t(secondCrc)|(uint32_t(END)<<16));CHECK(stream.prepared==2 && !controller.locked);
 command(COMMIT,7,1,firstCrc);command(PREPARE,7,0,uint32_t(seedCrc)|(uint32_t(SEED)<<16));CHECK(stream.prepared==2 && !controller.locked && stream.reason==OK);
 command(COMMIT,7,2,uint32_t(secondCrc)|(uint32_t(END)<<16));CHECK(stream.committed==2 && !controller.locked);
 reset();open();endpoint(0,{0,0,0},SEED);endpoint(1,{0,0,0},END);command(START,7,1,2000);ticks(2001);
 stream.Fault(SYNC_DRIFT);CHECK(controller.StreamActive() && stream.stopRequired && !controller.locked);
 command(ABORT,7,0,0);CHECK(controller.locked && !stream.stopRequired && stream.state==STATE_FAULT);
 reset();open();stream.Fault(BAD_CRC);controller.CancelStream(OK);CHECK(stream.stopRequired && controller.StreamActive());
 controller.PollStreamMain();CHECK(controller.locked && !stream.stopRequired);
 reset();open();stream.Fault(BAD_CRC);irqReleaseHook=[] {ServiceStop(controller);};ServiceStop(controller);
 CHECK(controller.locked && controller.stopCalls==1 && !stream.stopRequired && !stopDispatching);
 // A settled native error does not reject OPEN; v2 preserves the held target.
 for(unsigned node: {1U,7U}) {
  reset();boardConfig.canNodeId=node;controller.goalPosition=controller.softPosition=node==1?600:800;
  CHECK(controller.goalPosition<=MotionCompletionProtocol::PositionToleranceSteps(node));
  open();CHECK(stream.position==controller.goalPosition);
  endpoint(0,{controller.goalPosition,0,0},SEED);
  CHECK(stream.seedCommitted && controller.softPosition==controller.goalPosition);
 }
 boardConfig.canNodeId=1;
 reset();controller.goalPosition=controller.softPosition=600;open();
 command(Q,7,0,0);command(V,7,0,0);command(A,7,0,0);
 command(PREPARE,7,0,uint32_t(CrcEndpoint(7,0,SEED,0,0,0))|(uint32_t(SEED)<<16));
 CHECK(stream.reason==BAD_SEED && controller.goalPosition==600);
 // CAN receipt timestamps, not delayed main-loop time, own the start/sync clock.
 reset();open();endpoint(0,{0,0,0},SEED);endpoint(1,{0,0,0},END);
 uint8_t delayedStart[8]{};Write16(delayedStart,7);Write16(delayedStart+2,1);Write16(delayedStart+4,START_DELAY_TICKS);
 controller.ProcessStreamCan(START,delayedStart,8);ticks(START_DELAY_TICKS-START_GUARD_TICKS);
 controller.PollStreamMain();CHECK(!stream.armed && stream.reason==DEADLINE && controller.locked);
 reset();open();endpoint(0,{0,0,0},SEED);endpoint(1,{0,0,0},END);command(START,7,1,START_DELAY_TICKS);
 uint8_t delayedSync[8]{};Write16(delayedSync,7);Write16(delayedSync+2,1);
 controller.ProcessStreamCan(SYNC,delayedSync,8);const uint32_t received=stream.ticks;ticks(500);
 controller.PollStreamMain();CHECK(stream.lastSyncTick==received && stream.ticks==received+500);
 // Every OPEN decision bit is read from the same atomic snapshot.
 for(unsigned bit=0;bit<19;++bit) {
  reset();
  switch(bit) {
   case 0:stream.state=STATE_OPEN;break;case 1:stream.stopRequired=true;break;
   case 2:controller.locked=true;break;case 3:controller.feedback=false;break;
   case 4:controller.goalDisable=true;break;case 5:controller.softDisable=true;break;
   case 6:controller.goalBrake=true;break;case 7:controller.softBrake=true;break;
   case 8:controller.isStalled=true;break;case 9:controller.calibrationTask=true;break;
   case 10:calibrationBusy=true;break;case 11:controller.protectionPhase=1;break;
   case 12:stream.ticks+=1001;break;case 13:controller.softVelocity=1;break;
   case 14:controller.requestMode=Motor::MODE_STREAM_JOINT;break;
   case 15:controller.softPosition=1;break;case 16:controller.realPosition=10000;break;
   case 17:controller.goalPosition=INT32_MAX;device.config.motionParams.encoderHomeOffset=-1;break;
   case 18:controller.modeRunning=controller.requestMode=Motor::Mode(2);break;
  }
  CHECK(controller.CaptureStreamOpen(55)&(1U<<bit));CHECK(entrySnapshot[0]==55);
 }
 // Fresh completed-window evidence admits diagnostic instantaneous noise.
 for(int32_t speed:{-513,-512,512,513}) {
  reset();controller.estVelocity=speed;controller.positionSettled=true;
  const auto mode=controller.modeRunning;const auto goal=controller.goalPosition;
  command(OPEN,55,uint16_t(VERSION)|uint16_t(1)<<8);
  CHECK(entrySnapshot[1]==0);
  CHECK(int32_t(entrySnapshot[3])==speed && entrySnapshot[10]==1);
  CHECK(!stream.armed&&!stream.seeded&&controller.goalPosition==goal&&controller.modeRunning==mode);
  // A changed later velocity does not overwrite the first decision.
  controller.estVelocity=0;command(OPEN,55,uint16_t(VERSION)|uint16_t(1)<<8);
  CHECK(int32_t(entrySnapshot[3])==speed);
  command(ABORT,55);CHECK(replies.back().bytes[4]==STATE_IDLE&&Read16(replies.back().bytes)==55);
  CHECK(!controller.StreamActive()&&!controller.locked&&controller.stopCalls==0);
  for(unsigned field=23;field<=33;++field) {
   command(DIAG,123,uint16_t(field));CHECK(Read32(replies.back().bytes+4)==entrySnapshot[field-23]);
  }
  CHECK(entrySnapshot[0]==55);CHECK(controller.goalPosition==goal);
 }
 CHECK(irqMask==0);printf("protocol_controller_checks=%u\n",checks);
}
