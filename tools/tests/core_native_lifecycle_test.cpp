static bool RunCommand(StreamSink& output,const char* text){return HandleCoreStreamCommand(text,strlen(text),output);}
using namespace StreamProtocol;
static unsigned starts=0,closes=0,ordinary=0,heartbeats=0,frameCursor=0;
static bool ackClose=true,canComplete=true,respondHeld=true;
static uint32_t startAnchor=0;
static uint16_t committedMotor[7]{}, finalMotor[7]{};
static Axis endpoint[7]{};
static int32_t held[7]={20,-21,22,-23,24,-25,26};
static void StatusReply(uint8_t node,uint16_t session,uint16_t sequence,uint8_t status) {
 uint8_t d[8]{};Write16(d,session);Write16(d+2,sequence);d[4]=status;d[5]=CAPACITY;d[7]=VERSION;
 CoreStreamReceive(node,STATUS,d);
}
static void ConsumeFrames() {
 while(frameCursor<frames.size()) {
  const Frame f=frames[frameCursor++];CHECK(criticalNesting==0);
  const uint16_t token=Read16(f.data),seq=Read16(f.data+2);
  CHECK(f.command>=0x20); // No ordinary MoveJ/native target CAN in executor.
  if(f.command==CAPS) {
   uint8_t d[8]{};Write16(d,token);d[2]=VERSION;d[3]=CAP_TIMED|CAP_OPEN_SNAPSHOT|CAP_STABLE_ENTRY;
   Write16(d+4,SEGMENT_TICKS);d[6]=CAPACITY;d[7]=42;CoreStreamReceive(f.node,CAPS,d);
  } else if(f.command==OPEN) {
   committedMotor[f.node-1]=finalMotor[f.node-1]=0;
   CHECK(f.data[3]==ALL_AXES);StatusReply(f.node,token,0,STATE_OPEN);
  } else if(f.command==DIAG) {
   if(respondHeld){uint8_t d[8]{};memcpy(d,f.data,4);Write32(d+4,uint32_t(held[f.node-1]));CoreStreamReceive(f.node,DIAG,d);}
  } else if(f.command>=Q && f.command<=A) {
   auto& p=endpoint[f.node-1];const int32_t value=ReadI32(f.data+4);
   if(f.command==Q)p.q=value;else if(f.command==V)p.v=value;else p.a=value;
  } else if(f.command==PREPARE || f.command==COMMIT) {
   const auto& p=endpoint[f.node-1];CHECK(Read16(f.data+4)==CrcEndpoint(token,seq,f.data[6],p.q,p.v,p.a));
   if(f.command==COMMIT){committedMotor[f.node-1]=seq;if(f.data[6]&END)finalMotor[f.node-1]=seq;}
   StatusReply(f.node,token,seq,f.command==PREPARE?STATE_PREPARED:STATE_COMMITTED);
  } else if(f.command==START) {
   CHECK(!f.node);++starts;startAnchor=nowMs+Read16(f.data+4)/20;
   for(unsigned node=1;node<=7;++node)StatusReply(node,token,1,STATE_STARTED);
  } else if(f.command==SYNC) {
   CHECK(!f.node);const uint32_t elapsed=Read32(f.data+4)/20;
   for(unsigned node=1;node<=7;++node){const auto executed=uint16_t(std::min<uint32_t>(elapsed/20,committedMotor[node-1]));
    const bool completed=canComplete && finalMotor[node-1] && executed==finalMotor[node-1];
    StatusReply(node,token,executed,completed?STATE_COMPLETED:STATE_RUNNING);}
  } else if(f.command==ABORT) {
   ++closes;CHECK(f.node==0);
   if(ackClose && !f.data[2])for(unsigned node=1;node<=7;++node)StatusReply(node,token,0,STATE_IDLE);
  } else CHECK(false);
 }
}
static void Tick(bool heartbeat=true){++nowMs;if(heartbeat && CoreNativeOwned()){CoreNativeHeartbeat(nativeEpoch);++heartbeats;}CoreStreamPoll();ConsumeFrames();CoreStreamPrint();CHECK(!criticalNesting);}
static void WaitState(uint8_t state,unsigned budget=1000){for(unsigned i=0;i<budget && CoreNativeSnapshotRead().state!=state;++i)Tick();CHECK(CoreNativeSnapshotRead().state==state);}
static uint32_t Begin(unsigned mode=2) {
 dummy.locked=false;dummy.stopState=DummyRobot::NORMAL;dummy.commandMode=DummyRobot::CommandMode(mode);canBusy=false;
 const auto epoch=dummy.GetMotionEpoch();CHECK(CoreNativeBegin(epoch,1200+epoch)==OK);
 CHECK(CoreNativeOwned() && unsigned(dummy.commandMode)==mode);
 WaitState(CoreStreamEngine::OPEN);
 for(unsigned node=1;node<=7;++node){dummy.values[node-1].angle=float(held[node-1])*360.0f/(51200.0f*dummy.values[node-1].reduction);CHECK(CoreNativeReadHeld(node,epoch)==OK);}
 for(unsigned i=0;i<40;++i)Tick();
 for(unsigned node=1;node<=7;++node){int32_t q=0;CHECK(CoreNativeHeld(node,q,epoch)&&q==held[node-1]);CHECK(CoreNativeReadHeld(node,epoch)==OK);}
 return epoch;
}
static Record R(unsigned sequence,unsigned final=6) {
 Record r{};r.session=CoreNativeSnapshotRead().session;r.sequence=uint16_t(sequence);r.flags=sequence==0?SEED:sequence==final?END:0;
 for(unsigned axis=0;axis<7;++axis)r.axes[axis]={held[axis]+int32_t(sequence),0,0};
 return r;
}
static void Fill(unsigned final,uint32_t epoch) {
 for(unsigned i=0;i<=final;++i)CHECK(CoreNativeSubmit(R(i,final),epoch)==OK);
 WaitState(CoreStreamEngine::ARMED);
 for(unsigned i=0;i<600 && CoreNativeSnapshotRead().committed<final;++i)Tick();
 CHECK(CoreNativeSnapshotRead().committed==final);
}
static void CloseAndRelease(uint32_t epoch) {
 CHECK(CoreNativeClose(epoch)==OK);for(unsigned i=0;i<40 && CoreNativeOwned();++i)Tick(false);
 CHECK(!CoreNativeOwned() && !engine.Session());CHECK(CoreNativeRelease(epoch));
}
static uint32_t validationRaceEpoch=0;
static uint16_t validationNewToken=0;
static void ReplaceOwnerDuringValidation(){
 CloseAndRelease(validationRaceEpoch);
 CHECK(Begin()==validationRaceEpoch);validationNewToken=CoreNativeSnapshotRead().session;
}
int main() {
 StreamSink output;
 for(unsigned mode: {1U,2U,3U}){
  const auto epoch=Begin(mode);CHECK(CoreNativeBegin(epoch,99)==BUSY);
  for(const char* c:{"#CARTESIAN_STATUS","#KIN_CONFIG","#TOOL","#TCPPOSE","#IKCHECK 0 0 0 0 0 0"})CHECK(CoreStreamAuxAllowed(c,strlen(c)));
  for(const char* c:{"#TOOL_SET 1 2 3","#TOOL_SAVE","#SETJACC 1 5"})CHECK(!CoreStreamAuxAllowed(c,strlen(c)));
  const auto token=engine.Session();CHECK(RunCommand(output,"!STREAM_OPEN 19 127"));CHECK(engine.Session()==token);
  CHECK(RunCommand(output,"!STREAM_START 19"));CHECK(engine.Session()==token);
  CloseAndRelease(epoch);
 }
 for(unsigned mode:{4U,7U}){dummy.commandMode=DummyRobot::CommandMode(mode);CHECK(CoreNativeBegin(dummy.GetMotionEpoch(),99)==BLOCKED);CHECK(!CoreNativeOwned());}
 // New commands may share an epoch, but retired endpoint tokens never gain
 // a fresh native owner by having their session silently overwritten.
 auto sameEpoch=Begin();const auto retired=R(0);CloseAndRelease(sameEpoch);
 CHECK(Begin()==sameEpoch);CHECK(engine.Session()!=retired.session);
 CHECK(CoreNativeSubmit(retired,sameEpoch)==BAD_SESSION);
 CHECK(CoreNativeSubmit(R(0),sameEpoch)==OK);CloseAndRelease(sameEpoch);
 // Geometry is intentionally outside IRQ masking. Replace an owner while the
 // actual validator is running and prove its lock-side token check rejects
 // the endpoint instead of publishing into a fresh same-epoch session.
 validationRaceEpoch=Begin();const auto racing=R(0);
 validationHook=ReplaceOwnerDuringValidation;
 CHECK(CoreNativeSubmit(racing,validationRaceEpoch)==BAD_SESSION);
 CHECK(validationNewToken!=racing.session && CoreNativeSnapshotRead().session==validationNewToken);
 CHECK(CoreNativeSnapshotRead().next==0);CloseAndRelease(validationRaceEpoch);
 // DIAG7 receives must bind independent high nonce, node, field and reserved.
 auto epoch=Begin();respondHeld=false;nativeHeldReady=0;
 CHECK(CoreNativeReadHeld(1,epoch)==OK);Tick();const auto nonce=nativeHeldNonce[0];CHECK(nonce&0x8000);
 uint8_t d[8]{};Write16(d,uint16_t(nonce&0x7FFF));d[2]=7;Write32(d+4,uint32_t(-12345));CoreStreamReceive(1,DIAG,d);
 int32_t q=0;CHECK(!CoreNativeHeld(1,q,epoch));Write16(d,nonce);CoreStreamReceive(2,DIAG,d);CHECK(!CoreNativeHeld(1,q,epoch));
 d[2]=6;CoreStreamReceive(1,DIAG,d);CHECK(!CoreNativeHeld(1,q,epoch));d[2]=7;d[3]=1;CoreStreamReceive(1,DIAG,d);CHECK(!CoreNativeHeld(1,q,epoch));
 d[3]=0;CoreStreamReceive(1,DIAG,d);CHECK(CoreNativeHeld(1,q,epoch)&&q==-12345);respondHeld=true;CloseAndRelease(epoch);
 // Main-command heartbeat and actual completion: elapsed END alone is not DONE.
 epoch=Begin();Fill(6,epoch);const auto beforeStart=starts;CHECK(CoreNativeStart(epoch)==OK);
 CHECK(CoreNativeSnapshotRead().state==CoreStreamEngine::ARMED);
 canComplete=false;for(unsigned i=0;i<300;++i)Tick();CHECK(starts==beforeStart+1);
 CHECK(CoreNativeSnapshotRead().state==CoreStreamEngine::DRAINING);CHECK(CoreNativeClose(epoch)==BUSY);CHECK(!CoreNativeRelease(epoch));
 canComplete=true;WaitState(CoreStreamEngine::IDLE);for(unsigned i=0;i<30 && CoreNativeOwned();++i)Tick();
 CHECK(!CoreNativeOwned());CHECK(CoreNativeRelease(epoch));
 // A full two-second nonzero curve must refill beyond both finite FIFOs.
 // No ordinary MOVEJ is substituted; each emitted endpoint remains q/v/a.
 epoch=Begin();canComplete=true;unsigned next=0;
 while(next<=100 && CoreNativeSnapshotRead().free){CHECK(CoreNativeSubmit(R(next,100),epoch)==OK);++next;}
 WaitState(CoreStreamEngine::ARMED);CHECK(CoreNativeStart(epoch)==OK);
 for(unsigned budget=0;budget<2800 && CoreNativeSnapshotRead().state!=CoreStreamEngine::IDLE;++budget){
  CHECK(CoreNativeSnapshotRead().state!=CoreStreamEngine::FAULT);
  while(next<=100 && CoreNativeSnapshotRead().free){CHECK(CoreNativeSubmit(R(next,100),epoch)==OK);++next;}
  Tick();
 }
 CHECK(next==101 && CoreNativeSnapshotRead().state==CoreStreamEngine::IDLE);
 CHECK(CoreNativeSnapshotRead().committed==100&&CoreNativeSnapshotRead().executed==100);
 CloseAndRelease(epoch);
 // A preflight CLOSE with missing acknowledgments faults safely, retaining the
 // original close snapshot and owner until the real token is released.
 epoch=Begin();ackClose=false;CHECK(CoreNativeClose(epoch)==OK);
 for(unsigned i=0;i<2100;++i)Tick(false);
 CHECK(CoreNativeOwned()&&engine.Session());CHECK(dummy.locked);
 const auto closeFault=CoreNativeSnapshotRead();CHECK(closeFault.reason==DEADLINE);
 CHECK(closeFault.failure.source==CoreStreamEngine::SOURCE_CLOSE);
 ackClose=true;for(unsigned i=0;i<60&&CoreNativeOwned();++i)Tick(false);
 CHECK(!CoreNativeOwned());CHECK(CoreNativeRelease(epoch));
 CHECK(CoreNativeSnapshotRead().failure.source==CoreStreamEngine::SOURCE_CLOSE);
 // Buffered, never-STARTed rejection closes without manufacturing global STOP.
 epoch=Begin();CHECK(CoreNativeSubmit(R(0),epoch)==OK);CHECK(CoreNativeSubmit(R(1),epoch)==OK);
 auto stopsBefore=softStops;CloseAndRelease(epoch);CHECK(softStops==stopsBefore);
 // STOP changes epoch; only original cleanup survives. No tail or re-enable.
 epoch=Begin();Fill(6,epoch);CHECK(CoreNativeStart(epoch)==OK);Tick();dummy.RequestSoftStop();
 CHECK(dummy.GetMotionEpoch()!=epoch);const auto stoppedStarts=starts;
 CHECK(CoreNativeSubmit(R(7),epoch)==BAD_SESSION);CHECK(CoreNativeStart(epoch)==BAD_SESSION);CHECK(!CoreNativeHeartbeat(epoch));CHECK(!CoreNativeHeld(1,q,epoch));
 ackClose=false;CHECK(CoreNativeClose(epoch)==OK);CHECK(!CoreNativeRelease(epoch));
 const auto lossToken=engine.Session();const auto lossStart=nowMs;
 for(unsigned i=0;i<2100;++i)Tick(false);
 CHECK(CoreNativeOwned() && engine.Session()==lossToken && dummy.locked);CHECK(starts==stoppedStarts);
 CHECK(CoreNativeBegin(dummy.GetMotionEpoch(),777)==BUSY);CHECK(nowMs-lossStart>=2100);
 ackClose=true;for(unsigned i=0;i<60 && CoreNativeOwned();++i)Tick(false);
 CHECK(!CoreNativeOwned() && !engine.Session());CHECK(CoreNativeRelease(epoch));CHECK(dummy.locked && starts==stoppedStarts);
 const auto saved=CoreNativeSnapshotRead();CHECK(saved.epoch==epoch && saved.id==1200+epoch && !saved.owned);
 // Closed result stays bound despite a later ordinary diagnostic session.
 dummy.locked=false;dummy.commandMode=DummyRobot::COMMAND_STREAM_JOINT;
 CHECK(RunCommand(output,"!STREAM_OPEN 4321 127"));CHECK(CoreNativeSnapshotRead().id==saved.id);CoreStreamCancel();
 CHECK(RunCommand(output,"!STREAM_CLOSE 4321"));Tick(false);Tick(false);CHECK(!engine.Session());
 // #STREAM queries cannot renew the internal producer's 300ms heartbeat.
 epoch=Begin();Fill(6,epoch);CHECK(CoreNativeStart(epoch)==OK);canComplete=false;
 for(unsigned i=0;i<330;++i){if(i%10==0)CHECK(RunCommand(output,"#STREAM"));Tick(false);}
 CHECK(CoreNativeSnapshotRead().state==CoreStreamEngine::FAULT && CoreNativeSnapshotRead().reason==SYNC_TIMEOUT);
 CHECK(dummy.locked);const auto failure=CoreNativeSnapshotRead().failure;
 CloseAndRelease(epoch);const auto cached=CoreNativeSnapshotRead();CHECK(cached.reason==SYNC_TIMEOUT);
 CHECK(cached.failure.source==failure.source&&cached.failure.state==failure.state&&cached.epoch==epoch);
 printf("native_lifecycle: %u checks PASS; actual adapter+engine; seven-axis directed DIAG, CRC q/v/a, three native modes, actual completion, original-epoch persistent CLOSE 2100ms loss/recovery, no auto START/heartbeat; hardware=NO\n",checks);
}
