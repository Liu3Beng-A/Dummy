struct NodeApi {
 void(*reset)(int32_t);void(*receive)(uint8_t,const uint8_t*);void(*poll)();void(*tick)();
 bool(*peek)(Frame&);void(*complete)(bool);void(*stop)();
 uint32_t(*anchor)();uint16_t(*completed)();uint8_t(*state)();uint8_t(*reason)();int32_t(*position)();bool(*locked)();
 uint32_t(*retries)();uint32_t(*drops)();
};
#define API(n) {n::Reset,n::Receive,n::Poll,n::Tick,n::Peek,n::Complete,n::Stop,n::Anchor,n::Completed,n::State,n::Reason,n::Position,n::Locked,n::Retries,n::Drops}
NodeApi nodes[7]={API(node1),API(node2),API(node3),API(node4),API(node5),API(node6),API(node7)};
static unsigned checks=0;
#define CHECK(x) do{++checks;if(!(x)){std::fprintf(stderr,"FAIL %u %s\n",__LINE__,#x);std::exit(1);}}while(0)
enum Scenario { NORMAL, LOST_START, LOST_PREPARED, LOST_COMMITTED, LOST_VALUE, HOST_PAUSE, STOP_MID, OLD_SESSION, LOST_SYNC, FAST_CLOCK, LOST_COMMIT, BUS_OFF };
static uint64_t txok=0,alst=0;
struct Port:CoreStreamPort {
 bool allowed=true,stopped=false,refuseStart=false,refuseSync=false,refusedStart=false,refusedSync=false;
 bool Send(uint8_t node,uint8_t command,const uint8_t data[8])override{
  if(node==7&&command==StreamProtocol::START&&refuseStart&&!refusedStart){refusedStart=true;return false;}
  if(node==7&&command==StreamProtocol::SYNC&&refuseSync&&!refusedSync){refusedSync=true;return false;}
  return corehw::Send(node,command,data);
 }
 bool Allowed()const override{return allowed;}
 bool Validate(const StreamProtocol::Record&,const StreamProtocol::Record*,uint8_t)const override{return true;}
 void Stop()override{stopped=true;allowed=false;corehw::dummy.locked=true;++corehw::dummy.epoch;for(auto&n:nodes)n.stop();}
};
static bool dropped=false;
static unsigned scenario=0;
static bool probeCheck=false;static unsigned probeNode=4,probeLoss=0;
static uint8_t isolationMask=0;static bool holdDropStart=false;
void Wire(uint32_t startTick){
 using namespace StreamProtocol;
 if(simTick%3)return; // 150us classical-CAN frame slots, including bounded stuffing approximation.
 if(scenario==BUS_OFF&&startTick&&simTick>startTick+20000)return;
 std::vector<Frame> candidates;Frame f;
 if(corehw::Peek(f))candidates.push_back(f);
 for(auto&n:nodes)if(n.peek(f))candidates.push_back(f);
 if(candidates.empty())return;
 auto winner=std::min_element(candidates.begin(),candidates.end(),[](const Frame&a,const Frame&b){return a.id<b.id;});
 Frame delivered=*winner;
 // NART really completes every losing mailbox with failure. Production queues
 // decide whether/when to retry; the wire never retains a lost frame for them.
 for(auto&x:candidates)if(x.owner!=delivered.owner){++alst;if(x.owner)nodes[x.owner-1].complete(false);else corehw::Complete(x.slot,false);}
 const uint8_t node=uint8_t(delivered.id>>7),cmd=uint8_t(delivered.id&127);const uint16_t seq=Read16(delivered.bytes+2);
 if(probeCheck && !delivered.owner) {CHECK(node==probeNode);CHECK(cmd==CAPS||cmd==OPEN||cmd==ABORT);}
 if(isolationMask && !delivered.owner) {
  if(!node)CHECK(isolationMask==ALL_AXES&&(cmd==START||cmd==SYNC||cmd==ABORT));
  else CHECK(node<=7&&(isolationMask&(1U<<(node-1))));
  CHECK(cmd>=OPEN&&cmd<=CAPS);
  if(cmd==OPEN)CHECK(delivered.bytes[3]==(isolationMask==ALL_AXES?ALL_AXES:(1U<<(node-1))));
 }

 bool drop=false;
 if(isolationMask && holdDropStart && !dropped && !delivered.owner && node==7 && cmd==START)drop=dropped=true;
 if(probeCheck && delivered.owner && cmd==STATUS) {
  if(probeLoss==1 && !dropped && delivered.bytes[4]==STATE_OPEN)drop=dropped=true;
  if(probeLoss==2 && delivered.bytes[4]==STATE_IDLE)drop=true;
 }
 if(!dropped&&scenario==LOST_PREPARED&&delivered.owner==2&&cmd==STATUS&&seq==20&&delivered.bytes[4]==STATE_PREPARED)drop=dropped=true;
 if(!dropped&&scenario==LOST_COMMITTED&&delivered.owner==6&&cmd==STATUS&&seq==20&&delivered.bytes[4]==STATE_COMMITTED)drop=dropped=true;
 if(!dropped&&scenario==LOST_VALUE&&!delivered.owner&&node==3&&cmd==V&&seq==20)drop=dropped=true;
 if(!dropped&&scenario==LOST_COMMIT&&!delivered.owner&&node==6&&cmd==COMMIT&&seq==20)drop=dropped=true;
 if(scenario==LOST_SYNC&&startTick&&simTick>startTick+20000&&cmd==SYNC&&!delivered.owner)drop=true;
 if(!drop){
  if(delivered.owner)engine->Receive(node,cmd,delivered.bytes,simTick/20);
  else for(unsigned n=0;n<7;++n)if(!node||node==n+1){if(!dropped&&scenario==LOST_START&&cmd==START&&n==3){dropped=true;continue;}nodes[n].receive(cmd,delivered.bytes);}
 }
 ++txok;if(delivered.owner)nodes[delivered.owner-1].complete(true);else corehw::Complete(delivered.slot,true);
}
void simulate(const std::vector<StreamProtocol::Record>&records,unsigned selected,unsigned mainPeriod){
 using namespace StreamProtocol;
 scenario=selected;simTick=0;txok=alst=0;dropped=false;corehw::Reset();
 for(unsigned n=0;n<7;++n)nodes[n].reset(records.front().axes[n].q);
 Port port;CoreStreamEngine instance(port);engine=&instance;CHECK(engine->Begin(records.front().session,127,0)==OK);
 size_t next=0;uint32_t startTick=0,maxSkew=0;bool old=false,stop=false;
 for(simTick=0;simTick<records.size()*400+100000;++simTick){
  const uint32_t ms=simTick/20;corehw::nowMs=ms;
  for(auto&n:nodes)n.tick();
  if(scenario==FAST_CLOCK&&simTick%1000==0)nodes[6].tick();
  if(simTick%20==0){
   bool paused=scenario==HOST_PAUSE&&startTick&&simTick>startTick+18000;
   if(!paused&&ms%50==0)engine->Heartbeat(ms);
   auto s=engine->GetState();
   if(!paused&&next<records.size()&&(s==CoreStreamEngine::OPEN||s==CoreStreamEngine::BUFFERING||s==CoreStreamEngine::ARMED||s==CoreStreamEngine::RUNNING))
    for(unsigned b=0;b<4&&next<records.size()&&engine->Free();++b)CHECK(engine->Submit(records[next++],ms)==OK);
   if(s==CoreStreamEngine::ARMED&&!startTick){CHECK(engine->Start(records.front().session,ms)==OK);startTick=simTick;}
   if(scenario==STOP_MID&&startTick&&!stop&&simTick>startTick+20000){engine->ProtocolFault(ABORTED);stop=true;}
   engine->Poll(ms);corehw::Poll();
   if(ms%5==0)corehw::Feedback(1+(ms/5)%7);
  }
  Wire(startTick);
  for(unsigned n=0;n<7;++n)if((simTick+n)%mainPeriod==0)nodes[n].poll();
  if(scenario==OLD_SESSION&&startTick&&!old&&simTick>startTick+20000){uint8_t b[8]{};Write16(b,uint16_t(records.front().session-1));for(auto&n:nodes){n.receive(Q,b);n.receive(START,b);n.receive(ABORT,b);}old=true;}
  if(startTick){uint32_t low=UINT32_MAX,high=0;for(auto&n:nodes)if(n.anchor()){low=std::min(low,n.anchor());high=std::max(high,n.anchor());}if(low!=UINT32_MAX)maxSkew=std::max(maxSkew,high-low);}
  const bool expectFault=scenario==LOST_VALUE||scenario==HOST_PAUSE||scenario==STOP_MID||scenario==LOST_SYNC||scenario==FAST_CLOCK||scenario==BUS_OFF;
  if(engine->GetState()==CoreStreamEngine::FAULT){
   engine->Poll(ms+1);corehw::Poll();auto f=engine->Failure();
   std::printf("case=%u main=%u FAULT reason=%u source=%u node=%u cmd=%u seq=%u age=%u at=%.2fms commit=%u TXOK=%llu ALST=%llu\n",scenario,mainPeriod,engine->Reason(),f.source,f.node,f.command,f.sequence,f.age,simTick/20.,engine->LastCommitted(),txok,alst);
   for(unsigned n=0;n<7;++n)std::printf(" node%u state=%u reason=%u retries=%u drops=%u\n",n+1,nodes[n].state(),nodes[n].reason(),nodes[n].retries(),nodes[n].drops());
   CHECK(expectFault);CHECK(port.stopped);for(auto&n:nodes)CHECK(n.locked());
   return;
  }
  if(startTick&&engine->GetState()==CoreStreamEngine::IDLE){
   CHECK(!expectFault&&next==records.size()&&!port.stopped);CHECK(engine->LastExecuted()==records.back().sequence);
   for(unsigned n=0;n<7;++n){CHECK(nodes[n].completed()==records.back().sequence);CHECK(nodes[n].position()==records.back().axes[n].q);}
   CHECK(maxSkew<=40);
   std::printf("case=%u main=%u DONE at=%.2fms TXOK=%llu ALST=%llu skew=%u\n",scenario,mainPeriod,simTick/20.,txok,alst,maxSkew);
   CHECK(engine->Close(records.front().session,ms)==OK);
   const auto end=simTick+30000;
   for(++simTick;simTick<end&&engine->Session();++simTick){corehw::nowMs=simTick/20;for(auto&n:nodes)n.tick();if(simTick%20==0){engine->Poll(simTick/20);corehw::Poll();}Wire(startTick);for(auto&n:nodes)n.poll();}
   CHECK(engine->Session()==0);for(auto&n:nodes)CHECK(!n.locked());std::printf(" CLOSE confirmed all7\n");return;
  }
 }
 CHECK(false);
}
void probe(unsigned target,bool blocked,unsigned loss) {
 using namespace StreamProtocol;
 scenario=NORMAL;simTick=0;txok=alst=0;dropped=false;corehw::Reset();
 for(auto&n:nodes)n.reset(1234);
 probeCheck=true;probeNode=target;probeLoss=loss;
 Port port;CoreStreamEngine instance(port);engine=&instance;CHECK(engine->Begin(555,uint8_t(1U<<(target-1)),0,true)==OK);
 for(simTick=0;simTick<100000;++simTick) {
  corehw::nowMs=simTick/20;for(auto&n:nodes)n.tick();
  if(blocked)node4::controller.InvalidatePositionEntry();
  if(simTick%20==0){engine->Poll(simTick/20);corehw::Poll();}
  Wire(0);for(auto&n:nodes)n.poll();
  if(!engine->Active()||engine->TestCloseFailed())break;
 }
 CHECK(simTick<100000);CHECK(!port.stopped);CHECK(engine->TestOpened()==!blocked);
 CHECK(engine->TestCloseFailed()==(loss==2));CHECK(engine->Active()==(loss==2));
 CHECK(engine->Reason()==(blocked?BLOCKED:loss==2?DEADLINE:OK));
 for(auto&n:nodes){CHECK(!n.locked());CHECK(n.anchor()==0);CHECK(n.position()==1234);CHECK(n.completed()==0);}
 if(blocked){CHECK(node4::entrySnapshot[0]==555);CHECK(node4::entrySnapshot[1]==(1U<<12));CHECK(node4::entrySnapshot[3]==0);}
 std::printf("OPEN_ONLY node=%u blocked=%u loss=%u opened=%u closed=%u reason=%u TXOK=%llu no_motion=PASS\n",target,blocked,loss,engine->TestOpened(),!engine->Active(),engine->Reason(),txok);
 probeCheck=false;
}
// Stage a ready, unarmed session on an unselected motor using the actual
// production CAN parser. Its token intentionally collides with the test group.
// Addressed controls must leave that complete dormant session untouched.
static void dormant(unsigned node,uint16_t token,int32_t q) {
 using namespace StreamProtocol;
 auto send=[&](uint8_t cmd,const uint8_t*bytes) {
  nodes[node-1].receive(cmd,bytes);nodes[node-1].poll();
  Frame f;while(nodes[node-1].peek(f)) {nodes[node-1].complete(true);nodes[node-1].poll();}
 };
 uint8_t b[8]{};Write16(b,token);b[2]=VERSION;b[3]=uint8_t(1U<<(node-1));send(OPEN,b);
 for(unsigned sequence=0;sequence<2;++sequence) {
  const uint8_t flag=sequence?END:SEED;
  for(unsigned part=0;part<3;++part) {
   memset(b,0,8);Write16(b,token);Write16(b+2,uint16_t(sequence));
   Write32(b+4,part?0:uint32_t(q));send(uint8_t(Q+part),b);
  }
  memset(b,0,8);Write16(b,token);Write16(b+2,uint16_t(sequence));
  Write16(b+4,CrcEndpoint(token,uint16_t(sequence),flag,q,0,0));b[6]=flag;
  send(PREPARE,b);send(COMMIT,b);
 }
 CHECK(nodes[node-1].state()==STATE_OPEN);CHECK(nodes[node-1].anchor()==0);
 CHECK(nodes[node-1].completed()==0);CHECK(nodes[node-1].reason()==OK);
}
void hold(unsigned mask,bool lostStart,unsigned mainPeriod,unsigned admission=0) {
 using namespace StreamProtocol;
 scenario=NORMAL;simTick=0;txok=alst=0;dropped=false;corehw::Reset();
 for(auto&n:nodes)n.reset(1234);
 unsigned sentinel=0;
 for(unsigned node=1;node<=7;++node)if(!(mask&(1U<<(node-1)))){sentinel=node;break;}
 constexpr uint16_t token=819;
 if(sentinel)dormant(sentinel,token,1234);
 std::vector<Record> records;
 for(unsigned seq=0;seq<=100;++seq) {
  Record r{};r.session=token;r.sequence=uint16_t(seq);r.flags=seq==0?SEED:seq==100?END:0;
  for(auto&a:r.axes)a={1234,0,0};records.push_back(r);
 }
 isolationMask=uint8_t(mask);holdDropStart=lostStart;
 Port port;port.refuseStart=admission==1;port.refuseSync=admission==2;CoreStreamEngine instance(port);engine=&instance;CHECK(engine->Begin(token,uint8_t(mask),0)==OK);
 size_t next=0;uint32_t startTick=0,maxSkew=0;
 for(simTick=0;simTick<150000;++simTick) {
  const uint32_t ms=simTick/20;corehw::nowMs=ms;for(auto&n:nodes)n.tick();
  if(simTick%20==0) {
   if(ms%50==0)engine->Heartbeat(ms);
   const auto state=engine->GetState();
   if(next<records.size()&&(state==CoreStreamEngine::OPEN||state==CoreStreamEngine::BUFFERING||state==CoreStreamEngine::ARMED||state==CoreStreamEngine::RUNNING))
    for(unsigned b=0;b<4&&next<records.size()&&engine->Free();++b)CHECK(engine->Submit(records[next++],ms)==OK);
   if(state==CoreStreamEngine::ARMED&&!startTick){CHECK(engine->Start(token,ms)==OK);startTick=simTick;}
   engine->Poll(ms);corehw::Poll();
  }
  Wire(startTick);for(unsigned n=0;n<7;++n)if((simTick+n)%mainPeriod==0)nodes[n].poll();
  if(startTick) {
   uint32_t low=UINT32_MAX,high=0;
   for(unsigned n=0;n<7;++n)if((mask&(1U<<n))&&nodes[n].anchor()){low=std::min(low,nodes[n].anchor());high=std::max(high,nodes[n].anchor());}
   if(low!=UINT32_MAX)maxSkew=std::max(maxSkew,high-low);
  }
  if(engine->GetState()==CoreStreamEngine::FAULT) {
   const auto f=engine->Failure();std::printf("HOLD mask=%u FAULT reason=%u source=%u node=%u cmd=%u seq=%u age=%u at=%u\n",mask,engine->Reason(),f.source,f.node,f.command,f.sequence,f.age,ms);
   CHECK(false);
  }
  if(sentinel){CHECK(nodes[sentinel-1].anchor()==0);CHECK(nodes[sentinel-1].state()==STATE_OPEN);}
  if(startTick&&engine->GetState()==CoreStreamEngine::IDLE) {
   CHECK(next==records.size()&&!port.stopped);CHECK(engine->LastExecuted()==100);
   for(unsigned n=0;n<7;++n) {
    CHECK(nodes[n].position()==1234&&!nodes[n].locked());
    CHECK(nodes[n].completed()==((mask&(1U<<n))?100:0));
   }
   CHECK(maxSkew<=40);if(lostStart)CHECK(dropped);
   if(admission==1)CHECK(port.refusedStart);if(admission==2)CHECK(port.refusedSync);
   CHECK(engine->Close(token,ms)==OK);
   const auto end=simTick+30000;
   for(++simTick;simTick<end&&engine->Session();++simTick) {
    corehw::nowMs=simTick/20;for(auto&n:nodes)n.tick();
    if(simTick%20==0){engine->Poll(simTick/20);corehw::Poll();}
    Wire(startTick);for(auto&n:nodes)n.poll();
   }
   CHECK(engine->Session()==0&&!port.stopped);
   if(sentinel)CHECK(nodes[sentinel-1].state()==STATE_OPEN&&nodes[sentinel-1].anchor()==0);
   for(unsigned n=0;n<7;++n)if(mask&(1U<<n))CHECK(nodes[n].state()==STATE_IDLE);
   std::printf("HOLD mask=%u main=%u lostStart=%u reject=%u DONE/CLOSE TXOK=%llu ALST=%llu skew=%u constant_goal=PASS unselected_same_token=%s\n",mask,mainPeriod,lostStart,admission,txok,alst,maxSkew,sentinel?"PASS":"NA");
   isolationMask=0;holdDropStart=false;return;
  }
 }
 CHECK(false);
}
int main(int argc,char**argv){
 CHECK(argc==2);std::ifstream file(argv[1]);CHECK(bool(file));std::vector<StreamProtocol::Record> records;std::string line;
 while(std::getline(file,line)){if(!line.empty()&&line.back()=='\r')line.pop_back();if(line.empty())continue;StreamProtocol::Record r{};CHECK(StreamProtocol::DecodeLine(line.data(),line.size(),r));records.push_back(r);}
 for(unsigned s=0;s<=BUS_OFF;++s)simulate(records,s,1);
 simulate(records,NORMAL,10);simulate(records,NORMAL,20);
 for(unsigned target:{1U,4U,7U})probe(target,false,0);
 probe(4,true,0);probe(4,false,1);probe(4,false,2);
 for(unsigned mask:{1U,2U,4U,8U,16U,32U,64U,65U,127U})hold(mask,false,1);
 hold(65,true,1);hold(65,false,20);hold(65,false,1,1);hold(65,false,1,2);
 std::printf("PASS actual CAN queues + core +7 motor protocol/curve adapters, checks=%u\n",checks);
}
