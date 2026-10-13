#include <cstdint>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include <deque>
#include <algorithm>
#include <fstream>
#include "core_stream_engine.h"
#include "stream_runtime.h"
#include "motion_completion_protocol.h"
struct BusFrame {uint32_t id;uint8_t data[8];bool core;};
std::vector<BusFrame> bus;
static bool motorSend(uint32_t id,const uint8_t* p){BusFrame f{id,{},false};std::memcpy(f.data,p,8);bus.push_back(f);return true;}
static void motorCancel(unsigned node){bus.erase(std::remove_if(bus.begin(),bus.end(),[node](const BusFrame&f){return !f.core&&(f.id>>7)==node;}),bus.end());}
// This legacy bus retains arbitration losers and has no finite transport queue.
// Generated namespaces contain seven independent copies of the PRODUCTION
// stream_controller.cpp. Only the Motor/HAL boundary is replaced by a stub.
#include "seven_motor_nodes.inc"
struct NodeApi {
 void(*reset)(int32_t);void(*receive)(uint8_t,const uint8_t*);void(*poll)();void(*tick)();
 uint32_t(*anchor)();uint16_t(*completed)();uint8_t(*state)();uint8_t(*reason)();int32_t(*q)();bool(*locked)();void(*stop)();
};
#define API(n) {n::Reset,n::Receive,n::Poll,n::Tick,n::Anchor,n::Completed,n::State,n::Reason,n::Position,n::Locked,n::Stop}
NodeApi nodes[7]={API(node1),API(node2),API(node3),API(node4),API(node5),API(node6),API(node7)};
static unsigned checks=0;
#define CHECK(x) do{++checks;if(!(x)){std::fprintf(stderr,"FAIL line%d %s\n",__LINE__,#x);std::exit(1);}}while(0)
enum Scenario { NORMAL, LOST_START, LOST_READY, LOST_COMMIT_ACK, LOST_VALUE, HOST_PAUSE, STOP_MID, OLD_SESSION, LOST_SYNC, FAST_CLOCK, BUS_BACKGROUND, LOST_COMMIT };
struct Port:CoreStreamPort {
 bool allowed=true,stopped=false;unsigned sent=0;
 bool Send(uint8_t node,uint8_t command,const uint8_t data[8])override{
  unsigned queued=0;for(const auto& f:bus)queued+=f.core;
  if(queued>=3)return false;
  BusFrame f{(uint32_t(node)<<7)|command,{},true};std::memcpy(f.data,data,8);bus.push_back(f);++sent;return true;
 }
 bool Allowed()const override{return allowed;}
 bool Validate(const StreamProtocol::Record&,const StreamProtocol::Record*,uint8_t)const override{return true;}
 void Stop()override{stopped=true;allowed=false;for(auto& node:nodes)node.stop();bus.erase(std::remove_if(bus.begin(),bus.end(),[](const BusFrame&f){return f.core;}),bus.end());}
};
void simulate(const std::vector<StreamProtocol::Record>& records,Scenario scenario){
 using namespace StreamProtocol;
 bus.clear();for(unsigned i=0;i<7;++i)nodes[i].reset(records[0].axes[i].q);
 Port port;CoreStreamEngine core(port);CHECK(core.Begin(records[0].session,127,0)==OK);
 size_t next=0;bool started=false,dropped=false,oldInjected=false,stopInjected=false;
 uint32_t lastBusTick=0,startTick=0,maxSkew=0,frames=0;uint16_t minFree=8;
 const uint32_t endTicks=uint32_t(records.size())*400+60000;
 for(uint32_t tick=0;tick<endTicks;++tick){
  const uint32_t ms=tick/20;
  for(auto& node:nodes)node.tick();
  if(scenario==FAST_CLOCK && tick%1000==0)nodes[6].tick(); // Deliberate1000ppm oscillator mismatch.
  if(tick%20==0){
   bool pause=scenario==HOST_PAUSE && started && tick>=startTick+18000;
   if(!pause && ms%50==0)core.Heartbeat(ms);
   auto s=core.GetState();
   if(!pause && next<records.size() && (s==CoreStreamEngine::OPEN||s==CoreStreamEngine::BUFFERING||s==CoreStreamEngine::ARMED||s==CoreStreamEngine::RUNNING)){
    for(unsigned i=0;i<4&&core.Free()&&next<records.size();++i){CHECK(core.Submit(records[next],ms)==OK);++next;}
   }
   if(s==CoreStreamEngine::ARMED&&!started){CHECK(core.Start(records[0].session,ms)==OK);started=true;startTick=tick;}
   if(scenario==STOP_MID&&started&&!stopInjected&&tick>startTick+20000){core.ProtocolFault(ABORTED);stopInjected=true;}
   core.Poll(ms);
  }
  if(scenario==BUS_BACKGROUND && tick%100==0){
   // Two competing frames every 5ms: the existing 200Hz position request/reply.
   // These have higher arbitration priority than stream transfer frames.
   for(unsigned n=0;n<2;++n){BusFrame background{0x20+n,{},false};bus.push_back(background);}
  }
  // 150us/frame is slightly more conservative than the protocol's 140bit
  // occupancy budget on a1Mbit/s classical CAN bus. Lowest ID wins arbitration.
  if(!bus.empty()&&tick-lastBusTick>=3){
   auto it=std::min_element(bus.begin(),bus.end(),[](const BusFrame&a,const BusFrame&b){return a.id<b.id;});
   BusFrame f=*it;bus.erase(it);lastBusTick=tick;++frames;
   const uint8_t node=uint8_t(f.id>>7),cmd=uint8_t(f.id&127);const uint16_t seq=Read16(f.data+2);
   bool drop=false;
   if(!dropped&&scenario==LOST_COMMIT&&f.core&&node==6&&cmd==COMMIT&&seq==20){drop=dropped=true;}
   if(!dropped&&scenario==LOST_VALUE&&f.core&&node==3&&cmd==V&&seq==20){drop=dropped=true;}
   if(!dropped&&scenario==LOST_READY&&!f.core&&node==2&&cmd==STATUS&&seq==20&&f.data[4]==STATE_PREPARED){drop=dropped=true;}
   if(!dropped&&scenario==LOST_COMMIT_ACK&&!f.core&&node==6&&cmd==STATUS&&seq==20&&f.data[4]==STATE_COMMITTED){drop=dropped=true;}
   if(scenario==LOST_SYNC&&started&&tick>startTick+20000&&f.core&&cmd==SYNC)drop=true;
   if(!drop){
    if(f.core){
     for(unsigned n=0;n<7;++n)if(!node||node==n+1){
      if(!dropped&&scenario==LOST_START&&cmd==START&&n==3){dropped=true;continue;}
      nodes[n].receive(cmd,f.data);
     }
    }else core.Receive(node,cmd,f.data,ms);
   }
  }
  for(auto& node:nodes)node.poll();
  if(scenario==OLD_SESSION&&started&&!oldInjected&&tick>startTick+20000){
   uint8_t old[8]{};Write16(old,uint16_t(records[0].session-1));Write16(old+2,17);
   for(auto&node:nodes){node.receive(Q,old);node.receive(START,old);node.receive(ABORT,old);}
   oldInjected=true;
  }
  if(started){uint32_t a=UINT32_MAX,b=0;for(auto&node:nodes)if(node.anchor()){a=std::min(a,node.anchor());b=std::max(b,node.anchor());}if(a!=UINT32_MAX)maxSkew=std::max(maxSkew,b-a);}
  if(core.GetState()==CoreStreamEngine::FAULT){
   core.Poll(ms+1);
   std::printf("scenario=%u FAULT reason=%u tick=%u sent=%u frames=%u next=%u committed=%u skewTicks=%u\n",scenario,core.Reason(),tick,port.sent,frames,unsigned(next),core.LastCommitted(),maxSkew);
   CHECK(scenario==LOST_VALUE||scenario==HOST_PAUSE||scenario==STOP_MID||scenario==LOST_SYNC||scenario==FAST_CLOCK);CHECK(port.stopped);
   // Existing coordinated-stop adapter is modeled explicitly. Late stream
   // commands then pass through the real motor admission code and cannot revive it.
   uint8_t late[8]{};Write16(late,records[0].session);Write16(late+2,1);Write16(late+4,2000);
   for(auto&node:nodes){node.receive(START,late);node.poll();for(unsigned i=0;i<25;++i)node.tick();CHECK(node.locked());CHECK(node.anchor()==0);}
   return;
  }
  if(started&&core.GetState()==CoreStreamEngine::IDLE){
   CHECK(scenario!=LOST_VALUE&&scenario!=HOST_PAUSE&&scenario!=STOP_MID&&scenario!=LOST_SYNC&&scenario!=FAST_CLOCK);
   CHECK(next==records.size());CHECK(!port.stopped);CHECK(core.LastExecuted()==records.back().sequence);
   for(unsigned n=0;n<7;++n){CHECK(nodes[n].completed()==records.back().sequence);CHECK(nodes[n].q()==records.back().axes[n].q);CHECK(!nodes[n].locked());}
   CHECK(maxSkew<=40); // ≤2ms model bound including rounded1ms remaining-delay scheduling.
   std::printf("scenario=%u DONE tick=%u frames=%u sent=%u segments=%u skewTicks=%u\n",scenario,tick,frames,port.sent,unsigned(records.size()-1),maxSkew);
   CHECK(core.Close(records[0].session,ms)==OK);
   for(unsigned close=0;close<3000&&core.Session();++close){
    const uint32_t closeMs=ms+close/20;
    if(close%20==0)core.Poll(closeMs);
    if(!bus.empty()&&close%3==0){BusFrame f=bus.front();bus.erase(bus.begin());const uint8_t n=uint8_t(f.id>>7),c=uint8_t(f.id&127);
     if(f.core){for(unsigned i=0;i<7;++i)if(!n||n==i+1)nodes[i].receive(c,f.data);}else core.Receive(n,c,f.data,closeMs);}
    for(auto&node:nodes){node.tick();node.poll();}
   }
   CHECK(core.Session()==0);for(auto&node:nodes)CHECK(!node.locked());
   return;
  }
 }
 std::fprintf(stderr,"TIMEOUT scenario=%u state=%s reason=%u next=%u commit=%u\n",scenario,CoreStreamEngine::Name(core.GetState()),core.Reason(),unsigned(next),core.LastCommitted());
 for(unsigned i=0;i<7;++i)std::fprintf(stderr,"node%u state%u why%u done%u lock%d\n",i+1,nodes[i].state(),nodes[i].reason(),nodes[i].completed(),nodes[i].locked());
 CHECK(false);
}
int main(int argc,char**argv){
 CHECK(argc==2);std::ifstream file(argv[1]);CHECK(bool(file));std::string line;std::vector<StreamProtocol::Record> records;
 while(std::getline(file,line)){if(!line.empty()&&line.back()=='\r')line.pop_back();if(line.empty())continue;StreamProtocol::Record record{};CHECK(StreamProtocol::DecodeLine(line.data(),line.size(),record));records.push_back(record);}
 CHECK(records.size()>100);for(unsigned s=0;s<=LOST_COMMIT;++s)simulate(records,Scenario(s));
 std::printf("PASS combined checks=%u actualPCrecords=%u productioncore+7motoradapters+20kHzcurves IDEAL_CAN_NOT_DRIVER_VALIDATION\n",checks,unsigned(records.size()));
}
