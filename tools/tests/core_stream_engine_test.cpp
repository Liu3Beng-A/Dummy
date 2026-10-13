#include "../../firmware_new/dummy-ref-core-fw/Robot/instances/core_stream_engine.h"
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <array>
#include <algorithm>
using namespace StreamProtocol;
static unsigned checks=0;
#define CHECK(x) do { ++checks; if(!(x)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x); exit(1); } } while(0)
struct Sim : CoreStreamPort {
    struct Frame { unsigned due; uint8_t node,command; std::array<uint8_t,8> data; };
    struct Motor { uint16_t session=0,committed=0,executed=0,end=0; unsigned anchor=0,startCount=0; bool open=false,started=false,done=false; };
    CoreStreamEngine e{*this}; std::vector<Frame> inbound,outbound,sent,heldCommitAcks; Motor motors[7]{};
    unsigned now=1,budget=3,stopCount=0; bool allowed=true,valid=true;
    int absent=-1,dropFirstStart=-1,dropPrepared=-1,dropCommitAck=-1,dropAllStarts=-1;
    bool losePreparedOnce=false,loseCommitOnce=false,denyPairStartOnce=false,deniedPairStart=false;
    unsigned startBroadcasts=0; int blockedSequence=-1,heldCommitSequence=-1; unsigned blockedAt=0; bool dropClose=false,blockOpen=false;uint8_t features=CAP_TIMED|CAP_OPEN_SNAPSHOT;uint8_t stableNodes=0;
    bool Send(uint8_t node,uint8_t command,const uint8_t d[8]) override {
        if(command>=Q && command<=COMMIT && int(Read16(d+2))==blockedSequence) { if(!blockedAt) blockedAt=now; return false; }
        if(command==START && node==7 && denyPairStartOnce && !deniedPairStart) { deniedPairStart=true;return false; }
        if(!budget) return false; --budget;
        Frame f{now+1,node,command,{}}; std::copy(d,d+8,f.data.begin()); outbound.push_back(f);sent.push_back(f);
        if(command==START) ++startBroadcasts;
        return true;
    }
    bool Allowed() const override { return allowed; }
    bool Validate(const Record&,const Record*,uint8_t) const override { return valid; }
    void Stop() override { ++stopCount; allowed=false; e.Cancel(); }
    void Reply(unsigned n,unsigned state,unsigned seq,unsigned reason=0) {
        const auto& m=motors[n]; Frame f{now+1,uint8_t(n+1),STATUS,{}};
        Write16(f.data.data(),m.session); Write16(f.data.data()+2,uint16_t(seq));
        f.data[4]=uint8_t(state); f.data[5]=uint8_t(CAPACITY-(m.committed-m.executed)); f.data[6]=uint8_t(reason); f.data[7]=VERSION;
        if(state==STATE_COMMITTED && int(seq)==heldCommitSequence) heldCommitAcks.push_back(f);
        else inbound.push_back(f);
    }
    void Deliver(const Frame& f,unsigned n) {
        if(int(n)==absent) return;
        auto& m=motors[n]; const auto* d=f.data.data(); const auto session=Read16(d),seq=Read16(d+2);
        if(f.command==CAPS) { Frame reply{now+1,uint8_t(n+1),CAPS,{}}; Write16(reply.data.data(),session);reply.data[2]=VERSION;reply.data[3]=features|((stableNodes&(1U<<n))?CAP_STABLE_ENTRY:0);Write16(reply.data.data()+4,SEGMENT_TICKS);reply.data[6]=CAPACITY;reply.data[7]=42;inbound.push_back(reply); }
        else if(f.command==OPEN) { m.session=session;m.open=!blockOpen;Reply(n,blockOpen?STATE_FAULT:STATE_OPEN,0,blockOpen?BLOCKED:OK); }
        else if(session!=m.session) return;
        else if(f.command==PREPARE) { if(int(n)==dropPrepared && !losePreparedOnce) { losePreparedOnce=true; return; } Reply(n,STATE_PREPARED,seq); }
        else if(f.command==COMMIT) { m.committed=std::max(m.committed,seq); if(d[6]&END) m.end=seq; if(int(n)==dropCommitAck && !loseCommitOnce) { loseCommitOnce=true; return; } Reply(n,STATE_COMMITTED,seq); }
        else if(f.command==START) {
            if(int(n)==dropAllStarts) return;
            if(int(n)==dropFirstStart && !m.startCount++) return;
            if(!m.started) { m.started=true; m.anchor=now*20+Read16(d+4); }
            Reply(n,STATE_STARTED,1);
        }
        else if(f.command==ABORT) { if(dropClose) return; m.started=false; m.open=false; if(!d[2]) Reply(n,STATE_IDLE,m.executed); }
    }
    void Tick(bool heartbeat=true) {
        budget=3;
        if(heartbeat && now%20==0) e.Heartbeat(now);
        e.Poll(now);
        auto sent=outbound; outbound.clear();
        for(const auto& f:sent) {
            if(f.due>now) {outbound.push_back(f);continue;}
            if(f.node) Deliver(f,f.node-1); else for(unsigned i=0;i<7;++i) Deliver(f,i);
        }
        for(unsigned i=0;i<7;++i) { auto& m=motors[i]; if(!m.started || m.done || now*20<m.anchor) continue;
            const unsigned elapsed=now*20-m.anchor;
            const uint16_t done=uint16_t(std::min(elapsed/SEGMENT_TICKS,unsigned(m.committed)));
            if(done>m.executed || elapsed%SEGMENT_TICKS==0) { m.executed=done; Reply(i,STATE_RUNNING,done); }
            if(m.end && elapsed>=unsigned(m.end)*SEGMENT_TICKS+1000) {m.done=true;Reply(i,STATE_COMPLETED,m.end);}
        }
        auto received=inbound; inbound.clear();
        for(const auto& f:received) { if(f.due>now) inbound.push_back(f); else e.Receive(f.node,f.command,f.data.data(),now); }
        ++now;
    }
    void Run(unsigned ms,bool heartbeat=true) { for(unsigned i=0;i<ms;++i) Tick(heartbeat); }
    void Open(uint8_t mask=ALL_AXES) { CHECK(e.Begin(123,mask,now)==OK); Run(100); CHECK(e.GetState()==CoreStreamEngine::OPEN); }
    void Fill(unsigned last,bool end=true) { for(unsigned i=e.Next();i<=last;++i) { Record r{};r.session=123;r.sequence=uint16_t(i);r.flags=i==0?SEED:i==last&&end?END:0;CHECK(e.Submit(r,now)==OK); } }
    void Arm(unsigned last=12,bool end=true) { Open(); Fill(last,end); Run(200); CHECK(e.GetState()==CoreStreamEngine::ARMED); CHECK(e.Start(123,now)==OK); }
    // Reach the exact 20ms lookahead boundary through public APIs. The motor
    // simulator has committed endpoint16 but its ACKs remain in the wire until
    // this test delivers them immediately before the critical coordinator Poll.
    void LookaheadBoundary(uint8_t mask,unsigned transferAge=30,bool end=false,bool submit=true) {
        Open(mask);Fill(15,false);Run(200);CHECK(e.GetState()==CoreStreamEngine::ARMED);
        const unsigned anchor=now+START_DELAY_TICKS/20;
        CHECK(e.Start(123,now)==OK);
        while(now<anchor+280-transferAge) Tick();
        CHECK(e.LastCommitted()==15&&e.GetState()==CoreStreamEngine::RUNNING);
        if(submit) {
            heldCommitSequence=16;
            Record r{};r.session=123;r.sequence=16;r.flags=end?END:0;
            CHECK(e.Submit(r,now)==OK);
        }
        while(now<anchor+280) Tick();
        CHECK(e.GetState()==CoreStreamEngine::RUNNING&&e.LastCommitted()==15);
        if(submit) {
            unsigned received=0;
            for(const auto& f:heldCommitAcks) received|=1U<<(f.node-1);
            CHECK((received&mask)==mask);
        }
    }
    void DeliverHeldCommitAcks(uint8_t mask) {
        for(const auto& f:heldCommitAcks) if(mask&(1U<<(f.node-1)))
            e.Receive(f.node,f.command,f.data.data(),now);
        heldCommitAcks.clear();
    }
};
int main() {
    // Already received COMMITTED ACKs must be reflected in lookahead before it
    // can declare underrun, for single-axis, partial pair and all-axis sessions.
    for(unsigned mask:{1U,65U,127U}) {
      {Sim s;s.LookaheadBoundary(uint8_t(mask));s.DeliverHeldCommitAcks(uint8_t(mask));
       s.budget=3;s.e.Poll(s.now);
       CHECK(s.e.GetState()==CoreStreamEngine::RUNNING&&s.e.LastCommitted()==16);
       CHECK(s.e.Free()==CORE_CAPACITY&&s.stopCount==0);}
      // Every missing selected ACK must still fail. A node outside this mask
      // cannot contribute an ACK or a credit to satisfy the group.
      for(unsigned node=0;node<7;++node) if(mask&(1U<<node)) {
       Sim s;s.LookaheadBoundary(uint8_t(mask));
       const uint8_t partial=uint8_t(mask&~(1U<<node));
       s.DeliverHeldCommitAcks(partial);s.budget=3;s.e.Poll(s.now);
       const auto f=s.e.Failure();
       CHECK(s.e.Reason()==UNDERRUN&&s.e.LastCommitted()==15);
       CHECK(f.source==CoreStreamEngine::SOURCE_PROTOCOL&&f.at==s.now&&f.age==0);
       CHECK(f.sequence==16&&f.phase==3&&f.prepared==mask&&f.committed==partial);
       CHECK(s.e.Free()==CORE_CAPACITY);s.Tick();CHECK(s.stopCount==1);
      }
      {Sim s;s.LookaheadBoundary(uint8_t(mask),30,false,false);s.budget=3;s.e.Poll(s.now);
       CHECK(s.e.Reason()==UNDERRUN&&s.e.LastCommitted()==15);
       CHECK(s.e.Failure().at==s.now&&s.e.Failure().sequence==15&&s.e.Failure().phase==0);}
      // Publication changes no deadline. At59ms all ACKs may publish; at60ms
      // their endpoint must be rejected even when it would restore lookahead.
      for(unsigned age:{59U,60U}) {
       Sim s;s.LookaheadBoundary(uint8_t(mask),age);s.DeliverHeldCommitAcks(uint8_t(mask));
       s.budget=3;s.e.Poll(s.now);
       if(age==59)CHECK(s.e.GetState()==CoreStreamEngine::RUNNING&&s.e.LastCommitted()==16);
       else {
        CHECK(s.e.Reason()==DEADLINE&&s.e.LastCommitted()==15);
        CHECK(s.e.Failure().source==CoreStreamEngine::SOURCE_TRANSFER);
        CHECK(s.e.Failure().age==60&&s.e.Failure().at==s.now);
       }
      }
      {Sim s;s.LookaheadBoundary(uint8_t(mask),30,true);s.DeliverHeldCommitAcks(uint8_t(mask));
       s.budget=3;s.e.Poll(s.now);
       CHECK(s.e.GetState()==CoreStreamEngine::DRAINING&&s.e.LastCommitted()==16);
       s.Run(120);CHECK(s.e.GetState()==CoreStreamEngine::IDLE&&s.e.LastExecuted()==16&&s.stopCount==0);}
      {Sim s;s.LookaheadBoundary(uint8_t(mask));s.DeliverHeldCommitAcks(uint8_t(mask));
       s.budget=3;s.e.Poll(s.now+10);CHECK(s.e.Reason()==DEADLINE&&s.e.LastCommitted()==15);
       CHECK(s.e.Failure().source==CoreStreamEngine::SOURCE_POLL&&s.e.Failure().age==11);}
      {Sim s;s.LookaheadBoundary(uint8_t(mask));s.DeliverHeldCommitAcks(uint8_t(mask));
       s.e.Cancel(ABORTED);s.budget=3;s.e.Poll(s.now);
       CHECK(s.e.Reason()==ABORTED&&s.e.GetState()==CoreStreamEngine::FAULT&&s.e.LastCommitted()==15);
       CHECK(!s.e.CanTransmit(123));}
      {Sim s;s.LookaheadBoundary(uint8_t(mask));s.DeliverHeldCommitAcks(uint8_t(mask));
       s.allowed=false;s.budget=3;s.e.Poll(s.now);
       CHECK(s.e.Reason()==BLOCKED&&s.e.LastCommitted()==15);}
      // Pre-START publication keeps the separate500ms limit as well. Only the
      // old fully confirmed transfer may finish; it cannot arm past its deadline.
      for(unsigned age:{499U,500U}) {
       Sim s;s.Open(uint8_t(mask));s.heldCommitSequence=1;s.Fill(1);s.Run(45);
       CHECK(s.e.GetState()==CoreStreamEngine::BUFFERING&&s.e.LastCommitted()==0);
       unsigned began=0,received=0;
       for(const auto& f:s.sent)if(f.command==Q&&Read16(f.data.data()+2)==1&&!began)began=f.due-1;
       for(const auto& f:s.heldCommitAcks)received|=1U<<(f.node-1);
       CHECK(began&&(received&mask)==mask);
       s.now=began+age;s.DeliverHeldCommitAcks(uint8_t(mask));s.budget=3;s.e.Poll(s.now);
       if(age==499)CHECK(s.e.GetState()==CoreStreamEngine::ARMED&&s.e.LastCommitted()==1);
       else {
        CHECK(s.e.Reason()==DEADLINE&&s.e.LastCommitted()==0);
        CHECK(s.e.Failure().source==CoreStreamEngine::SOURCE_TRANSFER);
        CHECK(s.e.Failure().age==500&&s.e.Failure().at==s.now);
       }
      }
    }
    { Sim s; CHECK(!s.e.Active());CHECK(!s.e.Busy());s.allowed=false;CHECK(s.e.Begin(1,127,1)==BLOCKED); }
    { Sim s;CHECK(s.e.Begin(0,127,1)==BAD_SESSION);CHECK(s.e.Begin(1,3,1)==BAD_FLAGS); }
    { Sim s;s.Open();CHECK(s.e.CapabilityMask()==127);CHECK(s.e.Begin(124,127,s.now)==BUSY);CHECK(s.e.Start(123,s.now)==BAD_STATE); }
    { Sim s;s.Open(1);s.Fill(1);s.Run(50);CHECK(s.e.GetState()==CoreStreamEngine::ARMED);CHECK(s.e.Start(123,s.now)==OK);s.Run(300);CHECK(s.e.GetState()==CoreStreamEngine::IDLE);CHECK(s.stopCount==0);CHECK(s.e.Close(123,s.now)==OK);s.Run(20);CHECK(!s.e.Active());CHECK(s.e.Begin(123,1,s.now)==BAD_SESSION); }
    { Sim s;s.Arm();s.Run(600);CHECK(s.e.GetState()==CoreStreamEngine::IDLE);CHECK(s.stopCount==0);CHECK(s.e.LastExecuted()==12);for(auto& m:s.motors)CHECK(m.done); }
    { Sim s;s.dropFirstStart=6;s.Arm();s.Run(600);CHECK(s.e.GetState()==CoreStreamEngine::IDLE);CHECK(s.startBroadcasts>=2);for(auto& m:s.motors)CHECK(m.anchor==s.motors[0].anchor); }
    { Sim s;s.dropAllStarts=6;s.Arm();s.Run(100);CHECK(s.e.GetState()==CoreStreamEngine::FAULT);CHECK(s.e.Reason()==INCOMPLETE || s.e.Reason()==DEADLINE);CHECK(s.stopCount==1); }
    { Sim s;s.dropPrepared=6;s.dropCommitAck=3;s.Arm();s.Run(600);CHECK(s.e.GetState()==CoreStreamEngine::IDLE);CHECK(s.losePreparedOnce && s.loseCommitOnce);CHECK(s.stopCount==0); }
    { Sim s;s.absent=6;CHECK(s.e.Begin(123,127,s.now)==OK);s.Run(3100);CHECK(s.e.GetState()==CoreStreamEngine::FAULT);CHECK(s.stopCount==1); }
    { Sim s;s.Open();s.Fill(15,false);Record r{};r.session=123;r.sequence=16;CHECK(s.e.Submit(r,s.now)==BUSY);CHECK(s.e.Free()==0);s.Run(100);CHECK(s.e.Free()>0); }
    { Sim s;s.Open();s.Fill(15,false);s.Run(200);CHECK(s.e.LastCommitted()==8);uint8_t old[8]{};Write16(old,123);Write16(old+2,1);old[4]=STATE_COMMITTED;old[5]=8;old[7]=VERSION;for(unsigned n=1;n<=7;++n)s.e.Receive(n,STATUS,old,s.now);s.Run(30);CHECK(s.e.LastCommitted()==8);CHECK(s.e.GetState()==CoreStreamEngine::ARMED);CHECK(s.stopCount==0); }
    { Sim s;s.Open();Record r{};r.session=9;r.flags=SEED;CHECK(s.e.Submit(r,s.now)==BAD_SESSION);r.session=123;r.sequence=2;r.flags=0;CHECK(s.e.Submit(r,s.now)==BAD_SEQUENCE);CHECK(s.e.Next()==0); }
    { Sim s;s.Open();s.valid=false;Record r{};r.session=123;r.flags=SEED;CHECK(s.e.Submit(r,s.now)==BAD_LIMIT);s.Tick();CHECK(s.stopCount==1); }
    { Sim s;s.Arm(5,false);s.Run(240);CHECK(s.e.GetState()==CoreStreamEngine::FAULT);CHECK(s.e.Reason()==UNDERRUN);CHECK(s.stopCount==1); }
    { Sim s;s.Arm();s.Run(30);s.e.Cancel();s.Run(100);CHECK(s.e.GetState()==CoreStreamEngine::FAULT);CHECK(s.e.Reason()==ABORTED);CHECK(!s.e.CanTransmit(123)); }
    { Sim s;s.Arm();s.Tick();s.now+=11;s.Tick();CHECK(s.e.GetState()==CoreStreamEngine::FAULT);CHECK(s.e.Reason()==DEADLINE);CHECK(s.stopCount==1); }
    { Sim s;s.Open();uint8_t d[8]{};Write16(d,999);d[4]=STATE_FAULT;d[6]=BLOCKED;d[7]=VERSION;s.e.Receive(1,STATUS,d,s.now);CHECK(s.e.GetState()==CoreStreamEngine::OPEN);Write16(d,123);s.e.Receive(1,STATUS,d,s.now);CHECK(s.stopCount==0);s.Tick();CHECK(s.stopCount==1); }
    { Sim s;s.Open();s.allowed=false;s.Run(2);CHECK(s.e.GetState()==CoreStreamEngine::FAULT);CHECK(s.stopCount==1);CHECK(s.e.Reason()==BLOCKED); }
    { Sim s;s.Open();CHECK(s.e.Close(123,s.now)==OK);CHECK(s.e.Active());s.Run(20);CHECK(!s.e.Active());CHECK(s.stopCount==0); }
    { Sim s;s.Open();Record r{};r.session=123;r.flags=SEED;r.axes[0].v=1;CHECK(s.e.Submit(r,s.now)==BAD_FLAGS);s.Tick();CHECK(s.stopCount==1); }
    { Sim s;s.Arm();s.Run(200);uint8_t d[8]{};Write16(d,123);Write16(d+2,65500);d[4]=STATE_RUNNING;d[5]=8;d[7]=VERSION;s.e.Receive(1,STATUS,d,s.now);s.Tick();CHECK(s.e.Reason()==BAD_SEQUENCE);CHECK(s.stopCount==1); }
    { Sim s;s.Arm();s.Run(180);const auto latest=s.e.LastExecuted();CHECK(latest>0);
      const auto committed=s.e.LastCommitted();uint8_t d[8]{};Write16(d,123);Write16(d+2,uint16_t(latest-1));d[4]=STATE_RUNNING;d[5]=8;d[7]=VERSION;
      for(unsigned n=1;n<=7;++n)s.e.Receive(n,STATUS,d,s.now);
      CHECK(s.e.GetState()!=CoreStreamEngine::FAULT);CHECK(s.e.LastExecuted()==latest);CHECK(s.e.LastCommitted()==committed);
      s.Run(500);CHECK(s.e.GetState()==CoreStreamEngine::IDLE);CHECK(s.stopCount==0); }
    { Sim s;s.Arm();s.Run(220);CHECK(s.e.LastCommitted()==12);const auto latest=s.e.LastExecuted();CHECK(latest>0);
      uint8_t d[8]{};Write16(d,123);Write16(d+2,uint16_t(latest-1));d[4]=STATE_RUNNING;d[5]=8;d[7]=VERSION;
      for(unsigned ms=0;ms<110;++ms) { for(unsigned n=1;n<=7;++n)s.e.Receive(n,STATUS,d,s.now);s.e.Heartbeat(s.now);s.budget=3;s.e.Poll(s.now++); }
      CHECK(s.e.LastExecuted()==latest);CHECK(s.e.GetState()==CoreStreamEngine::FAULT);CHECK(s.e.Reason()==SYNC_TIMEOUT);CHECK(s.stopCount==1); }
    // START must not renew an in-progress endpoint transfer deadline.
    { Sim s;s.Open();s.blockedSequence=6;s.Fill(12);while(!s.blockedAt)s.Tick();
      CHECK(s.e.GetState()==CoreStreamEngine::ARMED);s.Run(39);CHECK(s.e.Start(123,s.now)==OK);s.Run(22);
      CHECK(s.e.GetState()==CoreStreamEngine::FAULT);CHECK(s.e.Reason()==DEADLINE);
      const auto f=s.e.Failure();CHECK(f.source==CoreStreamEngine::SOURCE_TRANSFER);CHECK(f.age==60);
      CHECK(f.sequence==6);CHECK(f.state==CoreStreamEngine::ARMED);CHECK(f.phase==1);CHECK(s.stopCount==1); }
    { Sim s;s.Open();s.dropClose=true;CHECK(s.e.Close(123,s.now)==OK);s.Run(1002);
      CHECK(s.e.Reason()==DEADLINE);CHECK(s.e.Failure().source==CoreStreamEngine::SOURCE_CLOSE);
      CHECK(s.e.Failure().age==1000);CHECK(s.stopCount==1); }
    { Sim s;s.Open();uint8_t d[8]{};Write16(d,123);Write16(d+2,47);d[4]=STATE_FAULT;d[6]=DEADLINE;d[7]=VERSION;
      s.e.Receive(7,STATUS,d,s.now);const auto f=s.e.Failure();CHECK(f.source==CoreStreamEngine::SOURCE_MOTOR);
      CHECK(f.node==7&&f.command==STATUS&&f.sequence==47);CHECK(f.state==CoreStreamEngine::OPEN);CHECK(f.at==s.now);
      s.e.TransportFailure(123,INCOMPLETE,1,Q,99,50,s.now+1);CHECK(s.e.Failure().sequence==47);CHECK(s.e.Reason()==DEADLINE); }
    { Sim s;s.Open();s.e.TransportFailure(122,DEADLINE,7,START,1,30,s.now);CHECK(s.e.GetState()==CoreStreamEngine::OPEN);
      s.e.TransportFailure(123,DEADLINE,7,Q,19,30,s.now);const auto f=s.e.Failure();CHECK(f.source==CoreStreamEngine::SOURCE_CAN);
      CHECK(f.node==7&&f.command==Q&&f.sequence==19&&f.age==30);s.Tick();CHECK(s.stopCount==1); }
    // Only the requested single-axis/all-axis masks and J1 + J7 pair are legal.
    for(unsigned mask=0;mask<256;++mask) {Sim s;CHECK(s.e.Begin(123,uint8_t(mask),s.now)==
        ((ValidMask(uint8_t(mask))||mask==0x41)?OK:BAD_FLAGS));}
    {Sim s;CHECK(s.e.Begin(123,0x41,s.now,true)==BAD_FLAGS);}
    // Old unselected sessions deliberately have the same token and a ready
    // endpoint. Directed START/SYNC/CLOSE must neither arm nor release them.
    for(unsigned mask:{1U,64U,65U}) {
      Sim s;const unsigned sentinel=mask==64?0:3;
      s.motors[sentinel].session=123;s.motors[sentinel].open=true;
      s.motors[sentinel].committed=1;s.motors[sentinel].end=1;
      s.Open(uint8_t(mask));s.Fill(12);s.Run(200);CHECK(s.e.GetState()==CoreStreamEngine::ARMED);
      CHECK(s.e.Start(123,s.now)==OK);s.Run(600);CHECK(s.e.GetState()==CoreStreamEngine::IDLE);
      CHECK(s.stopCount==0);CHECK(s.e.LastExecuted()==12);
      CHECK(s.e.Close(123,s.now)==OK);s.Run(20);CHECK(!s.e.Active());
      for(unsigned n=0;n<7;++n) if(mask&(1U<<n))CHECK(s.motors[n].done&&!s.motors[n].open);
      CHECK(s.motors[sentinel].open&&!s.motors[sentinel].started&&!s.motors[sentinel].done);
      for(const auto& f:s.sent) {
        CHECK(f.node>=1&&f.node<=7&&(mask&(1U<<(f.node-1))));
        if(f.command==OPEN)CHECK(f.data[3]==(1U<<(f.node-1)));
      }
    }
    // A rejected local J7 admission and a lost J7 START retry share the same
    // absolute anchor, including J1 already armed by its first accepted START.
    for(unsigned loss=0;loss<2;++loss) {Sim s;
      s.denyPairStartOnce=loss==0;if(loss==1)s.dropFirstStart=6;
      s.Open(0x41);s.Fill(12);s.Run(200);CHECK(s.e.Start(123,s.now)==OK);s.Run(600);
      CHECK(s.e.GetState()==CoreStreamEngine::IDLE&&s.stopCount==0);
      CHECK(s.motors[0].anchor==s.motors[6].anchor);
      if(loss==0)CHECK(s.deniedPairStart);else CHECK(s.motors[6].startCount>1);
    }
    // The first admitted START already enables 10ms coordinator-gap protection,
    // even when the second selected motor has not accepted its packet yet.
    {Sim s;s.denyPairStartOnce=true;s.Open(0x41);s.Fill(12);s.Run(200);
      CHECK(s.e.Start(123,s.now)==OK);s.Tick();CHECK(s.deniedPairStart);
      s.now+=11;s.Tick();CHECK(s.e.GetState()==CoreStreamEngine::FAULT);
      CHECK(s.e.Reason()==DEADLINE&&s.stopCount==1);
      CHECK(s.e.Failure().source==CoreStreamEngine::SOURCE_POLL);}
    // A disconnected partial-session host faults before START, keeps ownership
    // for explicit cleanup, and cannot silently be replaced after reconnection.
    for(unsigned mask:{1U,64U,65U}) {Sim s;s.Open(uint8_t(mask));s.Run(1001,false);
      CHECK(s.e.Active()&&s.e.GetState()==CoreStreamEngine::FAULT);
      CHECK(s.e.Reason()==SYNC_TIMEOUT&&s.stopCount==1);
      CHECK(s.e.Failure().source==CoreStreamEngine::SOURCE_POLL);
      CHECK(s.e.Begin(124,uint8_t(mask),s.now)==BUSY);
      for(const auto& f:s.sent)if(f.command==ABORT)CHECK(f.node&&(mask&(1U<<(f.node-1))));}
    {Sim s;s.Open();s.Run(1100,false);CHECK(s.e.GetState()==CoreStreamEngine::OPEN&&s.stopCount==0);}
    // OPEN-only never validates a curve, changes reference, broadcasts or stops.
    for(unsigned node=1;node<=7;++node) for(unsigned reject=0;reject<2;++reject) {
      Sim s;s.blockOpen=reject!=0;s.valid=false;
      CHECK(s.e.Begin(44,uint8_t(1U<<(node-1)),s.now,true)==OK);
      Record r{};r.session=44;r.flags=SEED;
      CHECK(s.e.Submit(r,s.now)==BAD_STATE);CHECK(s.e.Start(44,s.now)==BAD_STATE);
      s.Run(150,false);CHECK(!s.e.Active());CHECK(!s.e.TestCloseFailed());
      CHECK(s.e.TestSession()==44&&s.e.TestNode()==node);CHECK(s.e.TestOpened()==!reject);
      CHECK(s.e.Reason()==(reject?BLOCKED:OK));CHECK(s.stopCount==0);
      bool sawOpen=false,sawClose=false;
      for(const auto& f:s.sent){CHECK(f.node==node);CHECK(f.command==CAPS||f.command==OPEN||f.command==ABORT);sawOpen|=f.command==OPEN;sawClose|=f.command==ABORT;}
      CHECK(sawOpen&&sawClose);CHECK(s.e.Begin(44,1,s.now,true)==BAD_SESSION);
    }
    {Sim s;s.features=CAP_TIMED;CHECK(s.e.Begin(44,8,s.now,true)==OK);s.Run(50);
     CHECK(!s.e.Active()&&s.e.Reason()==BAD_VERSION&&!s.e.TestOpened());CHECK(s.stopCount==0);
     for(const auto& f:s.sent)CHECK(f.command==CAPS&&f.node==4);}
    {Sim s;s.absent=3;CHECK(s.e.Begin(44,8,s.now,true)==OK);s.Run(3100,false);
     CHECK(!s.e.Active()&&s.e.Reason()==DEADLINE);CHECK(s.stopCount==0);}
    {Sim s;s.dropClose=true;CHECK(s.e.Begin(44,8,s.now,true)==OK);s.Run(1200,false);
     CHECK(s.e.Active()&&s.e.TestCloseFailed()&&s.e.TestOpened());CHECK(s.e.Reason()==DEADLINE);CHECK(s.stopCount==0);
     CHECK(s.e.Begin(45,8,s.now,true)==BUSY);s.dropClose=false;CHECK(s.e.Close(44,s.now)==OK);s.Run(20);CHECK(!s.e.Active());}
    {Sim s;CHECK(s.e.Begin(44,8,s.now,true)==OK);s.e.Cancel();s.Run(20);CHECK(!s.e.Active());CHECK(s.stopCount==0);}
    // Late motor faults during cleanup cannot rewrite the first failure sequence.
    { Sim s;s.Open();uint8_t d[8]{};Write16(d,123);Write16(d+2,47);
      d[4]=STATE_FAULT;d[6]=BAD_SEED;d[7]=VERSION;
      s.e.Receive(4,STATUS,d,s.now);s.Tick();CHECK(s.e.Reason()==BAD_SEED);
      CHECK(s.e.Close(123,s.now)==OK);
      Write16(d+2,88);d[6]=BLOCKED;s.e.Receive(6,STATUS,d,s.now);
      CHECK(s.e.Reason()==BAD_SEED);CHECK(s.e.Failure().sequence==47);
      CHECK(s.e.Failure().node==4&&s.e.Failure().source==CoreStreamEngine::SOURCE_MOTOR);
      s.Run(20);CHECK(!s.e.Active()); }
    // A new core must not infer support from its own build version.
    for(unsigned selected:{1U,8U,64U,65U,127U}) for(unsigned modern:{0U,8U,65U,127U}) {
      Sim s;s.stableNodes=uint8_t(modern);s.Open(uint8_t(selected));
      CHECK(s.e.FeatureSession()==123);CHECK(s.e.StableEntryMask()==(selected&modern));
      CHECK(s.e.Close(123,s.now)==OK);s.Run(20);CHECK(!s.e.Active());
      CHECK(s.e.FeatureSession()==123&&s.e.StableEntryMask()==(selected&modern));
      CHECK(s.e.Begin(124,uint8_t(selected),s.now)==OK);
      CHECK(s.e.FeatureSession()==124&&s.e.StableEntryMask()==0&&s.e.CapabilityMask()==0);
      uint8_t cap[8]{};Write16(cap,123);cap[2]=VERSION;cap[3]=CAP_TIMED|CAP_OPEN_SNAPSHOT|CAP_STABLE_ENTRY;
      Write16(cap+4,SEGMENT_TICKS);cap[6]=CAPACITY;cap[7]=42;
      for(unsigned node=1;node<=7;++node)s.e.Receive(node,CAPS,cap,s.now);
      CHECK(s.e.StableEntryMask()==0&&s.e.CapabilityMask()==0);
    }
    printf("core_stream: %u checks passed; no hardware access\n",checks);
}
