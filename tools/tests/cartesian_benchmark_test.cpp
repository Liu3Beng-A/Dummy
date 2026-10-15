#include "cartesian_benchmark.h"
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>

namespace {
unsigned assertions=0,failures=0,rows=0;
void Expect(bool test,const char* name){
    ++assertions;if(!test){++failures;std::cerr<<"FAIL "<<name<<'\n';}
}
struct Clock {
    uint32_t cycles=0,ticks=0,cycleStep=1,tickStep=0,checkpoints=0,stopAt=0;
    uint8_t stopFlag=CartBench::COMPLETE;
    static uint32_t Cycles(void* p){auto& c=*static_cast<Clock*>(p);const auto out=c.cycles;c.cycles+=c.cycleStep;return out;}
    static uint32_t Ticks(void* p){auto& c=*static_cast<Clock*>(p);const auto out=c.ticks;c.ticks+=c.tickStep;return out;}
    static uint8_t Checkpoint(void* p){auto& c=*static_cast<Clock*>(p);++c.checkpoints;return c.stopAt&&c.checkpoints>=c.stopAt?c.stopFlag:uint8_t(CartBench::COMPLETE);}
    CartBench::Hooks Hooks(uint32_t hz=168000000U){return {this,Cycles,Ticks,Checkpoint,hz};}
};
void Row(const char* name,const CartBench::Config& c,const CartBench::Sample& r){
    if(rows++)std::cout<<",\n";
    std::cout<<"{\"case\":\""<<name<<"\",\"op\":"<<unsigned(c.operation)
        <<",\"fixture\":"<<unsigned(c.fixture)<<",\"requested_work\":"<<c.work
        <<",\"cycles\":"<<r.cycles<<",\"ticks\":"<<r.ticks<<",\"work\":"<<r.work
        <<",\"fk\":"<<r.fk<<",\"ik\":"<<r.ik<<",\"valid\":"<<r.valid
        <<",\"selected\":"<<r.selected<<",\"failed\":"<<r.failed
        <<",\"flag\":"<<unsigned(r.flag)<<",\"checksum\":"<<r.checksum<<'}';
}
}
int main(){
    using namespace CartBench;
    std::cout<<"{\"mcu_timing\":false,\"clock\":\"synthetic deterministic boundary clock\",\"rows\":[\n";
    Compute engine;
    Config c{};
    Clock unpreparedClock;
    auto unprepared=engine.Measure(c,unpreparedClock.Hooks());
    Expect(unprepared.flag==ABORTED&&unprepared.work==0,"requires Prepare");
    for(unsigned fixture=0;fixture<3;++fixture)for(unsigned op=0;op<6;++op)for(unsigned count:{1U,5U,192U}){
        c={uint8_t(op),uint8_t(fixture),uint16_t(count),128,20000};
        Expect(Valid(c)&&engine.Prepare(c),"valid matrix prepare");
        Clock clock;
        const auto r=engine.Measure(c,clock.Hooks());Row("matrix",c,r);
        Expect(r.flag==COMPLETE&&r.work==count,"matrix completion");
        Expect(clock.checkpoints==2*count,"pre and post work checkpoints");
        Expect(r.cycles==2*count&&r.ticks==0,"raw full sample deltas");
        Expect(r.checksum!=0,"observable computation checksum");
        if(op==FK){Expect(r.fk==count&&r.ik==0&&r.valid==0&&r.selected==0&&r.failed==0,"FK accounting");}
        else if(op==UNREACHABLE){Expect(r.fk==0&&r.ik==count&&r.valid==0&&r.selected==0&&r.failed==count,"expected unreachable accounting");}
        else{
            Expect(r.ik==count&&r.valid>=count&&r.failed==0,"IK branches available");
            Expect(r.fk==(op==IK?0U:(op==ROUND_TRIP?count:2*count)),"FK calls counted");
            Expect(r.selected==((op==TCP_SELECT||op==RAIL_CANDIDATE)?count:0U),"legal branch residual validated");
        }
        Clock repeatClock;
        const auto repeat=engine.Measure(c,repeatClock.Hooks());
        Expect(repeat.checksum==r.checksum,"deterministic checksum");
    }
    c={ROUND_TRIP,0,10,1,100};Expect(engine.Prepare(c),"budget prepare");
    Clock budget;budget.cycleStep=25;
    auto r=engine.Measure(c,budget.Hooks(1000000));Row("budget_exact",c,r);
    Expect(r.flag==BUDGET&&r.work==2&&r.cycles==100&&r.fk==2&&r.ik==2,"budget reached after completed work");
    Clock before;before.cycleStep=100;
    r=engine.Measure(c,before.Hooks(1000000));Row("budget_before_work",c,r);
    Expect(r.flag==BUDGET&&r.work==0&&r.cycles==100&&r.ik==0,"budget reached before first work");
    Clock highClock;highClock.cycleStep=0x1000000U;
    c.budgetUs=20000;
    r=engine.Measure(c,highClock.Hooks(4000000000U));Row("budget_u64",c,r);
    Expect(r.flag==BUDGET&&r.work==2&&r.cycles==5*0x1000000U,"budget multiplication uses 64 bits");
    Clock cancel;cancel.stopAt=2;cancel.stopFlag=CANCELLED;
    r=engine.Measure(c,cancel.Hooks());Row("cancel_after_work",c,r);
    Expect(r.flag==CANCELLED&&r.work==1&&r.cycles==2,"cancel preserves completed work and delta");
    Clock abort;abort.stopAt=1;abort.stopFlag=ABORTED;
    r=engine.Measure(c,abort.Hooks());Row("abort_before_work",c,r);
    Expect(r.flag==ABORTED&&r.work==0&&r.cycles==1,"abort checked before work");
    Clock invalidFlag;invalidFlag.stopAt=1;invalidFlag.stopFlag=99;
    r=engine.Measure(c,invalidFlag.Hooks());
    Expect(r.flag==ABORTED&&r.work==0,"unknown checkpoint flag aborts");
    Clock wrap;wrap.cycles=0xfffffffaU;wrap.cycleStep=3;wrap.ticks=0xfffffffeU;wrap.tickStep=1;
    c.work=2;
    r=engine.Measure(c,wrap.Hooks());Row("clock_wrap",c,r);
    Expect(r.flag==COMPLETE&&r.work==2&&r.cycles==12&&r.ticks==4,"unsigned cycle and tick wrap");
    Clock dead;dead.cycleStep=0;dead.tickStep=1000;
    r=engine.Measure(c,dead.Hooks());Row("independent_tick_guard",c,r);
    Expect(r.flag==ABORTED&&r.work==0&&r.cycles==0&&r.ticks==1000,"independent tick guard catches stopped DWT");
    Clock missingClock;
    auto h=missingClock.Hooks();h.clockHz=0;
    Expect(engine.Measure(c,h).flag==ABORTED,"zero frequency aborts");
    h=missingClock.Hooks();h.cycles=nullptr;
    Expect(engine.Measure(c,h).flag==ABORTED,"missing cycles aborts");
    h=missingClock.Hooks();h.ticks=nullptr;
    Expect(engine.Measure(c,h).flag==ABORTED,"missing ticks aborts");
    h=missingClock.Hooks();h.checkpoint=nullptr;
    Expect(engine.Measure(c,h).flag==ABORTED,"missing checkpoint aborts");
    Config invalid=c;
    invalid.operation=6;Expect(!Valid(invalid),"invalid op");
    invalid=c;invalid.fixture=3;Expect(!Valid(invalid),"invalid fixture");
    invalid=c;invalid.work=0;Expect(!Valid(invalid),"zero work");
    invalid=c;invalid.work=193;Expect(!Valid(invalid),"work upper bound");
    invalid=c;invalid.samples=0;Expect(!Valid(invalid),"zero samples");
    invalid=c;invalid.samples=129;Expect(!Valid(invalid),"samples upper bound");
    invalid=c;invalid.budgetUs=99;Expect(!Valid(invalid),"budget lower bound");
    invalid=c;invalid.budgetUs=20001;Expect(!Valid(invalid),"budget upper bound");
    Expect(!engine.Prepare(invalid),"Prepare rejects invalid configuration");
    Expect(engine.Measure(c,missingClock.Hooks()).flag==ABORTED,"failed Prepare invalidates previous seed");
    Expect(engine.Prepare(c),"reprepare after invalid");
    invalid=c;invalid.fixture=1;
    Expect(engine.Measure(invalid,missingClock.Hooks()).flag==ABORTED,"fixture mismatch cannot reuse old seed");
    float q=0;
    Expect(DOF6Kinematic::NearestInLimits(330,-10,-175,175,q)&&std::abs(q+30)<.001f,"nearest legal equivalent");
    Expect(!DOF6Kinematic::NearestInLimits(180,0,-100,100,q),"no legal equivalent rejected");
    std::cout<<"\n],\"assertions\":"<<assertions<<",\"failures\":"<<failures<<"}\n";
    return failures?1:0;
}
