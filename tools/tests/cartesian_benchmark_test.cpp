#include "cartesian_benchmark.h"
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <utility>
#include <vector>

namespace {
unsigned assertions=0,failures=0,rows=0;
void Expect(bool test,const char* name){
    ++assertions;if(!test){++failures;std::cerr<<"FAIL "<<name<<'\n';}
}
struct Clock {
    uint32_t cycles=0,ticks=0,cycleStep=1,tickStep=0,checkpoints=0,stopAt=0;
    uint8_t stopFlag=CartBench::COMPLETE;
    std::vector<std::pair<unsigned,uint32_t>> boundaries;
    static uint32_t Cycles(void* p){auto& c=*static_cast<Clock*>(p);const auto out=c.cycles;c.cycles+=c.cycleStep;return out;}
    static uint32_t Ticks(void* p){auto& c=*static_cast<Clock*>(p);const auto out=c.ticks;c.ticks+=c.tickStep;return out;}
    static uint8_t Checkpoint(void* p){auto& c=*static_cast<Clock*>(p);++c.checkpoints;return c.stopAt&&c.checkpoints>=c.stopAt?c.stopFlag:uint8_t(CartBench::COMPLETE);}
    static void Boundary(void* p,unsigned completed,uint32_t cumulativeCycles){
        static_cast<Clock*>(p)->boundaries.emplace_back(completed,cumulativeCycles);
    }
    CartBench::Hooks Hooks(uint32_t hz=168000000U,bool profile=false){
        return {this,Cycles,Ticks,Checkpoint,hz,profile?Boundary:nullptr};
    }
};
bool SameSample(const CartBench::Sample& a,const CartBench::Sample& b){
    return a.cycles==b.cycles&&a.ticks==b.ticks&&a.work==b.work&&a.fk==b.fk&&a.ik==b.ik
        &&a.valid==b.valid&&a.selected==b.selected&&a.failed==b.failed
        &&a.flag==b.flag&&a.checksum==b.checksum;
}
void TestBoundaryCallbacks(CartBench::Compute& engine){
    using namespace CartBench;
    Config c{ROUND_TRIP,0,10,1,100};Expect(engine.Prepare(c),"callback Prepare");
    Clock budget;budget.cycleStep=25;
    auto r=engine.Measure(c,budget.Hooks(1000000,true));
    Expect(r.flag==BUDGET&&r.work==2&&budget.boundaries==std::vector<std::pair<unsigned,uint32_t>>{{1,50},{2,100}},
        "post-work budget records last completed boundary");
    // Keep the public 100us minimum, with a 75-cycle budget at 750kHz.
    Clock nextBudget;nextBudget.cycleStep=25;
    r=engine.Measure(c,nextBudget.Hooks(750000,true));
    Expect(r.flag==BUDGET&&r.work==1&&nextBudget.boundaries==std::vector<std::pair<unsigned,uint32_t>>{{1,50}},
        "pre-work budget does not invent another completed boundary");
    Clock before;before.cycleStep=100;
    r=engine.Measure(c,before.Hooks(1000000,true));
    Expect(r.flag==BUDGET&&r.work==0&&before.boundaries.empty(),"pre-work budget has no callback");
    c.budgetUs=20000;
    for(uint8_t flag:{uint8_t(CANCELLED),uint8_t(ABORTED),uint8_t(99)}){
        Clock terminal;terminal.stopAt=2;terminal.stopFlag=flag;
        r=engine.Measure(c,terminal.Hooks(168000000,true));
        Expect(r.work==1&&r.flag==(flag==CANCELLED?CANCELLED:ABORTED)
            &&terminal.boundaries==std::vector<std::pair<unsigned,uint32_t>>{{1,2}},
            "post-work cancellation or abort preserves completed boundary");
    }
    Clock preCancel;preCancel.stopAt=1;preCancel.stopFlag=CANCELLED;
    r=engine.Measure(c,preCancel.Hooks(168000000,true));
    Expect(r.work==0&&r.flag==CANCELLED&&preCancel.boundaries.empty(),"pre-work cancel has no callback");
    Clock wrap;wrap.cycles=0xfffffffaU;wrap.cycleStep=3;wrap.ticks=0xfffffffeU;wrap.tickStep=1;
    c.work=2;r=engine.Measure(c,wrap.Hooks(168000000,true));
    Expect(r.flag==COMPLETE&&r.cycles==12&&r.ticks==4
        &&wrap.boundaries==std::vector<std::pair<unsigned,uint32_t>>{{1,6},{2,12}},
        "callback cumulative cycles use unsigned wrap");
}
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
        const auto repeat=engine.Measure(c,repeatClock.Hooks(168000000,true));
        Expect(SameSample(repeat,r),"callback leaves all baseline sample fields unchanged");
        Expect(repeatClock.checkpoints==clock.checkpoints&&repeatClock.cycles==clock.cycles&&repeatClock.ticks==clock.ticks,
            "callback does not add timing reads or checkpoints");
        Expect(repeatClock.boundaries.size()==count,"one callback per completed work");
        for(unsigned i=0;i<count&&i<repeatClock.boundaries.size();++i)
            Expect(repeatClock.boundaries[i]==std::make_pair(i+1,uint32_t(2*(i+1))),"callback follows post-work checkpoint");
    }
    TestBoundaryCallbacks(engine);
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
