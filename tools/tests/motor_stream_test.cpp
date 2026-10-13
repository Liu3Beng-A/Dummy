#include "stream_runtime.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <climits>
#include <random>
using namespace MotorStream;
using namespace StreamProtocol;
static unsigned checks=0;
#define CHECK(x) do {++checks;if(!(x)){std::fprintf(stderr,"FAIL line %u: %s\n",__LINE__,#x);std::exit(1);}}while(0)
static void reference(Point f,Point t,double u,double& q,double& v) {
    double d=double(t.q)-f.q;
    double c[6]={double(f.q),f.v*.02,f.a*.0002,
      10*d-.12*f.v-.08*t.v-.0006*f.a+.0002*t.a,
      -15*d+.16*f.v+.14*t.v+.0006*f.a-.0004*t.a,
      6*d-.06*f.v-.06*t.v-.0002*f.a+.0002*t.a};
    q=c[5];for(int i=4;i>=0;--i)q=q*u+c[i];
    v=5*c[5];for(int i=3;i>=0;--i)v=v*u+(i+1)*c[i+1];v*=50;
}
int main() {
    std::mt19937 rng(20261012);
    double maxQ=0,maxV=0;unsigned curves=0;
    for(unsigned test=0;test<20000;++test) {
        Point f{int32_t(rng()%2000000000)-1000000000,int32_t(rng()%100001)-50000,int32_t(rng()%100001)-50000};
        Point t{f.q+int32_t(rng()%1001)-500,int32_t(rng()%100001)-50000,int32_t(rng()%100001)-50000};
        Curve c;
        if(!c.Build(f,t,1536000,5120000))continue;
        ++curves;int32_t q=0,v=0;
        for(unsigned tick=1;tick<=400;++tick) {
            c.Tick(q,v);double rq,rv;reference(f,t,tick/400.,rq,rv);
            maxQ=fmax(maxQ,fabs(q-rq));maxV=fmax(maxV,fabs(v-rv));
            CHECK(fabs(q-rq)<.59);CHECK(fabs(v-rv)<.501);
            CHECK(c.Finished()==(tick==400));
        }
        CHECK(q==t.q && v==t.v);c.Tick(q,v);CHECK(q==t.q && v==t.v);
    }
    CHECK(curves>100);
    Curve c;
    CHECK(!c.Build({0,0,0},{INT32_MAX,0,0},1536000,5120000));
    CHECK(!c.Build({INT32_MAX-1,0,0},{INT32_MAX-1,0,0},1536000,5120000,5));
    CHECK(!c.Build({INT32_MAX-1,10000,0},{INT32_MAX-1,-10000,0},1536000,5120000,-10000));
    CHECK(!c.Build({INT32_MIN+1,-10000,0},{INT32_MIN+1,10000,0},1536000,5120000,10000));
    CHECK(!c.Build({0,1536001,0},{0,0,0},1536000,5120000));
    CHECK(!c.Build({0,0,0},{0,0,0},0,5120000));
    CHECK(c.Build({INT32_MIN,0,0},{INT32_MIN,0,0},1536000,5120000));
    Runtime r;r.Clear();r.state=STATE_STARTED;r.armed=true;r.anchor=2000;r.lastSyncTick=2000;
    CHECK(r.slots[1].curve.Build({0,0,0},{10,0,0},1536000,5120000));
    r.slots[1].state=2;r.slots[1].sequence=1;
    for(r.ticks=0;r.ticks<=2000;++r.ticks)r.Tick();
    CHECK(r.position==0 && r.completed==0);
    for(r.ticks=2001;r.ticks<=2400;++r.ticks)r.Tick();
    CHECK(r.completed==1 && r.position==10 && r.Free()==8);
    r.Tick();CHECK(r.state==STATE_FAULT && r.reason==UNDERRUN && r.stopRequired);
    r.Clear();CHECK(!r.armed && r.Free()==8 && !r.stopRequired);
    // Exact 8-slot wrap and heartbeat stationary segments must progress.
    r.state=STATE_STARTED;r.armed=true;r.anchor=0;r.lastSyncTick=0;
    for(unsigned i=1;i<=8;++i){auto&s=r.slots[i%8];CHECK(s.curve.Build({0,0,0},{0,0,0},1536000,5120000));s.sequence=i;s.state=2;s.flags=i==8?END:0;}
    CHECK(r.Free()==0);
    for(r.ticks=1;r.ticks<=3200;++r.ticks){if(r.ticks%400==0)r.lastSyncTick=r.ticks;r.Tick();}
    CHECK(r.completed==8 && r.terminal && r.Free()==8 && r.position==0 && r.velocity==0);
    r.lastSyncTick=r.ticks;for(unsigned i=0;i<100;++i){++r.ticks;r.Tick();}CHECK(r.reason==OK);
    r.ticks+=2001;r.Tick();CHECK(r.reason==SYNC_TIMEOUT);
    // Wrap-safe scheduling after uint32 clock wraps.
    r.Clear();r.anchor=0xfffffff0U;r.ticks=r.anchor-1;r.lastSyncTick=r.anchor;r.armed=true;r.state=STATE_STARTED;
    CHECK(r.slots[1].curve.Build({0,0,0},{0,0,0},1536000,5120000));r.slots[1].state=2;r.slots[1].sequence=1;r.slots[1].flags=END;
    for(unsigned i=0;i<402;++i){++r.ticks;r.Tick();}CHECK(r.terminal && r.completed==1);
    std::printf("checks=%u curves=%u max_position_error_steps=%.9f max_velocity_error_steps_s=%.9f runtime_bytes=%zu curve_bytes=%zu\n",checks,curves,maxQ,maxV,sizeof(Runtime),sizeof(Curve));
}
