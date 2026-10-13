#include "../../firmware_new/stream_protocol.h"
#include "../../firmware_new/motion_completion_protocol.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
using namespace StreamProtocol;
constexpr float RAIL_MM_PER_CIRCLE=5.0f;
struct Motor { bool inverseDirection; int reduction; float angleLimitMin,angleLimitMax,angle; };
struct Pose { float a[6]; };
struct Robot {
    Motor values[7]={{false,50,-175,175,0},{true,50,-75,90,85},{true,50,0,180,-100},
        {true,50,-270,270,0},{true,50,-100,100,90},{true,30,-180,180,0},{true,1,-250,250,0}};
    Motor* motorJ[8]={nullptr,&values[0],&values[1],&values[2],&values[3],&values[4],&values[5],&values[6]};
    Pose initPose{{0,-75,180,0,0,0}};
    float jointAccRuntime[8]={0,30,30,30,30,30,30,30};
} dummy;
static unsigned checks=0;
#define CHECK(x) do {++checks;if(!(x)){fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x);exit(1);}}while(0)
#include "core_stream_geometry_under_test.inc"
static Record Seed() {
    Record r{};r.session=1;r.flags=SEED;
    for(unsigned i=0;i<7;++i) { const auto& m=dummy.values[i];r.axes[i].q=int32_t(lround(double(m.angle)/360*m.reduction*51200*(m.inverseDirection?-1:1))); }
    return r;
}
int main() {
    auto seed=Seed();CHECK(ProductionValidate(seed,nullptr,127));
    CHECK(seed.axes[1].q==-604444);CHECK(seed.axes[2].q==711111);CHECK(seed.axes[4].q==-640000);
    auto next=seed;next.sequence=1;next.flags=END;CHECK(ProductionValidate(next,&seed,127));
    auto wrong=seed;wrong.axes[0].q+=713;CHECK(!ProductionValidate(wrong,nullptr,127));
    wrong=seed;wrong.axes[0].q+=711;CHECK(ProductionValidate(wrong,nullptr,127));
    wrong=seed;wrong.axes[0].v=1;CHECK(!ProductionValidate(wrong,nullptr,127));
    next=seed;next.sequence=1;next.flags=0;next.axes[6].q=-2560001;auto begin=next;
    CHECK(ProductionValidate(next,&begin,127)); // one wire-count endpoint tolerance
    next.axes[6].q=-2560002;CHECK(!ProductionValidate(next,&begin,127));
    next=seed;next.sequence=1;next.flags=0;next.axes[6].q=2560002;begin=next;CHECK(!ProductionValidate(next,&begin,127));
    next=seed;next.sequence=1;next.flags=0;next.axes[0].v=1536001;begin=next;CHECK(!ProductionValidate(next,&begin,127));
    next=seed;next.sequence=1;next.flags=0;next.axes[0].a=1536001;begin=next;CHECK(!ProductionValidate(next,&begin,127));
    // Both endpoints fit the joint range; opposing endpoint velocities create
    // an interior Bezier hull beyond +175 degrees and must still be rejected.
    next=seed;next.sequence=1;next.flags=0;next.axes[0]={1244344,-20000,0};begin=next;begin.axes[0].v=20000;
    CHECK(!ProductionValidate(next,&begin,127));
    next=seed;next.sequence=1;next.flags=END;begin=seed;next.axes[0].q=300;
    CHECK(!ProductionValidate(next,&begin,127)); // 20 ms acceleration envelope
    next.axes[0].q=1;CHECK(ProductionValidate(next,&begin,127));
    next=seed;next.sequence=1;next.flags=0;begin=seed;next.axes[6].q=-1;CHECK(ProductionValidate(next,&begin,127));
    next.axes[0].q=2000000000;CHECK(ProductionValidate(next,&begin,64)); // bench mask never applies unused axes
    CHECK(!ProductionValidate(next,&begin,127));
    printf("core_stream_geometry: %u checks passed; actual production validator\n",checks);
}
