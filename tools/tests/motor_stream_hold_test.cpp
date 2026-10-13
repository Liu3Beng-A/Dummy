#include "stream_curve.h"
#include "stream_curve_hold_oracle.h"
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <random>

static uint64_t checks=0,acceptedHolds=0,rejectedHolds=0,acceptedGeneral=0;
#define CHECK(x) do {++checks;if(!(x)){std::fprintf(stderr,"FAIL line%u %s\n",__LINE__,#x);std::exit(1);}}while(0)
using Point=MotorStream::Point;
Oracle::Point old(Point p){return {p.q,p.v,p.a};}
void compareTicks(Oracle::Curve& oracle,MotorStream::Curve& candidate,unsigned n=400) {
 for(unsigned tick=0;tick<n;++tick){int32_t oq=0,ov=0,cq=0,cv=0;oracle.Tick(oq,ov);candidate.Tick(cq,cv);CHECK(oq==cq);CHECK(ov==cv);CHECK(oracle.Finished()==candidate.Finished());}
 CHECK(oracle.End().q==candidate.End().q && oracle.End().v==candidate.End().v && oracle.End().a==candidate.End().a);
}
bool compare(Point f,Point t,int32_t vl=1536000,int32_t al=5120000,int32_t home=0,int expected=-1) {
 Oracle::Curve oracle;MotorStream::Curve candidate;
 const bool a=oracle.Build(old(f),old(t),vl,al,home),b=candidate.Build(f,t,vl,al,home);
 CHECK(a==b);if(expected>=0)CHECK(a==bool(expected));
 const bool hold=f.q==t.q && !f.v && !f.a && !t.v && !t.a;
 if(a){if(hold)++acceptedHolds;else++acceptedGeneral;compareTicks(oracle,candidate);compareTicks(oracle,candidate,2);}
 else if(hold)++rejectedHolds;
 return a;
}
int main(){
 const int32_t positions[]={INT32_MIN,INT32_MIN+1,-1000000000,-3,-1,0,1,3,1000000000,INT32_MAX-1,INT32_MAX};
 const int32_t homes[]={INT32_MIN,INT32_MIN+1,-1000000000,-3,-1,0,1,3,1000000000,INT32_MAX-1,INT32_MAX};
 for(int32_t q:positions)for(int32_t home:homes){const int64_t abs=int64_t(q)+home;compare({q,0,0},{q,0,0},1536000,5120000,home,abs>=INT32_MIN && abs<=INT32_MAX);}
 const int32_t speeds[]={INT32_MIN,-1,0,1,512,1536000,1536001,INT32_MAX};
 const int32_t accelerations[]={INT32_MIN,-1,0,1,512,5120000,5120001,INT32_MAX};
 for(int32_t v:speeds)for(int32_t a:accelerations)compare({-3,0,0},{-3,0,0},v,a,5105,v>0&&v<=1536000&&a>0&&a<=5120000);
 std::mt19937 random(20261013);
 for(unsigned i=0;i<12000;++i){int32_t q=int32_t(random()),home=int32_t(random());const int64_t abs=int64_t(q)+home;compare({q,0,0},{q,0,0},1536000,5120000,home,abs>=INT32_MIN&&abs<=INT32_MAX);}
 for(unsigned i=0;i<20000;++i){Point f{int32_t(random()%2000000000)-1000000000,int32_t(random()%100001)-50000,int32_t(random()%100001)-50000};Point t{f.q+int32_t(random()%1001)-500,int32_t(random()%100001)-50000,int32_t(random()%100001)-50000};compare(f,t);}
 // Equal q with nonzero endpoint derivatives must remain on the old route.
 for(int32_t v:{-10000,-1,1,10000}){compare({0,v,0},{0,v,0});compare({0,0,v},{0,0,v});compare({0,v,0},{0,0,0});}
 // Reused curve storage: ordinary->hold->ordinary->extreme hold, including
 // rejected Build while an earlier ordinary curve is still being consumed.
 Oracle::Curve oracle;MotorStream::Curve candidate;
 const Point phases[][2]={{{0,0,0},{10,0,0}},{{10,0,0},{10,0,0}},{{10,0,0},{-10,0,0}},{{INT32_MAX,0,0},{INT32_MAX,0,0}},{{0,0,0},{10,0,0}}};
 for(const auto& phase:phases){CHECK(oracle.Build(old(phase[0]),old(phase[1]),1536000,5120000));CHECK(candidate.Build(phase[0],phase[1],1536000,5120000));compareTicks(oracle,candidate);}
 CHECK(oracle.Build({0,0,0},{10,0,0},1536000,5120000));CHECK(candidate.Build({0,0,0},{10,0,0},1536000,5120000));compareTicks(oracle,candidate,50);
 CHECK(!oracle.Build({INT32_MAX,0,0},{INT32_MAX,0,0},1536000,5120000,1));CHECK(!candidate.Build({INT32_MAX,0,0},{INT32_MAX,0,0},1536000,5120000,1));compareTicks(oracle,candidate,350);
 CHECK(acceptedHolds>8000 && rejectedHolds>2000 && acceptedGeneral>500);
 CHECK(sizeof(oracle)==sizeof(candidate));
 std::printf("checks=%llu accepted_holds=%llu rejected_holds=%llu accepted_general=%llu all_400tick_outputs_exact=PASS curve_bytes=%zu production_curve=YES hardware=NO\n",(unsigned long long)checks,(unsigned long long)acceptedHolds,(unsigned long long)rejectedHolds,(unsigned long long)acceptedGeneral,sizeof(candidate));
}
