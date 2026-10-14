#include "motion_values.h"
#include "cartesian_planner.h"
#include <string>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
static unsigned checks=0;
#define CHECK(x) do {++checks;if(!(x)){std::fprintf(stderr,"FAIL line%d %s\n",__LINE__,#x);std::exit(1);}}while(0)
static void Bad(const std::string& text,bool cartesian,unsigned min=8,unsigned max=14) {
    float values[20];for(float& v:values)v=777;
    CHECK((cartesian?MotionValues::ParseCartesian(text.data(),text.size(),values,min,max):MotionValues::Parse(text.data(),text.size(),values,min,max))==0);
    for(float v:values)CHECK(v==777);
}
int main() {
    float values[20];for(float& v:values)v=777;
    const std::string legacy="220,20,180,180,0,180,35";
    CHECK(MotionValues::MaxPayload==63);CHECK(MotionValues::Parse(legacy.data(),legacy.size(),values,6,7)==7);
    CHECK(values[0]==220&&values[3]==180&&values[6]==35&&values[7]==777);
    const std::string joints="0,10,80,0,90,0,37.25,35";
    CHECK(MotionValues::Parse(joints.data(),joints.size(),values,7,8)==8);CHECK(values[6]==37.25f);
    const std::string eight="220,20,180,10,20,30,35,1",fourteen="220,20,180,10,20,30,35,1,10,-20,114,8,5,-7";
    CHECK(MotionValues::ParseCartesian(eight.data(),eight.size(),values,8,14)==8);
    CHECK(values[7]==1);CHECK(MotionValues::ParseCartesian(fourteen.data(),fourteen.size(),values,8,14)==14);
    CHECK(values[8]==10&&values[9]==-20&&values[10]==114&&values[13]==-7);
    Bad(fourteen,false,8,14); // Legacy parser's maximum argument cannot expand.
    Bad(fourteen,false,8,8); // Fourteen actual fields cannot pass the old parser.
    for(unsigned i=14;i<20;++i)CHECK(values[i]==777); // No out-of-range write.
    const std::string old63(63-joints.size(),' ');const std::string boundOld=old63+joints;
    CHECK(MotionValues::Parse(boundOld.data(),boundOld.size(),values,7,8)==8);Bad(" "+boundOld,false,7,8);
    for(unsigned length=1;length<=159;++length) {
        if(length<fourteen.size())continue;
        const std::string payload=std::string(length-fourteen.size(),'0')+fourteen;
        CHECK(MotionValues::ParseCartesian(payload.data(),payload.size(),values,8,14)==14);
        CHECK(values[0]==220&&values[13]==-7);
    }
    Bad(std::string(160-fourteen.size(),'0')+fourteen,true);
    for(const char* token:{"nan","NaN","nan(1)","inf","-inf","1e100","1e-100","2garbage","","0x","++1"}) {
        Bad(std::string(token)+",0,0,0,0,0,35,1",true);
        Bad(std::string(token)+",0,0,0,0,0,35",false,6,7);
    }
    for(const std::string& bad:{eight+",",eight+" garbage",eight+",,0",std::string("0 0,0,0,0,0,35,1"),std::string("0,0,0,0,0,35,1")})Bad(bad,true);
    Bad(eight+std::string(1,'\0')+"garbage",true);Bad(legacy+std::string(1,'\0')+",999",false,6,7);
    for(unsigned n=9;n<=14;++n) {std::string s="0";for(unsigned i=1;i<n;++i)s+=",0";CHECK(MotionValues::ParseCartesian(s.data(),s.size(),values,n,n)==n);}
    Bad("0",true,0,1);Bad("0",true,2,1);Bad("0",true,1,15);
    CHECK(MotionValues::ParseCartesian(fourteen.data(),fourteen.size(),values,8,14)==14);
    const auto target=NativeCartesian::FromXYZABC(values[0]*.001f,values[1]*.001f,values[2]*.001f,values[3],values[4],values[5]);
    const auto tool=NativeCartesian::FromXYZABC(values[8]*.001f,values[9]*.001f,values[10]*.001f,values[11],values[12],values[13]);
    CHECK(std::fabs(target.X-.220f)<1e-6f&&std::fabs(target.Y-.020f)<1e-6f&&std::fabs(target.Z-.180f)<1e-6f);
    CHECK(std::fabs(tool.X-.010f)<1e-6f&&std::fabs(tool.Y+.020f)<1e-6f&&std::fabs(tool.Z-.114f)<1e-6f);
    constexpr double rad=3.14159265358979323846/180;
    CHECK(std::fabs(target.R[0]-std::cos(30*rad)*std::cos(20*rad))<1e-6);
    CHECK(std::fabs(target.R[6]+std::sin(20*rad))<1e-6);
    NativeCartesian::Request request;request.goal=target;request.tool=tool;request.speed=values[6];request.automaticRail=values[7]==1;
    values[10]=499;CHECK(request.tool.Z==tool.Z&&request.automaticRail&&request.speed==35);
    const auto identity=NativeCartesian::Compose(target,NativeCartesian::Inverse(target));
    CHECK(std::fabs(identity.X)<1e-6&&std::fabs(identity.Y)<1e-6&&std::fabs(identity.Z)<1e-6);
    for(unsigned i=0;i<9;++i)CHECK(std::fabs(identity.R[i]-(i%4==0?1.0f:0.0f))<1e-6);
    std::printf("cartesian_values: %u checks passed; production Parse/ParseCartesian + Cartesian transforms, no hardware\n",checks);
}
