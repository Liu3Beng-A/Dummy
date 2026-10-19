#include "cartesian_cycle_diagnostic.h"
#include <climits>
#include <iostream>
#include <sstream>
// Frozen offline reference port copied for complete-chain parity checks.
// Source e4a61a666d7b producer remains separately compiled and unchanged.
// Offline fixed-work producer experiment. Production IK/transforms and motor
// Curve are linked unchanged; host time is not MCU time, no CAN or PID plant.
#include "bounded_cartesian_follow.h"
#include "cartesian_planner.h"
#include "stream_curve.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <vector>
using namespace BoundedFollow;
using Pose=DOF6Kinematic::Pose6D_t;
constexpr double Pi=3.14159265358979323846;
unsigned assertions=0;
void Require(bool value,const char* text){++assertions;if(!value)throw std::runtime_error(text);}
double SegmentDistance(Vec p,Vec a,Vec b){Vec d=b-a;double dd=Dot(d,d);return Norm(p-(a+d*(dd>1e-16?Clamp(Dot(p-a,d)/dd,0,1):0)));}
Vec XYZ(const Pose& p){return {p.X*1000.,p.Y*1000.,p.Z*1000.};}
struct DoublePose {Vec p{};double r[9]={1,0,0,0,1,0,0,0,1};};
// Independent double DH: separate from production single-precision FK.
DoublePose DoubleFK(const State& state){
    const double home[6]={0,-Pi/2,Pi/2,0,0,0},alpha[6]={-Pi/2,0,Pi/2,-Pi/2,Pi/2,0};
    const double offset[6][3]={{0,-.2073,0},{.170,0,0},{-.0695,0,.117},{0,0,0},{0,0,0},{0,0,.113}};
    DoublePose pose;double p[3]={};
    for(unsigned j=0;j<6;++j){double t=state.q[j]*Pi/180,c=std::cos(t+home[j]),s=std::sin(t+home[j]),
        ca=std::cos(alpha[j]),sa=std::sin(alpha[j]);double local[9]={c,-ca*s,sa*s,s,ca*c,-sa*c,0,sa,ca},next[9]={};
        for(unsigned a=0;a<3;++a)for(unsigned b=0;b<3;++b)for(unsigned k=0;k<3;++k)next[a*3+b]+=pose.r[a*3+k]*local[k*3+b];
        for(unsigned a=0;a<3;++a)for(unsigned b=0;b<3;++b)p[a]+=next[a*3+b]*offset[j][b];
        for(unsigned k=0;k<9;++k)pose.r[k]=next[k];
    }
    for(unsigned a=0;a<3;++a)p[a]+=pose.r[a*3+2]*.114;
    p[1]+=state.q[6]*.001;pose.p={p[0]*1000,p[1]*1000,p[2]*1000};return pose;
}
double RotationError(const DoublePose& a,const DoublePose& b){double tr=0;for(unsigned j=0;j<9;++j)tr+=a.r[j]*b.r[j];return std::acos(Clamp((tr-1)*.5,-1,1))*180/Pi;}
State Fixture(unsigned n){
    const double q[6][7]={{0,-30,80,20,35,0,0},{35,-20,100,-40,50,20,0},
        {-40,10,65,35,-45,-20,0},{-160,-20,100,-40,50,20,0},
        {-160,-68,100,-40,-92,20,0},{35,-25,83,20,.01,-45,0}};
    State s;for(unsigned j=0;j<7;++j)s.q[j]=q[n][j];return s;
}
struct TestPort:Port {
    DOF6Kinematic solver{.2073f,0,.170f,.117f,.0695f,.113f};
    Pose tool=NativeCartesian::FromXYZABC(0,0,.114f,0,0,0),orientation{};
    unsigned ik=0,fk=0,segmentChecks=0,segmentRejected=0;
    unsigned rejectKind=0,rejectCounts[5]{},ikFailureKind=0,ikFailures[6]{};
    StreamProtocol::Record rejectedFrom{},rejectedTo{};
    bool forceIKFailure=false,forceSegmentRejectOnce=false;
    bool Reject(unsigned kind,const StreamProtocol::Record& from,const StreamProtocol::Record& to){
        ++segmentRejected;rejectKind=kind;++rejectCounts[kind];rejectedFrom=from;rejectedTo=to;return false;
    }
    bool IKReject(unsigned kind){ikFailureKind=kind;++ikFailures[kind];return false;}
    Pose Forward(const State& state){
        DOF6Kinematic::Joint6D_t joints;for(unsigned j=0;j<6;++j)joints.a[j]=float(state.q[j]);
        Pose flange;Require(solver.SolveFK(joints,flange),"production FK");++fk;
        Pose tcp=NativeCartesian::Compose(flange,tool);tcp.Y+=float(state.q[6]*.001);return tcp;
    }
    bool Inverse(Vec tcp,double rail,const State& seed,State& goal)override{
        ++ik;ikFailureKind=0;if(forceIKFailure||!Finite(tcp.x)||!Finite(tcp.y)||!Finite(tcp.z))return IKReject(1);
        Pose target=orientation;target.X=float(tcp.x*.001);target.Y=float((tcp.y-rail)*.001);target.Z=float(tcp.z*.001);
        Pose flange=NativeCartesian::Compose(target,NativeCartesian::Inverse(tool));
        DOF6Kinematic::Joint6D_t joints;for(unsigned j=0;j<6;++j)joints.a[j]=float(seed.q[j]);
        DOF6Kinematic::IKSolves_t solutions{};
        if(!solver.SolveIKMeters(flange,joints,solutions))return IKReject(2);
        bool found=false,legalBranch=false;double best=1e100;
        for(unsigned branch=0;branch<8;++branch){
            if(!DOF6Kinematic::IsSolutionValid(solutions,branch))continue;
            State candidate{};candidate.q[6]=rail;double score=0,maxDelta=0;bool legal=true;
            for(unsigned j=0;j<6;++j){float q;
                if(!DOF6Kinematic::NearestInLimits(solutions.config[branch].a[j],float(seed.q[j]),float(Lower[j]),float(Upper[j]),q)){legal=false;break;}
                candidate.q[j]=q;const double delta=q-seed.q[j];score+=delta*delta;maxDelta=std::fmax(maxDelta,std::fabs(delta));
            }
            legalBranch=legalBranch||legal;
            if(legal&&maxDelta<20&&(!found||score<best)){goal=candidate;best=score;found=true;}
        }
        if(!found)return IKReject(legalBranch?4:3);
        if(found){Pose checked=Forward(goal);if(Norm(XYZ(checked)-tcp)>.11)return IKReject(5);}
        return found;
    }
    bool Segment(const StreamProtocol::Record& from,const StreamProtocol::Record& to)override{
        ++segmentChecks;const State a=Decode(from),b=Decode(to);Polynomial segment;segment.Build(a,b,Dt);
        if(forceSegmentRejectOnce){forceSegmentRejectOnce=false;return Reject(4,from,to);}
        if(!segment.Bounded(1))return Reject(1,from,to);
        uint8_t wire[StreamProtocol::RECORD_SIZE];StreamProtocol::Record roundtrip;
        StreamProtocol::EncodeRecord(to,wire);
        if(!StreamProtocol::DecodeRecord(wire,sizeof(wire),roundtrip))return Reject(2,from,to);
        for(unsigned j=0;j<7;++j)if(roundtrip.axes[j].q!=to.axes[j].q||roundtrip.axes[j].v!=to.axes[j].v||roundtrip.axes[j].a!=to.axes[j].a)return Reject(2,from,to);
        for(unsigned j=0;j<7;++j){MotorStream::Curve curve;
            const auto& p=from.axes[j];const auto& q=to.axes[j];
            if(!curve.Build({p.q,p.v,p.a},{q.q,q.v,q.a},int32_t(std::floor(MaxV(j)*std::fabs(Factor[j]))),int32_t(std::floor(MaxA(j)*std::fabs(Factor[j])))))return Reject(2,from,to);
        }
        // Bounded local geometry sampling, not an all-time geometric proof.
        const Vec x=XYZ(Forward(a)),y=XYZ(Forward(b));
        for(double t:{Dt*.25,Dt*.5,Dt*.75}){
            const Pose p=Forward(segment.At(t));double rotation=0;
            for(unsigned k=0;k<9;++k)rotation+=p.R[k]*orientation.R[k];
            if(SegmentDistance(XYZ(p),x,y)>.1||std::acos(Clamp((rotation-1)*.5,-1,1))*180/Pi>.5)return Reject(3,from,to);
        }
        return true;
    }
};

struct Case {const char* name;bool joystick=false,rail=false;double release=.8,turn=-1;};
Input InputAt(const Case& test,Vec start,double t){
    Input in;in.target=start;in.joystick=test.joystick;in.automaticRail=test.rail;
    const std::string name=test.name;
    if(test.joystick){
        in.generation=t<test.turn?1:t<test.release?2:3;
        if(t<test.release){
            if(name=="joystick_reverse")in.velocity=t<test.turn?Vec{30,0,0}:Vec{-30,0,0};
            else if(name=="joystick_xyz")in.velocity=t<test.turn?Vec{25,10,-5}:Vec{-15,20,8};
            else if(name=="joystick_corner")in.velocity=t<test.turn?Vec{30,0,0}:Vec{0,25,10};
            else in.velocity={0,25,0};
        }
    }else if(name=="line_x")in.target=start+Vec{6,0,0};
    else if(name=="line_xyz")in.target=start+Vec{6,3,-2};
    else if(name=="slider_burst"){
        unsigned k=unsigned(std::floor(std::fmin(t,.6)/.04+1e-7));in.generation=k+1;
        in.target=start+Vec{2+6*std::sin(k*.6),1.5*std::sin(k*.3),0};
    }else if(name=="slider_smooth"){
        unsigned k=unsigned(std::floor(std::fmin(t,.8)/Dt+1e-7));in.generation=k+1;
        in.target=start+Vec{8*std::sin(k*Dt*5),3*std::sin(k*Dt*3),-2*std::sin(k*Dt*4)};
    }else if(name=="slider_release"){
        unsigned k=unsigned(std::floor(std::fmin(t,.6)/Dt+1e-7));in.generation=k+1;
        in.target=start+Vec{6+4*std::sin(k*Dt*7),0,0};
    }else if(name=="rail_line")in.target=start+Vec{0,60,0};
    else if(name=="unreachable")in.target=start+Vec{1000,0,0};
    else if(name=="ik_loss"){
        in.target=start+Vec{t<.3?8.0:-5.0,0,0};in.generation=t<.3?1:2;
    }else if(name=="invalid_input"){
        in.target=start+Vec{t<.3?8.0:0.0,0,0};in.generation=t<.3?1:2;
        if(t>=.3)in.target.x=std::numeric_limits<double>::quiet_NaN();
    }
    return in;
}
struct Clock {
    uint32_t cycle=0,increment=0,tick=0,checks=0,stopAt=0;
    uint8_t condition=CartBench::CANCELLED;
    static uint32_t Cycles(void* p){auto& c=*static_cast<Clock*>(p);c.cycle+=c.increment;return c.cycle;}
    static uint32_t Ticks(void* p){return static_cast<Clock*>(p)->tick;}
    static uint8_t Check(void* p){auto& c=*static_cast<Clock*>(p);return c.stopAt&&++c.checks>=c.stopAt?c.condition:CartBench::COMPLETE;}
    CartBench::Hooks Hooks(){return {this,Cycles,Ticks,Check,1000000,nullptr};}
};
bool SameAxes(const StreamProtocol::Axis* a,const StreamProtocol::Axis* b){
    for(unsigned j=0;j<7;++j)if(a[j].q!=b[j].q||a[j].v!=b[j].v||a[j].a!=b[j].a)return false;return true;
}
Case Scenario(unsigned n){
    static const Case cases[]={{"hold"},{"line_x"},{"line_xyz"},{"slider_burst"},{"slider_smooth"},
        {"joystick_reverse",true,false,.8,.4},{"joystick_xyz",true,false,.8,.4},
        {"rail_line",false,true,0,-1},{"rail_recovery",false,true,0,-1},
        {"ik_loss"},{"ik_loss"},{"unreachable",false,true,0,-1},{"invalid_input"}};
    return cases[n];
}
unsigned admissionChecks=0,tickChecks=0,nonzeroFaults=0,singleRejects=0,maxIK=0,maxFK=0;
void AdmissionChecks(){
    uint32_t random=0x12345678u;
    auto next=[&](){random^=random<<13;random^=random>>17;random^=random<<5;return random;};
    for(unsigned n=0;n<12000;++n){
        StreamProtocol::Axis a{},b{};
        int32_t vl=300000,al=1000000,home=0;
        if(n%3==0){a={int32_t(next()%20000000)-10000000,int32_t(next()%10000)-5000,0};b={a.q+int32_t(std::round(a.v*.02)),a.v,0};}
        else if(n%3==1){a={int32_t(next()),int32_t(next()),int32_t(next())};b={int32_t(next()),int32_t(next()),int32_t(next())};}
        else {a={int32_t(next()%20000000)-10000000,0,0};b=a;home=int32_t(next());}
        MotorStream::Curve curve;
        const bool expected=curve.Build({a.q,a.v,a.a},{b.q,b.v,b.a},vl,al,home);
        Require(expected==CartCycle::AdmitAxis(a,b,vl,al,home),"exact integer admission parity");++admissionChecks;
        if(expected){int32_t q=0,v=0;for(unsigned tick=0;tick<400;++tick)curve.Tick(q,v);
            Require(q==b.q&&v==b.v,"motor exact400 tick endpoint");++tickChecks;}
    }
    StreamProtocol::Axis a{0,0,0};
    Require(!CartCycle::AdmitAxis(a,a,0,1),"zero velocity limit rejected");
    Require(!CartCycle::AdmitAxis(a,a,1536001,1),"maximum velocity protocol bound");
    Require(!CartCycle::AdmitAxis(a,a,1,5120001),"maximum acceleration protocol bound");
    Require(!CartCycle::AdmitAxis({INT32_MAX,0,0},{INT32_MAX,0,0},1,1,1),"absolute home integer overflow");
}
void Boundaries(){
    CartCycle::Config c;CartCycle::Compute compute;Clock clock;
    Require(sizeof(CartCycle::Frame)==148,"frame fixed148 bytes");
    Require(sizeof(compute)<2048,"fixed compute state under2KiB");
    Require(compute.Prepare(c),"default prepare");
    for(unsigned field=0;field<6;++field){CartCycle::Config bad=c;
        switch(field){case 0:bad.scenario=13;break;case 1:bad.fixture=6;break;case 2:bad.variant=0;break;
            case 3:bad.samples=0;break;case 4:bad.samples=161;break;default:bad.budgetUs=19999;break;}
        Require(!compute.Prepare(bad),"configuration rejected");}
    c.scenario=CartCycle::LINE_X;
    for(uint8_t condition:{uint8_t(CartBench::CANCELLED),uint8_t(CartBench::ABORTED)}){
        for(unsigned warm:{0u,15u}){
            Require(compute.Prepare(c),"checkpoint count prepare");clock={};
            for(unsigned n=0;n<warm;++n)compute.Measure(clock.Hooks());
            clock.stopAt=UINT32_MAX;compute.Measure(clock.Hooks());const unsigned checkpoints=clock.checks;
            Require(checkpoints>10,"full work has fine-grained checkpoints");
            for(unsigned point=1;point<=checkpoints;++point){
                Require(compute.Prepare(c),"cancel prepare");clock={};
                for(unsigned n=0;n<warm;++n)compute.Measure(clock.Hooks());
                clock.stopAt=point;clock.condition=condition;
                const auto before=compute.Last();const auto f=compute.Measure(clock.Hooks());
                Require((f.flags&(condition==CartBench::CANCELLED?CartCycle::CANCELLED:CartCycle::ABORTED))!=0,"every checkpoint stop classified");
                Require(f.ik<=5,"stopped IK remains bounded");
                Require(!(f.flags&CartCycle::EMITTED)&&!(f.flags&CartCycle::TCP_VALID),"uncommitted endpoint and TCP not presented as valid");
                Require(SameAxes(f.axes,before.axes)&&SameAxes(compute.Last().axes,before.axes),"late stop retains exact last committed q/v/a");
                const auto repeat=compute.Measure(clock.Hooks());
                Require((repeat.flags&CartCycle::FATAL)&&repeat.reason==CartCycle::NOT_PREPARED,"stopped computation cannot silently resume");
                Require(SameAxes(repeat.axes,before.axes),"resume rejection does not zero nonzero old v/a");
            }
        }
    }
    Require(compute.Prepare(c),"budget prepare");clock={};clock.increment=3000;
    const auto over=compute.Measure(clock.Hooks());
    Require((over.flags&CartCycle::BUDGET)!=0,"synthetic elapsed budget cutoff");
    Require(over.cycles>20000,"budget measured from full input chain origin");
    Require(over.ik<=5,"budget failed IK bounded");
    auto h=clock.Hooks();h.clockHz=0;const auto invalid=compute.Measure(h);
    Require((invalid.flags&CartCycle::FATAL)&&invalid.reason==CartCycle::NOT_PREPARED,"invalid environment rejected");
}
void Prefix(unsigned scenario,unsigned fixture,unsigned variant,std::ostream& rows,std::ostream& frames){
    CartCycle::Config config;config.scenario=uint8_t(scenario);config.fixture=uint8_t(fixture);config.variant=uint8_t(variant);config.samples=160;
    CartCycle::Compute backend;Require(backend.Prepare(config),"all synthetic fixtures prepare");
    TestPort reference;State initial=Fixture(fixture);StreamProtocol::Record seed;
    Require(Encode(initial,0,seed),"reference seed");initial=Decode(seed);
    reference.orientation=reference.Forward(initial);const Vec start=DoubleFK(initial).p;
    Producer producer(reference,initial,start,static_cast<Producer::Variant>(variant));
    Clock clock;unsigned count=0,emitted=0,accepted=0,fallback=0,fatal=0,ik=0,fk=0,forced=0;
    const Case test=Scenario(scenario);
    for(unsigned n=0;n<config.samples;++n){
        const double t=n*Dt;Input in=InputAt(test,start,t);
        reference.forceIKFailure=scenario==CartCycle::IK_LOSS&&n>=15;
        reference.forceSegmentRejectOnce=scenario==CartCycle::SEG_REJECT&&n==15;
        const auto before=producer.Last();const unsigned previousIK=reference.ik;
        StreamProtocol::Record out;const Observation observation=producer.Update(in,out);
        const auto frame=backend.Measure(clock.Hooks());++count;
        Require(frame.ik==reference.ik-previousIK,"real backend external IK matches frozen prefix");
        Require(frame.ik<=5,"all searches/retries share five IK");
        Require(bool(frame.flags&CartCycle::FATAL)==observation.fatal,"fatal status matches frozen prefix");
        Require(bool(frame.flags&CartCycle::PLANNED)==observation.planned,"planning cache matches frozen prefix");
        Require(bool(frame.flags&CartCycle::ACCEPTED)==observation.acceptedInput,"generation acceptance matches frozen prefix");
        Require(bool(frame.flags&CartCycle::FALLBACK)==observation.fallback,"fallback matches frozen prefix");
        Require(bool(frame.flags&CartCycle::BRAKING)==observation.braking,"braking matches frozen prefix");
        Require(SameAxes(frame.axes,producer.Last().axes),"previous quantized integer endpoints match original");
        Require(SameAxes(backend.Last().axes,producer.Last().axes),"no state reset on rejection or fatal");
        const bool didEmit=producer.Last().sequence!=before.sequence;
        Require(bool(frame.flags&CartCycle::EMITTED)==didEmit,"emitted classification");
        if(didEmit){
            uint8_t wire[StreamProtocol::RECORD_SIZE];StreamProtocol::Record expected=producer.Last(),decoded;
            StreamProtocol::EncodeRecord(expected,wire);Require(StreamProtocol::DecodeRecord(wire,sizeof(wire),decoded),"92 byte protocol CRC and flags");
            uint32_t hash=2166136261u;for(uint8_t byte:wire){hash^=byte;hash*=16777619u;}
            Require(frame.checksum==hash,"full92 byte wire checksum");
            for(unsigned j=0;j<7;++j){MotorStream::Curve motor;const auto& a=before.axes[j];const auto& b=frame.axes[j];
                Require(motor.Build({a.q,a.v,a.a},{b.q,b.v,b.a},int32_t(std::floor(MaxV(j)*std::fabs(Factor[j]))),
                    int32_t(std::floor(MaxA(j)*std::fabs(Factor[j])))),"all emitted segments admitted by unchanged motor");
                if(n==0||n==15||n==159){int32_t q,v;for(unsigned k=0;k<400;++k)motor.Tick(q,v);
                    Require(q==b.q&&v==b.v,"emitted segment400 tick exact end");++tickChecks;}}
            ++emitted;
        }
        if(n==15&&(scenario==CartCycle::IK_LOSS||scenario==CartCycle::SEG_REJECT||scenario==CartCycle::INVALID_INPUT)){
            bool moving=false;for(const auto& a:before.axes)moving=moving||a.v!=0||a.a!=0;
            nonzeroFaults+=moving;
            if(scenario==CartCycle::SEG_REJECT&&moving){Require(frame.flags&CartCycle::INJECTED,"single rejection motion injection");++singleRejects;}
        }
        accepted+=bool(frame.flags&CartCycle::ACCEPTED);fallback+=bool(frame.flags&CartCycle::FALLBACK);fatal+=bool(frame.flags&CartCycle::FATAL);
        forced+=bool(frame.flags&CartCycle::INJECTED);ik+=frame.ik;fk+=frame.fk;
        maxIK=std::max(maxIK,unsigned(frame.ik));maxFK=std::max(maxFK,unsigned(frame.fk));
        frames<<scenario<<','<<fixture<<','<<variant<<','<<n<<','<<frame.ik<<','<<frame.fk<<','<<frame.reason<<','<<frame.flags<<','<<frame.checksum;
        for(const auto& axis:frame.axes)frames<<','<<axis.q<<','<<axis.v<<','<<axis.a;frames<<'\n';
        if(observation.fatal)break;
    }
    Require(backend.Totals().updates==count&&backend.Totals().emitted==emitted,"fixed summary conservation");
    rows<<"{\"scenario\":"<<scenario<<",\"fixture\":"<<fixture<<",\"variant\":"<<variant<<",\"samples\":"<<count<<",\"emitted\":"<<emitted
        <<",\"accepted\":"<<accepted<<",\"fallback\":"<<fallback<<",\"fatal\":"<<fatal<<",\"injected\":"<<forced<<",\"ik\":"<<ik<<",\"fk\":"<<fk<<"}";
}
int main(){
    try{
        AdmissionChecks();Boundaries();
        std::ofstream frames("frames.csv");frames<<"scenario,fixture,variant,index,ik,fk,reason,flags,checksum";
        for(unsigned j=0;j<7;++j)frames<<",q"<<j<<",v"<<j<<",a"<<j;frames<<'\n';
        std::ostringstream rows;bool comma=false;
        for(unsigned scenario=0;scenario<CartCycle::SCENARIO_COUNT;++scenario)for(unsigned fixture=0;fixture<6;++fixture)for(unsigned variant=1;variant<=3;++variant){
            if(comma)rows<<',';comma=true;Prefix(scenario,fixture,variant,rows,frames);}
        Require(nonzeroFaults>0&&singleRejects>0,"faults exercised at nonzero q/v/a without synthetic reset");
        Require(maxIK==5,"five candidate shared budget exercised");
        std::cout<<"{\"failures\":0,\"assertions\":"<<assertions<<",\"producer_size\":"<<sizeof(Producer)<<",\"backend_size\":"<<sizeof(CartCycle::Compute)
            <<",\"frame_size\":"<<sizeof(CartCycle::Frame)<<",\"ik_peak\":"<<maxIK<<",\"fk_peak\":"<<maxFK<<",\"admission_parity\":"<<admissionChecks
            <<",\"motor_400tick_checks\":"<<tickChecks<<",\"nonzero_faults\":"<<nonzeroFaults<<",\"single_motion_rejects\":"<<singleRejects
            <<",\"hardware\":false,\"f405_timing\":false,\"rows\":["<<rows.str()<<"]}\n";return 0;
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';std::cout<<"{\"failures\":1,\"assertions\":"<<assertions<<",\"rows\":[]}\n";return 1;}
}
