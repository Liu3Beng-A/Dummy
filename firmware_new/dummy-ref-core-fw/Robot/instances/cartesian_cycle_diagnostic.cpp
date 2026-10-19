#include "cartesian_cycle_diagnostic.h"
#include <climits>
#include <cmath>
#include <cstring>

namespace CartCycle {
namespace {
using namespace CartCycleKernel;
constexpr double Pi=3.14159265358979323846;
State Fixture(unsigned n){
    static constexpr double q[6][7]={{0,-30,80,20,35,0,0},{35,-20,100,-40,50,20,0},
        {-40,10,65,35,-45,-20,0},{-160,-20,100,-40,50,20,0},
        {-160,-68,100,-40,-92,20,0},{35,-25,83,20,.01,-45,0}};
    State s;for(unsigned j=0;j<7;++j)s.q[j]=q[n][j];return s;
}
Vec XYZ(const DOF6Kinematic::Pose6D_t& p){return {p.X*1000.,p.Y*1000.,p.Z*1000.};}
double SegmentDistance(Vec p,Vec a,Vec b){Vec d=b-a;const double dd=Dot(d,d);
    return Norm(p-(a+d*(dd>1e-16?Clamp(Dot(p-a,d)/dd,0,1):0)));}
// Preserve the offline candidate's double-DH initial Cartesian anchor. Runtime
// solves, residuals and local geometry all use the unchanged production FK.
Vec InitialAnchor(const State& state){
    static constexpr double home[6]={0,-Pi/2,Pi/2,0,0,0},alpha[6]={-Pi/2,0,Pi/2,-Pi/2,Pi/2,0};
    static constexpr double offset[6][3]={{0,-.2073,0},{.170,0,0},{-.0695,0,.117},{0,0,0},{0,0,0},{0,0,.113}};
    double r[9]={1,0,0,0,1,0,0,0,1},p[3]={};
    for(unsigned j=0;j<6;++j){const double t=state.q[j]*Pi/180,c=std::cos(t+home[j]),s=std::sin(t+home[j]),
        ca=std::cos(alpha[j]),sa=std::sin(alpha[j]);const double local[9]={c,-ca*s,sa*s,s,ca*c,-sa*c,0,sa,ca};double next[9]={};
        for(unsigned a=0;a<3;++a)for(unsigned b=0;b<3;++b)for(unsigned k=0;k<3;++k)next[a*3+b]+=r[a*3+k]*local[k*3+b];
        for(unsigned a=0;a<3;++a)for(unsigned b=0;b<3;++b)p[a]+=next[a*3+b]*offset[j][b];
        for(unsigned k=0;k<9;++k)r[k]=next[k];
    }
    for(unsigned a=0;a<3;++a)p[a]+=r[a*3+2]*.114;
    p[1]+=state.q[6]*.001;return {p[0]*1000,p[1]*1000,p[2]*1000};
}
bool PoseFinite(const DOF6Kinematic::Pose6D_t& p){
    if(!Finite(p.X)||!Finite(p.Y)||!Finite(p.Z))return false;
    for(unsigned j=0;j<9;++j)if(!Finite(p.R[j]))return false;return true;
}
}

bool Valid(const Config& c){return c.scenario<SCENARIO_COUNT&&c.fixture<6&&c.variant>=1&&c.variant<=3&&
    c.samples>=1&&c.samples<=MaxSamples&&c.budgetUs==20000;}

bool AdmitAxis(const StreamProtocol::Axis& from,const StreamProtocol::Axis& to,
               int32_t velocityLimit,int32_t accelerationLimit,int32_t home){
    if(velocityLimit<=0||velocityLimit>1536000||accelerationLimit<=0||accelerationLimit>5120000)return false;
    const int64_t d=int64_t(to.q)-from.q;
    const int64_t b[6]={0,int64_t(200)*from.v,int64_t(400)*from.v+from.a,
        50000*d-int64_t(400)*to.v+to.a,50000*d-int64_t(200)*to.v,50000*d};
    for(unsigned i=0;i<6;++i){
        const int64_t relative=int64_t(from.q)*50000+b[i],absolute=relative+int64_t(home)*50000;
        if(relative<int64_t(INT32_MIN)*50000||relative>int64_t(INT32_MAX)*50000||
           absolute<int64_t(INT32_MIN)*50000||absolute>int64_t(INT32_MAX)*50000||
           b[i]<-int64_t(40000)*50000||b[i]>int64_t(40000)*50000)return false;
        if(i<5){const int64_t delta=b[i+1]-b[i],bound=int64_t(velocityLimit)*200;if(delta < -bound||delta > bound)return false;}
        if(i<4){const int64_t delta=b[i+2]-2*b[i+1]+b[i];if(delta < -int64_t(accelerationLimit)||delta > accelerationLimit)return false;}
    }
    return true;
}

Compute::Compute():solver(.2073f,0,.170f,.117f,.0695f,.113f){
    State state{};producer=new(producerMemory) CartCycleKernel::Producer(*this,state,{},CartCycleKernel::Producer::BOUNDED_CHORD);
}
bool Compute::Prepare(const Config& c){
    prepared=false;hooks=nullptr;summary={};index=0;firstReason=OK;stopFlags=0;currentIK=currentFK=0;
    forceIK=false;rejectOnce=false;if(!Valid(c))return false;config=c;
    tool=NativeCartesian::FromXYZABC(0,0,.114f,0,0,0);
    State initial=Fixture(c.fixture);StreamProtocol::Record seed{};
    if(!CartCycleKernel::Encode(initial,0,seed))return false;
    initial=CartCycleKernel::Decode(seed);
    if(!Forward(initial,orientation))return false;
    start=InitialAnchor(initial);
    producer->~Producer();
    producer=new(producerMemory) CartCycleKernel::Producer(*this,initial,start,
        static_cast<CartCycleKernel::Producer::Variant>(c.variant));
    committed=producer->Last();currentFK=0;prepared=!producer->Failed();return prepared;
}
bool Compute::Fail(Reason reason){if(firstReason==OK)firstReason=reason;return false;}
bool Compute::Check(){
    if(stopFlags)return false;
    if(!hooks)return true; // Prepare uses only synthetic initialization.
    const uint8_t condition=hooks->checkpoint(hooks->context);
    if(condition==CartBench::CANCELLED){stopFlags=CANCELLED;return Fail(CANCEL);}
    if(condition!=CartBench::COMPLETE){stopFlags=ABORTED;return Fail(ENVIRONMENT);}
    const uint32_t elapsed=hooks->cycles(hooks->context)-beginCycles;
    if(uint64_t(elapsed)*1000000u>uint64_t(config.budgetUs)*hooks->clockHz){stopFlags=BUDGET;return Fail(TIME_BUDGET);}
    return true;
}
bool Compute::Forward(const State& state,Pose& tcp){
    if(!Check())return false;
    DOF6Kinematic::Joint6D_t joints;
    for(unsigned j=0;j<6;++j){if(!CartCycleKernel::Finite(state.q[j]))return Fail(INVALID);joints.a[j]=float(state.q[j]);}
    Pose flange;++currentFK;
    if(!solver.SolveFK(joints,flange))return Fail(GEOMETRY);
    tcp=NativeCartesian::Compose(flange,tool);tcp.Y+=float(state.q[6]*.001);
    if(!PoseFinite(tcp))return Fail(GEOMETRY);
    return Check();
}
bool Compute::Inverse(Vec tcp,double rail,const State& seed,State& goal){
    using namespace CartCycleKernel;
    if(!Check())return false;++currentIK;
    if(forceIK)return Fail(INJECTED_IK);
    if(!Finite(tcp.x)||!Finite(tcp.y)||!Finite(tcp.z)||!Finite(rail))return Fail(INVALID);
    Pose target=orientation;target.X=float(tcp.x*.001);target.Y=float((tcp.y-rail)*.001);target.Z=float(tcp.z*.001);
    const Pose flange=NativeCartesian::Compose(target,NativeCartesian::Inverse(tool));
    DOF6Kinematic::Joint6D_t joints;for(unsigned j=0;j<6;++j)joints.a[j]=float(seed.q[j]);
    DOF6Kinematic::IKSolves_t solutions{};
    if(!solver.SolveIKMeters(flange,joints,solutions)||!Check())return Fail(IK);
    bool found=false;double best=1e100;
    for(unsigned branch=0;branch<8;++branch){
        if(!DOF6Kinematic::IsSolutionValid(solutions,branch))continue;
        State candidate{};candidate.q[6]=rail;double score=0,maxDelta=0;bool legal=true;
        for(unsigned j=0;j<6;++j){float q;
            if(!DOF6Kinematic::NearestInLimits(solutions.config[branch].a[j],float(seed.q[j]),float(Lower[j]),float(Upper[j]),q)){legal=false;break;}
            candidate.q[j]=q;const double delta=q-seed.q[j];score+=delta*delta;maxDelta=std::fmax(maxDelta,std::fabs(delta));
        }
        if(legal&&maxDelta<20&&(!found||score<best)){goal=candidate;best=score;found=true;}
    }
    if(!found)return Fail(IK);
    Pose checked;if(!Forward(goal,checked)||Norm(XYZ(checked)-tcp)>.11)return Fail(IK);
    return Check();
}
bool Compute::Segment(const StreamProtocol::Record& from,const StreamProtocol::Record& to){
    using namespace CartCycleKernel;
    if(!Check())return false;++summary.segments;
    if(rejectOnce){rejectOnce=false;++summary.rejected;return Fail(INJECTED_SEGMENT);}
    const State a=Decode(from),b=Decode(to);Polynomial segment;segment.Build(a,b,Dt);
    if(!segment.Bounded(1)){++summary.rejected;return Fail(DYNAMIC);}
    uint8_t wire[StreamProtocol::RECORD_SIZE];StreamProtocol::Record decoded{};
    StreamProtocol::EncodeRecord(to,wire);
    if(!StreamProtocol::DecodeRecord(wire,sizeof(wire),decoded)){++summary.rejected;return Fail(WIRE);}
    for(unsigned j=0;j<7;++j){
        if(decoded.axes[j].q!=to.axes[j].q||decoded.axes[j].v!=to.axes[j].v||decoded.axes[j].a!=to.axes[j].a){++summary.rejected;return Fail(WIRE);}
        if(!AdmitAxis(from.axes[j],to.axes[j],int32_t(std::floor(MaxV(j)*std::fabs(Factor[j]))),
            int32_t(std::floor(MaxA(j)*std::fabs(Factor[j]))))){++summary.rejected;return Fail(ADMISSION);}
    }
    Pose x,y;if(!Forward(a,x)||!Forward(b,y)){++summary.rejected;return false;}
    for(unsigned k=1;k<=3;++k){
        Pose p;if(!Forward(segment.At(Dt*.25*k),p)){++summary.rejected;return false;}
        double rotation=0;for(unsigned j=0;j<9;++j)rotation+=p.R[j]*orientation.R[j];
        if(SegmentDistance(XYZ(p),XYZ(x),XYZ(y))>.1||std::acos(Clamp((rotation-1)*.5,-1,1))*180/Pi>.5){++summary.rejected;return Fail(GEOMETRY);}
    }
    return Check();
}
CartCycleKernel::Input Compute::InputAt(){
    using namespace CartCycleKernel;
    const double t=index*Dt;Input in;in.target=start;
    in.joystick=config.scenario==JOYSTICK_REVERSE||config.scenario==JOYSTICK_XYZ;
    in.automaticRail=config.scenario==RAIL_LINE||config.scenario==RAIL_RECOVERY||config.scenario==UNREACHABLE;
    switch(config.scenario){
    case LINE_X:in.target=start+Vec{6,0,0};break;
    case LINE_XYZ:in.target=start+Vec{6,3,-2};break;
    case SLIDER_BURST:{const unsigned k=unsigned(std::floor(std::fmin(t,.6)/.04+1e-7));in.generation=k+1;
        in.target=start+Vec{2+6*std::sin(k*.6),1.5*std::sin(k*.3),0};break;}
    case SLIDER_SMOOTH:{const unsigned k=unsigned(std::floor(std::fmin(t,.8)/Dt+1e-7));in.generation=k+1;
        in.target=start+Vec{8*std::sin(k*Dt*5),3*std::sin(k*Dt*3),-2*std::sin(k*Dt*4)};break;}
    case JOYSTICK_REVERSE:case JOYSTICK_XYZ:
        in.generation=t<.4?1:t<.8?2:3;
        if(t<.8)in.velocity=config.scenario==JOYSTICK_REVERSE?(t<.4?Vec{30,0,0}:Vec{-30,0,0}):
            (t<.4?Vec{25,10,-5}:Vec{-15,20,8});break;
    case RAIL_LINE:in.target=start+Vec{0,60,0};break;
    case IK_LOSS:case SEG_REJECT:in.target=start+Vec{t<.3?8.0:-5.0,0,0};in.generation=t<.3?1:2;break;
    case UNREACHABLE:in.target=start+Vec{1000,0,0};break;
    case INVALID_INPUT:in.target=start+Vec{t<.3?8.0:0.0,0,0};in.generation=t<.3?1:2;
        if(t>=.3)in.target.x=std::numeric_limits<double>::quiet_NaN();break;
    default:break;
    }
    return in;
}
Frame Compute::Measure(const CartBench::Hooks& h){
    Frame f{};firstReason=OK;stopFlags=0;currentIK=currentFK=0;hooks=&h;
    if(!prepared||index>=config.samples||!h.cycles||!h.ticks||!h.checkpoint||!h.clockHz){
        f.reason=NOT_PREPARED;f.flags=FATAL;for(unsigned j=0;j<7;++j)f.axes[j]=committed.axes[j];hooks=nullptr;return f;}
    beginCycles=h.cycles(h.context);f.startRtos=h.ticks(h.context);
    const auto before=committed;CartCycleKernel::Observation o{};
    if(Check()){
        const auto in=InputAt();forceIK=config.scenario==IK_LOSS&&index>=15;
        rejectOnce=config.scenario==SEG_REJECT&&index==15;
        StreamProtocol::Record emitted{};o=producer->Update(in,emitted);
    }
    f.flags=(o.planned?PLANNED:0)|(o.acceptedInput?ACCEPTED:0)|(o.fallback?FALLBACK:0)|
        (o.braking?BRAKING:0)|(o.railSearch?RAIL_SEARCH:0)|(o.fatal?FATAL:0);
    if(forceIK||config.scenario==SEG_REJECT&&index==15)f.flags|=INJECTED;
    const auto& last=producer->Last();
    if(last.sequence!=before.sequence)f.flags|=EMITTED;
    if(firstReason==OK&&o.reason)firstReason=uint16_t(o.reason);
    if(o.fatal&&firstReason==OK)firstReason=BRAKE;
    const auto state=CartCycleKernel::Decode(last);Pose actual;
    if(Check()&&Forward(state,actual)){const Vec p=XYZ(actual);f.tcp[0]=float(p.x);f.tcp[1]=float(p.y);f.tcp[2]=float(p.z);f.flags|=TCP_VALID;}
    else if(!stopFlags)f.flags|=FATAL;
    const Vec intent=producer->Intent();f.intent[0]=float(intent.x);f.intent[1]=float(intent.y);f.intent[2]=float(intent.z);
    f.railTarget=float(producer->RailTarget());f.margin=float(CartCycleKernel::Margin(state));
    for(unsigned j=0;j<7;++j)f.axes[j]=last.axes[j];
    if(Check()){
        uint8_t wire[StreamProtocol::RECORD_SIZE];StreamProtocol::Record decoded{};
        StreamProtocol::EncodeRecord(last,wire);uint32_t hash=2166136261u;
        for(unsigned j=0;j<sizeof(wire);++j){hash^=wire[j];hash*=16777619u;}
        f.checksum=hash;
        if(!StreamProtocol::DecodeRecord(wire,sizeof(wire),decoded)){Fail(WIRE);f.flags|=FATAL;}
    }
    const bool complete=Check();
    if(!complete||(f.flags&FATAL)){
        // Update may have advanced its private speculative state before a late
        // cancel/budget checkpoint. No segment was submitted. Keep the last
        // fully checked integer endpoint and require a fresh Prepare.
        f.flags|=stopFlags;f.flags&=uint16_t(~(EMITTED|TCP_VALID));
        for(unsigned j=0;j<7;++j)f.axes[j]=committed.axes[j];
        f.checksum=0;prepared=false;
    }else committed=last;
    f.reason=firstReason;f.ik=currentIK;f.fk=currentFK;
    ++summary.updates;summary.emitted+=(f.flags&EMITTED)!=0;summary.accepted+=(f.flags&ACCEPTED)!=0;
    summary.fallback+=(f.flags&FALLBACK)!=0;summary.braking+=(f.flags&BRAKING)!=0;summary.fatal+=(f.flags&FATAL)!=0;
    summary.ik+=f.ik;summary.fk+=f.fk;summary.curveTrials+=o.curveTrials;summary.brakeTrials+=o.brakeTrials;
    if(f.ik>summary.ikPeak)summary.ikPeak=f.ik;if(f.fk>summary.fkPeak)summary.fkPeak=f.fk;
    if(summary.firstReason==OK&&f.reason)summary.firstReason=f.reason;
    ++index;f.endRtos=h.ticks(h.context);f.cycles=h.cycles(h.context)-beginCycles;hooks=nullptr;return f;
}
} // namespace CartCycle
