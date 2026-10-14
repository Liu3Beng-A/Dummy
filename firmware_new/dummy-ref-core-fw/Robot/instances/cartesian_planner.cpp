#include "cartesian_planner.h"
#include <cmath>
#include <cstring>
#include <climits>
#include <initializer_list>
namespace NativeCartesian {
namespace {
constexpr float Pi=3.14159265358979323846f, Rad=Pi/180;
constexpr float PositionTolerance=.001f, RotationTolerance=1.0f*Rad;
constexpr unsigned WorkBudget=180000;
bool Finite(float x){uint32_t u;std::memcpy(&u,&x,4);return (u&0x7f800000U)!=0x7f800000U;}
float Clamp(float x,float lo,float hi){return x<lo?lo:x>hi?hi:x;}
float Smooth(float u){return u*u*u*(10+u*(-15+6*u));}
float SmoothD(float u){return 30*u*u*(1-u)*(1-u);}
float SmoothDD(float u){return 60*u*(1-u)*(1-2*u);}
void Quaternion(const float R[9],float q[4]) {
    float trace=R[0]+R[4]+R[8];
    if(trace>0){float s=2*std::sqrt(trace+1);q[0]=s/4;q[1]=(R[7]-R[5])/s;q[2]=(R[2]-R[6])/s;q[3]=(R[3]-R[1])/s;}
    else {unsigned i=R[4]>R[0]?1:0;if(R[8]>R[i*3+i])i=2;unsigned j=(i+1)%3,k=(i+2)%3;
        float s=2*std::sqrt(1+R[i*3+i]-R[j*3+j]-R[k*3+k]);q[i+1]=s/4;q[0]=(R[k*3+j]-R[j*3+k])/s;
        q[j+1]=(R[j*3+i]+R[i*3+j])/s;q[k+1]=(R[k*3+i]+R[i*3+k])/s;}
    float norm=0;for(float x: {q[0],q[1],q[2],q[3]})norm+=x*x;
    norm=std::sqrt(norm);for(unsigned i=0;i<4;++i)q[i]/=norm;
}
void Rotation(const float q[4],float R[9]) {
    float w=q[0],x=q[1],y=q[2],z=q[3];
    R[0]=1-2*(y*y+z*z);R[1]=2*(x*y-w*z);R[2]=2*(x*z+w*y);
    R[3]=2*(x*y+w*z);R[4]=1-2*(x*x+z*z);R[5]=2*(y*z-w*x);
    R[6]=2*(x*z-w*y);R[7]=2*(y*z+w*x);R[8]=1-2*(x*x+y*y);
}
float OrientationError(const Pose& a,const Pose& b){float sum=0;for(unsigned i=0;i<9;++i)sum+=a.R[i]*b.R[i];return std::acos(Clamp((sum-1)*.5f,-1,1));}
float PositionError(const Pose& a,const Pose& b){float x=a.X-b.X,y=a.Y-b.Y,z=a.Z-b.Z;return std::sqrt(x*x+y*y+z*z);}
void Coeff(float q0,float v0,float a0,float q1,float v1,float a1,float c[6]){
    float d=q1-q0;c[0]=q0;c[1]=v0;c[2]=a0*.5f;
    c[3]=10*d-6*v0-4*v1-1.5f*a0+.5f*a1;
    c[4]=-15*d+8*v0+7*v1+1.5f*a0-a1;
    c[5]=6*d-3*(v0+v1)-.5f*(a0-a1);
}
float Value(const float c[6],float u){return c[0]+u*(c[1]+u*(c[2]+u*(c[3]+u*(c[4]+u*c[5]))));}
void MidSplit(const float b[6],float left[6],float right[6]){
    float row[6];std::memcpy(row,b,sizeof(row));left[0]=row[0];right[5]=row[5];
    for(unsigned n=5;n>0;--n){for(unsigned i=0;i<n;++i)row[i]=(row[i]+row[i+1])*.5f;left[6-n]=row[0];right[n-1]=row[n-1];}
}
int32_t Quantize(float x){return int32_t(x>=0?x+.5f:x-.5f);}
bool Proper(const Pose& p){if(!p.hasR)return false;for(float x:p.R)if(!Finite(x))return false;
    for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j){float v=0;for(unsigned k=0;k<3;++k)v+=p.R[k*3+i]*p.R[k*3+j];if(std::fabs(v-(i==j?1.f:0.f))>.002f)return false;}
    float det=p.R[0]*(p.R[4]*p.R[8]-p.R[5]*p.R[7])-p.R[1]*(p.R[3]*p.R[8]-p.R[5]*p.R[6])+p.R[2]*(p.R[3]*p.R[7]-p.R[4]*p.R[6]);return std::fabs(det-1)<.002f;}
}
const char* Name(Result r){static const char* names[]={"OK","INVALID","NO_IK","JOINT_LIMIT","DISCONTINUITY","GEOMETRY","DYNAMIC_LIMIT","BUDGET","CANCELLED"};return names[unsigned(r)<=CANCELLED?r:INVALID];}
Pose FromXYZABC(float x,float y,float z,float a,float b,float c){
    a=std::remainderf(a,360);b=std::remainderf(b,360);c=std::remainderf(c,360);
    Pose p{x,y,z,a,b,c};float ca=std::cos(a*Rad),sa=std::sin(a*Rad),cb=std::cos(b*Rad),sb=std::sin(b*Rad),cc=std::cos(c*Rad),sc=std::sin(c*Rad);
    p.R[0]=cc*cb;p.R[1]=cc*sb*sa-sc*ca;p.R[2]=cc*sb*ca+sc*sa;
    p.R[3]=sc*cb;p.R[4]=sc*sb*sa+cc*ca;p.R[5]=sc*sb*ca-cc*sa;
    p.R[6]=-sb;p.R[7]=cb*sa;p.R[8]=cb*ca;p.hasR=true;return p;
}
Pose Compose(const Pose& a,const Pose& b){Pose p{};float t[3]={b.X,b.Y,b.Z},base[3]={a.X,a.Y,a.Z},out[3];
    for(unsigned i=0;i<3;++i){out[i]=base[i];for(unsigned k=0;k<3;++k)out[i]+=a.R[3*i+k]*t[k];for(unsigned j=0;j<3;++j){p.R[i*3+j]=0;for(unsigned k=0;k<3;++k)p.R[i*3+j]+=a.R[i*3+k]*b.R[k*3+j];}}
    p.X=out[0];p.Y=out[1];p.Z=out[2];p.hasR=true;return p;}
Pose Inverse(const Pose& p){Pose out{};float t[3]={p.X,p.Y,p.Z},v[3]{};for(unsigned i=0;i<3;++i){for(unsigned j=0;j<3;++j){out.R[i*3+j]=p.R[j*3+i];v[i]-=out.R[i*3+j]*t[j];}}out.X=v[0];out.Y=v[1];out.Z=v[2];out.hasR=true;return out;}
void ToXYZABC(const Pose& p,float out[6]){out[0]=p.X*1000;out[1]=p.Y*1000;out[2]=p.Z*1000;
    out[4]=std::atan2(-p.R[6],std::sqrt(p.R[0]*p.R[0]+p.R[3]*p.R[3]))/Rad;
    if(std::fabs(p.R[6])>.999999f){out[3]=0;out[5]=std::atan2(-p.R[1],p.R[4])/Rad;}
    else{out[3]=std::atan2(p.R[7],p.R[8])/Rad;out[5]=std::atan2(p.R[3],p.R[0])/Rad;}}
bool Planner::TickWork(unsigned amount){work+=amount;if(work>WorkBudget)return false;return progress->Checkpoint();}
Pose Planner::Target(float s) const {
    Pose p{};p.X=begin.X+s*(end.X-begin.X);p.Y=begin.Y+s*(end.Y-begin.Y);p.Z=begin.Z+s*(end.Z-begin.Z);
    float q[4],dot=0;for(unsigned i=0;i<4;++i)dot+=quaternion[0][i]*quaternion[1][i];
    if(dot>.9995f){float n=0;for(unsigned i=0;i<4;++i){q[i]=quaternion[0][i]+s*(quaternion[1][i]-quaternion[0][i]);n+=q[i]*q[i];}n=std::sqrt(n);for(float& v:q)v/=n;}
    else{float theta=std::acos(Clamp(dot,-1,1)),denom=std::sin(theta),w0=std::sin((1-s)*theta)/denom,w1=std::sin(s*theta)/denom;for(unsigned i=0;i<4;++i)q[i]=w0*quaternion[0][i]+w1*quaternion[1][i];}
    Rotation(q,p.R);p.hasR=true;return p;
}
Result Planner::SolvePath(float rail){
    railGoal=rail;std::memcpy(node[0],request.start,sizeof(request.start));
    DOF6Kinematic::Joint6D_t previous{};std::memcpy(previous.a,request.start,sizeof(previous.a));
    const Pose inverseTool=Inverse(request.tool);
    for(unsigned n=1;n<=Intervals;++n){if(!TickWork())return work>WorkBudget?BUDGET:CANCELLED;
        float s=float(n)/Intervals,r=request.start[6]+(rail-request.start[6])*Smooth(s);
        Pose tcp=Target(s);tcp.Y-=r*.001f;Pose flange=Compose(tcp,inverseTool);
        DOF6Kinematic::IKSolves_t solutions{};if(!solver->SolveIKMeters(flange,previous,solutions))return NO_IK;
        float best=1e30f;DOF6Kinematic::Joint6D_t chosen{};bool found=false;
        for(unsigned branch=0;branch<8;++branch){if(!solutions.solFlag[branch][0]||!solutions.solFlag[branch][1]||!solutions.solFlag[branch][2])continue;
            DOF6Kinematic::Joint6D_t candidate{};float score=0;bool valid=true;
            for(unsigned j=0;j<6;++j){if(!DOF6Kinematic::NearestInLimits(solutions.config[branch].a[j],previous.a[j],limits.lower[j],limits.upper[j],candidate.a[j])){valid=false;break;}
                float delta=std::fabs(candidate.a[j]-previous.a[j]);if(delta>score)score=delta;}
            if(valid&&score<best){best=score;chosen=candidate;found=true;}}
        if(!found)return JOINT_LIMIT;
        if(best>12.0f)return DISCONTINUITY;
        std::memcpy(node[n],chosen.a,sizeof(chosen.a));node[n][6]=r;previous=chosen;
    }
    return OK;
}
void Planner::Evaluate(float time,float q[7],float v[7],float a[7]) const {
    float duration=Duration(),u=Clamp(time/duration,0,1),s=Smooth(u),sd=SmoothD(u)/duration,sdd=SmoothDD(u)/(duration*duration);
    float scaled=s*Intervals;unsigned n=unsigned(scaled);if(n>=Intervals)n=Intervals-1;float h=scaled-n;
    for(unsigned j=0;j<7;++j){
        auto derivative=[&](unsigned at,float& d,float& dd){
            if(at==0){d=(-3*node[0][j]+4*node[1][j]-node[2][j])*.5f;dd=node[0][j]-2*node[1][j]+node[2][j];}
            else if(at==Intervals){d=(3*node[at][j]-4*node[at-1][j]+node[at-2][j])*.5f;dd=node[at][j]-2*node[at-1][j]+node[at-2][j];}
            else{d=(node[at+1][j]-node[at-1][j])*.5f;dd=node[at+1][j]-2*node[at][j]+node[at-1][j];}};
        float d0,dd0,d1,dd1,c[6];derivative(n,d0,dd0);derivative(n+1,d1,dd1);Coeff(node[n][j],d0,dd0,node[n+1][j],d1,dd1,c);
        q[j]=Value(c,h);float ds=Intervals*(c[1]+h*(2*c[2]+h*(3*c[3]+h*(4*c[4]+h*5*c[5]))));
        float dds=Intervals*Intervals*(2*c[2]+h*(6*c[3]+h*(12*c[4]+h*20*c[5])));
        v[j]=ds*sd;a[j]=dds*sd*sd+ds*sdd;
        if(time<=0||time>=duration){q[j]=time<=0?node[0][j]:node[Intervals][j];v[j]=a[j]=0;}
    }
}
bool Planner::Record(unsigned sequence,uint16_t session,StreamProtocol::Record& out) const {
    return ready&&GenerateRecord(sequence,session,out);
}
bool Planner::GenerateRecord(unsigned sequence,uint16_t session,StreamProtocol::Record& out) const {
    if(!segments||sequence>segments||!session)return false;
    float q[7],v[7],a[7];Evaluate(sequence*.020f,q,v,a);out={};out.session=session;out.sequence=uint16_t(sequence);
    out.flags=sequence==0?StreamProtocol::SEED:sequence==segments?StreamProtocol::END:0;
    for(unsigned j=0;j<7;++j){float factor=limits.stepsPerUnit[j];float value[3]={(q[j]-limits.origin[j])*factor,v[j]*factor,a[j]*factor};
        for(float x:value)if(!Finite(x)||std::fabs(x)>2147483000.0f)return false;
        out.axes[j]={sequence==0&&request.exactSeed?request.seed[j]:Quantize(value[0]),Quantize(value[1]),Quantize(value[2])};}
    return true;
}
Result Planner::CheckGeometry(const float controls[7][6],float t0,float t1,unsigned depth){
    unsigned pending=1;std::memcpy(geometryStack[0].controls,controls,sizeof(geometryStack[0].controls));geometryStack[0].t0=t0;geometryStack[0].t1=t1;geometryStack[0].depth=depth;
    while(pending){
    const GeometryFrame frame=geometryStack[--pending];t0=frame.t0;t1=frame.t1;depth=frame.depth;
    if(!TickWork())return work>WorkBudget?BUDGET:CANCELLED;
    float mid=(t0+t1)*.5f,sm=Smooth(mid/Duration()),s0=Smooth(t0/Duration()),s1=Smooth(t1/Duration());
    float q[7],left[7][6],right[7][6],range[7];
    for(unsigned j=0;j<7;++j){MidSplit(frame.controls[j],left[j],right[j]);q[j]=left[j][5];range[j]=0;for(unsigned k=0;k<6;++k)range[j]=std::fmax(range[j],std::fabs(frame.controls[j][k]-q[j]));}
    DOF6Kinematic::Joint6D_t joint{};std::memcpy(joint.a,q,sizeof(joint.a));Pose flange{};
    if(!solver->SolveFK(joint,flange))return GEOMETRY;
    Pose tcp=Compose(flange,request.tool);tcp.Y+=q[6]*.001f;Pose wanted=Target(sm);
    float ep=PositionError(tcp,wanted),er=OrientationError(tcp,wanted);
    if(ep>PositionTolerance||er>RotationTolerance)return GEOMETRY;
    // Every rotation's TCP displacement is bounded by its downstream link
    // lengths times angular range. Bernstein hull + FK midpoint certifies all
    // times, rather than assuming endpoint IK proves a straight segment.
    float toolLength=std::sqrt(request.tool.X*request.tool.X+request.tool.Y*request.tool.Y+request.tool.Z*request.tool.Z);
    const float reach[6]={.170f+.136086f+.113f+toolLength,.170f+.136086f+.113f+toolLength,.136086f+.113f+toolLength,.113f+toolLength,.113f+toolLength,toolLength};
    float progressRange=std::fmax(sm-s0,s1-sm),pb=ep+distance*progressRange+range[6]*.001f,rb=er+angle*progressRange;
    // Two encoded steps conservatively cover Q40 preparation/difference and
    // final nearest-integer execution error; END itself is exact.
    pb+=2*.001f/std::fabs(limits.stepsPerUnit[6]);
    for(unsigned j=0;j<6;++j){float variation=range[j]+2/std::fabs(limits.stepsPerUnit[j]);pb+=reach[j]*variation*Rad;rb+=variation*Rad;}
    if(pb<=PositionTolerance&&rb<=RotationTolerance){maxPositionBound=std::fmax(maxPositionBound,pb);maxRotationBound=std::fmax(maxRotationBound,rb);continue;}
    if(depth==9)return GEOMETRY;
    if(pending+2>11)return BUDGET;
    GeometryFrame& r=geometryStack[pending++];std::memcpy(r.controls,right,sizeof(right));r.t0=mid;r.t1=t1;r.depth=depth+1;
    GeometryFrame& l=geometryStack[pending++];std::memcpy(l.controls,left,sizeof(left));l.t0=t0;l.t1=mid;l.depth=depth+1;
    }
    return OK;
}
Result Planner::Validate(){
    StreamProtocol::Record from{},to{};if(!GenerateRecord(0,1,from))return INVALID;
    maxPositionBound=maxRotationBound=0;
    for(unsigned n=1;n<=segments;++n){if(!GenerateRecord(n,1,to))return INVALID;
        float controls[7][6];
        for(unsigned j=0;j<7;++j){const auto& a=from.axes[j];const auto& b=to.axes[j];float factor=limits.stepsPerUnit[j];
            const double p[6]={double(a.q),a.q+a.v*.004,a.q+a.v*.008+a.a*.00002,b.q-b.v*.008+b.a*.00002,b.q-b.v*.004,double(b.q)};
            for(unsigned k=0;k<6;++k){controls[j][k]=float(p[k]/factor+limits.origin[j]);if(controls[j][k]<limits.lower[j]-1e-4f||controls[j][k]>limits.upper[j]+1e-4f)return JOINT_LIMIT;}
            for(unsigned k=0;k<5;++k)if(std::fabs(250*(p[k+1]-p[k])/factor)>limits.velocity[j])return DYNAMIC_LIMIT;
            for(unsigned k=0;k<4;++k)if(std::fabs(50000*(p[k+2]-2*p[k+1]+p[k])/factor)>limits.acceleration[j])return DYNAMIC_LIMIT;
            for(unsigned k=0;k<3;++k)if(std::fabs(7500000*(p[k+3]-3*p[k+2]+3*p[k+1]-p[k])/factor)>limits.jerk[j])return DYNAMIC_LIMIT;
        }
        Result result=CheckGeometry(controls,(n-1)*.020f,n*.020f,0);if(result!=OK)return result;from=to;
    }
    return OK;
}
Result Planner::Build(DOF6Kinematic& inputSolver,const Request& input,const Limits& inputLimits,Progress& inputProgress){
    ready=false;segments=work=0;solver=&inputSolver;progress=&inputProgress;request=input;limits=inputLimits;
    for(unsigned j=0;j<7;++j){if(!Finite(request.start[j])||!Finite(limits.lower[j])||!Finite(limits.upper[j])||limits.lower[j]>=limits.upper[j]||request.start[j]<limits.lower[j]||request.start[j]>limits.upper[j]||!Finite(limits.stepsPerUnit[j])||std::fabs(limits.stepsPerUnit[j])<1||!Finite(limits.origin[j])||!Finite(limits.velocity[j])||!Finite(limits.acceleration[j])||!Finite(limits.jerk[j])||limits.velocity[j]<=0||limits.acceleration[j]<=0||limits.jerk[j]<=0)return INVALID;}
    if(!Proper(request.goal)||!Proper(request.tool)||request.speed<1||request.speed>100)return INVALID;
    for(float x:{request.goal.X,request.goal.Y,request.goal.Z,request.tool.X,request.tool.Y,request.tool.Z,request.speed})if(!Finite(x))return INVALID;
    for(unsigned i=0;i<9;++i)if(!Finite(request.goal.R[i])||!Finite(request.tool.R[i]))return INVALID;
    DOF6Kinematic::Joint6D_t joint{};std::memcpy(joint.a,request.start,sizeof(joint.a));Pose flange{};if(!solver->SolveFK(joint,flange))return INVALID;
    begin=Compose(flange,request.tool);begin.Y+=request.start[6]*.001f;end=request.goal;
    Quaternion(begin.R,quaternion[0]);Quaternion(end.R,quaternion[1]);float dot=0;for(unsigned i=0;i<4;++i)dot+=quaternion[0][i]*quaternion[1][i];
    if(dot<0){for(float& q:quaternion[1])q=-q;dot=-dot;}angle=2*std::acos(Clamp(dot,-1,1));distance=PositionError(begin,end);
    float slider=Clamp(request.speed,1,100),minimumTime=std::fmax(.4f,std::fmax(1.875f*distance/(slider*.001f),1.875f*angle/(slider*.45f*Rad)));
    auto candidate=[&](float rail)->Result{
        Result r=SolvePath(rail);if(r!=OK)return r;
        float time=std::fmax(minimumTime,1.875f*std::fabs(rail-request.start[6])/limits.velocity[6]);
        for(unsigned attempt=0;attempt<10;++attempt){segments=unsigned(std::ceil(time/.020f));if(segments>MaxSegments)return BUDGET;
            r=Validate();if(r==OK){ready=true;return OK;}if(r!=DYNAMIC_LIMIT)return r;time*=1.35f;}
        return DYNAMIC_LIMIT;
    };
    Result fixed=candidate(request.start[6]);if(fixed==OK||fixed==CANCELLED||fixed==BUDGET||!request.automaticRail)return fixed;
    // Geometry/limits only trigger rail search. Dynamic-only failure is not a
    // reason to move the lower-precision rail.
    if(fixed==DYNAMIC_LIMIT)return fixed;
    float preferred=Clamp((end.Y-begin.Y)*1000, -1,1);if(std::fabs(preferred)<.01f)preferred=end.Y>=request.start[6]*.001f?1:-1;
    const float batches[8]={30,60,120,200,-30,-60,-120,-200};Result last=fixed;
    for(float batch:batches){float rail=Clamp(request.start[6]+preferred*batch,limits.lower[6]+5,limits.upper[6]-5);if(std::fabs(rail-request.start[6])<5)continue;
        last=candidate(rail);if(last==OK||last==CANCELLED||last==BUDGET)return last;}
    // Bounded coarse search also covers disconnected feasible rail bands and
    // narrow safe moves. Batch preference never becomes a hard minimum.
    for(unsigned i=0;i<9;++i){float rail=limits.lower[6]+5+(limits.upper[6]-limits.lower[6]-10)*i/8;
        last=candidate(rail);if(last==OK||last==CANCELLED||last==BUDGET)return last;}
    return last;
}
}
