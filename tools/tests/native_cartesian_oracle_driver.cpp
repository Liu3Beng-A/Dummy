// Offline numerical driver: real planner/IK/motor Curve; independent double DH goal.
// No device access. Run via run_native_cartesian_oracle.py --output <scratch>.
#include "cartesian_planner.h"
#include "stream_curve.h"
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>
using namespace NativeCartesian;
constexpr double pi=3.1415926535897932384626433832795;
struct Reference {double p[3]{},r[9]={1,0,0,0,1,0,0,0,1};};
Reference DoubleFK(const double* q){
    const double home[6]={0,-pi/2,pi/2,0,0,0},alpha[6]={-pi/2,0,pi/2,-pi/2,pi/2,0};
    const double offsets[6][3]={{0,-.2073,0},{.170,0,0},{-.0695,0,.117},{0,0,0},{0,0,0},{0,0,.113}};
    Reference p;
    for(unsigned i=0;i<6;++i){double theta=q[i]*pi/180+home[i],c=std::cos(theta),s=std::sin(theta),ca=std::cos(alpha[i]),sa=std::sin(alpha[i]);
        double local[9]={c,-ca*s,sa*s,s,ca*c,-sa*c,0,sa,ca},next[9]{};
        for(unsigned a=0;a<3;++a)for(unsigned b=0;b<3;++b)for(unsigned k=0;k<3;++k)next[a*3+b]+=p.r[a*3+k]*local[k*3+b];
        for(unsigned a=0;a<3;++a)for(unsigned b=0;b<3;++b)p.p[a]+=next[a*3+b]*offsets[i][b];
        for(unsigned k=0;k<9;++k)p.r[k]=next[k];
    }
    p.p[1]+=q[6]*.001;return p;
}
Pose DoubleTCP(const double* q,const Pose& tool){auto p=DoubleFK(q);Pose out{};const float t[3]={tool.X,tool.Y,tool.Z};
    for(unsigned i=0;i<3;++i)for(unsigned k=0;k<3;++k)p.p[i]+=p.r[i*3+k]*t[k];
    out.X=float(p.p[0]);out.Y=float(p.p[1]);out.Z=float(p.p[2]);for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)for(unsigned k=0;k<3;++k)out.R[i*3+j]+=float(p.r[i*3+k]*tool.R[k*3+j]);out.hasR=true;return out;}
struct Work:Progress{unsigned calls=0,stopAfter=~0U;bool Checkpoint()override{return ++calls<stopAfter;}};
struct TestCase{const char* name;std::array<double,7> q;float dx,dy,dz,turn;bool rail;};
void Array(std::ostream& f,const float* x,unsigned n){f<<'[';for(unsigned i=0;i<n;++i){if(i)f<<',';f<<x[i];}f<<']';}
void PoseJson(std::ostream& f,const Pose& x){float p[3]={x.X,x.Y,x.Z};f<<"{\"p\":";Array(f,p,3);f<<",\"r\":";Array(f,x.R,9);f<<'}';}
int main(int argc,char** argv){
    if(argc!=2){std::fprintf(stderr,"curves output path required\n");return 1;}
    std::ofstream out(argv[1]);if(!out.is_open())return 1;out<<std::setprecision(10)<<"{\"cases\":[";
    DOF6Kinematic solver(.2073f,0,.170f,.117f,.0695f,.113f);
    Limits limits{};const float lower[7]={-175,-75,0,-270,-100,-180,-250},upper[7]={175,90,180,270,100,180,250},origin[7]={0,-75,180,0,0,0,0};
    for(unsigned j=0;j<7;++j){limits.lower[j]=lower[j];limits.upper[j]=upper[j];limits.origin[j]=origin[j];limits.stepsPerUnit[j]=(j==0?1:-1)*(j==6?10240.f:(j==5?30.f:50.f)*51200.f/360.f);limits.velocity[j]=j==6?20:40;limits.acceleration[j]=j==6?40:80;limits.jerk[j]=j==6?3000:20000;}
    const TestCase cases[]={
        {"hold",{0,10,80,0,90,0,0},0,0,0,0,false},
        {"short_x",{0,10,80,0,90,0,0},.005f,0,0,0,false},
        {"short_y",{0,10,80,0,90,0,0},0,.005f,0,0,false},
        {"short_z",{0,10,80,0,90,0,0},0,0,.005f,0,false},
        {"wrist_turn",{0,10,80,0,90,0,0},0,0,0,5,false},
        {"wrist180",{0,10,80,0,90,0,0},0,0,0,180,false},
        {"legacy_flange",{0,10,80,0,90,0,0},.005f,0,0,0,false},
        {"custom_tool",{37,-25,83,267,-42,-171,0},.005f,0,0,0,false},
        {"near_singular",{35,-25,83,20,.01,-45,0},.002f,0,0,0,false},
        {"rail350",{0,10,80,0,90,0,0},0,.350f,0,0,true},
        {"rail450",{0,10,80,0,90,0,0},0,.450f,0,0,true},
        {"rail_negative",{0,10,80,0,90,0,0},0,-.350f,0,0,true}};
    bool comma=false;unsigned accepted=0;
    for(const auto& test:cases){Request request{};request.tool=std::strcmp(test.name,"legacy_flange")==0?FromXYZABC(0,0,0,0,0,0):std::strcmp(test.name,"custom_tool")==0?FromXYZABC(.023f,-.008f,.101f,25,35,-15):FromXYZABC(0,0,.114f,0,0,0);request.goal=DoubleTCP(test.q.data(),request.tool);request.goal.X+=test.dx;request.goal.Y+=test.dy;request.goal.Z+=test.dz;request.speed=35;request.automaticRail=test.rail;request.exactSeed=true;
        for(unsigned j=0;j<7;++j){request.seed[j]=int32_t(std::round((test.q[j]-origin[j])*limits.stepsPerUnit[j]));request.start[j]=origin[j]+request.seed[j]/limits.stepsPerUnit[j];}
        if(test.turn){auto r=FromXYZABC(0,0,0,0,0,test.turn);auto goal=Compose(r,request.goal);goal.X=request.goal.X;goal.Y=request.goal.Y;goal.Z=request.goal.Z;request.goal=goal;}
        Work work;Planner planner;Result result=planner.Build(solver,request,limits,work);std::printf("%s result=%s n=%u rail=%.5f work=%u bound_mm=%.8f\n",test.name,Name(result),planner.Segments(),planner.RailTarget(),planner.Work(),planner.PositionBound()*1000);
        Result expected=std::strcmp(test.name,"near_singular")==0?DISCONTINUITY:std::strcmp(test.name,"rail450")==0?JOINT_LIMIT:OK;
        if(result!=expected){std::fprintf(stderr,"unexpected planner result for %s\n",test.name);return 10;}
        if(result!=OK){StreamProtocol::Record rejected{};if(planner.Record(0,1,rejected))return 11;}
        if(result==OK&&test.rail&&((test.dy>0&&planner.RailTarget()<=0)||(test.dy<0&&planner.RailTarget()>=0)))return 12;
        if(comma)out<<',';comma=true;out<<"{\"name\":\""<<test.name<<"\",\"result\":\""<<Name(result)<<"\",\"segments\":"<<planner.Segments()<<",\"rail\":"<<planner.RailTarget()<<",\"bound_m\":"<<planner.PositionBound()<<",\"bound_rad\":"<<planner.RotationBound()<<",\"duration\":"<<planner.Duration()<<",\"work\":"<<planner.Work()<<",\"start\":";Array(out,request.start,7);out<<",\"goal\":";PoseJson(out,request.goal);out<<",\"tool\":";PoseJson(out,request.tool);out<<",\"factor\":";Array(out,limits.stepsPerUnit,7);out<<",\"origin\":";Array(out,limits.origin,7);
        if(result==OK){++accepted;std::vector<StreamProtocol::Record> records(planner.Segments()+1);bool good=true;for(unsigned n=0;n<records.size();++n)good&=planner.Record(n,1,records[n]);if(!good)return 2;
            if(records.front().flags!=StreamProtocol::SEED||records.back().flags!=StreamProtocol::END)return 13;
            for(unsigned j=0;j<7;++j)if(records.front().axes[j].q!=request.seed[j]||records.front().axes[j].v||records.front().axes[j].a||records.back().axes[j].v||records.back().axes[j].a)return 14;
            double maxCurveError=0,maxVelocityError=0;out<<",\"records\":[";for(unsigned n=0;n<records.size();++n){if(n)out<<',';out<<'[';for(unsigned j=0;j<7;++j){if(j)out<<',';auto p=records[n].axes[j];out<<'['<<p.q<<','<<p.v<<','<<p.a<<']';}out<<']';}out<<"],\"samples\":[";
            bool sampleComma=false;
            for(unsigned n=1;n<records.size();++n){MotorStream::Curve curve[7];double coef[7][6];
                for(unsigned j=0;j<7;++j){auto a=records[n-1].axes[j],b=records[n].axes[j];if(!curve[j].Build({a.q,a.v,a.a},{b.q,b.v,b.a},1536000,5120000))return 3;
                    double d=double(b.q)-a.q;coef[j][0]=a.q;coef[j][1]=a.v*.020;coef[j][2]=a.a*.0002;coef[j][3]=10*d-.12*a.v-.08*b.v-.0006*a.a+.0002*b.a;coef[j][4]=-15*d+.16*a.v+.14*b.v+.0006*a.a-.0004*b.a;coef[j][5]=6*d-.06*a.v-.06*b.v-.0002*a.a+.0002*b.a;}
                for(unsigned tick=1;tick<=400;++tick){float q[7];int32_t motor[7];for(unsigned j=0;j<7;++j){int32_t p,v;curve[j].Tick(p,v);motor[j]=p;q[j]=limits.origin[j]+p/limits.stepsPerUnit[j];double t=tick/400.,poly=coef[j][5];for(int k=4;k>=0;--k)poly=coef[j][k]+t*poly;maxCurveError=std::fmax(maxCurveError,std::fabs(p-poly));double derivative=5*coef[j][5];for(int k=4;k>=1;--k)derivative=k*coef[j][k]+t*derivative;maxVelocityError=std::fmax(maxVelocityError,std::fabs(v-derivative/.020));}
                    if(tick%20==0||tick==1){if(sampleComma)out<<',';sampleComma=true;out<<"["<<(n-1)*.020+tick*.00005<<',';Array(out,q,7);out<<",[";for(unsigned j=0;j<7;++j){if(j)out<<',';out<<motor[j];}out<<"]]";}}
            }
            out<<"],\"motor_curve_error_steps_max\":"<<maxCurveError<<",\"motor_velocity_error_steps_per_s_max\":"<<maxVelocityError;if(maxCurveError>2||maxVelocityError>2)return 4;
        }
        out<<'}';
    }
    Request invalid{};for(unsigned j=0;j<7;++j)invalid.start[j]=cases[0].q[j];invalid.tool=FromXYZABC(0,0,.114f,0,0,0);invalid.goal=DoubleTCP(cases[0].q.data(),invalid.tool);Work work;Planner malformed;Limits bad=limits;bad.velocity[0]=std::numeric_limits<float>::quiet_NaN();auto code=malformed.Build(solver,invalid,bad,work);
    StreamProtocol::Record leaked{};Work cancelled;cancelled.stopAfter=131;auto cancelledCode=malformed.Build(solver,invalid,limits,cancelled);bool cancelledRecords=malformed.Record(0,1,leaked);
    out<<"],\"invalid_velocity_result\":\""<<Name(code)<<"\",\"cancelled_result\":\""<<Name(cancelledCode)<<"\",\"cancelled_Record_allowed\":"<<(cancelledRecords?"true":"false")<<",\"accepted\":"<<accepted<<"}";
    std::printf("invalid_velocity=%s cancelled_Record=%d accepted=%u\n",Name(code),int(cancelledRecords),accepted);out.flush();
    return code==INVALID&&cancelledCode==CANCELLED&&!cancelledRecords&&accepted==10&&out.good()?0:15;
}
