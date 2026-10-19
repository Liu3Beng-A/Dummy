#include "cartesian_benchmark.h"
#include "cartesian_planner.h"
#include <cmath>
#include <cstring>

namespace CartBench {
namespace {
constexpr float Fixtures[3][6]={{0,-30,80,20,35,0},
    {35,-20,100,-40,50,20},{-40,10,65,35,-45,-20}};
constexpr float Lower[6]={-175,-75,0,-270,-100,-180};
constexpr float Upper[6]={175,90,180,270,100,180};
constexpr float RailMm[5]={-2,-1,0,1,2};

void Hash(uint32_t& sum,uint32_t value){sum=(sum^value)*16777619U;}
void HashFloat(uint32_t& sum,float value){uint32_t bits;std::memcpy(&bits,&value,sizeof(bits));Hash(sum,bits);}
void HashPose(uint32_t& sum,const DOF6Kinematic::Pose6D_t& p){
    HashFloat(sum,p.X);HashFloat(sum,p.Y);HashFloat(sum,p.Z);
    HashFloat(sum,p.A);HashFloat(sum,p.B);HashFloat(sum,p.C);
    for(float r:p.R)HashFloat(sum,r);
}
unsigned HashSolutions(uint32_t& sum,const DOF6Kinematic::IKSolves_t& solutions){
    unsigned valid=0;
    for(unsigned i=0;i<8;++i){
        const bool accepted=DOF6Kinematic::IsSolutionValid(solutions,i);
        Hash(sum,accepted?i+1U:0U);
        if(accepted){++valid;for(float q:solutions.config[i].a)HashFloat(sum,q);}
    }
    return valid;
}
bool Residual(const DOF6Kinematic::Pose6D_t& a,const DOF6Kinematic::Pose6D_t& b){
    const float x=a.X-b.X,y=a.Y-b.Y,z=a.Z-b.Z;
    if(x*x+y*y+z*z>DOF6Kinematic::PositionToleranceM*DOF6Kinematic::PositionToleranceM)return false;
    for(unsigned i=0;i<9;++i)
        if(std::fabs(a.R[i]-b.R[i])>DOF6Kinematic::RotationElementTolerance)return false;
    return true;
}
}

bool Valid(const Config& c){
    return c.operation<=UNREACHABLE&&c.fixture<3&&c.work>=1&&c.work<=MaxWork
        &&c.samples>=1&&c.samples<=MaxSamples&&c.budgetUs>=100&&c.budgetUs<=20000;
}

Compute::Compute():solver(.2073f,0,.170f,.117f,.0695f,.113f){}

bool Compute::Prepare(const Config& c){
    prepared=false;
    if(!Valid(c))return false;
    std::memcpy(seed.a,Fixtures[c.fixture],sizeof(seed.a));
    tool=NativeCartesian::FromXYZABC(0,0,.114f,0,0,0);
    if(!solver.SolveFK(seed,flange))return false;
    preparedFixture=c.fixture;prepared=true;
    return true;
}

Sample Compute::Measure(const Config& c,const Hooks& h){
    Sample out{};
    if(!Valid(c)||!prepared||preparedFixture!=c.fixture||!h.cycles||!h.ticks
        ||!h.checkpoint||h.clockHz==0){out.flag=ABORTED;return out;}
    const uint32_t beginCycles=h.cycles(h.context),beginTicks=h.ticks(h.context);
    const uint64_t cycleBudget=uint64_t(c.budgetUs)*h.clockHz;
    out.checksum=2166136261U;
    auto checkpoint=[&](){
        const uint8_t flag=h.checkpoint(h.context);
        out.cycles=h.cycles(h.context)-beginCycles;
        out.ticks=h.ticks(h.context)-beginTicks;
        if(flag!=COMPLETE){out.flag=flag==CANCELLED?CANCELLED:ABORTED;return false;}
        if(out.ticks>=1000U){out.flag=ABORTED;return false;}
        if(uint64_t(out.cycles)*1000000U>=cycleBudget){out.flag=BUDGET;return false;}
        return true;
    };
    for(unsigned work=0;work<c.work;++work){
        if(!checkpoint())break;
        DOF6Kinematic::Pose6D_t target=flange;
        bool success=true;
        if(c.operation==FK||c.operation==ROUND_TRIP||c.operation==TCP_SELECT||c.operation==RAIL_CANDIDATE){
            ++out.fk;success=solver.SolveFK(seed,target);HashPose(out.checksum,target);
        }
        if(success&&c.operation!=FK){
            if(c.operation==TCP_SELECT||c.operation==RAIL_CANDIDATE){
                auto tcp=NativeCartesian::Compose(target,tool);
                if(c.operation==RAIL_CANDIDATE)tcp.Y-=RailMm[work%5]*.001f;
                target=NativeCartesian::Compose(tcp,NativeCartesian::Inverse(tool));
                HashPose(out.checksum,target);
            }else if(c.operation==UNREACHABLE)target.X+=10;
            DOF6Kinematic::IKSolves_t solutions{};
            ++out.ik;
            success=solver.SolveIKMeters(target,seed,solutions);
            out.valid+=uint16_t(HashSolutions(out.checksum,solutions));
            if(success&&(c.operation==TCP_SELECT||c.operation==RAIL_CANDIDATE)){
                DOF6Kinematic::Joint6D_t nearest{};
                float best=0;bool found=false;
                for(unsigned i=0;i<8;++i){
                    if(!DOF6Kinematic::IsSolutionValid(solutions,i))continue;
                    DOF6Kinematic::Joint6D_t candidate{};bool legal=true;float score=0;
                    for(unsigned j=0;j<6;++j){
                        if(!DOF6Kinematic::NearestInLimits(solutions.config[i].a[j],seed.a[j],Lower[j],Upper[j],candidate.a[j])){legal=false;break;}
                        const float delta=candidate.a[j]-seed.a[j];score+=delta*delta;
                    }
                    if(legal&&(!found||score<best)){found=true;best=score;nearest=candidate;}
                }
                success=found;
                if(found){
                    DOF6Kinematic::Pose6D_t checked{};++out.fk;
                    success=solver.SolveFK(nearest,checked)&&Residual(target,checked);
                    HashPose(out.checksum,checked);
                    for(float q:nearest.a)HashFloat(out.checksum,q);
                    if(success)++out.selected;
                }
            }
        }
        if(!success)++out.failed;
        ++out.work;
        const bool proceed=checkpoint();
        if(h.workBoundary)h.workBoundary(h.context,out.work,out.cycles);
        if(!proceed)break;
    }
    return out;
}
}
