#include "6dof_kinematic.h"
#include <cmath>
#include <cstdint>
#include <cstring>
#if defined(STM32F405xx)
#include "stm32f405xx.h"
#endif
#include "arm_math.h"

namespace {
constexpr float Pi = 3.14159265358979323846f;
constexpr float HalfPi = Pi * 0.5f;
constexpr float DegToRad = Pi / 180.0f;
constexpr float RadToDeg = 180.0f / Pi;
constexpr float ReachTolerance = 0.000001f;
constexpr float WristSingularTolerance = 0.0001f;

// Release uses -Ofast. std::isfinite may be folded away under finite-math-only.
bool Finite(float value) {
    uint32_t bits; std::memcpy(&bits, &value, sizeof(bits));
    return (bits & 0x7f800000U) != 0x7f800000U;
}
void SinCos(float x,float& sine,float& cosine) {
    sine=arm_sin_f32(x);cosine=arm_cos_f32(x);
    // CMSIS fast-table interpolation has a small radial error. Restore a unit
    // pair so analytic inversion is consistent near elbow/wrist boundaries.
    const float norm=std::sqrt(sine*sine+cosine*cosine);sine/=norm;cosine/=norm;
}
float ClampUnit(float x) { return x < -1.0f ? -1.0f : (x > 1.0f ? 1.0f : x); }
float Radians(float degrees) { return std::remainderf(degrees, 360.0f) * DegToRad; }
float NearDegrees(float radians, float seed) {
    return seed + std::remainderf(radians * RadToDeg - std::remainderf(seed,360.0f),360.0f);
}
void Multiply(const float* a, const float* b, float* out) {
    for(unsigned i=0;i<3;++i) for(unsigned j=0;j<3;++j) {
        float value=0; for(unsigned k=0;k<3;++k) value += a[i*3+k]*b[k*3+j];
        out[i*3+j]=value;
    }
}
void Identity(float* r) { for(unsigned i=0;i<9;++i) r[i]=(i%4==0)?1.0f:0.0f; }
void JointRotation(unsigned i, float q, float* r) {
    const float ca[6]={0,1,0,0,0,1}, sa[6]={-1,0,1,-1,1,0};
    const float home[6]={0,-HalfPi,HalfPi,0,0,0};
    float c,s;SinCos(q+home[i],s,c);
    r[0]=c;r[1]=-ca[i]*s;r[2]=sa[i]*s;
    r[3]=s;r[4]=ca[i]*c;r[5]=-sa[i]*c;
    r[6]=0;r[7]=sa[i];r[8]=ca[i];
}
void RotationFirstThree(float q1,float q2,float q3,float* out) {
    float r[9], tmp[9]; Identity(out);
    const float q[3]={q1,q2,q3};
    for(unsigned i=0;i<3;++i) { JointRotation(i,q[i],r);Multiply(out,r,tmp);std::memcpy(out,tmp,sizeof(tmp)); }
}
bool ProperRotation(const float* r) {
    for(unsigned i=0;i<9;++i) if(!Finite(r[i])) return false;
    for(unsigned i=0;i<3;++i) for(unsigned j=0;j<3;++j) {
        float dot=0;for(unsigned k=0;k<3;++k)dot+=r[k*3+i]*r[k*3+j];
        if(std::fabs(dot-(i==j?1.0f:0.0f))>0.002f)return false;
    }
    const float determinant=r[0]*(r[4]*r[8]-r[5]*r[7])-r[1]*(r[3]*r[8]-r[5]*r[6])+r[2]*(r[3]*r[7]-r[4]*r[6]);
    return std::fabs(determinant-1.0f)<=0.003f;
}
void EulerToRotation(float roll,float pitch,float yaw,float* r) {
    float cr,sr,cp,sp,cy,sy;SinCos(roll,sr,cr);SinCos(pitch,sp,cp);SinCos(yaw,sy,cy);
    r[0]=cy*cp;r[1]=cy*sp*sr-sy*cr;r[2]=cy*sp*cr+sy*sr;
    r[3]=sy*cp;r[4]=sy*sp*sr+cy*cr;r[5]=sy*sp*cr-cy*sr;
    r[6]=-sp;r[7]=cp*sr;r[8]=cp*cr;
}
void RotationToEuler(const float* r,float& roll,float& pitch,float& yaw) {
    const float horizontal=std::sqrt(r[0]*r[0]+r[3]*r[3]);
    pitch=std::atan2(-r[6],horizontal)*RadToDeg;
    if(horizontal>0.000001f) { roll=std::atan2(r[7],r[8])*RadToDeg;yaw=std::atan2(r[3],r[0])*RadToDeg; }
    else { roll=0;yaw=std::atan2(-r[1],r[4])*RadToDeg; }
}
}

DOF6Kinematic::DOF6Kinematic(float base,float radial,float arm,float forearm,float elbow,float wrist)
    : armConfig{base,radial,arm,forearm,elbow,wrist} {
    geometryValid=Finite(base)&&Finite(radial)&&Finite(arm)&&Finite(forearm)&&Finite(elbow)&&Finite(wrist)
        &&base>=0&&arm>0&&forearm>0&&elbow>=0&&wrist>=0;
    if(geometryValid) {
        l_se=arm;l_se_2=arm*arm;l_ew_2=forearm*forearm+elbow*elbow;
        l_ew=std::sqrt(l_ew_2);atan_e=std::atan2(elbow,forearm);
        geometryValid=Finite(l_se_2)&&Finite(l_ew_2)&&Finite(l_ew)&&l_ew>0;
    }
}
bool DOF6Kinematic::SolveFK(const Joint6D_t& joints,Pose6D_t& out) {
    out=Pose6D_t{}; if(!geometryValid)return false;
    for(float q:joints.a)if(!Finite(q))return false;
    const float offsets[6][3]={{armConfig.D_BASE,-armConfig.L_BASE,0},{armConfig.L_ARM,0,0},
        {-armConfig.D_ELBOW,0,armConfig.L_FOREARM},{0,0,0},{0,0,0},{0,0,armConfig.L_WRIST}};
    float rotation[9],joint[9],next[9],p[3]{};Identity(rotation);
    for(unsigned i=0;i<6;++i) {
        JointRotation(i,Radians(joints.a[i]),joint);Multiply(rotation,joint,next);
        std::memcpy(rotation,next,sizeof(next));
        for(unsigned row=0;row<3;++row) for(unsigned k=0;k<3;++k)p[row]+=rotation[row*3+k]*offsets[i][k];
    }
    if(!Finite(p[0])||!Finite(p[1])||!Finite(p[2])||!ProperRotation(rotation))return false;
    out.X=p[0];out.Y=p[1];out.Z=p[2];std::memcpy(out.R,rotation,sizeof(rotation));out.hasR=true;
    RotationToEuler(rotation,out.A,out.B,out.C);return true;
}
bool DOF6Kinematic::IsSolutionValid(const IKSolves_t& solutions,unsigned index) {
    return index<8&&solutions.solFlag[index][0]!=INVALID&&solutions.solFlag[index][1]!=INVALID&&solutions.solFlag[index][2]!=INVALID;
}
bool DOF6Kinematic::NearestInLimits(float angle,float seed,float minimum,float maximum,float& result) {
    if(!Finite(angle)||!Finite(seed)||!Finite(minimum)||!Finite(maximum)||minimum>maximum)return false;
    const float base=std::remainderf(angle,360.0f);
    const float lo=std::ceil((minimum-base)/360.0f),hi=std::floor((maximum-base)/360.0f);
    if(lo>hi)return false;
    float turn=std::round((seed-base)/360.0f);if(turn<lo)turn=lo;if(turn>hi)turn=hi;
    const float candidate=base+360.0f*turn;
    if(!Finite(candidate)||candidate<minimum||candidate>maximum)return false;
    result=candidate;return true;
}
bool DOF6Kinematic::SolveIK(const Pose6D_t& millimetres,const Joint6D_t& seed,IKSolves_t& out) {
    Pose6D_t metres=millimetres;metres.X*=0.001f;metres.Y*=0.001f;metres.Z*=0.001f;
    return SolveIKMeters(metres,seed,out);
}
bool DOF6Kinematic::SolveIKMeters(const Pose6D_t& pose,const Joint6D_t& seed,IKSolves_t& out) {
    out=IKSolves_t{};
    if(!geometryValid||!Finite(pose.X)||!Finite(pose.Y)||!Finite(pose.Z))return false;
    for(float q:seed.a)if(!Finite(q))return false;
    float target[9];
    if(pose.hasR)std::memcpy(target,pose.R,sizeof(target));
    else { if(!Finite(pose.A)||!Finite(pose.B)||!Finite(pose.C))return false;
        EulerToRotation(Radians(pose.A),Radians(pose.B),Radians(pose.C),target); }
    if(!ProperRotation(target))return false;
    const float wrist[3]={pose.X-target[2]*armConfig.L_WRIST,pose.Y-target[5]*armConfig.L_WRIST,pose.Z-target[8]*armConfig.L_WRIST};
    const float radius=std::sqrt(wrist[0]*wrist[0]+wrist[1]*wrist[1]);
    if(!Finite(radius))return false;
    const bool shoulderSingular=radius<=ReachTolerance;
    const float shoulder[2]={shoulderSingular?Radians(seed.a[0]):std::atan2(wrist[1],wrist[0]),
        shoulderSingular?Radians(seed.a[0]):std::atan2(-wrist[1],-wrist[0])};
    bool any=false;
    for(unsigned arm=0;arm<2;++arm) {
        float cs,ss;SinCos(shoulder[arm],ss,cs);
        const float x=cs*wrist[0]+ss*wrist[1]-armConfig.D_BASE,y=armConfig.L_BASE-wrist[2];
        const float square=x*x+y*y,length=std::sqrt(square),minimum=std::fabs(l_se-l_ew),maximum=l_se+l_ew;
        if(!Finite(length)||length<minimum-ReachTolerance||length>maximum+ReachTolerance||length<=ReachTolerance)continue;
        const float ce=(square-l_se_2-l_ew_2)/(2.0f*l_se*l_ew);
        if(!Finite(ce)||ce>1.0001f||ce< -1.0001f)continue;
        // Both triangle angles use the same clamped cosine. Independent acos
        // calls lose their geometric consistency near full extension/folding.
        const float cosineElbow=ClampUnit(ce),sineElbow=std::sqrt(std::fmax(0.0f,1.0f-cosineElbow*cosineElbow));
        const float delta=std::atan2(sineElbow,cosineElbow);
        const float angle=std::atan2(y,x),aa=std::atan2(l_ew*sineElbow,l_se+l_ew*cosineElbow);
        const bool elbowSingular=std::fabs(length-maximum)<=ReachTolerance||std::fabs(length-minimum)<=ReachTolerance;
        for(unsigned elbow=0;elbow<2;++elbow) {
            const bool reverse=(elbow!=arm);
            const float q2=angle+(reverse?aa:-aa)+HalfPi,q3=atan_e+(reverse?-delta:delta);
            float r03[9],transpose[9],r36[9];RotationFirstThree(shoulder[arm],q2,q3,r03);
            for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)transpose[i*3+j]=r03[j*3+i];
            Multiply(transpose,target,r36);
            const float sine=std::sqrt(r36[2]*r36[2]+r36[5]*r36[5]),cosine=ClampUnit(r36[8]);
            const bool wristSingular=sine<=WristSingularTolerance;
            for(unsigned wristBranch=0;wristBranch<2;++wristBranch) {
                float q4,q5,q6;
                if(wristSingular) {
                    const bool folded=cosine<0;const float theta=std::atan2(folded?-r36[3]:r36[3],folded?-r36[0]:r36[0]);
                    q5=folded?Pi:0.0f;
                    if(wristBranch==0) {q4=Radians(seed.a[3]);q6=folded?q4-theta:theta-q4;}
                    else {q6=Radians(seed.a[5]);q4=folded?theta+q6:theta-q6;}
                } else {
                    const bool negative=(wristBranch!=arm);q5=std::atan2(negative?-sine:sine,cosine);
                    q4=std::atan2(negative?-r36[5]:r36[5],negative?-r36[2]:r36[2]);
                    q6=std::atan2(negative?-r36[7]:r36[7],negative?r36[6]:-r36[6]);
                }
                const unsigned index=arm*4+elbow*2+wristBranch;
                const float angles[6]={shoulder[arm],q2,q3,q4,q5,q6};Joint6D_t candidate;
                for(unsigned j=0;j<6;++j)candidate.a[j]=NearDegrees(angles[j],seed.a[j]);
                Pose6D_t fk;if(!SolveFK(candidate,fk))continue;
                const float dx=fk.X-pose.X,dy=fk.Y-pose.Y,dz=fk.Z-pose.Z;
                if(dx*dx+dy*dy+dz*dz>PositionToleranceM*PositionToleranceM)continue;
                bool rotationMatches=true;for(unsigned j=0;j<9;++j)if(std::fabs(fk.R[j]-target[j])>RotationElementTolerance)rotationMatches=false;
                if(!rotationMatches)continue;
                out.config[index]=candidate;
                out.solFlag[index][0]=shoulderSingular?SINGULAR:REGULAR;
                out.solFlag[index][1]=elbowSingular?SINGULAR:REGULAR;
                out.solFlag[index][2]=wristSingular?SINGULAR:REGULAR;any=true;
            }
        }
    }
    return any;
}
DOF6Kinematic::Joint6D_t operator-(const DOF6Kinematic::Joint6D_t& a,const DOF6Kinematic::Joint6D_t& b) {
    DOF6Kinematic::Joint6D_t out;for(unsigned i=0;i<6;++i)out.a[i]=a.a[i]-b.a[i];return out;
}
