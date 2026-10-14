#pragma once
#include "../algorithms/kinematic/6dof_kinematic.h"
#include "../../../stream_protocol.h"

// Bounded, allocation-free finite Cartesian planner. Public joints are deg/mm;
// rigid-transform translations are metres. No device or RTOS dependencies.
namespace NativeCartesian {
using Pose=DOF6Kinematic::Pose6D_t;
struct Limits {
    float lower[7],upper[7],velocity[7],acceleration[7],jerk[7];
    float stepsPerUnit[7],origin[7];
};
struct Request {
    float start[7]{};
    int32_t seed[7]{};
    Pose goal{},tool{};
    float speed=35;
    bool automaticRail=false;
    bool exactSeed=false;
};
class Progress {
public:
    virtual bool Checkpoint()=0; // Must check cancellation and yield to CAN/STOP.
};
enum Result { OK, INVALID, NO_IK, JOINT_LIMIT, DISCONTINUITY, GEOMETRY,
              DYNAMIC_LIMIT, BUDGET, CANCELLED };
const char* Name(Result r);
Pose FromXYZABC(float x,float y,float z,float a,float b,float c);
Pose Compose(const Pose& a,const Pose& b);
Pose Inverse(const Pose& p);
void ToXYZABC(const Pose& p,float out[6]);
class Planner {
public:
    static constexpr unsigned Intervals=128, MaxSegments=6000;
    Result Build(DOF6Kinematic& solver,const Request& request,const Limits& limits,Progress& progress);
    bool Record(unsigned sequence,uint16_t session,StreamProtocol::Record& out) const;
    unsigned Segments() const {return segments;}
    float Duration() const {return segments*.020f;}
    float RailTarget() const {return railGoal;}
    const float* GoalJoints() const {return node[Intervals];}
    unsigned Work() const {return work;}
    float PositionBound() const {return maxPositionBound;}
    float RotationBound() const {return maxRotationBound;}
private:
    float node[Intervals+1][7]{};
    Limits limits{};
    Request request{};
    Pose begin{},end{};
    float quaternion[2][4]{},angle=0,distance=0,railGoal=0;
    float maxPositionBound=0,maxRotationBound=0;
    unsigned segments=0,work=0;
    DOF6Kinematic* solver=nullptr;
    Progress* progress=nullptr;
    bool ready=false;
    struct GeometryFrame {float controls[7][6];float t0,t1;unsigned depth;};
    GeometryFrame geometryStack[11]{};
    bool TickWork(unsigned amount=1);
    Pose Target(float s) const;
    Result SolvePath(float rail);
    void Evaluate(float time,float q[7],float v[7],float a[7]) const;
    Result Validate();
    bool GenerateRecord(unsigned sequence,uint16_t session,StreamProtocol::Record& out) const;
    Result CheckGeometry(const float controls[7][6],float t0,float t1,unsigned depth);
};
}
