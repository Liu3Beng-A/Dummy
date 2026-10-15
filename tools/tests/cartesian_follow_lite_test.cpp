// Host-only experimental follower. This is NOT firmware or a validated servo.
// Compile the production IK/FK, transforms and PositionTracker without edits.
// The simulated plant is precisely the motor's generated position reference;
// PID, encoder feedback, payload, communication losses and protection are absent.
#include "6dof_kinematic.h"
#include "cartesian_planner.h"
#include "motion_planner.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using Pose = DOF6Kinematic::Pose6D_t;
using Joint = DOF6Kinematic::Joint6D_t;
using Axes = std::array<double, 7>;
constexpr double Pi = 3.14159265358979323846;
constexpr int Hz = 20000, Steps = 51200;
constexpr double Lower[7] = {-175,-75,0,-270,-100,-180,-250};
constexpr double Upper[7] = {175,90,180,270,100,180,250};
constexpr double Origin[7] = {0,-75,180,0,0,0,0};
constexpr double Factor[7] = {50.*Steps/360,-50.*Steps/360,-50.*Steps/360,
    -50.*Steps/360,-50.*Steps/360,-30.*Steps/360,-Steps/5.};
struct Vec {
    double x=0,y=0,z=0;
    Vec operator+(Vec b) const {return {x+b.x,y+b.y,z+b.z};}
    Vec operator-(Vec b) const {return {x-b.x,y-b.y,z-b.z};}
    Vec operator*(double s) const {return {x*s,y*s,z*s};}
};
double Dot(Vec a,Vec b){return a.x*b.x+a.y*b.y+a.z*b.z;}
double Length(Vec a){return std::sqrt(Dot(a,a));}
Vec Unit(Vec a){double n=Length(a);return n>1e-12?a*(1/n):Vec{};}
double Clamp(double v,double a,double b){return std::max(a,std::min(b,v));}
Vec XYZ(const Pose& p){return {p.X*1000.,p.Y*1000.,p.Z*1000.};}
Pose At(Pose p,Vec v){p.X=float(v.x*.001);p.Y=float(v.y*.001);p.Z=float(v.z*.001);return p;}
double SegmentDistance(Vec p,Vec a,Vec b){Vec d=b-a;double n=Dot(d,d);return Length(p-(a+d*(n>0?Clamp(Dot(p-a,d)/n,0,1):0)));}
int32_t Encode(unsigned j,double q){double n=(q-Origin[j])*Factor[j];if(!std::isfinite(n)||std::abs(n)>2147483000.)throw std::runtime_error("encoder range");return int32_t(std::llround(n));}
double Decode(unsigned j,int32_t q){return Origin[j]+q/Factor[j];}
double Margin(const Axes& q){double m=1;for(unsigned j=0;j<6;++j)m=std::min(m,std::min(q[j]-Lower[j],Upper[j]-q[j])/(Upper[j]-Lower[j]));return m;}
void Require(bool value,const char* message){if(!value)throw std::runtime_error(message);}

struct Geometry {
    DOF6Kinematic solver{.2073f,0,.170f,.117f,.0695f,.113f};
    Pose tool=NativeCartesian::FromXYZABC(0,0,.114f,0,0,0);
    unsigned ik=0,fk=0;
    std::string inverseFailure;
    Pose Forward(const Axes& q){Joint j;for(unsigned a=0;a<6;++a)j.a[a]=float(q[a]);Pose flange;
        Require(solver.SolveFK(j,flange),"FK failed");++fk;Pose tcp=NativeCartesian::Compose(flange,tool);tcp.Y+=float(q[6]*.001);return tcp;}
    bool Inverse(Pose tcp,double rail,const Axes& seed,Axes& out,bool continuity=true){
        ++ik;inverseFailure.clear();if(rail<Lower[6]||rail>Upper[6]){inverseFailure="rail_limit";return false;}tcp.Y-=float(rail*.001);
        Pose flange=NativeCartesian::Compose(tcp,NativeCartesian::Inverse(tool));Joint j;
        for(unsigned a=0;a<6;++a)j.a[a]=float(seed[a]);DOF6Kinematic::IKSolves_t solutions;
        if(!solver.SolveIKMeters(flange,j,solutions)){inverseFailure="no_ik";return false;}
        double best=1e100;bool found=false,anyLegal=false;
        for(unsigned b=0;b<8;++b){if(!DOF6Kinematic::IsSolutionValid(solutions,b))continue;Axes q{};double score=0,maxDelta=0;bool legal=true;
            for(unsigned a=0;a<6;++a){float value;if(!DOF6Kinematic::NearestInLimits(solutions.config[b].a[a],j.a[a],float(Lower[a]),float(Upper[a]),value)){legal=false;break;}
                q[a]=value;double d=value-seed[a];score+=d*d;maxDelta=std::max(maxDelta,std::abs(d));}
            anyLegal=anyLegal||legal;
            if(legal&&(!continuity||maxDelta<=12)&&score<best){q[6]=rail;out=q;best=score;found=true;}}
        if(!found)inverseFailure=anyLegal?"branch_jump_over_12deg":"joint_limits";
        return found;
    }
};

struct Motor {
    MotionPlanner planner;
    MotionPlanner::Config_t config{};
    int32_t goal=0;
    Motor(double acceleration,int32_t initial){config.ratedVelocity=30*Steps;config.ratedVelocityAcc=int32_t(acceleration*Steps);
        planner.AttachConfig(&config);planner.positionTracker.Init();planner.positionTracker.NewTask(initial,0);
        // NewTask sets private state; one tick also publishes its output fields.
        goal=initial;planner.positionTracker.CalcSoftGoal(goal);}
    void Receive(int32_t position,int32_t cap){goal=position;config.ratedVelocity=std::max(1,cap);}
    void Tick(){planner.positionTracker.CalcSoftGoal(goal);}
};
// Read-only observer of the *existing* PositionTracker branch, before Tick().
// Oracle speed is for diagnostics only; it never enters either controller.
enum MotorReason { Locked, GoalBrake, ReverseBrake, CapBrake, DistanceBrake, Accelerating, Cruising, MotorReasonCount };
const char* MotorReasonNames[]={"locked","goal_brake","reverse_brake","cap_brake","distance_brake","accelerating","cruising"};
int ObserveMotor(const Motor& m){
    int32_t delta=m.goal-m.planner.positionTracker.go_location,v=m.planner.positionTracker.go_velocity;
    if(delta==0)return std::abs(v)<=m.config.ratedVelocityAcc/1000?Locked:GoalBrake;
    if(v==0)return Accelerating;
    if((delta>0)!=(v>0))return ReverseBrake;
    if(std::abs(v)>m.config.ratedVelocity)return CapBrake;
    // Same float arithmetic / integer truncation as the unchanged production kernel.
    float quick=.5f/float(m.config.ratedVelocityAcc);
    int32_t need=int32_t(float(v)*float(v)*quick);
    if(std::abs(delta)<=need)return DistanceBrake;
    return std::abs(v)<m.config.ratedVelocity?Accelerating:Cruising;
}
enum SlowReason { TurnBrake, RailChangeBrake, InputRelease, JointSpeed, CurvatureSpeed, FeedbackScale,
    TerminalBrake, AccelerationUp, AccelerationDown, StaleFeedback, BatchWait, ProbeIK, StepIK, GuideIK,
    TrackingFault, BaselineHold, AxisCap, GuideSignReset, Inactive, ReferenceAcceleration, SlowReasonCount };
const char* SlowReasonNames[]={"turn_brake","rail_change_brake","input_release","joint_speed_bound","curvature_speed_bound",
    "feedback_scale","terminal_brake","acceleration_ramp_up","acceleration_ramp_down","stale_feedback","batch_wait",
    "probe_ik_failure","step_ik_failure","guide_ik_failure","tracking_fault","baseline_lead_hold","axis_cap","guide_sign_reset","inactive","reference_acceleration_over_physical"};
const char* PhaseNames[]={"all","cruise_window","turn_to_release"};
struct ReasonStat {unsigned count=0;double ms=0,runMs=0,longestMs=0;
    void Add(bool active,double dt){if(active){++count;ms+=dt;runMs+=dt;longestMs=std::max(longestMs,runMs);}else runMs=0;}};
struct ControlDiagnostic {
    double t=0,dtMs=0,input=0,afterTurn=0,afterJoint=0,afterFeedback=0,allowed=0,previous=0,common=0,acceleration=0;
    double velocityBound=1e9,curvatureBound=1e9,terminalBound=1e9,feedbackFactor=1,worstRatio=0,tcpRatio=0,orientationRatio=0;
    double guideMs=0,remaining=1e9,tcpLag=0,alongLag=0,crossLag=0,actualSpeed=0;
    int velocityAxis=-1,curvatureAxis=-1,accelerationAxis=-1,worstSource=-1;
    bool active=true,cruise=false,turnWindow=false,sent=false;
    std::array<bool,SlowReasonCount> reasons{};
    Vec direction{},requestedDirection{},actual{},referenceBefore{},referenceAfter{};
    Axes observed{},estimatedVelocity{},feedbackAgeMs{},ratio{},referenceJoints{},goals{},motorPosition{},motorVelocity{},motorCap{},sentGoals{},sentCaps{},referenceVelocity{},referenceAccelerationRatio{};
};
struct Variant {
    const char* name;int periodMs;double horizonMs,skewMs,acceleration,speedScale;
    double feedbackDelayMs=0,feedbackSkewMs=0;bool feedbackDropout=false;
};
const Variant Variants[]={
    {"near20",20,20,.15,30,1},{"base60",20,60,.15,30,1},
    {"far120",20,120,.15,30,1},{"zero_skew",20,60,0,30,1},
    {"skew6ms",20,60,1,30,1},{"period40",40,60,.15,30,1},
    {"acc10",20,60,.15,10,1},{"speed2x",20,60,.15,30,2},
    {"speed10",20,60,.15,30,.25},{"speed20",20,60,.15,30,.5},
    {"feedback20ms",20,60,.15,30,1,20,1,false},
    {"feedback_dropout",20,60,.15,30,1,0,0,true}};
const Axes Fixtures[]={
    {0,-30,80,20,35,0,0}, {35,-20,100,-40,50,20,0},
    {-40,10,65,35,-45,-20,0}};
const char* Cases[]={"line_x","line_xyz","slider_burst","joystick_corner",
    "joystick_reverse","rail_line","rail_hold_tcp","rail_recovery","input_expiry","far_goal",
    "short_line_x","short_line_xyz","slider_short","rapid_short_reverse","rapid_short_corner"};
// Predeclared factorial ablations: no changes to the default A/B/C experiment.
// Disabling turn braking is an intentionally dynamically unchecked diagnostic,
// never a deployable controller. Hard feedback/limit/IK checks stay enabled.
struct Experiment {const char* name;double guideMs;bool feedbackScale,stopBeforeTurn;};
const Experiment Experiments[]={
    {"r_h60_on",60,true,true},{"r_h60_off",60,false,true},
    {"r_h120_on",120,true,true},{"r_h120_off",120,false,true},
    {"r_h240_on",240,true,true},{"r_h240_off",240,false,true},
    {"r_turn_off",60,true,false},{"r_turn_off_no_feedback",60,false,false}};
bool ResearchVariant(const std::string& n){return n=="base60"||n=="acc10"||n=="speed20"||n=="speed2x"||n=="feedback20ms"||n=="zero_skew"||n=="skew6ms";}
bool ResearchCase(const std::string& n){return n=="short_line_x"||n=="short_line_xyz"||n=="slider_short"||n=="rail_line"||n=="rail_hold_tcp"||n=="rapid_short_reverse"||n=="rapid_short_corner";}
struct Delivery {int tick;unsigned axis;int32_t q,cap;unsigned batch=0;};
struct FeedbackDelivery {int tick;unsigned axis;double position;};
struct Sample {
    double t;Vec actual,reference;Axes joints,referenceJoints{},motorCaps{};
    Axes diagnosticVelocity{},diagnosticGoals{},diagnosticReasons{};
    double orientation=0,speed=0,railReference=0,commonSpeed=0;bool cruise=false;
};
struct Result {
    std::string name,variant,controller;unsigned fixture=0,ik=0,fk=0,ikPeak=0,rejected=0,leadHolds=0,searchIk=0,expired=0;
    double pathMax=0,pathRms=0,orientationMax=0,referenceLeadMax=0,speedMean=0,speedCv=0,speedMin=0,speedMax=0,requested=0;
    double finalError=0,finalSpeed=0,reverseDelay=-1,reverseOvershoot=0,stopDelay=-1,stopTravel=0;
    double railEnd=0,railReferenceEnd=0,railRange=0,marginStart=0,marginEnd=0,limitExcess=0,selectedRail=0;
    double firstRejectAt=-1,brakeAt=-1,plannedRemaining=0;
    std::string rejectReason;
    double intentPathMax=0,intentEndError=0,travel=0,settledAt=-1,jointErrorRatio=0;
    double guideHorizonMaxMs=0;
    double referenceVelocityRatioMax=0,referenceAccelerationRatioMax=0;
    unsigned limitedUpdates=0,turnBrakes=0,feedbackFaults=0,batchWaits=0,normalStops=0,trackingFaults=0;
    unsigned guideRetryUpdates=0,guideRetrySuccesses=0,guideRetryExtraIk=0,guideIkPeak=0;
    bool finiteTarget=false,targetComplete=false;
    std::vector<Sample> samples;std::vector<Vec> referencePath;
    std::vector<ControlDiagnostic> controls;
    std::array<std::array<ReasonStat,SlowReasonCount>,3> slowStats{};
    std::array<double,3> phaseMs{};
    std::array<unsigned,3> phaseUpdates{};
    std::array<std::array<std::array<unsigned long long,MotorReasonCount>,7>,3> motorReasons{};
    unsigned long long observerChecks=0,observerMismatches=0;
};

class Simulation {
    Geometry geometry;
    const Variant v;
    const unsigned fixture;
    const std::string name;
    const bool coordinated;
    const bool adaptiveGuide;
    const bool boundedGuide;
    const Experiment* experiment=nullptr;
    std::array<std::unique_ptr<Motor>,7> motors;
    std::vector<Delivery> pending;
    std::vector<FeedbackDelivery> feedbackPending;
    Axes observed{},observedVelocity{};
    std::array<int,7> receivedAt{};
    unsigned batchSequence=0;
    unsigned trackingViolationStreak=0;
    Vec commonDirection{};
    double commonSpeed=0,commonRailSlope=0,lastPathAcceleration=100;
    bool normalStopRequested=false;
    Pose orientation;
    Vec begin,reference,finalGoal;
    Axes qref{},caps{};
    double railReference=0,railGoal=0;
    bool stopped=false;
    int searchIndex=-1;
    Pose searchPose;
    Axes searchSeed{};
    double searchRail=0,searchBestMargin=0,searchBestRail=0;
    std::array<double,6> candidateOffsets{-60,-30,-10,10,30,60};
    Result result;
    ControlDiagnostic diagnostic;
    Axes diagnosticReferenceBefore{},previousReferenceVelocity{};
    unsigned diagnosticHoldsBefore=0;
    double speed=40,stopAt=-1,reverseAt=-1,duration=5;
    Vec reverseDirection{1,0,0};
    double TurnAt()const{return name=="rapid_short_reverse"||name=="rapid_short_corner"?.25:
        (name=="joystick_reverse"||name=="joystick_corner"?1:-1);}
    bool TurnWindow(double t)const{return TurnAt()>=0&&t>=TurnAt()&&t<stopAt;}
    void BeginDiagnostic(int tick,double t){
        diagnostic={};diagnostic.t=t;diagnostic.dtMs=std::max(0.,std::min(v.periodMs*1.,(duration-t)*1000));
        diagnostic.active=!stopped;diagnostic.cruise=Cruise(t);diagnostic.turnWindow=TurnWindow(t);diagnosticHoldsBefore=result.leadHolds;
        diagnostic.referenceBefore=reference;diagnostic.previous=commonSpeed;
        diagnosticReferenceBefore=qref;
        diagnostic.actual=result.samples.empty()?begin:result.samples.back().actual;
        diagnostic.actualSpeed=result.samples.empty()?0:result.samples.back().speed;
        diagnostic.observed=observed;diagnostic.estimatedVelocity=observedVelocity;
        for(unsigned j=0;j<7;++j)diagnostic.feedbackAgeMs[j]=double(tick-receivedAt[j])*1000/Hz;
        if(stopped)diagnostic.reasons[Inactive]=true;
        if(!coordinated){Vec goal=FiniteGoal(t),requested=IsJoystick()?JoystickVelocity(t):Unit(goal-reference)*speed;
            if(name=="rail_hold_tcp"||name=="rail_recovery")requested={};
            diagnostic.input=Length(requested);diagnostic.requestedDirection=Unit(requested);
            if(name=="rail_hold_tcp"||name=="rail_recovery")diagnostic.input=20*v.speedScale;}
    }
    void EndDiagnostic(){
        diagnostic.common=commonSpeed;diagnostic.direction=commonDirection;diagnostic.referenceAfter=reference;
        diagnostic.referenceJoints=qref;
        if(coordinated&&diagnostic.sent&&!stopped){double dt=v.periodMs*.001;
            for(unsigned j=0;j<7;++j){double velocity=(qref[j]-diagnosticReferenceBefore[j])/dt;
                double maxV=30*Steps/std::abs(Factor[j]),maxA=v.acceleration*Steps/std::abs(Factor[j]);
                diagnostic.referenceVelocity[j]=velocity;
                diagnostic.referenceAccelerationRatio[j]=std::abs(velocity-previousReferenceVelocity[j])/(dt*maxA);
                result.referenceVelocityRatioMax=std::max(result.referenceVelocityRatioMax,std::abs(velocity)/maxV);
                result.referenceAccelerationRatioMax=std::max(result.referenceAccelerationRatioMax,diagnostic.referenceAccelerationRatio[j]);
                if(diagnostic.referenceAccelerationRatio[j]>1.01)diagnostic.reasons[ReferenceAcceleration]=true;
                previousReferenceVelocity[j]=velocity;}}
        for(unsigned j=0;j<7;++j){const Motor& m=*motors[j];diagnostic.goals[j]=Decode(j,m.goal);
            diagnostic.motorPosition[j]=Decode(j,m.planner.positionTracker.go_location);
            diagnostic.motorVelocity[j]=m.planner.positionTracker.go_velocity/Factor[j];
            diagnostic.motorCap[j]=m.config.ratedVelocity/std::abs(Factor[j]);}
        if(!coordinated){double dt=v.periodMs*.001;
            diagnostic.common=Length(reference-diagnostic.referenceBefore)/dt;
            diagnostic.direction=Unit(reference-diagnostic.referenceBefore);
            diagnostic.reasons[BaselineHold]=result.leadHolds>diagnosticHoldsBefore;
            diagnostic.reasons[InputRelease]=diagnostic.active&&IsJoystick()&&diagnostic.input==0;
            diagnostic.afterTurn=diagnostic.afterJoint=diagnostic.afterFeedback=diagnostic.allowed=diagnostic.common;}
        bool phases[]={true,diagnostic.cruise,diagnostic.turnWindow};
        if(diagnostic.dtMs>0)for(unsigned p=0;p<3;++p){
            if(phases[p]){result.phaseMs[p]+=diagnostic.dtMs;++result.phaseUpdates[p];}
            for(unsigned reason=0;reason<SlowReasonCount;++reason)
                result.slowStats[p][reason].Add(phases[p]&&diagnostic.reasons[reason],phases[p]?diagnostic.dtMs:0);}
        result.controls.push_back(diagnostic);
    }
    void ObservedMotorTick(unsigned j,const std::array<bool,3>& phases){
        Motor& m=*motors[j];int reason=ObserveMotor(m),before=m.planner.positionTracker.go_velocity;
        for(unsigned p=0;p<3;++p)if(phases[p])++result.motorReasons[p][j][reason];
        m.Tick();int after=m.planner.positionTracker.go_velocity;++result.observerChecks;
        // Independent check that the observed branch agrees with the actual
        // kernel's speed change, including integer integration plateaus.
        bool decel=reason==GoalBrake||reason==ReverseBrake||reason==CapBrake||reason==DistanceBrake;
        if((decel&&std::abs(after)>std::abs(before))||(reason==Accelerating&&std::abs(after)<std::abs(before))
            ||(reason==Cruising&&after!=before)||(reason==Locked&&after!=0))++result.observerMismatches;
    }
    Axes Actual(){Axes q;for(unsigned j=0;j<7;++j)q[j]=Decode(j,motors[j]->planner.positionTracker.go_location);return q;}
    void Send(int tick,const Axes& q,const Axes& velocity){
        diagnostic.sent=true;diagnostic.sentGoals=q;diagnostic.sentCaps=velocity;
        unsigned batch=++batchSequence;
        for(unsigned j=0;j<7;++j){int delay=int(std::llround(j*v.skewMs*Hz/1000.));
            pending.push_back({tick+delay,j,Encode(j,q[j]),int32_t(std::llround(Clamp(std::abs(velocity[j]*Factor[j]),1,30.*Steps))),batch});}
    }
    void Feedback(int tick){
        // Position messages only: the coordinator never reads the oracle's
        // internal go_velocity. No measurement timestamp is exposed to it.
        if(tick%(Hz/50)==0){Axes q=Actual();for(unsigned j=0;j<7;++j){
            int delay=int(std::llround((v.feedbackDelayMs+j*v.feedbackSkewMs)*Hz/1000));
            feedbackPending.push_back({tick+delay,j,q[j]});}}
        for(size_t i=0;i<feedbackPending.size();){auto d=feedbackPending[i];
            if(d.tick>tick){++i;continue;}
            bool lost=v.feedbackDropout&&double(tick)/Hz>=.30&&double(tick)/Hz<.44;
            if(!lost){int elapsed=tick-receivedAt[d.axis];if(elapsed>0){double rate=(d.position-observed[d.axis])*Hz/elapsed;
                    observedVelocity[d.axis]=.5*observedVelocity[d.axis]+.5*rate;}
                observed[d.axis]=d.position;receivedAt[d.axis]=tick;}
            feedbackPending.erase(feedbackPending.begin()+i);
        }
    }
    void FailCoordinated(int tick,double t,const std::string& reason){
        ++result.rejected;result.firstRejectAt=t;result.rejectReason=reason;result.brakeAt=t;
        stopped=true;pending.clear();Axes q,velocity;
        for(unsigned j=0;j<7;++j){double a=v.acceleration*Steps/std::abs(Factor[j]);
            q[j]=Clamp(observed[j]+observedVelocity[j]*std::abs(observedVelocity[j])/(2*a),Lower[j],Upper[j]);
            velocity[j]=std::abs(observedVelocity[j]);}
        // Fault fallback uses delayed observed position/estimated speed, not
        // hidden plant state. It is not a validated production STOP contract.
        Send(tick,q,velocity);
    }
    void Brake(int tick){
        if(stopped)return;stopped=true;result.brakeAt=double(tick)/Hz;pending.clear();Axes q,velocity;
        for(unsigned j=0;j<7;++j){auto& m=*motors[j];double p=m.planner.positionTracker.go_location,s=m.planner.positionTracker.go_velocity;
            double target=p+s*std::abs(s)/(2*m.config.ratedVelocityAcc);
            q[j]=Decode(j,int32_t(std::llround(target)));velocity[j]=std::abs(s/Factor[j]);}
        // Deliberately per-axis braking, with ordinary transport skew. This is
        // not the production STOP protocol or a common Cartesian braking curve.
        Send(tick,q,velocity);
    }
    Vec FiniteGoal(double t){
        if(name=="slider_short"){
            if(t<.10)return begin+Vec{6,0,0};
            if(t<.30){int n=int((t-.10)/.01+1e-7);return begin+Vec{8.+n*.3,2*std::sin(n*.6),.6*std::sin(n*.35)};}
            return finalGoal;
        }
        if(name!="slider_burst")return finalGoal;
        // Absolute targets arrive every 10 ms during this short burst. At each
        // planning tick only the latest complete pose is observed (no FIFO).
        if(t<.10)return begin+Vec{25,0,0};
        if(t<.30){int n=int((t-.10)/.01+1e-7);return begin+Vec{35.+n,12*std::sin(n*.6),6*std::sin(n*.35)};}
        return finalGoal;
    }
    Vec JoystickVelocity(double t){
        if(name=="joystick_corner"){if(t<1)return {speed,0,0};if(t<2)return {0,speed,0};return {};}
        if(name=="joystick_reverse"){if(t<1)return {speed,0,0};if(t<2)return {-speed,0,0};return {};}
        if(name=="rapid_short_reverse"){if(t<.25)return {speed,0,0};if(t<.50)return {-speed,0,0};return {};}
        if(name=="rapid_short_corner"){if(t<.25)return {speed,0,0};if(t<.50)return {0,speed,0};return {};}
        if(name=="input_expiry")return t<1.10?Vec{speed*.6,speed*.8,0}:Vec{};
        return {};
    }
    void Search(double t,Vec velocity){
        if(name!="rail_recovery")return;
        // Bounded experimental 1D allocation: at most two candidate IK per
        // motion update, cached target, 100 ms strategy interval. It is NOT a
        // certified whole-path 10%/25% recovery planner.
        if(searchIndex<0&&Margin(qref)<.10&&int(std::llround(t*1000))%100==0&&std::abs(railReference-railGoal)<.5){
            searchPose=At(orientation,reference+velocity*.1);searchSeed=qref;searchRail=railReference;
            searchBestMargin=Margin(qref);searchBestRail=railGoal;searchIndex=0;}
        if(searchIndex<0)return;
        for(int n=0;n<2&&searchIndex<6;++n,++searchIndex){double rail=Clamp(searchRail+candidateOffsets[searchIndex],-245,245);Axes q;++result.searchIk;
            if(!geometry.Inverse(searchPose,rail,searchSeed,q,false))continue;double margin=Margin(q);bool noNewShortage=true;
            for(unsigned j=0;j<6;++j){double a=std::min(q[j]-Lower[j],Upper[j]-q[j])/(Upper[j]-Lower[j]);
                double b=std::min(searchSeed[j]-Lower[j],Upper[j]-searchSeed[j])/(Upper[j]-Lower[j]);
                if(a<std::min(.10,b)-.005)noNewShortage=false;}
            if(noNewShortage&&margin>searchBestMargin+.002){searchBestMargin=margin;searchBestRail=rail;}}
        if(searchIndex>=6){railGoal=searchBestRail;result.selectedRail=railGoal;searchIndex=-1;}
    }
    void UpdateBaseline(int tick,double t,Vec actual){
        if(stopped)return;unsigned ikBefore=geometry.ik;double dt=v.periodMs*.001,h=v.horizonMs*.001;
        bool joystick=name=="joystick_corner"||name=="joystick_reverse"||name=="input_expiry"||name=="rapid_short_reverse"||name=="rapid_short_corner";
        Vec goal=FiniteGoal(t),velocity=joystick?JoystickVelocity(t):Unit(goal-reference)*speed;
        if(name=="rail_hold_tcp"||name=="rail_recovery")velocity={};
        if(joystick&&Length(velocity)==0){if(name=="input_expiry")++result.expired;Brake(tick);return;}
        Search(t,velocity);
        double railVelocity=0;
        if(name=="rail_line"){railGoal=90;railVelocity=1.5*speed;}
        if(name=="rail_hold_tcp"||name=="rail_recovery")railVelocity=20*v.speedScale;
        railVelocity=Clamp(railGoal-railReference,-railVelocity*dt,railVelocity*dt)/dt;
        // Bound reference lead, without substituting asynchronous feedback for
        // the continuous reference cursor. Holding is observable in the report.
        double leadLimit=std::max(3.,speed*.15);
        bool hold=Length(reference-actual)>leadLimit;
        if(hold){velocity={};railVelocity=0;++result.leadHolds;}
        Vec step=velocity*dt;
        if(!joystick&&Length(step)>Length(goal-reference))step=goal-reference;
        Vec next=reference+step;double nextRail=railReference+railVelocity*dt;
        Vec direction=Unit(velocity);double ahead=Length(velocity)*h;
        if(!joystick)ahead=std::min(ahead,Length(goal-next));
        Vec guide=next+direction*ahead;
        double railAhead=Clamp(railGoal-nextRail,-std::abs(railVelocity)*h,std::abs(railVelocity)*h);
        double guideRail=nextRail+railAhead;
        Axes qr,qg;
        if(!geometry.Inverse(At(orientation,next),nextRail,qref,qr)||!geometry.Inverse(At(orientation,guide),guideRail,qr,qg)){
            ++result.rejected;result.firstRejectAt=t;result.rejectReason=geometry.inverseFailure;
            result.ikPeak=std::max(result.ikPeak,geometry.ik-ikBefore);Brake(tick);return;}
        Axes newCaps=caps;double largestRate=0;
        for(unsigned j=0;j<7;++j){double rate=std::abs(qr[j]-qref[j])/dt;largestRate=std::max(largestRate,rate*std::abs(Factor[j])/Steps);
            // Hold the last approaching cap only when the reference is at its
            // final fixed target. Updating it to zero would strand the follower.
            bool terminal=!joystick&&Length(goal-next)<1e-6&&std::abs(railGoal-nextRail)<1e-6;
            if(!terminal||rate>1e-6)newCaps[j]=rate;}
        if(largestRate>30){++result.rejected;result.firstRejectAt=t;result.rejectReason="motor_rate_over_30rps";
            result.ikPeak=std::max(result.ikPeak,geometry.ik-ikBefore);Brake(tick);return;}
        if(hold){
            // Keep the previous guide and speed limits until the lag shrinks;
            // don't create an unbounded stream of ever farther virtual goals.
            result.ikPeak=std::max(result.ikPeak,geometry.ik-ikBefore);return;}
        reference=next;railReference=nextRail;qref=qr;caps=newCaps;
        Send(tick,qg,caps);result.referencePath.push_back(reference);
        result.ikPeak=std::max(result.ikPeak,geometry.ik-ikBefore);
    }
    bool IsJoystick()const{return name=="joystick_corner"||name=="joystick_reverse"||name=="input_expiry"||name=="rapid_short_reverse"||name=="rapid_short_corner";}
    void UpdateCoordinated(int tick,double t){
        if(stopped)return;unsigned ikBefore=geometry.ik;double dt=v.periodMs*.001;
        auto finish=[&](){result.ikPeak=std::max(result.ikPeak,geometry.ik-ikBefore);};
        for(unsigned j=0;j<7;++j)if(tick-receivedAt[j]>int(.080*Hz)){
            diagnostic.reasons[StaleFeedback]=true;
            ++result.feedbackFaults;FailCoordinated(tick,t,"feedback_older_than_80ms");finish();return;}
        // Never enqueue a new seven-axis batch over an incomplete older batch.
        // This is local dispatch discipline, NOT simultaneous board execution.
        if(!pending.empty()){diagnostic.reasons[BatchWait]=true;++result.batchWaits;finish();return;}
        bool joystick=IsJoystick();Vec goal=FiniteGoal(t),requested=joystick?JoystickVelocity(t):Unit(goal-reference)*speed;
        if(name=="rail_hold_tcp"||name=="rail_recovery")requested={};
        Search(t,requested);double railRequest=0;
        if(name=="rail_line"){railGoal=90;railRequest=1.5*speed;}
        if(name=="rail_hold_tcp"||name=="rail_recovery")railRequest=20*v.speedScale;
        if(std::abs(railGoal-railReference)<1e-7)railRequest=0;
        double desiredRate=Length(requested);Vec desiredDirection=Unit(requested);double desiredRailSlope=0;
        if(desiredRate>1e-8){desiredRailSlope=railRequest/desiredRate;}
        else if(railRequest>0){desiredRate=railRequest;desiredRailSlope=railGoal>railReference?1:-1;}
        diagnostic.input=desiredRate;diagnostic.requestedDirection=desiredDirection;
        if(joystick&&Length(requested)==0){desiredRate=0;
            if(!normalStopRequested){normalStopRequested=true;++result.normalStops;result.brakeAt=t;if(name=="input_expiry")++result.expired;}}
        bool moving=commonSpeed>.05;
        bool sharp=moving&&Length(desiredDirection)>0&&Length(commonDirection)>0&&Dot(desiredDirection,commonDirection)<.94&&(!experiment||experiment->stopBeforeTurn);
        bool railChange=moving&&std::abs(desiredRailSlope-commonRailSlope)>.15;
        diagnostic.reasons[TurnBrake]=sharp;diagnostic.reasons[RailChangeBrake]=railChange;
        diagnostic.reasons[InputRelease]=joystick&&Length(requested)==0;
        if(sharp||railChange){desiredRate=0;++result.turnBrakes;}
        else if(desiredRate>0){commonDirection=desiredDirection;commonRailSlope=desiredRailSlope;}
        if(!moving&&desiredRate==0&&joystick)normalStopRequested=true;
        diagnostic.afterTurn=desiredRate;
        double remaining=1e9;
        if(!joystick&&Length(commonDirection)>0){Vec delta=goal-reference;double projection=Dot(delta,commonDirection);
            // When braking for a changed target, keep the old direction until
            // speed is zero; a new distant target cannot teleport the cursor.
            if(!sharp)remaining=std::max(0.,projection);}
        if(std::abs(commonRailSlope)>1e-8)remaining=std::min(remaining,std::abs((railGoal-railReference)/commonRailSlope));
        bool idle=Length(commonDirection)<1e-8&&std::abs(commonRailSlope)<1e-8;
        Axes gradient{},curvature{};double allowed=desiredRate,pathAcceleration=400;
        if(!idle&&remaining>1e-5){double probe=std::min(.5,remaining*.5);Axes qa,qb;
            Pose pa=At(orientation,reference+commonDirection*probe),pb=At(orientation,reference+commonDirection*(2*probe));
            bool valid=geometry.Inverse(pa,railReference+commonRailSlope*probe,qref,qa)
                &&geometry.Inverse(pb,railReference+commonRailSlope*2*probe,qa,qb);
            if(!valid){diagnostic.reasons[ProbeIK]=true;FailCoordinated(tick,t,geometry.inverseFailure);finish();return;}
            for(unsigned j=0;j<7;++j){gradient[j]=(qa[j]-qref[j])/probe;curvature[j]=(qb[j]-2*qa[j]+qref[j])/(probe*probe);
                double maxV=30*Steps/std::abs(Factor[j]),maxA=v.acceleration*Steps/std::abs(Factor[j]);
                // Keep the common guide inside a bounded 300 ms window while
                // leaving enough local stopping distance. Fixed 60 ms can be
                // too short for the rail even at ordinary Cartesian speeds.
                maxV=adaptiveGuide?std::min(.75*maxV,std::max(.01,.300-dt)*maxA/.60):.75*maxV;
                if(std::abs(gradient[j])>1e-6&&maxV/std::abs(gradient[j])<diagnostic.velocityBound){
                    diagnostic.velocityBound=maxV/std::abs(gradient[j]);diagnostic.velocityAxis=int(j);}
                if(std::abs(curvature[j])>1e-6&&std::sqrt(.30*maxA/std::abs(curvature[j]))<diagnostic.curvatureBound){
                    diagnostic.curvatureBound=std::sqrt(.30*maxA/std::abs(curvature[j]));diagnostic.curvatureAxis=int(j);}
                if(std::abs(gradient[j])>1e-6)allowed=std::min(allowed,maxV/std::abs(gradient[j]));
                if(std::abs(curvature[j])>1e-6)allowed=std::min(allowed,std::sqrt(.30*maxA/std::abs(curvature[j])));
                double available=std::max(.05*maxA,.65*maxA-std::abs(curvature[j])*commonSpeed*commonSpeed);
                if(std::abs(gradient[j])>1e-6&&available/std::abs(gradient[j])<pathAcceleration)diagnostic.accelerationAxis=int(j);
                if(std::abs(gradient[j])>1e-6)pathAcceleration=std::min(pathAcceleration,available/std::abs(gradient[j]));}
        }else if(!joystick||normalStopRequested){allowed=0;}
        pathAcceleration=Clamp(pathAcceleration,1,400);lastPathAcceleration=pathAcceleration;
        diagnostic.afterJoint=allowed;diagnostic.acceleration=pathAcceleration;
        diagnostic.reasons[JointSpeed]=diagnostic.velocityBound<desiredRate-.01;
        diagnostic.reasons[CurvatureSpeed]=diagnostic.curvatureBound<desiredRate-.01;
        // Inspect each joint including stationary ones, rather than dividing
        // by its segment displacement. Tolerances are experiment parameters.
        double worstError=0;
        for(unsigned j=0;j<7;++j){double tolerance=j==6?.5:.30;worstError=std::max(worstError,std::abs(qref[j]-observed[j])/tolerance);}
        Pose feedbackPose=geometry.Forward(observed);double rotationTrace=0;
        for(unsigned k=0;k<9;++k)rotationTrace+=feedbackPose.R[k]*orientation.R[k];
        double orientationError=std::acos(Clamp((rotationTrace-1)*.5,-1,1))*180/Pi;
        worstError=std::max(worstError,std::max(Length(XYZ(feedbackPose)-reference),orientationError/.5));
        for(unsigned j=0;j<7;++j){diagnostic.ratio[j]=std::abs(qref[j]-observed[j])/(j==6?.5:.30);
            if(diagnostic.worstSource<0||diagnostic.ratio[j]>diagnostic.ratio[unsigned(diagnostic.worstSource)])diagnostic.worstSource=int(j);}
        diagnostic.tcpRatio=Length(XYZ(feedbackPose)-reference);diagnostic.orientationRatio=orientationError/.5;
        double axisWorst=diagnostic.ratio[unsigned(diagnostic.worstSource)];
        if(diagnostic.tcpRatio>axisWorst)diagnostic.worstSource=7;
        if(diagnostic.orientationRatio>std::max(axisWorst,diagnostic.tcpRatio))diagnostic.worstSource=8;
        diagnostic.worstRatio=worstError;diagnostic.tcpLag=Length(XYZ(feedbackPose)-reference);
        Vec lag=reference-XYZ(feedbackPose);diagnostic.alongLag=Dot(lag,commonDirection);
        diagnostic.crossLag=Length(lag-commonDirection*diagnostic.alongLag);
        trackingViolationStreak=worstError>12?trackingViolationStreak+1:0;
        if(trackingViolationStreak>=3){diagnostic.reasons[TrackingFault]=true;++result.trackingFaults;FailCoordinated(tick,t,"tracking_over_12x_for_3updates");finish();return;}
        bool applyFeedback=worstError>1&&(!experiment||experiment->feedbackScale);
        if(applyFeedback){allowed*=Clamp(1/worstError,.05,1);++result.leadHolds;}
        diagnostic.afterFeedback=allowed;diagnostic.feedbackFactor=applyFeedback?Clamp(1/worstError,.05,1):1;
        diagnostic.reasons[FeedbackScale]=applyFeedback;
        if(remaining<1e8)diagnostic.terminalBound=std::max(0.,std::sqrt(2*pathAcceleration*remaining)-pathAcceleration*dt);
        diagnostic.reasons[TerminalBrake]=diagnostic.terminalBound<allowed-.01;
        if(remaining<1e8)allowed=std::min(allowed,std::max(0.,std::sqrt(2*pathAcceleration*remaining)-pathAcceleration*dt));
        if(allowed<desiredRate-.01)++result.limitedUpdates;
        diagnostic.allowed=allowed;diagnostic.remaining=remaining;
        diagnostic.reasons[AccelerationUp]=allowed>commonSpeed+pathAcceleration*dt+1e-9;
        diagnostic.reasons[AccelerationDown]=allowed<std::max(0.,commonSpeed-pathAcceleration*dt)-1e-9;
        double previousSpeed=commonSpeed;commonSpeed=Clamp(allowed,std::max(0.,commonSpeed-pathAcceleration*dt),commonSpeed+pathAcceleration*dt);
        if(commonSpeed<1e-6)commonSpeed=0;
        double step=.5*(previousSpeed+commonSpeed)*dt;
        if(remaining<1e8&&step>remaining){step=remaining;commonSpeed=0;}
        Vec next=reference+commonDirection*step;double nextRail=railReference+commonRailSlope*step;Axes qr=qref,qg=qref;
        if(step>1e-9&&!geometry.Inverse(At(orientation,next),nextRail,qref,qr)){diagnostic.reasons[StepIK]=true;FailCoordinated(tick,t,geometry.inverseFailure);finish();return;}
        Axes velocity{},signedVelocity{};double guideTime=v.horizonMs*.001;
        if(experiment)guideTime=experiment->guideMs*.001;
        for(unsigned j=0;j<7;++j){double ff=(qr[j]-qref[j])/dt;
            // Feedback is compared with the reference due NOW. qr is the next
            // boundary: using it here adds an unintended dt*velocity lead.
            double correction=12*(qref[j]-observed[j])+.15*(ff-observedVelocity[j]);
            signedVelocity[j]=ff+correction;double maxV=30*Steps/std::abs(Factor[j]);
            velocity[j]=std::min(.90*maxV,std::abs(signedVelocity[j]));
            if(std::abs(signedVelocity[j])>.90*maxV)diagnostic.reasons[AxisCap]=true;
            double maxA=v.acceleration*Steps/std::abs(Factor[j]);
            if(adaptiveGuide)guideTime=std::max(guideTime,.60*velocity[j]/maxA+dt);}
        guideTime=std::min(.300,guideTime);result.guideHorizonMaxMs=std::max(result.guideHorizonMaxMs,guideTime*1000);
        diagnostic.guideMs=guideTime*1000;
        double ahead=commonSpeed*guideTime;
        if(remaining<1e8)ahead=std::min(ahead,std::max(0.,remaining-step));
        qg=qr;
        unsigned guideIkBefore=geometry.ik;
        bool guideValid=ahead<=1e-9||geometry.Inverse(At(orientation,next+commonDirection*ahead),nextRail+commonRailSlope*ahead,qr,qg);
        if(!guideValid&&boundedGuide&&geometry.inverseFailure=="branch_jump_over_12deg"){
            // A remote cumulative change is not a 20 ms branch jump. Validate
            // this same geometric path in two bounded substeps, retaining all
            // ordinary IK/limit/12-degree checks. This does not time-parameterize
            // the guide or synchronize independently executing motor boards.
            ++result.guideRetryUpdates;Axes midpoint;
            unsigned retryBefore=geometry.ik;
            guideValid=geometry.Inverse(At(orientation,next+commonDirection*(.5*ahead)),nextRail+commonRailSlope*(.5*ahead),qr,midpoint)
                &&geometry.Inverse(At(orientation,next+commonDirection*ahead),nextRail+commonRailSlope*ahead,midpoint,qg);
            result.guideRetryExtraIk+=geometry.ik-retryBefore;
            if(guideValid)++result.guideRetrySuccesses;
        }
        result.guideIkPeak=std::max(result.guideIkPeak,geometry.ik-guideIkBefore);
        if(!guideValid){
            diagnostic.reasons[GuideIK]=true;
            // An infeasible guide must not quietly pass as a feasible reference.
            FailCoordinated(tick,t,geometry.inverseFailure);finish();return;}
        for(unsigned j=0;j<7;++j){double delta=qg[j]-observed[j];
            // If the nominal guide would pull an already leading axis farther
            // away, use its current reference for the local correction.
            if(signedVelocity[j]*delta<0){diagnostic.reasons[GuideSignReset]=true;qg[j]=qref[j];}}
        reference=next;railReference=nextRail;qref=qr;caps=velocity;
        Send(tick,qg,velocity);result.referencePath.push_back(reference);finish();
    }
    void ExtraMetrics(){
        // Independent, unfiltered user-intent path. A coordinator must not earn
        // a good geometric score just by redefining its own slower/rounded path.
        std::vector<Vec> intent{begin};Vec cursor=begin;
        auto append=[&](Vec p){if(Length(p-intent.back())<1e-9)return;
            if(intent.size()>=2&&SegmentDistance(intent.back(),intent[intent.size()-2],p)<1e-8)intent.back()=p;
            else intent.push_back(p);};
        for(int ms=0;ms<int(std::llround(duration*1000));++ms){double t=ms*.001;Vec delta;
            if(IsJoystick())delta=JoystickVelocity(t)*.001;
            else if(name!="rail_hold_tcp"&&name!="rail_recovery"){Vec goal=FiniteGoal(t),d=goal-cursor;delta=Unit(d)*std::min(speed*.001,Length(d));}
            cursor=cursor+delta;append(cursor);}
        if(result.finiteTarget||name=="far_goal")append(finalGoal);
        Vec expectedEnd=result.finiteTarget||name=="far_goal"?finalGoal:cursor;
        result.intentEndError=Length(result.samples.back().actual-expectedEnd);
        Vec previous=begin;unsigned stable=0;
        for(const auto& s:result.samples){double error=Length(s.actual-intent[0]);
            for(size_t i=1;i<intent.size();++i)error=std::min(error,SegmentDistance(s.actual,intent[i-1],intent[i]));
            result.intentPathMax=std::max(result.intentPathMax,error);result.travel+=Length(s.actual-previous);previous=s.actual;
            for(unsigned j=0;j<7;++j)result.jointErrorRatio=std::max(result.jointErrorRatio,std::abs(s.referenceJoints[j]-s.joints[j])/(j==6?.5:.30));
            bool atEnd=result.finiteTarget&&Length(s.actual-finalGoal)<.1&&std::abs(s.joints[6]-railGoal)<.1&&s.speed<1&&s.t>.35;
            stable=atEnd?stable+1:0;if(stable>=100&&result.settledAt<0)result.settledAt=s.t-.099;
        }
        result.targetComplete=result.finiteTarget&&result.plannedRemaining<.01&&result.finalError<.1&&result.finalSpeed<1&&std::abs(result.railEnd-railGoal)<.1&&!result.rejected;
    }
    bool Cruise(double t)const{
        if(name=="rail_hold_tcp"||name=="rail_recovery"||name=="far_goal")return false;
        if(name=="joystick_corner"||name=="joystick_reverse")return (t>.30&&t<.80)||(t>1.30&&t<1.80);
        if(name=="input_expiry")return t>.30&&t<.80;
        if(name=="slider_burst")return t>.55&&t<.95;
        if(name=="slider_short")return t>.32&&t<.42;
        if(name=="rapid_short_reverse"||name=="rapid_short_corner")return (t>.12&&t<.20)||(t>.37&&t<.45);
        if(name=="short_line_x"||name=="short_line_xyz")return t>.12&&t<20/speed-.08;
        double length=name=="line_x"?80:60;return t>.30&&t<length/speed-.25;
    }
public:
    Simulation(const Variant& variant,unsigned f,const char* scenario,bool coordinate=false,bool brakeGuide=false,const Experiment* e=nullptr,bool splitGuide=false):v(variant),fixture(f),name(scenario),coordinated(coordinate),adaptiveGuide(brakeGuide),boundedGuide(splitGuide),experiment(e){
        Axes initial=Fixtures[f];
        if(name=="rail_recovery"){initial={45,80,70,20,35,0,0};}
        for(unsigned j=0;j<7;++j)motors[j]=std::make_unique<Motor>(v.acceleration,Encode(j,initial[j]));
        qref=Actual();observed=qref;orientation=geometry.Forward(qref);begin=reference=XYZ(orientation);result.referencePath.push_back(begin);
        speed=40*v.speedScale;finalGoal=begin+Vec{80,0,0};
        if(name=="line_xyz")finalGoal=begin+Vec{36,38.4,28.8};
        if(name=="short_line_x")finalGoal=begin+Vec{20,0,0};
        if(name=="short_line_xyz")finalGoal=begin+Vec{12,12.8,9.6};
        if(name=="slider_burst")finalGoal=begin+Vec{55,-12,8};
        if(name=="slider_short")finalGoal=begin+Vec{15,-4,2};
        if(name=="rail_line"){finalGoal=begin+Vec{0,60,0};railGoal=90;}
        if(name=="rail_hold_tcp"){finalGoal=begin;railGoal=40;duration=5;}
        if(name=="rail_recovery"){finalGoal=begin;duration=6;}
        // A remote goal deliberately tests only rolling local validity. There
        // is no endpoint precheck; failure to reject within 4s is reported,
        // never counted as successful unreachable-goal protection.
        if(name=="far_goal"){finalGoal=begin+Vec{1000,0,0};duration=4;}
        if(name=="joystick_corner"||name=="joystick_reverse")stopAt=2;
        if(name=="input_expiry")stopAt=1.10; // Last heartbeat at 1.00 s, 100 ms timeout.
        if(name=="joystick_reverse")reverseAt=1;
        if(name=="rapid_short_reverse"||name=="rapid_short_corner"){stopAt=.50;duration=3;}
        if(name=="rapid_short_reverse")reverseAt=.25;
        if(name=="line_x"||name=="line_xyz"||name=="short_line_x"||name=="short_line_xyz"||name=="slider_burst"||name=="slider_short"||name=="rail_line")
            duration=std::max(duration,Length(finalGoal-begin)/speed+2);
        if(name=="rail_hold_tcp")duration=std::max(duration,40/(20*v.speedScale)+2);
        if(name=="rail_recovery")duration=std::max(duration,60/(20*v.speedScale)+2);
        result.name=name;result.variant=v.name;result.fixture=f;result.controller=coordinated?(adaptiveGuide?"coordinated_brake_lead":"coordinated"):"baseline";result.requested=(name=="rail_hold_tcp"||name=="rail_recovery")?0:speed;
        if(experiment)result.controller=experiment->name;
        if(boundedGuide)result.controller="coordinated_brake_lead_midpoint";
        if(!coordinated)result.guideHorizonMaxMs=v.horizonMs;
        result.marginStart=Margin(initial);
    }
    Result Run(){
        Vec previous=begin;std::vector<Vec> positions;positions.push_back(begin);
        int end=int(duration*Hz),period=v.periodMs*Hz/1000;
        for(int tick=0;tick<=end;++tick){double t=double(tick)/Hz;
            Feedback(tick);
            if(tick%period==0){BeginDiagnostic(tick,t);if(coordinated)UpdateCoordinated(tick,t);
                else {Axes q=(v.feedbackDelayMs||v.feedbackSkewMs||v.feedbackDropout)?observed:Actual();UpdateBaseline(tick,t,XYZ(geometry.Forward(q)));}EndDiagnostic();}
            for(size_t i=0;i<pending.size();){if(pending[i].tick<=tick){auto d=pending[i];motors[d.axis]->Receive(d.q,d.cap);pending.erase(pending.begin()+i);}else ++i;}
            if(tick==end)break;
            std::array<bool,3> phases{true,Cruise(t),TurnWindow(t)};
            for(unsigned j=0;j<7;++j)ObservedMotorTick(j,phases);
            if((tick+1)%(Hz/1000)==0){Sample s;s.t=double(tick+1)/Hz;s.joints=Actual();Pose tcp=geometry.Forward(s.joints);s.actual=XYZ(tcp);s.reference=reference;s.railReference=railReference;
                s.referenceJoints=qref;s.commonSpeed=commonSpeed;for(unsigned j=0;j<7;++j)s.motorCaps[j]=double(motors[j]->config.ratedVelocity)/Steps;
                for(unsigned j=0;j<7;++j){s.diagnosticVelocity[j]=motors[j]->planner.positionTracker.go_velocity/Factor[j];
                    s.diagnosticGoals[j]=Decode(j,motors[j]->goal);s.diagnosticReasons[j]=ObserveMotor(*motors[j]);}
                double trace=0;for(unsigned i=0;i<9;++i)trace+=tcp.R[i]*orientation.R[i];s.orientation=std::acos(Clamp((trace-1)*.5,-1,1))*180/Pi;
                positions.push_back(s.actual);size_t n=positions.size()-1;
                if(n>=20)s.speed=Length(positions[n]-positions[n-20])/.020;
                s.cruise=Cruise(s.t);result.samples.push_back(s);
                for(unsigned j=0;j<7;++j)result.limitExcess=std::max(result.limitExcess,std::max(Lower[j]-s.joints[j],s.joints[j]-Upper[j]));
            }
        }
        double sumSquares=0,sumSpeed=0,sumSpeed2=0;unsigned countSpeed=0;
        result.speedMin=std::numeric_limits<double>::infinity();double railMin=0,railMax=0;
        for(const auto& s:result.samples){double error=std::numeric_limits<double>::infinity();
            if(result.referencePath.size()==1)error=Length(s.actual-begin);
            for(size_t i=1;i<result.referencePath.size();++i)error=std::min(error,SegmentDistance(s.actual,result.referencePath[i-1],result.referencePath[i]));
            result.pathMax=std::max(result.pathMax,error);sumSquares+=error*error;
            result.orientationMax=std::max(result.orientationMax,s.orientation);result.referenceLeadMax=std::max(result.referenceLeadMax,Length(s.reference-s.actual));
            railMin=std::min(railMin,s.joints[6]);railMax=std::max(railMax,s.joints[6]);
            if(s.cruise){sumSpeed+=s.speed;sumSpeed2+=s.speed*s.speed;++countSpeed;result.speedMin=std::min(result.speedMin,s.speed);result.speedMax=std::max(result.speedMax,s.speed);}
        }
        if(countSpeed){result.speedMean=sumSpeed/countSpeed;double variance=std::max(0.,sumSpeed2/countSpeed-result.speedMean*result.speedMean);
            result.speedCv=result.speedMean>1e-9?100*std::sqrt(variance)/result.speedMean:0;}else result.speedMin=0;
        result.pathRms=std::sqrt(sumSquares/result.samples.size());const auto& last=result.samples.back();
        bool finite=name=="line_x"||name=="line_xyz"||name=="slider_burst"||name=="rail_line"||name=="rail_hold_tcp"||name=="rail_recovery"||name=="short_line_x"||name=="short_line_xyz"||name=="slider_short";
        result.finiteTarget=finite;
        result.finalError=Length(last.actual-(finite?finalGoal:last.reference));result.finalSpeed=last.speed;
        result.plannedRemaining=finite||name=="far_goal"?Length(reference-finalGoal):0;
        result.railEnd=last.joints[6];result.railReferenceEnd=railReference;result.railRange=railMax-railMin;result.marginEnd=Margin(last.joints);
        if(reverseAt>0&&(result.brakeAt<0||result.brakeAt>=reverseAt)){size_t at=size_t(reverseAt*1000)-1;Vec p=result.samples[at].actual;double forward=0;
            for(const auto& s:result.samples)if(s.t>=reverseAt&&s.t<stopAt){double displacement=Dot(s.actual-p,reverseDirection);forward=std::max(forward,displacement);
                if(result.reverseDelay<0&&s.t>=reverseAt+.020){size_t n=size_t(std::llround(s.t*1000))-1;
                    if(n>=20&&Dot(s.actual-result.samples[n-20].actual,reverseDirection)<-.005)result.reverseDelay=(s.t-reverseAt)*1000;}}
            result.reverseOvershoot=forward;}
        if(stopAt>0&&(result.brakeAt<0||result.brakeAt>=stopAt)){size_t at=size_t(std::llround(stopAt*1000))-1;Vec lastPoint=result.samples[at].actual;unsigned stable=0;
            for(size_t i=at+1;i<result.samples.size();++i){const auto& s=result.samples[i];result.stopTravel+=Length(s.actual-lastPoint);lastPoint=s.actual;
                stable=s.speed<1?stable+1:0;if(stable>=100&&result.stopDelay<0)result.stopDelay=(s.t-stopAt-.099)*1000;}}
        result.ik=geometry.ik;result.fk=geometry.fk;ExtraMetrics();
        Require(result.observerMismatches==0,"motor observer disagrees with unchanged kernel");return result;
    }
};

void ModelChecks(){
    Require(std::abs(SegmentDistance({1,2,0},{0,0,0},{2,0,0})-2)<1e-12,"distance metric");
    for(unsigned j=0;j<7;++j)Require(std::abs(Decode(j,Encode(j,Fixtures[1][j]))-Fixtures[1][j])<=.51/std::abs(Factor[j]),"units roundtrip");
    Geometry g;for(auto q:Fixtures){Axes recovered;Pose p=g.Forward(q);Require(g.Inverse(p,q[6],q,recovered),"fixture IK roundtrip");
        Require(Length(XYZ(g.Forward(recovered))-XYZ(p))<.1,"fixture FK residual");}
    Motor m(30,0);m.Receive(10*Steps,Steps);for(int i=0;i<4000;++i)m.Tick();int before=m.planner.positionTracker.go_velocity;
    Require(before>0,"tracker accelerates");m.Receive(20*Steps,Steps);m.Tick();Require(m.planner.positionTracker.go_velocity>=before,"same-mode target preserves motion");
    m.Receive(-20*Steps,Steps);m.Tick();Require(m.planner.positionTracker.go_velocity>0,"reverse cannot change velocity instantly");
}
void WriteJson(std::ostream& out,const Result& r){
    out<<std::setprecision(10)<<"{\"case\":\""<<r.name<<"\",\"variant\":\""<<r.variant<<"\",\"controller\":\""<<r.controller<<"\",\"fixture\":"<<r.fixture;
#define FIELD(key,value) out<<",\"" key "\":"<<(value)
    FIELD("path_max_mm",r.pathMax);FIELD("path_rms_mm",r.pathRms);FIELD("orientation_max_deg",r.orientationMax);
    FIELD("reference_lag_max_mm",r.referenceLeadMax);FIELD("cruise_speed_mean_mm_s",r.speedMean);FIELD("cruise_speed_cv_pct",r.speedCv);
    FIELD("cruise_speed_min_mm_s",r.speedMin);FIELD("cruise_speed_max_mm_s",r.speedMax);FIELD("requested_speed_mm_s",r.requested);
    FIELD("final_error_mm",r.finalError);FIELD("final_speed_mm_s",r.finalSpeed);FIELD("reverse_delay_ms",r.reverseDelay);
    FIELD("reverse_extra_forward_mm",r.reverseOvershoot);FIELD("stop_delay_ms",r.stopDelay);FIELD("stop_travel_mm",r.stopTravel);
    FIELD("rail_final_mm",r.railEnd);FIELD("rail_reference_final_mm",r.railReferenceEnd);FIELD("rail_range_mm",r.railRange);
    FIELD("minimum_arm_margin_start",r.marginStart);FIELD("minimum_arm_margin_end",r.marginEnd);FIELD("limit_excess_deg_or_mm",r.limitExcess);
    FIELD("rail_selected_target_mm",r.selectedRail);FIELD("ik_calls",r.ik);FIELD("fk_external_calls",r.fk);FIELD("max_ik_calls_per_update",r.ikPeak);
    FIELD("rejected_updates",r.rejected);FIELD("lead_hold_updates",r.leadHolds);FIELD("rail_search_ik_calls",r.searchIk);FIELD("expired_input_stops",r.expired);
    FIELD("first_reject_at_s",r.firstRejectAt);FIELD("brake_at_s",r.brakeAt);FIELD("planned_remaining_mm",r.plannedRemaining);
    FIELD("intent_path_max_mm",r.intentPathMax);FIELD("intent_end_error_mm",r.intentEndError);FIELD("actual_travel_mm",r.travel);
    FIELD("settled_at_s",r.settledAt);FIELD("joint_tracking_ratio_max",r.jointErrorRatio);
    FIELD("guide_horizon_max_ms",r.guideHorizonMaxMs);
    FIELD("reference_velocity_ratio_max",r.referenceVelocityRatioMax);FIELD("reference_acceleration_ratio_max",r.referenceAccelerationRatioMax);
    FIELD("limited_updates",r.limitedUpdates);FIELD("turn_braking_updates",r.turnBrakes);FIELD("feedback_faults",r.feedbackFaults);
    FIELD("batch_wait_updates",r.batchWaits);FIELD("coordinated_normal_stops",r.normalStops);
    FIELD("tracking_faults",r.trackingFaults);
    FIELD("guide_retry_updates",r.guideRetryUpdates);FIELD("guide_retry_successes",r.guideRetrySuccesses);
    FIELD("guide_retry_extra_ik",r.guideRetryExtraIk);FIELD("guide_ik_peak",r.guideIkPeak);
    FIELD("finite_target",r.finiteTarget?"true":"false");FIELD("target_complete",r.targetComplete?"true":"false");
    out<<",\"reject_reason\":\""<<r.rejectReason<<"\"";
#undef FIELD
    out<<",\"diagnostics\":{\"observer_checks\":"<<r.observerChecks<<",\"observer_mismatches\":"<<r.observerMismatches<<",\"control_phases\":{";
    for(unsigned p=0;p<3;++p){if(p)out<<",";out<<"\""<<PhaseNames[p]<<"\":{\"observed_ms\":"<<r.phaseMs[p]<<",\"updates\":"<<r.phaseUpdates[p]<<",\"reasons\":{";
        for(unsigned i=0;i<SlowReasonCount;++i){if(i)out<<",";const auto& s=r.slowStats[p][i];out<<"\""<<SlowReasonNames[i]<<"\":{\"updates\":"<<s.count<<",\"ms\":"<<s.ms<<",\"longest_ms\":"<<s.longestMs<<"}";}
        out<<"}}";}
    out<<"},\"motor_phases\":{";
    for(unsigned p=0;p<3;++p){if(p)out<<",";out<<"\""<<PhaseNames[p]<<"\":{";
        for(unsigned j=0;j<7;++j){if(j)out<<",";out<<"\"J"<<j+1<<"\":{";
            for(unsigned i=0;i<MotorReasonCount;++i){if(i)out<<",";out<<"\""<<MotorReasonNames[i]<<"\":"<<r.motorReasons[p][j][i]*1000./Hz;}out<<"}";}out<<"}";}
    out<<"}}";
    out<<"}";
}
void ControlHeader(std::ostream& out){
    out<<"case,variant,fixture,controller,t_s,dt_ms,active,cruise_window,turn_to_release,sent,input_rate,after_turn,after_joint,after_feedback,allowed,previous_common,common,acceleration,velocity_bound,velocity_axis,curvature_bound,curvature_axis,acceleration_axis,terminal_bound,feedback_factor,worst_ratio,worst_source,tcp_ratio,orientation_ratio,tcp_lag_mm,along_lag_mm,cross_lag_mm,guide_ms,remaining_mm,actual_speed_20ms";
    for(const char* prefix:{"direction","requested","actual","reference_before","reference_after"})for(const char* xyz:{"x","y","z"})out<<","<<prefix<<"_"<<xyz;
    for(unsigned i=0;i<SlowReasonCount;++i)out<<",reason_"<<SlowReasonNames[i];
    for(unsigned j=0;j<7;++j)for(const char* key:{"observed","estimated_velocity","feedback_age_ms","tracking_ratio","qref","goal_before_dispatch","motor_position","oracle_velocity","cap_before_dispatch","sent_goal","sent_cap","reference_velocity","reference_acceleration_ratio"})out<<",j"<<j+1<<"_"<<key;
    out<<"\n";
}
void WriteControls(std::ostream& out,const Result& r){
    for(const auto& d:r.controls){out<<r.name<<","<<r.variant<<","<<r.fixture<<","<<r.controller<<","<<d.t<<","<<d.dtMs<<","<<d.active<<","<<d.cruise<<","<<d.turnWindow<<","<<d.sent
        <<","<<d.input<<","<<d.afterTurn<<","<<d.afterJoint<<","<<d.afterFeedback<<","<<d.allowed<<","<<d.previous<<","<<d.common<<","<<d.acceleration
        <<","<<d.velocityBound<<","<<d.velocityAxis+1<<","<<d.curvatureBound<<","<<d.curvatureAxis+1<<","<<d.accelerationAxis+1<<","<<d.terminalBound
        <<","<<d.feedbackFactor<<","<<d.worstRatio<<","<<d.worstSource+1<<","<<d.tcpRatio<<","<<d.orientationRatio<<","<<d.tcpLag<<","<<d.alongLag<<","<<d.crossLag<<","<<d.guideMs<<","<<d.remaining<<","<<d.actualSpeed;
        for(Vec p:{d.direction,d.requestedDirection,d.actual,d.referenceBefore,d.referenceAfter})out<<","<<p.x<<","<<p.y<<","<<p.z;
        for(bool reason:d.reasons)out<<","<<int(reason);
        for(unsigned j=0;j<7;++j)out<<","<<d.observed[j]<<","<<d.estimatedVelocity[j]<<","<<d.feedbackAgeMs[j]<<","<<d.ratio[j]<<","<<d.referenceJoints[j]<<","<<d.goals[j]<<","<<d.motorPosition[j]<<","<<d.motorVelocity[j]<<","<<d.motorCap[j]<<","<<d.sentGoals[j]<<","<<d.sentCaps[j]<<","<<d.referenceVelocity[j]<<","<<d.referenceAccelerationRatio[j];
        out<<"\n";}
}
}
int main(int argc,char** argv){
    try{
        Require(argc==2||(argc==3&&(std::string(argv[2])=="research"||std::string(argv[2])=="guide-budget")),"usage: cartesian_follow_lite_test <output-directory> [research|guide-budget]");ModelChecks();std::string folder=argv[1];
        bool research=argc==3&&std::string(argv[2])=="research",guideBudget=argc==3&&std::string(argv[2])=="guide-budget";
        std::ofstream json(folder+"/results.json"),trace(folder+"/traces.csv"),controls(folder+"/control_updates.csv"),motorTrace(folder+"/motor_observations.csv");
        Require(bool(json)&&bool(trace)&&bool(controls)&&bool(motorTrace),"open report files");
        ControlHeader(controls);controls<<std::setprecision(10);
        motorTrace<<"case,variant,fixture,controller,t_s,speed_20ms_mm_s,x_mm,y_mm,z_mm,common_path_speed";
        for(unsigned j=0;j<7;++j)motorTrace<<",j"<<j+1<<"_position,j"<<j+1<<"_oracle_velocity,j"<<j+1<<"_goal,j"<<j+1<<"_cap,j"<<j+1<<"_reason";
        motorTrace<<"\n"<<std::setprecision(10);
        trace<<"case,variant,fixture,t_s,x_mm,y_mm,z_mm,ref_x_mm,ref_y_mm,ref_z_mm,speed_20ms_mm_s,rail_mm,rail_ref_mm,orientation_deg,cruise";
        for(unsigned j=0;j<6;++j)trace<<",j"<<j+1<<"_deg";
        trace<<",controller,common_path_speed";for(unsigned j=0;j<7;++j)trace<<",qref"<<j+1<<",cap"<<j+1<<"_rps";
        trace<<"\n"<<std::setprecision(10);
        json<<"{\"model_checks\":true,\"hardware\":false,\"sample_ms\":1,\"speed_window_ms\":20,\"results\":[\n";bool first=true;unsigned runs=0;
        for(unsigned controller=0;controller<(guideBudget?2u:(research?8u:3u));++controller)for(const auto& variant:Variants)for(unsigned f=0;f<3;++f)for(const auto* name:Cases){
            if((research||guideBudget)&&(!ResearchVariant(variant.name)||!ResearchCase(name)))continue;
            // Recovery has its own near-limit fixture; do not count it three times.
            if(std::string(name)=="rail_recovery"&&f!=0)continue;
            Result r=Simulation(variant,f,name,guideBudget||research||controller!=0,guideBudget||(!research&&controller==2),research?&Experiments[controller]:nullptr,guideBudget&&controller==1).Run();if(!first)json<<",\n";first=false;WriteJson(json,r);++runs;
            if(research||guideBudget||f==0||r.variant=="base60")WriteControls(controls,r);
            if((f==0||r.variant=="base60")&&(r.name=="rail_line"||r.name=="rail_hold_tcp"||r.name=="rapid_short_reverse"||r.name=="rapid_short_corner"))
                for(const auto& s:r.samples){motorTrace<<r.name<<","<<r.variant<<","<<f<<","<<r.controller<<","<<s.t<<","<<s.speed<<","<<s.actual.x<<","<<s.actual.y<<","<<s.actual.z<<","<<s.commonSpeed;
                    for(unsigned j=0;j<7;++j)motorTrace<<","<<s.joints[j]<<","<<s.diagnosticVelocity[j]<<","<<s.diagnosticGoals[j]<<","<<s.motorCaps[j]*Steps/std::abs(Factor[j])<<","<<MotorReasonNames[unsigned(s.diagnosticReasons[j])];motorTrace<<"\n";}
            // Baseline and sensitivity traces, sampled every 5 ms. Metrics above
            // use every 1 ms FK sample, not just these plotting rows.
            if(research||guideBudget||f==0||r.variant=="base60")for(size_t i=4;i<r.samples.size();i+=5){const auto& s=r.samples[i];trace<<r.name<<","<<r.variant<<","<<f<<","<<s.t<<","<<s.actual.x<<","<<s.actual.y<<","<<s.actual.z
                <<","<<s.reference.x<<","<<s.reference.y<<","<<s.reference.z<<","<<s.speed<<","<<s.joints[6]<<","<<s.railReference<<","<<s.orientation<<","<<int(s.cruise);
                for(unsigned j=0;j<6;++j)trace<<","<<s.joints[j];trace<<","<<r.controller<<","<<s.commonSpeed;
                for(unsigned j=0;j<7;++j)trace<<","<<s.referenceJoints[j]<<","<<s.motorCaps[j];trace<<"\n";}
            std::printf("%s %s f%u %-18s path=%7.3fmm speed=%7.2f CV=%6.2f%% reject=%u rail=%7.2f\n",r.controller.c_str(),variant.name,f,name,r.pathMax,r.speedMean,r.speedCv,r.rejected,r.railEnd);
        }
        json<<"\n],\"run_count\":"<<runs<<"}\n";Require(bool(json)&&bool(trace)&&bool(controls)&&bool(motorTrace),"write reports");
        std::printf("model checks passed; %u experimental runs; no hardware or MCU timing\n",runs);return 0;
    }catch(const std::exception& e){std::fprintf(stderr,"MODEL ERROR: %s\n",e.what());return 2;}
}
