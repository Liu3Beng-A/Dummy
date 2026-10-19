#pragma once
#include "cartesian_benchmark.h"
#include "cartesian_cycle_producer.h"
#include "cartesian_planner.h"
#include <new>

// Synthetic complete-chain diagnostic only. No robot, CAN, Flash or RTOS API.
namespace CartCycle {
constexpr unsigned MaxSamples=160;
enum Scenario : uint8_t { HOLD, LINE_X, LINE_XYZ, SLIDER_BURST, SLIDER_SMOOTH,
    JOYSTICK_REVERSE, JOYSTICK_XYZ, RAIL_LINE, RAIL_RECOVERY, IK_LOSS,
    SEG_REJECT, UNREACHABLE, INVALID_INPUT, SCENARIO_COUNT };
enum Reason : uint16_t { OK, IK, BOUNDS, SEGMENT, IK_BUDGET, BRAKE,
    INVALID, INVALID_CONFIG, CANCEL, ENVIRONMENT, TIME_BUDGET, NOT_PREPARED,
    GEOMETRY, WIRE, DYNAMIC, ADMISSION, INJECTED_IK, INJECTED_SEGMENT };
enum Flag : uint16_t { EMITTED=1, PLANNED=2, ACCEPTED=4, FALLBACK=8,
    BRAKING=16, RAIL_SEARCH=32, FATAL=64, CANCELLED=128, ABORTED=256,
    BUDGET=512, INJECTED=1024, TCP_VALID=2048 };
struct Config {
    uint8_t scenario=HOLD,fixture=0,variant=1;
    uint16_t samples=100;
    uint32_t budgetUs=20000;
};
struct Frame {
    uint32_t plannedRtos=0,startRtos=0,endRtos=0,cycles=0,windowCycles=0,checksum=0;
    uint16_t ik=0,fk=0,reason=OK,flags=0;
    float tcp[3]{},intent[3]{},railTarget=0,margin=0;
    StreamProtocol::Axis axes[7]{};
};
static_assert(sizeof(Frame)==148,"Reuse bounded existing diagnostic storage");
struct Summary {
    uint32_t updates=0,emitted=0,accepted=0,fallback=0,braking=0,fatal=0;
    uint32_t ik=0,fk=0,segments=0,rejected=0,curveTrials=0,brakeTrials=0;
    uint16_t ikPeak=0,fkPeak=0,firstReason=OK;
};
bool Valid(const Config& config);
// Same integer admission predicate as MotorStream::Curve::Build. Only the
// bounded Bernstein controls are checked here, never its 400-tick execution.
bool AdmitAxis(const StreamProtocol::Axis& from,const StreamProtocol::Axis& to,
               int32_t velocityLimit,int32_t accelerationLimit,int32_t home=0);
class Compute final : private CartCycleKernel::Port {
public:
    Compute();
    bool Prepare(const Config& config);
    Frame Measure(const CartBench::Hooks& hooks);
    const Summary& Totals()const{return summary;}
    const StreamProtocol::Record& Last()const{return committed;}
private:
    using State=CartCycleKernel::State;
    using Vec=CartCycleKernel::Vec;
    using Pose=DOF6Kinematic::Pose6D_t;
    alignas(CartCycleKernel::Producer) unsigned char producerMemory[sizeof(CartCycleKernel::Producer)];
    CartCycleKernel::Producer* producer=nullptr;
    StreamProtocol::Record committed{};
    DOF6Kinematic solver;
    Pose tool{},orientation{};
    Vec start{};
    Config config{};
    Summary summary{};
    const CartBench::Hooks* hooks=nullptr;
    uint32_t beginCycles=0,index=0;
    uint16_t firstReason=OK,stopFlags=0,currentIK=0,currentFK=0;
    bool prepared=false,forceIK=false,rejectOnce=false;
    bool Check();
    bool Fail(Reason reason);
    bool Forward(const State& state,Pose& tcp);
    bool Inverse(Vec tcp,double rail,const State& seed,State& goal)override;
    bool Segment(const StreamProtocol::Record& from,const StreamProtocol::Record& to)override;
    CartCycleKernel::Input InputAt();
};
} // namespace CartCycle
