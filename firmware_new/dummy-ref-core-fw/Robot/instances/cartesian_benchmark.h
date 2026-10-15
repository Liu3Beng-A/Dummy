#pragma once
#include "../algorithms/kinematic/6dof_kinematic.h"
#include <cstdint>

// Pure bounded component benchmark. No robot, CAN, actuator, Flash or RTOS API.
namespace CartBench {
constexpr unsigned MaxSamples = 128, MaxWork = 192;
enum Operation : uint8_t { FK, IK, ROUND_TRIP, TCP_SELECT, RAIL_CANDIDATE, UNREACHABLE };
enum Flag : uint8_t { COMPLETE, BUDGET, CANCELLED, ABORTED };
struct Config {
    uint8_t operation = ROUND_TRIP, fixture = 0;
    uint16_t work = 1, samples = 100;
    uint32_t budgetUs = 20000;
};
struct Sample {
    uint32_t cycles = 0, ticks = 0, checksum = 0;
    uint16_t work = 0, fk = 0, ik = 0, valid = 0, selected = 0, failed = 0;
    uint8_t flag = COMPLETE;
};
struct Hooks {
    void* context;
    uint32_t (*cycles)(void*);
    uint32_t (*ticks)(void*); // Independent millisecond timebase, for sanity only.
    uint8_t (*checkpoint)(void*); // COMPLETE=continue; CANCELLED/ABORTED=stop.
    uint32_t clockHz;
};
bool Valid(const Config& config);
class Compute {
public:
    Compute();
    bool Prepare(const Config& config); // Setup FK excluded from sample timing.
    Sample Measure(const Config& config, const Hooks& hooks);
private:
    bool prepared = false;
    uint8_t preparedFixture = 0;
    DOF6Kinematic solver;
    DOF6Kinematic::Joint6D_t seed{};
    DOF6Kinematic::Pose6D_t flange{}, tool{};
};
}

// Firmware adapter, implemented with the Cartesian command owner. Heavy work
// runs only in the normal-priority command task; requests never send motor data.
bool CartesianBenchmarkActive();
void CartesianBenchmarkPoll();
// Serializes calibration entry with diagnostic reservation, without holding a
// critical section over calibration dispatch/printing.
bool CartesianBenchmarkEnterCalibration();
void CartesianBenchmarkLeaveCalibration();
