#ifndef DUMMY_BENCH_PROFILE_H
#define DUMMY_BENCH_PROFILE_H

#include <stdint.h>

/* C linkage keeps the FreeRTOS C task-switch hook independent of C++ headers. */
#ifdef __cplusplus
extern "C" {
#endif
extern volatile uint32_t benchProfileWindowActive;
extern volatile uint32_t benchOledSessionActive;
void BenchProfileSwitchedIn(void* task);
#ifdef __cplusplus
}

/* FreeRTOSConfig can be included inside an enclosing extern "C" block. Keep
 * the C++ adapter API's linkage stable regardless of that include order. */
extern "C++" {
namespace BenchProfile {
constexpr unsigned MaxWork = 5;
constexpr unsigned TaskCount = 7;
constexpr unsigned OledStageCount = 5;

enum OledStage : unsigned {
    OLED_IMU = 0,
    OLED_SERVICE = 1,
    OLED_DRAW = 2,
    OLED_SEND = 3,
    OLED_OTHER = 4
};

enum Flag : uint32_t {
    NESTED_BEGIN = 1U << 0,         // Begin refused; the existing window is intact.
    INACTIVE_SWITCH = 1U << 1,     // Direct SwitchAt called without an open window.
    INACTIVE_END = 1U << 2,        // End returned an empty window; no window was open.
    INVALID_MAPPING = 1U << 3,     // A configured task is missing or duplicated.
    SET_TASKS_WHILE_ACTIVE = 1U << 4, // Mapping change refused; old mapping retained.
    INVALID_OLED_STAGE = 1U << 5     // Valid-frame stage change refused.
};

struct Window {
    uint32_t cycles = 0;
    uint32_t resident[TaskCount]{};
    uint32_t oledResident[OledStageCount]{};
    uint32_t oledElapsed[OledStageCount]{};
    uint32_t displayResident[OledStageCount]{};
    uint32_t displayElapsed[OledStageCount]{};
    uint32_t flags = 0;
};

struct OledToken {
    uint32_t generation = 0;
    uint32_t cycles = 0;
    bool skip = false;
    bool rateLimited = false;
    uint8_t lane = 0;
};

struct OledSummary {
    uint32_t begun = 0;
    uint32_t ended = 0;
    uint32_t skipped = 0;
    uint32_t lateFrames = 0;
    uint32_t maxLateTicks = 0;
    uint32_t overrunFrames = 0;
    uint32_t maxFrameCycles = 0;
    uint32_t renderBegun = 0;
    uint32_t rateLimited = 0;
    uint32_t minRenderGapTicks = 0;
    uint32_t maxRenderGapTicks = 0;
    uint32_t maxLoopGapTicks = 0;
};

/* Buckets: command, realtime, OLED worker, RGB, USB, display, other. Selected-task
 * residency intervals and INCLUDE interrupts and profiling overhead. They are
 * not exclusive task CPU times. Begin/End/SetTasks/Reset require a short caller
 * critical section; SwitchAt is normally called by the RTOS context switch.
 * Timestamps are raw DWT values; each window must be shorter than 2^32 cycles.
 */
void SetTasks(void* command, void* realtime, void* oled, void* rgb, void* usb, void* display);
// Bits 0..5 mean the corresponding non-null, unique task mapping is valid.
uint32_t MapMask();
bool Begin(uint32_t now, void* currentTask);
void SwitchAt(uint32_t now, void* nextTask);
Window End(uint32_t now);
// Discard any window/misuse flags, retaining the configured task mapping.
void Reset();

/* Caller holds a short critical section for every OLED operation. Session
 * lifecycle changes occur OUTSIDE a profile window. A session spans the job,
 * while individual windows may cover only part of a frame. Elapsed stages
 * include preemption; resident stages include only the selected OLED task,
 * including interrupts. Unmarked/frame-external time belongs to OLED_OTHER.
 * Reset() above deliberately retains this independently managed session.
 */
void OledSessionReset();
// mode 0: normal; 1: skip drawing/sending; 2: allow render opportunities no
// faster than renderPeriodTicks. Modes 3/4 split service/display and gate only
// display frame beginnings. This gates frame beginnings, not completed
// panel updates. Mode-2 periods must be in 1..0x7fffffff ticks; a session and
// its actual-tick gaps must remain below half the uint32 range. Invalid mode
// or period leaves the session off.
void OledSessionStart(unsigned mode, uint32_t clockHz, uint32_t renderPeriodTicks = 0);
void OledSessionStop();
OledSummary OledSnapshot();
OledSummary OledDisplaySnapshot();
OledToken OledFrameBegin(uint32_t now, uint32_t actualTick, uint32_t plannedTick);
OledToken OledDisplayFrameBegin(uint32_t now, uint32_t actualTick, uint32_t plannedTick);
void OledStageAt(const OledToken& token, unsigned stage, uint32_t now);
void OledFrameEnd(const OledToken& token, uint32_t now);
}
// Implemented by the task owner. Start/Stop may wait and must run outside a
// critical section; Busy is a nonblocking admission guard. Snapshot reports
// latched session results, including the actual restoration and applied priority.
namespace OledDiagnostic {
struct RuntimeSnapshot {
    uint32_t overwritten = 0, maxSnapshotAgeTicks = 0, flags = 0, restored = 1;
    uint32_t priority = 0, displayStackFree = 0;
    uint32_t maxSendGapTicks = 0;
};
bool Start(unsigned mode);
bool Stop();
bool Busy();
RuntimeSnapshot Snapshot();
}
}
#endif
#endif
