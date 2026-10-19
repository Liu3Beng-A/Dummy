#include "bench_profile.h"

#if defined(BENCH_PROFILE_TEST_CLOCK)
// Host tests supply a deterministic clock; this path is absent from firmware.
extern "C" uint32_t BenchProfileTestCycles(void);
#elif defined(STM32F405xx)
#include "stm32f405xx.h"
#endif

extern "C" {
volatile uint32_t benchProfileWindowActive = 0;
volatile uint32_t benchOledSessionActive = 0;
}

namespace {
void* tasks[6]{};
uint32_t mapping = 0;
uint32_t began = 0, last = 0, flags = 0, inactiveFlags = 0;
uint32_t resident[BenchProfile::TaskCount]{};
unsigned selected = BenchProfile::TaskCount - 1;
unsigned oledMode = 0;
uint32_t oledRenderPeriodTicks = 0;
struct Lane {
    uint32_t resident[BenchProfile::OledStageCount]{};
    uint32_t elapsed[BenchProfile::OledStageCount]{};
    unsigned stage = BenchProfile::OLED_OTHER;
    uint32_t generation = 0, overrunCycles = 0, lastRenderTick = 0, lastLoopTick = 0;
    bool haveRender = false, haveLoop = false, frameOpen = false;
    BenchProfile::OledSummary summary{};
};
Lane lanes[2]{};

void InvalidateOledToken(Lane& lane) {
    ++lane.generation;
    if (!lane.generation) ++lane.generation;
    lane.frameOpen = false;
}
bool ValidOledToken(const BenchProfile::OledToken& token) {
    return benchOledSessionActive && token.lane < 2U
        && lanes[token.lane].frameOpen && token.generation == lanes[token.lane].generation;
}
void ClearLaneWindow(Lane& lane) {
    for (unsigned i = 0; i < BenchProfile::OledStageCount; ++i)
        lane.resident[i] = lane.elapsed[i] = 0;
}
void ResetLane(Lane& lane) {
    InvalidateOledToken(lane);
    lane.stage = BenchProfile::OLED_OTHER;
    lane.overrunCycles = lane.lastRenderTick = lane.lastLoopTick = 0;
    lane.haveRender = lane.haveLoop = false;
    lane.summary = BenchProfile::OledSummary{};
}

unsigned Bucket(void* task) {
    for (unsigned i = 0; i < 6; ++i)
        if ((mapping & (1U << i)) && tasks[i] == task) return i;
    return BenchProfile::TaskCount - 1;
}
void Settle(uint32_t now) {
    const uint32_t delta = now - last;
    resident[selected] += delta;
    if (benchOledSessionActive) {
        // Two independent elapsed clocks; neither lane steals the other's phase.
        lanes[0].elapsed[lanes[0].stage] += delta;
        lanes[1].elapsed[lanes[1].stage] += delta;
        if (selected == 2) lanes[0].resident[lanes[0].stage] += delta;
        if (selected == 5) lanes[1].resident[lanes[1].stage] += delta;
    }
    last = now;
}
}

namespace BenchProfile {
void SetTasks(void* command, void* realtime, void* oled, void* rgb, void* usb, void* display) {
    if (benchProfileWindowActive) {
        flags |= SET_TASKS_WHILE_ACTIVE;
        return;
    }
    void* supplied[6] = {command, realtime, oled, rgb, usb, display};
    mapping = 0;
    for (unsigned i = 0; i < 6; ++i) {
        tasks[i] = supplied[i];
        bool unique = supplied[i] != nullptr;
        for (unsigned j = 0; j < 6; ++j)
            if (j != i && supplied[i] == supplied[j]) unique = false;
        if (unique) mapping |= 1U << i;
    }
}

uint32_t MapMask() { return mapping; }

bool Begin(uint32_t now, void* currentTask) {
    if (benchProfileWindowActive) {
        flags |= NESTED_BEGIN;
        return false;
    }
    began = last = now;
    selected = Bucket(currentTask);
    for (unsigned i = 0; i < TaskCount; ++i) resident[i] = 0;
    ClearLaneWindow(lanes[0]); ClearLaneWindow(lanes[1]);
    flags = inactiveFlags | (mapping == 0x3FU ? 0U : uint32_t(INVALID_MAPPING));
    inactiveFlags = 0;
    // Publish only after all tracking state is initialized.
    benchProfileWindowActive = 1;
    return true;
}

void SwitchAt(uint32_t now, void* nextTask) {
    if (!benchProfileWindowActive) {
        inactiveFlags |= INACTIVE_SWITCH;
        return;
    }
    Settle(now);
    selected = Bucket(nextTask);
}

Window End(uint32_t now) {
    Window result{};
    if (!benchProfileWindowActive) {
        result.flags = inactiveFlags | INACTIVE_END;
        inactiveFlags = 0;
        return result;
    }
    // Close first; no callback after this point belongs to the window.
    benchProfileWindowActive = 0;
    Settle(now);
    result.cycles = now - began;
    for (unsigned i = 0; i < TaskCount; ++i) result.resident[i] = resident[i];
    for (unsigned i = 0; i < OledStageCount; ++i) {
        result.oledResident[i] = lanes[0].resident[i];
        result.oledElapsed[i] = lanes[0].elapsed[i];
        result.displayResident[i] = lanes[1].resident[i];
        result.displayElapsed[i] = lanes[1].elapsed[i];
    }
    result.flags = flags;
    return result;
}

void Reset() {
    benchProfileWindowActive = 0;
    began = last = flags = inactiveFlags = 0;
    selected = TaskCount - 1;
    for (unsigned i = 0; i < TaskCount; ++i) resident[i] = 0;
    ClearLaneWindow(lanes[0]); ClearLaneWindow(lanes[1]);
}

void OledSessionReset() {
    benchOledSessionActive = 0;
    ResetLane(lanes[0]); ResetLane(lanes[1]);
    oledMode = 0; oledRenderPeriodTicks = 0;
}

void OledSessionStart(unsigned mode, uint32_t clockHz, uint32_t renderPeriodTicks) {
    OledSessionReset();
    // Round up: a partial cycle cannot classify a frame below its threshold late.
    lanes[0].overrunCycles = clockHz / 50U + (clockHz % 50U != 0U ? 1U : 0U);
    lanes[1].overrunCycles = clockHz / 10U + (clockHz % 10U != 0U ? 1U : 0U);
    if (mode > 4U || (mode >= 2U
        && (!renderPeriodTicks || renderPeriodTicks >= 0x80000000U))) return;
    oledMode = mode;
    oledRenderPeriodTicks = mode >= 2U ? renderPeriodTicks : 0;
    benchOledSessionActive = 1;
}

void OledSessionStop() {
    benchOledSessionActive = 0;
    InvalidateOledToken(lanes[0]); InvalidateOledToken(lanes[1]);
    lanes[0].stage = lanes[1].stage = OLED_OTHER;
    oledMode = 0; oledRenderPeriodTicks = 0;
    lanes[0].haveRender = lanes[1].haveRender = false;
}

OledSummary OledSnapshot() { return lanes[0].summary; }
OledSummary OledDisplaySnapshot() { return lanes[1].summary; }

OledToken FrameBegin(unsigned laneIndex, uint32_t now, uint32_t actualTick, uint32_t plannedTick) {
    OledToken token{}; token.lane = uint8_t(laneIndex);
    token.skip = laneIndex == 1U;
    if (!benchOledSessionActive || (laneIndex == 1U && oledMode < 3U)) return token;
    if (benchProfileWindowActive) Settle(now);
    Lane& lane = lanes[laneIndex];
    InvalidateOledToken(lane); lane.frameOpen = true;
    lane.stage = laneIndex == 0U ? OLED_IMU : OLED_OTHER;
    auto& s = lane.summary;
    ++s.begun;
    const uint32_t lateness = actualTick - plannedTick;
    if (lateness && lateness < 0x80000000U) {
        ++s.lateFrames;
        if (lateness > s.maxLateTicks) s.maxLateTicks = lateness;
    }
    if (lane.haveLoop) {
        const uint32_t gap = actualTick - lane.lastLoopTick;
        if (gap > s.maxLoopGapTicks) s.maxLoopGapTicks = gap;
    }
    lane.haveLoop = true; lane.lastLoopTick = actualTick;
    token.generation = lane.generation; token.cycles = now;
    const bool gated = laneIndex == 1U || oledMode == 2U;
    token.rateLimited = gated && lane.haveRender
        && actualTick - lane.lastRenderTick < oledRenderPeriodTicks;
    token.skip = (laneIndex == 0U && (oledMode == 1U || oledMode >= 3U)) || token.rateLimited;
    if (!token.skip) {
        if (lane.haveRender) {
            const uint32_t gap = actualTick - lane.lastRenderTick;
            if (s.renderBegun == 1U || gap < s.minRenderGapTicks) s.minRenderGapTicks = gap;
            if (gap > s.maxRenderGapTicks) s.maxRenderGapTicks = gap;
        }
        ++s.renderBegun; lane.lastRenderTick = actualTick; lane.haveRender = true;
    }
    return token;
}
OledToken OledFrameBegin(uint32_t now, uint32_t actualTick, uint32_t plannedTick) {
    return FrameBegin(0, now, actualTick, plannedTick);
}
OledToken OledDisplayFrameBegin(uint32_t now, uint32_t actualTick, uint32_t plannedTick) {
    return FrameBegin(1, now, actualTick, plannedTick);
}

void OledStageAt(const OledToken& token, unsigned stage, uint32_t now) {
    if (!ValidOledToken(token)) return;
    if (stage >= OledStageCount) {
        if (benchProfileWindowActive) flags |= INVALID_OLED_STAGE;
        return;
    }
    if (benchProfileWindowActive) Settle(now);
    lanes[token.lane].stage = stage;
}

void OledFrameEnd(const OledToken& token, uint32_t now) {
    if (!ValidOledToken(token)) return;
    if (benchProfileWindowActive) Settle(now);
    Lane& lane = lanes[token.lane];
    lane.stage = OLED_OTHER;
    const uint32_t elapsed = now - token.cycles;
    ++lane.summary.ended;
    if (token.skip) ++lane.summary.skipped;
    if (token.rateLimited) ++lane.summary.rateLimited;
    if (elapsed > lane.summary.maxFrameCycles) lane.summary.maxFrameCycles = elapsed;
    if (lane.overrunCycles && elapsed >= lane.overrunCycles) ++lane.summary.overrunFrames;
    InvalidateOledToken(lane);
}
}

extern "C" void BenchProfileSwitchedIn(void* task) {
    // Keep idle instrumentation to a flag read; do not read a host/hardware clock.
    if (!benchProfileWindowActive) return;
#if defined(BENCH_PROFILE_TEST_CLOCK)
    BenchProfile::SwitchAt(BenchProfileTestCycles(), task);
#elif defined(STM32F405xx)
    BenchProfile::SwitchAt(DWT->CYCCNT, task);
#else
    // Non-MCU builds must use SwitchAt explicitly or inject the test clock.
    (void)task;
#endif
}
