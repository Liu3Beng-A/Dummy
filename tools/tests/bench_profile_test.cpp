// Mirror common_inc.h / FreeRTOS.h: the config is first included from an
// enclosing C-linkage block. Backend definitions include their own header
// normally, so this executable also detects mismatched namespace linkage.
extern "C" {
#include "FreeRTOSConfig.h"
}
#include "bench_profile.h"
#include <cstdint>
#include <iostream>

namespace {
unsigned assertions = 0, failures = 0, cases = 0;
int taskTokens[7]{};
void* Task(unsigned i) { return &taskTokens[i]; }
uint32_t fakeCycle = 0, fakeReads = 0, afterReadCost = 0;
void Expect(bool ok, const char* name) {
    ++assertions;
    if (!ok) { ++failures; std::cerr << "FAIL " << name << '\n'; }
}
uint64_t Sum(const BenchProfile::Window& window) {
    uint64_t sum = 0;
    for (auto value : window.resident) sum += value;
    return sum;
}
void Fresh() {
    ++cases;
    BenchProfile::Reset();
    BenchProfile::OledSessionReset();
    BenchProfile::SetTasks(Task(0), Task(1), Task(2), Task(3), Task(4), Task(5));
}
void Balanced(const BenchProfile::Window& result, const char* name) {
    Expect(Sum(result) == result.cycles, name);
}
void OledBalanced(const BenchProfile::Window& result, const char* name) {
    uint64_t resident = 0, elapsed = 0;
    for (unsigned i = 0; i < BenchProfile::OledStageCount; ++i) {
        resident += result.oledResident[i];
        elapsed += result.oledElapsed[i];
        Expect(result.oledResident[i] <= result.oledElapsed[i], "OLED stage residency bounded by elapsed time");
    }
    Expect(resident == result.resident[2] && elapsed == result.cycles, name);
    uint64_t dr = 0, de = 0;
    for (unsigned i = 0; i < BenchProfile::OledStageCount; ++i) {
        dr += result.displayResident[i]; de += result.displayElapsed[i];
        Expect(result.displayResident[i] <= result.displayElapsed[i], "display phase residency bounded");
    }
    Expect(dr == result.resident[5] && de == result.cycles, "display lane conservation");
    Balanced(result, "OLED window task sum");
}
}

extern "C" uint32_t BenchProfileTestCycles(void) {
    ++fakeReads;
    const uint32_t sampled = fakeCycle;
    fakeCycle += afterReadCost;
    return sampled;
}

int main() {
    using namespace BenchProfile;
    static_assert(TaskCount == 7 && MaxWork == 5 && OledStageCount == 5, "fixed diagnostic capacity");
    static_assert(sizeof(Window) == 116 && sizeof(OledSummary) == 48 && sizeof(OledToken) == 12,
                  "fixed-width diagnostic storage");
    Fresh();
    Expect(MapMask() == 0x3f && benchProfileWindowActive == 0, "complete mapping, initially inactive");
    Expect(Begin(100, Task(0)) && benchProfileWindowActive == 1, "begin publishes active window");
    auto result = End(125);
    Expect(result.cycles == 25 && result.resident[0] == 25 && result.flags == 0,
           "first and final interval assigned to command");
    Expect(benchProfileWindowActive == 0, "end closes active window");
    Balanced(result, "single interval sum");

    Fresh();
    Expect(Begin(0, Task(0)), "multi begin");
    for (unsigned i = 1; i < TaskCount; ++i) SwitchAt(i * 10, Task(i));
    SwitchAt(60, nullptr);
    result = End(70);
    for (unsigned i = 0; i < 6; ++i) Expect(result.resident[i] == 10, "each mapped task gets its interval");
    Expect(result.resident[6] == 10 && result.cycles == 70 && result.flags == 0,
           "unknown and null selected tasks use other");
    Balanced(result, "multi interval sum");

    Fresh();
    Expect(Begin(50, Task(2)), "same task begin");
    SwitchAt(55, Task(2)); SwitchAt(55, Task(2));
    result = End(65);
    Expect(result.resident[2] == 15 && result.flags == 0, "same-task and zero-time switches do not lose time");
    Balanced(result, "same task sum");

    Fresh();
    Expect(Begin(0xfffffff0U, Task(0)), "wrap begin");
    SwitchAt(0xfffffff8U, Task(1)); SwitchAt(7, Task(4));
    result = End(16);
    Expect(result.cycles == 32 && result.resident[0] == 8 && result.resident[1] == 15
           && result.resident[4] == 9 && result.flags == 0, "unsigned DWT wrap is accounted once");
    Balanced(result, "wrapped interval sum");

    Fresh();
    Expect(Begin(42, Task(0)), "zero-length begin");
    SwitchAt(42, Task(3)); result = End(42);
    Expect(result.cycles == 0 && Sum(result) == 0 && result.flags == 0, "zero-length window is valid");

    Fresh();
    Expect(Begin(100, Task(0)), "nested outer begin");
    SwitchAt(110, Task(1));
    Expect(!Begin(999, Task(3)), "nested begin is refused");
    SwitchAt(120, Task(0)); result = End(130);
    Expect(result.cycles == 30 && result.resident[0] == 20 && result.resident[1] == 10
           && result.flags == NESTED_BEGIN, "nested begin retains original origin and counters");
    Balanced(result, "nested refusal sum");

    Fresh();
    SwitchAt(100, Task(0)); SwitchAt(200, Task(1));
    result = End(300);
    Expect(result.cycles == 0 && Sum(result) == 0
           && result.flags == (INACTIVE_SWITCH | INACTIVE_END), "inactive misuse never accumulates time");
    result = End(999);
    Expect(result.cycles == 0 && Sum(result) == 0 && result.flags == INACTIVE_END,
           "repeated inactive end does not return stale measurements");

    Fresh();
    SwitchAt(10, Task(1)); Expect(Begin(20, Task(0)), "begin after inactive switch");
    result = End(30);
    Expect(result.flags == INACTIVE_SWITCH && result.cycles == 10,
           "inactive switch is visible in next window flags");
    Expect(Begin(40, Task(0)), "begin clean next window");
    result = End(50);
    Expect(result.flags == 0 && result.cycles == 10, "previous window counts and flags do not carry over");

    Fresh();
    Expect(Begin(100, Task(0)), "active mapping guard begin");
    SetTasks(Task(4), Task(3), Task(2), Task(1), Task(0), Task(5));
    SwitchAt(110, Task(1)); result = End(120);
    Expect(MapMask() == 0x3f && result.flags == SET_TASKS_WHILE_ACTIVE
           && result.resident[0] == 10 && result.resident[1] == 10,
           "active mapping changes are refused without changing bucket identity");
    Balanced(result, "mapping refusal sum");

    Fresh();
    SetTasks(Task(0), Task(0), Task(2), nullptr, Task(4), Task(5));
    Expect(MapMask() == 0x34, "missing and all duplicated mappings are invalid");
    Expect(Begin(0, Task(0)), "invalid mapping begin remains bounded");
    SwitchAt(10, Task(2)); result = End(20);
    Expect(result.flags == INVALID_MAPPING && result.resident[6] == 10 && result.resident[2] == 10,
           "invalid mapping goes to other and is explicitly flagged");
    Balanced(result, "invalid mapping sum");

    Fresh();
    Expect(Begin(100, Task(0)), "reset abort begin");
    SwitchAt(110, Task(3)); Reset();
    Expect(benchProfileWindowActive == 0 && MapMask() == 0x3f, "reset aborts tracking and retains mapping");
    result = End(200);
    Expect(result.cycles == 0 && Sum(result) == 0 && result.flags == INACTIVE_END,
           "reset cannot leak discarded counters");

    Fresh();
    fakeCycle = 100; fakeReads = 0;
    void* pxCurrentTCB = Task(0);
    traceTASK_SWITCHED_IN(); BenchProfileSwitchedIn(Task(1));
    Expect(fakeReads == 0 && benchProfileWindowActive == 0,
           "inactive macro and callback do not read the clock or start a window");
    result = End(100);
    Expect(result.flags == INACTIVE_END, "inactive production callback does not create a misuse flag");

#if defined(BENCH_PROFILE_TEST_CLOCK)
    Fresh();
    Expect(Begin(100, Task(0)), "hook begin");
    fakeCycle = 110; fakeReads = 0; afterReadCost = 17;
    pxCurrentTCB = Task(1); traceTASK_SWITCHED_IN();
    result = End(fakeCycle); afterReadCost = 0;
    Expect(fakeReads == 1 && result.cycles == 27 && result.resident[0] == 10
           && result.resident[1] == 17 && result.flags == 0,
           "hook uses one timestamp; work after timestamp belongs to selected next task");
    Balanced(result, "synthetic hook-overhead accounting sum");
    // Caller activity before Begin and after End is outside the window. The
    // test assigns 17 synthetic cycles after a hook timestamp; it does not
    // measure or bound MCU overhead, exclusive CPU time, or interrupt time.
    fakeCycle += 1000; pxCurrentTCB = Task(3); traceTASK_SWITCHED_IN();
    Expect(fakeReads == 1, "post-end macro cannot extend a closed window");
#else
    Fresh();
    Expect(Begin(100, Task(0)), "portable no-clock callback begin");
    pxCurrentTCB = Task(1); traceTASK_SWITCHED_IN(); result = End(120);
    Expect(result.resident[0] == 20 && result.resident[1] == 0 && result.flags == 0,
           "non-MCU uninjected callback is a documented no-op");
#endif

    Fresh();
    const auto inactiveToken = OledFrameBegin(10, 10, 5);
    OledStageAt(inactiveToken, OLED_SEND, 15); OledFrameEnd(inactiveToken, 20);
    Expect(inactiveToken.generation == 0 && !inactiveToken.skip && !inactiveToken.rateLimited
           && benchOledSessionActive == 0,
           "inactive OLED instrumentation returns a harmless token");
    Expect(Begin(20, Task(2)), "ordinary profile without OLED session begin");
    result = End(30);
    auto summary = OledSnapshot();
    bool zeroStages = true;
    for (unsigned i = 0; i < OledStageCount; ++i)
        zeroStages = zeroStages && result.oledResident[i] == 0 && result.oledElapsed[i] == 0;
    Expect(zeroStages && result.resident[2] == 10 && summary.begun == 0 && summary.ended == 0,
           "old profile keeps task residency and zero OLED fields when session is off");

    Fresh();
    OledSessionStart(false, 1000);
    Expect(benchOledSessionActive == 1 && Begin(0, Task(2)), "OLED session and first window begin");
    const auto token = OledFrameBegin(2, 12, 10);
    OledStageAt(token, OLED_SERVICE, 5);
    SwitchAt(8, Task(0));
    OledStageAt(token, OLED_DRAW, 11);
    SwitchAt(13, Task(2));
    OledStageAt(token, OLED_SEND, 17);
    OledFrameEnd(token, 22);
    result = End(25); summary = OledSnapshot();
    const uint32_t firstResident[OledStageCount] = {3, 3, 4, 5, 5};
    const uint32_t firstElapsed[OledStageCount] = {3, 6, 6, 5, 5};
    for (unsigned i = 0; i < OledStageCount; ++i) {
        Expect(result.oledResident[i] == firstResident[i], "phase selected-task residency is exact");
        Expect(result.oledElapsed[i] == firstElapsed[i], "phase elapsed time includes preemption");
    }
    Expect(result.resident[2] == 20 && result.resident[0] == 5 && result.flags == 0,
           "phase boundaries do not change task accounting");
    OledBalanced(result, "phase elapsed and OLED residency both conserve");
    Expect(summary.begun == 1 && summary.ended == 1 && summary.skipped == 0
           && summary.lateFrames == 1 && summary.maxLateTicks == 2
           && summary.overrunFrames == 1 && summary.maxFrameCycles == 20
           && summary.renderBegun == 1 && summary.rateLimited == 0
           && summary.minRenderGapTicks == 0 && summary.maxRenderGapTicks == 0,
           "complete frame records lateness and exact 20ms overrun boundary");
    OledFrameEnd(token, 999); OledStageAt(token, OLED_IMU, 999);
    Expect(OledSnapshot().ended == 1 && OledSnapshot().maxFrameCycles == 20,
           "completed token cannot duplicate or rewrite summary");
    OledSessionStop();
    Expect(benchOledSessionActive == 0 && OledSnapshot().ended == 1,
           "stop preserves the completed summary");

    Fresh();
    OledSessionStart(false, 1000);
    const auto partial = OledFrameBegin(100, 0, 0);
    OledStageAt(partial, OLED_SERVICE, 110);
    Expect(Begin(120, Task(0)), "window opens midway through service phase");
    SwitchAt(130, Task(2)); OledStageAt(partial, OLED_DRAW, 140);
    result = End(150);
    Expect(result.oledElapsed[OLED_SERVICE] == 20 && result.oledResident[OLED_SERVICE] == 10
           && result.oledElapsed[OLED_DRAW] == 10 && result.oledResident[OLED_DRAW] == 10,
           "partial window starts from current phase without counting earlier frame time");
    OledBalanced(result, "partial first window conservation");
    Expect(Begin(160, Task(2)), "next window preserves the unfinished frame phase");
    OledStageAt(partial, OLED_SEND, 170); OledFrameEnd(partial, 180);
    result = End(190); summary = OledSnapshot();
    Expect(result.oledElapsed[OLED_DRAW] == 10 && result.oledElapsed[OLED_SEND] == 10
           && result.oledElapsed[OLED_OTHER] == 10 && summary.maxFrameCycles == 80,
           "window counters reset while frame duration spans both windows and gap");
    OledBalanced(result, "partial second window conservation");

    Fresh();
    OledSessionStart(true, 1000);
    const auto skipped = OledFrameBegin(100, 20, 21);
    Expect(skipped.skip && !skipped.rateLimited && skipped.generation != 0,
           "skip mode is returned by active frame token and is not a rate gate");
    OledStageAt(skipped, OLED_SERVICE, 105);
    Expect(Begin(110, Task(2)), "reset-retention window begin");
    Reset();
    Expect(benchOledSessionActive == 1 && Begin(120, Task(2)), "main profile reset retains OLED session");
    OledFrameEnd(skipped, 125);
    result = End(130); summary = OledSnapshot();
    Expect(result.oledResident[OLED_SERVICE] == 5 && result.oledResident[OLED_OTHER] == 5
           && summary.begun == 1 && summary.ended == 1 && summary.skipped == 1
           && summary.lateFrames == 0 && summary.maxLateTicks == 0
           && summary.renderBegun == 0 && summary.rateLimited == 0
           && summary.minRenderGapTicks == 0 && summary.maxRenderGapTicks == 0,
           "reset retains valid frame token and skipped frame completion; early start is not late");
    OledBalanced(result, "reset retained session conservation");

    Fresh();
    OledSessionStart(false, 1600);
    Expect(Begin(0xfffffff0U, Task(2)), "OLED wrapped window begin");
    const auto wrapped = OledFrameBegin(0xfffffff0U, 1, 0xfffffffdU);
    OledStageAt(wrapped, OLED_SEND, 0);
    SwitchAt(8, Task(1)); OledFrameEnd(wrapped, 16);
    result = End(32); summary = OledSnapshot();
    Expect(result.cycles == 48 && result.oledElapsed[OLED_IMU] == 16
           && result.oledElapsed[OLED_SEND] == 16 && result.oledElapsed[OLED_OTHER] == 16
           && result.oledResident[OLED_IMU] == 16 && result.oledResident[OLED_SEND] == 8
           && summary.maxFrameCycles == 32 && summary.maxLateTicks == 4
           && summary.overrunFrames == 1,
           "DWT and tick wraps preserve stage accounting and exact frame duration");
    OledBalanced(result, "wrapped phase conservation");

    Fresh();
    OledSessionStart(false, 1001); // 20.02 cycles rounds up to 21.
    auto boundary = OledFrameBegin(0, 0, 0); OledFrameEnd(boundary, 20);
    boundary = OledFrameBegin(30, 0, 0); OledFrameEnd(boundary, 51);
    summary = OledSnapshot();
    Expect(summary.begun == 2 && summary.ended == 2 && summary.overrunFrames == 1
           && summary.maxFrameCycles == 21,
           "non-divisible clock rounds 20ms threshold up instead of flagging early");
    OledSessionStart(false, 0);
    boundary = OledFrameBegin(0, 0, 0); OledFrameEnd(boundary, 100);
    Expect(OledSnapshot().overrunFrames == 0 && OledSnapshot().maxFrameCycles == 100,
           "unknown zero clock suppresses overrun classification but keeps duration");

    Fresh();
    OledSessionStart(false, 1000);
    const auto previousFrame = OledFrameBegin(0, 0, 0);
    const auto nextFrame = OledFrameBegin(10, 0, 0);
    Expect(previousFrame.generation != nextFrame.generation && Begin(10, Task(2)),
           "new frame invalidates previous unfinished frame token");
    OledStageAt(previousFrame, OLED_SEND, 15); OledFrameEnd(previousFrame, 15);
    OledStageAt(nextFrame, OLED_SERVICE, 20); OledFrameEnd(nextFrame, 30);
    result = End(40); summary = OledSnapshot();
    Expect(result.flags == 0 && result.oledElapsed[OLED_IMU] == 10
           && result.oledElapsed[OLED_SERVICE] == 10 && result.oledElapsed[OLED_SEND] == 0
           && summary.begun == 2 && summary.ended == 1,
           "stale frame token cannot change a newer frame phase or counters");
    OledBalanced(result, "new frame stale token conservation");

    Fresh();
    OledSessionStart(true, 1000);
    const auto oldSession = OledFrameBegin(0, 0, 0);
    OledSessionStop(); OledSessionStart(false, 1000);
    const auto newSession = OledFrameBegin(100, 0, 0);
    Expect(!newSession.skip && oldSession.generation != newSession.generation
           && Begin(100, Task(2)), "new session invalidates old token and resets skip mode");
    OledStageAt(oldSession, 999, 105); OledFrameEnd(oldSession, 105);
    OledStageAt(newSession, OLED_SERVICE, 110); OledFrameEnd(newSession, 120);
    result = End(130); summary = OledSnapshot();
    Expect(result.flags == 0 && summary.begun == 1 && summary.ended == 1 && summary.skipped == 0
           && result.oledElapsed[OLED_IMU] == 10 && result.oledElapsed[OLED_SERVICE] == 10,
           "stale session token including invalid stage is harmless to active session");
    OledBalanced(result, "new session stale token conservation");
    OledSessionReset();
    OledFrameEnd(newSession, 999);
    summary = OledSnapshot();
    Expect(benchOledSessionActive == 0 && summary.begun == 0 && summary.ended == 0
           && summary.maxFrameCycles == 0 && summary.renderBegun == 0 && summary.rateLimited == 0
           && summary.minRenderGapTicks == 0 && summary.maxRenderGapTicks == 0,
           "session reset discards summary and invalidates current token");

    Fresh();
    OledSessionStart(false, 1000);
    const auto invalidStage = OledFrameBegin(10, 0, 0);
    Expect(Begin(10, Task(2)), "invalid stage test begin");
    OledStageAt(invalidStage, OledStageCount, 15);
    OledFrameEnd(invalidStage, 20); result = End(30);
    Expect(result.flags == INVALID_OLED_STAGE && result.oledElapsed[OLED_IMU] == 10
           && result.oledElapsed[OLED_OTHER] == 10,
           "valid-token invalid stage is flagged without indexing outside arrays");
    OledBalanced(result, "invalid stage conservation");

    Fresh();
    OledSessionStart(false, 1000);
    Expect(Begin(100, Task(2)), "session begins while old uninstrumented frame is still running");
    result = End(120);
    Expect(result.oledElapsed[OLED_OTHER] == 20 && result.oledResident[OLED_OTHER] == 20
           && OledSnapshot().begun == 0 && OledSnapshot().ended == 0,
           "unmarked old-frame tail belongs to OTHER and does not invent a frame");
    OledBalanced(result, "unmarked frame tail conservation");

    Fresh();
    OledSessionStart(2U, 1000, 100);
    auto rateToken = OledFrameBegin(0, 0, 0);
    Expect(!rateToken.skip && !rateToken.rateLimited,
           "rate gate permits the first frame even when the actual tick is zero");
    OledFrameEnd(rateToken, 1);
    rateToken = OledFrameBegin(10, 99, 20);
    Expect(rateToken.skip && rateToken.rateLimited, "rate gate refuses a 99-tick opportunity");
    OledFrameEnd(rateToken, 11);
    rateToken = OledFrameBegin(20, 100, 40);
    Expect(!rateToken.skip && !rateToken.rateLimited, "rate gate permits the exact 100-tick boundary");
    OledFrameEnd(rateToken, 21);
    rateToken = OledFrameBegin(30, 100, 60);
    Expect(rateToken.skip && rateToken.rateLimited, "same-tick catch-up loop cannot render twice");
    OledFrameEnd(rateToken, 31);
    rateToken = OledFrameBegin(40, 199, 80);
    Expect(rateToken.skip && rateToken.rateLimited, "rate gate remains anchored to last allowed render");
    OledFrameEnd(rateToken, 41);
    rateToken = OledFrameBegin(50, 200, 100);
    Expect(!rateToken.skip && !rateToken.rateLimited, "second exact period is permitted");
    OledFrameEnd(rateToken, 51); summary = OledSnapshot();
    Expect(summary.begun == 6 && summary.ended == 6 && summary.skipped == 3
           && summary.renderBegun == 3 && summary.rateLimited == 3
           && summary.minRenderGapTicks == 100 && summary.maxRenderGapTicks == 100,
           "rate counters distinguish all loop frames, permitted starts and completed gated frames");

    Fresh();
    OledSessionStart(2U, 1000, 100);
    rateToken = OledFrameBegin(0, 0, 0); OledFrameEnd(rateToken, 1);
    rateToken = OledFrameBegin(10, 450, 20);
    Expect(!rateToken.skip, "late opportunity renders the latest frame once");
    OledFrameEnd(rateToken, 11);
    for (uint32_t planned = 40; planned <= 400; planned += 20) {
        rateToken = OledFrameBegin(20 + planned, 450, planned);
        Expect(rateToken.skip && rateToken.rateLimited, "late missed periods never become render backlog");
        OledFrameEnd(rateToken, 21 + planned);
    }
    rateToken = OledFrameBegin(500, 549, 420);
    Expect(rateToken.skip && rateToken.rateLimited, "late-render gate uses actual tick, not planned wake tick");
    OledFrameEnd(rateToken, 501);
    rateToken = OledFrameBegin(510, 550, 440);
    Expect(!rateToken.skip, "next render is allowed one full actual period after the late start");
    OledFrameEnd(rateToken, 511); summary = OledSnapshot();
    Expect(summary.renderBegun == 3 && summary.rateLimited == 20 && summary.skipped == 20
           && summary.begun == 23 && summary.ended == 23
           && summary.minRenderGapTicks == 100 && summary.maxRenderGapTicks == 450,
           "late frame summary reports actual gaps without inventing missing frames");

    Fresh();
    OledSessionStart(2U, 1000, 100);
    rateToken = OledFrameBegin(0xfffffff0U, 0xfffffff0U, 0xfffffff0U);
    Expect(!rateToken.skip, "rate gate permits a first frame near tick wrap");
    OledFrameEnd(rateToken, 0xfffffff1U);
    rateToken = OledFrameBegin(0, 83, 4);
    Expect(rateToken.skip && rateToken.rateLimited, "wrapped actual tick still refuses a 99-tick gap");
    OledFrameEnd(rateToken, 1);
    rateToken = OledFrameBegin(10, 84, 24);
    Expect(!rateToken.skip && !rateToken.rateLimited, "wrapped actual tick permits exactly 100 ticks");
    OledFrameEnd(rateToken, 11);
    rateToken = OledFrameBegin(20, 184, 44); OledFrameEnd(rateToken, 21);
    summary = OledSnapshot();
    Expect(summary.renderBegun == 3 && summary.rateLimited == 1
           && summary.minRenderGapTicks == 100 && summary.maxRenderGapTicks == 100,
           "tick wrap preserves actual opportunity spacing and counts");

    Fresh();
    OledSessionStart(2U, 1000, 100);
    rateToken = OledFrameBegin(0, 0, 0); OledFrameEnd(rateToken, 1);
    Reset();
    rateToken = OledFrameBegin(10, 20, 20);
    Expect(rateToken.skip && rateToken.rateLimited,
           "profile-window Reset retains the independent rate-gate anchor");
    OledFrameEnd(rateToken, 11);
    OledSessionStop();
    summary = OledSnapshot();
    Expect(!benchOledSessionActive && summary.renderBegun == 1 && summary.rateLimited == 1,
           "session stop preserves rate summary while disabling the gate");
    OledSessionStart(2U, 1000, 100);
    const auto newRateSession = OledFrameBegin(20, 20, 20);
    Expect(!newRateSession.skip && newRateSession.generation != rateToken.generation,
           "new rate session resets anchor and immediately permits its first opportunity");
    OledStageAt(rateToken, 999, 21); OledFrameEnd(rateToken, 21);
    OledFrameEnd(newRateSession, 22); summary = OledSnapshot();
    Expect(summary.begun == 1 && summary.ended == 1 && summary.renderBegun == 1
           && summary.skipped == 0 && summary.rateLimited == 0,
           "stale gated tokens cannot leak counters into the next session");
    OledSessionReset(); OledSessionStart(2U, 1000, 100);
    rateToken = OledFrameBegin(30, 21, 21);
    Expect(!rateToken.skip, "explicit session reset also discards the previous rate anchor");
    OledFrameEnd(rateToken, 31);

    Fresh();
    OledSessionStart(2U, 1000, 100);
    rateToken = OledFrameBegin(0, 0, 0);
    OledSessionStop(); OledFrameEnd(rateToken, 1000); summary = OledSnapshot();
    Expect(summary.begun == 1 && summary.renderBegun == 1 && summary.ended == 0
           && summary.skipped == 0 && summary.rateLimited == 0,
           "terminal in-flight render counts a permitted beginning without inventing completion");
    OledSessionStart(2U, 1000, 100);
    rateToken = OledFrameBegin(10, 0, 0); OledFrameEnd(rateToken, 11);
    rateToken = OledFrameBegin(20, 20, 20);
    OledSessionStop(); OledFrameEnd(rateToken, 1000); summary = OledSnapshot();
    Expect(summary.begun == 2 && summary.renderBegun == 1 && summary.ended == 1
           && summary.skipped == 0 && summary.rateLimited == 0,
           "terminal in-flight gated frame is excluded from completed skip counters");

    Fresh();
    OledSessionStart(2U, 1000, 100);
    Expect(Begin(0, Task(2)), "rate-limited mixed-phase window begins");
    for (uint32_t frame = 0; frame <= 5; ++frame) {
        const uint32_t start = frame * 20;
        rateToken = OledFrameBegin(start, start, start);
        OledStageAt(rateToken, OLED_SERVICE, start + 2);
        if (!rateToken.skip) {
            OledStageAt(rateToken, OLED_DRAW, start + 4);
            SwitchAt(start + 6, Task(0));
            SwitchAt(start + 7, Task(2));
            OledStageAt(rateToken, OLED_SEND, start + 8);
            OledFrameEnd(rateToken, start + 12);
        } else {
            OledFrameEnd(rateToken, start + 4);
        }
    }
    result = End(120); summary = OledSnapshot();
    Expect(result.flags == 0 && result.cycles == 120 && result.resident[0] == 2
           && result.resident[2] == 118 && summary.begun == 6 && summary.ended == 6
           && summary.renderBegun == 2 && summary.rateLimited == 4 && summary.skipped == 4,
           "rate-limited windows retain exact task accounting and completion counts");
    Expect(result.oledElapsed[OLED_IMU] == 12 && result.oledElapsed[OLED_SERVICE] == 12
           && result.oledElapsed[OLED_DRAW] == 8 && result.oledElapsed[OLED_SEND] == 8
           && result.oledElapsed[OLED_OTHER] == 80 && result.oledResident[OLED_DRAW] == 6,
           "gated loops still account IMU and service but never invent draw or send time");
    OledBalanced(result, "rate-limited stage and task conservation");

    Fresh();
    OledSessionStart(2U, 1000, 100);
    rateToken = OledFrameBegin(0, 0, 0);
    OledSessionStart(5U, 1000, 100);
    auto refusedRate = OledFrameBegin(10, 100, 100);
    OledStageAt(rateToken, OLED_DRAW, 11); OledFrameEnd(rateToken, 12);
    summary = OledSnapshot();
    Expect(!benchOledSessionActive && refusedRate.generation == 0 && !refusedRate.skip
           && !refusedRate.rateLimited && summary.begun == 0 && summary.renderBegun == 0
           && summary.rateLimited == 0 && summary.ended == 0,
           "invalid mode safely disables and resets session, invalidating an old live token");
    OledSessionStart(2U, 1000, 0);
    refusedRate = OledFrameBegin(20, 200, 200); summary = OledSnapshot();
    Expect(!benchOledSessionActive && refusedRate.generation == 0 && summary.begun == 0,
           "zero-period rate mode is refused instead of accidentally rendering unrestricted");
    OledSessionStart(2U, 1000, 0x80000000U);
    Expect(!benchOledSessionActive && OledFrameBegin(21, 201, 201).generation == 0,
           "ambiguous half-range rate period is safely refused");
    OledSessionStart(2U, 1000, 0xffffffffU);
    Expect(!benchOledSessionActive && OledFrameBegin(22, 202, 202).generation == 0,
           "larger ambiguous rate period is safely refused");
    OledSessionStart(0U, 1000, 100);
    rateToken = OledFrameBegin(30, 1, 1); OledFrameEnd(rateToken, 31);
    rateToken = OledFrameBegin(40, 1, 1);
    Expect(!rateToken.skip && !rateToken.rateLimited,
           "normal mode ignores optional period and remains unrestricted at the same tick");
    OledFrameEnd(rateToken, 41);
    rateToken = OledFrameBegin(50, 11, 11); OledFrameEnd(rateToken, 51);
    summary = OledSnapshot();
    Expect(summary.renderBegun == 3 && summary.minRenderGapTicks == 0
           && summary.maxRenderGapTicks == 10 && summary.rateLimited == 0,
           "normal opportunity gap statistics preserve a genuine zero minimum");
    OledSessionStart(1U, 1000, 100);
    rateToken = OledFrameBegin(60, 1, 1); OledFrameEnd(rateToken, 61);
    summary = OledSnapshot();
    Expect(rateToken.skip && !rateToken.rateLimited && summary.renderBegun == 0
           && summary.skipped == 1 && summary.rateLimited == 0,
           "skip-render mode ignores optional period and keeps its original semantics");

    for (unsigned mode = 3; mode <= 4; ++mode) {
        Fresh(); OledSessionStart(mode, 1000, 100);
        Expect(Begin(0, Task(2)), "split lane window begins");
        const auto worker = OledFrameBegin(1, 10, 10);
        Expect(worker.skip && !worker.rateLimited && worker.lane == 0, "split worker skips rendering without rate limiting");
        OledStageAt(worker, OLED_SERVICE, 3);
        SwitchAt(5, Task(5));
        const auto display = OledDisplayFrameBegin(6, 10, 10);
        Expect(!display.skip && display.lane == 1, "split first display opportunity admitted");
        OledStageAt(display, OLED_DRAW, 8);
        SwitchAt(10, Task(0));
        // Worker can advance while display remains DRAW; both elapsed clocks run.
        OledStageAt(worker, OLED_OTHER, 12);
        SwitchAt(14, Task(5)); OledStageAt(display, OLED_SEND, 16);
        OledFrameEnd(worker, 21); OledFrameEnd(display, 106);
        result = End(110); const auto ds = OledDisplaySnapshot(); summary = OledSnapshot();
        OledBalanced(result, "split overlapping lanes preserve both clocks");
        Expect(result.oledElapsed[OLED_SERVICE] == 9 && result.displayElapsed[OLED_DRAW] == 8,
               "worker phase cannot overwrite display phase");
        Expect(result.displayResident[OLED_DRAW] == 4 && result.oledResident[OLED_SERVICE] == 2,
               "per-lane residency uses distinct worker/display task buckets");
        Expect(summary.renderBegun == 0 && summary.skipped == 1 && summary.overrunFrames == 1,
               "worker uses 20ms overrun threshold");
        Expect(ds.renderBegun == 1 && ds.overrunFrames == 1 && ds.maxFrameCycles == 100,
               "display uses 100ms overrun threshold");
        auto d2 = OledDisplayFrameBegin(120, 30, 30);
        Expect(d2.skip && d2.rateLimited, "split display is rate gated independently");
        OledFrameEnd(d2, 121);
        auto w2 = OledFrameBegin(125, 70, 70); OledFrameEnd(w2, 126);
        auto d3 = OledDisplayFrameBegin(130, 110, 110);
        Expect(!d3.skip && !d3.rateLimited, "split display permits exact 100tick opportunity");
        // Deliberate replacement of a worker frame cannot invalidate display.
        auto w3 = OledFrameBegin(135, 90, 90); OledFrameEnd(w2, 136);
        OledFrameEnd(w3, 137); OledFrameEnd(d3, 138);
        summary = OledSnapshot(); const auto doneDisplay = OledDisplaySnapshot();
        Expect(summary.maxLoopGapTicks == 60 && doneDisplay.maxLoopGapTicks == 80,
               "max loop gap includes gated attempts rather than render-only beginnings");
        Expect(doneDisplay.ended == 3 && doneDisplay.rateLimited == 1 && doneDisplay.minRenderGapTicks == 100,
               "display summary remains independent of worker tokens");
        OledSessionStop(); OledFrameEnd(d3, 200); OledStageAt(display, 999, 201);
        const auto inactiveDisplay = OledDisplayFrameBegin(210, 0, 0);
        Expect(inactiveDisplay.skip && !inactiveDisplay.generation && OledDisplaySnapshot().ended == 3,
               "inactive display is fail closed and stale tokens do not mutate summary");
        OledSessionStart(mode, 1000, 100);
        auto freshDisplay = OledDisplayFrameBegin(220, 0xfffffff0U, 0xfffffff0U);
        OledFrameEnd(display, 221); OledFrameEnd(freshDisplay, 222);
        auto wrappedDisplay = OledDisplayFrameBegin(230, 84, 84); OledFrameEnd(wrappedDisplay, 231);
        Expect(OledDisplaySnapshot().ended == 2 && OledDisplaySnapshot().minRenderGapTicks == 100
               && OledDisplaySnapshot().maxLoopGapTicks == 100, "split tick wrap and session generations");
        Reset();
        auto retained = OledDisplayFrameBegin(240, 184, 184);
        Reset(); OledFrameEnd(retained, 241);
        Expect(OledDisplaySnapshot().ended == 3, "profile reset retains independent display session");
    }
    Fresh(); OledSessionStart(2U, 1000, 100);
    Expect(OledDisplayFrameBegin(1, 0, 0).skip && OledDisplayFrameBegin(2, 0, 0).generation == 0,
           "unsplit session never permits display ownership");

    uint32_t random = 0x581378b1U;
    for (unsigned run = 0; run < 200; ++run) {
        Fresh();
        random = random * 1664525U + 1013904223U;
        uint32_t now = random, total = 0, expected[TaskCount]{};
        unsigned current = run % TaskCount;
        Expect(Begin(now, Task(current)), "sequence begin");
        for (unsigned i = 0; i < 50; ++i) {
            random = random * 1664525U + 1013904223U;
            const uint32_t delta = random % 100000U;
            expected[current] += delta; total += delta; now += delta;
            current = (random >> 16) % TaskCount;
            SwitchAt(now, Task(current));
        }
        now += 7; expected[current] += 7; total += 7; result = End(now);
        Expect(result.cycles == total && result.flags == 0, "sequence total and flags");
        for (unsigned i = 0; i < TaskCount; ++i)
            Expect(result.resident[i] == expected[i], "sequence exact bucket accounting");
        Balanced(result, "sequence sum equals window");
    }

    for (unsigned run = 0; run < 200; ++run) {
        Fresh();
        OledSessionStart((run & 1U) != 0, 168000000);
        random = random * 1664525U + 1013904223U;
        uint32_t now = random, total = 0;
        uint32_t expectedTask[TaskCount]{}, expectedResident[OledStageCount]{}, expectedElapsed[OledStageCount]{};
        unsigned current = run % TaskCount, stage = OLED_IMU;
        const auto sequenceToken = OledFrameBegin(now, run, run);
        Expect(Begin(now, Task(current)), "mixed OLED/task sequence begin");
        for (unsigned i = 0; i < 50; ++i) {
            random = random * 1664525U + 1013904223U;
            const uint32_t delta = random % 100000U;
            expectedTask[current] += delta;
            expectedElapsed[stage] += delta;
            if (current == 2) expectedResident[stage] += delta;
            now += delta; total += delta;
            if (i & 1U) {
                current = (random >> 16) % TaskCount;
                SwitchAt(now, Task(current));
            } else {
                stage = (random >> 16) % OledStageCount;
                OledStageAt(sequenceToken, stage, now);
            }
        }
        OledFrameEnd(sequenceToken, now);
        result = End(now); summary = OledSnapshot();
        Expect(result.cycles == total && result.flags == 0 && summary.begun == 1
               && summary.ended == 1 && summary.skipped == (run & 1U)
               && summary.maxFrameCycles == total,
               "mixed sequence preserves frame counts and total");
        for (unsigned i = 0; i < TaskCount; ++i)
            Expect(result.resident[i] == expectedTask[i], "mixed sequence exact task residency");
        for (unsigned i = 0; i < OledStageCount; ++i) {
            Expect(result.oledResident[i] == expectedResident[i], "mixed sequence exact stage residency");
            Expect(result.oledElapsed[i] == expectedElapsed[i], "mixed sequence exact stage elapsed");
        }
        OledBalanced(result, "mixed sequence dual conservation");
    }

    std::cout << "{\"mcu_timing\":false,\"exclusive_task_cpu\":false,"
                 "\"clock\":\"synthetic deterministic cycle timestamps\","
                 "\"interrupts_included_in_residency\":true,\"cases\":" << cases
              << ",\"assertions\":" << assertions << ",\"failures\":" << failures << "}\n";
    return failures ? 1 : 0;
}
