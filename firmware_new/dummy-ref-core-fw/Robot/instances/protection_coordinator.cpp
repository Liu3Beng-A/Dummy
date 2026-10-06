#include "common_inc.h"
#include "protection_coordinator.h"
#include <cstring>
#include <cstdio>
#include <cstdarg>
#include <cstdlib>
#include <cmath>
using namespace ProtectionProtocol;

static uint16_t Read16(const uint8_t* p) { return p[0] | (uint16_t(p[1]) << 8); }
static bool Finite(float v) {
    uint32_t bits; std::memcpy(&bits, &v, 4);
    return (bits & 0x7f800000U) != 0x7f800000U;
}
static const char* ReasonName(uint8_t reason) {
    static const char* const names[] = {"NONE", "STALL", "NO_DIRECTION", "NO_SPACE", "NO_ENCODER",
        "INVALID_PROFILE", "GRANT_TIMEOUT", "LEASE_TIMEOUT", "RETREAT_TIMEOUT", "REBLOCKED",
        "CANCELLED_STOP", "CANCELLED_DISABLE", "SEARCH_TIMEOUT", "SEARCH_TRAVEL", "CONTACT_UNSTABLE",
        "EARLY_CONTACT", "RETURN_TIMEOUT", "SAVE_FAILED", "NOT_DISABLED", "BAD_TASK", "BAD_VALUE",
        "BUSY", "UNSUPPORTED", "COORDINATE_INVALID", "COMMUNICATION_TIMEOUT", "CAPTURE_UNSTABLE"};
    return reason <= CAPTURE_UNSTABLE ? names[reason] : "UNKNOWN";
}
uint16_t ProtectionCoordinator::NextId() {
    if (!serial) serial = uint16_t(HAL_GetTick() ^ 0xB571U);
    if (!++serial) ++serial;
    return serial;
}
void ProtectionCoordinator::Enter(Job next, uint16_t mask) {
    job = next; expected = mask; received = failed = 0; cursor = 1;
    phaseAt = HAL_GetTick(); lastSend = phaseAt - 25; replyReady = false;
}
void ProtectionCoordinator::Log(const char* format, ...) {
    // Single control-task producer except command entry: guard the tiny ring.
    taskENTER_CRITICAL();
    uint8_t next = uint8_t((writeNotice + 1) % 24);
    if (next == readNotice) readNotice = uint8_t((readNotice + 1) % 24);
    Notice& n = notices[writeNotice]; n.sink = sink;
    va_list args; va_start(args, format);
    vsnprintf(n.text, sizeof(n.text), format, args); va_end(args);
    writeNotice = next;
    taskEXIT_CRITICAL();
}
void ProtectionCoordinator::Print() {
    for (int i = 0; i < 4; ++i) {
        Notice n{};
        taskENTER_CRITICAL();
        if (readNotice == writeNotice) { taskEXIT_CRITICAL(); break; }
        n = notices[readNotice]; readNotice = uint8_t((readNotice + 1) % 24);
        taskEXIT_CRITICAL();
        if (n.sink) Respond(*n.sink, "%s", n.text);
        else Respond(*usbStreamOutputPtr, "%s", n.text);
    }
}
bool ProtectionCoordinator::Send(uint8_t node, uint8_t command, const uint8_t* data) {
    taskENTER_CRITICAL();
    if (!allowTransmit || txUsed || pollEpoch != robot.motionEpoch) {
        taskEXIT_CRITICAL(); return false;
    }
    bool sent = (node == 8 ? static_cast<CtrlStepMotor*>(robot.hand) : robot.motorJ[node])
        ->TrySendProtection(command, data);
    if (sent) txUsed = true;
    taskEXIT_CRITICAL();
    return sent;
}
bool ProtectionCoordinator::Context(uint8_t n, uint8_t op, uint16_t id, float v, uint32_t event) {
    uint8_t frame[8] = {op, nodes[n].version, uint8_t(id), uint8_t(id >> 8)};
    if (op == RETREAT_GO || op == PROTECT_PREPARE || op == RENEW) std::memcpy(frame + 4, &event, 4);
    else std::memcpy(frame + 4, &v, 4);
    uint16_t previous = nodes[n].issuedProbe;
    if (op == QUERY) nodes[n].issuedProbe = id;
    bool sent = Send(n, CONTEXT, frame);
    if (op == QUERY && !sent) nodes[n].issuedProbe = previous;
    if (op == QUERY && sent) {
        Node& a = nodes[n];
        if (previous != id || !a.probePending) a.probeFirstMs = HAL_GetTick();
        else ++a.probeRetry;
        a.probeSentMs = HAL_GetTick(); a.probePending = true; ++a.probeTx;
    }
    return sent;
}
bool ProtectionCoordinator::Config(uint8_t n, uint8_t op, uint32_t arg) {
    uint8_t frame[8] = {op, 0, uint8_t(task), uint8_t(task >> 8)};
    std::memcpy(frame + 4, &arg, 4); return Send(n, CONFIG, frame);
}
bool ProtectionCoordinator::Parameter(uint8_t n, bool set, uint8_t p, float v) {
    uint8_t frame[8] = {p, 0, uint8_t(task), uint8_t(task >> 8)};
    std::memcpy(frame + 4, &v, 4); return Send(n, set ? ProtectionProtocol::PARAM_SET : ProtectionProtocol::PARAM_GET, frame);
}
void ProtectionCoordinator::Receive(uint8_t n, uint8_t command, const uint8_t* data) {
    if (n < 1 || n > 8) return;
    Node& a = nodes[n]; uint32_t now = HAL_GetTick();
    a.aliveMs = now;
    if (command == EVENT) {
        uint32_t event; std::memcpy(&event, data, 4);
        const bool bootReset = !event && a.event && data[4] == IDLE &&
            (data[7] & ASSUMED_BOOT) && !(data[7] & PROFILE_VALID) &&
            a.probe == a.issuedProbe && a.issuedProbe && now - a.healthMs < 200 &&
            (a.probeFlags & ASSUMED_BOOT) && !(a.probeFlags & PROFILE_VALID);
        if (a.event && event != a.event && !bootReset) {
            int16_t sessionDelta = int16_t(uint16_t(event >> 16) - uint16_t(a.event >> 16));
            int16_t counterDelta = int16_t(uint16_t(event) - uint16_t(a.event));
            if (!event || sessionDelta < 0 || (!sessionDelta && counterDelta <= 0)) return;
        }
        if (!bootReset && event == a.event && a.version && int8_t(data[6] - a.version) < 0) return;
        if (event == a.event && (data[7] & LOCAL_STALL) &&
            ((a.eventPhase == IDLE && seenEvents[n] == event) ||
             (data[4] != IDLE && data[4] < a.eventPhase && a.eventPhase <= HOLD_FAILED))) return;
        const bool changed = event != a.event || data[4] != a.eventPhase || data[5] != a.reason;
        a.event = event; a.eventPhase = data[4]; a.reason = data[5];
        a.version = data[6]; a.flags = data[7]; a.eventMs = now;
        a.dirty |= changed; a.ack = true;
        if (data[4] == IDLE && (data[7] & ASSUMED_BOOT) && !(data[7] & PROFILE_VALID))
            seenEvents[n] = 0; // A fresh boot invalidates the former episode namespace.
        if (!suppressEpisodes && robot.stopState != DummyRobot::DISABLING && n <= 7 && (data[7] & LOCAL_STALL) &&
            data[4] >= LOCAL_STOPPING && data[4] <= HOLD_FAILED &&
            seenEvents[n] != event) {
            // Interrupt the regular dispatch immediately; queue cleanup is deferred.
            if (!trigger && !protectionSeen) ++robot.motionEpoch;
            robot.motionActive = false; robot.isEnabled = false;
            locked = alarm = trigger = true;
        }
    } else if (command == PROBE_REPLY) {
        const uint16_t probe = Read16(data + 2);
        const bool ownerQuery = job == START_QUERY || job == START_VERIFY || job == CAL_QUERY;
        if (!probe || (probe != a.issuedProbe && !(ownerQuery && probe == queryProbe))) return;
        float position; std::memcpy(&position, data + 4, 4);
        a.probe = probe; a.healthMs = now; a.probeFlags = data[1];
        if (!Finite(position)) a.probeFlags &= ~FEEDBACK_VALID;
        a.version = a.probeVersion = data[0]; a.probePending = false; ++a.probeRx;
        if (Finite(position)) a.position = position;
        if (ownerQuery && probe == queryProbe) {
            if (data[0] && (data[1] & FEEDBACK_VALID) && Finite(position)) received |= uint16_t(1U << n);
            else failed |= uint16_t(1U << n);
        }
    } else if (command == CONTEXT_REPLY) {
        if (data[0] & LEASE_REPLY_FLAG) {
            uint32_t nonce; std::memcpy(&nonce, data + 4, 4);
            const uint8_t nextVersion = a.leaseVersion == 255 ? 1 : uint8_t(a.leaseVersion + 1);
            if (nonce && nonce == a.leaseNonce && Read16(data + 2) == a.leaseTask &&
                (data[1] == a.leaseVersion || (calibration && data[1] == nextVersion))) {
                a.leaseAckMs = now; a.leaseNonce = 0;
            }
            return;
        }
        float position; std::memcpy(&position, data + 4, 4);
        if (data[0] & QUERY_REPLY_FLAG) {
            // Old firmware lacks a coherent coordinate version. It is alive,
            // but cannot satisfy the new unlock/protection status proof.
            return;
        } else {
            a.task = Read16(data + 2); a.phase = data[0];
            a.reason = data[1]; a.contextMs = now; a.position = position;
        }
    } else if (command == PARAM_REPLY && Read16(data + 2) == task && n == active) {
        replyNode = n; replyParam = data[0]; replyStatus = data[1];
        replyTransaction = Read16(data + 2); replyOp = data[4];
        if (!data[0] && data[4] == COMMIT && data[5] != nodes[n].version)
            replyStatus = INVALID;
        std::memcpy(&replyValue, data + 4, 4); replyReady = true;
    }
}
void ProtectionCoordinator::ObserveCommunication(uint8_t node) {
    if (node >= 1 && node <= 8) nodes[node].aliveMs = HAL_GetTick();
}
void ProtectionCoordinator::BeginStop(bool protecting, bool includeHand) {
    taskENTER_CRITICAL();
    robot.isEnabled = robot.motionActive = false; ++robot.motionEpoch;
    robot.stopPerNodeTokens = protecting || robot.stopToken == 0;
    robot.stopWithGripper = includeHand;
    robot.stopSyncedMask = robot.stopAckMask = robot.stopHoldingMask = 0;
    ++robot.stopToken;
    robot.stopFaultMask = robot.stopFaultPrinted = 0;
    robot.pendingStopNotice = nullptr;
    robot.stopAction = protecting ? STOP_PROTECTION : SoftStopProtocol::STOP;
    robot.stopState = DummyRobot::STOPPING; robot.stopNeedsSend = true;
    robot.ResetStopDiagnostics(); robot.tuningHelper.SetTuningFlag(0);
    robot.commandHandler.ClearFifo();
    pollEpoch = robot.motionEpoch;
    taskEXIT_CRITICAL();
}
bool ProtectionCoordinator::InterceptStart() {
    if (internal) return false;
    if (calibration) { Log("error CALIBRATION busy"); return true; }
    if (job == PROTECT_STOP || job == PROTECT_RUN) {
        robot.startPending = true; return true;
    }
    if (job != NONE) return true;
    if (robot.stopState == DummyRobot::STOPPING || robot.stopState == DummyRobot::DISABLING)
        return false;
    if (robot.stopState == DummyRobot::NORMAL && !locked) return false;
    // A fresh health snapshot and coordinate-bound limits precede every unlock.
    task = NextId(); queryProbe = task; startAfterProfile = true;
    Enter(START_QUERY, 0x1FE); return true;
}
void ProtectionCoordinator::Started() { locked = alarm = protectionSeen = false; }
void ProtectionCoordinator::Disabled() {
    if (!calibration) { locked = alarm = protectionSeen = false; }
}
void ProtectionCoordinator::CancelContexts() {
    for (uint8_t count = 0; count < 8; ++count) {
        if (cancelCursor > 8) cancelCursor = 1;
        uint8_t n = cancelCursor++;
        if (!cancelTasks[n]) continue;
        uint8_t frame[8] = {CANCEL, cancelVersions[n], uint8_t(cancelTasks[n]),
                            uint8_t(cancelTasks[n] >> 8)};
        if (Send(n, CONTEXT, frame)) cancelMask |= uint16_t(1U << n);
        // Retry until node confirms the old task is no longer executing.
        if (nodes[n].contextMs > cancelAt[n] && (nodes[n].task != cancelTasks[n] ||
            nodes[n].phase == HOLD_FAILED || nodes[n].phase == CAL_FAILED ||
            nodes[n].phase == PEER_HOLD || nodes[n].phase == IDLE)) cancelTasks[n] = 0;
        break;
    }
}
void ProtectionCoordinator::Cancel(bool disable) {
    if (internal) return;
    const bool wasCalibration = calibration;
    suppressEpisodes = true;
    for (uint8_t n = 1; n <= 8; ++n) {
        seenEvents[n] = nodes[n].event;
        if ((wasCalibration || (stalled & (1U << n))) && task) {
            cancelTasks[n] = task; cancelVersions[n] = nodes[n].version; cancelAt[n] = HAL_GetTick();
        }
    }
    if (wasCalibration) {
        cancelCursor = 8;
        Log("error CALIBRATION cancelled saved=%03X pending=%03X", saved, saveInFlight);
    }
    calibration = false; robot.startPending = false; trigger = false;
    job = NONE; stalled = finished = 0; locked = !disable;
    protectionSeen = !disable; abortedCalibration = false; sink = nullptr;
}
void ProtectionCoordinator::Error(const char* reason, uint8_t n) {
    bool wasCal = calibration;
    if (wasCal) {
        Log("error CALIBRATION J%u %s", n, reason);
        Log("error CALIBRATION saved=%03X pending=%03X", saved, saveInFlight);
    }
    else if (job == SETTINGS_GET || job == SETTINGS_SET || job == SETTINGS_SAVE)
        Log("error %s_%s J%u %s", calParameters ? "CAL" : "STALL",
            job == SETTINGS_GET ? "GET" : job == SETTINGS_SET ? "SET" : "SAVE", n, reason);
    if (job == SETTINGS_SET || job == SETTINGS_SAVE) Log("error config partial applied=0x%03X", received);
    else if (!wasCal && job != SETTINGS_GET)
        Log(stalled ? "STALL J%u failed reason=%s" : "PROTECT J%u failed reason=%s", n, reason);
    const bool readOnly = job == SETTINGS_GET;
    for (uint8_t id = 1; id <= 8; ++id)
        if ((wasCal || (stalled & (1U << id))) && task) {
            cancelTasks[id] = task; cancelVersions[id] = nodes[id].version; cancelAt[id] = HAL_GetTick();
        }
    job = NONE; calibration = false; robot.startPending = false;
    locked = !readOnly; protectionSeen = locked; trigger = false;
    if (locked) BeginStop(false, wasCal);
}
void ProtectionCoordinator::SyncPosition(uint8_t n) {
    if (!Finite(nodes[n].position)) return;
    if (n == 8) robot.hand->UpdateAngleCallback(nodes[n].position, true);
    else robot.motorJ[n]->UpdateAngleCallback(nodes[n].position, true);
    robot.UpdateJointAnglesCallback();
    robot.targetJoints = robot.currentJoints; robot.targetRailPos = robot.currentRailPos;
}
float ProtectionCoordinator::Bound(uint8_t n, bool max) const {
    CtrlStepMotor* motor = n == 8 ? static_cast<CtrlStepMotor*>(robot.hand) : robot.motorJ[n];
    float low, high;
    if (n == 7) { low = -50; high = 50; }
    else if (n == 8) { low = 0; high = 100.0f * 16 / 360; }
    else {
        low = (motor->angleLimitMin - robot.initPose.a[n-1]) * motor->reduction / 360;
        high = (motor->angleLimitMax - robot.initPose.a[n-1]) * motor->reduction / 360;
    }
    if (motor->inverseDirection) { float old = low; low = -high; high = -old; }
    return max ? high : low;
}
bool ProtectionCoordinator::TakeReply(uint8_t p, uint8_t op) {
    taskENTER_CRITICAL();
    bool ready = replyReady && replyNode == active && replyTransaction == task &&
                 replyParam == p && (!op || replyOp == op);
    if (ready) replyReady = false;
    taskEXIT_CRITICAL(); return ready;
}
void ProtectionCoordinator::PollProfile() {
    uint8_t p = stage == 1 ? MIN_POSITION_R : stage == 2 ? MAX_POSITION_R : DIRECTION_MASK;
    uint8_t op = stage == 0 ? BEGIN : COMMIT;
    if (TakeReply(stage == 0 || stage == 4 ? 0 : p, stage == 0 || stage == 4 ? op : 0)) {
        if (replyStatus != OK) { Error("PROFILE_REJECTED", active); return; }
        lastSend = HAL_GetTick() - 25; phaseAt = HAL_GetTick();
        if (++stage == 5) {
            nodes[active].flags |= PROFILE_VALID;
            nodes[active].profileVersion = nodes[active].version;
            stage = 0; if (++active > 7) {
                task = NextId(); queryProbe = task; Enter(START_VERIFY, 0x1FE);
                return;
            }
            task = NextId();
        }
    }
    if (HAL_GetTick() - phaseAt > 1500) { Error("PROFILE_TIMEOUT", active); return; }
    if (HAL_GetTick() - lastSend < 20) return;
    p = stage == 1 ? MIN_POSITION_R : stage == 2 ? MAX_POSITION_R : DIRECTION_MASK;
    bool sent = stage == 0 ? Config(active, BEGIN) : stage == 4 ? Config(active, COMMIT, nodes[active].version) :
        Parameter(active, true, p, stage == 1 ? Bound(active, false) : stage == 2 ? Bound(active, true) : 3);
    if (sent) lastSend = HAL_GetTick();
}
void ProtectionCoordinator::PollGroup(uint8_t op, uint8_t phase, uint16_t mask) {
    uint32_t now = HAL_GetTick();
    for (uint8_t n = 1; n <= 8; ++n) {
        if (!(mask & (1U << n))) continue;
        if (nodes[n].task == task && nodes[n].contextMs >= phaseAt) {
            if (nodes[n].phase == phase && nodes[n].reason == NONE &&
                (phase == SAVED || phase == IDLE || Finite(nodes[n].position))) received |= uint16_t(1U << n);
            else if (nodes[n].phase == CAL_FAILED || nodes[n].phase == HOLD_FAILED ||
                     nodes[n].reason == NOT_DISABLED || nodes[n].reason == BAD_TASK ||
                     nodes[n].reason == NO_ENCODER || nodes[n].reason == SAVE_FAILED) {
                Error(ReasonName(nodes[n].reason), n); return;
            }
        }
    }
    if ((received & mask) == mask) return;
    if (now - phaseAt > 2000) { Error("NODE_TIMEOUT", cursor); return; }
    if (now - lastSend < 20) return;
    for (uint8_t count = 0; count < 8; ++count) {
        if (cursor > 8) cursor = 1;
        uint8_t n = cursor++;
        if ((mask & (1U << n)) && !(received & (1U << n))) {
            if (Context(n, op, task)) lastSend = now;
            break;
        }
    }
}
void ProtectionCoordinator::Lease() {
    uint32_t now = HAL_GetTick();
    // Only moving CAL nodes need leases. Ordinary WAIT_GRANT nodes also have
    // a local deadline. Every eligible node gets an 80ms renewal with 20ms
    // acknowledged retries, instead of a fragile 320ms eight-node round.
    for (uint8_t count = 0; count < 8; ++count) {
        if (renewNode > 8) renewNode = 1;
        uint8_t n = renewNode++;
        Node& a = nodes[n];
        const bool movingCal = calibration && n == active && (job == CAL_HAND || job == CAL_RAIL);
        if (a.task != task || !(movingCal || ((stalled & (1U << n)) && !(finished & (1U << n))))) continue;
        if (a.leaseTask != task) { a.leaseTask = task; a.leaseNonce = 0; a.leaseAckMs = now - 80; }
        if (a.leaseNonce ? now - a.leaseSentMs < 20 : now - a.leaseAckMs < 80) continue;
        if (!a.leaseNonce) {
            if (!++leaseSerial) ++leaseSerial;
            a.leaseNonce = leaseSerial; a.leaseVersion = a.version; a.leaseSentMs = now - 20;
        }
        if (Context(n, RENEW, task, 0, a.leaseNonce)) { a.leaseSentMs = now; lastRenew = now; }
        break; // Rotate on busy as well: a missing low ID cannot starve a peer.
    }
}
void ProtectionCoordinator::HealthFault(uint8_t n, const char* reason) {
    sink = nullptr; const uint32_t now = HAL_GetTick(); const Node& a = nodes[n];
    Log("PROTECT J%u failed reason=%s", n, reason);
    Log("PROTECT J%u age rx=%lu probe=%lu flags=%02X", n,
        (unsigned long)(now-a.aliveMs), (unsigned long)(now-a.healthMs), a.probeFlags);
    Log("PROTECT J%u queued=%lu rx=%lu retry=%lu", n,
        (unsigned long)a.probeTx, (unsigned long)a.probeRx, (unsigned long)a.probeRetry);
    const CAN_context* ctx = get_can_ctx(robot.hcan);
    if (ctx) Log("PROTECT CAN alst=%lu terr=%lu full=%lu", (unsigned long)ctx->tx_arbitration_losses,
        (unsigned long)ctx->tx_transmission_errors, (unsigned long)ctx->RxFifo0FullCallbackCnt);
    locked = alarm = protectionSeen = true; robot.startPending = false;
    BeginStop(false);
}
void ProtectionCoordinator::Health() {
    uint32_t now = HAL_GetTick();
    if (now - lastHealth >= 20) {
        for (uint8_t count = 0; count < 8; ++count) {
            if (healthNode > 8) healthNode = 1;
            const uint8_t n = healthNode++; Node& a = nodes[n];
            if (a.probePending && now - a.probeFirstMs > 300) a.probePending = false;
            if (a.probePending ? now - a.probeSentMs < 40 : now - a.healthMs < 100) continue;
            const uint16_t probe = a.probePending ? a.issuedProbe : NextId();
            if (Context(n, QUERY, probe)) lastHealth = now;
            break;
        }
    }
    if (robot.stopState == DummyRobot::NORMAL && robot.isEnabled) {
        for (uint8_t n = 1; n <= 7; ++n) {
            const Node& a = nodes[n];
            const char* reason = now-a.aliveMs > 1000 ? "NODE_UNRESPONSIVE" :
                now-a.healthMs > 1000 ? "PROTECTION_STATUS_TIMEOUT" :
                !(a.probeFlags & FEEDBACK_VALID) ? "NO_VALID_ENCODER" :
                !(a.probeFlags & PROFILE_VALID) ? "PROFILE_INVALID" :
                a.probeVersion != a.profileVersion ? "COORDINATE_CHANGED" : nullptr;
            if (reason) { HealthFault(n, reason); break; }
        }
    }
}
void ProtectionCoordinator::FinishProtection() {
    for (uint8_t n = 1; n <= 7; ++n) SyncPosition(n);
    robot.commandHandler.ClearFifo(); robot.motionActive = false;
    job = NONE; locked = true; protectionSeen = true; Log("STALL locked");
    stalled = finished = 0;
    if (abortedCalibration) {
        robot.startPending = false; abortedCalibration = false;
        Log("error CALIBRATION failed; START required");
    } else if (robot.startPending) { robot.startPending = false; InterceptStart(); }
}
void ProtectionCoordinator::Poll() {
    pollEpoch = robot.motionEpoch;
    const uint8_t round = ++pollRound & 7;
    txUsed = false;
    // STOP gets its even 10ms slots first. Leases use other available cycles
    // by elapsed time, with an ACK and short retry budget for every node.
    allowTransmit = !stopUsed; stopUsed = false;
    if (allowTransmit) Lease();
    if (round == 1 && !txUsed) CancelContexts();
    allowTransmit = round == 5;
    if (allowTransmit && (pollRound & 8)) CancelContexts();
    // Snapshots are copied under interrupt masking by the task; never print in ISR.
    for (uint8_t n = 1; n <= 8; ++n) {
        if (nodes[n].ack && n == ackCursor) {
            uint8_t frame[8] = {}; uint32_t event;
            taskENTER_CRITICAL(); event = nodes[n].event; frame[4] = nodes[n].eventPhase; taskEXIT_CRITICAL();
            std::memcpy(frame, &event, 4);
            if (Send(n, EVENT_ACK, frame)) {
                taskENTER_CRITICAL(); if (event == nodes[n].event && frame[4] == nodes[n].eventPhase) nodes[n].ack = false; taskEXIT_CRITICAL();
            }
        }
        if (nodes[n].dirty) {
            taskENTER_CRITICAL(); nodes[n].dirty = false;
            uint8_t phase = nodes[n].eventPhase, reason = nodes[n].reason; taskEXIT_CRITICAL();
            if (calibration) Log("CAL J%u phase=%u reason=%s", n, phase, ReasonName(reason));
            else if (n == 8 && phase != IDLE && phase < CAL_READY)
                Log("HAND protection phase=%u reason=%s", phase, ReasonName(reason));
            else if (phase != IDLE && phase < CAL_READY) Log("STALL J%u phase=%u reason=%s", n, phase, ReasonName(reason));
        }
    }
    if (round == 5) { if (++ackCursor > 8) ackCursor = 1; }
    allowTransmit = round == 3 || round == 7;
    if (trigger) {
        trigger = false;
        if (calibration) {
            Log("error CALIBRATION abnormal contact; retreat required");
            for (uint8_t n = 1; n <= 8; ++n)
                if (!(nodes[n].flags & LOCAL_STALL)) {
                    cancelTasks[n] = task; cancelVersions[n] = nodes[n].version; cancelAt[n] = HAL_GetTick();
                }
            calibration = false; abortedCalibration = true; robot.startPending = false;
        }
        if (job != PROTECT_STOP && job != PROTECT_RUN) {
            sink = nullptr; task = NextId(); stalled = finished = 0;
            protectionSeen = true; Enter(PROTECT_STOP); BeginStop(true, abortedCalibration);
        }
        for (uint8_t n = 1; n <= 7; ++n)
            if ((nodes[n].flags & LOCAL_STALL) && seenEvents[n] != nodes[n].event) {
                stalled |= uint16_t(1U << n); seenEvents[n] = nodes[n].event;
            }
    }
    uint32_t now = HAL_GetTick();
    switch (job) {
    case NONE:
        if (robot.startPending && (robot.stopState == DummyRobot::STOPPED || robot.stopState == DummyRobot::DISABLED)) {
            robot.startPending = false; InterceptStart();
        } else Health();
        break;
    case START_QUERY: case START_VERIFY: case CAL_QUERY:
        if (failed) { Error("NO_VALID_ENCODER"); break; }
        if ((received & expected) == expected) {
            // PROBE_REPLY carries version, flags and real feedback atomically.
            bool fresh = true;
            for (uint8_t n = 1; n <= 8; ++n)
                if (!nodes[n].version || nodes[n].probe != queryProbe || now - nodes[n].healthMs > 300) fresh = false;
            if (fresh) {
                if (job == CAL_QUERY) {
                    for (uint8_t n = 1; n <= 8; ++n) if (!(nodes[n].probeFlags & DISABLED_FLAG)) {
                        Log("error CALIBRATION require DISABLE J%u", n);
                        job = NONE; calibration = false; locked = true;
                        robot.startPending = false; return;
                    }
                    task = NextId(); Enter(CAL_START, 0x1FE);
                }
                else if (job == START_VERIFY) {
                    bool valid = true;
                    for (uint8_t n = 1; n <= 7; ++n)
                        if (!(nodes[n].probeFlags & PROFILE_VALID) || nodes[n].probeVersion != nodes[n].profileVersion) {
                            Error("START_PROFILE_CHANGED", n); valid = false; break;
                        }
                    if (valid) {
                        job = NONE; suppressEpisodes = false;
                        internal = true; robot.RequestStart(); internal = false;
                    }
                } else { active = 1; stage = 0; task = NextId(); Enter(PROFILE); }
                break;
            }
        }
        if (now - phaseAt > 2000) { Error("START_QUERY_TIMEOUT"); break; }
        if (now - lastSend >= 20) {
            if (cursor > 8) cursor = 1;
            if (Context(cursor, QUERY, queryProbe)) { ++cursor; lastSend = now; }
        }
        break;
    case PROFILE: PollProfile(); break;
    case PROTECT_STOP:
        for (uint8_t n = 1; n <= 7; ++n) if (stalled & (1U << n)) {
            if (nodes[n].eventPhase == HOLD_FAILED) { Error(ReasonName(nodes[n].reason), n); return; }
            if (nodes[n].task != task) Context(n, PROTECT_PREPARE, task, 0, seenEvents[n]);
        }
        if (robot.stopState == DummyRobot::STOPPED) {
            active = 1; while (active <= 7 && !(stalled & (1U << active))) ++active;
            if (active > 7) { Error("NO_STALL_EVENT"); break; }
            Enter(PROTECT_RUN);
        }
        break;
    case PROTECT_RUN:
        for (uint8_t n = 1; n <= 7; ++n) if ((stalled & (1U << n)) && !(finished & (1U << n))) {
            if (nodes[n].eventPhase == HOLD_FAILED) { Error(ReasonName(nodes[n].reason), n); return; }
            // A late event still needs its own PREPARE before GO or lease.
            if (nodes[n].task != task) Context(n, PROTECT_PREPARE, task, 0, seenEvents[n]);
        }
        if (nodes[active].task == task && nodes[active].phase == HOLD_SUCCESS &&
            Finite(nodes[active].position)) {
            SyncPosition(active); finished |= uint16_t(1U << active);
            active = 1;
            while (active <= 7 && (!(stalled & (1U << active)) || (finished & (1U << active)))) ++active;
            if (active > 7) { FinishProtection(); break; }
            phaseAt = now; lastSend = now - 25;
        } else if ((nodes[active].eventPhase == HOLD_FAILED || nodes[active].phase == HOLD_FAILED) ||
                   now - phaseAt > 31000 || (now - phaseAt > 1500 && now - nodes[active].contextMs > 1500)) {
            Error(nodes[active].reason ? ReasonName(nodes[active].reason) : "RETREAT_TIMEOUT_OR_OFFLINE", active); break;
        }
        if (now - lastSend >= 20 && Context(active, RETREAT_GO, task, 0, seenEvents[active])) lastSend = now;
        break;
    case CAL_START:
        PollGroup(ProtectionProtocol::CAL_BEGIN, CAL_READY, 0x1FE);
        if (job == CAL_START && received == 0x1FE) Enter(CAL_ZERO, 0x7E);
        break;
    case CAL_ZERO:
        PollGroup(SET_CAPTURE_COORD, CAL_DONE, 0x7E);
        if (job == CAL_ZERO && received == 0x7E) {
            robot.initPose = robot.currentJoints = robot.targetJoints = robot.REST_POSE;
            active = 8; Enter(CAL_HAND); Log("CAL J8 SEEK");
        }
        break;
    case CAL_HAND: case CAL_RAIL:
        if (nodes[active].task == task && nodes[active].contextMs >= phaseAt &&
            nodes[active].phase == CAL_DONE && nodes[active].reason == NONE && Finite(nodes[active].position)) {
            SyncPosition(active);
            if (job == CAL_HAND) { active = 7; Enter(CAL_RAIL); Log("CAL J7 SEEK"); }
            else {
                Enter(CAL_DISABLE); Log("CAL DISABLING");
                internal = true; robot.RequestDisable(); internal = false;
            }
            break;
        }
        if ((nodes[active].task == task && nodes[active].phase == CAL_FAILED) ||
            now - phaseAt > 1270000U || (now - phaseAt > 1500 && now - nodes[active].contextMs > 1500)) {
            Error(nodes[active].reason ? ReasonName(nodes[active].reason) : "HOMING_TIMEOUT_OR_OFFLINE", active); break;
        }
        if (now - lastSend >= 100 && Context(active, CAL_RUN, task, active == 8 ? robot.hand->current : 0)) lastSend = now;
        break;
    case CAL_DISABLE:
        if (robot.stopState == DummyRobot::DISABLED) {
            active = 1; saved = saveInFlight = 0; Enter(CAL_SAVE); Log("CAL SAVING");
        } else if (now - phaseAt > 3000) Error("DISABLE_TIMEOUT");
        break;
    case CAL_SAVE:
        if (nodes[active].task == task && nodes[active].contextMs >= phaseAt && nodes[active].phase == SAVED) {
            saved |= uint16_t(1U << active);
            saveInFlight &= ~uint16_t(1U << active);
            if (++active > 8) { Enter(CAL_FINISH, 0x1FE); break; }
            phaseAt = now; lastSend = now - 25;
        }
        if ((nodes[active].task == task && nodes[active].reason == SAVE_FAILED) || now - phaseAt > 2500) {
            Error("EEPROM_SAVE_FAILED", active); break;
        }
        if (now - lastSend >= 50 && Context(active, SAVE_COORD, task)) {
            lastSend = now; saveInFlight |= uint16_t(1U << active);
        }
        break;
    case CAL_FINISH:
        PollGroup(ProtectionProtocol::CAL_END, IDLE, 0x1FE);
        if (job == CAL_FINISH && received == 0x1FE) {
            calibration = locked = alarm = protectionSeen = false; job = NONE;
            robot.currentJoints = robot.targetJoints = robot.REST_POSE;
            robot.currentRailPos = robot.targetRailPos = 0;
            robot.commandHandler.ClearFifo(); robot.motionActive = false; robot.startPending = false;
            Log("!CALIBRATION ok"); Log("Disabled ok");
        }
        break;
    case SETTINGS_GET:
        if (round == 3) Health();
        if (job == SETTINGS_GET) PollParameters();
        break;
    case SETTINGS_SET: case SETTINGS_SAVE: PollParameters(); break;
    }
}

const char* ProtectionCoordinator::ParameterName(uint8_t id, uint8_t n) const {
    static const char* const names[] = {"", "ENABLE", "CURRENT_A", "SPEED_RPS", "ERROR_R",
        "TRIGGER_MS", "STARTUP_MS", "CURRENT_FILTER_MS", "SPEED_FILTER_MS", "BACK_DISTANCE_R",
        "BACK_RPS", "BACK_ACCEL", "BACK_CURRENT_A", "HOLD_CURRENT_A", "BACK_POS_TOL_R",
        "BACK_TIMEOUT_MS", "REBLOCK_MS", "GRANT_WAIT_MS", "MIN_POSITION_R", "MAX_POSITION_R",
        "DIRECTION_MASK", "SEEK_RPS", "RETURN_RPS", "RETOUCH_RPS", "ACCEL", "CURRENT_A",
        "CONTACT_RATIO", "SPEED_RPS", "ERROR_R", "TRIGGER_MS", "STARTUP_MS", "MAX_TRAVEL_R",
        "SEEK_TIMEOUT_MS", "RETURN_TIMEOUT_MS", "RETOUCH_TIMEOUT_MS", "BACKOFF_R", "REPEAT_TOL_R",
        "POSITION_TOL_R", "CONTACT_WINDOW_R"};
    if (id == BACK_DISTANCE_R) return n == 7 ? "BACK_MM" : "BACK_DEG";
    return id <= PARAMETER_COUNT ? names[id] : "UNKNOWN";
}
bool ProtectionCoordinator::Command(const char* cmd, unsigned len, StreamSink& response) {
    std::string text(cmd, len);
    size_t first = text.find_first_not_of(" \t\r\n"), last = text.find_last_not_of(" \t\r\n");
    if (first == std::string::npos) return false;
    text = text.substr(first, last-first+1);
    if (text == "!CALIBRATION") {
        if (Locked() || robot.stopState != DummyRobot::DISABLED) {
            Respond(response, "error CALIBRATION require DISABLE"); return true;
        }
        sink = &response; calibration = locked = true; saved = saveInFlight = 0; task = NextId();
        suppressEpisodes = false;
        robot.startPending = false; ++robot.motionEpoch; robot.motionActive = false;
        robot.commandHandler.ClearFifo(); robot.tuningHelper.SetTuningFlag(0);
        queryProbe = task;
        Enter(CAL_QUERY, 0x1FE); Respond(response, "!CALIBRATION pending"); return true;
    }
    if (text.rfind("#STALL_", 0) != 0 && text.rfind("#CAL_", 0) != 0) return false;
    char verb[20]{}, key[32]{}, extra[2]{}; int node = -1; float v = 0;
    int count = sscanf(text.c_str(), "%19s %d %31s %f %1s", verb, &node, key, &v, extra);
    bool cal = text.rfind("#CAL_", 0) == 0;
    const char* operation = verb + (cal ? 5 : 7);
    bool get = std::strcmp(operation, "GET") == 0, set = std::strcmp(operation, "SET") == 0,
         save = std::strcmp(operation, "SAVE") == 0;
    if (!(get || set || save) || node < 0 || node > 8 || (cal && node != 7 && node != 8) ||
        count != (set ? 4 : 2) || (set && !Finite(v))) {
        Respond(response, "error %s syntax", verb); return true;
    }
    if (Locked()) { Respond(response, "error %s busy", verb); return true; }
    if (!cal && node == 8) {
        Respond(response, get ? "ok STALL [8] EXCLUDED=1" : "error STALL_SET J8 EXCLUDED"); return true;
    }
    if ((set || save) && robot.stopState != DummyRobot::DISABLED) {
        Respond(response, "error %s require DISABLE", verb); return true;
    }
    uint8_t chosen = 0;
    if (set) {
        uint8_t from = cal ? CAL_SEEK_RPS : ProtectionProtocol::ENABLE, to = cal ? PARAMETER_COUNT : DIRECTION_MASK;
        for (uint8_t id = from; id <= to; ++id)
            if (!std::strcmp(key, ParameterName(id, node == 0 ? 1 : uint8_t(node)))) chosen = id;
        if (!cal && (!std::strcmp(key, "BACK_DEG") || !std::strcmp(key, "BACK_MM") ||
                     !std::strcmp(key, "BACK_DISTANCE_R"))) chosen = BACK_DISTANCE_R;
        if (!chosen || chosen == MIN_POSITION_R || chosen == MAX_POSITION_R || chosen == DIRECTION_MASK ||
            (!std::strcmp(key,"BACK_DEG") && node == 7) ||
            (!std::strcmp(key,"BACK_MM") && node >= 1 && node <= 6)) {
            Respond(response, "error %s parameter", verb); return true;
        }
    }
    sink = &response; calParameters = cal; requestNode = uint8_t(node);
    expected = node == 0 ? (!std::strcmp(key, "BACK_MM") ? 0x80 :
                !std::strcmp(key,"BACK_DEG") ? 0x7E : 0xFE) : uint16_t(1U << node);
    uint16_t mask = expected;
    active = 1; while (!(mask & (1U << active))) ++active;
    firstParam = cal ? CAL_SEEK_RPS : ProtectionProtocol::ENABLE; lastParam = cal ? PARAMETER_COUNT : DIRECTION_MASK;
    param = get ? firstParam : chosen; value = v;
    if (set && chosen == BACK_DISTANCE_R && std::strcmp(key, "BACK_DISTANCE_R"))
        value = v * (active == 7 ? 0.2f : float(robot.motorJ[active]->reduction) / 360);
    // ALL distance is stored in user units until each node conversion (J6 differs).
    if (set && chosen == BACK_DISTANCE_R && node == 0) {
        firstParam = !std::strcmp(key,"BACK_DISTANCE_R") ? 0 : !std::strcmp(key,"BACK_MM") ? 2 : 1;
        value = v;
    } else if (set) firstParam = 0;
    stage = 0; task = NextId(); Enter(get ? SETTINGS_GET : set ? SETTINGS_SET : SETTINGS_SAVE, mask);
    Respond(response, "ok %s pending", verb); return true;
}
void ProtectionCoordinator::PollParameters() {
    uint8_t op = stage == 0 ? BEGIN : COMMIT;
    bool reply = job == SETTINGS_GET ? TakeReply(param) : job == SETTINGS_SAVE ? TakeReply(0, SAVE) :
        TakeReply(stage == 1 ? param : 0, stage == 1 ? 0 : op);
    if (reply) {
        if (replyStatus != OK) { Error("REJECTED", active); return; }
        bool done = false;
        if (job == SETTINGS_GET) {
            float v = replyValue;
            if (param == BACK_DISTANCE_R) v *= active == 7 ? 5 : 360.0f / robot.motorJ[active]->reduction;
            Log("ok %s [%u] %s=%.6g", calParameters ? "CAL" : "STALL", active, ParameterName(param, active), double(v));
            done = ++param > lastParam;
        } else if (job == SETTINGS_SET) {
            done = ++stage > 2;
            if (done) {
                float v = value;
                if (param == BACK_DISTANCE_R && !(requestNode == 0 && firstParam))
                    v *= active == 7 ? 5 : 360.0f / robot.motorJ[active]->reduction;
                Log("ok %s_SET [%u] %s=%.6g", calParameters ? "CAL" : "STALL", active, ParameterName(param, active), double(v));
            }
        } else { done = true; Log("ok %s_SAVE [%u] saved", calParameters ? "CAL" : "STALL", active); }
        phaseAt = HAL_GetTick(); lastSend = phaseAt - 25;
        if (done) {
            received |= uint16_t(1U << active);
            do { ++active; } while (active <= 8 && !(expected & (1U << active)));
            if (active > 8) { job = NONE; return; }
            if (job == SETTINGS_GET) param = firstParam;
            stage = 0; task = NextId();
        }
    }
    if (HAL_GetTick() - phaseAt > (job == SETTINGS_SAVE ? 2500U : 1500U)) { Error("TIMEOUT", active); return; }
    if (HAL_GetTick() - lastSend < 20) return;
    bool sent;
    if (job == SETTINGS_GET) sent = Parameter(active, false, param);
    else if (job == SETTINGS_SAVE) sent = Config(active, SAVE);
    else if (stage == 0) sent = Config(active, BEGIN);
    else if (stage == 2) sent = Config(active, COMMIT);
    else {
        float v = value;
        if (param == BACK_DISTANCE_R && requestNode == 0 && firstParam)
            v *= firstParam == 2 ? 0.2f : float(robot.motorJ[active]->reduction) / 360;
        sent = Parameter(active, true, param, v);
    }
    if (sent) lastSend = HAL_GetTick();
}
