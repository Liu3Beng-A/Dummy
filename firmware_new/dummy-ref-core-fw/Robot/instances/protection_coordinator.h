#ifndef DUMMY_PROTECTION_COORDINATOR_H
#define DUMMY_PROTECTION_COORDINATOR_H
#include <stdint.h>
#include "../../../protection_protocol.h"
class DummyRobot;
class StreamSink;

// One owner for internal protection motion, coordinate changes and flash saves.
// The CAN ISR only records snapshots. Control task sends/retries; OLED prints.
class ProtectionCoordinator {
public:
    explicit ProtectionCoordinator(DummyRobot& robot) : robot(robot) {}
    bool Locked() const { return locked || job != NONE; }
    bool CalibrationBusy() const { return calibration; }
    bool Busy() const { return job != NONE; }
    bool ParameterBusy() const { return job == SETTINGS_GET || job == SETTINGS_SET || job == SETTINGS_SAVE; }
    bool BlockAutoStart() const { return job != NONE || locked; }
    // STOP runs before Poll increments: reserve new rounds 3/7 for PREPARE/leases.
    bool StopSendSlot() const { return job == PROTECT_STOP ? (pollRound & 3) != 2 : (pollRound & 1) != 0; }
    bool FastStopConfirm() const { return job == PROTECT_STOP; }
    void StopAttempted() { stopUsed = true; }
    void ObserveCommunication(uint8_t node);
    bool InterceptStart();
    void Cancel(bool disable);
    void MotionDispatchFailed();
    void Started();
    void Disabled();
    bool Command(const char* command, unsigned length, StreamSink& response);
    void Receive(uint8_t node, uint8_t command, const uint8_t* data);
    void Poll();
    void Print();
private:
    enum Job : uint8_t { NONE, START_QUERY, START_VERIFY, CAL_QUERY, PROFILE, PROTECT_STOP, PROTECT_RUN,
        CAL_START, CAL_ZERO, CAL_WAIT, CAL_MOVE, CAL_DISABLE, CAL_SAVE, CAL_FINISH,
        SETTINGS_GET, SETTINGS_SET, SETTINGS_SAVE };
    struct Node {
        uint32_t event = 0, eventMs = 0, contextMs = 0, healthMs = 0;
        uint32_t aliveMs = 0, probeSentMs = 0, probeFirstMs = 0;
        uint32_t probeTx = 0, probeRx = 0, probeRetry = 0;
        uint32_t leaseNonce = 0, leaseSentMs = 0, leaseAckMs = 0;
        uint16_t leaseTask = 0;
        uint8_t profileVersion = 0, probeVersion = 0, leaseVersion = 0;
        bool probePending = false;
        uint16_t task = 0, probe = 0, issuedProbe = 0;
        uint8_t phase = 0, reason = 0, version = 0, flags = 0, eventPhase = 0, probeFlags = 0;
        uint8_t eventReason = 0; // CONTEXT request rejection cannot overwrite EVENT cause.
        float position = 0;
        bool dirty = false, ack = false;
    };
    DummyRobot& robot;
    volatile Job job = NONE;
    volatile bool locked = false, calibration = false;
    bool internal = false, alarm = false, protectionSeen = false, abortedCalibration = false;
    volatile bool trigger = false;
    volatile bool suppressEpisodes = false;
    Node nodes[9]{};
    uint32_t seenEvents[9]{};
    uint32_t phaseAt = 0, lastSend = 0, lastRenew = 0, lastHealth = 0;
    uint32_t calRunAt[2]{}; // Independent CAL_RUN retries for J7 and J8.
    uint32_t pollEpoch = 0;
    uint32_t leaseSerial = 0;
    uint16_t serial = 0, task = 0, queryProbe = 0, expected = 0, received = 0;
    uint16_t failed = 0, stalled = 0, finished = 0, saved = 0, saveInFlight = 0, cancelMask = 0;
    uint16_t cancelTasks[9]{};
    uint8_t cancelVersions[9]{};
    uint32_t cancelAt[9]{};
    uint8_t active = 0, cursor = 1, stage = 0, param = 0, firstParam = 0;
    uint8_t lastParam = 0, requestNode = 0, healthNode = 1, renewNode = 1;
    uint8_t pollRound = 0, cancelCursor = 1, ackCursor = 1;
    bool txUsed = false, allowTransmit = false, stopUsed = false;
    bool goPriorityPending = false; // One first-GO opportunity, never unlimited priority.
    float value = 0;
    bool calParameters = false, startAfterProfile = false;
    volatile bool replyReady = false;
    uint8_t replyNode = 0, replyParam = 0, replyStatus = 0, replyOp = 0;
    uint16_t replyTransaction = 0;
    float replyValue = 0;
    StreamSink* sink = nullptr;
    // Independent read-only transaction; never contributes to Locked()/Busy().
    enum DiagnosticState : uint8_t { DIAG_IDLE, DIAG_COLLECT, DIAG_READY, DIAG_TIMEOUT };
    volatile DiagnosticState diagnosticState = DIAG_IDLE;
    StreamSink* diagnosticSink = nullptr;
    uint32_t diagnosticValues[ProtectionProtocol::DIAG_FIELD_COUNT]{};
    uint32_t diagnosticMask = 0, diagnosticAt = 0, diagnosticLastSend = 0;
    uint16_t diagnosticSerial = 0, diagnosticRequest = 0;
    uint8_t diagnosticNode = 0, diagnosticLine = 0;
    uint8_t diagnosticSelector = ProtectionProtocol::DS_LIVE;
    struct Notice { StreamSink* sink; char text[64]; };
    Notice notices[24]{};
    uint8_t writeNotice = 0, readNotice = 0;
    uint16_t NextId();
    void Enter(Job next, uint16_t mask = 0);
    void Log(const char* format, ...);
    void Error(const char* reason, uint8_t node = 0);
    bool Send(uint8_t node, uint8_t command, const uint8_t* data);
    bool Context(uint8_t node, uint8_t operation, uint16_t id, float v = 0,
                 uint32_t event = 0);
    bool Config(uint8_t node, uint8_t operation, uint32_t argument = 0);
    bool Parameter(uint8_t node, bool set, uint8_t parameter, float v = 0);
    void BeginStop(bool protecting, bool includeHand = false);
    void FinishProtection();
    void SyncPosition(uint8_t node);
    void PollGroup(uint8_t operation, uint8_t phase, uint16_t mask);
    void PollProfile();
    void PollParameters();
    void PollDiagnostic();
    void PrintDiagnostic();
    void CancelContexts();
    void Lease();
    void Health();
    void HealthFault(uint8_t node, const char* reason);
    bool TakeReply(uint8_t parameter, uint8_t operation = 0);
    float Bound(uint8_t node, bool maximum) const;
    const char* ParameterName(uint8_t id, uint8_t node) const;
};
#endif
