#include "motor.h"
#include "configurations.h"
#include "can.h"
#include "fw_build_identity.h"
#include <cstring>
#include <cmath>
#include <initializer_list>

using namespace ProtectionProtocol;
// Main owns the electrical encoder calibrator and legacy EEPROM save state.
extern bool MotorDefaultsRestoreAllowed();
// Keep diagnostics within the existing 95KiB application area. Startup,
// CAN/main queries and one-shot phase/capture work use size optimization;
// periodic 20kHz control/detection functions retain their original options.
#if defined(__GNUC__)
#define PROTECTION_CAN_SIZE __attribute__((optimize("Os")))
#else
#define PROTECTION_CAN_SIZE
#endif
namespace {
static_assert(sizeof(FWIdentity::WORDS) == FW_VERSION_FIELD_COUNT * sizeof(uint32_t), "Firmware identity schema size mismatch");
static_assert(FW_VERSION_FIELD_COUNT <= DIAG_FIELD_COUNT, "Firmware identity exceeds diagnostic snapshot");
static_assert(MOTOR_CAN_DIAG_WORDS == CAN_DIAG_FIELD_COUNT && unsigned(MC_BOOT_SESSION) == unsigned(CD_BOOT_SESSION) &&
              unsigned(MC_CONTROL_MAX) == unsigned(CD_CONTROL_MAX) && unsigned(MC_QUERY_RX) == unsigned(CD_QUERY_RX) &&
              unsigned(MC_NONCES) == unsigned(CD_NONCES),
              "C/C++ CAN diagnostic field numbering must agree");
constexpr int32_t STEPS = Motor::MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
constexpr uint32_t HZ = 20000;
struct IrqGuard {
    uint32_t mask;
    IrqGuard() : mask(__get_PRIMASK()) { __disable_irq(); }
    ~IrqGuard() { __set_PRIMASK(mask); }
};
uint16_t U16(const uint8_t* p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
uint32_t U32(const uint8_t* p) { uint32_t v; memcpy(&v, p, 4); return v; }
float F32(const uint8_t* p) { float v; memcpy(&v, p, 4); return v; }
int32_t Steps(float v) { return static_cast<int32_t>(v * STEPS); }
uint32_t Ticks(float ms) { return static_cast<uint32_t>(ms * 20); }
int32_t Min(int32_t a, int32_t b) { return a < b ? a : b; }
int32_t Max(int32_t a, int32_t b) { return a > b ? a : b; }
int8_t Sign(int32_t v) { return v > 0 ? 1 : v < 0 ? -1 : 0; }
bool MovingPhase(uint8_t phase) {
    return phase == RETREATING || phase == CAL_SEEK || phase == CAL_BACKOFF ||
           phase == CAL_RETOUCH || phase == CAL_RETURN;
}
int32_t Filter(int32_t previousQ, int32_t sample, uint32_t timeTicks) {
    if (!timeTicks) return sample * 1024;
    const int32_t target = sample * 1024, delta = target - previousQ;
    const int32_t step = delta / static_cast<int32_t>(timeTicks);
    // Fixed-point quantisation must not make a threshold equal to the exact
    // current cap unreachable forever. Snap only the sub-quantum remainder.
    return step ? previousQ + step : target;
}
void Send(uint8_t command, const uint8_t* data) {
    CAN_TxHeaderTypeDef header{};
    header.StdId = (boardConfig.canNodeId << 7) | command;
    header.IDE = CAN_ID_STD; header.RTR = CAN_RTR_DATA; header.DLC = 8;
    CAN_Send(&header, const_cast<uint8_t*>(data));
}
}

PROTECTION_CAN_SIZE void Motor::Controller::InitProtection() {
    protectionSettings.Defaults(boardConfig.canNodeId, context->config.motionParams.ratedCurrent);
    const float maxVelocity = context->config.motionParams.ratedVelocity / static_cast<float>(STEPS);
    const float maxAcceleration = context->config.motionParams.ratedVelocityAcc / static_cast<float>(STEPS);
    protectionSettings.value[BACK_RPS] = fminf(protectionSettings.Get(BACK_RPS), maxVelocity);
    protectionSettings.value[BACK_ACCEL] = fminf(5, maxAcceleration);
    protectionSettings.value[CAL_ACCEL] = fminf(protectionSettings.Get(CAL_ACCEL), maxAcceleration);
    protectionSettings.value[CAL_SEEK_RPS] = fminf(protectionSettings.Get(CAL_SEEK_RPS), maxVelocity);
    protectionSettings.value[CAL_RETURN_RPS] = fminf(protectionSettings.Get(CAL_RETURN_RPS), maxVelocity);
    protectionSettings.value[CAL_RETOUCH_RPS] = fminf(protectionSettings.Get(CAL_RETOUCH_RPS), maxVelocity);
    const bool loaded = LoadProtectionSettings(protectionSettings, boardConfig.canNodeId, context->config.motionParams.ratedCurrent,
                                              context->config.motionParams.ratedVelocity, context->config.motionParams.ratedVelocityAcc);
    configurationFault = !loaded && !ProtectionStorageIsErased();
    if (configurationFault) protectionReason = INVALID_PROFILE;
    if (boardConfig.canNodeId == 8) protectionSettings.value[ProtectionProtocol::ENABLE] = 0;
    context->config.ctrlParams.stallProtectSwitch = protectionSettings.Get(ProtectionProtocol::ENABLE) == 1;
    internalPlanner.AttachConfig(&internalConfig);
    internalConfig = context->config.motionParams;
    internalPlanner.positionTracker.Init(); internalPlanner.velocityTracker.Init();
    // Backup registers survive ordinary MCU reset. No per-boot EEPROM erase.
    // A fresh boot always invalidates the motion profile and old GO context.
    __HAL_RCC_PWR_CLK_ENABLE(); __HAL_RCC_BKP_CLK_ENABLE(); HAL_PWR_EnableBkUpAccess();
    uint16_t boot = static_cast<uint16_t>(BKP->DR1);
    if (boot == 0 || boot == 0xFFFF) boot = static_cast<uint16_t>(HAL_GetTick() ^ *(volatile uint32_t*)0x1FFFF7E8);
    if (++boot == 0) ++boot;
    BKP->DR1 = boot; bootSession = boot;
    UpdateProtectionLimits(); ResetDetection();
}

PROTECTION_CAN_SIZE void Motor::Controller::UpdateProtectionLimits() {
    // Software-float conversions run with the 20kHz timer allowed to preempt;
    // only the prepared integer snapshot publication is an IRQ critical section.
    const int32_t newCurrent = static_cast<int32_t>(protectionSettings.Get(CURRENT_A) * 1000);
    const int32_t newSpeed = Steps(protectionSettings.Get(SPEED_RPS)), newError = Steps(protectionSettings.Get(ERROR_R));
    const uint32_t newTrigger = Ticks(protectionSettings.Get(TRIGGER_MS)), newStartup = Ticks(protectionSettings.Get(STARTUP_MS));
    const uint32_t newCurrentFilter = Ticks(protectionSettings.Get(CURRENT_FILTER_MS)), newVelocityFilter = Ticks(protectionSettings.Get(SPEED_FILTER_MS));
    const uint32_t newReblock = Ticks(protectionSettings.Get(REBLOCK_MS)), newGrant = Ticks(protectionSettings.Get(GRANT_WAIT_MS));
    const int32_t newDistance = Steps(protectionSettings.Get(BACK_DISTANCE_R)), newVelocity = Steps(protectionSettings.Get(BACK_RPS));
    const int32_t newAcceleration = Steps(protectionSettings.Get(BACK_ACCEL));
    const int32_t newRetreatCurrent = static_cast<int32_t>(protectionSettings.Get(BACK_CURRENT_A) * 1000);
    const int32_t newHoldCurrent = static_cast<int32_t>(protectionSettings.Get(HOLD_CURRENT_A) * 1000);
    const int32_t newTolerance = Max(1, Steps(protectionSettings.Get(BACK_POS_TOL_R)));
    const uint32_t newTimeout = Ticks(protectionSettings.Get(BACK_TIMEOUT_MS));
    const int32_t newLow = Steps(protectionSettings.Get(MIN_POSITION_R)), newHigh = Steps(protectionSettings.Get(MAX_POSITION_R));
    const uint8_t newDirections = static_cast<uint8_t>(protectionSettings.Get(DIRECTION_MASK));
    IrqGuard guard;
    detectorCurrent = newCurrent; detectorSpeed = newSpeed; detectorError = newError;
    detectorTriggerTicks = newTrigger; detectorStartupTicks = newStartup;
    currentFilterTicks = newCurrentFilter; velocityFilterTicks = newVelocityFilter;
    reblockTicks = newReblock; grantWaitTicks = newGrant;
    retreatDistance = newDistance; retreatVelocity = newVelocity; retreatAcceleration = newAcceleration;
    retreatCurrent = newRetreatCurrent; holdingCurrent = newHoldCurrent; positionTolerance = newTolerance;
    retreatTimeoutTicks = newTimeout; profileLow = newLow; profileHigh = newHigh; profileDirections = newDirections;
    ClearDetectionTimer(DR_PARAMETERS); filteredCurrentQ = filteredVelocityQ = 0;
}
PROTECTION_CAN_SIZE void Motor::Controller::OnMotionLimitChange() {
    const float maxCurrent = boardConfig.currentLimit / 1000.0f;
    const float maxSpeed = boardConfig.velocityLimit / static_cast<float>(STEPS);
    const float maxAcceleration = boardConfig.velocityAcc / static_cast<float>(STEPS);
    // Lowering a legacy rated limit must not leave an unreachable detector
    // threshold or an over-limit internal protection profile. Query exposes
    // these adjusted live values; increasing a limit does not raise thresholds.
    if (protectionSettings.Get(CURRENT_A) > maxCurrent)
        protectionSettings.value[CURRENT_A] = maxCurrent * .8f;
    for (uint8_t id : {static_cast<uint8_t>(BACK_CURRENT_A), static_cast<uint8_t>(HOLD_CURRENT_A), static_cast<uint8_t>(CAL_CURRENT_A)})
        protectionSettings.value[id] = fminf(protectionSettings.Get(id), maxCurrent);
    for (uint8_t id : {static_cast<uint8_t>(BACK_RPS), static_cast<uint8_t>(CAL_SEEK_RPS), static_cast<uint8_t>(CAL_RETURN_RPS), static_cast<uint8_t>(CAL_RETOUCH_RPS)})
        protectionSettings.value[id] = fminf(protectionSettings.Get(id), maxSpeed);
    protectionSettings.value[BACK_ACCEL] = fminf(protectionSettings.Get(BACK_ACCEL), maxAcceleration);
    protectionSettings.value[CAL_ACCEL] = fminf(protectionSettings.Get(CAL_ACCEL), maxAcceleration);
    configStaging = false; UpdateProtectionLimits();
}
bool Motor::Controller::IsFeedbackValid() const {
    return feedbackInitialized && context->encoder && context->encoder->IsCalibrated() &&
           context->encoder->IsSampleValid() && invalidFeedbackTicks == 0;
}
void Motor::Controller::OnMasterCommunication() {
    masterSeen = true; masterSilenceTicks = 0;
}
void Motor::Controller::ResetDetection(bool newMotion) {
    if (newMotion) {
        if (++detectionSession == 0) ++detectionSession;
        detectionPeak = detectionLastTimer = detectionBadTotal = detectionBadPeak = 0;
        detectionReset = DR_NONE; detectionResetDetail = 0;
    } else if (detectionGate != DG_PROTECTED) ClearDetectionTimer(DR_CONTROL_RESET);
    candidateTicks = 0; directionTicks = 0; motionDirection = 0;
    filteredCurrentQ = filteredVelocityQ = 0;
    startupTicks = newMotion ? detectorStartupTicks : 0;
    if (!newMotion) motionIntent = false;
}
void Motor::Controller::ClearDetectionTimer(uint8_t reason, uint8_t detail) {
    if (candidateTicks) {
        detectionLastTimer = candidateTicks;
        detectionReset = reason; detectionResetDetail = detail;
    }
    candidateTicks = 0;
}
PROTECTION_CAN_SIZE void Motor::Controller::FillStallDiagnostic(uint32_t* s, uint8_t source, uint8_t reason) const {
    s[D_META] = uint32_t(modeRunning) | (uint32_t(requestMode) << 8) |
        (uint32_t(protectionPhase) << 16) | (uint32_t(detectionGate) << 24);
    s[D_FLAGS] = (motionIntent ? 1U : 0) | (IsFeedbackValid() ? 2U : 0) |
        (!goalDisable && !softDisable ? 4U : 0) |
        (context->config.ctrlParams.stallProtectSwitch ? 8U : 0) |
        (IsSoftStopLocked() ? 16U : 0) | (uint32_t(detectionUnmet) << 8);
    s[D_CURRENT] = uint16_t(focCurrent) | (uint32_t(filteredCurrentQ / 1024) << 16);
    s[D_RAW_SPEED] = uint32_t(estVelocity); s[D_FILTER_SPEED] = filteredVelocityQ / 1024;
    const int64_t error = int64_t(softPosition) - realPosition;
    memcpy(s + D_ERROR_LOW, &error, sizeof(error));
    s[D_TIMER] = source == DS_TRIGGER || detectionGate == DG_ELIGIBLE ? candidateTicks : 0;
    s[D_PEAK_TIMER] = detectionPeak;
    s[D_RESET] = detectionReset | (uint32_t(detectionResetDetail) << 8) | (uint32_t(detectionSession) << 16);
    s[D_BAD_TOTAL] = detectionBadTotal; s[D_BAD_PEAK] = detectionBadPeak;
    s[D_STARTUP] = startupTicks;
    s[D_CURRENT_LIMIT] = detectorCurrent; s[D_SPEED_LIMIT] = detectorSpeed;
    s[D_ERROR_LIMIT] = detectorError; s[D_TRIGGER] = detectorTriggerTicks;
    s[D_LAST_TIMER] = detectionLastTimer; s[D_CAPTURE_MS] = HAL_GetTick();
    s[D_EVENT] = protectionEvent;
    const int64_t goalError = int64_t(goalPosition) - realPosition;
    memcpy(s + D_GOAL_ERROR_LOW, &goalError, sizeof(goalError));
    s[D_REFERENCE_SPEED] = uint32_t(softVelocity);
    s[D_DIRECTION] = uint8_t(motionDirection) | (uint32_t(uint8_t(stallDirection)) << 8) |
        (uint32_t(reason) << 16) | (uint32_t(source) << 24);
}
PROTECTION_CAN_SIZE void Motor::Controller::FillCalDiagnostic(uint32_t* s, uint8_t source, uint8_t reason) const {
    FillStallDiagnostic(s, source, reason);
    s[D_META] = (s[D_META] & 0x00FFFFFFU) | (uint32_t(calDetectionGate) << 24);
    const bool search = protectionPhase == CAL_SEEK || protectionPhase == CAL_RETOUCH;
    s[D_FLAGS] = (s[D_FLAGS] & ~0xFF09U) |
        (calibrationTask && MovingPhase(protectionPhase) && !internalStopping ? 1U : 0) |
        (calibrationTask && search && !internalStopping ? 8U : 0) | (uint32_t(calDetectionUnmet) << 8);
    s[D_TIMER] = candidateTicks; s[D_PEAK_TIMER] = calDetectionPeak;
    s[D_LAST_TIMER] = calDetectionLastTimer;
    s[D_RESET] = calDetectionReset | (uint32_t(calDetectionResetDetail) << 8) | (uint32_t(protectionTask) << 16);
    s[D_CURRENT_LIMIT] = contactCurrent; s[D_SPEED_LIMIT] = contactSpeed;
    s[D_ERROR_LIMIT] = contactError;
    s[D_TRIGGER] = search ? contactTriggerTicks : MovingPhase(protectionPhase) ? reblockTicks : 0;
    s[D_BAD_TOTAL] = Min(calibrationCurrent, EffectiveCurrentLimit());
    s[D_BAD_PEAK] = searchMaxTravel; s[D_EVENT] = protectionTask;
    s[D_GOAL_ERROR_LOW] = uint32_t(int64_t(realPosition) - searchStart);
    s[D_GOAL_ERROR_HIGH] = uint32_t(int64_t(softPosition) - searchStart);
    s[D_DIRECTION] = uint8_t(searchDirection) | (uint32_t(reason) << 16) | (uint32_t(source) << 24);
}
PROTECTION_CAN_SIZE void Motor::Controller::FreezeCalFailure(uint8_t reason) {
    if (!calibrationTask || boardConfig.canNodeId < 7 || calFailureSnapshotValid) return;
    FillCalDiagnostic(calFailureSnapshot, DS_CAL_FAILURE, reason);
    calFailureSnapshotValid = true;
}
void Motor::Controller::ClearCalDetectionTimer(uint8_t reason, uint8_t detail) {
    if (candidateTicks) {
        calDetectionLastTimer = candidateTicks;
        calDetectionReset = reason; calDetectionResetDetail = detail;
    }
    candidateTicks = 0;
}
PROTECTION_CAN_SIZE void Motor::Controller::CaptureStallDiagnostic(uint16_t request, uint8_t selector) {
    IrqGuard guard;
    if (request == diagnosticRequest && selector != diagnosticSelector) return;
    if (request != diagnosticRequest) {
        if (selector == DS_CAN_DIAG) {
            CAN_CaptureDiagnostic(diagnosticSnapshot);
            diagnosticSnapshot[CD_MAGIC] = CAN_DIAG_MAGIC;
            // Type is a build identity, distinct from the configurable node ID.
            diagnosticSnapshot[CD_META] = CAN_DIAG_SCHEMA | (uint32_t(DS_CAN_DIAG) << 8) |
                (uint32_t(FW_TARGET_TYPE) << 16) |
                ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) ? (1U << 24) : 0U);
            diagnosticSnapshot[CD_BOOT_SESSION] = bootSession;
        }
        else if (selector == DS_FW_VERSION) memcpy(diagnosticSnapshot, FWIdentity::WORDS, sizeof(FWIdentity::WORDS));
        else if (selector == DS_LIVE) FillStallDiagnostic(diagnosticSnapshot, DS_LIVE);
        else if (selector == DS_CAL_LIVE) FillCalDiagnostic(diagnosticSnapshot, DS_CAL_LIVE);
        else if (selector == DS_CAL_FAILURE && calFailureSnapshotValid)
            memcpy(diagnosticSnapshot, calFailureSnapshot, sizeof(calFailureSnapshot));
        else if (selector == DS_TRIGGER && triggerSnapshotValid) memcpy(diagnosticSnapshot, triggerSnapshot, sizeof(triggerSnapshot));
        else {
            memset(diagnosticSnapshot, 0, sizeof(diagnosticSnapshot));
            diagnosticSnapshot[D_DIRECTION] = uint32_t(selector == DS_CAL_FAILURE ? DS_NO_CAL_FAILURE : DS_NO_TRIGGER) << 24;
        }
        diagnosticRequest = request;
        diagnosticSelector = selector;
    }
    diagnosticCursor = 0;
}
PROTECTION_CAN_SIZE void Motor::Controller::PollStallDiagnostic() {
    uint8_t data[8]{};
    const uint32_t now = HAL_GetTick();
    {
        IrqGuard guard;
        const uint8_t fieldCount = diagnosticSelector == DS_FW_VERSION ? FW_VERSION_FIELD_COUNT : DIAG_FIELD_COUNT;
        if (diagnosticCursor >= fieldCount || now - diagnosticSendMs < 1) return;
        data[0] = diagnosticCursor++; data[1] = DIAG_VERSION;
        data[2] = uint8_t(diagnosticRequest); data[3] = uint8_t(diagnosticRequest >> 8);
        memcpy(data + 4, diagnosticSnapshot + data[0], 4);
        diagnosticSendMs = now;
    }
    // Ordinary queue: protection replies retain priority over diagnostics.
    Send(DIAG_REPLY, data);
}
void Motor::Controller::SetProtectionPhase(uint8_t phase, uint8_t reason) {
    if (protectionPhase == phase && protectionReason == reason) return;
    protectionPhase = phase; protectionReason = reason; phaseTicks = 0;
    eventAcknowledged = false;
}
PROTECTION_CAN_SIZE void Motor::Controller::BuildProtectionEvent(uint8_t* data) const {
    const uint32_t event = protectionEvent;
    memcpy(data, &event, 4);
    data[4] = protectionPhase; data[5] = protectionReason; data[6] = coordinateVersion;
    data[7] = (IsFeedbackValid() ? FEEDBACK_VALID : 0) | (profileValid ? PROFILE_VALID : 0) |
              (assumedBoot ? ASSUMED_BOOT : 0) | (coordinateDirty ? COORD_DIRTY : 0) |
              (distanceClipped ? DISTANCE_CLIPPED : 0) | (localStall ? LOCAL_STALL : 0) |
              (boardConfig.canNodeId == 8 ? EXCLUDED : 0) | (IsActuallyDisabled() ? DISABLED_FLAG : 0);
}
PROTECTION_CAN_SIZE void Motor::Controller::BuildProtectionContext(uint8_t* data, uint16_t query, bool probe) const {
    uint8_t event[8]{}; BuildProtectionEvent(event);
    data[0] = protectionPhase | (probe ? QUERY_REPLY_FLAG : 0);
    data[1] = probe ? event[7] : protectionReason;
    const uint16_t task = probe ? query : protectionTask;
    data[2] = static_cast<uint8_t>(task); data[3] = static_cast<uint8_t>(task >> 8);
    const float position = IsFeedbackValid() ? static_cast<float>(realPosition - context->config.motionParams.encoderHomeOffset) / STEPS : NAN;
    memcpy(data + 4, &position, 4);
}
void Motor::Controller::ReplyParameter(uint8_t id, uint8_t status, uint16_t transaction, float value) {
    uint8_t data[8]{id, status, static_cast<uint8_t>(transaction), static_cast<uint8_t>(transaction >> 8)};
    memcpy(data + 4, &value, 4); Send(PARAM_REPLY, data);
}
void Motor::Controller::ReplyConfiguration(uint8_t op, uint8_t status, uint16_t transaction) {
    uint8_t data[8]{0, status, static_cast<uint8_t>(transaction), static_cast<uint8_t>(transaction >> 8), op, coordinateVersion, 0, 0};
    Send(PARAM_REPLY, data);
}

PROTECTION_CAN_SIZE bool Motor::Controller::ProcessProtectionCan(uint8_t command, const uint8_t* data, uint32_t length) {
    if (command == DIAG_QUERY) {
        if (length != 8) return true;
        const bool stall = boardConfig.canNodeId >= 1 && boardConfig.canNodeId <= 7 && data[0] <= DS_TRIGGER;
        const bool cal = (boardConfig.canNodeId == 7 || boardConfig.canNodeId == 8) &&
            (data[0] == DS_CAL_LIVE || data[0] == DS_CAL_FAILURE);
        const bool version = boardConfig.canNodeId >= 1 && boardConfig.canNodeId <= 8 && data[0] == DS_FW_VERSION;
        const bool can = boardConfig.canNodeId >= 1 && boardConfig.canNodeId <= 8 && data[0] == DS_CAN_DIAG;
        if ((stall || cal || version || can) && data[1] == DIAG_VERSION && U16(data + 2))
            CaptureStallDiagnostic(U16(data + 2), data[0]);
        return true;
    }
    if (command != EVENT_ACK && command != PARAM_SET && command != PARAM_GET && command != CONFIG && command != CONTEXT) return false;
    if (length != 8) return true;
    OnMasterCommunication();
    const uint32_t irq = __get_PRIMASK();
    if (command == EVENT_ACK) {
        IrqGuard guard;
        if (U32(data) == protectionEvent && data[4] == protectionPhase) eventAcknowledged = true;
    } else if (command == PARAM_GET) {
        const uint8_t id = data[0]; const uint16_t transaction = U16(data + 2);
        uint8_t status = id == 0 || id > PARAMETER_COUNT ? INVALID : OK;
        if ((boardConfig.canNodeId == 8 && id <= DIRECTION_MASK) ||
            (boardConfig.canNodeId != 7 && boardConfig.canNodeId != 8 && id >= CAL_SEEK_RPS)) status = NOT_SUPPORTED;
        ReplyParameter(id, status, transaction, status == OK ? protectionSettings.Get(id) : 0);
    } else if (command == PARAM_SET) {
        const uint8_t id = data[0]; const uint16_t transaction = U16(data + 2); const float value = F32(data + 4);
        uint8_t status = OK;
        if (defaultsRestoreRequested) status = LOCKED;
        else if (!configStaging || configTransaction != transaction) status = TRANSACTION_MISMATCH;
        else if (IsSoftStopLocked() && id != MIN_POSITION_R && id != MAX_POSITION_R && id != DIRECTION_MASK) status = LOCKED;
        else if (!stagedSettings.Set(id, value, boardConfig.canNodeId, boardConfig.currentLimit,
                                     boardConfig.velocityLimit, boardConfig.velocityAcc)) status = INVALID;
        else if (id == MIN_POSITION_R || id == MAX_POSITION_R || id == DIRECTION_MASK) stagedProfileChanged = true;
        ReplyParameter(id, status, transaction, value);
    } else if (command == CONFIG) {
        const uint8_t op = data[0]; const uint16_t transaction = U16(data + 2); const uint32_t argument = U32(data + 4);
        uint8_t status = OK;
        if (op == RESTORE_DEFAULTS) {
            IrqGuard guard;
            const uint32_t nonce = uint32_t(BKP->DR2) | (uint32_t(BKP->DR3) << 16);
            if (!argument || !transaction) status = INVALID;
            else if (nonce == argument && BKP->DR4 == transaction &&
                     (BKP->DR5 == RESTORE_READY || BKP->DR5 == RESTORE_FAILED))
                status = BKP->DR5 == RESTORE_READY ? OK : WRITE_FAILED;
            else if (defaultsRestoreRequested) status = nonce == argument && BKP->DR4 == transaction ? PENDING : LOCKED;
            else if (!IsActuallyDisabled() || IsSoftStopLocked() || calibrationTask || configStaging ||
                     saveRequested || parameterSaveRequested || parameterSavingInProgress || coordinateSavingInProgress ||
                     !MotorDefaultsRestoreAllowed()) status = LOCKED;
            else {
                BKP->DR2 = uint16_t(argument); BKP->DR3 = uint16_t(argument >> 16);
                BKP->DR4 = transaction; BKP->DR5 = 0;
                defaultsRestoreRequested = true; softStopLocked = true; status = PENDING;
            }
            ReplyConfiguration(op, status, transaction); return true;
        }
        if (defaultsRestoreRequested) { ReplyConfiguration(op, LOCKED, transaction); return true; }
        if (op == BEGIN) {
            IrqGuard guard;
            if (parameterSaveRequested || parameterSavingInProgress) {
                ReplyConfiguration(op, LOCKED, transaction); return true;
            }
            if (!configStaging || transaction != configTransaction) {
                stagedSettings = protectionSettings; configTransaction = transaction;
                configStaging = true; stagedProfileChanged = false;
            }
            lastConfigOperation = BEGIN;
        } else if (op == COMMIT) {
            bool changesNormalSettings = false;
            for (uint8_t id = 1; id <= PARAMETER_COUNT; ++id)
                if (id != MIN_POSITION_R && id != MAX_POSITION_R && id != DIRECTION_MASK &&
                    stagedSettings.Get(id) != protectionSettings.Get(id)) changesNormalSettings = true;
            if (!configStaging) {
                if (transaction != configTransaction || lastConfigOperation != COMMIT) status = TRANSACTION_MISMATCH;
                else if (argument && static_cast<uint8_t>(argument) != coordinateVersion) status = INVALID;
            } else if (transaction != configTransaction) status = TRANSACTION_MISMATCH;
            else if (IsSoftStopLocked() && changesNormalSettings) status = LOCKED;
            else if (!stagedSettings.Validate(boardConfig.canNodeId, boardConfig.currentLimit, boardConfig.velocityLimit, boardConfig.velocityAcc)) status = INVALID;
            else if ((argument && static_cast<uint8_t>(argument) != coordinateVersion) || (stagedProfileChanged && !argument)) status = INVALID;
            else {
                {
                IrqGuard guard;
                if (!configStaging || configTransaction != transaction || (IsSoftStopLocked() && changesNormalSettings)) {
                    ReplyConfiguration(op, TRANSACTION_MISMATCH, transaction); return true;
                }
                if (argument && static_cast<uint8_t>(argument) != coordinateVersion) {
                    ReplyConfiguration(op, INVALID, transaction); return true;
                }
                protectionSettings = stagedSettings; configStaging = false; lastConfigOperation = COMMIT;
                context->config.ctrlParams.stallProtectSwitch = boardConfig.canNodeId != 8 && protectionSettings.Get(ProtectionProtocol::ENABLE) == 1;
                if (argument) profileValid = true;
                else configurationFault = false; // explicit validated settings commit repairs a damaged record
                }
                UpdateProtectionLimits();
            }
        } else if (op == SAVE) {
            if (lastParameterSavedTransaction != 0 && transaction == lastParameterSavedTransaction) {
                status = lastParameterSaveStatus;
            }
            else if (!IsActuallyDisabled() || calibrationTask) status = LOCKED;
            else if (parameterSaveRequested || parameterSavingInProgress) {
                if (transaction == configTransaction) return true; // no final ACK before physical save finishes
                status = LOCKED;
            }
            else {
                configTransaction = transaction; parameterSaveRequested = true; lastConfigOperation = SAVE;
                __set_PRIMASK(irq); return true; // completion ACK comes after physical Flash verification
            }
        } else if (op == ABORT) {
            if (transaction == configTransaction) configStaging = false;
        } else status = INVALID;
        ReplyConfiguration(op, status, transaction);
    } else {
        const uint8_t op = data[0], version = data[1]; const uint16_t task = U16(data + 2);
        const uint32_t value = U32(data + 4);
        if (defaultsRestoreRequested && op != QUERY && op != CANCEL) {
            uint8_t reply[8]{}; BuildProtectionContext(reply); reply[1] = BUSY;
            reply[2] = uint8_t(task); reply[3] = uint8_t(task >> 8);
            Send(CONTEXT_REPLY, reply); return true;
        }
        if (op == QUERY) {
            uint8_t reply[8]{}, event[8]{};
            bool needsEvent;
            { IrqGuard guard; BuildProtectionContext(reply, task, true);
              reply[0] = coordinateVersion; BuildProtectionEvent(event);
              needsEvent = protectionPhase != IDLE && !eventAcknowledged; }
            Send(PROBE_REPLY, reply);
            if (needsEvent) Send(EVENT, event);
            __set_PRIMASK(irq); return true;
        }
        const bool validCalParams = op != CAL_RUN || protectionSettings.Validate(boardConfig.canNodeId, boardConfig.currentLimit, boardConfig.velocityLimit, boardConfig.velocityAcc);
        IrqGuard contextGuard;
        uint8_t failure = NONE;
        if (op == CANCEL) {
            if (task == protectionTask && task != 0 && task != lastCancelledTask) {
                lastCancelledTask = task; CancelInternal(CANCELLED_STOP);
                softStopRequest = SoftStopProtocol::STOP; softStopLocked = true;
                softStopPhase = IsActuallyDisabled() ? SoftStopProtocol::DISABLED : SoftStopProtocol::DECELERATING;
            }
        } else if (op == CAL_BEGIN) {
            if (calibrationTask && task == protectionTask) {} // same-task BEGIN never rolls coordinates back
            else if (!task || task == lastCancelledTask || task == lastSavedTask) failure = BAD_TASK;
            else if (!IsActuallyDisabled()) failure = NOT_DISABLED;
            else if (!IsFeedbackValid()) failure = NO_ENCODER;
            else if (configurationFault) failure = INVALID_PROFILE;
            else if (version != coordinateVersion) failure = COORDINATE_INVALID;
            else if (protectionPhase != IDLE && protectionPhase != CAL_FAILED) failure = BUSY;
            else {
                calibrationTask = true; calibrationCoordinateSet = false; protectionTask = task;
                calFailureSnapshotValid = false;
                calDetectionPeak = calDetectionLastTimer = 0;
                calDetectionReset = DR_NONE; calDetectionResetDetail = calDetectionUnmet = 0;
                calDetectionGate = DG_DISABLED;
                searchStart = realPosition; searchMaxTravel = calibrationCurrent = 0;
                contactCurrent = contactSpeed = contactError = 0; contactTriggerTicks = 0;
                localStall = false; softStopLocked = true; softStopPhase = SoftStopProtocol::DISABLED;
                contextRequest = 0; leaseTicks = 0; protectionTakenOver = true;
                SetProtectionPhase(CAL_READY); ResetDetection();
            }
        } else if (op == RENEW) {
            const uint8_t previousVersion = coordinateVersion == 1 ? 255 : static_cast<uint8_t>(coordinateVersion - 1);
            if (value && task == protectionTask && protectionTakenOver &&
                (version == coordinateVersion || (calibrationTask && calibrationCoordinateSet && version == previousVersion))) {
                leaseTicks = 0;
                uint8_t reply[8] = {LEASE_REPLY_FLAG, coordinateVersion, uint8_t(task), uint8_t(task >> 8)};
                memcpy(reply + 4, &value, 4); Send(CONTEXT_REPLY, reply);
            }
            // A delayed keepalive cannot modify a task or overwrite its actual
            // completion snapshot with a BAD_TASK pseudo-failure. Missing or
            // wrong leases naturally reach the existing bounded timeout.
            return true;
        } else if (op == PROTECT_PREPARE) {
            if (!localStall || U32(data + 4) != protectionEvent || version != coordinateVersion || !task) failure = BAD_TASK;
            else if (protectionPhase != LOCAL_STOPPING && protectionPhase != WAIT_GRANT && protectionPhase != RETREATING) failure = BUSY;
            else if (protectionTakenOver && protectionTask != task) failure = BAD_TASK;
            else { protectionTask = task; protectionTakenOver = true; leaseTicks = 0; }
        } else if (op == RETREAT_GO) {
            if (protectionPhase == RETREATING && task == protectionTask && value == lastGrantedEvent && version == coordinateVersion) leaseTicks = 0;
            else if (protectionPhase == HOLD_SUCCESS && task == protectionTask && value == lastGrantedEvent) {} // never repeat displacement
            else if (!localStall || protectionPhase != WAIT_GRANT || value != protectionEvent || task != protectionTask || !protectionTakenOver) failure = BAD_TASK;
            else if (version != coordinateVersion || !profileValid) failure = INVALID_PROFILE;
            else if (!contextRequest) { contextRequestTask = task; contextRequestVersion = version; contextRequestValue = value; contextRequest = op; leaseTicks = 0; }
        } else if (op == CAL_END && !calibrationTask && task != 0 && task == lastSavedTask &&
                   protectionPhase == IDLE && IsActuallyDisabled()) {
            // Terminal ACK tombstone: retransmission confirms completion,
            // without recreating an active task or executing any motion.
        } else if (op == SET_CAPTURE_COORD || op == CAL_RUN || op == SAVE_COORD || op == CAL_END) {
            if (!calibrationTask || task != protectionTask || !task) failure = BAD_TASK;
            else if (version != coordinateVersion &&
                     !(op == SET_CAPTURE_COORD && protectionPhase == CAL_DONE && calibrationCoordinateSet && F32(data + 4) == captureCoordinateLabel) &&
                     !(op == CAL_RUN && (MovingPhase(protectionPhase) || protectionPhase == CAL_DONE))) failure = COORDINATE_INVALID;
            else if (op == SET_CAPTURE_COORD) {
                const float label = F32(data + 4);
                if (protectionPhase == CAL_DONE && calibrationCoordinateSet) {
                    if (label != captureCoordinateLabel) failure = BAD_VALUE;
                } // retry cannot recapture a moved shaft
                else if (protectionPhase != CAL_READY || !IsActuallyDisabled()) failure = BUSY;
                else if (!ProtectionSettings::Finite(label) || fabsf(label) > 10000) failure = BAD_VALUE;
                else if (!contextRequest) { contextRequestTask = task; contextRequestValue = value; contextRequest = op; }
            } else if (op == CAL_RUN) {
                if (boardConfig.canNodeId != 7 && boardConfig.canNodeId != 8) failure = UNSUPPORTED;
                else if (MovingPhase(protectionPhase) || protectionPhase == CAL_DONE) {} // same task cannot seek a second time
                else if (protectionPhase != CAL_READY || !IsActuallyDisabled()) failure = BUSY;
                else if (!ProtectionSettings::Finite(F32(data + 4)) || F32(data + 4) < 0 ||
                         !validCalParams) failure = BAD_VALUE;
                else if (!contextRequest) { contextRequestTask = task; contextRequestValue = value; contextRequest = op; leaseTicks = 0; }
            } else if (op == SAVE_COORD) {
                if (!IsActuallyDisabled()) failure = NOT_DISABLED;
                else if (protectionPhase == SAVED && lastSavedTask == task) {} // flash idempotence
                else if (protectionPhase != CAL_DONE || !calibrationCoordinateSet) failure = BUSY;
                else { saveRequested = true; SetProtectionPhase(SAVE_PENDING); }
            } else {
                if (protectionPhase != SAVED || !IsActuallyDisabled()) failure = BUSY;
                else { calibrationTask = false; protectionTask = 0; softStopLocked = false; SetProtectionPhase(IDLE); }
            }
        } else failure = UNSUPPORTED;
        uint8_t reply[8]{}; BuildProtectionContext(reply);
        if (op == CAL_END && !failure) {
            reply[2] = static_cast<uint8_t>(task); reply[3] = static_cast<uint8_t>(task >> 8);
        }
        if (failure) {
            // A same-task GO/PREPARE arriving after failure reports the real
            // terminal cause, not a BAD_TASK pseudo-failure over that snapshot.
            const bool terminal = task && task == protectionTask &&
                (protectionPhase == HOLD_FAILED || protectionPhase == CAL_FAILED) && protectionReason != NONE;
            reply[1] = terminal ? protectionReason : failure;
            reply[2] = static_cast<uint8_t>(task); reply[3] = static_cast<uint8_t>(task >> 8);
        }
        Send(CONTEXT_REPLY, reply); BuildProtectionEvent(reply); Send(EVENT, reply);
    }
    __set_PRIMASK(irq); return true;
}

void Motor::Controller::ResetStableWindow() {
    softStopStableTicks = 0; softStopStableAnchor = softStopStableMax = realPosition; softStopVelocitySum = 0;
}
bool Motor::Controller::TickStableWindow(bool eligible) {
    if (!eligible || !IsFeedbackValid()) { ResetStableWindow(); return false; }
    softStopStableAnchor = Min(softStopStableAnchor, realPosition);
    softStopStableMax = Max(softStopStableMax, realPosition);
    if (static_cast<int64_t>(softStopStableMax) - softStopStableAnchor > 8) { ResetStableWindow(); return false; }
    softStopVelocitySum += estVelocity;
    if (++softStopStableTicks < 1000) return false;
    const int64_t limit = static_cast<int64_t>(STEPS / 100) * softStopStableTicks;
    if (softStopVelocitySum < -limit || softStopVelocitySum > limit) { ResetStableWindow(); return false; }
    return true;
}
int32_t Motor::Controller::EffectiveCurrentLimit() const {
    int32_t limit = context->config.motionParams.ratedCurrent;
    if (MovingPhase(protectionPhase)) limit = Min(limit, calibrationTask ? calibrationCurrent : retreatCurrent);
    else if (localStall || protectionPhase == PEER_HOLD || protectionPhase == HOLD_FAILED || protectionPhase == HOLD_SUCCESS)
        limit = Min(limit, holdingCurrent);
    return Max(0, limit);
}
PROTECTION_CAN_SIZE void Motor::Controller::BeginLocalProtection(uint8_t reason, bool isLocal) {
    FreezeCalFailure(reason);
    contextRequest = 0; internalStopping = false;
    calibrationTask = false; calibrationCoordinateSet = false;
    localStall = isLocal; protectionTakenOver = false; protectionTask = 0;
    if (isLocal) {
        if (++eventCounter == 0 || eventCounter > 0xFFFF) eventCounter = 1;
        protectionEvent = (static_cast<uint32_t>(bootSession) << 16) | eventCounter;
    }
    stallDirection = motionDirection;
    if (directionTicks < 100) stallDirection = 0;
    const int64_t followingError = static_cast<int64_t>(softPosition) - realPosition;
    if (followingError * stallDirection <= 0) stallDirection = 0;
    if (abs(estVelocity) > detectorSpeed && Sign(estVelocity) != stallDirection) stallDirection = 0;
    if (isLocal && reason == STALL) {
        IrqGuard guard;
        // Freeze before STOP changes the reference/current or resets detection.
        FillStallDiagnostic(triggerSnapshot, DS_TRIGGER, reason);
        triggerSnapshot[D_RESET] = DR_TRIGGERED | (uint32_t(detectionSession) << 16);
        triggerSnapshot[D_LAST_TIMER] = candidateTicks;
        ClearDetectionTimer(DR_TRIGGERED);
        triggerSnapshotValid = true;
    }
    SetProtectionPhase(isLocal ? LOCAL_STOPPING : PEER_STOPPING, reason);
    RequestSoftStop(STOP_PROTECTION, static_cast<uint16_t>(softStopToken + 1));
    ResetDetection(); leaseTicks = 0; masterSilenceTicks = 0;
}
PROTECTION_CAN_SIZE void Motor::Controller::FailProtection(uint8_t reason) {
    FreezeCalFailure(reason);
    contextRequest = 0; internalStopping = false;
    coordinateCapturePending = false;
    SetProtectionPhase(calibrationTask ? CAL_FAILED : HOLD_FAILED, reason);
    softStopLocked = true;
    if (!IsFeedbackValid()) {
        goalDisable = softDisable = true; modeRunning = requestMode = MODE_STOP;
        softStopPhase = SoftStopProtocol::FAULT; context->driver->Sleep();
    } else {
        softStopRequest = SoftStopProtocol::STOP; softStopPhase = SoftStopProtocol::DECELERATING;
    }
    ResetDetection();
}
PROTECTION_CAN_SIZE void Motor::Controller::CancelInternal(uint8_t reason) {
    if (protectionPhase == IDLE) return;
    if ((protectionPhase == HOLD_FAILED || protectionPhase == CAL_FAILED) && protectionReason != NONE)
        reason = protectionReason; // Keep the first fault when STOP/CANCEL follows it.
    FreezeCalFailure(reason);
    contextRequest = 0; internalStopping = false; saveRequested = false;
    coordinateCapturePending = false;
    if (protectionTask) lastCancelledTask = protectionTask;
    protectionTakenOver = false;
    SetProtectionPhase(calibrationTask ? CAL_FAILED : HOLD_FAILED, reason);
    ResetDetection();
}
void Motor::Controller::OnPeerProtection(uint8_t node, uint32_t event, uint8_t phase, uint8_t flags) {
    if (boardConfig.canNodeId == 8 || node < 1 || node > 7 || node == boardConfig.canNodeId ||
        !(flags & LOCAL_STALL) || !event || (phase != LOCAL_STOPPING && phase != WAIT_GRANT && phase != RETREATING && phase != HOLD_FAILED)) return;
    if (peerEvents[node] == event) return;
    peerEvents[node] = event;
    if (protectionPhase == IDLE || (calibrationTask && MovingPhase(protectionPhase))) BeginLocalProtection(STALL, false);
}

void Motor::Controller::TickProtection() {
    detectionUnmet = 0;
    detectionGate = protectionPhase == IDLE ? DG_ELIGIBLE : DG_PROTECTED;
    if (!IsFeedbackValid()) {
        if (detectionBadTotal != UINT32_MAX) ++detectionBadTotal;
        if (invalidFeedbackTicks > detectionBadPeak) detectionBadPeak = invalidFeedbackTicks;
    }
    if (masterSeen && masterSilenceTicks < HZ * 60) ++masterSilenceTicks;
    if (protectionPhase != IDLE) ++phaseTicks;
    if (protectionTakenOver && leaseTicks < HZ * 60) ++leaseTicks;
    if (invalidFeedbackTicks >= 100 && !IsActuallyDisabled() && protectionPhase != HOLD_FAILED && protectionPhase != CAL_FAILED) {
        if (protectionPhase == IDLE) BeginLocalProtection(NO_ENCODER, boardConfig.canNodeId != 8);
        FailProtection(NO_ENCODER); return;
    }
    if (MovingPhase(protectionPhase) && protectionTakenOver && leaseTicks > LEASE_MS * 20) { FailProtection(LEASE_TIMEOUT); return; }
    if (masterSeen && masterSilenceTicks > COMMUNICATION_WATCHDOG_MS * 20 &&
        protectionPhase == IDLE && !IsActuallyDisabled() && motionIntent && boardConfig.canNodeId != 8) {
        BeginLocalProtection(COMMUNICATION_TIMEOUT, false); return;
    }
    if (protectionPhase == LOCAL_STOPPING || protectionPhase == PEER_STOPPING) {
        if (softStopPhase == SoftStopProtocol::FAULT) { FailProtection(NO_ENCODER); return; }
        if (softStopPhase == SoftStopProtocol::HOLDING || softStopPhase == SoftStopProtocol::DISABLED)
            SetProtectionPhase(localStall ? WAIT_GRANT : PEER_HOLD, protectionReason);
        return;
    }
    if (protectionPhase == WAIT_GRANT) {
        if ((!protectionTakenOver && phaseTicks > grantWaitTicks) ||
            (protectionTakenOver && leaseTicks > LEASE_MS * 20)) FailProtection(GRANT_TIMEOUT);
        return;
    }
    // The internal controller owns its continuous contact/reblock timer.
    // Ordinary detection must not clear that timer every protected tick.
    if (protectionPhase != IDLE) return;
    if (boardConfig.canNodeId == 8) detectionGate = DG_EXCLUDED;
    else if (!context->config.ctrlParams.stallProtectSwitch) detectionGate = DG_SWITCH_OFF;
    else if (IsSoftStopLocked()) detectionGate = DG_STOP_LOCKED;
    else if (goalDisable || softDisable) detectionGate = DG_DISABLED;
    else if (!IsFeedbackValid()) detectionGate = DG_BAD_FEEDBACK;
    if (detectionGate != DG_ELIGIBLE) { ClearDetectionTimer(DR_GATE, detectionGate); return; }
    const bool positionMode = modeRunning == MODE_COMMAND_POSITION || modeRunning == MODE_COMMAND_Trajectory ||
                              modeRunning == MODE_PWM_POSITION || modeRunning == MODE_STEP_DIR;
    if (!positionMode || !motionIntent) {
        detectionGate = positionMode ? DG_NO_MOTION : DG_MODE;
        ClearDetectionTimer(DR_GATE, detectionGate); return;
    }
    if (IsOrdinaryPositionArrival(false)) {
        // Acceptable endpoint correction is not a blocked move. Keep intent
        // until the existing encoder stability window confirms completion.
        detectionGate = DG_ARRIVAL; ClearDetectionTimer(DR_GATE, detectionGate); return;
    }
    const int32_t directionReference = softVelocity;
    if (abs(directionReference) >= STEPS / 1000) {
        const int8_t direction = Sign(directionReference);
        if (motionDirection != direction) { motionDirection = direction; directionTicks = 0; ClearDetectionTimer(DR_DIRECTION); }
        if (directionTicks < HZ) ++directionTicks;
    }
    if (startupTicks) { --startupTicks; detectionGate = DG_STARTUP; ClearDetectionTimer(DR_STARTUP); return; }
    if (currentFilterTicks == 0 || velocityFilterTicks == 0) {
        detectionGate = DG_FILTER; ClearDetectionTimer(DR_GATE, detectionGate); return;
    }
    const int32_t boundedSpeed = Min(abs(estVelocity), INT32_MAX / 1024);
    filteredCurrentQ = Filter(filteredCurrentQ, abs(focCurrent), currentFilterTicks);
    filteredVelocityQ = Filter(filteredVelocityQ, boundedSpeed, velocityFilterTicks);
    const int64_t error = static_cast<int64_t>(softPosition) - realPosition;
    detectionUnmet = (filteredCurrentQ / 1024 < detectorCurrent ? 1 : 0) |
        (filteredVelocityQ / 1024 > detectorSpeed ? 2 : 0) |
        (error < detectorError && error > -detectorError ? 4 : 0);
    if (detectionUnmet) ClearDetectionTimer(DR_CONDITIONS, detectionUnmet);
    else {
        ++candidateTicks;
        if (candidateTicks > detectionPeak) detectionPeak = candidateTicks;
        if (candidateTicks >= detectorTriggerTicks) {
            BeginLocalProtection(STALL, true);
        }
    }
    // Command-position intent is retired by actual completion, not by the
    // protection retreat tolerance. Preserve non-command-position behavior.
    if (modeRunning != MODE_COMMAND_POSITION && softVelocity == 0 &&
        abs(goalPosition - realPosition) <= positionTolerance) motionIntent = false;
}

PROTECTION_CAN_SIZE void Motor::Controller::SetCaptureCoordinate(float motorR) {
    const int64_t offset = static_cast<int64_t>(realPosition) - Steps(motorR);
    if (offset < INT32_MIN || offset > INT32_MAX) { FailProtection(BAD_VALUE); return; }
    context->config.motionParams.encoderHomeOffset = static_cast<int32_t>(offset);
    if (++coordinateVersion == 0) ++coordinateVersion;
    assumedBoot = false; coordinateDirty = true; profileValid = false;
    calibrationCoordinateSet = true;
    captureCoordinateLabel = motorR;
    configStaging = false; contextRequest = 0;
    // The actual unwrap and last raw encoder sample remain untouched.
    if (IsActuallyDisabled()) {
        goalPosition = softPosition = softStopPosition = realPosition;
        goalVelocity = softVelocity = 0; goalCurrent = softCurrent = 0; softNewCurve = false;
    } else CaptureStopPosition();
    ResetDetection(); ResetStableWindow();
}
PROTECTION_CAN_SIZE void Motor::Controller::BeginInternalMove(int32_t target, int32_t velocity, int32_t acceleration,
                                         int32_t current, uint8_t phase) {
    internalConfig = context->config.motionParams;
    internalConfig.ratedVelocity = Min(velocity, boardConfig.velocityLimit);
    internalConfig.ratedVelocityAcc = Min(acceleration, boardConfig.velocityAcc);
    internalConfig.ratedCurrent = Min(current, boardConfig.currentLimit);
    if (internalConfig.ratedVelocity <= 0 || internalConfig.ratedVelocityAcc <= 0 || internalConfig.ratedCurrent <= 0) {
        FailProtection(BAD_VALUE); return;
    }
    internalPlanner.positionTracker.Init();
    internalPlanner.positionTracker.NewTask(realPosition, estVelocity);
    internalStart = realPosition; internalTarget = target;
    goalDisable = softDisable = false; goalBrake = softBrake = false;
    softStopLocked = true; softStopPhase = SoftStopProtocol::DECELERATING;
    modeRunning = requestMode = MODE_COMMAND_POSITION; softNewCurve = false;
    softPosition = realPosition; softVelocity = estVelocity; goalPosition = target;
    internalStopping = false; candidateTicks = 0; startupTicks = detectorStartupTicks;
    referenceFinishedTicks = 0;
    if (calibrationTask) {
        calDetectionPeak = calDetectionLastTimer = 0;
        calDetectionReset = DR_NONE; calDetectionResetDetail = calDetectionUnmet = 0;
        calDetectionGate = DG_STARTUP;
    }
    ResetStableWindow(); SetProtectionPhase(phase);
}
PROTECTION_CAN_SIZE void Motor::Controller::BeginSearch(bool retouch) {
    contactCurrent = static_cast<int32_t>(calibrationCurrent * protectionSettings.Get(CAL_CONTACT_RATIO));
    contactSpeed = Steps(protectionSettings.Get(CAL_SPEED_RPS)); contactError = Steps(protectionSettings.Get(CAL_ERROR_R));
    contactTriggerTicks = Ticks(protectionSettings.Get(CAL_TRIGGER_MS));
    contactStartupTicks = Ticks(protectionSettings.Get(CAL_STARTUP_MS));
    searchMaxTravel = Steps(protectionSettings.Get(CAL_MAX_TRAVEL_R));
    contactWindow = Steps(protectionSettings.Get(CAL_CONTACT_WINDOW_R));
    repeatTolerance = Steps(protectionSettings.Get(CAL_REPEAT_TOL_R));
    searchTimeoutTicks = Ticks(protectionSettings.Get(retouch ? CAL_RETOUCH_TIMEOUT_MS : CAL_SEEK_TIMEOUT_MS));
    // J7 is inverted by the master: positive motor travel seeks rail -250mm.
    searchDirection = 1;
    searchStart = realPosition;
    const int64_t target = static_cast<int64_t>(realPosition) + static_cast<int64_t>(searchDirection) *
                           (retouch ? Steps(protectionSettings.Get(CAL_BACKOFF_R)) + 2 * contactError : searchMaxTravel);
    if (target < INT32_MIN || target > INT32_MAX) { FailProtection(SEARCH_TRAVEL); return; }
    BeginInternalMove(static_cast<int32_t>(target), Steps(protectionSettings.Get(retouch ? CAL_RETOUCH_RPS : CAL_SEEK_RPS)),
                      Steps(protectionSettings.Get(CAL_ACCEL)), calibrationCurrent, retouch ? CAL_RETOUCH : CAL_SEEK);
    startupTicks = contactStartupTicks; filteredCurrentQ = filteredVelocityQ = 0;
}

bool Motor::Controller::TickInternalProtection() {
    const uint8_t operation = contextRequest;
    if (operation) {
        const uint16_t task = contextRequestTask; const uint8_t version = contextRequestVersion;
        const uint32_t value = contextRequestValue; contextRequest = 0;
        if (task != protectionTask) return false;
        if (operation == SET_CAPTURE_COORD) {
            if (!coordinateCapturePending) {
                memcpy(&pendingCaptureLabel, &value, 4);
                coordinateCapturePending = true; capturePendingTicks = 0; ResetStableWindow();
            }
        } else if (operation == CAL_RUN) {
            float cap; memcpy(&cap, &value, 4);
            calibrationCurrent = static_cast<int32_t>(protectionSettings.Get(CAL_CURRENT_A) * 1000);
            if (cap > 0) calibrationCurrent = Min(calibrationCurrent, static_cast<int32_t>(cap * 1000));
            if (boardConfig.canNodeId == 7 && !profileValid) {
                // The user has explicitly confirmed the rail's +/-250mm
                // range and 5mm/revolution. This context may bind that known
                // range so abnormal calibration blocking can still retreat.
                protectionSettings.value[MIN_POSITION_R] = -50;
                protectionSettings.value[MAX_POSITION_R] = 50;
                protectionSettings.value[DIRECTION_MASK] = 3;
                profileLow = -50 * STEPS; profileHigh = 50 * STEPS;
                profileDirections = 3; profileValid = true;
            }
            BeginSearch(false);
        } else if (operation == RETREAT_GO) {
            if (version != coordinateVersion || !profileValid) { FailProtection(INVALID_PROFILE); return false; }
            if (!IsFeedbackValid()) { FailProtection(NO_ENCODER); return false; }
            if (!stallDirection) { FailProtection(NO_DIRECTION); return false; }
            const int32_t direction = -stallDirection;
            if (!(profileDirections & (direction > 0 ? 2 : 1))) { FailProtection(NO_DIRECTION); return false; }
            const int64_t low = static_cast<int64_t>(context->config.motionParams.encoderHomeOffset) + profileLow;
            const int64_t high = static_cast<int64_t>(context->config.motionParams.encoderHomeOffset) + profileHigh;
            int64_t target = static_cast<int64_t>(realPosition) + static_cast<int64_t>(direction) * retreatDistance;
            const int64_t requested = target;
            if (target < low) target = low; if (target > high) target = high;
            const int64_t displacement = (target - realPosition) * direction;
            if (target < INT32_MIN || target > INT32_MAX || displacement <= positionTolerance || displacement > retreatDistance) {
                FailProtection(NO_SPACE); return false;
            }
            distanceClipped = target != requested; lastGrantedEvent = value; leaseTicks = 0;
            BeginInternalMove(static_cast<int32_t>(target), retreatVelocity, retreatAcceleration, retreatCurrent, RETREATING);
        }
    }
    if (coordinateCapturePending) {
        if (!calibrationTask || protectionPhase != CAL_READY || !IsActuallyDisabled()) {
            coordinateCapturePending = false; FailProtection(NOT_DISABLED); return false;
        }
        if (++capturePendingTicks > 2000 * 20) {
            coordinateCapturePending = false; FailProtection(IsFeedbackValid() ? CAPTURE_UNSTABLE : NO_ENCODER); return false;
        }
        if (TickStableWindow(true)) {
            coordinateCapturePending = false; SetCaptureCoordinate(pendingCaptureLabel);
            if (protectionPhase != CAL_FAILED) SetProtectionPhase(CAL_DONE);
        }
        // TickSoftStop preserves DISABLED; setting an origin never energises
        // a joint, even when its encoder proves it is being moved by hand.
        return false;
    }
    if (!MovingPhase(protectionPhase)) return false;
    // Every moving subphase, including contact deceleration and settling,
    // shares the phase timeout. Waiting for physical stability cannot bypass it.
    const bool search = protectionPhase == CAL_SEEK || protectionPhase == CAL_RETOUCH;
    const uint32_t timeout = search ? searchTimeoutTicks : calibrationTask ?
        Ticks(protectionSettings.Get(protectionPhase == CAL_BACKOFF ? CAL_RETOUCH_TIMEOUT_MS : CAL_RETURN_TIMEOUT_MS)) : retreatTimeoutTicks;
    if (phaseTicks > timeout) {
        if (protectionPhase == CAL_RETURN && boardConfig.canNodeId == 7) {
            motionDirection = Sign(internalTarget - internalStart); directionTicks = 100;
            BeginLocalProtection(RETURN_TIMEOUT, true);
        } else FailProtection(search ? SEARCH_TIMEOUT : calibrationTask ? RETURN_TIMEOUT : RETREAT_TIMEOUT);
        return false;
    }
    if (!IsFeedbackValid()) {
        ResetStableWindow();
        if (calibrationTask) { calDetectionGate = DG_BAD_FEEDBACK; ClearCalDetectionTimer(DR_GATE, DG_BAD_FEEDBACK); }
        else candidateTicks = 0;
        return true; // grace period handled by TickProtection; no stale completion
    }
    if (softStopRequest == SoftStopProtocol::DISABLE || softStopRequest == SoftStopProtocol::STOP) return false;
    const int64_t actualTravel = int64_t(realPosition) - searchStart;
    if (search && (actualTravel > searchMaxTravel || actualTravel < -int64_t(searchMaxTravel))) {
        FailProtection(SEARCH_TRAVEL); return false;
    }
    if (internalStopping) {
        internalPlanner.velocityTracker.CalcSoftGoal(0); softVelocity = internalPlanner.velocityTracker.goVelocity;
        if (!TickStableWindow(softVelocity == 0)) return true;
        CaptureStopPosition(); softStopPhase = SoftStopProtocol::HOLDING; internalStopping = false;
        if (protectionPhase == CAL_SEEK) {
            firstContact = realPosition;
            const int64_t target = static_cast<int64_t>(realPosition) - static_cast<int64_t>(searchDirection) * Steps(protectionSettings.Get(CAL_BACKOFF_R));
            if (target < INT32_MIN || target > INT32_MAX) { FailProtection(NO_SPACE); return false; }
            BeginInternalMove(static_cast<int32_t>(target), Steps(protectionSettings.Get(CAL_RETURN_RPS)), Steps(protectionSettings.Get(CAL_ACCEL)), calibrationCurrent, CAL_BACKOFF);
        } else if (protectionPhase == CAL_RETOUCH) {
            if (abs(realPosition - firstContact) > repeatTolerance) { FailProtection(CONTACT_UNSTABLE); return false; }
            SetCaptureCoordinate(boardConfig.canNodeId == 7 ? 50.0f : 0.0f);
            if (boardConfig.canNodeId == 7) {
                protectionSettings.value[MIN_POSITION_R] = -50;
                protectionSettings.value[MAX_POSITION_R] = 50;
                protectionSettings.value[DIRECTION_MASK] = 3;
                profileLow = -50 * STEPS; profileHigh = 50 * STEPS;
                profileDirections = 3; profileValid = true;
            }
            const int64_t target = static_cast<int64_t>(context->config.motionParams.encoderHomeOffset) -
                                   (boardConfig.canNodeId == 8 ? Steps(100.0f * 16 / 360) : 0);
            if (target < INT32_MIN || target > INT32_MAX) { FailProtection(BAD_VALUE); return false; }
            BeginInternalMove(static_cast<int32_t>(target), Steps(protectionSettings.Get(CAL_RETURN_RPS)), Steps(protectionSettings.Get(CAL_ACCEL)), calibrationCurrent, CAL_RETURN);
        }
        return true;
    }
    internalPlanner.positionTracker.CalcSoftGoal(internalTarget);
    softPosition = internalPlanner.positionTracker.go_location; softVelocity = internalPlanner.positionTracker.go_velocity;
    if (search && (int64_t(softPosition) - internalTarget) * searchDirection >= 0) {
        // Only CAL's bounded reference is clamped. Reset its integrators as
        // well as position/velocity so a discrete overshoot cannot escape the
        // contact-proof window. Actual travel and contact remain independent.
        if (softPosition != internalTarget || softVelocity != 0)
            internalPlanner.positionTracker.NewTask(internalTarget, 0);
        softPosition = internalTarget; softVelocity = 0;
    }
    if (startupTicks) {
        --startupTicks;
        if (calibrationTask) { calDetectionGate = DG_STARTUP; calDetectionUnmet = 0; ClearCalDetectionTimer(DR_STARTUP); }
        else candidateTicks = 0;
    }
    else {
        filteredCurrentQ = Filter(filteredCurrentQ, abs(focCurrent), currentFilterTicks);
        const int32_t speed = Min(abs(estVelocity), INT32_MAX / 1024);
        filteredVelocityQ = Filter(filteredVelocityQ, speed, velocityFilterTicks);
        const int64_t error = static_cast<int64_t>(softPosition) - realPosition;
        const int32_t currentLimit = calibrationTask ? contactCurrent : Min(detectorCurrent, retreatCurrent * 8 / 10);
        const int32_t speedLimit = calibrationTask ? contactSpeed : detectorSpeed;
        const int32_t errorLimit = calibrationTask ? contactError : detectorError;
        const bool blocked = filteredCurrentQ / 1024 >= currentLimit && filteredVelocityQ / 1024 <= speedLimit &&
                             (error >= errorLimit || error <= -errorLimit);
        if (calibrationTask) {
            calDetectionGate = DG_ELIGIBLE;
            calDetectionUnmet = (filteredCurrentQ / 1024 < currentLimit ? 1 : 0) |
                (filteredVelocityQ / 1024 > speedLimit ? 2 : 0) |
                (error < errorLimit && error > -errorLimit ? 4 : 0);
        }
        if (!blocked) {
            if (calibrationTask) ClearCalDetectionTimer(DR_CONDITIONS, calDetectionUnmet);
            else candidateTicks = 0;
        } else {
            ++candidateTicks;
            if (calibrationTask && candidateTicks > calDetectionPeak) calDetectionPeak = candidateTicks;
        }
        if (blocked && candidateTicks >= (search ? contactTriggerTicks : reblockTicks)) {
            if (search) {
                // An assumed single-turn boot coordinate cannot prove end
                // location. Only a measured coordinate uses the prior window.
                if (!assumedBoot && protectionPhase == CAL_SEEK) {
                    const int32_t expected = context->config.motionParams.encoderHomeOffset + (boardConfig.canNodeId == 7 ? 50 * STEPS : 0);
                    if (abs(realPosition - expected) > contactWindow) {
                        if (boardConfig.canNodeId == 7) {
                            motionDirection = static_cast<int8_t>(searchDirection); directionTicks = 100;
                            BeginLocalProtection(EARLY_CONTACT, true);
                        } else FailProtection(EARLY_CONTACT);
                        return false;
                    }
                }
                internalPlanner.velocityTracker.SetVelocityAcc(internalConfig.ratedVelocityAcc);
                internalPlanner.velocityTracker.NewTask(estVelocity);
                modeRunning = requestMode = MODE_COMMAND_VELOCITY;
                ClearIntegral(); config->pid.vError = config->pid.vErrorLast = 0;
                config->pid.outputKi = focCurrent * 1024;
                internalStopping = true; ResetStableWindow(); ClearCalDetectionTimer(DR_TRIGGERED);
            } else if (calibrationTask && boardConfig.canNodeId == 7) {
                motionDirection = Sign(softVelocity);
                if (!motionDirection) motionDirection = Sign(internalTarget - internalStart);
                directionTicks = 100; BeginLocalProtection(REBLOCKED, true);
            } else FailProtection(REBLOCKED);
            return true;
        }
    }
    if (search) {
        if (softVelocity == 0 && softPosition == internalTarget) {
            // Especially during slow retouch, the reference reaches its
            // bounded overtravel target before actual settling + continuous
            // contact proof can finish. Give settling its own one-second
            // allowance, then the configured continuous proof time. Keep the
            // target fixed and the phase timeout active; interrupted candidates
            // never renew this deadline or count as encoder-confirmed contact.
            const uint32_t proofWindow = HZ + contactTriggerTicks;
            if (++referenceFinishedTicks > proofWindow) { FailProtection(CONTACT_NOT_CONFIRMED); return false; }
        } else referenceFinishedTicks = 0;
        return true;
    }
    const int32_t tolerance = calibrationTask ? Steps(protectionSettings.Get(CAL_POSITION_TOL_R)) : positionTolerance;
    const bool actualArrival = softVelocity == 0 && softPosition == internalTarget && abs(realPosition - internalTarget) <= tolerance;
    if (!TickStableWindow(actualArrival)) return true;
    if (abs(realPosition - internalStart) <= tolerance) { FailProtection(NO_SPACE); return false; }
    CaptureStopPosition(); softStopPhase = SoftStopProtocol::HOLDING;
    if (protectionPhase == CAL_BACKOFF) BeginSearch(true);
    else SetProtectionPhase(calibrationTask ? CAL_DONE : HOLD_SUCCESS);
    return true;
}

PROTECTION_CAN_SIZE void Motor::Controller::PollDefaultsRestore() {
    if (!defaultsRestoreRequested) return;
    // Only the final 1KiB user page is rewritten; the encoder table is separate.
    bool erased = IsActuallyDisabled() && MotorDefaultsRestoreAllowed();
    if (erased) {
        for (uint32_t i = 0; i <= E2END; ++i) eeprom_buffered_write_byte(i, 0xFF);
        eeprom_buffer_flush();
        for (uint32_t i = 0; i <= E2END; ++i)
            if (eeprom_flash_read_byte(i) != 0xFF) { erased = false; break; }
    }
    BKP->DR5 = erased ? RESTORE_ERASED : RESTORE_FAILED;
    if (erased) HAL_NVIC_SystemReset();
    defaultsRestoreRequested = false;
    ReplyConfiguration(RESTORE_DEFAULTS, WRITE_FAILED, uint16_t(BKP->DR4));
}
PROTECTION_CAN_SIZE void Motor::Controller::PollProtectionMain() {
    if (saveRequested || parameterSaveRequested) {
        const uint32_t irq = __get_PRIMASK(); __disable_irq();
        const bool coordinateSave = saveRequested;
        const bool safe = IsActuallyDisabled() && (!coordinateSave || (calibrationTask && protectionPhase == SAVE_PENDING));
        const uint16_t task = protectionTask, transaction = configTransaction;
        const uint8_t version = coordinateVersion;
        const int32_t offset = context->config.motionParams.encoderHomeOffset;
        const ProtectionSettings settings = protectionSettings;
        saveRequested = false; parameterSaveRequested = false;
        if (!coordinateSave) parameterSavingInProgress = true;
        else coordinateSavingInProgress = true;
        __set_PRIMASK(irq);
        const bool saved = safe && SaveProtectionSettings(settings, task, version, offset, !assumedBoot);
        const uint32_t resultIrq = __get_PRIMASK(); __disable_irq();
        if (coordinateSave && task == protectionTask && calibrationTask) {
            coordinateSavingInProgress = false;
            if (saved) { lastSavedTask = task; coordinateDirty = false; SetProtectionPhase(SAVED); }
            else { FreezeCalFailure(SAVE_FAILED); SetProtectionPhase(CAL_FAILED, SAVE_FAILED); }
        } else if (!coordinateSave) {
            parameterSavingInProgress = false;
            lastParameterSavedTransaction = transaction;
            lastParameterSaveStatus = saved ? OK : WRITE_FAILED;
            ReplyConfiguration(SAVE, saved ? OK : WRITE_FAILED, transaction);
        }
        else coordinateSavingInProgress = false;
        __set_PRIMASK(resultIrq);
    }
    const uint32_t now = HAL_GetTick();
    if ((!eventAcknowledged && protectionPhase != IDLE && now - eventSendMillis >= 20) || now - eventSendMillis >= 250) {
        uint8_t event[8]{}, contextData[8]{};
        const uint32_t irq = __get_PRIMASK(); __disable_irq();
        BuildProtectionEvent(event); BuildProtectionContext(contextData);
        __set_PRIMASK(irq);
        Send(EVENT, event); Send(CONTEXT_REPLY, contextData); eventSendMillis = now;
    }
    PollStallDiagnostic();
}
