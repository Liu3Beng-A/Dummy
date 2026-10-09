#ifndef CTRL_STEP_FW_MOTOR_H
#define CTRL_STEP_FW_MOTOR_H

#include "Motor/motion_planner.h"
#include "Sensor/Encoder/encoder_base.h"
#include "Driver/driver_base.h"
#include "../../../stop_protocol.h"
#include "../../../motion_completion_protocol.h"
#include "protection_config.h"

class Motor
{
public:
    Motor() :
        controller(&controllerInstance)
    {
        /****************** Default Configs *******************/
        config.motionParams.encoderHomeOffset = 0;
        config.motionParams.caliCurrent = 2000;             // (mA)
        config.motionParams.ratedCurrent = 1000;            // (mA)
        config.motionParams.ratedCurrentAcc = 2 * 1000;     // (mA/s)
        config.motionParams.ratedVelocity = 30 * MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
        config.motionParams.ratedVelocityAcc = 1000 * MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;

        config.ctrlParams.stallProtectSwitch = false;
        config.ctrlParams.pid =
            Controller::PID_t{
                .kp = 5,
                .ki = 30,
                .kd = 0
            };
        config.ctrlParams.dce =
            Controller::DCE_t{
                .kp = 200,
                .kv = 80,
                .ki = 300,
                .kd = 250
            };

        /*****************************************************/


        motionPlanner.AttachConfig(&config.motionParams);
        controller->AttachConfig(&config.ctrlParams);
    }


    static constexpr int32_t MOTOR_ONE_CIRCLE_HARD_STEPS = 200; // for 1.8° step-motors
    static constexpr int32_t SOFT_DIVIDE_NUM = 256;
    static constexpr int32_t MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS = MOTOR_ONE_CIRCLE_HARD_STEPS * SOFT_DIVIDE_NUM;

    typedef enum
    {
        MODE_STOP,
        MODE_COMMAND_POSITION,
        MODE_COMMAND_VELOCITY,
        MODE_COMMAND_CURRENT,
        MODE_COMMAND_Trajectory,
        MODE_PWM_POSITION,
        MODE_PWM_VELOCITY,
        MODE_PWM_CURRENT,
        MODE_STEP_DIR,
    } Mode_t;

    typedef enum
    {
        STATE_STOP,
        STATE_FINISH,
        STATE_RUNNING,
        STATE_OVERLOAD,
        STATE_STALL,
        STATE_NO_CALIB
    } State_t;


    class Controller
    {
    public:
        friend Motor;

        typedef struct
        {
            bool kpValid, kiValid, kdValid;
            int32_t kp, ki, kd;
            int32_t vError, vErrorLast;
            int32_t outputKp, outputKi, outputKd;
            int32_t integralRound;
            int32_t integralRemainder;
            int32_t output;
        } PID_t;

        typedef struct
        {
            int32_t kp, kv, ki, kd;
            int32_t pError, vError;
            int32_t outputKp, outputKi, outputKd;
            int32_t integralRound;
            int32_t integralRemainder;
            int32_t output;
        } DCE_t;

        typedef struct
        {
            PID_t pid;
            DCE_t dce;

            bool stallProtectSwitch;
        } Config_t;


        explicit Controller(Motor* _context)
        {
            context = _context;

            requestMode = MODE_STOP;
            modeRunning = MODE_STOP;
        }


        Config_t* config = nullptr;
        Mode_t requestMode;
        Mode_t modeRunning;
        State_t state = STATE_STOP;
        bool isStalled = false;


        void Init();
        void SetCtrlMode(Mode_t _mode);
        void SetCurrentSetPoint(int32_t _cur);
        void SetVelocitySetPoint(int32_t _vel);
        void SetPositionSetPoint(int32_t _pos);
        bool SetPositionSetPointWithTime(int32_t _pos, float _time);
        float GetPosition(bool _isLap = false);
        float GetVelocity();
        float GetFocCurrent();
        uint8_t GetPositionCompletionStatus(float expectedMotorR) const;
        void AddTrajectorySetPoint(int32_t _pos, int32_t _vel);
        void SetDisable(bool _disable);
        void SetBrake(bool _brake);
        void ApplyPosAsHomeOffset();
        void ClearStallFlag();
        void RequestSoftStop(uint8_t action, uint16_t token);
        void DisableForStop();
        bool IsSoftStopLocked() const { return softStopLocked || defaultsRestoreRequested; }
        uint8_t GetSoftStopPhase() const { return softStopPhase; }
        uint16_t GetSoftStopToken() const { return softStopToken; }
        float GetSoftStopPosition();
        bool IsFeedbackValid() const;
        bool IsActuallyDisabled() const { return goalDisable && softDisable && modeRunning == MODE_STOP; }
        bool IsCalibrationContext() const { return calibrationTask; }
        bool ProcessProtectionCan(uint8_t command, const uint8_t* data, uint32_t length);
        void OnMasterCommunication();
        void OnPeerProtection(uint8_t node, uint32_t event, uint8_t phase, uint8_t flags);
        void PollProtectionMain();
        void InitProtection();
        void PollDefaultsRestore();
        void OnMotionLimitChange();
        void BuildProtectionEvent(uint8_t* data) const;
        void BuildProtectionContext(uint8_t* data, uint16_t query = 0, bool probe = false) const;
        uint8_t GetProtectionPhase() const { return protectionPhase; }
        uint8_t GetCoordinateVersion() const { return coordinateVersion; }
        ProtectionSettings protectionSettings;


    private:
        Motor* context;
        void ResetPositionCompletion(bool newGoal = false);
        bool TickPositionCompletion();
        bool IsOrdinaryPositionArrival(bool referenceFinished = true) const;
        uint16_t positionStableTicks = 0;
        int32_t positionStableMin = 0, positionStableMax = 0;
        int64_t positionVelocitySum = 0;
        int32_t positionCompletionTarget = 0;
        bool positionSettled = false;
        bool positionCompleted = false; // Result of this local goal, separate from live settlement.
        volatile bool softStopLocked = false;
        volatile uint8_t softStopRequest = 0;
        volatile uint8_t softStopPhase = SoftStopProtocol::NORMAL;
        volatile uint16_t softStopToken = 0;
        bool softStopTokenValid = false;
        uint8_t softStopTokenAction = 0; // Idempotence key is action + token.
        bool softStopKeepHold = false;
        int32_t softStopPosition = 0;
        int32_t softStopStableAnchor = 0;
        int32_t softStopStableMax = 0;
        int64_t softStopVelocitySum = 0;
        uint32_t softStopStableTicks = 0;
        bool feedbackInitialized = false;
        uint32_t invalidFeedbackTicks = 0;
        uint32_t transportMissingTicks = 0;
        uint32_t masterSilenceTicks = 0;
        bool masterSeen = false;
        volatile uint8_t protectionPhase = ProtectionProtocol::IDLE;
        volatile uint8_t protectionReason = ProtectionProtocol::NONE;
        volatile uint32_t protectionEvent = 0;
        uint32_t eventCounter = 0;
        uint16_t bootSession = 0;
        uint32_t peerEvents[8]{};
        volatile uint16_t protectionTask = 0;
        uint16_t lastCancelledTask = 0;
        uint16_t lastSavedTask = 0;
        uint32_t lastGrantedEvent = 0;
        volatile uint8_t coordinateVersion = 1;
        bool assumedBoot = true, coordinateDirty = false, profileValid = false;
        bool configurationFault = false;
        bool distanceClipped = false, localStall = false;
        bool calibrationTask = false, calibrationCoordinateSet = false;
        volatile bool defaultsRestoreRequested = false;
        float captureCoordinateLabel = 0;
        bool coordinateCapturePending = false;
        float pendingCaptureLabel = 0;
        uint32_t capturePendingTicks = 0;
        bool protectionTakenOver = false;
        uint32_t phaseTicks = 0, leaseTicks = 0;
        uint32_t candidateTicks = 0, startupTicks = 0, directionTicks = 0;
        int8_t motionDirection = 0, stallDirection = 0;
        bool motionIntent = false;
        int32_t filteredCurrentQ = 0, filteredVelocityQ = 0;
        int32_t detectorCurrent = 2000, detectorSpeed = 1024, detectorError = 1024;
        uint32_t detectorTriggerTicks = 3000, detectorStartupTicks = 2000;
        uint32_t currentFilterTicks = 200, velocityFilterTicks = 400;
        uint32_t detectionPeak = 0, detectionLastTimer = 0, detectionBadTotal = 0, detectionBadPeak = 0;
        uint16_t detectionSession = 0;
        uint8_t detectionGate = ProtectionProtocol::DG_DISABLED, detectionUnmet = 0;
        uint8_t detectionReset = ProtectionProtocol::DR_NONE, detectionResetDetail = 0;
        uint32_t diagnosticSnapshot[ProtectionProtocol::DIAG_FIELD_COUNT]{};
        uint32_t triggerSnapshot[ProtectionProtocol::DIAG_FIELD_COUNT]{};
        bool triggerSnapshotValid = false;
        uint32_t calFailureSnapshot[ProtectionProtocol::DIAG_FIELD_COUNT]{};
        bool calFailureSnapshotValid = false;
        uint32_t calDetectionPeak = 0, calDetectionLastTimer = 0;
        uint8_t calDetectionReset = ProtectionProtocol::DR_NONE, calDetectionResetDetail = 0;
        uint8_t calDetectionUnmet = 0, calDetectionGate = ProtectionProtocol::DG_DISABLED;
        uint16_t diagnosticRequest = 0;
        uint8_t diagnosticSelector = ProtectionProtocol::DS_LIVE;
        uint8_t diagnosticCursor = ProtectionProtocol::DIAG_FIELD_COUNT;
        uint32_t diagnosticSendMs = 0;
        uint32_t reblockTicks = 2000, grantWaitTicks = 10000;
        int32_t retreatDistance = 0, retreatVelocity = 0, retreatAcceleration = 0;
        int32_t retreatCurrent = 0, holdingCurrent = 0, positionTolerance = 51;
        uint32_t retreatTimeoutTicks = 100000;
        int32_t profileLow = 0, profileHigh = 0;
        uint8_t profileDirections = 3;
        MotionPlanner internalPlanner;
        MotionPlanner::Config_t internalConfig{};
        int32_t internalTarget = 0, internalStart = 0;
        int32_t firstContact = 0, searchStart = 0, referenceTravel = 0;
        uint32_t referenceFinishedTicks = 0;
        int32_t searchDirection = 0, contactCurrent = 0, contactSpeed = 0, contactError = 0;
        int32_t searchMaxTravel = 0, contactWindow = 0, repeatTolerance = 0;
        uint32_t contactTriggerTicks = 0, contactStartupTicks = 0, searchTimeoutTicks = 0;
        int32_t calibrationCurrent = 0;
        bool internalStopping = false;
        bool saveRequested = false, parameterSaveRequested = false;
        bool parameterSavingInProgress = false;
        bool coordinateSavingInProgress = false;
        uint16_t lastParameterSavedTransaction = 0;
        uint8_t lastParameterSaveStatus = ProtectionProtocol::OK;
        bool eventAcknowledged = false;
        uint32_t eventSendMillis = 0;
        ProtectionSettings stagedSettings;
        uint16_t configTransaction = 0;
        uint8_t lastConfigOperation = 0;
        bool configStaging = false, stagedProfileChanged = false;
        volatile uint8_t contextRequest = 0;
        volatile uint16_t contextRequestTask = 0;
        volatile uint8_t contextRequestVersion = 0;
        volatile uint32_t contextRequestValue = 0;
        void TickProtection();
        bool TickInternalProtection();
        void ResetDetection(bool newMotion = false);
        void ClearDetectionTimer(uint8_t reason, uint8_t detail = 0);
        void FillStallDiagnostic(uint32_t* snapshot, uint8_t source, uint8_t reason = ProtectionProtocol::NONE) const;
        void FillCalDiagnostic(uint32_t* snapshot, uint8_t source, uint8_t reason = ProtectionProtocol::NONE) const;
        void FreezeCalFailure(uint8_t reason);
        void ClearCalDetectionTimer(uint8_t reason, uint8_t detail = 0);
        void CaptureStallDiagnostic(uint16_t request, uint8_t selector);
        void PollStallDiagnostic();
        void UpdateProtectionLimits();
        void SetProtectionPhase(uint8_t phase, uint8_t reason = ProtectionProtocol::NONE);
        void BeginLocalProtection(uint8_t reason, bool isLocal);
        void FailProtection(uint8_t reason);
        void CancelInternal(uint8_t reason);
        void BeginInternalMove(int32_t target, int32_t velocity, int32_t acceleration,
                               int32_t current, uint8_t phase);
        void BeginSearch(bool retouch);
        bool TickStableWindow(bool eligible);
        void ResetStableWindow();
        void SetCaptureCoordinate(float motorR);
        int32_t EffectiveCurrentLimit() const;
        void ReplyParameter(uint8_t id, uint8_t status, uint16_t transaction, float value);
        void ReplyConfiguration(uint8_t operation, uint8_t status, uint16_t transaction);
        bool TickSoftStop();
        void CaptureStopPosition();
        int32_t realLapPosition{};
        int32_t realLapPositionLast{};
        int32_t realPosition{};
        int32_t realPositionLast{};
        int32_t estVelocity{};
        int32_t estVelocityIntegral{};
        int32_t estLeadPosition{};
        int32_t estPosition{};
        int32_t estError{};
        int32_t focCurrent{};
        int32_t goalPosition{};
        int32_t goalVelocity{};
        int32_t goalCurrent{};
        bool goalDisable{};
        bool goalBrake{};
        int32_t softPosition{};
        int32_t softVelocity{};
        int32_t softCurrent{};
        bool softDisable{};
        bool softBrake{};
        bool softNewCurve{};
        int32_t focPosition{};
        uint32_t stalledTime{};
        uint32_t overloadTime{};
        bool overloadFlag{};


        void AttachConfig(Config_t* _config);
        void CalcCurrentToOutput(int32_t current);
        void CalcPidToOutput(int32_t _speed);
        void CalcDceToOutput(int32_t _location, int32_t _speed);
        void ClearIntegral() const;

        static int32_t CompensateAdvancedAngle(int32_t _vel);
    };


    struct Config_t
    {
        MotionPlanner::Config_t motionParams{};
        Controller::Config_t ctrlParams{};
    };
    Config_t config;

    MotionPlanner motionPlanner;
    Controller* controller = nullptr;
    EncoderBase* encoder = nullptr;
    DriverBase* driver = nullptr;


    void Tick20kHz();
    void AttachEncoder(EncoderBase* _encoder);
    void AttachDriver(DriverBase* _driver);


private:
    Controller controllerInstance = Controller(this);


    void CloseLoopControlTick();
};

#endif
