#ifndef REF_STM32F4_FW_DUMMY_ROBOT_H
#define REF_STM32F4_FW_DUMMY_ROBOT_H

#include "algorithms/kinematic/6dof_kinematic.h"
#include "actuators/ctrl_step/ctrl_step.hpp"
#include "string"
#include <cstring>
#define ALL 0
class ProtectionCoordinator;

// 地轨相关常量
#define RAIL_FIXED_NODE_ID 7   // 地轨电机固定 Node ID（与 configurations.h 对齐）
#define RAIL_MM_PER_CIRCLE 5.0f // 丝杆1605：5mm/圈

// §3.3.1 物理同步参数（v2.7，加强版）
// 数组索引：[0]=地轨（J7）, [1..6]=J1~J6
// 各轴电机轴减速比：地轨直连丝杆1605(reduction=1)，J1~J5=50，J6=30
constexpr uint8_t MOTOR_REDUCTION[7] = {1, 50, 50, 50, 50, 50, 30};
// 各轴电机轴 r/s 上限（与电机端 EEPROM velocityLimit 对齐；本期统一 30）
constexpr float SLIDER_CAPS_V[7] = {30.0f, 30.0f, 30.0f, 30.0f, 30.0f, 30.0f, 30.0f};
// 各轴加速度兜底（r/s²）：运行时回填未就绪时使用；地轨=30（57 电机端），J1=150, J2=100, J3~J6=200
constexpr float JOINT_ACC_FALLBACK[7] = {30.0f, 150.0f, 100.0f, 200.0f, 200.0f, 200.0f, 200.0f};

/*
  |   PARAMS   | `current_limit` | `acceleration` | `dce_kp` | `dce_kv` | `dce_ki` | `dce_kd` |
  | ---------- | --------------- | -------------- | -------- | -------- | -------- | -------- |
  | **Joint1** | 2               | 30             | 1000     | 80       | 200      | 250      |
  | **Joint2** | 2               | 30             | 1000     | 80       | 200      | 200      |
  | **Joint3** | 2               | 30             | 1500     | 80       | 200      | 250      |
  | **Joint4** | 2               | 30             | 1000     | 80       | 200      | 250      |
  | **Joint5** | 2               | 30             | 1000     | 80       | 200      | 250      |
  | **Joint6** | 2               | 30             | 1000     | 80       | 200      | 250      |
 */


/**
 * @brief 夹爪控制类（继承闭环步进电机基类，参考 firmware/ref_core_f405 StepHand 实现）
 * @note  reduction=16（物理减速比），inverse=true（与 ref 对齐，基类 SetPositionWithMotorRps 自动取反）
 *        角度范围 -100~100（电机轴目标角度单位 = 度，由 SetAngleWithMotorRps 内部做 _angle/360*reduction 度→圈换算）
 *        pos0=闭合，pos100=张开；100 是开度，经 inverse 映射为 -100/360*16 电机圈。
 *        不 override SetAngle / SetAngleWithVelocityLimit / UpdateAngleCallback（与 ref 一致）
 *        MakeProtocolDefinitions 注册 4 条 fibre，set_enable/set_current_limit 用 R1 强转写法避免 fibre 库继承二义性
 */
class StepHand : public CtrlStepMotor
{
public:
    // 构造 reduction=16（物理减速比），inverse=true（让基类 SetAngleWithMotorRps 自动取反）
    // 角度范围 -100~100（电机轴目标角度，单位 = 度）
    StepHand(CAN_HandleTypeDef* hcan, uint8_t id)
        : CtrlStepMotor(hcan, id, true, 16, -100, 100)
    {
    }

    // 运行时电流幅值（A），由 !HAND_I 设置；掉电丢；默认 1.0A（基类无此字段）
    // [Step 6 用户决策] 留 1.0A，不沿用 ref 1.2A
    float current = 1.0f;

    // 标定语义：
    //   pos0 (ClosedMotorAngle) = 夹爪闭合时电机目标角
    //   pos100 (OpenMotorAngle) = 全开100度映射（约4.444电机圈，inverse后为负）。
    float ClosedMotorAngle = 0.0f;   // pos0   时电机目标角（默认标定零点 = 0 度）
    float OpenMotorAngle   = 100.0f; // pos100 时电机目标角（正方向 100 度）

    // [Step 6 用户决策] HAND_MAX_RPS = 30（与决策"统一 30"一致，不沿用 ref 的 20）
    static constexpr float HAND_MAX_RPS = 30.0f;

    /**
     * @brief 基于速度规划的位置环夹爪控制
     * @param _angle 夹爪百分比开度 (0 = 完全闭合, 100 = 完全张开)
     *
     * 映射：用户开度 0~100 → 电机目标角 ClosedMotorAngle~OpenMotorAngle
     *   _angle=0   → motor=ClosedMotorAngle   (闭合)
     *   _angle=100 → motor=OpenMotorAngle     (张开100开度，约4.444电机圈)
     * 内部走 SetAngleWithMotorRps：inverseDirection 取反 + _angle/360*reduction 度→圈换算
     */
    void SetAngleWithSpeedLimit(float _angle)
    {
        float target_angle = ClosedMotorAngle + (OpenMotorAngle - ClosedMotorAngle) * (_angle / 100.0f);
        SetAngleWithMotorRps(target_angle, HAND_MAX_RPS);
    }

    /**
     * @brief 基于电流环的夹爪力度控制模式 (透传纯力矩)
     * @param inverse 夹爪运动受力方向 (+1 代表闭合施力, -1 代表张开施力)
     */
    void SetAngleWithCurrentLimit(float inverse)
    {
        SetCurrentSetPoint(inverse * current);
    }

    /**
     * @brief 查询当前夹爪是否已解除急停/进入工作状态
     */
    bool isEnabled() const
    {
        return state != STOP;
    }

    // Communication protocol definitions (4 fibre entries，对齐 ref 写法)
    auto MakeProtocolDefinitions()
    {
        return make_protocol_member_list(
            // [R1] set_enable 强转写法：避免 fibre 库对继承类成员指针解析的二义性（StepHand 不 override SetEnable）
            make_protocol_function("set_enable",
                static_cast<CtrlStepMotor&>(*this), &CtrlStepMotor::SetEnable, "enable"),
            // set_angle 指 StepHand 自己的 SetAngleWithSpeedLimit（带速度规划 + 度→圈换算）
            make_protocol_function("set_angle",
                *this, &StepHand::SetAngleWithSpeedLimit, "angle"),
            // set_current 指 StepHand 自己的 SetAngleWithCurrentLimit（透传 inverse*current 电流）
            make_protocol_function("set_current",
                *this, &StepHand::SetAngleWithCurrentLimit, "direction"),
            // [R1] set_current_limit 强转写法：StepHand 不 override SetCurrentLimit
            make_protocol_function("set_current_limit",
                static_cast<CtrlStepMotor&>(*this), &CtrlStepMotor::SetCurrentLimit, "current")
        );
    }
};


class DummyRobot
{
public:
    explicit DummyRobot(CAN_HandleTypeDef* _hcan);
    ~DummyRobot();


    enum CommandMode
    {
        COMMAND_TARGET_POINT_SEQUENTIAL = 1,
        COMMAND_TARGET_POINT_INTERRUPTABLE,
        COMMAND_CONTINUES_TRAJECTORY,
        COMMAND_MOTOR_TUNING
    };


    class TuningHelper
    {
    public:
        explicit TuningHelper(DummyRobot* _context) : context(_context)
        {
        }

        void SetTuningFlag(uint8_t _flag);
        void Tick(uint32_t _timeMillis);
        void SetFreqAndAmp(float _freq, float _amp);


        // Communication protocol definitions
        auto MakeProtocolDefinitions()
        {
            return make_protocol_member_list(
                make_protocol_function("set_tuning_freq_amp", *this,
                                       &TuningHelper::SetFreqAndAmp, "freq", "amp"),
                make_protocol_function("set_tuning_flag", *this,
                                       &TuningHelper::SetTuningFlag, "flag")
            );
        }


    private:
        DummyRobot* context;
        float time = 0;
        uint8_t tuningFlag = 0;
        float frequency = 1;
        float amplitude = 1;
    };
    TuningHelper tuningHelper = TuningHelper(this);


    // This is the pose when power on.
    const DOF6Kinematic::Joint6D_t REST_POSE = {0, -75, 180, 0, 0, 0};
    const float DEFAULT_JOINT_SPEED = 50; // 速度档位 1..100；100 = 电机轴 30 r/s
    const CommandMode DEFAULT_COMMAND_MODE = COMMAND_TARGET_POINT_INTERRUPTABLE;


    DOF6Kinematic::Joint6D_t currentJoints = REST_POSE;
    DOF6Kinematic::Joint6D_t targetJoints = REST_POSE;
    DOF6Kinematic::Joint6D_t initPose = REST_POSE;
    DOF6Kinematic::Pose6D_t currentPose6D = {};
    volatile uint8_t jointsStateFlag = 0b00000000;
    CommandMode commandMode = DEFAULT_COMMAND_MODE;
    enum StopState { NORMAL, STOPPING, STOPPED, STARTING, DISABLING, DISABLED };
    bool IsStopLocked() const;
    bool IsCalibrationBusy() const;
    bool IsProtectionParameterBusy() const;
    bool HandleProtectionCommand(const char* cmd, unsigned len, StreamSink& response);
    void ProtectionCallback(uint8_t node, uint8_t command, const uint8_t* data);
    void ObserveMotorCommunication(uint8_t node);
    void PollProtection();
    void PollProtectionPrint();
    uint32_t GetMotionEpoch() const { return motionEpoch; }
    void RequestSoftStop();
    void RequestStart();
    void RequestDisable();
    void PollSoftStop();
    void PollStopPrint();
    void SoftStopCallback(uint8_t node, const uint8_t* data);
    CtrlStepMotor* motorJ[8] = {nullptr};  // [FIX] 0=ALL代理, 1~6=J1~J6, 7=地轨
    StepHand* hand = {nullptr};

    // 地轨状态变量（mm）
    float currentRailPos = 0.0f;
    float targetRailPos = 0.0f;

    // 7 个轴的速度缓存（电机轴 r/s, 索引 0=地轨, 1..6=J1..J6）
    float dynamicJointSpeeds7[7] = {0};

    // §3.3.1 各轴运行时加速度（r/s²）：异步从电机端 EEPROM 回填（§9 #7）；
    // 本期用 JOINT_ACC_FALLBACK 兜底（初始值同兜底，回填协议就绪后由电机端覆盖）
    // [v2026-10-03] 查询端补齐：索引 [7] 扩到 [9]
    //   [0]=ALL 代理空闲槽；[1..7]=J1~J7/地轨（与 motorJ[1..7] 对齐）；[8]=夹爪
    //   初始值沿用 JOINT_ACC_FALLBACK[0..6] + 夹爪 30；由 Init() 末尾 SyncAllMotorAcceleration() 异步覆盖
    float jointAccRuntime[9] = {0};
    uint32_t jointAccLastUpdateMs[9] = {0};
    float jointCurrentLimitRuntime[9] = {0};
    uint32_t jointCurrentLimitLastUpdateMs[9] = {0};

    // [v2026-10-03] PID 缓存（0x28~0x2B 4 帧合并）
    //   [1..7]=J1~J7/地轨；[8]=夹爪；[0]=空闲
    int32_t motorDceKps[9] = {0};
    int32_t motorDceKvs[9] = {0};
    int32_t motorDceKis[9] = {0};
    int32_t motorDceKds[9] = {0};
    volatile uint8_t motorPidRcvdCount[9] = {0};               // 4 帧合并计数器，ISR 写、ASCII 读
    volatile uint32_t motorPidLastUpdateMs[9] = {0};           // [v2026-10-04] ISR 合并 4 帧时同步设置，ASCII 同步等待用
    volatile uint8_t _pid_print_pending[9] = {0};             // [v2026-10-04] ASCII #GET_PID 0 (ALL) 写、OledTask 读+清 0 + printf；ISR 已不再写（避免单节点双发）
    volatile uint8_t _pid_pending_node[9] = {0};               // 触发合并的节点 ID（1~8），ASCII ALL 写


    void Init();
    bool MoveJ(float _j1, float _j2, float _j3, float _j4, float _j5, float _j6, float _j7_mm, float _speed = 0);
    bool MoveL(float _x, float _y, float _z, float _a, float _b, float _c);
    void MoveJoints(DOF6Kinematic::Joint6D_t _joints);
    void SetJointSpeed(float _speed); // 档位 1..100（非物理速度）
    void SetJointAcceleration(float _acc); // 电机轴 r/s²，J1..J7，运行时应用
    void UpdateJointAngles();
    void UpdateJointAnglesCallback();
    bool MotionCompletionCallback(uint8_t node, const uint8_t* data);
    void ObserveMotionFeedback(uint8_t node, float position, bool finished);
    enum MotionPhase { MOTION_IDLE, MOTION_DISPATCHING, MOTION_RUNNING,
                       MOTION_COMPLETED, MOTION_CANCELLED, MOTION_FAILED };
    uint32_t GetMotionGeneration() const { return motionCompletionGeneration; }
    bool IsMotionBusy() const { return motionActive; }
    bool CanSendMotion(uint32_t epoch, uint32_t generation) const {
        return motionActive && isEnabled && !IsStopLocked() &&
               epoch == motionEpoch && generation == motionCompletionGeneration;
    }
    void PollMotionCompletion();
    void CancelMotion();
    bool WaitMotionCompletion(uint32_t id, uint32_t epoch);
    bool TrySetCommandMode(uint32_t mode);
    void PrintMotionStatus(StreamSink& response);
    uint32_t AllocateMotionTaskId() { if (!++issuedMotionId) ++issuedMotionId; return issuedMotionId; }
    uint32_t LastIssuedMotionId() const { return issuedMotionId; }
    void UpdateJointPose6D();
    void Reboot();
    /** 重启全部电机 + 夹爪（**不**重启主控系统）。用于串口 #REBOOT 0。
     *  - J1~J8（含地轨 id=7 + 夹爪 id=8）通过 motorJ[ALL] 广播 **1 帧** StdId=0x7F
     *    （CAN 接收端 id==0 触发所有电机响应，见 dummy-motor-fw/Core/Src/can.c:202-204；
     *      夹爪端 interface_can.cpp case 0x7F 也会响应 ALL 帧）
     *  - **不**再单独调 hand->Reboot()：Step 2 已删除冗余帧（ALL 帧已覆盖夹爪 id=8）
     *  - 不调 HAL_NVIC_SystemReset()，避免主控重启后串口收不到 ok 回包
     *  - 不调 osDelay：1 帧 CAN 在 µs 级装入 Mailbox，硬件异步发完；
     *    block 主任务 500ms 会导致 FixUpdate / OLED / 串口命令处理全部暂停
     *  - 与 Reboot() 的区别：Reboot() 含 HAL_NVIC_SystemReset()，给内部协议层 reboot 调用 */
    void RebootAllMotors();
    void SetEnable(bool _enable);
    void SetRGBEnable(bool _enable);
    bool GetRGBEnabled();
    void SetRGBMode(uint32_t mode);
    uint32_t GetRGBMode();
    void CalibrateHomeOffset();
    void Homing();
    void Resting();
    bool IsMoving();
    bool IsEnabled();
    void SetCommandMode(uint32_t _mode);

    // [v2026-10-03] 查询端补齐：3 个 Sync + 1 个 Poll
    //  - SyncAll* 被 #GET_PID 0 / #GETJACC 0 / #GETI 0 内部调用（ALL 模式触发）
    //  - PollPidPrint 由 OledTask 周期调用，print 4 帧合并完成的 PID 节点
    void SyncAllMotorAcceleration();
    bool RequestMotorAccelerationSync(StreamSink& response);
    void PollMotorAccelerationSync();
    void MotorAccelerationCallback(uint8_t node, float value, uint32_t requestId);
    void SyncAllMotorCurrentLimit();
    void SyncAllMotorPid();
    void PollPidPrint();


    // Communication protocol definitions
    auto MakeProtocolDefinitions()
    {
        return make_protocol_member_list(
            make_protocol_function("calibrate_home_offset", *this, &DummyRobot::CalibrateHomeOffset),
            make_protocol_function("homing", *this, &DummyRobot::Homing),
            make_protocol_function("resting", *this, &DummyRobot::Resting),
            make_protocol_object("joint_1", motorJ[1]->MakeProtocolDefinitions()),
            make_protocol_object("joint_2", motorJ[2]->MakeProtocolDefinitions()),
            make_protocol_object("joint_3", motorJ[3]->MakeProtocolDefinitions()),
            make_protocol_object("joint_4", motorJ[4]->MakeProtocolDefinitions()),
            make_protocol_object("joint_5", motorJ[5]->MakeProtocolDefinitions()),
            make_protocol_object("joint_6", motorJ[6]->MakeProtocolDefinitions()),
            make_protocol_object("joint_all", motorJ[ALL]->MakeProtocolDefinitions()),
            make_protocol_object("hand", hand->MakeProtocolDefinitions()),
            make_protocol_function("reboot", *this, &DummyRobot::Reboot),
            make_protocol_function("set_enable", *this, &DummyRobot::SetEnable, "enable"),
            make_protocol_function("set_rgb_enable", *this, &DummyRobot::SetRGBEnable, "enable"),
            make_protocol_function("set_rgb_mode", *this, &DummyRobot::SetRGBMode, "mode"),
            make_protocol_function("move_j", *this, &DummyRobot::MoveJ, "j1", "j2", "j3", "j4", "j5", "j6", "j7", "speed"),
            make_protocol_function("move_l", *this, &DummyRobot::MoveL, "x", "y", "z", "a", "b", "c"),
            make_protocol_function("set_joint_speed", *this, &DummyRobot::SetJointSpeed, "speed"),
            make_protocol_function("set_joint_acc", *this, &DummyRobot::SetJointAcceleration, "acc"),
            make_protocol_function("set_command_mode", *this, &DummyRobot::SetCommandMode, "mode"),
            make_protocol_object("tuning", tuningHelper.MakeProtocolDefinitions())
        );
    }


    class CommandHandler
    {
    public:
        explicit CommandHandler(DummyRobot* _context) : context(_context)
        {
            commandFifo = osMessageQueueNew(16, sizeof(QueueItem), nullptr);
            cancelledFifo = osMessageQueueNew(32, sizeof(CancelledItem), nullptr);
        }

        uint32_t Push(const std::string &_cmd, StreamSink* response = nullptr);
        uint32_t PushSystem(const std::string& cmd, StreamSink* response = nullptr);
        bool IsBusy() const { return parsing || osMessageQueueGetSpace(commandFifo) != 16; }
        void PollCancelled();
        uint32_t CancellationNoticeLosses() const { return droppedCancellationNotices; }
        static const char* Name(const std::string& cmd) {
            return Name(cmd.c_str());
        }
        static const char* Name(const char* cmd) {
            return std::strcmp(cmd, "!HOME") == 0 ? "HOME" : std::strcmp(cmd, "!RESET") == 0 ? "RESET" :
                   std::strcmp(cmd, "!CALIBRATION") == 0 ? "CALIBRATION" : cmd[0] == '@' ? "MOVEL" : "MOVEJ";
        }
        bool IsTaskCancelled() const { return parsing && activeEpoch != context->GetMotionEpoch(); }
        std::string Pop(uint32_t timeout);
        uint32_t ParseCommand(const std::string &_cmd);
        uint32_t GetSpace();
        void ClearFifo();
        void EmergencyStop();


    private:
        DummyRobot* context;
        osMessageQueueId_t commandFifo;
        char strBuffer[64]{};
        struct QueueItem {
            uint32_t epoch, id;
            CommandMode mode;
            StreamSink* response;
            char command[64];
        };
        struct CancelledItem { uint32_t id; StreamSink* response; const char* kind; };
        osMessageQueueId_t cancelledFifo;
        uint32_t poppedId = 0;
        CommandMode poppedMode = COMMAND_TARGET_POINT_INTERRUPTABLE;
        StreamSink* poppedResponse = nullptr;
        volatile uint32_t announcedId[3] = {};
        uint32_t droppedCancellationNotices = 0;
        void Announce(const QueueItem& item);
        bool IsAnnounced(uint32_t id, StreamSink* response) const;
        uint32_t poppedEpoch = 0;
        bool parsing = false;
        uint32_t activeEpoch = 0;
        struct ExecutionScope
        {
            CommandHandler& handler;
            ExecutionScope(CommandHandler& h, uint32_t epoch) : handler(h)
            { h.activeEpoch = epoch; h.parsing = true; }
            ~ExecutionScope() { handler.parsing = false; }
        };
    };
    CommandHandler commandHandler = CommandHandler(this);


private:
    friend class ProtectionCoordinator;
    ProtectionCoordinator* protection = nullptr;
    // A tagged, one-at-a-time USB synchronization job. ISR only records replies.
    enum AccSyncPhase { ACC_SYNC_IDLE, ACC_SYNC_ARMING, ACC_SYNC_RUNNING, ACC_SYNC_REPORTING };
    static constexpr uint16_t ACC_SYNC_ALL_NODES = 0x1FE; // J1..J8
    static constexpr uint32_t ACC_SYNC_TIMEOUT_MS = 500;
    volatile AccSyncPhase accSyncPhase = ACC_SYNC_IDLE;
    uint32_t accSyncRequestId = 0;
    uint32_t accSyncEpoch = 0;
    uint32_t accSyncStartedMs = 0;
    volatile uint16_t accSyncSentMask = 0;
    volatile uint16_t accSyncReceivedMask = 0;
    volatile uint16_t accSyncInvalidMask = 0;
    float accSyncValues[9] = {};
    uint32_t accSyncUpdateMs[9] = {};
    StreamSink* accSyncResponse = nullptr;

    CAN_HandleTypeDef* hcan;
    float jointSpeed = DEFAULT_JOINT_SPEED;
    DOF6Kinematic::Joint6D_t dynamicJointSpeeds = {1, 1, 1, 1, 1, 1};
    DOF6Kinematic* dof6Solver;
    volatile bool isEnabled = false;
    volatile StopState stopState = DISABLED;
    volatile uint32_t motionEpoch = 0;
    volatile uint16_t stopToken = 0;
    volatile uint16_t stopAckMask = 0;
    volatile uint8_t stopAction = 0;
    volatile bool startPending = false;
    bool stopStartAllEight = true;
    bool stopWithGripper = false; // Only cancellation of J8's internal calibration.
    volatile uint16_t stopHoldingMask = 0;
    bool stopPerNodeTokens = false;
    volatile uint16_t stopSyncedMask = 0;
    volatile bool motionActive = false;
    volatile bool motionDispatchPending = false;
    volatile uint32_t motionCompletionGeneration = 0;
    uint32_t motionQueryCounter = 0;
    volatile uint32_t motionQueryNonce[8] = {};
    volatile uint32_t motionQueryGeneration[8] = {};
    volatile uint32_t motionQueryAt[8] = {};
    volatile uint32_t motionReplyAt[8] = {};
    volatile float motionMotorTargets[8] = {};
    volatile uint16_t motionConfirmedMask = 0;
    volatile MotionPhase motionPhase = MOTION_IDLE;
    volatile uint32_t motionTaskId = 0;
    uint32_t issuedMotionId = 0;
    uint32_t motionTaskEpoch = 0, motionStartedAt = 0, motionCompletedAt = 0;
    CommandMode motionMode = DEFAULT_COMMAND_MODE;
    const char* motionKind = "MOVEJ";
    float motionMotorSpeeds[8] = {};
    volatile uint16_t motionTargetReadyMask = 0, motionSentMask = 0;
    volatile uint16_t motionProgressMask = 0;
    uint32_t motionSendAt[8] = {}, motionQueryAttemptAt[8] = {};
    volatile uint32_t motionRound = 0;
    volatile uint32_t motionQueryRound[8] = {};
    uint32_t motionRoundAt = 0;
    volatile uint16_t motionRoundSeenMask = 0, motionRoundGoodMask = 0;
    volatile uint8_t motionReplyFlags[8] = {};
    volatile float motionReplyPositions[8] = {};
    volatile uint32_t motionRejectedReplies = 0;
    uint32_t motionSendCount = 0, motionQueryCount = 0;
    uint8_t motionQueryNode = 1;
    void BeginMotionRound();
    uint16_t FreshMotionCompletionMask();
    bool NeedsMotionCompletion() const {
        return isEnabled && motionActive && motionMode >= COMMAND_TARGET_POINT_SEQUENTIAL &&
               motionMode <= COMMAND_CONTINUES_TRAJECTORY && !IsStopLocked();
    }
    volatile bool stopNeedsSend = false;
    uint8_t stopPollNode = 1;
    uint32_t stopLastSendMs = 0;
    float stopPositions[9] = {};
    volatile uint16_t stopNodeTokens[9] = {};
    const char* volatile pendingStopNotice = nullptr;
    volatile uint16_t stopFaultMask = 0;
    uint16_t stopFaultPrinted = 0;
    volatile uint16_t stopRxMask = 0;
    volatile uint16_t stopBadPositionMask = 0;
    volatile uint8_t stopRxPhases[9] = {};
    volatile uint16_t stopRxTokens[9] = {};
    volatile uint32_t stopDiagPollCount = 0;
    volatile bool stopDiagTxBusy = false;
    volatile uint8_t stopDiagTxNode = 0;
    uint32_t stopDiagSinceMs = 0;
    bool stopDiagPending = false;
    void ResetStopDiagnostics();
    bool PlanMoveJ(float j1, float j2, float j3, float j4, float j5, float j6,
                   float rail, float speed, uint32_t epoch, uint32_t id = 0, const char* kind = "MOVEJ");
    bool PlanMoveL(float x, float y, float z, float a, float b, float c, uint32_t epoch, uint32_t id = 0);
    void SetMotionSpeed(float speed, uint32_t epoch);
    bool HomingForGeneration(uint32_t epoch, uint32_t id = 0);
    bool RestingForGeneration(uint32_t epoch, uint32_t id = 0);
    void CalibrateForGeneration(uint32_t epoch);
    void RebootForGeneration(uint32_t epoch);
    bool isRGBEnabled = true;
    uint32_t rgbMode = 0;
};


#endif //REF_STM32F4_FW_DUMMY_ROBOT_H
