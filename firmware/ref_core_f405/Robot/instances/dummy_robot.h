#ifndef REF_STM32F4_FW_DUMMY_ROBOT_H
#define REF_STM32F4_FW_DUMMY_ROBOT_H

#include "algorithms/kinematic/6dof_kinematic.h"
#include "actuators/ctrl_step/ctrl_step.hpp"
#include "string"
// =====================================================================
// MoveJ 速度重构（D-Q1/D-Q2/D-Q4 已拍板，2026-08-27 修订）
// 单位体系：电机轴 r/s（与电机端 CAN 0x07 ratedVelocity 完全一致）
// =====================================================================

// === slider → r/s 换算（slider 100 = 30 r/s）===
static constexpr float SLIDER_TO_RPS = 0.30f;

// === 各轴物理上限（r/s），与电机端 boardConfig.velocityLimit 对齐 ===
// 注意：超过物理极限会失步/堵转，由电机端 FOC/EMF 约束
static constexpr float AXIS_MAX_RPS[7] = {
    30.0f,   // 地轨
    30.0f,   // J1
    30.0f,   // J2
    30.0f,   // J3
    30.0f,   // J4
    30.0f,   // J5
    30.0f    // J6
};

// 电机减速比（用于距离 → 电机转数换算）
// index [0]=地轨 (reduction=1, 直连), [1~5]=J1~J5 (50), [6]=J6 (30)
static constexpr uint8_t MOTOR_REDUCTION[7] = {1, 50, 50, 50, 50, 50, 30};

// 夹爪速度上限（电机轴 r/s，与35关节电机一致）
// 夹爪 CAN ID=8，使用35电机 reduction=16，输出轴速度 = 20/16 ≈ 1.25 r/s
static constexpr float HAND_MAX_RPS = 20.0f;

// v2.7: jointAccRuntime[i]==0 时的兜底加速度（r/s²）
// 与其他速度/减速比常量保持文件作用域，便于 ComputeSyncSpeeds 静态函数访问
static constexpr float FALLBACK_JOINT_ACCELERATION = 10.0f;

#include <cstdint>
#include "rgb.hpp"
#include "eeprom_interface.h"

#define EEPROM_MAGIC 0x12345679

/**
 * @brief 存储在 EEPROM 中的系统固化参数
 *
 * 注意 v2.6 起删除了 jointAccBases[6]（加速度单位统一为电机轴 r/s²，
 * 不再走"基准 × 百分比"双层语义）。EEPROM 结构前移 24 字节，老设备
 * 首次上电会因 magic 不匹配而自动走默认配置（rgbStateStart/Enable/
 * Disable + 亮度），加速度全部以 DEFAULT_JOINT_ACCELERATION=150 起步。
 */
struct EepromConfig {
    uint32_t magic;           // EEPROM 校验魔数，用于判定 Flash 数据是否有效
    uint8_t static_r[3];      // RGB 灯效 R 分量 (索引 0~2 对应不同模式)
    uint8_t static_g[3];      // RGB 灯效 G 分量
    uint8_t static_b[3];      // RGB 灯效 B 分量
    uint8_t rgbBrightness;    // RGB 亮度 0~100 (掉电保持)
    uint32_t rgbStateStart;   // 设备开机启动时的默认灯效模式
    uint32_t rgbStateEnable;  // 机械臂激活/使能状态下的灯效模式
    uint32_t rgbStateDisable; // 机械臂断电/失能状态下的灯效模式
};

/**
 * @brief 机械臂末端执行器夹爪控制类（继承于闭环步进电机基类）
 */
class StepHand : public CtrlStepMotor
{
public:
    StepHand(CAN_HandleTypeDef* hcan, uint8_t id)
        : CtrlStepMotor(hcan, id, false, 16, -100, 100)
    {
    }

    float current     = 1.2f;   // 夹爪闭合或张开时允许的最大驱动电流幅值 (A)
    // 电机角=0 → 夹爪完全张开（对应用户开度 100）
    // 电机角=100 → 夹爪完全闭合（对应用户开度 0）
    float OpenedAngle = 100.0f; // 用户开度 100 时电机应到达的目标角 → 实际夹爪闭合（与命名"张开"矛盾，固件物理特性如此）
    float ClosedAngle = 0.0f;   // 用户开度 0 时电机应到达的目标角 → 实际夹爪张开（与命名"闭合"矛盾，固件物理特性如此）

    /**
     * @brief 基于速度规划的位置环夹爪控制
     * @param _angle 夹爪百分比开度 (0 = 完全闭合, 100 = 完全张开)
     */
    void SetAngleWithSpeedLimit(float _angle)
    {
        float target_angle = OpenedAngle + (ClosedAngle - OpenedAngle) * (_angle / 100.0f);
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

    /**
     * @brief 构建并暴露夹爪对外的通讯层 Fibre 协议接口
     */
    auto MakeProtocolDefinitions()
    {
        return make_protocol_member_list(
            make_protocol_function("set_enable",
                static_cast<CtrlStepMotor&>(*this), &CtrlStepMotor::SetEnable, "enable"),
            make_protocol_function("set_angle",
                *this, &StepHand::SetAngleWithSpeedLimit, "angle"),
            make_protocol_function("set_current",
                *this, &StepHand::SetAngleWithCurrentLimit, "direction"),
            make_protocol_function("set_current_limit",
                static_cast<CtrlStepMotor&>(*this), &CtrlStepMotor::SetCurrentLimit, "current")
        );
    }
};

/**
 * @brief Dummy 6自由度机械臂系统总控与运动学调度类
 */
class DummyRobot
{
public:
    // 地轨相关常量
    // 直连丝杆1605（无减速）：200步/圈 × 1024微步 / 5mm/圈 = 40960 步/mm
    static constexpr float RAIL_STEPS_PER_MM = 40960.0f;

    // 地轨状态变量
    float currentRailPos = 0.0f;   // 地轨当前位置 (mm)
    float targetRailPos = 0.0f;    // 地轨目标位置 (mm)

    float targetRailCurrent = 0.0f; // 地轨目标电流 (mA)

    // ===== 替换原 jointSpeed（旧的含义是关节 °/s，错误） =====
    // 含义改为：电机轴 r/s（slider 换算后的目标速度上限，所有轴共用）
    float jointSpeedRps = 20.0f;  // 默认值 = AXIS_MAX_RPS[1~6] 的关节上限

    // ===== 替换原 dynamicJointSpeeds（旧的 6 个关节 °/s） =====
    // 含义改为：7 个轴的电机轴 r/s（含地轨）
    // 索引 [0]=地轨, [1~6]=关节
    struct DynamicJointSpeeds7 {
        float rps[7] = {0};
    };
    DynamicJointSpeeds7 dynamicJointSpeeds7;

    // 旧字段保留（ServoJ 暂时还用）
    DOF6Kinematic::Joint6D_t dynamicJointSpeeds = {0.5f, 0.5f, 0.5f, 1.5f, 1.5f, 1.5f};

    // 硬编码 30 r/s，非 MoveJ 路径（Homing/Resting/EmergencyStop）的兜底速度。
    float railSpeedRps = 30.0f;

    float targetCurrents[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    // CAN 回传的电机 PID 参数缓存（用于查询）
    int32_t motorDceKps[8] = {0};
    int32_t motorDceKvs[8] = {0};
    int32_t motorDceKis[8] = {0};
    int32_t motorDceKds[8] = {0};

public:
    explicit DummyRobot(CAN_HandleTypeDef* _hcan);
    ~DummyRobot();

    uint32_t rgbStateStart   = RGB::PURE_COLOR_0;
    uint32_t rgbStateEnable  = RGB::PURE_COLOR_1;
    uint32_t rgbStateDisable = RGB::PURE_COLOR_2;

    uint32_t GetRGBMode()    const;
    void     SetRGBMode(uint32_t mode);
    
    void     LoadConfig();
    void     SaveConfig();

    /**
     * @brief 机械臂工作与路径插补调度模式枚举
     */
    enum CommandMode
    {
        COMMAND_TARGET_POINT_SEQUENTIAL    = 1,  // 顺序点位执行模式: 当上一指令完全抵达后方才解锁进行下一运动点插补
        COMMAND_TARGET_POINT_INTERRUPTABLE = 2,  // 可中断点位执行模式: 直接清空历史缓存栈并让新位姿即刻覆盖生效
        COMMAND_CONTINUES_TRAJECTORY       = 3,  // 连续圆滑轨迹模式: 点位与点位间不停顿降速, 以保持动量实现流畅随动
        COMMAND_MOTOR_TUNING               = 4,  // 电机扫频调参模式: 强行将目标切为内部自带信号发生器注入用以测定伯德图等特征
        COMMAND_TORQUE_CONTROL             = 5,  // 直接力矩透传模式: 关闭位置内环, 六轴直接听取并响应电流大小控制信号
        COMMAND_SERVO_J                    = 6,  // 关节高频伺服模式: 取消冗长梯形加减速, 不阻塞高频响应位姿闭环跟随
    };

    /**
     * @brief 用于电机辨识校准的内置低频振荡发生器
     */
    class TuningHelper
    {
    public:
        explicit TuningHelper(DummyRobot* _context) : context(_context) {}
        void StartSweep();
        void Stop();
        bool IsRunning() const;
        auto MakeProtocolDefinitions()
        {
            return make_protocol_member_list(
                make_protocol_function("set_tuning_freq_amp", context->tuningHelper,
                                      &DummyRobot::TuningHelper::SetFreqAndAmp, "freq", "amp"),
                make_protocol_function("set_tuning_flag", context->tuningHelper,
                                      &DummyRobot::TuningHelper::SetTuningFlag, "flag")
            );
        }
        void SetTuningFlag(uint8_t _flag);
        void Tick(uint32_t _timeMillis);
        void SetFreqAndAmp(float _freq, float _amp);
    private:
        DummyRobot* context;
        friend class DummyRobot;
        float   time      = 0;
        uint8_t tuningFlag = 0;
        float   frequency  = 1;
        float   amplitude  = 1;
    };
    TuningHelper tuningHelper = TuningHelper(this);

    // 结构硬变量缺省状态与初始化约束池
    const DOF6Kinematic::Joint6D_t REST_POSE = {0, -75, 180, 0, 0, 0};
    const float DEFAULT_JOINT_SPEED     = 80;

    // v2.6 加速度单位统一：电机轴 r/s²（与 CAN 0x14 入参 float 完全一致）
    // 启动后通过 SyncAllMotorAcceleration() 从电机 EEPROM 读真实值回填
    // FALLBACK_JOINT_ACCELERATION 已提升为文件作用域 constexpr（前面定义），
    // 便于 ComputeSyncSpeeds 静态函数访问；此处不再重复声明。
    const float DEFAULT_JOINT_ACCELERATION = 150.0f;  // r/s²（电机端默认 1000）

    // v2.6 缓存：每个电机轴当前生效加速度（r/s²），由 CAN 0x2C 回包更新
    // 索引: [0]=地轨(node=9), [1~6]=关节(node=1~6), [7]=夹爪(node=8)
    // 初值 0 表示未知，ComputeSyncSpeeds 等算法应使用 FALLBACK_JOINT_ACCELERATION
    float jointAccRuntime[8] = {0};
    // v3.0 (2026-09-01): 与 jointAccRuntime[] 配对的"最后更新 ms 时间戳"
    // 解决 #GETJACC 异步 printf 与 ASCII 任务 UART TX 竞争导致的合并怪行
    // 同步查询通过比对时间戳判断 CAN 回包是否到达
    uint32_t jointAccLastUpdateMs[8] = {0};

    // v3.0: 新增 0x2D 电流限制回包缓存与时间戳（与加速度对称）
    // ASCII #GETI 同步查询使用
    float     jointCurrentLimitRuntime[8]      = {0};
    uint32_t  jointCurrentLimitLastUpdateMs[8] = {0};

    const CommandMode DEFAULT_COMMAND_MODE = COMMAND_TARGET_POINT_SEQUENTIAL;

    // 系统位姿记忆变量与实时状态寄存层
    DOF6Kinematic::Joint6D_t currentJoints  = REST_POSE;  // 当前各关节角度读取缓存 (度)
    DOF6Kinematic::Joint6D_t targetJoints   = REST_POSE;  // 下一插补目标关节位置参数缓存 (度)
    DOF6Kinematic::Joint6D_t initPose       = REST_POSE;  // 装配或上电标定时的机械零点偏移补偿映射表
    DOF6Kinematic::Pose6D_t  currentPose6D  = {};         // 当前设备工作空间笛卡尔位姿坐标投影信息 (系统自动解算保持更新)
    volatile uint8_t jointsStateFlag = 0b00000000;        // 每一位(bit)严格监控和指示对应关节底层的轨迹插补到位触发状况

    CommandMode commandMode = DEFAULT_COMMAND_MODE;        
    uint32_t lastServoTime = 0;                            

    // 分布式通讯执行器操作列表
    // motorJ 包含 7 个空间: index[0] 作为广播掩码，index[1-6] 为 6个活动物理自由度
    CtrlStepMotor* motorJ[7] = {nullptr};
    StepHand* hand = {nullptr};

    // 系统调度与控制函数对外调用面板
    void Init();
    bool MoveJ(float _j1, float _j2, float _j3, float _j4, float _j5, float _j6, float _j7_mm, float _slider);
    bool MoveL(float _x, float _y, float _z, float _a, float _b, float _c);
    bool ServoJ(float _j1, float _j2, float _j3, float _j4, float _j5, float _j6, float _j7_mm);
    void MoveJoints(DOF6Kinematic::Joint6D_t _joints);
    void MoveRailRelative(float _delta_mm);
    void SetJointSpeed(float _speed);
    void SetJointAcceleration(float _acc);
    /**
     * @brief 向所有 8 个电机（地轨+6 关节+夹爪）发 CAN 0x2C 查询，
     *        触发电机回包更新 jointAccRuntime[]（在 can_protocol.cpp 0x2C 处理里写）。
     *        异步执行，不阻塞。回包延迟 ≤1 CAN 帧周期。
     */
    void SyncAllMotorAcceleration();
    void UpdateJointAngles();
    void UpdateJointAnglesCallback();
    void UpdateJointPose6D();
    void Reboot();
    void SetEnable(bool _enable);
    void SetStallMode();
    void SetStallMode(int motorIndex);
    bool IsStalled() const { return isStalled; }
    void BroadcastUnlock();
    void QueryStallStatus();
    void Homing();
    void Resting();
    bool IsMoving();
    bool IsEnabled();
    void SetCommandMode(uint32_t _mode);
    void SetJointCurrents(float c0, float c1, float c2, float c3, float c4, float c5, float c6, float c7);

    // 暴露出厂端网络可调用结点供调试软件(Reftool等)拉取调用
    auto MakeProtocolDefinitions()
    {
        return make_protocol_member_list(
            make_protocol_function("homing",   *this, &DummyRobot::Homing),
            make_protocol_function("resting",  *this, &DummyRobot::Resting),
            make_protocol_object("joint_1",   motorJ[1]->MakeProtocolDefinitions()),
            make_protocol_object("joint_2",   motorJ[2]->MakeProtocolDefinitions()),
            make_protocol_object("joint_3",   motorJ[3]->MakeProtocolDefinitions()),
            make_protocol_object("joint_4",   motorJ[4]->MakeProtocolDefinitions()),
            make_protocol_object("joint_5",   motorJ[5]->MakeProtocolDefinitions()),
            make_protocol_object("joint_6",   motorJ[6]->MakeProtocolDefinitions()),
            make_protocol_object("hand",      hand->MakeProtocolDefinitions()),
            make_protocol_function("reboot",           *this, &DummyRobot::Reboot),
            make_protocol_function("set_enable",       *this, &DummyRobot::SetEnable,       "enable"),
            make_protocol_function("set_stall_mode",   *this, &DummyRobot::SetStallMode),
            make_protocol_function("set_rgb_mode",     *this, &DummyRobot::SetRGBMode,      "mode"),
            make_protocol_function("set_joint_speed",  *this, &DummyRobot::SetJointSpeed,       "speed"),
            make_protocol_function("set_joint_acc",    *this, &DummyRobot::SetJointAcceleration, "acc"),
            make_protocol_function("set_command_mode", *this, &DummyRobot::SetCommandMode,       "mode"),
            make_protocol_object("tuning", tuningHelper.MakeProtocolDefinitions())
        );
    }

    /**
     * @brief 主循环事件排队与命令行字符串分离执行中心
     */
    class CommandHandler
    {
    public:
        explicit CommandHandler(DummyRobot* _context) : context(_context) {}

        void Init()
        {
            commandFifo = osMessageQueueNew(32, 128, nullptr);
        }

        uint32_t    Push(const char *_cmd);
        const char* Pop(uint32_t timeout);
        uint32_t    ParseCommand(const char *_cmd);
        uint32_t    GetSpace();
        void        ClearFifo();
        void        EmergencyStop();

    private:
        DummyRobot*         context;
        osMessageQueueId_t  commandFifo;
        char                strBuffer[128]{};
    };
    CommandHandler commandHandler = CommandHandler(this);

private:
    CAN_HandleTypeDef* hcan;
    float jointSpeedRatio = 1;
    DOF6Kinematic* dof6Solver;
    bool     isEnabled    = false;
    bool     isStalled    = false;
    uint32_t rgbMode      = 0;
};

#endif //REF_STM32F4_FW_DUMMY_ROBOT_H
