#ifndef REF_STM32F4_FW_DUMMY_ROBOT_H
#define REF_STM32F4_FW_DUMMY_ROBOT_H

#include "algorithms/kinematic/6dof_kinematic.h"
#include "actuators/ctrl_step/ctrl_step.hpp"
#include "string"
#define ALL 0

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
 * @note  reduction=16（物理减速比），inverse=true（与 ref 对齐，基类 SetPositionWithVelocityLimit 自动取反）
 *        角度范围 0~100：override SetAngle 后 _angle 直接对应电机轴圈数（0=闭合，100=张开）
 *        override 仅 2 个方法（SetAngle / SetAngleWithVelocityLimit），不 override UpdateAngleCallback（与 ref 一致）
 *        MakeProtocolDefinitions 注册 4 条 fibre，set_enable 用 R1 强转写法避免 fibre 库继承二义性
 */
class StepHand : public CtrlStepMotor
{
public:
    // 构造 reduction=16（物理减速比），inverse=true（让基类 SetPositionWithVelocityLimit 自动取反）
    // 角度范围 0~100（开度百分比，override 后 _angle 直接对应电机圈数）
    StepHand(CAN_HandleTypeDef* hcan, uint8_t id)
        : CtrlStepMotor(hcan, id, true, 16, 0, 100)
    {
    }

    // 运行时电流幅值（A），由 !HAND_I 设置；掉电丢；默认 1.0A（基类无此字段）
    float current = 1.0f;

    // 跳过基类 _angle/360*reduction 归一化，_angle 直接作为电机圈数（基类按 inverse=true 自动取反）
    // 由 !HAND_POS [0-100] 调用，0=闭合，100=张开
    void SetAngle(float _angle)
    {
        if (_angle < 0) _angle = 0;
        if (_angle > 100) _angle = 100;
        SetPositionWithVelocityLimit(_angle, 30.0f);
    }

    // 同 SetAngle 跳过归一化，供带 vel 参数的位置命令使用
    void SetAngleWithVelocityLimit(float _angle, float _vel)
    {
        if (_angle < 0) _angle = 0;
        if (_angle > 100) _angle = 100;
        SetPositionWithVelocityLimit(_angle, _vel);
    }

    // Communication protocol definitions (4 fibre entries，对齐 ref 写法)
    auto MakeProtocolDefinitions()
    {
        return make_protocol_member_list(
            // set_angle 指向 StepHand 自己的 override
            make_protocol_function("set_angle", *this, &StepHand::SetAngle, "angle"),
            // [R1] set_enable 强转写法：避免 fibre 库对继承类成员指针解析的二义性（StepHand 不 override SetEnable）
            make_protocol_function("set_enable", static_cast<CtrlStepMotor&>(*this), &CtrlStepMotor::SetEnable, "enable"),
            // set_current 强转写法：StepHand 不 override SetCurrentSetPoint（基类直接走 cmd=0x03 透传电流）
            make_protocol_function("set_current", static_cast<CtrlStepMotor&>(*this), &CtrlStepMotor::SetCurrentSetPoint, "current"),
            // set_current_limit 强转写法：StepHand 不 override SetCurrentLimit
            make_protocol_function("set_current_limit", static_cast<CtrlStepMotor&>(*this), &CtrlStepMotor::SetCurrentLimit, "current")
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
//    const float DEFAULT_JOINT_SPEED = 30;  // degree/s
    const float DEFAULT_JOINT_SPEED = 50;  // degree/s
    const DOF6Kinematic::Joint6D_t DEFAULT_JOINT_ACCELERATION_BASES = {150, 100, 200, 200, 200, 200};
//    const float DEFAULT_JOINT_ACCELERATION_LOW = 30;    // 0~100
    const float DEFAULT_JOINT_ACCELERATION_LOW = 15;    // 0~100
    const float DEFAULT_JOINT_ACCELERATION_HIGH = 100;  // 0~100
    const CommandMode DEFAULT_COMMAND_MODE = COMMAND_TARGET_POINT_INTERRUPTABLE;


    DOF6Kinematic::Joint6D_t currentJoints = REST_POSE;
    DOF6Kinematic::Joint6D_t targetJoints = REST_POSE;
    DOF6Kinematic::Joint6D_t initPose = REST_POSE;
    DOF6Kinematic::Pose6D_t currentPose6D = {};
    volatile uint8_t jointsStateFlag = 0b00000000;
    CommandMode commandMode = DEFAULT_COMMAND_MODE;
    CtrlStepMotor* motorJ[8] = {nullptr};  // [FIX] 0=ALL代理, 1~6=J1~J6, 7=地轨
    StepHand* hand = {nullptr};

    // 地轨状态变量（mm）
    float currentRailPos = 0.0f;
    float targetRailPos = 0.0f;

    // 7 个轴的速度缓存（电机轴 r/s, 索引 0=地轨, 1..6=J1..J6）
    float dynamicJointSpeeds7[7] = {0};

    // §3.3.1 各轴运行时加速度（r/s²）：异步从电机端 EEPROM 回填（§9 #7）；
    // 本期用 JOINT_ACC_FALLBACK 兜底（初始值同兜底，回填协议就绪后由电机端覆盖）
    float jointAccRuntime[7] = {30.0f, 150.0f, 100.0f, 200.0f, 200.0f, 200.0f, 200.0f};


    void Init();
    bool MoveJ(float _j1, float _j2, float _j3, float _j4, float _j5, float _j6, float _j7_mm, float _speed = 0);
    bool MoveL(float _x, float _y, float _z, float _a, float _b, float _c);
    void MoveJoints(DOF6Kinematic::Joint6D_t _joints);
    void SetJointSpeed(float _speed);
    void SetJointAcceleration(float _acc);
    void UpdateJointAngles();
    void UpdateJointAnglesCallback();
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
            commandFifo = osMessageQueueNew(16, 64, nullptr);
        }

        uint32_t Push(const std::string &_cmd);
        std::string Pop(uint32_t timeout);
        uint32_t ParseCommand(const std::string &_cmd);
        uint32_t GetSpace();
        void ClearFifo();
        void EmergencyStop();


    private:
        DummyRobot* context;
        osMessageQueueId_t commandFifo;
        char strBuffer[64]{};
    };
    CommandHandler commandHandler = CommandHandler(this);


private:
    CAN_HandleTypeDef* hcan;
    float jointSpeed = DEFAULT_JOINT_SPEED;
    float jointSpeedRatio = 1;
    DOF6Kinematic::Joint6D_t dynamicJointSpeeds = {1, 1, 1, 1, 1, 1};
    DOF6Kinematic* dof6Solver;
    bool isEnabled = false;
    bool isRGBEnabled = true;
    uint32_t rgbMode = 0;
};


#endif //REF_STM32F4_FW_DUMMY_ROBOT_H
