#ifndef REF_STM32F4_FW_DUMMY_ROBOT_H
#define REF_STM32F4_FW_DUMMY_ROBOT_H

#include "algorithms/kinematic/6dof_kinematic.h"
#include "actuators/ctrl_step/ctrl_step.hpp"
#include "string"
#define ALL 0

#include <cstdint>
#include "rgb.hpp"
#include "eeprom_interface.h"

#define EEPROM_MAGIC 0x12345679

/**
 * @brief 存储在 EEPROM 中的系统固化参数
 */
struct EepromConfig {
    uint32_t magic;           // EEPROM 校验魔数，用于判定 Flash 数据是否有效
    uint8_t static_r[3];      // RGB 灯效 R 分量 (索引 0~2 对应不同模式)
    uint8_t static_g[3];      // RGB 灯效 G 分量
    uint8_t static_b[3];      // RGB 灯效 B 分量
    uint32_t rgbStateStart;   // 设备开机启动时的默认灯效模式
    uint32_t rgbStateEnable;  // 机械臂激活/使能状态下的灯效模式
    uint32_t rgbStateDisable; // 机械臂断电/失能状态下的灯效模式
    float jointAccBases[6];   // 各个关节电机基准加速度参数
    float railSpeed_mm_s;     // 地轨速度 (mm/s)
    float railAcc_mm_s2;      // 地轨加速度 (mm/s²)
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
    float OpenedAngle = 100.0f; // 夹爪系统处于完全张开状态时对应的内部编码器角度
    float ClosedAngle = 0.0f;   // 夹爪系统处于完全闭合状态时对应的内部编码器角度

    /**
     * @brief 基于速度规划的位置环夹爪控制
     * @param _angle 夹爪百分比开度 (0 = 完全张开, 100 = 完全闭合)
     */
    void SetAngleWithSpeedLimit(float _angle)
    {
        float target_angle = OpenedAngle + (ClosedAngle - OpenedAngle) * (_angle / 100.0f);
        SetAngleWithVelocityLimit(target_angle, 70.0f);
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
    float railSpeed_mm_s = 50.0f;  // 地轨当前速度 (mm/s)，可通过 #SPEED_RAIL 修改
    float railAcc_mm_s2 = 500.0f;  // 地轨加速度 (mm/s²)，可通过 #ACC_RAIL 修改，默认 500

    float targetRailCurrent = 0.0f; // 地轨目标电流 (mA)

    float targetCurrents[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

public:
    explicit DummyRobot(CAN_HandleTypeDef* _hcan);
    ~DummyRobot();

    uint32_t rgbStateStart   = RGB::PURE_COLOR_0;
    uint32_t rgbStateEnable  = RGB::PURE_COLOR_1;
    uint32_t rgbStateDisable = RGB::PURE_COLOR_2;

    bool     GetRGBEnabled() const;
    uint32_t GetRGBMode()    const;
    void     SetRGBEnabled(bool enable);
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

        void SetTuningFlag(uint8_t _flag);
        void Tick(uint32_t _timeMillis);
        void SetFreqAndAmp(float _freq, float _amp);

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
        float   time      = 0;
        uint8_t tuningFlag = 0;
        float   frequency  = 1;
        float   amplitude  = 1;

        bool    rgbEnabled = false;
        uint8_t rgbMode    = RGB::ALL_OFF;
    };
    TuningHelper tuningHelper = TuningHelper(this);

    // 结构硬变量缺省状态与初始化约束池
    const DOF6Kinematic::Joint6D_t REST_POSE = {0, -75, 180, 0, 0, 0};
    const float DEFAULT_JOINT_SPEED     = 80;    
    DOF6Kinematic::Joint6D_t jointAccBases = {150, 100, 200, 200, 200, 200}; 
    const float DEFAULT_JOINT_ACCELERATION_LOW  = 5;     
    const float DEFAULT_JOINT_ACCELERATION_HIGH = 100;   
    const CommandMode DEFAULT_COMMAND_MODE = COMMAND_TARGET_POINT_INTERRUPTABLE;

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
    bool MoveJ(float _j1, float _j2, float _j3, float _j4, float _j5, float _j6, float _j7_mm);
    bool MoveL(float _x, float _y, float _z, float _a, float _b, float _c);
    bool ServoJ(float _j1, float _j2, float _j3, float _j4, float _j5, float _j6, float _j7_mm);
    void MoveJoints(DOF6Kinematic::Joint6D_t _joints);
    void MoveRail(float _railPos_mm);
    void MoveRailRelative(float _delta_mm);
    void SetRailSpeed(float _speed_mm_s);
    void SetRailAcc(float _acc_mm_s2);
    void SetJointSpeed(float _speed);
    void SetJointAcceleration(float _acc);
    void UpdateJointAngles();
    void UpdateJointAnglesCallback();
    void UpdateJointPose6D();
    void Reboot();
    void SetEnable(bool _enable);
    void SetStallMode();
    void SetStallMode(int motorIndex);
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
            make_protocol_function("set_rgb_enable",   *this, &DummyRobot::SetRGBEnabled,   "enable"),
            make_protocol_function("set_rgb_mode",     *this, &DummyRobot::SetRGBMode,      "mode"),
            make_protocol_function("set_joint_speed",  *this, &DummyRobot::SetJointSpeed,       "speed"),
            make_protocol_function("set_joint_acc",    *this, &DummyRobot::SetJointAcceleration, "acc"),
            make_protocol_function("set_rail_speed",  *this, &DummyRobot::SetRailSpeed,  "speed"),
            make_protocol_function("set_rail_acc",    *this, &DummyRobot::SetRailAcc,    "acc"),
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
            commandFifo = osMessageQueueNew(16, 64, nullptr);
        }

        uint32_t    Push(const std::string &_cmd);
        std::string Pop(uint32_t timeout);
        uint32_t    ParseCommand(const std::string &_cmd);
        uint32_t    GetSpace();
        void        ClearFifo();
        void        EmergencyStop();

    private:
        DummyRobot*         context;
        osMessageQueueId_t  commandFifo;
        char                strBuffer[64]{};
    };
    CommandHandler commandHandler = CommandHandler(this);

private:
    CAN_HandleTypeDef* hcan;
    float jointSpeed      = DEFAULT_JOINT_SPEED;
    float jointSpeedRatio = 1;
    DOF6Kinematic::Joint6D_t dynamicJointSpeeds = {0.5f, 0.5f, 0.5f, 1.5f, 1.5f, 1.5f};
    DOF6Kinematic* dof6Solver;
    bool     isEnabled    = false;
    bool     isRGBEnabled = false;
    uint32_t rgbMode      = 0;
};

#endif //REF_STM32F4_FW_DUMMY_ROBOT_H
