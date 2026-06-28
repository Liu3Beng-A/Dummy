#include "communication.hpp"
#include "dummy_robot.h"
#include "time_utils.h"
#include <cstring>

extern RGB rgb;
EEPROMClass EEPROM;

/**
 * @brief 求算传入六自由度关节数组中幅值（绝对值）最极端的成员
 * @param _joints 待查验比较的六路关节量向量
 * @param _index 引用导出目标轴在数组中的定位索引 (0 到 5)
 * @return 提取出最大的绝对极值
 * @note 此算法常作为协同速度降幅规划或从逆运动学众解中发掘平滑最近节点的参考极规。
 */
inline float AbsMaxOf6(DOF6Kinematic::Joint6D_t _joints, uint8_t &_index)
{
    float max = -1;
    for (uint8_t i = 0; i < 6; i++)
    {
        if (fabsf(_joints.a[i]) > max)
        {
            max = fabsf(_joints.a[i]);
            _index = i;
        }
    }
    return max;
}

/**
 * @brief 主脑对象初始化部署程序
 * @param _hcan 通信层所强依赖的 CAN 指令下发数据流桥接通道句柄
 * @note 构建完备系统骨骼：motorJ[0]=地轨(ID=9, 固定), motorJ[1-6]=臂关节(ID=1-6), hand=夹爪(ID=8, 固定)
 */
DummyRobot::DummyRobot(CAN_HandleTypeDef* _hcan) :
    hcan(_hcan)
{
    // motorJ[0]: 地轨（线性滑轨，直连丝杆1605，转1圈=5mm，行程 -250~250mm）
    motorJ[0] = new CtrlStepMotor(_hcan, 9, false, 1, -250, 250);

    motorJ[1] = new CtrlStepMotor(_hcan, 1, false, 50, -175, 175);
    motorJ[2] = new CtrlStepMotor(_hcan, 2, true,  50,  -75,  90);
    motorJ[3] = new CtrlStepMotor(_hcan, 3, true,  50,    0, 180);
    motorJ[4] = new CtrlStepMotor(_hcan, 4, true,  50, -270, 270);
    motorJ[5] = new CtrlStepMotor(_hcan, 5, true,  50, -100, 100);
    motorJ[6] = new CtrlStepMotor(_hcan, 6, true,  30, -180, 180);

    hand = new StepHand(_hcan, 8);

    // 地轨位置初始化（mm）
    currentRailPos = 0.0f;
    targetRailPos = 0.0f;

    // 载入 D-H 标准模型基建长度数值搭建系统运算内核空间
    dof6Solver = new DOF6Kinematic(0.165f, 0.0f, 0.170f, 0.117f, 0.0695f, 0.113f);
}

/**
 * @brief 系统垃圾回收：安全清理掉为底层各传动端分配的指针空间防溢出漏错
 */
DummyRobot::~DummyRobot()
{
    for (int j = 0; j <= 6; j++)
        delete motorJ[j];

    delete hand;       
    delete dof6Solver;
}

/**
 * @brief 连接 Flash 介质取用长期休眠前的运行变量存根
 * @note 读取灯效风格预设与核心运动关节平稳性保护加速度上限规范，
 *       若首次启动匹配不到特解标识字，会自动写回原始初始化表覆写空白位。
 */
void DummyRobot::LoadConfig()
{
    EepromConfig config;
    EEPROM.get(0, config);
    if (config.magic == EEPROM_MAGIC)
    {
        for (int i = 0; i < 3; i++)
        {
            rgb.static_r[i] = config.static_r[i];
            rgb.static_g[i] = config.static_g[i];
            rgb.static_b[i] = config.static_b[i];
        }

        if (config.rgbBrightness <= 100) {
            rgb.brightness = (float)config.rgbBrightness / 100.0f;
            rgb.targetBrightness = rgb.brightness;
        }

        if (config.rgbStateStart <= 9)   rgbStateStart = config.rgbStateStart;
        if (config.rgbStateEnable <= 9)  rgbStateEnable = config.rgbStateEnable;
        if (config.rgbStateDisable <= 9) rgbStateDisable = config.rgbStateDisable;

        for (int i = 0; i < 6; i++)
        {
            if (config.jointAccBases[i] >= 1.0f && config.jointAccBases[i] <= 2000.0f)
                jointAccBases.a[i] = config.jointAccBases[i];
        }

        if (config.railSpeed_mm_s >= 0.5f && config.railSpeed_mm_s <= 100.0f)
            railSpeed_mm_s = config.railSpeed_mm_s;
        if (config.railAcc_mm_s2 >= 10.0f && config.railAcc_mm_s2 <= 5000.0f)
            railAcc_mm_s2 = config.railAcc_mm_s2;
    }
}

/**
 * @brief 将内存中所挂载配置变更改写印录在 EEPROM 以提供掉电恢复功能
 */
void DummyRobot::SaveConfig()
{
    EepromConfig config;
    config.magic = EEPROM_MAGIC;
    for (int i = 0; i < 3; i++)
    {
        config.static_r[i] = rgb.static_r[i];
        config.static_g[i] = rgb.static_g[i];
        config.static_b[i] = rgb.static_b[i];
    }
    config.rgbBrightness = (uint8_t)(rgb.targetBrightness * 100.0f + 0.5f);
    config.rgbStateStart = rgbStateStart;
    config.rgbStateEnable = rgbStateEnable;
    config.rgbStateDisable = rgbStateDisable;

    for (int i = 0; i < 6; i++)
    {
        config.jointAccBases[i] = jointAccBases.a[i];
    }
    config.railSpeed_mm_s = railSpeed_mm_s;
    config.railAcc_mm_s2 = railAcc_mm_s2;

    EEPROM.put(0, config);
    EEPROM.commit();
}

/**
 * @brief 对主系统实行上电挂载初始化指令集派发并赋默认预定状态
 */
void DummyRobot::Init()
{
    commandHandler.Init();
    LoadConfig();

    SetRGBMode(rgbStateStart);
    SetCommandMode(DEFAULT_COMMAND_MODE);
    SetJointSpeed(DEFAULT_JOINT_SPEED);
}

/**
 * @brief 群发全局急停保护且指令微控制器彻底脱壳软重启
 */
void DummyRobot::Reboot()
{
    motorJ[0]->Reboot();
    for (int i = 1; i <= 6; i++)
        motorJ[i]->Reboot();
    hand->Reboot();
    osDelay(500);
    HAL_NVIC_SystemReset();
}

/**
 * @brief 向所有关节推入带有限速补偿的目标逼近指令点
 * @param _joints 各电机关节待命执行的目标刻度(带零偏补偿考量)
 */
void DummyRobot::MoveJoints(DOF6Kinematic::Joint6D_t _joints)
{
    for (int j = 1; j <= 6; j++)
    {
        motorJ[j]->SetAngleWithVelocityLimit(_joints.a[j - 1] - initPose.a[j - 1],
                                             dynamicJointSpeeds.a[j - 1]);
    }
}

/**
 * @brief 下发地轨指令（mm → 圈）
 * @param _railPos_mm 地轨目标位置 (mm)
 * @note 地轨不纳入6-DOF运动学求解，单独管理
 * @note 电机固件 CAN 协议期望接收：位置(圈)、速度(圈/s)，内部乘以细分系数
 */
void DummyRobot::MoveRail(float _railPos_mm)
{
    // 丝杆1605直连：5mm/圈
    float rail_laps = _railPos_mm / 5.0f;  // mm → 圈
    float speed_laps = railSpeed_mm_s / 5.0f;  // mm/s → 圈/s
    float acc_laps = railAcc_mm_s2 / 5.0f;     // mm/s² → 圈/s²

    // 先下发加速度（CAN 0x14），再下发位置+速度（CAN 0x07）
    motorJ[0]->SetAcceleration(acc_laps);
    motorJ[0]->SetPositionWithVelocityLimit(rail_laps, speed_laps);
}

void DummyRobot::MoveRailRelative(float _delta_mm)
{
    targetRailPos += _delta_mm;
    // 硬限位保护，防止超出 [-250, 250]
    if (targetRailPos > motorJ[0]->angleLimitMax)
        targetRailPos = motorJ[0]->angleLimitMax;
    if (targetRailPos < motorJ[0]->angleLimitMin)
        targetRailPos = motorJ[0]->angleLimitMin;
    MoveRail(targetRailPos);
}

/**
 * @brief 设置地轨运行速度
 * @param _speed_mm_s 地轨目标速度 (mm/s)
 * @note 限幅范围 [0.5, 100] mm/s，超出范围自动截断
 */
void DummyRobot::SetRailSpeed(float _speed_mm_s)
{
    if (_speed_mm_s < 0.5f)        _speed_mm_s = 0.5f;
    else if (_speed_mm_s > 100.0f) _speed_mm_s = 100.0f;
    railSpeed_mm_s = _speed_mm_s;
}

/**
 * @brief 设置地轨运行加速度
 * @param _acc_mm_s2 地轨目标加速度 (mm/s²)
 * @note 限幅范围 [10, 5000] mm/s²，超出范围自动截断
 * @note 每次 MoveRail 时自动下发到电机固件
 */
void DummyRobot::SetRailAcc(float _acc_mm_s2)
{
    if (_acc_mm_s2 < 10.0f)         _acc_mm_s2 = 10.0f;
    else if (_acc_mm_s2 > 5000.0f)  _acc_mm_s2 = 5000.0f;
    railAcc_mm_s2 = _acc_mm_s2;
}

/**
 * @brief 解析空间六维坐标并令其映射入安全界域内化为电机目标偏角实现平稳直线位移
 * @param _x, _y, _z 工作空间末端探针位置参考系 (标准计度)
 * @param _a, _b, _c 空间内姿态偏转对应四元欧拉角反算值
 * @return 布尔反馈代表其能否在有限的运动机能与逆求解内完成安全收敛响应
 */
bool DummyRobot::MoveL(float _x, float _y, float _z, float _a, float _b, float _c)
{
    DOF6Kinematic::Pose6D_t pose6D(_x, _y, _z, _a, _b, _c);
    DOF6Kinematic::IKSolves_t ikSolves{};

    dof6Solver->SolveIK(pose6D, currentJoints, ikSolves);

    float   minDist    = 1e9f;
    int     bestConfig = -1;

    for (int i = 0; i < 8; i++)
    {
        bool valid = true;
        for (int j = 1; j <= 6; j++)
        {
            if (ikSolves.config[i].a[j - 1] > motorJ[j]->angleLimitMax ||
                ikSolves.config[i].a[j - 1] < motorJ[j]->angleLimitMin)
            {
                valid = false;
                break;
            }
        }
        if (valid)
        {
            uint8_t idx;
            DOF6Kinematic::Joint6D_t delta = currentJoints - ikSolves.config[i];
            float d = AbsMaxOf6(delta, idx);
            if (d < minDist)
            {
                minDist    = d;
                bestConfig = i;
            }
        }
    }

    if (bestConfig >= 0)
    {
        return MoveJ(ikSolves.config[bestConfig].a[0],
                     ikSolves.config[bestConfig].a[1],
                     ikSolves.config[bestConfig].a[2],
                     ikSolves.config[bestConfig].a[3],
                     ikSolves.config[bestConfig].a[4],
                     ikSolves.config[bestConfig].a[5],
                     currentRailPos);  // 地轨位置保持不变
    }
    return false;
}

/**
 * @brief 向定点旋转并发规划驱动组群下属协同运转指令
 * @param _j1~_j6: 臂关节角度 (°), _j7_mm: 地轨位置 (mm)
 * @note 内置基于极限基准点运算降维匹配同步缩放比例限速引擎保护
 */
bool DummyRobot::MoveJ(float _j1, float _j2, float _j3, float _j4, float _j5, float _j6, float _j7_mm)
{
    DOF6Kinematic::Joint6D_t targetJointsTmp(_j1, _j2, _j3, _j4, _j5, _j6);
    uint8_t maxIndex;

    // 地轨限位检查
    if (_j7_mm > motorJ[0]->angleLimitMax || _j7_mm < motorJ[0]->angleLimitMin)
        return false;

    // 臂关节限位检查
    for (int j = 1; j <= 6; j++)
    {
        if (targetJointsTmp.a[j - 1] > motorJ[j]->angleLimitMax ||
            targetJointsTmp.a[j - 1] < motorJ[j]->angleLimitMin)
            return false;
    }

    // 计算各轴速度（保证所有关节同时到达）
    DOF6Kinematic::Joint6D_t deltaAngles = targetJointsTmp - currentJoints;
    float maxAngle = AbsMaxOf6(deltaAngles, maxIndex);
    float timeSec  = maxAngle / jointSpeed;

    for (int j = 1; j <= 6; j++)
    {
        dynamicJointSpeeds.a[j - 1] = fabsf(deltaAngles.a[j - 1]) / timeSec;
        if (dynamicJointSpeeds.a[j - 1] < 0.05f)
            dynamicJointSpeeds.a[j - 1] = 0.05f;
    }

    targetJoints = targetJointsTmp;
    targetRailPos = _j7_mm;  // 存储地轨目标位置

    // 写入目标角度（纯位置误差判定用，不再依赖 jointsStateFlag）
    for (int j = 1; j <= 6; j++) {
        motorJ[j]->targetAngle = targetJointsTmp.a[j - 1] - initPose.a[j - 1];
    }

    return true;
}

/**
 * @brief 无阻塞高通量前馈跟随驱动随动策略
 * @param _j1~_j6: 臂关节角度 (°), _j7_mm: 地轨位置 (mm)
 * @note 基于微秒精度的指令脉冲插补微分求导实时计算需求速度完成极速响应映射闭环跟踪
 */
bool DummyRobot::ServoJ(float _j1, float _j2, float _j3, float _j4, float _j5, float _j6, float _j7_mm)
{
    DOF6Kinematic::Joint6D_t targetJointsTmp(_j1, _j2, _j3, _j4, _j5, _j6);

    // 地轨限位检查
    if (_j7_mm > motorJ[0]->angleLimitMax || _j7_mm < motorJ[0]->angleLimitMin)
        return false;

    for (int j = 1; j <= 6; j++)
    {
        if (targetJointsTmp.a[j - 1] > motorJ[j]->angleLimitMax ||
            targetJointsTmp.a[j - 1] < motorJ[j]->angleLimitMin)
            return false;
    }

    uint32_t nowUs = micros();
    float dt = (nowUs - lastServoTime) / 1000000.0f;
    if (dt <= 0.001f) dt = 0.02f;
    lastServoTime = nowUs;

    for (int j = 1; j <= 6; j++)
    {
        float deltaAngle = fabsf(targetJointsTmp.a[j - 1] - currentJoints.a[j - 1]);
        float reqSpeed   = deltaAngle / dt * 1.5f;

        dynamicJointSpeeds.a[j - 1] = (reqSpeed < 0.05f)   ? 0.05f   :
                                      (reqSpeed > 200.0f) ? 200.0f : reqSpeed;
    }

    targetJoints = targetJointsTmp;
    targetRailPos = _j7_mm;  // 存储地轨目标位置

    // 写入目标角度（ServoJ 也用纯位置误差判定）
    for (int j = 1; j <= 6; j++) {
        motorJ[j]->targetAngle = targetJointsTmp.a[j - 1] - initPose.a[j - 1];
    }

    return true;
}

/**
 * @brief 利用抽屉分时循环结构避让单点查询打满 CAN 信道容量引发断线隐患
 */
void DummyRobot::UpdateJointAngles()
{
    static uint8_t group = 0;

    switch (group)
    {
        case 0:
            motorJ[1]->UpdateAngle();
            motorJ[2]->UpdateAngle();
            break;
        case 1:
            motorJ[3]->UpdateAngle();
            motorJ[4]->UpdateAngle();
            break;
        case 2:
            motorJ[5]->UpdateAngle();
            motorJ[6]->UpdateAngle();
            break;
        default:
            break;
    }

    group = (group + 1) % 3;
}

/**
 * @brief 在触发回调接管解析到的节点坐标包进而推至逻辑状态判断矩阵更新标记
 */
void DummyRobot::UpdateJointAnglesCallback()
{
    for (int i = 1; i <= 6; i++)
    {
        currentJoints.a[i - 1] = motorJ[i]->angle + initPose.a[i - 1];
        // jointsStateFlag 不再操作，IsMoving() 直接用位置误差判定
    }
}

/**
 * @brief 分配调准基准速率运行档位
 */
void DummyRobot::SetJointSpeed(float _speed)
{
    if (_speed < 0)        _speed = 0;
    else if (_speed > 100) _speed = 100;

    jointSpeed = _speed * jointSpeedRatio;
}

/**
 * @brief 基于底层参数配给换算映射应用新加速度限制表尺
 */
void DummyRobot::SetJointAcceleration(float _acc)
{
    if (_acc < 0)        _acc = 0;
    else if (_acc > 100) _acc = 100;

    for (int i = 1; i <= 6; i++)
        motorJ[i]->SetAcceleration(_acc / 100.0f * jointAccBases.a[i - 1]);
}

/**
 * @brief 复归寻位归零标定启动策略流程
 */
void DummyRobot::SetStallMode()
{
    SetStallMode(-1);  // 不指定电机，全部停住
}

void DummyRobot::SetStallMode(int motorIndex)
{
    // 切换 RGB 为红色心跳，视觉提示堵转
    SetRGBMode(RGB::RED_HEARTBEAT);
    // 停发新位置指令，保持当前位置（同步 targetAngle 避免误判）
    targetJoints = currentJoints;
    for (int j = 1; j <= 6; j++) {
        motorJ[j]->targetAngle = currentJoints.a[j - 1] - initPose.a[j - 1];
    }
    // 清空指令队列，防止残留指令堆积
    commandHandler.ClearFifo();
    (void)motorIndex;  // 未来可用于区分哪个电机堵转并做针对性处理
}

void DummyRobot::Homing()
{
    float lastSpeed = jointSpeed;
    SetJointSpeed(10);

    MoveJ(0, 0, 90, 0, 0, 0, 0);  // 归零姿态，地轨=0mm
    MoveJoints(targetJoints);
    MoveRail(targetRailPos);
    while (IsMoving())
        osDelay(10);

    SetJointSpeed(lastSpeed);
}

/**
 * @brief 将设备挂入无伤放松的安全缩骨隐蔽初始安睡位置
 */
void DummyRobot::Resting()
{
    float lastSpeed = jointSpeed;
    SetJointSpeed(10);

    MoveJ(REST_POSE.a[0], REST_POSE.a[1], REST_POSE.a[2],
          REST_POSE.a[3], REST_POSE.a[4], REST_POSE.a[5], 0);  // 待机姿态，地轨=0mm
    MoveJoints(targetJoints);
    MoveRail(targetRailPos);
    while (IsMoving())
        osDelay(10);

    SetJointSpeed(lastSpeed);
}

/**
 * @brief 发送节点通断电流源动力配置及 RGB 等附属工作展示配合转换
 */
void DummyRobot::SetEnable(bool _enable)
{
    if (_enable)
    {
        SetRGBMode(rgbStateEnable);
    }
    else
    {
        SetRGBMode(rgbStateDisable);

        for (int i = 0; i < 6; i++)
            targetCurrents[i] = 0.0f;

        SetCommandMode(DEFAULT_COMMAND_MODE);
        targetJoints = currentJoints; 
    }

    for (int i = 1; i <= 6; i++)
        motorJ[i]->SetEnable(_enable);
    motorJ[0]->SetEnable(_enable);  // 地轨
    hand->SetEnable(_enable);       // 夹爪
    isEnabled = _enable;
}

/**
 * @brief 获取映射渲染花式编号
 */
uint32_t DummyRobot::GetRGBMode() const
{
    return rgbMode;
}

/**
 * @brief 设定映射渲染花式编号
 */
void DummyRobot::SetRGBMode(uint32_t mode)
{
    rgbMode = mode;
}

/**
 * @brief 同步激活解算器矩阵变换方程计算求出物理坐标系投射输出给UI面板等终端查询组件
 */
void DummyRobot::UpdateJointPose6D()
{
    dof6Solver->SolveFK(currentJoints, currentPose6D);
    currentPose6D.X *= 1000;
    currentPose6D.Y *= 1000;
    currentPose6D.Z *= 1000;
}

/**
 * @brief 利用纯位置误差判定所有关节是否已完成收敛
 * @note 废弃 jointsStateFlag 和电机 state 字段的双层判定。
 *       直接比较 motorJ[i]->angle（实测）和 motorJ[i]->targetAngle（目标）。
 *       当 |实测 - 目标| <= 1.0° 时认为该轴到位。
 */
bool DummyRobot::IsMoving()
{
    static constexpr float EPSILON_DEG = 1.0f;
    for (int i = 1; i <= 6; i++)
    {
        if (fabsf(motorJ[i]->angle - motorJ[i]->targetAngle) > EPSILON_DEG)
            return true;   // 有轴未到位，还在动
    }
    return false;          // 所有轴都到位
}

/**
 * @brief 反馈外围调配安全控制开关当前情况
 */
bool DummyRobot::IsEnabled()
{
    return isEnabled;
}

/**
 * @brief 进行动力指令响应分发机制转盘的切入与挂载新特例算法配置的套用执行
 */
void DummyRobot::SetCommandMode(uint32_t _mode)
{
    if (_mode < COMMAND_TARGET_POINT_SEQUENTIAL ||
        _mode > COMMAND_TORQUE_CONTROL)
        return;

    commandMode = static_cast<CommandMode>(_mode);

    switch (commandMode)
    {
        case COMMAND_TARGET_POINT_SEQUENTIAL:
        case COMMAND_TARGET_POINT_INTERRUPTABLE:
            jointSpeedRatio = 1;
            SetJointAcceleration(DEFAULT_JOINT_ACCELERATION_LOW);
            break;

        case COMMAND_CONTINUES_TRAJECTORY:
            SetJointAcceleration(DEFAULT_JOINT_ACCELERATION_LOW);
            jointSpeedRatio = 0.5f; 
            break;

        case COMMAND_MOTOR_TUNING:
            break;

        case COMMAND_TORQUE_CONTROL:
            break;

        case COMMAND_SERVO_J:
            SetJointAcceleration(100.0f); 
            break;
    }
}

/**
 * @brief 使用安全字节转移封包放入信道队列排位阻断越界爆破可能
 */
uint32_t DummyRobot::CommandHandler::Push(const std::string &_cmd)
{
    char buf[64] = {0};
    strncpy(buf, _cmd.c_str(), sizeof(buf) - 1);
    osStatus_t status = osMessageQueuePut(commandFifo, buf, 0U, 0U);
    if (status == osOK)
        return osMessageQueueGetSpace(commandFifo);

    return 0xFF; 
}

/**
 * @brief 清仓强制切断挂载流任务并施下锁盘制动保全安全边界
 */
void DummyRobot::CommandHandler::EmergencyStop()
{
    context->MoveJ(context->currentJoints.a[0], context->currentJoints.a[1],
                   context->currentJoints.a[2], context->currentJoints.a[3],
                   context->currentJoints.a[4], context->currentJoints.a[5],
                   context->currentRailPos);
    context->MoveJoints(context->targetJoints);
    context->MoveRail(context->targetRailPos);
    context->isEnabled = false;
    ClearFifo();
}

/**
 * @brief 在队列排布端向外吐出封存任务项
 */
std::string DummyRobot::CommandHandler::Pop(uint32_t timeout)
{
    osStatus_t status = osMessageQueueGet(commandFifo, strBuffer, nullptr, timeout);
    return std::string{strBuffer};
}

/**
 * @brief 提供查询通信存蓄负荷的容积指示
 */
uint32_t DummyRobot::CommandHandler::GetSpace()
{
    return osMessageQueueGetSpace(commandFifo);
}

/**
 * @brief ASCII 原生命令文本处理工厂
 * @note 提取包头前置标志分类送入多分支行为反应生成节点进行解包运作分配
 */
uint32_t DummyRobot::CommandHandler::ParseCommand(const std::string &_cmd)
{
    uint8_t argNum;

    // [分支拦截] $ 高频投递的力控透传包剥离拦截，规避无意义繁杂判决延宕
    if (_cmd[0] == '$')
    {
        // $c0(地轨),c1~c6(关节),c7(夹爪)
        float cur[7];
        argNum = sscanf(_cmd.c_str(), "$%f,%f,%f,%f,%f,%f,%f",
                        &cur[0], &cur[1], &cur[2], &cur[3], &cur[4], &cur[5], &cur[6]);

        if (argNum == 7)
        {
            if (context->commandMode != COMMAND_TORQUE_CONTROL)
                context->SetCommandMode(COMMAND_TORQUE_CONTROL);

            context->SetJointCurrents(cur[0], cur[1], cur[2], cur[3], cur[4], cur[5], cur[6], 0.0f);
        }
        return osMessageQueueGetSpace(commandFifo);
    }

    // [分支拦截] 模式异常纠正与反弹防呆保护处理屏障
    if ((_cmd[0] == '>' || _cmd[0] == '@' || _cmd[0] == '&') &&
        context->commandMode == COMMAND_TORQUE_CONTROL)
    {
        context->SetCommandMode(context->DEFAULT_COMMAND_MODE);
        context->targetJoints = context->currentJoints; 
    }

    switch (context->commandMode)
    {
        case COMMAND_TARGET_POINT_SEQUENTIAL:
            if (_cmd[0] == '>' || _cmd[0] == '&')
            {
                // >j0(地轨),j1~j6(关节),j7(夹爪),speed
                float joints[6];
                float j7 = 0.0f;
                float speed = 0.0f;

                argNum = sscanf(_cmd.c_str(), (_cmd[0] == '>') ?
                                ">%f,%f,%f,%f,%f,%f,%f,%f" : "&%f,%f,%f,%f,%f,%f,%f,%f",
                                joints, joints+1, joints+2, joints+3, joints+4, joints+5, &j7, &speed);

                if (argNum == 8) context->SetJointSpeed(speed);
                if (argNum >= 7)
                {
                    if (context->MoveJ(joints[0], joints[1], joints[2],
                                   joints[3], joints[4], joints[5], j7))
                    {
                        context->MoveJoints(context->targetJoints);
                        context->MoveRail(context->targetRailPos);

                        while (context->IsMoving() && context->IsEnabled())
                            osDelay(5);

                        Respond(*usbStreamOutputPtr,  "ok");
                        Respond(*uart4StreamOutputPtr, "ok");
                    }
                    else
                    {
                        Respond(*usbStreamOutputPtr,  "error: joint out of limits"); Respond(*uart4StreamOutputPtr, "error: joint out of limits");
                    }
                }
            }
            else if (_cmd[0] == '@')
            {
                float pose[6], speed;
                argNum = sscanf(_cmd.c_str(), "@%f,%f,%f,%f,%f,%f,%f", pose, pose+1, pose+2, pose+3, pose+4, pose+5, &speed);
                if (argNum == 7) context->SetJointSpeed(speed);
                if (argNum >= 6)
                {
                    if (context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]))
                    {
                        while (context->IsMoving() && context->IsEnabled()) osDelay(5);
                        Respond(*usbStreamOutputPtr,  "ok"); Respond(*uart4StreamOutputPtr, "ok");
                    }
                    else
                    {
                        Respond(*usbStreamOutputPtr,  "error: IK fail or out of limits"); Respond(*uart4StreamOutputPtr, "error: IK fail or out of limits");
                    }
                }
            }
            break;

        case COMMAND_CONTINUES_TRAJECTORY:
            if (_cmd[0] == '>' || _cmd[0] == '&')
            {
                // >j0(地轨),j1~j6(关节),j7(夹爪),speed
                float joints[6];
                float j7 = 0.0f;
                float speed = 0.0f;

                argNum = sscanf(_cmd.c_str(), (_cmd[0] == '>') ?
                                ">%f,%f,%f,%f,%f,%f,%f,%f" : "&%f,%f,%f,%f,%f,%f,%f,%f",
                                joints, joints+1, joints+2, joints+3, joints+4, joints+5, &j7, &speed);

                if (argNum == 8) context->SetJointSpeed(speed);
                if (argNum >= 7)
                {
                    if (context->MoveJ(joints[0], joints[1], joints[2],
                                   joints[3], joints[4], joints[5], j7))
                    {
                        context->MoveJoints(context->targetJoints);
                        context->MoveRail(context->targetRailPos);

                        Respond(*usbStreamOutputPtr,  "ok");
                        Respond(*uart4StreamOutputPtr, "ok");
                    }
                    else
                    {
                        Respond(*usbStreamOutputPtr,  "error: joint out of limits"); Respond(*uart4StreamOutputPtr, "error: joint out of limits");
                    }
                }
            }
            else if (_cmd[0] == '@')
            {
                float pose[6], speed;
                argNum = sscanf(_cmd.c_str(), "@%f,%f,%f,%f,%f,%f,%f", pose, pose+1, pose+2, pose+3, pose+4, pose+5, &speed);
                if (argNum == 7) context->SetJointSpeed(speed);
                if (argNum >= 6)
                {
                    if (context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]))
                    {
                        Respond(*usbStreamOutputPtr,  "ok"); Respond(*uart4StreamOutputPtr, "ok");
                    }
                    else
                    {
                        Respond(*usbStreamOutputPtr,  "error: IK fail or out of limits"); Respond(*uart4StreamOutputPtr, "error: IK fail or out of limits");
                    }
                }
            }
            break;

        case COMMAND_TARGET_POINT_INTERRUPTABLE:
            if (_cmd[0] == '>' || _cmd[0] == '&')
            {
                // >j0(地轨),j1~j6(关节),j7(夹爪),speed
                float joints[6];
                float j7 = 0.0f;
                float speed = 0.0f;

                argNum = sscanf(_cmd.c_str(), (_cmd[0] == '>') ?
                                ">%f,%f,%f,%f,%f,%f,%f,%f" : "&%f,%f,%f,%f,%f,%f,%f,%f",
                                joints, joints+1, joints+2, joints+3, joints+4, joints+5, &j7, &speed);

                if (argNum == 8) context->SetJointSpeed(speed);
                if (argNum >= 7)
                {
                    if (context->MoveJ(joints[0], joints[1], joints[2],
                                   joints[3], joints[4], joints[5], j7))
                    {
                        Respond(*usbStreamOutputPtr,  "ok");
                        Respond(*uart4StreamOutputPtr, "ok");
                    }
                    else
                    {
                        Respond(*usbStreamOutputPtr,  "error: joint out of limits"); Respond(*uart4StreamOutputPtr, "error: joint out of limits");
                    }
                }
            }
            else if (_cmd[0] == '@')
            {
                float pose[6], speed;
                argNum = sscanf(_cmd.c_str(), "@%f,%f,%f,%f,%f,%f,%f", pose, pose+1, pose+2, pose+3, pose+4, pose+5, &speed);
                
                ClearFifo(); 

                if (argNum == 7) context->SetJointSpeed(speed);
                if (argNum >= 6)
                {
                    if (context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]))
                    {
                        Respond(*usbStreamOutputPtr,  "ok"); Respond(*uart4StreamOutputPtr, "ok");
                    }
                    else
                    {
                        Respond(*usbStreamOutputPtr,  "error: IK fail or out of limits"); Respond(*uart4StreamOutputPtr, "error: IK fail or out of limits");
                    }
                }
            }
            break;

        case COMMAND_MOTOR_TUNING:
            break;

        case COMMAND_TORQUE_CONTROL:
            break;
        case COMMAND_SERVO_J:
            if (_cmd[0] == '>' || _cmd[0] == '&')
            {
                // >j0(地轨),j1~j6(关节),j7(夹爪)
                float joints[6];
                float j7 = 0.0f;
                argNum = sscanf(_cmd.c_str(), (_cmd[0] == '>') ?
                                ">%f,%f,%f,%f,%f,%f,%f" : "&%f,%f,%f,%f,%f,%f,%f",
                                joints, joints+1, joints+2, joints+3, joints+4, joints+5, &j7);

                if (argNum >= 7)
                {
                    if (context->ServoJ(joints[0], joints[1], joints[2], joints[3], joints[4], joints[5], j7))
                    {
                        context->MoveJoints(context->targetJoints);
                        context->MoveRail(context->targetRailPos);
                    }
                }
            }
            break;
    }
    return osMessageQueueGetSpace(commandFifo);
}

/**
 * @brief 拔除一切残留滞后推演堆栈队列强行清仓置空
 */
void DummyRobot::CommandHandler::ClearFifo()
{
    osMessageQueueReset(commandFifo);
}

/**
 * @brief 指向性赋权系统测试标志位
 */
void DummyRobot::TuningHelper::SetTuningFlag(uint8_t _flag)
{
    tuningFlag = _flag;
}

/**
 * @brief 通过给定时差演进数学周期以构建连续正弦振幅测试波
 */
void DummyRobot::TuningHelper::Tick(uint32_t _timeMillis)
{
    time += (float)M_PI * 2.0f * frequency * (float)_timeMillis / 1000.0f;
    time = fmodf(time, (float)M_PI * 2.0f);

    float delta = amplitude * sinf(time);

    for (int i = 1; i <= 6; i++)
        if (tuningFlag & (1 << (i - 1)))
            context->motorJ[i]->SetAngle(delta);
}

/**
 * @brief 在受控界限中重新配置发波发生器震幅和振频
 */
void DummyRobot::TuningHelper::SetFreqAndAmp(float _freq, float _amp)
{
    if (_freq > 5)         _freq = 5;
    else if (_freq < 0.1f) _freq = 0.1f;
    if (_amp > 50)         _amp = 50;
    else if (_amp < 1)     _amp = 1;

    frequency = _freq;
    amplitude = _amp;
}

/**
 * @brief 用于高速模式七维动力透传执行封装调用模块
 * @param c0: 地轨电流 (A), c1~c6: 臂关节电流 (A), c7: 夹爪电流 (A, 由!HAND_I单独控制，此参数填0)
 */
void DummyRobot::SetJointCurrents(float c0, float c1, float c2, float c3, float c4, float c5, float c6, float c7)
{
    targetCurrents[0] = c1;
    targetCurrents[1] = c2;
    targetCurrents[2] = c3;
    targetCurrents[3] = c4;
    targetCurrents[4] = c5;
    targetCurrents[5] = c6;
    targetRailCurrent = c0;

    if (!isEnabled)
        SetEnable(true);

    for (int i = 1; i <= 6; i++)
        motorJ[i]->SetCurrentSetPoint(targetCurrents[i - 1]);
    motorJ[0]->SetCurrentSetPoint(targetRailCurrent);  // 地轨电流单独下发
}
