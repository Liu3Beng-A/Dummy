#include "communication.hpp"
#include "dummy_robot.h"
#include "time_utils.h"
#include <cstring>
#include <cmath>

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
 * @note 构建完备系统骨骼：其中 0 标识群广播指令信道，1 至 6 分管大臂到腕部的独立回环执行端，
 *       并依据真实机械骨架标定了各自的操作限位极性，同时载具配置末端特殊夹取执行器。
 */
DummyRobot::DummyRobot(CAN_HandleTypeDef* _hcan) :
    hcan(_hcan)
{
    motorJ[ALL] = new CtrlStepMotor(_hcan, 0, false, 1, -180, 180);

    motorJ[1]   = new CtrlStepMotor(_hcan, 1, false, 50, -175, 175);
    motorJ[2]   = new CtrlStepMotor(_hcan, 2, true,  50,  -75,  90);
    motorJ[3]   = new CtrlStepMotor(_hcan, 3, true,  50,    0, 180);
    motorJ[4]   = new CtrlStepMotor(_hcan, 4, true,  50, -270, 270);
    motorJ[5]   = new CtrlStepMotor(_hcan, 5, true,  50, -100, 100);
    motorJ[6]   = new CtrlStepMotor(_hcan, 6, true,  30, -180, 180);

    hand = new StepHand(_hcan, 7);

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
        
        if (config.rgbStateStart <= 9)   rgbStateStart = config.rgbStateStart;
        if (config.rgbStateEnable <= 9)  rgbStateEnable = config.rgbStateEnable;
        if (config.rgbStateDisable <= 9) rgbStateDisable = config.rgbStateDisable;

        if (config.initialJointSpeed >= 1.0f && config.initialJointSpeed <= 100.0f)
            initialJointSpeed = config.initialJointSpeed;
        if (config.homeSpeed >= 0.0f && config.homeSpeed <= 100.0f)
            homeSpeed = config.homeSpeed;
    }
    else
    {
        SaveConfig();
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
    config.rgbStateStart = rgbStateStart;
    config.rgbStateEnable = rgbStateEnable;
    config.rgbStateDisable = rgbStateDisable;

    config.initialJointSpeed = initialJointSpeed;
    config.homeSpeed = homeSpeed;

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

    SetRGBEnabled(true);
    SetRGBMode(rgbStateStart);
    SetJointSpeed(initialJointSpeed);   // 使用保存的初始速度
    SetJointAcceleration(100);          // 先加载保存的各关节加速度基准值
    SetCommandMode(DEFAULT_COMMAND_MODE);  // 最后切模式，让模式生效（如默认 5% 加速度），避免上电后第一次 Home 过快
    trajPlanner.SetKinematicSolver(dof6Solver);
}

/**
 * @brief 群发全局急停保护且指令微控制器彻底脱壳软重启
 */
void DummyRobot::Reboot()
{
    motorJ[ALL]->Reboot();
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
 * @brief 解析空间六维坐标并令其映射入安全界域内化为电机目标偏角实现平稳直线位移
 * @param _x, _y, _z 工作空间末端探针位置参考系 (标准计度)
 * @param _a, _b, _c 空间内姿态偏转对应四元欧拉角反算值
 * @return 布尔反馈代表其能否在有限的运动机能与逆求解内完成安全收敛响应
 */
bool DummyRobot::MoveL(float _x, float _y, float _z, float _a, float _b, float _c)
{
    DOF6Kinematic::Pose6D_t startPose;
    dof6Solver->SolveFK(lastCmdJoints, startPose);
    startPose.X *= 1000.0f;
    startPose.Y *= 1000.0f;
    startPose.Z *= 1000.0f;

    DOF6Kinematic::Pose6D_t targetPose(_x, _y, _z, _a, _b, _c);

    // 动态融合策略：如果当前指令附带了合法的速度，优先使用局部速度；否则回退使用全局速度
    float target_v_max = (this->jointSpeed > 0.0f) ? this->jointSpeed : this->globalSpeed;
    float a_max = this->globalAcc;
    float j_max = (this->globalJerk > 0.0f) ? this->globalJerk : (a_max * 15.0f);

    if (trajPlanner.StartMoveL(startPose, targetPose, target_v_max, a_max, j_max))
        return true;
    return false;
}

/**
 * @brief 向定点旋转并发规划驱动组群下属协同运转指令
 * @note 使用 TrajectoryGenerator 集中式 S 型规划，由 TickTrajectory 周期性下发位置
 */
bool DummyRobot::MoveJ(float _j1, float _j2, float _j3, float _j4, float _j5, float _j6)
{
    DOF6Kinematic::Joint6D_t target(_j1, _j2, _j3, _j4, _j5, _j6);
    for (int j = 1; j <= 6; j++)
    {
        if (target.a[j - 1] > motorJ[j]->angleLimitMax ||
            target.a[j - 1] < motorJ[j]->angleLimitMin)
            return false;
    }
    // 动态融合策略：如果当前指令附带了合法的速度，优先使用局部速度；否则回退使用全局速度
    float target_v_max = (this->jointSpeed > 0.0f) ? this->jointSpeed : this->globalSpeed;
    float a_max = this->globalAcc;
    float j_max = (this->globalJerk > 0.0f) ? this->globalJerk : (a_max * 15.0f);
    // 轨迹起点使用上一帧理论指令 lastCmdJoints，避免物理反馈滞后导致起点脱节与首帧跳变
    if (!trajPlanner.StartMoveJ(lastCmdJoints, target, target_v_max, a_max, j_max))
        return false;
    targetJoints = target;
    return true;
}
 
/**
 * @brief 无阻塞高通量前馈跟随驱动随动策略
 * @note 基于微秒精度的指令脉冲插补微分求导实时计算需求速度完成极速响应映射闭环跟踪
 */
 bool DummyRobot::ServoJ(float _j1, float _j2, float _j3, float _j4, float _j5, float _j6)
 {
     DOF6Kinematic::Joint6D_t targetJointsTmp(_j1, _j2, _j3, _j4, _j5, _j6);
 
     for (int j = 1; j <= 6; j++)
     {
         if (targetJointsTmp.a[j - 1] > motorJ[j]->angleLimitMax ||
             targetJointsTmp.a[j - 1] < motorJ[j]->angleLimitMin)
             return false;
     }
 
     uint32_t nowUs = micros();
     float dt = (nowUs - lastServoTime) / 1000000.0f;
     if (dt <= 0.001f) return true;
     lastServoTime = nowUs;

     for (int j = 1; j <= 6; j++)
     {
         float deltaAngle = fabsf(targetJointsTmp.a[j - 1] - currentJoints.a[j - 1]);
         float reqSpeed   = deltaAngle / dt * 1.5f; 
         
         dynamicJointSpeeds.a[j - 1] = (reqSpeed < 0.05f)   ? 0.05f   : 
                                       (reqSpeed > 200.0f) ? 200.0f : reqSpeed;
     }
 
     targetJoints = targetJointsTmp;
     jointsStateFlag = 0;
 
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

        if (motorJ[i]->state == CtrlStepMotor::FINISH)
            jointsStateFlag |= (1 << i);
        else
            jointsStateFlag &= ~(1 << i);
    }
}

/**
 * @brief 分配调准基准速率运行档位
 */
void DummyRobot::SetJointSpeed(float _speed)
{
    if (_speed < 0)        _speed = 0;
    else if (_speed > 500) _speed = 500;

    jointSpeed = _speed * jointSpeedRatio;
}

/**
 * @brief 基于底层参数配给换算映射应用新加速度限制表尺
 * @note  轨迹模式下用软加速度 (150) 作为低通滤波平滑 200Hz 阶跃；非轨迹模式放宽至 3000
 */
void DummyRobot::SetJointAcceleration(float _acc)
{
    if (_acc < 0)        _acc = 0;
    else if (_acc > 100) _acc = 100;

    currentAccelerationPercent = _acc;
    bool isTrajectoryMode = (commandMode == COMMAND_TARGET_POINT_SEQUENTIAL ||
                             commandMode == COMMAND_TARGET_POINT_INTERRUPTABLE ||
                             commandMode == COMMAND_CONTINUES_TRAJECTORY);
    float driverAcc = isTrajectoryMode ? 150.0f : 3000.0f;
    for (int i = 1; i <= 6; i++) {
        motorJ[i]->SetAcceleration(driverAcc);
    }
}

/**
 * @brief 设置并保存初始速度，断电后上电将使用该值
 */
void DummyRobot::SetInitialJointSpeed(float _speed)
{
    if (_speed < 0)        _speed = 0;
    else if (_speed > 100) _speed = 100;
    initialJointSpeed = _speed;
    jointSpeed        = _speed * jointSpeedRatio;
    SaveConfig();
}

/**
 * @brief 设置并保存初始位置到 Home 的回零速度，断电后上电执行 !HOME 时使用该值
 */
void DummyRobot::SetHomeSpeed(float _speed)
{
    if (_speed < 0)        _speed = 0;
    else if (_speed > 100) _speed = 100;
    homeSpeed = _speed;
    SaveConfig();
}

/**
 * @brief 发送节点通断电流源动力配置及 RGB 等附属工作展示配合转换
 */
void DummyRobot::SetEnable(bool _enable)
{
    SetRGBEnabled(true);

    if (_enable)
    {
        isEStopped = false;
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

    motorJ[ALL]->SetEnable(_enable);
    isEnabled = _enable;
}

/**
 * @brief 确认灯效外设启用状态
 */
bool DummyRobot::GetRGBEnabled() const
{
    return isRGBEnabled;
}

/**
 * @brief 获取映射渲染花式编号
 */
uint32_t DummyRobot::GetRGBMode() const
{
    return rgbMode;
}

/**
 * @brief 允许控制或强裁下层灯光管脚功能运转
 */
void DummyRobot::SetRGBEnabled(bool enable)
{
    isRGBEnabled = enable;
}

/**
 * @brief 转场切换工作气氛及提示预设闪灯灯位色彩逻辑
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
 * @brief 利用并验核位标志位侦测判定是否各级机组完成定位收敛平息抖动
 */
/**
 * @brief RTOS 定时器高频调用：推进轨迹插补并向下发位置指令
 * @note  主控 S 曲线直通模式 (0x08)，跳过电机内部梯形规划，由 TickTrajectory 统一控制加减速
 */
void DummyRobot::TickTrajectory(float dt)
{
    if (isEStopped) return;

    if (!isEnabled) {
        trajPlanner.Stop();
        lastCmdJoints = currentJoints;
        return;
    }

    // P0：轮询 6 轴 + 夹爪 errorCode，堵转(1) 或 急停(4) 时立即停轨迹并失能
    for (int i = 1; i <= 6; i++) {
        uint8_t ec = motorJ[i]->errorCode;
        if (ec == 1 || ec == 4) {
            trajPlanner.Stop();
            lastCmdJoints = currentJoints;
            isEnabled = false;
            Respond(*uart4StreamOutputPtr, "[warning] axis %d fault (errorCode=%d)\n", i, (int)ec);
            Respond(*usbStreamOutputPtr,  "[warning] axis %d fault (errorCode=%d)\n", i, (int)ec);
            return;
        }
    }
    {
        uint8_t ec = hand->errorCode;
        if (ec == 1 || ec == 4) {
            trajPlanner.Stop();
            lastCmdJoints = currentJoints;
            isEnabled = false;
            Respond(*uart4StreamOutputPtr, "[warning] hand fault (errorCode=%d)\n", (int)ec);
            Respond(*usbStreamOutputPtr,  "[warning] hand fault (errorCode=%d)\n", (int)ec);
            return;
        }
    }

    if (!trajPlanner.IsMoving()) {
        lastCmdJoints = currentJoints;
        return;
    }

    bool was_moving = trajPlanner.IsMoving();
    DOF6Kinematic::Joint6D_t out_joints = lastCmdJoints;

    if (!trajPlanner.Update(dt, out_joints)) {
        if (was_moving) {
            Respond(*uart4StreamOutputPtr, "[error] IK singularity/solution jump detected - trajectory aborted\n");
            Respond(*usbStreamOutputPtr,  "[error] IK singularity/solution jump detected - trajectory aborted\n");

            trajPlanner.Stop();
            commandHandler.ClearFifo();

            lastCmdJoints = currentJoints;
            targetJoints = currentJoints;

            Respond(*uart4StreamOutputPtr, "[info] State reset: planner stopped, FIFO cleared, returning to IDLE\n");
            Respond(*usbStreamOutputPtr,  "[info] State reset: planner stopped, FIFO cleared, returning to IDLE\n");
        }
        return;
    }

    // 主控 S 曲线直通模式 (0x08)，跳过电机内部梯形规划，由 TickTrajectory 统一下发
    for (int i = 1; i <= 6; i++) {
        float target_angle = out_joints.a[i - 1] - initPose.a[i - 1];
        float stepMotorCnt = target_angle / 360.0f * (float)motorJ[i]->reduction;
        motorJ[i]->SetPositionDirect(stepMotorCnt);
    }
    lastCmdJoints = out_joints;
}

/**
 * @brief 反馈外围调配安全控制开关当前情况
 */
/**
 * @brief 进行动力指令响应分发机制转盘的切入与挂载新特例算法配置的套用执行
 */
void DummyRobot::SetCommandMode(uint32_t _mode)
{
    if (_mode < COMMAND_TARGET_POINT_SEQUENTIAL ||
        _mode > COMMAND_TORQUE_CONTROL)
        return;

    CommandMode newMode = static_cast<CommandMode>(_mode);

    if (commandMode != newMode)
    {
        commandHandler.ClearFifo();
        trajPlanner.Stop();
        lastCmdJoints = currentJoints;
        targetJoints = currentJoints;
    }

    commandMode = newMode;

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
    if (_cmd.length() >= 128) {
        return 0xFF;
    }
    char buf[128] = {0};
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

    CtrlStepMotor::BroadcastEmergencyStop(context->hcan);
    context->isEStopped = true;
    context->isEnabled = false;
    ClearFifo();
}

/**
 * @brief 在队列排布端向外吐出封存任务项
 */
const char* DummyRobot::CommandHandler::Pop(uint32_t timeout)
{
    osStatus_t status = osMessageQueueGet(commandFifo, strBuffer, nullptr, timeout);
    if (status == osOK)
        return strBuffer;
    return "";
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
uint32_t DummyRobot::CommandHandler::ParseCommand(const char* _cmd)
{
    uint8_t argNum;

    if (_cmd[0] == '\0')
        return osMessageQueueGetSpace(commandFifo);

    // [分支拦截] $ 高频投递的力控透传包剥离拦截，规避无意义繁杂判决延宕
    if (_cmd[0] == '$')
    {
        float cur[6];
        argNum = sscanf(_cmd, "$%f,%f,%f,%f,%f,%f",
                        &cur[0], &cur[1], &cur[2], &cur[3], &cur[4], &cur[5]);

        if (argNum == 6)
        {
            if (context->commandMode != COMMAND_TORQUE_CONTROL)
                context->SetCommandMode(COMMAND_TORQUE_CONTROL);

            context->SetJointCurrents(cur[0], cur[1], cur[2], cur[3], cur[4], cur[5]);
        }
        return osMessageQueueGetSpace(commandFifo);
    }

    float paramVal;
    if (sscanf(_cmd, "#SPEED %f", &paramVal) == 1) {
        if (paramVal >= 0.0f && paramVal <= 500.0f) {
            context->globalSpeed = paramVal;
        }
        return osMessageQueueGetSpace(commandFifo);
    }
    if (sscanf(_cmd, "#ACC %f", &paramVal) == 1) {
        if (paramVal >= 0.0f && paramVal <= 5000.0f) {
            context->globalAcc = paramVal;
        }
        return osMessageQueueGetSpace(commandFifo);
    }
    if (sscanf(_cmd, "#JERK %f", &paramVal) == 1) {
        if (paramVal >= 0.0f && paramVal <= 50000.0f) {
            context->globalJerk = paramVal;
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
                float joints[6];
                float speed = 0.0f;

                argNum = sscanf(_cmd, (_cmd[0] == '>') ?
                                ">%f,%f,%f,%f,%f,%f,%f" : "&%f,%f,%f,%f,%f,%f,%f",
                                joints, joints+1, joints+2, joints+3, joints+4, joints+5, &speed);

                if (argNum == 7) context->SetJointSpeed(speed);
                if (argNum >= 6)
                {
                    if (context->MoveJ(joints[0], joints[1], joints[2],
                                   joints[3], joints[4], joints[5]))
                    {
                        // 1. 等待主控 S 曲线轨迹插补完成
                        while (context->IsMoving() && !context->isEStopped && context->IsEnabled()) {
                            osDelay(5);
                        }
                        // 2. 等待 6 个物理电机全部回传 FINISH 标志 (1~6轴全到位时 jointsStateFlag 的低 1~6 位全为 1，即 0x7E)
                        while ((context->jointsStateFlag & 0x7E) != 0x7E && !context->isEStopped && context->IsEnabled()) {
                            osDelay(5);
                        }
                        // 3. 闭环确认：判断是正常完成还是因为故障被中断
                        if (!context->isEStopped && context->IsEnabled()) {
                            Respond(*usbStreamOutputPtr,  "ok Move [Speed: %.1f]", speed);
                            Respond(*uart4StreamOutputPtr, "ok Move [Speed: %.1f]", speed);
                        } else {
                            Respond(*usbStreamOutputPtr,  "error: move aborted due to fault");
                            Respond(*uart4StreamOutputPtr, "error: move aborted due to fault");
                        }
                    }
                    else
                    {
                        Respond(*usbStreamOutputPtr,  "error: joint out of limits"); Respond(*uart4StreamOutputPtr, "error: joint out of limits");
                    }
                }
            }
            else if (_cmd[0] == '@')
            {
                float pose[6], speed = 0.0f;
                argNum = sscanf(_cmd, "@%f,%f,%f,%f,%f,%f,%f", pose, pose+1, pose+2, pose+3, pose+4, pose+5, &speed);
                if (argNum == 7) context->SetJointSpeed(speed);
                if (argNum >= 6)
                {
                    if (context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]))
                    {
                        // 1. 等待主控 S 曲线轨迹插补完成
                        while (context->IsMoving() && !context->isEStopped && context->IsEnabled()) {
                            osDelay(5);
                        }
                        // 2. 等待 6 个物理电机全部回传 FINISH 标志 (1~6轴全到位时 jointsStateFlag 的低 1~6 位全为 1，即 0x7E)
                        while ((context->jointsStateFlag & 0x7E) != 0x7E && !context->isEStopped && context->IsEnabled()) {
                            osDelay(5);
                        }
                        // 3. 闭环确认：判断是正常完成还是因为故障被中断
                        if (!context->isEStopped && context->IsEnabled()) {
                            Respond(*usbStreamOutputPtr,  "ok Move [Speed: %.1f]", speed);
                            Respond(*uart4StreamOutputPtr, "ok Move [Speed: %.1f]", speed);
                        } else {
                            Respond(*usbStreamOutputPtr,  "error: move aborted due to fault");
                            Respond(*uart4StreamOutputPtr, "error: move aborted due to fault");
                        }
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
                float joints[6];
                float speed = 0.0f;

                argNum = sscanf(_cmd, (_cmd[0] == '>') ?
                                ">%f,%f,%f,%f,%f,%f,%f" : "&%f,%f,%f,%f,%f,%f,%f",
                                joints, joints+1, joints+2, joints+3, joints+4, joints+5, &speed);

                if (argNum == 7) context->SetJointSpeed(speed);
                if (argNum >= 6)
                {
                    if (context->MoveJ(joints[0], joints[1], joints[2],
                                   joints[3], joints[4], joints[5]))
                    {
                        Respond(*usbStreamOutputPtr,  "ok Move [Speed: %.1f]", speed);
                        Respond(*uart4StreamOutputPtr, "ok Move [Speed: %.1f]", speed);
                    }
                    else
                    {
                        Respond(*usbStreamOutputPtr,  "error: joint out of limits"); Respond(*uart4StreamOutputPtr, "error: joint out of limits");
                    }
                }
            }
            else if (_cmd[0] == '@')
            {
                float pose[6], speed = 0.0f;
                argNum = sscanf(_cmd, "@%f,%f,%f,%f,%f,%f,%f", pose, pose+1, pose+2, pose+3, pose+4, pose+5, &speed);
                if (argNum == 7) context->SetJointSpeed(speed);
                if (argNum >= 6)
                {
                    if (context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]))
                    {
                        Respond(*usbStreamOutputPtr,  "ok Move [Speed: %.1f]", speed);
                        Respond(*uart4StreamOutputPtr, "ok Move [Speed: %.1f]", speed);
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
                float joints[6];
                float speed = 0.0f;

                argNum = sscanf(_cmd, (_cmd[0] == '>') ?
                                ">%f,%f,%f,%f,%f,%f,%f" : "&%f,%f,%f,%f,%f,%f,%f",
                                joints, joints+1, joints+2, joints+3, joints+4, joints+5, &speed);

                if (argNum == 7) context->SetJointSpeed(speed);
                if (argNum >= 6)
                {
                    if (context->MoveJ(joints[0], joints[1], joints[2],
                                   joints[3], joints[4], joints[5]))
                    {
                        Respond(*usbStreamOutputPtr,  "ok Move [Speed: %.1f]", speed);
                        Respond(*uart4StreamOutputPtr, "ok Move [Speed: %.1f]", speed);
                    }
                    else
                    {
                        Respond(*usbStreamOutputPtr,  "error: joint out of limits"); Respond(*uart4StreamOutputPtr, "error: joint out of limits");
                    }
                }
            }
            else if (_cmd[0] == '@')
            {
                float pose[6], speed = 0.0f;
                argNum = sscanf(_cmd, "@%f,%f,%f,%f,%f,%f,%f", pose, pose+1, pose+2, pose+3, pose+4, pose+5, &speed);
                
                ClearFifo(); 

                if (argNum == 7) context->SetJointSpeed(speed);
                if (argNum >= 6)
                {
                    if (context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]))
                    {
                        Respond(*usbStreamOutputPtr,  "ok Move [Speed: %.1f]", speed);
                        Respond(*uart4StreamOutputPtr, "ok Move [Speed: %.1f]", speed);
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
                float joints[6];
                argNum = sscanf(_cmd, (_cmd[0] == '>') ?
                                ">%f,%f,%f,%f,%f,%f" : "&%f,%f,%f,%f,%f,%f",
                                joints, joints+1, joints+2, joints+3, joints+4, joints+5);

                if (argNum >= 6)
                {
                    if (context->ServoJ(joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]))
                    {
                        context->MoveJoints(context->targetJoints);
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
 * @brief 用于高速模式六维动力透传执行封装调用模块
 */
void DummyRobot::SetJointCurrents(float c1, float c2, float c3, float c4, float c5, float c6)
{
    targetCurrents[0] = c1;
    targetCurrents[1] = c2;
    targetCurrents[2] = c3;
    targetCurrents[3] = c4;
    targetCurrents[4] = c5;
    targetCurrents[5] = c6;

    if (!isEnabled)
        SetEnable(true);

    for (int i = 1; i <= 6; i++)
        motorJ[i]->SetCurrentSetPoint(targetCurrents[i - 1]);
}

/**
 * @brief 回到初始零点姿态 (安全降速)
 */
 void DummyRobot::Homing()
 {
     // 备份当前的工作速度
     float temp_speed = this->jointSpeed;
     // 临时切换为 EEPROM 中保存的安全回零速度
     this->jointSpeed = this->homeSpeed * this->jointSpeedRatio;
     
     // 调用 S 曲线规划器执行 MoveJ
     MoveJ(HOME_POSE.a[0], HOME_POSE.a[1], HOME_POSE.a[2],
           HOME_POSE.a[3], HOME_POSE.a[4], HOME_POSE.a[5]);
           
     // 恢复原来的工作速度
     this->jointSpeed = temp_speed; 
 }
 
 /**
  * @brief 回到休眠复位姿态 (安全降速)
  */
 void DummyRobot::Resting()
 {
     float temp_speed = this->jointSpeed;
     this->jointSpeed = this->homeSpeed * this->jointSpeedRatio;
     
     MoveJ(REST_POSE.a[0], REST_POSE.a[1], REST_POSE.a[2],
           REST_POSE.a[3], REST_POSE.a[4], REST_POSE.a[5]);
           
     this->jointSpeed = temp_speed;
 }