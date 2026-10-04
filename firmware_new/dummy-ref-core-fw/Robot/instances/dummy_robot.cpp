#include "communication.hpp"
#include "dummy_robot.h"
#include <cstring>

// Release 使用 -Ofast，按 IEEE-754 位模式检查，避免 isfinite 被优化掉。
static bool IsFiniteCommandValue(float value)
{
    uint32_t bits;
    static_assert(sizeof(bits) == sizeof(value), "32-bit float required");
    std::memcpy(&bits, &value, sizeof(bits));
    return (bits & 0x7f800000U) != 0x7f800000U;
}

inline float AbsMaxOf6(DOF6Kinematic::Joint6D_t _joints, uint8_t &_index)
{
    float max = -1;
    for (uint8_t i = 0; i < 6; i++)
    {
        if (abs(_joints.a[i]) > max)
        {
            max = abs(_joints.a[i]);
            _index = i;
        }
    }

    return max;
}


/**
 * @brief §3.3.1 单轴梯形速度曲线总时间（v2.7，与电机端 motion_planner.cpp PositionTracker 对齐）
 * @param d 距离（电机轴圈数）
 * @param v 目标速度（匀速段速度，r/s）
 * @param a 加速度（r/s²）
 * @return 该轴按梯形曲线跑 d 距离的总时间（s）
 * @note  距离太短（2*d_acc >= d）时退化为三角曲线：T = 2*sqrt(d/a)
 */
static float timeTrapezoid(float d, float v, float a)
{
    if (d < 1e-6f || v < 1e-6f || a < 1e-6f) return 0;
    float d_acc = v * v / (2.0f * a);  // 单边加速距离
    if (2.0f * d_acc >= d) {
        // 三角形：距离太短，没匀速段
        return 2.0f * sqrtf(d / a);
    }
    // 梯形：T = d/v + v/a，避免在临界距离附近做相近数相减。
    return d / v + v / a;
}


/**
 * @brief 7 轴同步抵达速度规划（固定最大时间预算 + 稳定小根）
 * @param deltaRails[7] 各轴距离，[0]=地轨(mm)，[1..6]=关节(°)
 * @param sliderCaps[7] 各轴电机轴 r/s 上限（地轨=30，关节=30）
 * @param accel[7]      各轴加速度（r/s²），0 用 JOINT_ACC_FALLBACK 兜底
 * @param sliderSpeed   统一基础速度（slider × 0.30，r/s）
 * @param outSpeeds[7]  输出每轴最终电机轴 r/s
 * @return timeBudget   同步抵达总时间（s）
 * @note  与电机端 motion_planner.cpp PositionTracker 公式一致：对称梯形加减速，
 *        距离太短退化为三角形曲线。运行时加速度通过 CAN 回读，
 *        未回包前用 JOINT_ACC_FALLBACK[i] 兜底。
 */
static float ComputeSyncSpeeds(const float deltaRails[7], const float sliderCaps[7],
                               const float accel[7], float sliderSpeed,
                               float outSpeeds[7])
{
    // 步骤 1：各轴距离 → 电机轴圈数
    float distMotor[7];
    for (int i = 0; i < 7; i++) {
        distMotor[i] = (i == 0)
            ? fabsf(deltaRails[i]) / RAIL_MM_PER_CIRCLE
            : fabsf(deltaRails[i]) * (float)MOTOR_REDUCTION[i] / 360.0f;
    }

    // 步骤 2：固定预算 = 各轴在用户/物理限速下的最短梯形时间，取 max。
    float timeBudget = 0;
    for (int i = 0; i < 7; i++) {
        if (distMotor[i] < 1e-6f) continue;
        float a = (accel[i] > 1e-6f) ? accel[i] : JOINT_ACC_FALLBACK[i];
        float v = fminf(sliderSpeed, sliderCaps[i]);
        float t = timeTrapezoid(distMotor[i], v, a);
        if (t > timeBudget) timeBudget = t;
    }

    if (timeBudget < 1e-6f) {
        // 所有轴距离都 ≈ 0，不动
        for (int i = 0; i < 7; i++) outSpeeds[i] = 0;
        return 0.0f;
    }

    // 步骤 3：用同一个预算直接反解各轴速度，无需重新反算预算。
    // v²/a - v*T + d = 0 的小根有理化后：v = 2d / (T + sqrt(T² - 4d/a))。
    // 避免 T-sqrt(...) 的相减消去，尤其是“不动轴”的微小编码器残差。
    // 不再用反解后的浮点误差放大 T，保证小残差不拖慢其他运动轴。
    for (int i = 0; i < 7; i++) {
        if (distMotor[i] < 1e-6f) {
            outSpeeds[i] = 0;
            continue;
        }
        float a = (accel[i] > 1e-6f) ? accel[i] : JOINT_ACC_FALLBACK[i];
        float disc = timeBudget * timeBudget - 4.0f * distMotor[i] / a;
        // 三角曲线临界处允许浮点舍入造成的小负数。
        if (disc < 0) disc = 0;
        float v = 2.0f * distMotor[i] / (timeBudget + sqrtf(disc));
        // 保持各轴物理上限与用户速度档位上限。
        if (v > sliderCaps[i]) v = sliderCaps[i];
        if (v > sliderSpeed) v = sliderSpeed;
        outSpeeds[i] = v;
    }
    return timeBudget;
}


DummyRobot::DummyRobot(CAN_HandleTypeDef* _hcan) :
    hcan(_hcan)
{
    motorJ[ALL] = new CtrlStepMotor(_hcan, 0, false, 1, -180, 180);
    motorJ[1] = new CtrlStepMotor(_hcan, 1, false, 50, -175, 175);
    motorJ[2] = new CtrlStepMotor(_hcan, 2, true, 50, -75, 90);
    motorJ[3] = new CtrlStepMotor(_hcan, 3, true, 50, 0, 180);
    motorJ[4] = new CtrlStepMotor(_hcan, 4, true, 50, -270, 270);
    motorJ[5] = new CtrlStepMotor(_hcan, 5, true, 50, -100, 100);
    motorJ[6] = new CtrlStepMotor(_hcan, 6, true, 30, -180, 180);
    motorJ[7] = new CtrlStepMotor(_hcan, 7, true, 1, -250, 250);  //  -250/+250 单位是 mm（MoveJ 按 mm 比较），不是°。直连丝杆1605（5mm/圈）

    hand = new StepHand(_hcan, 8);   

    dof6Solver = new DOF6Kinematic(0.165f, 0.0f, 0.170f, 0.117f, 0.0695f, 0.113f);
}


DummyRobot::~DummyRobot()
{
    for (int j = 0; j <= 7; j++)   // [FIX] 0..6 → 0..7（包含地轨）
        delete motorJ[j];

    delete hand;
    delete dof6Solver;
}


void DummyRobot::Init()
{
    SetCommandMode(DEFAULT_COMMAND_MODE);
    SetJointSpeed(DEFAULT_JOINT_SPEED);

    // [v2026-10-03] 启动后异步回填 jointAccRuntime[1..8]（0x2C 回包，~5ms 抵达）
    SyncAllMotorAcceleration();
}


void DummyRobot::Reboot()
{
    motorJ[ALL]->Reboot();
    osDelay(500); // waiting for all joints done
    HAL_NVIC_SystemReset();
}

void DummyRobot::RebootAllMotors()
{
    motorJ[ALL]->Reboot();   // J1~J7（含地轨），1 帧广播
    // 故意不 osDelay：
    //   原 Reboot() 末尾 500ms 是给 HAL_NVIC_SystemReset() 留出发帧时间，
    //   本函数不调 reset，且 CAN 帧已在 µs 级装入 Mailbox（can.c CAN_Send），
    //   无需 block 任何 RTOS 任务。
}

// [v2026-10-03] 查询端补齐：3 Sync + 1 Poll（ASCII ALL 模式内部使用 + OledTask 周期 poll）
void DummyRobot::SyncAllMotorAcceleration()
{
    // 触发 8 个电机发 0x2C 回包（电机端回 float 到 can_protocol ISR）
    for (int i = 1; i <= 7; i++) motorJ[i]->QueryAcceleration();
    if (hand) hand->QueryAcceleration();
}

void DummyRobot::SyncAllMotorCurrentLimit()
{
    // 触发 8 个电机发 0x2D 回包
    for (int i = 1; i <= 7; i++) motorJ[i]->QueryCurrentLimit();
    if (hand) hand->QueryCurrentLimit();
}

void DummyRobot::SyncAllMotorPid()
{
    // [v2026-10-04] 帧间加 osDelay(1)：32 帧错峰 ~32ms，避免撞 5ms 控制环
    // tx_busy_errors 不再累加；200Hz 控制环不掉拍
    for (int i = 1; i <= 7; i++) {
        motorJ[i]->QueryDceKp();
        motorJ[i]->QueryDceKv();
        motorJ[i]->QueryDceKi();
        motorJ[i]->QueryDceKd();
        osDelay(1);
    }
    if (hand) {
        hand->QueryDceKp();
        hand->QueryDceKv();
        hand->QueryDceKi();
        hand->QueryDceKd();
    }
}

void DummyRobot::PollPidPrint()
{
    // OledTask 50Hz 周期调用；扫描 8 节点，4 帧合并完成的 PID 立即 printf 并清 0
    for (int idx = 1; idx <= 8; idx++) {
        if (_pid_print_pending[idx]) {
            _pid_print_pending[idx] = 0;
            uint8_t node = _pid_pending_node[idx];
            if (node == 7) {
                printf("ok PID [7] kp=%ld kv=%ld ki=%ld kd=%ld (rail)\r\n",
                       (long)motorDceKps[idx], (long)motorDceKvs[idx],
                       (long)motorDceKis[idx], (long)motorDceKds[idx]);
            } else if (node == 8) {
                printf("ok PID [8] kp=%ld kv=%ld ki=%ld kd=%ld (HAND)\r\n",
                       (long)motorDceKps[idx], (long)motorDceKvs[idx],
                       (long)motorDceKis[idx], (long)motorDceKds[idx]);
            } else {
                printf("ok PID [%u] kp=%ld kv=%ld ki=%ld kd=%ld\r\n", node,
                       (long)motorDceKps[idx], (long)motorDceKvs[idx],
                       (long)motorDceKis[idx], (long)motorDceKds[idx]);
            }
            motorPidRcvdCount[idx] = 0;  // 重置合并计数器
        }
    }
}

void DummyRobot::MoveJoints(DOF6Kinematic::Joint6D_t _joints)
{
    for (int j = 1; j <= 6; j++)
    {
        motorJ[j]->SetAngleWithVelocityLimit(_joints.a[j - 1] - initPose.a[j - 1],
                                             dynamicJointSpeeds.a[j - 1]);
    }
    // [FIX] 地轨：reduction=1 直连丝杆1605（5mm/圈）。
    // targetRailPos 单位是 mm；motorJ[7]->angleLimitMin/Max 在此处按 **mm** 直接比较
    // （不要按 CtrlStepMotor 默认°语义解读，否则会把 250 当成 250°=6.94mm，误以为软限位过窄）。
    // 下发必须用 SetPositionWithVelocityLimit 直接传电机圈数（mm/5），
    // 否则被 SetAngle* 按°解读，实际走 1/72（100 mm → 1.39 mm）。
    motorJ[7]->SetPositionWithVelocityLimit(targetRailPos / RAIL_MM_PER_CIRCLE,
                                            dynamicJointSpeeds7[0]);
}


bool DummyRobot::MoveJ(float _j1, float _j2, float _j3, float _j4,
                       float _j5, float _j6, float _j7_mm, float _speed)
{
    DOF6Kinematic::Joint6D_t targetJointsTmp(_j1, _j2, _j3, _j4, _j5, _j6);

    // 地轨限位
    if (_j7_mm > motorJ[7]->angleLimitMax || _j7_mm < motorJ[7]->angleLimitMin)
        return false;

    // 关节限位
    bool valid = true;
    for (int j = 1; j <= 6; j++)
    {
        if (targetJointsTmp.a[j - 1] > motorJ[j]->angleLimitMax ||
            targetJointsTmp.a[j - 1] < motorJ[j]->angleLimitMin)
            valid = false;
    }
    if (!valid) return false;

    // 物理同步：固定最大时间预算 + 数值稳定的梯形曲线速度反解。
    // 按用户给定 speed (1~100) 换算电机轴 r/s：100 → 30 r/s
    if (!IsFiniteCommandValue(_speed)) return false;
    // 0 是内部“沿用当前档位”标记；串口助手只允许输入 1..100。
    float slider = _speed == 0 ? jointSpeed : _speed;
    if (slider < 1) slider = 1;
    if (slider > 100) slider = 100;
    float sliderSpeedRps = slider * 0.30f;  // 100 → 30 r/s

    // 各轴目标位移：索引 0=地轨(mm)，1..6=关节(°)
    float deltaRails[7] = {
        _j7_mm - currentRailPos,
        targetJointsTmp.a[0] - currentJoints.a[0],
        targetJointsTmp.a[1] - currentJoints.a[1],
        targetJointsTmp.a[2] - currentJoints.a[2],
        targetJointsTmp.a[3] - currentJoints.a[3],
        targetJointsTmp.a[4] - currentJoints.a[4],
        targetJointsTmp.a[5] - currentJoints.a[5],
    };
    // 各轴加速度：异步回填槽；本期用 JOINT_ACC_FALLBACK 兜底（§9 #7 后续接电机端回填）
    // [v2026-10-03] 索引迁移：[7]→[9]；[0] 仍对齐 deltaRails[0]=地轨
    //   jointAccRuntime[0] 留空（ALL 代理空闲槽）；地轨走 jointAccRuntime[7]
    //   J1~J6 走 jointAccRuntime[1..6]；夹爪走 jointAccRuntime[8]
    float jointAcc[7] = {
        jointAccRuntime[7],  // [0]=地轨（与 deltaRails[0] 对齐，ComputeSyncSpeeds i==0 走 RAIL_MM_PER_CIRCLE 分支）
        jointAccRuntime[1], jointAccRuntime[2], jointAccRuntime[3],
        jointAccRuntime[4], jointAccRuntime[5], jointAccRuntime[6],
    };

    float outSpeeds[7] = {0};
    float timeBudget = ComputeSyncSpeeds(deltaRails, SLIDER_CAPS_V, jointAcc,
                                        sliderSpeedRps, outSpeeds);

    // 写回 dynamicJointSpeeds7：[0]=地轨，[1..6]=J1..J6
    dynamicJointSpeeds7[0] = outSpeeds[0];
    for (int j = 1; j <= 6; j++) {
        dynamicJointSpeeds7[j] = outSpeeds[j];
        dynamicJointSpeeds.a[j - 1] = outSpeeds[j];   // ServoJ 兼容旧字段
    }

    jointsStateFlag = 0;
    targetJoints = targetJointsTmp;
    targetRailPos = _j7_mm;

    return true;
}


bool DummyRobot::MoveL(float _x, float _y, float _z, float _a, float _b, float _c)
{
    DOF6Kinematic::Pose6D_t pose6D(_x, _y, _z, _a, _b, _c);
    DOF6Kinematic::IKSolves_t ikSolves{};
    DOF6Kinematic::Joint6D_t lastJoint6D{};

    dof6Solver->SolveIK(pose6D, lastJoint6D, ikSolves);

    bool valid[8];
    int validCnt = 0;

    for (int i = 0; i < 8; i++)
    {
        valid[i] = true;

        for (int j = 1; j <= 6; j++)
        {
            if (ikSolves.config[i].a[j - 1] > motorJ[j]->angleLimitMax ||
                ikSolves.config[i].a[j - 1] < motorJ[j]->angleLimitMin)
            {
                valid[i] = false;
                continue;
            }
        }

        if (valid[i]) validCnt++;
    }

    if (validCnt)
    {
        float min = 1000;
        uint8_t indexConfig = 0, indexJoint = 0;
        for (int i = 0; i < 8; i++)
        {
            if (valid[i])
            {
                for (int j = 0; j < 6; j++)
                    lastJoint6D.a[j] = ikSolves.config[i].a[j];
                DOF6Kinematic::Joint6D_t tmp = currentJoints - lastJoint6D;
                float maxAngle = AbsMaxOf6(tmp, indexJoint);
                if (maxAngle < min)
                {
                    min = maxAngle;
                    indexConfig = i;
                }
            }
        }

        // [NEW] MoveL 不动地轨，地轨保持当前位置
        return MoveJ(ikSolves.config[indexConfig].a[0], ikSolves.config[indexConfig].a[1],
                     ikSolves.config[indexConfig].a[2], ikSolves.config[indexConfig].a[3],
                     ikSolves.config[indexConfig].a[4], ikSolves.config[indexConfig].a[5],
                     currentRailPos, 0);
    }

    return false;
}

void DummyRobot::UpdateJointAngles()
{
    // 轮询查询 1~7（J1~J7，含地轨），每帧查一个电机，100Hz 下约 14Hz/轴
    static int currentIndex = 1;
    motorJ[currentIndex]->UpdateAngle();
    currentIndex++;
    if (currentIndex > 7) currentIndex = 1;
}


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
    // 地轨：motorJ[7]->angle 单位是"电机轴圈×360"（reduction=1）
    // 反推 mm: angle / 360 × 5
    currentRailPos = motorJ[7]->angle / 360.0f * RAIL_MM_PER_CIRCLE;
    // bit7 跟随地轨 FINISH 状态，参与 OLED *-_ 指示
    if (motorJ[7]->state == CtrlStepMotor::FINISH)
        jointsStateFlag |= (1 << 7);
    else
        jointsStateFlag &= ~(1 << 7);
}


void DummyRobot::SetJointSpeed(float _speed)
{
    if (!IsFiniteCommandValue(_speed)) return;
    if (_speed < 1) _speed = 1;
    else if (_speed > 100) _speed = 100;
    jointSpeed = _speed;
}


void DummyRobot::SetJointAcceleration(float _acc)
{
    if (!IsFiniteCommandValue(_acc)) return;
    if (_acc < 1) _acc = 1;
    else if (_acc > 5000) _acc = 5000;

    for (int i = 1; i <= 7; i++)
    {
        motorJ[i]->SetAcceleration_persist(_acc, false);
        // 立即更新规划值，随后由电机回读确认，避免紧接的 MoveJ 使用旧值。
        jointAccRuntime[i] = _acc;
    }
    SyncAllMotorAcceleration();
}


void DummyRobot::CalibrateHomeOffset()
{
    // Disable FixUpdate, but not disable motors
    isEnabled = false;
    motorJ[ALL]->SetEnable(true);

    // 1.Manually move joints to L-Pose [precisely]
    // ...
    motorJ[2]->SetCurrentLimit(0.5);
    motorJ[3]->SetCurrentLimit(0.5);
    osDelay(500);

    // 2.Apply Home-Offset the first time
    motorJ[ALL]->ApplyPositionAsHome();
    osDelay(500);

    // 3.Go to Resting-Pose
    initPose = DOF6Kinematic::Joint6D_t(0, 0, 90, 0, 0, 0);
    currentJoints = DOF6Kinematic::Joint6D_t(0, 0, 90, 0, 0, 0);
    Resting();
    osDelay(500);

    // 4.Apply Home-Offset the second time
    motorJ[ALL]->ApplyPositionAsHome();
    osDelay(500);
    motorJ[2]->SetCurrentLimit(1);
    motorJ[3]->SetCurrentLimit(1);
    osDelay(500);

    Reboot();
}


void DummyRobot::Homing()
{
    float lastSpeed = jointSpeed;
    SetJointSpeed(10);

    MoveJ(0, 0, 90, 0, 0, 0, 0, 10);   // J1~J6 = 0,0,90,0,0,0 + 地轨 = 0mm
    MoveJoints(targetJoints);
    while (IsMoving())
        osDelay(10);

    SetJointSpeed(lastSpeed);
}


void DummyRobot::Resting()
{
    float lastSpeed = jointSpeed;
    SetJointSpeed(10);

    MoveJ(REST_POSE.a[0], REST_POSE.a[1], REST_POSE.a[2],
          REST_POSE.a[3], REST_POSE.a[4], REST_POSE.a[5], 0, 10);  // 地轨=0mm
    MoveJoints(targetJoints);
    while (IsMoving())
        osDelay(10);

    SetJointSpeed(lastSpeed);
}


void DummyRobot::SetEnable(bool _enable)
{
    motorJ[ALL]->SetEnable(_enable);
    isEnabled = _enable;
    if (_enable) motorJ[ALL]->UpdateAngle();  // 预发一次回包请求
}

void DummyRobot::SetRGBEnable(bool _enable)
{
    isRGBEnabled = _enable;
}

bool DummyRobot::GetRGBEnabled()
{
    return isRGBEnabled;
}

void DummyRobot::SetRGBMode(uint32_t mode)
{
    rgbMode = mode;
}

uint32_t DummyRobot::GetRGBMode()
{
    return rgbMode;
}

void DummyRobot::UpdateJointPose6D()
{
    dof6Solver->SolveFK(currentJoints, currentPose6D);
    currentPose6D.X *= 1000; // m -> mm
    currentPose6D.Y *= 1000; // m -> mm
    currentPose6D.Z *= 1000; // m -> mm
}


bool DummyRobot::IsMoving()
{
    return (jointsStateFlag & 0b11111110) != 0b11111110;  // bit1~7 全置位 → FINISH（夹爪位 bit8 不参与）
}


bool DummyRobot::IsEnabled()
{
    return isEnabled;
}


void DummyRobot::SetCommandMode(uint32_t _mode)
{
    if (_mode < COMMAND_TARGET_POINT_SEQUENTIAL ||
        _mode > COMMAND_MOTOR_TUNING)
        return;

    commandMode = static_cast<CommandMode>(_mode);

    // 切换 SEQ/INT/TRJ/TUN 保留速度档位及电机加速度设置。
    // 启动时也不覆盖 EEPROM 中的加速度，由 Init() 的查询回填。
}


uint32_t DummyRobot::CommandHandler::Push(const std::string &_cmd)
{
    osStatus_t status = osMessageQueuePut(commandFifo, _cmd.c_str(), 0U, 0U);
    if (status == osOK)
        return osMessageQueueGetSpace(commandFifo);

    return 0xFF; // failed
}


void DummyRobot::CommandHandler::EmergencyStop()
{
    context->MoveJ(context->currentJoints.a[0], context->currentJoints.a[1], context->currentJoints.a[2],
                   context->currentJoints.a[3], context->currentJoints.a[4], context->currentJoints.a[5],
                   context->currentRailPos, 0);  // 地轨保持当前位置
    context->MoveJoints(context->targetJoints);
    context->isEnabled = false;
    ClearFifo();
}


std::string DummyRobot::CommandHandler::Pop(uint32_t timeout)
{
    osStatus_t status = osMessageQueueGet(commandFifo, strBuffer, nullptr, timeout);

    return std::string{strBuffer};
}


uint32_t DummyRobot::CommandHandler::GetSpace()
{
    return osMessageQueueGetSpace(commandFifo);
}


uint32_t DummyRobot::CommandHandler::ParseCommand(const std::string &_cmd)
{
    uint8_t argNum;

    switch (context->commandMode)
    {
        case COMMAND_TARGET_POINT_SEQUENTIAL:
        case COMMAND_CONTINUES_TRAJECTORY:
            if (_cmd[0] == '>' || _cmd[0] == '&')
            {
                float j1, j2, j3, j4, j5, j6, j7;
                float speed = 0.0f;
                if (_cmd[0] == '>')
                    argNum = sscanf(_cmd.c_str(), ">%f,%f,%f,%f,%f,%f,%f,%f",
                                    &j1, &j2, &j3, &j4, &j5, &j6, &j7, &speed);
                if (_cmd[0] == '&')
                    argNum = sscanf(_cmd.c_str(), "&%f,%f,%f,%f,%f,%f,%f,%f",
                                    &j1, &j2, &j3, &j4, &j5, &j6, &j7, &speed);
                if (argNum >= 7 && IsFiniteCommandValue(speed))
                {
                    if (argNum == 8) context->SetJointSpeed(speed);
                    // 七参数为六关节 + 地轨；省略速度时沿用当前档位。
                    context->MoveJ(j1, j2, j3, j4, j5, j6, j7, speed);
                }
                // Trigger a transmission immediately, in case IsMoving() returns false
                context->MoveJoints(context->targetJoints);

                while (context->IsMoving() && context->IsEnabled())
                    osDelay(5);
                Respond(*usbStreamOutputPtr, "ok");
                Respond(*uart4StreamOutputPtr, "ok");
            } else if (_cmd[0] == '@')
            {
                float pose[6];
                float speed;

                argNum = sscanf(_cmd.c_str(), "@%f,%f,%f,%f,%f,%f,%f", pose, pose + 1, pose + 2,
                                pose + 3, pose + 4, pose + 5, &speed);
                if (argNum == 6)
                {
                    context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
                } else if (argNum == 7 && IsFiniteCommandValue(speed))
                {
                    context->SetJointSpeed(speed);
                    context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
                }
                Respond(*usbStreamOutputPtr, "ok");
                Respond(*uart4StreamOutputPtr, "ok");
            }

            break;

        case COMMAND_TARGET_POINT_INTERRUPTABLE:
            if (_cmd[0] == '>' || _cmd[0] == '&')
            {
                float j1, j2, j3, j4, j5, j6, j7;
                float speed = 0.0f;
                if (_cmd[0] == '>')
                    argNum = sscanf(_cmd.c_str(), ">%f,%f,%f,%f,%f,%f,%f,%f",
                                    &j1, &j2, &j3, &j4, &j5, &j6, &j7, &speed);
                if (_cmd[0] == '&')
                    argNum = sscanf(_cmd.c_str(), "&%f,%f,%f,%f,%f,%f,%f,%f",
                                    &j1, &j2, &j3, &j4, &j5, &j6, &j7, &speed);
                if (argNum >= 7 && IsFiniteCommandValue(speed))
                {
                    if (argNum == 8) context->SetJointSpeed(speed);
                    // 七参数为六关节 + 地轨；省略速度时沿用当前档位。
                    context->MoveJ(j1, j2, j3, j4, j5, j6, j7, speed);
                }
                Respond(*usbStreamOutputPtr, "ok");
                Respond(*uart4StreamOutputPtr, "ok");
            } else if (_cmd[0] == '@')
            {
                float pose[6];
                float speed;

                argNum = sscanf(_cmd.c_str(), "@%f,%f,%f,%f,%f,%f,%f", pose, pose + 1, pose + 2,
                                pose + 3, pose + 4, pose + 5, &speed);
                if (argNum == 6)
                {
                    context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
                } else if (argNum == 7 && IsFiniteCommandValue(speed))
                {
                    context->SetJointSpeed(speed);
                    context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
                }
                Respond(*usbStreamOutputPtr, "ok");
                Respond(*uart4StreamOutputPtr, "ok");
            }
            break;

        case COMMAND_MOTOR_TUNING:
            break;
    }

    return osMessageQueueGetSpace(commandFifo);
}


void DummyRobot::CommandHandler::ClearFifo()
{
    osMessageQueueReset(commandFifo);
}


void DummyRobot::TuningHelper::SetTuningFlag(uint8_t _flag)
{
    tuningFlag = _flag;
}


void DummyRobot::TuningHelper::Tick(uint32_t _timeMillis)
{
    time += PI * 2 * frequency * (float) _timeMillis / 1000.0f;
    float delta = amplitude * sinf(time);

    for (int i = 1; i <= 6; i++)
        if (tuningFlag & (1 << (i - 1)))
            context->motorJ[i]->SetAngle(delta);
}


void DummyRobot::TuningHelper::SetFreqAndAmp(float _freq, float _amp)
{
    if (_freq > 5)_freq = 5;
    else if (_freq < 0.1) _freq = 0.1;
    if (_amp > 50)_amp = 50;
    else if (_amp < 1) _amp = 1;

    frequency = _freq;
    amplitude = _amp;
}
