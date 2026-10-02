#include "communication.hpp"
#include "dummy_robot.h"

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
    // 梯形：加速→匀速→减速
    return 2.0f * v / a + (d - 2.0f * d_acc) / v;
}


/**
 * @brief §3.3.1 7 轴同步抵达速度规划 v2.7（梯形曲线 + 迭代收敛）
 * @param deltaRails[7] 各轴距离，[0]=地轨(mm)，[1..6]=关节(°)
 * @param sliderCaps[7] 各轴电机轴 r/s 上限（地轨=30，关节=30）
 * @param accel[7]      各轴加速度（r/s²），0 用 JOINT_ACC_FALLBACK 兜底
 * @param sliderSpeed   统一基础速度（slider × 0.30，r/s）
 * @param outSpeeds[7]  输出每轴最终电机轴 r/s
 * @return timeBudget   同步抵达总时间（s）
 * @note  与电机端 motion_planner.cpp PositionTracker 公式一致：对称梯形加减速，
 *        距离太短退化为三角形曲线。本期 jointAccRuntime[] 异步回填逻辑属 §9 #7，
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

    // 步骤 2：初始预算时间 = 各轴在 sliderSpeed（钳制到 cap）下的梯形时间，取 max
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

    // 步骤 3：迭代收敛（最多 10 轮，一般 2~3 轮收敛）
    // 解二次方程：v²/a - v*T + d = 0  →  v = (T - sqrt(T² - 4d/a)) × a / 2
    for (int iter = 0; iter < 10; iter++) {
        float newTimeBudget = 0;
        for (int i = 0; i < 7; i++) {
            if (distMotor[i] < 1e-6f) {
                outSpeeds[i] = 0;
                continue;
            }
            float a  = (accel[i] > 1e-6f) ? accel[i] : JOINT_ACC_FALLBACK[i];
            float vc = sliderCaps[i];
            float disc = timeBudget * timeBudget - 4.0f * distMotor[i] / a;
            // 临界抖动：disc 偶尔 <0 时钳到 0，让反推公式自然算出"刚够用"速度
            if (disc < 0) disc = 0;
            float v = (timeBudget - sqrtf(disc)) * a * 0.5f;
            // 双钳制：物理上限 (vc) + 用户意图上限 (sliderSpeed)
            if (v > vc) v = vc;
            if (v > sliderSpeed) v = sliderSpeed;
            outSpeeds[i] = v;
            float t = timeTrapezoid(distMotor[i], v, a);
            if (t > newTimeBudget) newTimeBudget = t;
        }
        if (fabsf(newTimeBudget - timeBudget) < 1e-3f) {
            timeBudget = newTimeBudget;
            break;
        }
        timeBudget = newTimeBudget;
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

    hand = new DummyHand(_hcan, 8);   

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
}


void DummyRobot::Reboot()
{
    motorJ[ALL]->Reboot();
    hand->Reboot();                          // [FIX] 夹爪不在 ALL 代理范围，需独立发 0x7F 帧
    osDelay(500); // waiting for all joints done
    HAL_NVIC_SystemReset();
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

    // §3.3.1 物理同步：梯形曲线 + 迭代收敛（替换原"按距离比例限速"）
    // 按用户给定 speed (1~100) 换算电机轴 r/s：100 → 30 r/s
    float slider = _speed > 0 ? _speed : 50.0f;  // 默认值 50（原 DEFAULT_JOINT_SPEED）
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
    float jointAcc[7] = {
        jointAccRuntime[0], jointAccRuntime[1], jointAccRuntime[2],
        jointAccRuntime[3], jointAccRuntime[4], jointAccRuntime[5],
        jointAccRuntime[6],
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
    if (_speed < 0)_speed = 0;
    else if (_speed > 100) _speed = 100;

    jointSpeed = _speed * jointSpeedRatio;
}


void DummyRobot::SetJointAcceleration(float _acc)
{
    if (_acc < 0)_acc = 0;
    else if (_acc > 100) _acc = 100;

    for (int i = 1; i <= 6; i++)
        motorJ[i]->SetAcceleration(_acc / 100 * DEFAULT_JOINT_ACCELERATION_BASES.a[i - 1]);
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
    hand->SetEnable(_enable);
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

    switch (commandMode)
    {
        case COMMAND_TARGET_POINT_SEQUENTIAL:
        case COMMAND_TARGET_POINT_INTERRUPTABLE:
            jointSpeedRatio = 1;
            SetJointAcceleration(DEFAULT_JOINT_ACCELERATION_LOW);
            break;
        case COMMAND_CONTINUES_TRAJECTORY:
            SetJointAcceleration(DEFAULT_JOINT_ACCELERATION_HIGH);
            jointSpeedRatio = 0.3;
            break;
        case COMMAND_MOTOR_TUNING:
            break;
    }
}


DummyHand::DummyHand(CAN_HandleTypeDef* _hcan, uint8_t
_id) :
    nodeID(_id), hcan(_hcan)
{
    txHeader =
        {
            .StdId = 0,
            .ExtId = 0,
            .IDE = CAN_ID_STD,
            .RTR = CAN_RTR_DATA,
            .DLC = 8,
            .TransmitGlobalTime = DISABLE
        };
}


void DummyHand::SetAngle(float _angle)
{
    if (_angle > 30)_angle = 30;
    if (_angle < 0)_angle = 0;

    uint8_t mode = 0x02;
    txHeader.StdId = nodeID << 7 | mode;   // [FIX] 使用构造函数传入的 nodeID（8），不再硬编码 7

    // Float to Bytes
    auto* b = (unsigned char*) &_angle;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void DummyHand::SetMaxCurrent(float _val)
{
    if (_val > 1)_val = 1;
    if (_val < 0)_val = 0;

    uint8_t mode = 0x01;
    txHeader.StdId = nodeID << 7 | mode;   // [FIX] 使用构造函数传入的 nodeID（8），不再硬编码 7

    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void DummyHand::SetEnable(bool _enable)
{
    if (_enable)
        SetMaxCurrent(maxCurrent);
    else
        SetMaxCurrent(0);
}

void DummyHand::Reboot()
{
    uint8_t mode = 0x7f;
    txHeader.StdId = nodeID << 7 | mode;
    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
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
                if (argNum >= 7)
                {
                    if (argNum == 8) context->SetJointSpeed(speed);
                    // 默认地轨保持当前位置
                    if (argNum == 7) j7 = context->currentRailPos;
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
                } else if (argNum == 7)
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
                if (argNum >= 7)
                {
                    if (argNum == 8) context->SetJointSpeed(speed);
                    // 默认地轨保持当前位置
                    if (argNum == 7) j7 = context->currentRailPos;
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
                } else if (argNum == 7)
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
