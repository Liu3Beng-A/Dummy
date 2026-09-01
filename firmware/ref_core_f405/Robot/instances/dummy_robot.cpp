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
 * @brief 单轴梯形速度曲线总时间（v2.7，与电机端 motion_planner.cpp PositionTracker 对齐）
 * @param d 距离（电机端圈数）
 * @param v 目标速度（匀速段速度，r/s）
 * @param a 加速度（r/s²）
 * @return 该轴按梯形曲线跑 d 距离的总时间（s）
 * @note  距离太短（2×d_acc >= d）时退化为三角曲线：T = 2√(d/a)
 */
static float timeTrapezoid(float d, float v, float a)
{
    if (d < 1e-6f || v < 1e-6f || a < 1e-6f) return 0;
    float d_acc = v * v / (2.0f * a);  // 单边加速距离
    if (2.0f * d_acc >= d) {
        // 三角曲线：距离太短，没匀速段
        return 2.0f * sqrtf(d / a);
    }
    // 梯形曲线：T = 2v/a + (d - v²/a)/v
    return 2.0f * v / a + (d - 2.0f * d_acc) / v;
}

/**
 * @brief 7 轴同步抵达速度规划 v2.7（梯形曲线模型）
 * @param deltaRails[7] 各轴距离（地轨=mm，关节=°）
 * @param sliderCaps[7] 各轴电机轴 r/s 上限（地轨=30，关节=20）
 * @param accel[7]      各轴加速度（r/s²，来自 jointAccRuntime，0 用 FALLBACK）
 * @param sliderSpeed   统一基础速度（slider × SLIDER_TO_RPS = r/s）
 * @param outSpeeds[7]  输出每轴最终电机轴 r/s
 * @return timeBudget   同步抵达总时间（s）
 * @note  与电机端 motion_planner.cpp PositionTracker 公式一致：对称梯形加减速，
 *        距离太短退化为三角曲线。上电后 jointAccRuntime[] 异步回填电机 EEPROM 值，
 *        未回包的轴用 FALLBACK_JOINT_ACCELERATION 兜底。
 */
static float ComputeSyncSpeeds(const float deltaRails[7], const float sliderCaps[7],
                               const float accel[7], float sliderSpeed,
                               float outSpeeds[7])
{
    // 步骤 1：各轴距离 → 电机轴转数
    float distMotor[7];
    for (int i = 0; i < 7; i++) {
        if (i == 0) {
            // 地轨：mm → 圈（直连丝杆 1605，5mm/圈，reduction=1）
            distMotor[i] = fabsf(deltaRails[i]) / 5.0f;
        } else {
            // 关节：° → 电机圈（reduction × / 360）
            distMotor[i] = fabsf(deltaRails[i]) * (float)MOTOR_REDUCTION[i] / 360.0f;
        }
    }

    // 步骤 2：初始预算时间 = 各轴在 sliderSpeed（钳制到 cap）下的梯形时间，取 max
    float timeBudget = 0;
    for (int i = 0; i < 7; i++) {
        if (distMotor[i] < 1e-6f) continue;
        float a = (accel[i] > 1e-6f) ? accel[i] : FALLBACK_JOINT_ACCELERATION;
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
    // 该公式在梯形曲线约束下反推匀速段速度 v_needed；
    // 然后用 cap 钳制、重新算 T_new 直到稳定。
    for (int iter = 0; iter < 10; iter++) {
        float newTimeBudget = 0;
        for (int i = 0; i < 7; i++) {
            if (distMotor[i] < 1e-6f) {
                outSpeeds[i] = 0;
                continue;
            }
            float a  = (accel[i] > 1e-6f) ? accel[i] : FALLBACK_JOINT_ACCELERATION;
            float vc = sliderCaps[i];
            float disc = timeBudget * timeBudget - 4.0f * distMotor[i] / a;
            // Bug1 修复：浮点临界抖动导致 disc 偶尔 <0 时，直接给 cap 会让短距离
            // 运动突然飙到最大速度。改为钳到 0，让反推公式自然算出"刚够用"的速度。
            if (disc < 0) disc = 0;
            float v = (timeBudget - sqrtf(disc)) * a * 0.5f;
            // 双钳制：物理上限 (vc) + 用户意图上限 (sliderSpeed)
            if (v > vc) v = vc;
            if (v > sliderSpeed) v = sliderSpeed;
            outSpeeds[i] = v;
            // 用 v 重算实际梯形时间，作为下一轮 T_new
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
 * @note 读取灯效风格预设。若首次启动匹配不到特解标识字，
 *       会自动写回原始初始化表覆写空白位（v2.6 起 EepromConfig
 *       不再包含 jointAccBases，加速度统一从 DEFAULT_JOINT_ACCELERATION 起步）。
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

    // v2.6 上电后立刻向 8 个电机发 CAN 0x2C 查询，把真实加速度缓存到 jointAccRuntime[]
    // 供后续 ComputeSyncSpeeds 算法使用。回包延迟 ≤1 帧 CAN 周期。
    SyncAllMotorAcceleration();
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
        motorJ[j]->SetAngleWithMotorRps(_joints.a[j - 1] - initPose.a[j - 1],
                                        dynamicJointSpeeds.a[j - 1]);
}

/**
 * @brief 下发地轨指令（mm → 圈）
 * @param _railPos_mm 地轨目标位置 (mm)
 * @note 地轨不纳入6-DOF运动学求解，单独管理
 * @note 电机固件 CAN 协议期望接收：位置(圈)、速度(圈/s)，内部乘以细分系数
 */
void DummyRobot::MoveRailRelative(float _delta_mm)
{
    targetRailPos += _delta_mm;
    // 硬限位保护，防止超出 [-250, 250]
    if (targetRailPos > motorJ[0]->angleLimitMax)
        targetRailPos = motorJ[0]->angleLimitMax;
    if (targetRailPos < motorJ[0]->angleLimitMin)
        targetRailPos = motorJ[0]->angleLimitMin;
    float rail_laps = targetRailPos / 5.0f;
    motorJ[0]->SetPositionWithMotorRps(rail_laps, railSpeedRps);
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
        // 用当前 jointSpeedRps 反推 slider（近似）
        float slider = jointSpeedRps / SLIDER_TO_RPS;
        if (slider < 1) slider = 1;
        if (slider > 100) slider = 100;
        return MoveJ(ikSolves.config[bestConfig].a[0],
                     ikSolves.config[bestConfig].a[1],
                     ikSolves.config[bestConfig].a[2],
                     ikSolves.config[bestConfig].a[3],
                     ikSolves.config[bestConfig].a[4],
                     ikSolves.config[bestConfig].a[5],
                     currentRailPos,
                     slider);  // 地轨位置保持不变
    }
    return false;
}

/**
 * @brief 向定点旋转并发规划驱动组群下属协同运转指令
 * @param _j1~_j6: 臂关节角度 (°), _j7_mm: 地轨位置 (mm), _slider: 速度滑块 (1~100)
 * @note 内置基于极限基准点运算降维匹配同步缩放比例限速引擎保护
 */
bool DummyRobot::MoveJ(float _j1, float _j2, float _j3, float _j4, float _j5, float _j6, float _j7_mm, float _slider)
{
    DOF6Kinematic::Joint6D_t targetJointsTmp(_j1, _j2, _j3, _j4, _j5, _j6);

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

    DOF6Kinematic::Joint6D_t deltaAngles = targetJointsTmp - currentJoints;
    float deltaRail = _j7_mm - currentRailPos;

    // 构造 7 轴距离向量
    float delta7[7] = {
        deltaRail,                              // 地轨 (mm)
        deltaAngles.a[0], deltaAngles.a[1], deltaAngles.a[2],
        deltaAngles.a[3], deltaAngles.a[4], deltaAngles.a[5]
    };

    // 统一基础速度：slider × SLIDER_TO_RPS = r/s（钳制前）
    float sliderSpeed = _slider * SLIDER_TO_RPS;
    const float* sliderCaps = AXIS_MAX_RPS;

    // 调用迭代收敛算法（v2.7: 加入加速度感知，复刻电机端梯形曲线模型）
    ComputeSyncSpeeds(delta7, sliderCaps, jointAccRuntime, sliderSpeed,
                      dynamicJointSpeeds7.rps);

    // 地轨速度写入 railSpeedRps（MoveJoints 会用到）
    railSpeedRps = dynamicJointSpeeds7.rps[0];

    targetJoints = targetJointsTmp;
    targetRailPos = _j7_mm;

    // 旧字段 dynamicJointSpeeds（Joint6D_t）保持同步（向后兼容 ServoJ）
    for (int j = 1; j <= 6; j++)
        dynamicJointSpeeds.a[j - 1] = dynamicJointSpeeds7.rps[j];

    // 写入目标角度（纯位置误差判定用）
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

    switch (group) {
        case 0:
            motorJ[1]->UpdateAngle();   // J1
            motorJ[2]->UpdateAngle();   // J2
            break;
        case 1:
            motorJ[3]->UpdateAngle();   // J3
            motorJ[4]->UpdateAngle();   // J4
            break;
        case 2:
            motorJ[5]->UpdateAngle();   // J5
            motorJ[6]->UpdateAngle();   // J6
            break;
        case 3:
            motorJ[0]->UpdateAngle();   // 地轨（T-2 新增）
            break;
    }

    group = (group + 1) % 4;
}

/**
 * @brief 在触发回调接管解析到的节点坐标包进而推至逻辑状态判断矩阵更新标记
 */
void DummyRobot::UpdateJointAnglesCallback()
{
    for (int i = 1; i <= 6; i++)
    {
        currentJoints.a[i - 1] = motorJ[i]->angle + initPose.a[i - 1];
    }
    // 地轨回包：motorJ[0]->angle 是"伪°"（= 圈×360，reduction=1）
    // 反推 mm：mm = angle / 360 × 5
    currentRailPos = motorJ[0]->angle / 360.0f * 5.0f;
}

/**
 * @brief 分配调准基准速率运行档位
 */
void DummyRobot::SetJointSpeed(float _slider)
{
    if (_slider < 0)        _slider = 0;
    else if (_slider > 100) _slider = 100;

    jointSpeedRps = _slider * SLIDER_TO_RPS * jointSpeedRatio;
    // 关节轴上限（D-Q4 修订）：slider 100 → 30 r/s → 由 AXIS_MAX_RPS[1~6] 钳制到 20 r/s
    if (jointSpeedRps > AXIS_MAX_RPS[1])
        jointSpeedRps = AXIS_MAX_RPS[1];
}

/**
 * @brief 直接下发电机轴 r/s² 加速度到电机端（与 CAN 0x14 入参 float 一致）
 * @param _acc 加速度值 (r/s²)，推荐范围 1~5000
 * @note  带 persist=false（不写电机 EEPROM）。
 *        下发后异步触发 SyncAllMotorAcceleration 让 jointAccRuntime[] 及时刷新。
 */
void DummyRobot::SetJointAcceleration(float _acc)
{
    if (_acc < 1.0f)     _acc = 1.0f;
    if (_acc > 5000.0f)  _acc = 5000.0f;

    for (int i = 1; i <= 6; i++)
        motorJ[i]->SetAcceleration_persist(_acc, false);

    // 异步回填 runtime 缓存（不阻塞）
    SyncAllMotorAcceleration();
}

/**
 * @brief 向 8 个电机发 CAN 0x2C QueryAcceleration，触发回包更新 jointAccRuntime[]
 * @note  顺序: 地轨(9) → J1~J6(1~6) → 夹爪(8)，与 ASCII "#SYNC_ACC" 协议一致。
 *        回包处理在 can_protocol.cpp 0x2C 分支里完成。电机掉线则对应 runtime[i]
 *        保持原值（不会清零），所以初值 0 与"未查询过"无法区分——这是已知设计。
 */
void DummyRobot::SyncAllMotorAcceleration()
{
    motorJ[0]->QueryAcceleration();   // 地轨  → runtime[0]
    for (int i = 1; i <= 6; i++)
        motorJ[i]->QueryAcceleration(); // J1~J6 → runtime[i]
    hand->QueryAcceleration();          // 夹爪  → runtime[7]
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
    // 标记堵转状态，拦截后续运动指令直到用户发送 !START 或 !DISABLE
    isStalled = true;
    // 停发新位置指令，保持当前位置（同步 targetAngle 避免误判）
    targetJoints = currentJoints;
    for (int j = 1; j <= 6; j++) {
        motorJ[j]->targetAngle = currentJoints.a[j - 1] - initPose.a[j - 1];
    }
    // Bug-11 修复: 同步 targetRailPos 到 currentRailPos，否则主循环 500ms 周期
    // MoveRail(targetRailPos) 仍按堵转前目标下发，enable 退出 LOCKED 后会覆盖
    // 电机端 ResetGoalsToCurrentPosition() 设置，导致再次向堵转点推进 → 死循环
    targetRailPos = currentRailPos;
    // 清空指令队列，防止残留指令堆积
    commandHandler.ClearFifo();
    (void)motorIndex;  // 未来可用于区分哪个电机堵转并做针对性处理
}

void DummyRobot::BroadcastUnlock()
{
    // 连续 3 次 UNLOCKED 广播，间隔 100ms 防丢包
    for (int i = 0; i < 3; i++)
    {
        motorJ[1]->BroadcastUnlock();  // 使用 motorJ[1] 作为 CAN 总线发送口
        osDelay(100);
    }
    printf("[UNLOCK] broadcast x3 sent\r\n");
}

void DummyRobot::QueryStallStatus()
{
    // 查询地轨 + J1~J6 共 7 个电机的 en/lock 状态
    // CAN IDs: 地轨=9, J1~J6 = 1~6
    motorJ[0]->QueryStallStatus(1);  // 查询 stallProtectSwitch
    motorJ[0]->QueryStallStatus(2);  // 查询 stallMode==LOCKED
    for (int i = 1; i <= 6; i++)
    {
        motorJ[i]->QueryStallStatus(1);
        motorJ[i]->QueryStallStatus(2);
    }
}

void DummyRobot::Homing()
{
    float lastSlider = jointSpeedRps / SLIDER_TO_RPS;  // r/s → slider 还原
    SetJointSpeed(10);

    MoveJ(0, 0, 90, 0, 0, 0, 0, 10);  // 归零姿态，地轨=0mm
    MoveJoints(targetJoints);
    // 地轨：直接下发（railSpeedRps 默认 30 r/s）
    motorJ[0]->SetPositionWithMotorRps(0, railSpeedRps);
    while (IsMoving())
        osDelay(10);

    SetJointSpeed(lastSlider);  // 还原用户原 slider
}

/**
 * @brief 将设备挂入无伤放松的安全缩骨隐蔽初始安睡位置
 */
void DummyRobot::Resting()
{
    float lastSlider = jointSpeedRps / SLIDER_TO_RPS;
    SetJointSpeed(10);

    MoveJ(REST_POSE.a[0], REST_POSE.a[1], REST_POSE.a[2],
          REST_POSE.a[3], REST_POSE.a[4], REST_POSE.a[5], 0, 10);  // 待机姿态，地轨=0mm
    MoveJoints(targetJoints);
    // 地轨：直接下发（railSpeedRps 默认 30 r/s）
    motorJ[0]->SetPositionWithMotorRps(0, railSpeedRps);
    while (IsMoving())
        osDelay(10);

    SetJointSpeed(lastSlider);
}

/**
 * @brief 发送节点通断电流源动力配置及 RGB 等附属工作展示配合转换
 */
void DummyRobot::SetEnable(bool _enable)
{
    if (_enable)
    {
        isStalled = false;  // !START 可解除堵转拦截状态
        SetRGBMode(rgbStateEnable);
    }
    else
    {
        isStalled = false;  // !DISABLE 可解除堵转拦截状态
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
    static constexpr float EPSILON_MM  = 0.5f;

    // 地轨判定：currentRailPos vs targetRailPos
    if (fabsf(currentRailPos - targetRailPos) > EPSILON_MM)
        return true;

    // 关节判定
    for (int i = 1; i <= 6; i++) {
        if (fabsf(motorJ[i]->angle - motorJ[i]->targetAngle) > EPSILON_DEG)
            return true;
    }
    return false;
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
            // v2.7: 不再调用 SetJointAcceleration 覆盖电机端加速度，
            // 由 SyncAllMotorAcceleration() 在 Init() 末尾从 EEPROM 读取真实值
            break;

        case COMMAND_CONTINUES_TRAJECTORY:
            // v2.7: 不再调用 SetJointAcceleration 覆盖电机端加速度，
            // 由 SyncAllMotorAcceleration() 在 Init() 末尾从 EEPROM 读取真实值
            // jointSpeedRatio 不再自动减半，由 SetJointSpeed 直接用 slider × SLIDER_TO_RPS
            break;

        case COMMAND_MOTOR_TUNING:
            break;

        case COMMAND_TORQUE_CONTROL:
            break;

        case COMMAND_SERVO_J:
            SetJointAcceleration(DEFAULT_JOINT_ACCELERATION);
            break;
    }
}

/**
 * @brief 使用安全字节转移封包放入信道队列排位阻断越界爆破可能
 */
uint32_t DummyRobot::CommandHandler::Push(const char *_cmd)
{
    char buf[128] = {0};
    strncpy(buf, _cmd, sizeof(buf) - 1);
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
                   context->currentRailPos, 10);  // 10 = 安全速度
    context->MoveJoints(context->targetJoints);
    // 地轨：直接下发（railSpeedRps 默认 30 r/s）
    context->motorJ[0]->SetPositionWithMotorRps(context->targetRailPos / 5.0f, context->railSpeedRps);
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
    return nullptr;
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
uint32_t DummyRobot::CommandHandler::ParseCommand(const char *_cmd)
{
    uint8_t argNum;

    // [分支拦截] $ 高频投递的力控透传包剥离拦截，规避无意义繁杂判决延宕
    if (_cmd[0] == '$')
    {
        // $c0(地轨),c1~c6(关节),c7(夹爪)
        float cur[7];
        argNum = sscanf(_cmd, "$%f,%f,%f,%f,%f,%f,%f",
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

                argNum = sscanf(_cmd, (_cmd[0] == '>') ?
                                ">%f,%f,%f,%f,%f,%f,%f,%f" : "&%f,%f,%f,%f,%f,%f,%f,%f",
                                joints, joints+1, joints+2, joints+3, joints+4, joints+5, &j7, &speed);

                if (argNum == 8) context->SetJointSpeed(speed);
                if (argNum >= 7)
                {
                    if (context->MoveJ(joints[0], joints[1], joints[2],
                                   joints[3], joints[4], joints[5], j7, speed))
                    {
                        context->MoveJoints(context->targetJoints);
                        // 地轨：MoveJ 已填入 railSpeedRps，直接下发
                        context->motorJ[0]->SetPositionWithMotorRps(
                            context->targetRailPos / 5.0f, context->railSpeedRps);

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
                argNum = sscanf(_cmd, "@%f,%f,%f,%f,%f,%f,%f", pose, pose+1, pose+2, pose+3, pose+4, pose+5, &speed);
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

                argNum = sscanf(_cmd, (_cmd[0] == '>') ?
                                ">%f,%f,%f,%f,%f,%f,%f,%f" : "&%f,%f,%f,%f,%f,%f,%f,%f",
                                joints, joints+1, joints+2, joints+3, joints+4, joints+5, &j7, &speed);

                if (argNum == 8) context->SetJointSpeed(speed);
                if (argNum >= 7)
                {
                    if (context->MoveJ(joints[0], joints[1], joints[2],
                                   joints[3], joints[4], joints[5], j7, speed))
                    {
                        context->MoveJoints(context->targetJoints);
                        // 地轨：MoveJ 已填入 railSpeedRps，直接下发
                        context->motorJ[0]->SetPositionWithMotorRps(
                            context->targetRailPos / 5.0f, context->railSpeedRps);

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
                argNum = sscanf(_cmd, "@%f,%f,%f,%f,%f,%f,%f", pose, pose+1, pose+2, pose+3, pose+4, pose+5, &speed);
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

                argNum = sscanf(_cmd, (_cmd[0] == '>') ?
                                ">%f,%f,%f,%f,%f,%f,%f,%f" : "&%f,%f,%f,%f,%f,%f,%f,%f",
                                joints, joints+1, joints+2, joints+3, joints+4, joints+5, &j7, &speed);

                if (argNum == 8) context->SetJointSpeed(speed);
                if (argNum >= 7)
                {
                    if (context->MoveJ(joints[0], joints[1], joints[2],
                                   joints[3], joints[4], joints[5], j7, speed))
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
                argNum = sscanf(_cmd, "@%f,%f,%f,%f,%f,%f,%f", pose, pose+1, pose+2, pose+3, pose+4, pose+5, &speed);
                
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
                argNum = sscanf(_cmd, (_cmd[0] == '>') ?
                                ">%f,%f,%f,%f,%f,%f,%f" : "&%f,%f,%f,%f,%f,%f,%f",
                                joints, joints+1, joints+2, joints+3, joints+4, joints+5, &j7);

                if (argNum >= 7)
                {
                    if (context->ServoJ(joints[0], joints[1], joints[2], joints[3], joints[4], joints[5], j7))
                    {
                        context->MoveJoints(context->targetJoints);
                        // 地轨：直接下发（railSpeedRps 默认 30 r/s）
                        context->motorJ[0]->SetPositionWithMotorRps(
                            context->targetRailPos / 5.0f, context->railSpeedRps);
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
