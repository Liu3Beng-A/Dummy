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
 *        [v3.1] 索引 i=0 已空闲（ALL 代理），i=1~6=J1~J6，i=7=J7/地轨。
 *        MOTOR_REDUCTION[7]=1（地轨直连丝杆1605，5mm/圈）。
 */
static float ComputeSyncSpeeds(const float deltaRails[7], const float sliderCaps[7],
                               const float accel[7], float sliderSpeed,
                               float outSpeeds[7])
{
    // 步骤 1：各轴距离 → 电机轴转数
    float distMotor[7];
    for (int i = 0; i < 7; i++) {
        if (i == 0) {
            // [v3.1] deltaRails[0] = J7/地轨: mm → 圈（直连丝杆 1605，5mm/圈，reduction=1）
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
 * @note 构建完备系统骨骼：[v3.1] 8 slot 架构 motorJ[0]=ALL代理(ID=0), motorJ[1..7]=J1~J7(含地轨 ID=7), hand=夹爪(ID=8, 固定)
 */
DummyRobot::DummyRobot(CAN_HandleTypeDef* _hcan) :
    hcan(_hcan)
{
    // motorJ[0]: ALL 广播代理（占位对象，nodeID=0，不参与实际下发）
    motorJ[0] = new CtrlStepMotor(_hcan, 0, false, 1, -180, 180);

    motorJ[1] = new CtrlStepMotor(_hcan, 1, false, 50, -175, 175);
    motorJ[2] = new CtrlStepMotor(_hcan, 2, true,  50,  -75,  90);
    motorJ[3] = new CtrlStepMotor(_hcan, 3, true,  50,    0, 180);
    motorJ[4] = new CtrlStepMotor(_hcan, 4, true,  50, -270, 270);
    motorJ[5] = new CtrlStepMotor(_hcan, 5, true,  50, -100, 100);
    motorJ[6] = new CtrlStepMotor(_hcan, 6, true,  30, -180, 180);

    // motorJ[7]: J7/地轨（线性滑轨，直连丝杆1605，转1圈=5mm，行程 -250~250mm，ID=7）
    motorJ[7] = new CtrlStepMotor(_hcan, 7, true, 1, -250, 250);

    hand = new StepHand(_hcan, 8);

    // 地轨位置初始化（mm）
    currentRailPos = 0.0f;
    targetRailPos = 0.0f;

    // 载入 D-H 标准模型基建长度数值搭建系统运算内核空间
    dof6Solver = new DOF6Kinematic(0.165f, 0.0f, 0.170f, 0.117f, 0.0695f, 0.113f);
}

/**
 * @brief 系统垃圾回收：安全清理掉为底层各传动端分配的指针空间防溢出漏错
 * @note [v3.1] motorJ 8 个槽位 0=ALL代理+1~7=7 个电机（含地轨）
 */
DummyRobot::~DummyRobot()
{
    for (int j = 0; j <= 7; j++)
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
 * @note [v3.2] ALL 代理 1 帧 + 夹爪 1 帧 = 2 帧
 *       motorJ[ALL] (nodeID=0) → StdId=(0<<7)|0x7F = 0x7F，所有 J1~J7/地轨电机接收后自重启
 *       hand (nodeID=8)        → StdId=(8<<7)|0x7F
 */
void DummyRobot::Reboot()
{
    motorJ[ALL]->Reboot();  // StdId = (0<<7)|0x7F = 0x7F，所有 J1~J7/地轨电机同时重启
    hand->Reboot();         // StdId = (8<<7)|0x7F
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

    // [v3.1] J7/地轨限位检查
    if (_j7_mm > motorJ[7]->angleLimitMax || _j7_mm < motorJ[7]->angleLimitMin)
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

    // [A3 修复] 不再写 motorJ[j]->targetAngle（给 IsMoving 用，已废弃）
    // ★ 必须保留复位：电机使能后 STOP 状态会把 flag 置 1（按 A1 新方向），
    //   必须在下发新目标时清零，否则 IsMoving() 永远返回 true 导致 !HOME 死循环。
    //   地轨位 bit0 恒为 0 不受影响，flag 复位不影响地轨逻辑。
    jointsStateFlag = 0;

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

    // [v3.1] J7/地轨限位检查
    if (_j7_mm > motorJ[7]->angleLimitMax || _j7_mm < motorJ[7]->angleLimitMin)
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

    // [A3 修复] 不再写 motorJ[j]->targetAngle（给 IsMoving 用，已废弃）
    // ★ 必须保留复位：同 MoveJ，ServoJ 下发新目标时清零 flag，
    //   防止电机使能后 STOP 状态把 flag 置 1（按 A1 新方向），导致 IsMoving 永远 true。
    jointsStateFlag = 0;

    return true;
}

/**
 * @brief 利用抽屉分时循环结构避让单点查询打满 CAN 信道容量引发断线隐患
 * @note [v3.2] 方案C修复：改回单播轮询，每帧只查一个电机（J1~J7循环），避免7节点同时回包导致总线冲突
 *       轮询查询每个电机，100Hz采样频率下每轴约14Hz查询速率，足够刷新状态
 */
void DummyRobot::UpdateJointAngles()
{
    static int currentIndex = 1;  // 轮询索引，1~7（J1~J7）
    
    // 轮询查询每个电机
    motorJ[currentIndex]->UpdateAngle();
    currentIndex++;
    if (currentIndex > 7) currentIndex = 1;  // J1~J7循环
}

/**
 * @brief 在触发回调接管解析到的节点坐标包进而推至逻辑状态判断矩阵更新标记
 */
void DummyRobot::UpdateJointAnglesCallback()
{
    for (int i = 1; i <= 6; i++)
    {
        currentJoints.a[i - 1] = motorJ[i]->angle + initPose.a[i - 1];
        // [A1 修复] CAN 0x23 回包时同步更新 jointsStateFlag：
        //   FINISH → 置位（bit i = 1 表示该轴到位）
        //   其余   → 清位（RUNNING/STOP/STALL 都算"未到位"）
        // 配合 [v3.2] ALL 代理，IsMoving() 基于 jointsStateFlag != ALL_FINISH_FLAG 判定，
        //   7 轴全部 FINISH 时 flag = ALL_FINISH_FLAG = 0x7E（bit1-6=1, bit0=0 地轨位恒 0），
        //   IsMoving() 返回 false，阻塞解除。
        if (motorJ[i]->state == CtrlStepMotor::FINISH)
            jointsStateFlag |= (1 << i);
        else
            jointsStateFlag &= ~(1 << i);
    }
    // [v3.1] J7/地轨回包：motorJ[7]->angle 是"伪°"（= 圈×360，reduction=1）
    // 反推 mm：mm = angle / 360 × 5
    currentRailPos = motorJ[7]->angle / 360.0f * 5.0f;
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
 *        [v3.1] 循环范围扩展到 i<=7，包含 J7/地轨。
 */
void DummyRobot::SetJointAcceleration(float _acc)
{
    if (_acc < 1.0f)     _acc = 1.0f;
    if (_acc > 5000.0f)  _acc = 5000.0f;

    for (int i = 1; i <= 7; i++)
        motorJ[i]->SetAcceleration_persist(_acc, false);

    // 异步回填 runtime 缓存（不阻塞）
    SyncAllMotorAcceleration();
}

/**
 * @brief 向 8 个电机发 CAN 0x2C QueryAcceleration，触发回包更新 jointAccRuntime[]
 * @note  [v3.1] 7 个电机（motorJ[1..7]=J1~J7/地轨, motorJ[0]=ALL代理不查询）+ 夹爪，
 *        J1~J7 → runtime[1..7], 夹爪 → runtime[8]。回包处理在 can_protocol.cpp 0x2C 分支里完成。
 *        电机掉线则对应 runtime[i] 保持原值（不会清零），所以初值 0 与"未查询过"无法区分——这是已知设计。
 */
void DummyRobot::SyncAllMotorAcceleration()
{
    // [v3.1] J1~J7/地轨全部查询（motorJ[0]=ALL 代理，无物理电机，不查询）
    for (int i = 1; i <= 7; i++)
        motorJ[i]->QueryAcceleration();  // J1~J7 → runtime[1..7]
    hand->QueryAcceleration();          // 夹爪 → runtime[8]
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
    (void)motorIndex;
}

void DummyRobot::BroadcastUnlock()
{
    // 连续 3 次 UNLOCKED 广播，间隔 100ms 防丢包
    for (int i = 0; i < 3; i++)
    {
        motorJ[1]->BroadcastUnlock();
        osDelay(100);
    }
    printf("[UNLOCK] broadcast x3 sent\r\n");
}

void DummyRobot::QueryStallStatus()
{
    // [v3.1] 7 个电机（J1~J7/地轨，motorJ[0]=ALL代理不参与）全部查询 en/lock
    // CAN IDs: J1~J6 = 1~6, J7/地轨 = 7
    for (int i = 1; i <= 7; i++)
    {
        motorJ[i]->QueryStallStatus(1);  // 查询 stallProtectSwitch
        motorJ[i]->QueryStallStatus(2);  // 查询 stallMode==LOCKED
    }
}

void DummyRobot::Homing()
{
    float lastSlider = jointSpeedRps / SLIDER_TO_RPS;  // r/s → slider 还原
    SetJointSpeed(10);

    MoveJ(0, 0, 90, 0, 0, 0, 0, 10);  // 归零姿态，j1=0,j2=0,j3=90,j4=0,j5=0,j6=0,j7=0,speed=10
    MoveJoints(targetJoints);
    // [v3.1] J7/地轨：直接下发（railSpeedRps 默认 30 r/s）
    motorJ[7]->SetPositionWithMotorRps(0, railSpeedRps);

    /* [DIAG-2026-09-16] 30s 兜底超时：避免某电机永远不到位时 handler 永久挂死
     * 原 bug：while(IsMoving()) 永真 → !HOME handler 永远不返回（I=2 O=1）。
     * 根因：见 docs/发送就卡死—心跳定位排查方案—2026-09-16.md */
    uint32_t _home_t0 = HAL_GetTick();
    while (IsMoving())
    {
        if (HAL_GetTick() - _home_t0 > 30000U)  // 30s 兜底
            break;
        osDelay(10);
    }

    /* 超时调试输出：哪个 bit 没就位 */
    if (IsMoving())
    {
        printf("[HOME TIMEOUT] jointsStateFlag=0x%02X (expected 0x7E)\n",
               (unsigned)jointsStateFlag);
        for (int i = 1; i <= 6; i++)
        {
            printf("  J%d: state=%d (0=STOP,1=RUN,2=FINISH,3=STALL)\n",
                   i, (int)motorJ[i]->state);
        }
    }

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
          REST_POSE.a[3], REST_POSE.a[4], REST_POSE.a[5], 0, 10);  // 待机姿态，j7/地轨=0mm
    MoveJoints(targetJoints);
    // [v3.1] J7/地轨：直接下发（railSpeedRps 默认 30 r/s）
    motorJ[7]->SetPositionWithMotorRps(0, railSpeedRps);

    /* [DIAG-2026-09-16] 30s 兜底超时：与 Homing() 保持一致
     * 原 bug：while(IsMoving()) 永真 → !RESET handler 永远不返回（I=2 O=1）。
     * 根因：见 docs/发送就卡死—心跳定位排查方案—2026-09-16.md */
    uint32_t _rest_t0 = HAL_GetTick();
    while (IsMoving())
    {
        if (HAL_GetTick() - _rest_t0 > 30000U)  // 30s 兜底
            break;
        osDelay(10);
    }

    /* 超时调试输出：哪个 bit 没就位 */
    if (IsMoving())
    {
        printf("[REST TIMEOUT] jointsStateFlag=0x%02X (expected 0x7E)\n",
               (unsigned)jointsStateFlag);
        for (int i = 1; i <= 6; i++)
        {
            printf("  J%d: state=%d (0=STOP,1=RUN,2=FINISH,3=STALL)\n",
                   i, (int)motorJ[i]->state);
        }
    }

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
        // [A3 修复] 不再写 targetJoints = currentJoints；
        //         jointsStateFlag 方案下，电机 STOP 时 jointsStateFlag 自动全置位
        //         (UpdateAngleCallback 收到 isFinished=0 → |= (1<<i))，
        //         IsMoving() 行为符合"等电机真实状态"的语义。
    }

    // [v3.2] ALL 代理 1 帧 + 夹爪 1 帧 = 2 帧
    motorJ[ALL]->SetEnable(_enable);  // StdId=(0<<7)|0x01，所有 J1~J7/地轨电机接收
    hand->SetEnable(_enable);         // StdId=(8<<7)|0x01
    isEnabled = _enable;

    // [F1 配套] enable 后预发 UpdateAngle，避免 jointsStateFlag 在电机回包前错填
    if (_enable) {
        motorJ[ALL]->UpdateAngle();  // 1 帧触发 7 电机回包，等首次 0x23 自动纠正 state
    }
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
 * @brief 利用 jointsStateFlag 判定所有关节是否已完成收敛
 * @note [v3.2] 失能短路：!DISABLE 后 isEnabled=false，IsMoving() 必须返回 false，
 *       否则后续 SEQUENTIAL 队列会被堵死。
 *       bit=1 表示该轴已 FINISH 到位（电机端 CAN 0x23 回包 isFinished=1），
 *       bit=0 表示该轴仍在运动/STOP/STALL。
 *       地轨位 bit0 恒为 0（UpdateJointAnglesCallback 只处理 i=1~6）。
 *       7 轴全部到位时：flag = ALL_FINISH_FLAG = 0x7E（bit1-6=1，bit0=0）。
 */
bool DummyRobot::IsMoving()
{
    // [v3.2] 失能短路：!DISABLE 后 isEnabled=false，IsMoving() 必须返回 false
    if (!isEnabled) return false;
    return jointsStateFlag != ALL_FINISH_FLAG;  // 0x7E = bit1~6 全置位
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
 * @note [v3.2] ALL 代理 1 帧急停 + 短路 IsMoving
 *       改走 CAN (0<<7)|0x09=0x09 广播急停，所有电机立即 SetBrake(true) 抱闸
 *       锁死在当前位置，不再走 MoveJ(currentJoints) 的"伪急停"（v2.7 旧版会让
 *       电机继续梯形规划"收敛"到当前点，语义不清且响应慢）。
 *       抱闸后电机端 state = STATE_STOP → 0x23 回包 isFinished=0 →
 *       主控 jointsStateFlag |= (1<<i) → IsMoving=true，所以这里手动置
 *       flag=ALL_FINISH_FLAG（物理已停），并清 FIFO + isEnabled=false
 *       短路 IsMoving()。
 */
void DummyRobot::CommandHandler::EmergencyStop()
{
    // 1. 发 0x09 广播（通过 motorJ[ALL]，StdId=(0<<7)|0x09=0x09，被电机端 `id==0` 路由识别）
    context->motorJ[ALL]->BroadcastEStop();
    // 2. 置 jointsStateFlag = ALL_FINISH_FLAG：电机已被抱闸锁死，物理上"全部到位"
    //    避免 !STOP 后 IsMoving() 永远返回 true 阻塞后续 SEQUENTIAL 指令
    context->jointsStateFlag = ALL_FINISH_FLAG;  // 0x7E = bit1~6 全置位
    // 3. 标记失能 + 清指令队列（短路 IsMoving）
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

    // [F2 修复] 堵转期间精确丢弃：运动指令（>/&/@）丢，控制指令（!START/!DISABLE/!STOP 等）放行
    if (context->isStalled) {
        if (_cmd[0] == '>' || _cmd[0] == '&' || _cmd[0] == '@') {
            Respond(*usbStreamOutputPtr,  "ok");
            Respond(*uart4StreamOutputPtr, "ok");
            return osMessageQueueGetSpace(commandFifo);
        }
    }

    // [分支拦截] $ 高频投递的力控透传包剥离拦截，规避无意义繁杂判决延宕
    if (_cmd[0] == '$')
    {
        // [v3.1] 新格式：$c1,c2,c3,c4,c5,c6,c7(rail) — j7(rail) 移到第7位，夹爪由 !HAND_I 单独控制
        float c[7];
        argNum = sscanf(_cmd, "$%f,%f,%f,%f,%f,%f,%f",
                        &c[0], &c[1], &c[2], &c[3], &c[4], &c[5], &c[6]);

        if (argNum == 7)
        {
            if (context->commandMode != COMMAND_TORQUE_CONTROL)
                context->SetCommandMode(COMMAND_TORQUE_CONTROL);

            // [v3.1] 映射：c[6]=c7(rail)→SetJointCurrents 第1参数(c0=地轨)
            //          c[0..5]=c1~c6 → SetJointCurrents 第2~7参数(J1~J6)，c7=夹爪填0
            context->SetJointCurrents(c[6], c[0], c[1], c[2], c[3], c[4], c[5], 0.0f);
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
                // [v3.1] 新格式：>j1,j2,j3,j4,j5,j6,j7(rail),speed — 7 个关节在前，地轨在第7位
                float j1, j2, j3, j4, j5, j6, j7;  // j7=地轨
                float speed = 0.0f;

                argNum = sscanf(_cmd, (_cmd[0] == '>') ?
                                ">%f,%f,%f,%f,%f,%f,%f,%f" : "&%f,%f,%f,%f,%f,%f,%f,%f",
                                &j1, &j2, &j3, &j4, &j5, &j6, &j7, &speed);

                if (argNum == 8) context->SetJointSpeed(speed);
                if (argNum >= 7)
                {
                    if (context->MoveJ(j1, j2, j3, j4, j5, j6, j7, speed))
                    {
                        // [v2.8 修复] SEQUENTIAL 模式下，若电机还在转，丢弃新指令回 ok，
                        // 保留"上一条做完再做下一条"的语义，但不阻塞 FIFO 消费线程。
                        if (context->IsMoving() && context->IsEnabled())
                        {
                            Respond(*usbStreamOutputPtr,  "ok");
                            Respond(*uart4StreamOutputPtr, "ok");
                            break;
                        }
                        context->MoveJoints(context->targetJoints);
                        // [v3.1] J7/地轨：MoveJ 已填入 railSpeedRps，直接下发
                        context->motorJ[7]->SetPositionWithMotorRps(
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
                        // [v2.8 修复] MoveL 同样改为非阻塞，新轨迹若电机还在跑则丢弃
                        if (context->IsMoving() && context->IsEnabled())
                        {
                            Respond(*usbStreamOutputPtr,  "ok");
                            Respond(*uart4StreamOutputPtr, "ok");
                            break;
                        }
                        context->MoveJoints(context->targetJoints);
                        // MoveL 内部其实是 MoveJ，ra 处理同上
                        context->motorJ[7]->SetPositionWithMotorRps(
                            context->targetRailPos / 5.0f, context->railSpeedRps);

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
                // [v3.1] 新格式：>j1,j2,j3,j4,j5,j6,j7(rail),speed
                float j1, j2, j3, j4, j5, j6, j7;  // j7=地轨
                float speed = 0.0f;

                argNum = sscanf(_cmd, (_cmd[0] == '>') ?
                                ">%f,%f,%f,%f,%f,%f,%f,%f" : "&%f,%f,%f,%f,%f,%f,%f,%f",
                                &j1, &j2, &j3, &j4, &j5, &j6, &j7, &speed);

                if (argNum == 8) context->SetJointSpeed(speed);
                if (argNum >= 7)
                {
                    if (context->MoveJ(j1, j2, j3, j4, j5, j6, j7, speed))
                    {
                        context->MoveJoints(context->targetJoints);
                        // [v3.1] J7/地轨：MoveJ 已填入 railSpeedRps，直接下发
                        context->motorJ[7]->SetPositionWithMotorRps(
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
                // [v3.1] 新格式：>j1,j2,j3,j4,j5,j6,j7(rail),speed
                float j1, j2, j3, j4, j5, j6, j7;  // j7=地轨
                float speed = 0.0f;

                argNum = sscanf(_cmd, (_cmd[0] == '>') ?
                                ">%f,%f,%f,%f,%f,%f,%f,%f" : "&%f,%f,%f,%f,%f,%f,%f,%f",
                                &j1, &j2, &j3, &j4, &j5, &j6, &j7, &speed);

                if (argNum == 8) context->SetJointSpeed(speed);
                if (argNum >= 7)
                {
                    if (context->MoveJ(j1, j2, j3, j4, j5, j6, j7, speed))
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
                // [v3.1] 新格式：>j1,j2,j3,j4,j5,j6,j7(rail) — SERVO_J 无 speed 字段，共 7 个 %f
                float j1, j2, j3, j4, j5, j6, j7;  // j7=地轨
                argNum = sscanf(_cmd, (_cmd[0] == '>') ?
                                ">%f,%f,%f,%f,%f,%f,%f" : "&%f,%f,%f,%f,%f,%f,%f",
                                &j1, &j2, &j3, &j4, &j5, &j6, &j7);

                if (argNum >= 7)
                {
                    if (context->ServoJ(j1, j2, j3, j4, j5, j6, j7))
                    {
                        context->MoveJoints(context->targetJoints);
                        // [v3.1] J7/地轨：直接下发（railSpeedRps 默认 30 r/s）
                        context->motorJ[7]->SetPositionWithMotorRps(
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
 * @param c0: J7/地轨电流 (A), c1~c6: 臂关节 J1~J6 电流 (A), c7: 夹爪电流 (A, 由!HAND_I单独控制，此参数填0)
 * @note [v3.1] c0 含义不变（地轨），只是地轨现在走 motorJ[7] 而非 motorJ[0]。
 */
void DummyRobot::SetJointCurrents(float c0, float c1, float c2, float c3, float c4, float c5, float c6, float c7)
{
    // [v3.1] c0~c6 = J1~J7(含地轨)，c7=夹爪(单独控制)
    targetCurrents[0] = c1;  // J1
    targetCurrents[1] = c2;  // J2
    targetCurrents[2] = c3;  // J3
    targetCurrents[3] = c4;  // J4
    targetCurrents[4] = c5;  // J5
    targetCurrents[5] = c6;  // J6
    targetRailCurrent = c0;  // [v3.1] J7/地轨 = c0（第1个值！）

    if (!isEnabled)
        SetEnable(true);

    for (int i = 1; i <= 6; i++)
        motorJ[i]->SetCurrentSetPoint(targetCurrents[i - 1]);
    motorJ[7]->SetCurrentSetPoint(targetRailCurrent);  // [v3.1] J7/地轨
}
