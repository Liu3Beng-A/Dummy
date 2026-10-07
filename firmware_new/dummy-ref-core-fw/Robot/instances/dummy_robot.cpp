#include "communication.hpp"
#include "dummy_robot.h"
#include "protection_coordinator.h"
#include <cstring>
#include "FreeRTOS.h"
#include "task.h"

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
    protection = new ProtectionCoordinator(*this);

    dof6Solver = new DOF6Kinematic(0.165f, 0.0f, 0.170f, 0.117f, 0.0695f, 0.113f);
}


DummyRobot::~DummyRobot()
{
    for (int j = 0; j <= 7; j++)   // [FIX] 0..6 → 0..7（包含地轨）
        delete motorJ[j];

    delete hand;
    delete dof6Solver;
    delete protection;
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
    RebootForGeneration(motionEpoch);
}

void DummyRobot::RebootForGeneration(uint32_t epoch)
{
    if (epoch != motionEpoch || IsStopLocked()) return;
    motorJ[ALL]->Reboot();
    osDelay(500); // waiting for all joints done
    if (epoch != motionEpoch || IsStopLocked()) return;
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

static void FormatMotorNodeList(uint16_t mask, char (&list)[16])
{
    size_t length = 0;
    for (uint8_t node = 1; node <= 8; ++node)
    {
        if (!(mask & (1U << node))) continue;
        if (length) list[length++] = ',';
        list[length++] = static_cast<char>('0' + node);
    }
    if (!length) list[length++] = '-';
    list[length] = '\0';
}

bool DummyRobot::RequestMotorAccelerationSync(StreamSink& response)
{
    taskENTER_CRITICAL();
    if (accSyncPhase != ACC_SYNC_IDLE)
    {
        taskEXIT_CRITICAL();
        return false;
    }
    accSyncPhase = ACC_SYNC_ARMING;
    if (++accSyncRequestId == 0) ++accSyncRequestId; // Zero belongs to ordinary queries.
    accSyncEpoch = motionEpoch;
    accSyncSentMask = accSyncReceivedMask = accSyncInvalidMask = 0;
    accSyncResponse = &response;
    taskEXIT_CRITICAL();

    // Publish RUNNING only after pending has been sent, preserving reply order.
    Respond(response, "ok SYNC_ACC pending");
    taskENTER_CRITICAL();
    accSyncStartedMs = HAL_GetTick();
    accSyncPhase = ACC_SYNC_RUNNING;
    taskEXIT_CRITICAL();
    return true;
}

void DummyRobot::MotorAccelerationCallback(uint8_t node, float value, uint32_t requestId)
{
    if (node < 1 || node > 8) return;
    if (requestId == 0)
    {
        // Preserve the existing startup and single-node query behavior.
        jointAccRuntime[node] = value;
        jointAccLastUpdateMs[node] = HAL_GetTick();
        return;
    }
    const uint16_t bit = static_cast<uint16_t>(1U << node);
    if (accSyncPhase != ACC_SYNC_RUNNING || requestId != accSyncRequestId ||
        !(accSyncSentMask & bit) || (accSyncReceivedMask & bit) ||
        static_cast<uint32_t>(HAL_GetTick() - accSyncStartedMs) >= ACC_SYNC_TIMEOUT_MS) return;
    if (!IsFiniteCommandValue(value) || value <= 0)
        accSyncInvalidMask |= bit;
    else
    {
        accSyncValues[node] = value;
        accSyncUpdateMs[node] = HAL_GetTick();
    }
    accSyncReceivedMask |= bit;
}

void DummyRobot::PollMotorAccelerationSync()
{
    enum Result { NONE, FINISHED, TIMED_OUT, INVALID, CANCELLED };
    Result result = NONE;
    uint8_t node = 0;
    uint16_t missing = 0, invalid = 0;
    uint32_t requestId;
    StreamSink* response;
    taskENTER_CRITICAL();
    if (accSyncPhase != ACC_SYNC_RUNNING)
    {
        taskEXIT_CRITICAL();
        return;
    }
    requestId = accSyncRequestId;
    response = accSyncResponse;
    missing = ACC_SYNC_ALL_NODES & ~accSyncReceivedMask;
    invalid = accSyncInvalidMask;
    if (IsStopLocked() || motionEpoch != accSyncEpoch) result = CANCELLED;
    else if (!missing) result = invalid ? INVALID : FINISHED;
    else if (static_cast<uint32_t>(HAL_GetTick() - accSyncStartedMs) >= ACC_SYNC_TIMEOUT_MS)
        result = TIMED_OUT;

    if (result != NONE)
    {
        if (result == FINISHED)
        {
            // Publish this job's complete sample set before reporting finish.
            for (uint8_t i = 1; i <= 8; ++i)
            {
                jointAccRuntime[i] = accSyncValues[i];
                jointAccLastUpdateMs[i] = accSyncUpdateMs[i];
            }
        }
        accSyncPhase = ACC_SYNC_REPORTING;
    }
    else
    {
        // One nonblocking send attempt per 20ms tick; retry busy CAN next tick.
        for (uint8_t i = 1; i <= 8; ++i)
            if (!(accSyncSentMask & (1U << i))) { node = i; break; }
        if (node) accSyncSentMask |= static_cast<uint16_t>(1U << node);
    }
    taskEXIT_CRITICAL();

    if (result != NONE)
    {
        char missingList[16], invalidList[16];
        FormatMotorNodeList(missing, missingList);
        FormatMotorNodeList(invalid, invalidList);
        if (result == FINISHED) Respond(*response, "ok SYNC_ACC finish");
        else if (result == CANCELLED) Respond(*response, "error SYNC_ACC cancelled");
        else if (result == INVALID) Respond(*response, "error SYNC_ACC invalid nodes=%s", invalidList);
        else Respond(*response, "error SYNC_ACC timeout missing=%s invalid=%s", missingList, invalidList);
        taskENTER_CRITICAL();
        accSyncResponse = nullptr;
        accSyncPhase = ACC_SYNC_IDLE;
        taskEXIT_CRITICAL();
    }
    else if (node)
    {
        CtrlStepMotor* motor = node == 8 ? hand : motorJ[node];
        if (!motor || !motor->TryQueryAcceleration(requestId))
        {
            taskENTER_CRITICAL();
            if (accSyncPhase == ACC_SYNC_RUNNING && accSyncRequestId == requestId)
                accSyncSentMask &= static_cast<uint16_t>(~(1U << node));
            taskEXIT_CRITICAL();
        }
    }
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

void DummyRobot::MoveJoints(DOF6Kinematic::Joint6D_t)
{
    float targets[8], speeds[8]; uint32_t epoch, generation; uint8_t first;
    taskENTER_CRITICAL();
    epoch = motionTaskEpoch; generation = motionCompletionGeneration;
    if (!CanSendMotion(epoch, generation)) { taskEXIT_CRITICAL(); return; }
    for (uint8_t n = 1; n <= 7; ++n) {
        targets[n] = motionMotorTargets[n]; speeds[n] = motionMotorSpeeds[n];
    }
    first = motionSendCursor;
    motionSendCursor = first == 7 ? 1 : first + 1;
    taskEXIT_CRITICAL();
    for (uint8_t offset = 0; offset < 7; ++offset) {
        const uint8_t n = (first - 1 + offset) % 7 + 1;
        taskENTER_CRITICAL();
        const uint16_t bit = 1U << n;
        const bool needed = CanSendMotion(epoch, generation) && !(motionTxOkMask & bit) &&
            !(motionTxPendingMask & bit) && motionTxAttempts[n] < MOTION_TX_MAX_ATTEMPTS &&
            static_cast<int32_t>(HAL_GetTick() - motionRetryAt[n]) >= 0;
        if (needed) motionTxPendingMask |= bit; // Reserve before a wait or an immediate IRQ result.
        taskEXIT_CRITICAL();
        if (!CanSendMotion(epoch, generation)) return;
        if (needed && !motorJ[n]->SendMotionTarget(targets[n], speeds[n], epoch, generation)) {
            taskENTER_CRITICAL();
            if (CanSendMotion(epoch, generation)) {
                motionTxPendingMask &= ~bit;
                motionRetryAt[n] = HAL_GetTick() + 5;
            }
            taskEXIT_CRITICAL();
        }
    }
}



bool DummyRobot::MoveJ(float _j1, float _j2, float _j3, float _j4,
                       float _j5, float _j6, float _j7_mm, float _speed)
{
    return PlanMoveJ(_j1, _j2, _j3, _j4, _j5, _j6, _j7_mm, _speed, motionEpoch);
}

bool DummyRobot::PlanMoveJ(float _j1, float _j2, float _j3, float _j4,
                          float _j5, float _j6, float _j7_mm, float _speed, uint32_t epoch, uint32_t id, const char* kind)
{
    if (IsStopLocked() || epoch != motionEpoch || commandMode > COMMAND_CONTINUES_TRAJECTORY) return false;
    const uint32_t planningGeneration = motionCompletionGeneration;
    if (!IsFiniteCommandValue(_j1) || !IsFiniteCommandValue(_j2) ||
        !IsFiniteCommandValue(_j3) || !IsFiniteCommandValue(_j4) ||
        !IsFiniteCommandValue(_j5) || !IsFiniteCommandValue(_j6) ||
        !IsFiniteCommandValue(_j7_mm)) return false;
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
    taskENTER_CRITICAL();
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

    taskEXIT_CRITICAL();
    float outSpeeds[7] = {0};
    float timeBudget = ComputeSyncSpeeds(deltaRails, SLIDER_CAPS_V, jointAcc,
                                        sliderSpeedRps, outSpeeds);

    // Publish only if this command still belongs to the current motion generation.
    taskENTER_CRITICAL();
    if (IsStopLocked() || epoch != motionEpoch || planningGeneration != motionCompletionGeneration)
    {
        taskEXIT_CRITICAL();
        return false;
    }
    // 写回 dynamicJointSpeeds7：[0]=地轨，[1..6]=J1..J6
    dynamicJointSpeeds7[0] = outSpeeds[0];
    for (int j = 1; j <= 6; j++) {
        dynamicJointSpeeds7[j] = outSpeeds[j];
        dynamicJointSpeeds.a[j - 1] = outSpeeds[j];   // ServoJ 兼容旧字段
    }

    jointsStateFlag = 0;
    ++motionCompletionGeneration;
    motionConfirmedMask = motionTargetReadyMask = motionSentMask = motionProgressMask = 0;
    motionTxPendingMask = motionTxOkMask = motionTxHistoryMask = 0;
    motionSendCursor = 1; motionMovingMask = 0; motionFailureReason = nullptr;
    motionPlanMs = timeBudget * 1000.0f;
    motionTaskId = id ? id : AllocateMotionTaskId();
    motionCurrentLimitedPrintMask = 0; motionCurrentLimitedPrintId = motionTaskId;
    motionTaskEpoch = epoch; motionMode = commandMode; motionKind = kind;
    motionStartedAt = HAL_GetTick(); motionCompletedAt = 0;
    motionRejectedReplies = motionSendCount = motionQueryCount = 0;
    for (uint8_t node = 1; node <= 7; ++node)
    {
        motorJ[node]->state = CtrlStepMotor::RUNNING;
        motionQueryNonce[node] = 0;
        motionMotorSpeeds[node] = outSpeeds[node == 7 ? 0 : node];
        motionReplyAt[node] = motionReplyFlags[node] = 0;
        motionRetryAt[node] = motionStartedAt;
        motionTxAttempts[node] = motionTxFailures[node] = 0;
        motionFirstTxMs[node] = motionFirstReadyMs[node] = MOTION_TIME_MISSING;
        motionFirstPlannerMs[node] = motionFirstPositionMs[node] = motionFirstSettledMs[node] = MOTION_TIME_MISSING;
        const uint8_t index = node == 7 ? 0 : node;
        motionPlanAcceleration[node] = jointAcc[index] > 1e-6f ? jointAcc[index] : JOINT_ACC_FALLBACK[index];
        motionPlanDistance[node] = node == 7 ? fabsf(deltaRails[0]) / RAIL_MM_PER_CIRCLE :
                                   fabsf(deltaRails[node]) * MOTOR_REDUCTION[node] / 360.0f;
        if (motionPlanDistance[node] > static_cast<float>(MotionCompletionProtocol::PositionToleranceSteps(node)) /
                                       MotionCompletionProtocol::STEPS_PER_REV) motionMovingMask |= 1U << node;
        motionReplyPositions[node] = motorJ[node]->angle / 360.0f * motorJ[node]->reduction *
                                    (motorJ[node]->inverseDirection ? -1.0f : 1.0f);
        float motorR = node == 7 ? _j7_mm / RAIL_MM_PER_CIRCLE :
            (targetJointsTmp.a[node - 1] - initPose.a[node - 1]) / 360.0f * motorJ[node]->reduction;
        motionMotorTargets[node] = motorJ[node]->inverseDirection ? -motorR : motorR;
    }
    targetJoints = targetJointsTmp;
    targetRailPos = _j7_mm;
    motionActive = true;
    motionDispatchPending = true;
    motionPhase = MOTION_DISPATCHING;
    BeginMotionRound();
    PollCanMotionTransmit(get_can_ctx(hcan)); // Abort only older task identities, before new dispatch.
    taskEXIT_CRITICAL();

    return true;
}


bool DummyRobot::MoveL(float _x, float _y, float _z, float _a, float _b, float _c)
{
    return PlanMoveL(_x, _y, _z, _a, _b, _c, motionEpoch);
}

bool DummyRobot::PlanMoveL(float _x, float _y, float _z, float _a, float _b, float _c,
                          uint32_t epoch, uint32_t id)
{
    if (IsStopLocked() || epoch != motionEpoch) return false;
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
        return PlanMoveJ(ikSolves.config[indexConfig].a[0], ikSolves.config[indexConfig].a[1],
                     ikSolves.config[indexConfig].a[2], ikSolves.config[indexConfig].a[3],
                     ikSolves.config[indexConfig].a[4], ikSolves.config[indexConfig].a[5],
                     currentRailPos, 0, epoch, id, "MOVEL");
    }

    return false;
}

void DummyRobot::UpdateJointAngles()
{
    const uint8_t node = motionQueryNode;
    if (++motionQueryNode > 7) motionQueryNode = 1;
    taskENTER_CRITICAL();
    const bool confirming = NeedsMotionCompletion();
    const uint32_t generation = motionCompletionGeneration, epoch = motionTaskEpoch;
    const float target = motionMotorTargets[node];
    uint32_t nonce = 0;
    if (confirming && (motionTxOkMask & (1U << node)) && !(motionRoundSeenMask & (1U << node))) {
        const uint32_t now = HAL_GetTick();
        if (!motionQueryNonce[node]) {
            do { motionQueryNonce[node] = ++motionQueryCounter & MotionCompletionProtocol::NONCE_MASK; }
            while (!motionQueryNonce[node]);
            motionQueryGeneration[node] = generation; motionQueryRound[node] = motionRound;
            motionQueryAt[node] = now; motionQueryAttemptAt[node] = now - 35;
        }
        if (now - motionQueryAttemptAt[node] >= 35) {
            nonce = motionQueryNonce[node]; motionQueryAttemptAt[node] = now;
        }
    }
    taskEXIT_CRITICAL();
    if (nonce) {
        if (motorJ[node]->QueryPositionCompletion(target, nonce, epoch, generation)) ++motionQueryCount;
    } else if (!confirming) motorJ[node]->UpdateAngle();
}



void DummyRobot::UpdateJointAnglesCallback()
{
    uint8_t live = 0;
    for (uint8_t n = 1; n <= 6; ++n) {
        currentJoints.a[n - 1] = motorJ[n]->angle + initPose.a[n - 1];
        if (motorJ[n]->state == CtrlStepMotor::FINISH) live |= 1U << n;
    }
    currentRailPos = motorJ[7]->angle / 360.0f * RAIL_MM_PER_CIRCLE;
    if (motorJ[7]->state == CtrlStepMotor::FINISH) live |= 1U << 7;
    // OLED reports progress of this goal, not a continuously reopened old task.
    jointsStateFlag = motionActive || (isEnabled && !IsStopLocked() && motionPhase == MOTION_COMPLETED) ? motionProgressMask : live;
}


uint16_t DummyRobot::FreshMotionCompletionMask()
{
    const uint32_t now = HAL_GetTick();
    for (uint8_t node = 1; node <= 7; ++node)
        if (now - motionReplyAt[node] > MotionCompletionProtocol::FEEDBACK_FRESH_MS)
            motionConfirmedMask &= ~(1U << node);
    return motionConfirmedMask;
}

bool DummyRobot::MotionCompletionCallback(uint8_t node, const uint8_t* data)
{
    using namespace MotionCompletionProtocol;
    if (node < 1 || node > 7 || !NeedsMotionCompletion() || !(data[4] & REPLY_MARKER)) return false;
    if (!(motionTxOkMask & (1U << node))) { ++motionRejectedReplies; return false; }
    const uint32_t nonce = Nonce(data), now = HAL_GetTick();
    if (!nonce || nonce != motionQueryNonce[node] ||
        motionQueryGeneration[node] != motionCompletionGeneration || motionQueryRound[node] != motionRound ||
        now - motionQueryAt[node] > FEEDBACK_FRESH_MS) { ++motionRejectedReplies; return false; }
    motionQueryNonce[node] = 0;
    float position; std::memcpy(&position, data, sizeof(position));
    const uint16_t bit = 1U << node;
    const float error = position - motionMotorTargets[node];
    const float tolerance = static_cast<float>(PositionToleranceSteps(node) + 1) / STEPS_PER_REV;
    const bool finished = MotionCompletionProtocol::Finite(position) && !(data[4] & (DRIVE_BLOCKED | CURRENT_CMD_LIMITED)) &&
                          (data[4] & (TARGET_READY | SETTLED)) == (TARGET_READY | SETTLED) &&
                          error >= -tolerance && error <= tolerance;
    if ((data[4] & CURRENT_CMD_LIMITED) && !(motionReplyFlags[node] & CURRENT_CMD_LIMITED))
        motionCurrentLimitedPrintMask |= bit;
    motionReplyAt[node] = now; motionReplyFlags[node] = data[4]; motionReplyPositions[node] = position;
    if (data[4] & TARGET_READY) {
        motionTargetReadyMask |= bit;
        const uint32_t elapsed = now - motionStartedAt;
        if (motionFirstReadyMs[node] == MOTION_TIME_MISSING) motionFirstReadyMs[node] = elapsed;
        if ((data[4] & PLANNER_READY) && motionFirstPlannerMs[node] == MOTION_TIME_MISSING) motionFirstPlannerMs[node] = elapsed;
        if ((data[4] & POSITION_READY) && motionFirstPositionMs[node] == MOTION_TIME_MISSING) motionFirstPositionMs[node] = elapsed;
        if (finished && motionFirstSettledMs[node] == MOTION_TIME_MISSING) motionFirstSettledMs[node] = elapsed;
    } else {
        motionTargetReadyMask &= ~bit;
        // Retry only an unaccepted goal. Faults or loss of an adopted goal
        // belong to protection, not to retransmission of the old target.
        if (!(data[4] & DRIVE_BLOCKED) && motionFirstReadyMs[node] == MOTION_TIME_MISSING) {
            motionTxOkMask &= ~bit; motionRetryAt[node] = now + 5;
        }
    }
    motionRoundSeenMask |= bit;
    if (finished) {
        motionConfirmedMask |= bit; motionRoundGoodMask |= bit; motionProgressMask |= bit;
    } else { motionConfirmedMask &= ~bit; motionRoundGoodMask &= ~bit; }
    return true;
}

void DummyRobot::MotionTargetQueued(uint8_t node, uint32_t epoch, uint32_t generation)
{
    // Caller holds the CAN send critical section. HAL acceptance is not TXOK.
    if (node < 1 || node > 7 || !CanSendMotion(epoch, generation)) return;
    const uint16_t bit = 1U << node;
    motionSentMask |= bit; motionTxPendingMask |= bit;
    ++motionTxAttempts[node]; ++motionSendCount;
    motionSendAt[node] = HAL_GetTick();
    motionQueryNonce[node] = 0;
    motionTargetReadyMask &= ~bit; motionConfirmedMask &= ~bit;
    motionRoundSeenMask &= ~bit; motionRoundGoodMask &= ~bit;
}

void DummyRobot::MotionTargetResult(uint8_t node, uint32_t epoch, uint32_t generation, bool delivered)
{
    // CAN ISR and receive ISR share priority 5; task-side mutations hold a
    // critical section. Late results release CAN ownership but cannot touch a new task.
    if (node < 1 || node > 7 || !CanSendMotion(epoch, generation)) return;
    const uint16_t bit = 1U << node;
    motionTxPendingMask &= ~bit;
    motionQueryNonce[node] = 0;
    if (delivered) {
        motionTxOkMask |= bit; motionTxHistoryMask |= bit;
        if (motionFirstTxMs[node] == MOTION_TIME_MISSING) motionFirstTxMs[node] = HAL_GetTick() - motionStartedAt;
    } else {
        motionTxOkMask &= ~bit;
        ++motionTxFailures[node];
        // 5/10/15ms backoff plus a rotating send cursor avoids a fixed collision phase.
        motionRetryAt[node] = HAL_GetTick() + 5U * (1U + ((node + motionTxAttempts[node]) % 3U));
    }
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
    CalibrateForGeneration(motionEpoch);
}

void DummyRobot::CalibrateForGeneration(uint32_t epoch)
{
    if (epoch == motionEpoch && protection)
        protection->Command("!CALIBRATION", 12, *usbStreamOutputPtr);
}

bool DummyRobot::IsStopLocked() const
{ return (stopState != NORMAL && stopState != DISABLED) || (protection && protection->Locked()); }
bool DummyRobot::IsCalibrationBusy() const
{ return protection && protection->CalibrationBusy(); }
bool DummyRobot::IsProtectionParameterBusy() const
{ return protection && protection->ParameterBusy(); }
bool DummyRobot::HandleProtectionCommand(const char* cmd, unsigned len, StreamSink& response)
{ return protection && protection->Command(cmd, len, response); }
void DummyRobot::ProtectionCallback(uint8_t node, uint8_t command, const uint8_t* data)
{ if (protection) protection->Receive(node, command, data); }
void DummyRobot::PollProtection() { if (protection) protection->Poll(); }
void DummyRobot::ObserveMotorCommunication(uint8_t node) {
    if (protection) protection->ObserveCommunication(node);
}
void DummyRobot::PollProtectionPrint() { if (protection) protection->Print(); }


void DummyRobot::Homing()
{
    HomingForGeneration(motionEpoch);
}

bool DummyRobot::HomingForGeneration(uint32_t epoch, uint32_t id)
{
    if (epoch != motionEpoch || IsStopLocked()) return false;
    if (!PlanMoveJ(0, 0, 90, 0, 0, 0, 0, 10, epoch, id, "HOME")) return false;
    const uint32_t ownId = motionTaskId;
    return WaitMotionCompletion(ownId, epoch);
}



void DummyRobot::Resting()
{
    RestingForGeneration(motionEpoch);
}

bool DummyRobot::RestingForGeneration(uint32_t epoch, uint32_t id)
{
    if (epoch != motionEpoch || IsStopLocked()) return false;
    if (!PlanMoveJ(REST_POSE.a[0], REST_POSE.a[1], REST_POSE.a[2], REST_POSE.a[3], REST_POSE.a[4], REST_POSE.a[5], 0, 10, epoch, id, "RESET")) return false;
    const uint32_t ownId = motionTaskId;
    return WaitMotionCompletion(ownId, epoch);
}



void DummyRobot::SetEnable(bool _enable)
{
    if (IsStopLocked()) return; // Only ASCII START/DISABLE can release a STOP.
    if (_enable) RequestStart();
    else RequestDisable();
}

void DummyRobot::ResetStopDiagnostics()
{
    // Called with the task critical section held. Diagnostics never release a lock.
    stopPollNode = 1; // Reset once per operation/stage, never per node reply.
    stopRxMask = stopBadPositionMask = 0;
    stopDiagPollCount = 0;
    stopDiagSinceMs = HAL_GetTick();
    stopDiagPending = true;
}

void DummyRobot::RequestSoftStop()
{
    const bool cancelCalibration = IsCalibrationBusy();
    if (protection) protection->Cancel(false);
    taskENTER_CRITICAL();
    isEnabled = false;
    CancelMotion();
    ++motionEpoch;
    stopPerNodeTokens = stopToken == 0;
    stopWithGripper = cancelCalibration;
    stopSyncedMask = 0;
    ++stopToken;
    stopAckMask = 0;
    stopFaultMask = stopFaultPrinted = 0;
    pendingStopNotice = nullptr;
    startPending = false;
    stopAction = SoftStopProtocol::STOP;
    stopHoldingMask = 0;
    stopState = STOPPING;
    stopNeedsSend = true;
    ResetStopDiagnostics();
    tuningHelper.SetTuningFlag(0);
    commandHandler.ClearFifo();
    taskEXIT_CRITICAL();
}

void DummyRobot::RequestStart()
{
    if (protection && protection->InterceptStart()) return;
    taskENTER_CRITICAL();
    if (stopState == STOPPING || stopState == DISABLING)
    {
        startPending = true;
        stopDiagPending = true;
    }
    else if (stopState == STOPPED || stopState == DISABLED)
    {
        stopStartAllEight = stopState == DISABLED || stopWithGripper;
        stopPerNodeTokens = false;
        stopAction = SoftStopProtocol::QUERY;
        stopState = STARTING;
        stopAckMask = 0;
        startPending = false;
        stopNeedsSend = true;
        ResetStopDiagnostics();
    }
    else if (stopState == STARTING) stopDiagPending = true;
    taskEXIT_CRITICAL();
}

void DummyRobot::RequestDisable()
{
    if (protection) protection->Cancel(true);
    taskENTER_CRITICAL();
    isEnabled = false;
    CancelMotion();
    ++motionEpoch;
    ++stopToken;
    stopPerNodeTokens = false;
    stopWithGripper = false;
    stopAckMask = 0;
    startPending = false;
    stopAction = SoftStopProtocol::DISABLE;
    stopFaultMask = stopFaultPrinted = 0;
    pendingStopNotice = nullptr;
    stopState = DISABLING;
    stopNeedsSend = true;
    ResetStopDiagnostics();
    tuningHelper.SetTuningFlag(0);
    commandHandler.ClearFifo();
    taskEXIT_CRITICAL();
}

void DummyRobot::SoftStopCallback(uint8_t node, const uint8_t* data)
{
    if (node < 1 || node > 8) return;
    const uint16_t token = static_cast<uint16_t>(data[6] | (data[7] << 8));
    if (IsStopLocked())
    {
        stopRxPhases[node] = data[4];
        stopRxTokens[node] = token;
        stopRxMask |= static_cast<uint16_t>(1U << node);
    }
    if (stopState == STOPPING && stopPerNodeTokens && (node <= 7 || stopWithGripper) &&
        !(stopSyncedMask & (1U << node)))
    {
        // A restarted master synchronizes each axis independently: a missing
        // node must never prevent responsive axes from receiving STOP.
        stopNodeTokens[node] = static_cast<uint16_t>(token + 1);
        stopSyncedMask |= static_cast<uint16_t>(1U << node);
        stopNeedsSend = true;
        return;
    }
    if (stopState == STARTING && stopAction == SoftStopProtocol::QUERY)
    {
        if (node == 8 && (stopState == STOPPING || !stopStartAllEight)) return;
        float position;
        std::memcpy(&position, data, sizeof(position));
        if (!IsFiniteCommandValue(position) || data[4] == SoftStopProtocol::FAULT) {
            stopBadPositionMask |= static_cast<uint16_t>(1U << node);
            return;
        }
        stopNodeTokens[node] = token;
        stopAckMask |= static_cast<uint16_t>(1U << node);
        return;
    }
    const uint16_t expectedToken = stopPerNodeTokens && (node <= 7 || stopWithGripper) ? stopNodeTokens[node] : stopToken;
    if (token != expectedToken || !IsStopLocked()) return;
    const bool allEight = stopAction == SoftStopProtocol::DISABLE ||
                          (stopState == STARTING && stopStartAllEight) ||
                          (stopState == STOPPING && stopWithGripper);
    if (node == 8 && !allEight) return;
    if (data[4] == SoftStopProtocol::FAULT)
    {
        stopFaultMask |= static_cast<uint16_t>(1U << node);
        stopAckMask &= ~static_cast<uint16_t>(1U << node);
        return;
    }
    if (stopState == STOPPING)
    {
        if (data[4] != SoftStopProtocol::HOLDING && data[4] != SoftStopProtocol::DISABLED) return;
        if (data[4] == SoftStopProtocol::HOLDING) stopHoldingMask |= static_cast<uint16_t>(1U << node);
    }
    else if (data[4] != (stopAction == SoftStopProtocol::DISABLE ?
                        SoftStopProtocol::DISABLED : SoftStopProtocol::NORMAL)) return;
    float position;
    std::memcpy(&position, data, sizeof(position));
    if (!IsFiniteCommandValue(position))
    {
        stopBadPositionMask |= static_cast<uint16_t>(1U << node);
        if (stopAction == SoftStopProtocol::DISABLE && data[4] == SoftStopProtocol::DISABLED)
            stopAckMask |= static_cast<uint16_t>(1U << node);
        return;
    }
    stopBadPositionMask &= ~static_cast<uint16_t>(1U << node);
    stopPositions[node] = position;
    stopAckMask |= static_cast<uint16_t>(1U << node);
}

void DummyRobot::PollSoftStop()
{
    const char* event = nullptr;
    taskENTER_CRITICAL();
    if (IsStopLocked() && stopState != STOPPED) ++stopDiagPollCount;
    const uint16_t expectedMask = (stopAction == SoftStopProtocol::DISABLE ||
                                   (stopState == STARTING && stopStartAllEight) ||
                                   (stopState == STOPPING && stopWithGripper)) ? 0x1FE : 0xFE;
    if (IsStopLocked() && (stopAckMask & expectedMask) == expectedMask)
    {
        if (stopAction == SoftStopProtocol::QUERY)
        {
            bool collision;
            do
            {
                ++stopToken;
                collision = false;
                const int lastNode = (stopState == STARTING && stopStartAllEight) ? 8 : 7;
                for (int node = 1; node <= lastNode; ++node)
                    if (stopNodeTokens[node] == stopToken) collision = true;
            } while (collision);
            stopAction = stopState == STOPPING ? SoftStopProtocol::STOP : SoftStopProtocol::ENABLE_HOLD;
            stopAckMask = 0;
            stopNeedsSend = true;
            ResetStopDiagnostics();
        }
        else if (stopState == STOPPING)
        {
            stopState = STOPPED;
            event = stopHoldingMask ? "STOP locked" : "STOP locked disabled";
            if (startPending && !protection)
            {
                stopAction = SoftStopProtocol::QUERY;
                stopStartAllEight = false;
                stopPerNodeTokens = false;
                stopState = STARTING;
                stopAckMask = 0;
                stopNeedsSend = true;
                startPending = false;
                ResetStopDiagnostics();
            }
        }
        else if (stopState == STARTING)
        {
            for (int node = 1; node <= 7; ++node)
                motorJ[node]->UpdateAngleCallback(stopPositions[node], true);
            if (stopStartAllEight) hand->UpdateAngleCallback(stopPositions[8], true);
            UpdateJointAnglesCallback();
            targetJoints = currentJoints;
            targetRailPos = currentRailPos;
            CancelMotion(); // Motor-local hold persists until a fresh motion command.
            motionPhase = MOTION_IDLE; motionProgressMask = jointsStateFlag = 0xFE;
            motionTaskId = 0; motionKind = "NONE"; motionStartedAt = HAL_GetTick();
            for (uint8_t node = 1; node <= 7; ++node) motionMotorTargets[node] = stopPositions[node];
            commandHandler.ClearFifo();
            isEnabled = true;
            stopState = NORMAL;
            stopAction = 0;
            if (protection) protection->Started();
            event = "Started ok";
        }
        else if (stopState == DISABLING)
        {
            stopState = DISABLED;
            stopAction = 0;
            event = "Disabled ok";
            if (protection) protection->Disabled();
            if (startPending && !protection)
            {
                stopState = STARTING;
                stopAction = SoftStopProtocol::QUERY;
                stopStartAllEight = true;
                stopAckMask = 0;
                stopNeedsSend = true;
                startPending = false;
                event = "Start pending";
                ResetStopDiagnostics();
            }
        }
    }
    const uint32_t epoch = motionEpoch;
    const uint16_t token = stopToken;
    const uint8_t action = stopAction;
    const bool sendFirst = stopNeedsSend;
    stopNeedsSend = false;
    const bool watchingHold = stopState == STOPPED;
    const bool perNodeTokens = stopPerNodeTokens;
    const bool send = stopState == STOPPING || stopState == STARTING || stopState == DISABLING || watchingHold;
    if (event) pendingStopNotice = event;
    taskEXIT_CRITICAL();
    if (!send) return;
    if (protection && !protection->StopSendSlot()) {
        taskENTER_CRITICAL();
        if (epoch == motionEpoch && token == stopToken && action == stopAction) stopNeedsSend |= sendFirst;
        taskEXIT_CRITICAL();
        return;
    }
    const uint32_t now = HAL_GetTick();
    const uint32_t interval = watchingHold ? 40U :
        (protection && protection->FastStopConfirm() ? 5U : 10U);
    if (!sendFirst && now - stopLastSendMs < interval) return;
    const uint8_t lastNode = (action == SoftStopProtocol::DISABLE ||
                              (stopState == STARTING && stopStartAllEight) ||
                              (stopWithGripper && (stopState == STOPPING || watchingHold))) ? 8 : 7;
    const auto submit = [&](uint8_t node) -> bool
    {
        taskENTER_CRITICAL();
        if (epoch != motionEpoch || token != stopToken || action != stopAction) {
            taskEXIT_CRITICAL(); return false;
        }
        const bool needsSync = perNodeTokens && !(stopSyncedMask & (1U << node));
        const uint8_t nodeAction = node == ALL ? action : watchingHold || needsSync ? SoftStopProtocol::QUERY :
            (node == 8 && action == ProtectionProtocol::STOP_PROTECTION ? SoftStopProtocol::STOP : action);
        const uint16_t nodeToken = perNodeTokens ? stopNodeTokens[node] : token;
        stopDiagTxNode = node;
        stopDiagTxBusy = true;
        const bool sent = (node == 8 ? static_cast<CtrlStepMotor*>(hand) : motorJ[node])->SendSoftStop(nodeAction, nodeToken);
        if (sent) {
            stopLastSendMs = now;
            if (protection) protection->StopAttempted();
        } else stopNeedsSend = true; // Keep it pending; a busy mailbox is not an ACK.
        stopDiagTxBusy = false;
        taskEXIT_CRITICAL();
        return sent;
    };
    if (sendFirst && action == SoftStopProtocol::DISABLE)
    {
        submit(ALL); // Preserve prompt all-eight DISABLE, then retry missing nodes.
        return;
    }
    // One CAN submission per slot, fair even when a low-ID node is missing.
    // A token-sync reply only wakes sending; it must not restart from J1.
    for (uint8_t count = 0; count < lastNode; ++count)
    {
        taskENTER_CRITICAL();
        if (epoch != motionEpoch || token != stopToken || action != stopAction) {
            taskEXIT_CRITICAL(); return;
        }
        if (stopPollNode > lastNode) stopPollNode = 1;
        const uint8_t node = stopPollNode++;
        const bool pending = watchingHold || !(stopAckMask & (1U << node));
        taskEXIT_CRITICAL();
        if (pending) { submit(node); return; }
    }
}

void DummyRobot::PollStopPrint()
{
    uint8_t phases[9] = {};
    uint16_t rxTokens[9] = {}, expectedTokens[9] = {};
    taskENTER_CRITICAL();
    const bool printMotionFailure = motionFailurePrintPending;
    const MotionFailureNotice motionFailure = motionFailureNotice;
    motionFailurePrintPending = false;
    const uint16_t limited = motionCurrentLimitedPrintMask;
    const uint32_t limitedId = motionCurrentLimitedPrintId;
    motionCurrentLimitedPrintMask = 0;
    const char* notice = pendingStopNotice;
    pendingStopNotice = nullptr;
    const uint16_t faults = stopFaultMask & ~stopFaultPrinted;
    stopFaultPrinted |= faults;
    const StopState state = stopState;
    const uint8_t action = stopAction;
    const uint16_t expectedMask = (action == SoftStopProtocol::DISABLE ||
                                   (state == STARTING && stopStartAllEight) ||
                                   (state == STOPPING && stopWithGripper)) ? 0x1FE : 0xFE;
    const uint16_t missing = expectedMask & ~stopAckMask;
    const bool printWait = stopDiagPending &&
                          (state == STOPPING || state == STARTING || state == DISABLING) &&
                          static_cast<uint32_t>(HAL_GetTick() - stopDiagSinceMs) >= 2000;
    const uint16_t ack = stopAckMask, received = stopRxMask, badPosition = stopBadPositionMask;
    const uint32_t polls = stopDiagPollCount;
    const bool txBusy = stopDiagTxBusy;
    const uint8_t txNode = stopDiagTxNode;
    if (printWait)
    {
        stopDiagPending = false;
        for (uint8_t node = 1; node <= 8; ++node)
        {
            phases[node] = stopRxPhases[node];
            rxTokens[node] = stopRxTokens[node];
            expectedTokens[node] = stopPerNodeTokens && node <= 7 ? stopNodeTokens[node] : stopToken;
        }
    }
    taskEXIT_CRITICAL();
    // This runs in the display task; USB/UART send waits never block motor control.
    for (uint8_t node = 1; node <= 7; ++node) if (limited & (1U << node))
        Respond(*usbStreamOutputPtr, "MOTION id=%lu J%u CURRENT_CMD_LIMITED", (unsigned long)limitedId, node);
    if (printMotionFailure) {
        Respond(*usbStreamOutputPtr, "MOTION fail id=%lu reason=%s", (unsigned long)motionFailure.id, motionFailure.reason);
        Respond(*usbStreamOutputPtr, "MOTION wait=%03X txok=%03X ready=%03X", motionFailure.wait, motionFailure.txok, motionFailure.ready);
        for (uint8_t node = 1; node <= 7; ++node) if (motionFailure.wait & (1U << node))
            Respond(*usbStreamOutputPtr, "MOTION J%u status f=%02X", node, motionFailure.flags[node]);
    }
    if (notice)
    {
        Respond(*usbStreamOutputPtr, "%s", notice);
        Respond(*uart4StreamOutputPtr, "%s", notice);
    }
    if (faults)
    {
        Respond(*usbStreamOutputPtr, "STOP fault mask=0x%03X", faults);
        Respond(*uart4StreamOutputPtr, "STOP fault mask=0x%03X", faults);
    }
    if (printWait)
    {
        static const char* const states[] = {"NORMAL", "STOPPING", "STOPPED", "STARTING", "DISABLING", "DISABLED"};
        static const char* const actions[] = {"QUERY", "STOP", "RELEASE", "ENABLE_HOLD", "DISABLE", "PROTECT_STOP"};
        // USB is the debugging port. Do not wait on the unused ESP32 UART for diagnostics.
        Respond(*usbStreamOutputPtr, "STOP wait state=%s action=%s", states[state], actions[action]);
        Respond(*usbStreamOutputPtr, "STOP ack=0x%03X wait=0x%03X rx=0x%03X badpos=0x%03X",
                ack, missing, received, badPosition);
        Respond(*usbStreamOutputPtr, "STOP polls=%lu txbusy=%u node=%u",
                static_cast<unsigned long>(polls), static_cast<unsigned>(txBusy), txNode);
        for (uint8_t node = 1; node <= 8; ++node)
        {
            if (!(missing & (1U << node))) continue;
            if (!(received & (1U << node)))
                Respond(*usbStreamOutputPtr, "STOP J%u no 0x0A reply", node);
            else
                Respond(*usbStreamOutputPtr, "STOP J%u phase=%u token=%u expect=%u",
                        node, phases[node], rxTokens[node], expectedTokens[node]);
        }
    }
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
    if (stopState == STOPPING) return true;
    if (stopState != NORMAL) return false;
    if (commandMode <= COMMAND_CONTINUES_TRAJECTORY) return motionActive;
    return (jointsStateFlag & 0xFE) != 0xFE;
}



bool DummyRobot::IsEnabled()
{
    return isEnabled;
}


void DummyRobot::SetCommandMode(uint32_t mode) { TrySetCommandMode(mode); }



uint32_t DummyRobot::CommandHandler::Push(const std::string& cmd, StreamSink* response)
{
    if (cmd.empty() || cmd.size() >= sizeof(QueueItem::command)) return 0xFF;
    QueueItem item{};
    taskENTER_CRITICAL();
    if (context->IsStopLocked() || !context->IsEnabled()) { taskEXIT_CRITICAL(); return 0xFF; }
    item.epoch = context->motionEpoch; item.id = context->AllocateMotionTaskId();
    item.mode = context->commandMode; item.response = response;
    std::memcpy(item.command, cmd.c_str(), cmd.size() + 1);
    const osStatus_t status = osMessageQueuePut(commandFifo, &item, 0U, 0U);
    taskEXIT_CRITICAL();
    if (status != osOK) return 0xFF;
    Announce(item);
    return GetSpace();
}


uint32_t DummyRobot::CommandHandler::PushSystem(const std::string& cmd, StreamSink* response)
{
    if (cmd != "!HOME" && cmd != "!RESET" && cmd != "!CALIBRATION") return 0xFF;
    QueueItem item{};
    taskENTER_CRITICAL();
    if (context->IsStopLocked() || (!context->IsEnabled() && cmd != "!CALIBRATION"))
        { taskEXIT_CRITICAL(); return 0xFF; }
    ++context->motionEpoch; context->CancelMotion(); ClearFifo();
    item.epoch = context->motionEpoch; item.id = context->AllocateMotionTaskId();
    item.mode = context->commandMode; item.response = response;
    std::memcpy(item.command, cmd.c_str(), cmd.size() + 1);
    const osStatus_t status = osMessageQueuePut(commandFifo, &item, 1U, 0U);
    taskEXIT_CRITICAL();
    if (status != osOK) return 0xFF;
    Announce(item);
    return GetSpace();
}



void DummyRobot::CommandHandler::EmergencyStop()
{
    context->RequestSoftStop();
}


std::string DummyRobot::CommandHandler::Pop(uint32_t timeout)
{
    QueueItem item{};
    if (osMessageQueueGet(commandFifo, &item, nullptr, timeout) != osOK) return {};
    poppedEpoch = item.epoch; poppedId = item.id; poppedMode = item.mode; poppedResponse = item.response;
    return std::string{item.command};
}



uint32_t DummyRobot::CommandHandler::GetSpace()
{
    return osMessageQueueGetSpace(commandFifo);
}


uint32_t DummyRobot::CommandHandler::ParseCommand(const std::string& cmd)
{
    if (cmd.empty()) return GetSpace();
    const uint32_t epoch = poppedId ? poppedEpoch : context->motionEpoch;
    const uint32_t id = poppedId ? poppedId : context->AllocateMotionTaskId();
    const CommandMode mode = poppedId ? poppedMode : context->commandMode;
    StreamSink* response = poppedResponse ? poppedResponse : usbStreamOutputPtr;
    const bool announced = poppedResponse != nullptr;
    poppedId = 0; poppedResponse = nullptr;
    ExecutionScope executing(*this, epoch);
    while (announced && !IsAnnounced(id, response)) osDelay(1);
    const char* kind = Name(cmd);
    auto cancelled = [&]() { return epoch != context->motionEpoch || context->IsStopLocked() || !context->IsEnabled(); };
    auto failure = [&](const char* reason) { Respond(*response, "error %s id=%lu reason=%s", kind, (unsigned long)id, reason); };
    if (cancelled()) { failure("CANCELLED"); return GetSpace(); }
    const uint32_t planningGeneration = context->motionCompletionGeneration;
    bool planned = false, wait = mode != COMMAND_TARGET_POINT_INTERRUPTABLE;
    if (cmd == "!HOME" || cmd == "!RESET") {
        planned = cmd == "!HOME" ? context->HomingForGeneration(epoch, id) : context->RestingForGeneration(epoch, id);
        wait = true;
    } else if (cmd == "!CALIBRATION") {
        context->CalibrateForGeneration(epoch); return GetSpace(); // ASCII calibration belongs to the protection coordinator.
    } else if (cmd[0] == '>' || cmd[0] == '&') {
        float j[7], speed = 0; int count;
        count = sscanf(cmd.c_str() + 1, "%f,%f,%f,%f,%f,%f,%f,%f", &j[0], &j[1], &j[2], &j[3], &j[4], &j[5], &j[6], &speed);
        if (count >= 7 && IsFiniteCommandValue(speed) && mode <= COMMAND_CONTINUES_TRAJECTORY) {
            planned = context->PlanMoveJ(j[0], j[1], j[2], j[3], j[4], j[5], j[6], speed, epoch, id, kind);
            if (planned && count == 8) context->SetMotionSpeed(speed, epoch);
        }
    } else if (cmd[0] == '@') {
        float p[6], speed = 0;
        const int count = sscanf(cmd.c_str() + 1, "%f,%f,%f,%f,%f,%f,%f", &p[0], &p[1], &p[2], &p[3], &p[4], &p[5], &speed);
        if (count >= 6 && IsFiniteCommandValue(speed) && mode <= COMMAND_CONTINUES_TRAJECTORY) {
            if (count == 7) context->SetMotionSpeed(speed, epoch);
            planned = context->PlanMoveL(p[0], p[1], p[2], p[3], p[4], p[5], epoch, id);
        }
    }
    if (context->motionTaskId == id && context->motionPhase == MOTION_FAILED && context->motionFailureReason) {
        failure(context->motionFailureReason); return GetSpace();
    }
    if (!planned) {
        const bool aborted = cancelled() || planningGeneration != context->motionCompletionGeneration;
        if (!aborted) {
            taskENTER_CRITICAL();
            if (!context->motionActive && epoch == context->motionEpoch && !context->IsStopLocked()) {
                context->motionTaskId = id; context->motionTaskEpoch = epoch;
                context->motionKind = kind; context->motionMode = mode; context->motionPhase = MOTION_FAILED;
                context->motionStartedAt = context->motionCompletedAt = HAL_GetTick();
            }
            taskEXIT_CRITICAL();
        }
        failure(aborted ? "CANCELLED" : mode > COMMAND_CONTINUES_TRAJECTORY ? "MODE_UNSUPPORTED" : "PLAN_FAILED"); return GetSpace();
    }
    if (wait && !context->WaitMotionCompletion(id, epoch)) {
        failure(context->motionTaskId == id && context->motionPhase == MOTION_FAILED && context->motionFailureReason ?
                context->motionFailureReason : "CANCELLED"); return GetSpace();
    }
    taskENTER_CRITICAL();
    const bool successful = !cancelled() && context->motionTaskId == id &&
        (!wait || context->motionPhase == MOTION_COMPLETED);
    const uint32_t elapsed = context->motionCompletedAt - context->motionStartedAt;
    taskEXIT_CRITICAL();
    if (!successful) failure("CANCELLED");
    else if (wait) Respond(*response, "ok %s id=%lu ms=%lu", kind, (unsigned long)id, (unsigned long)elapsed);
    else Respond(*response, "accepted %s id=%lu", kind, (unsigned long)id);
    return GetSpace();
}



void DummyRobot::CommandHandler::ClearFifo()
{
    QueueItem item{};
    while (osMessageQueueGet(commandFifo, &item, nullptr, 0) == osOK) {
        const CancelledItem notice{item.id, item.response, Name(item.command)};
        if (item.response && osMessageQueuePut(cancelledFifo, &notice, 0U, 0U) != osOK) ++droppedCancellationNotices;
    }
}


void DummyRobot::SetMotionSpeed(float speed, uint32_t epoch)
{
    taskENTER_CRITICAL();
    if (!IsStopLocked() && epoch == motionEpoch) SetJointSpeed(speed);
    taskEXIT_CRITICAL();
}


void DummyRobot::TuningHelper::SetTuningFlag(uint8_t _flag)
{
    tuningFlag = _flag;
}


void DummyRobot::TuningHelper::Tick(uint32_t _timeMillis)
{
    if (!context->IsEnabled() || context->IsStopLocked()) return;
    time += PI * 2 * frequency * (float) _timeMillis / 1000.0f;
    float delta = amplitude * sinf(time);

    for (int i = 1; i <= 6; i++)
        if (context->IsEnabled() && !context->IsStopLocked() && (tuningFlag & (1 << (i - 1))))
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

void DummyRobot::BeginMotionRound()
{
    ++motionRound; motionRoundAt = HAL_GetTick();
    motionRoundSeenMask = motionRoundGoodMask = 0;
    for (uint8_t n = 1; n <= 7; ++n) motionQueryNonce[n] = 0;
}

void DummyRobot::ObserveMotionFeedback(uint8_t node, float position, bool finished)
{
    if (node < 1 || node > 7 || !NeedsMotionCompletion()) return;
    const float error = position - motionMotorTargets[node];
    const float tolerance = static_cast<float>(MotionCompletionProtocol::PositionToleranceSteps(node) + 1) /
                            MotionCompletionProtocol::STEPS_PER_REV;
    if (!MotionCompletionProtocol::Finite(position) || !finished || error < -tolerance || error > tolerance) {
        motionConfirmedMask &= ~(1U << node); motionRoundGoodMask &= ~(1U << node);
        // A reply sampled before this contradictory live feedback cannot restore proof.
        motionQueryNonce[node] = 0;
    }
}

void DummyRobot::PollMotionCompletion()
{
    PollCanMotionTransmit(get_can_ctx(hcan));
    taskENTER_CRITICAL();
    if (motionActive && (!isEnabled || IsStopLocked() || motionTaskEpoch != motionEpoch)) CancelMotion();
    if (NeedsMotionCompletion()) {
        const char* failure = nullptr;
        uint16_t wait = 0;
        for (uint8_t n = 1; n <= 7; ++n)
            if (motionReplyFlags[n] & MotionCompletionProtocol::DRIVE_BLOCKED) wait |= 1U << n;
        if (wait) failure = "MOTOR_DRIVE_BLOCKED";
        else {
            for (uint8_t n = 1; n <= 7; ++n)
                if (motionFirstReadyMs[n] != MOTION_TIME_MISSING &&
                    !(motionReplyFlags[n] & MotionCompletionProtocol::TARGET_READY)) wait |= 1U << n;
            if (wait) failure = "TARGET_CHANGED";
        }
        if (!failure) {
            for (uint8_t n = 1; n <= 7; ++n)
                if (!(motionTxOkMask & (1U << n)) && !(motionTxPendingMask & (1U << n)) &&
                    motionTxAttempts[n] >= MOTION_TX_MAX_ATTEMPTS) wait |= 1U << n;
            if (wait) failure = "TARGET_TX_RETRIES";
        }
        if (!failure && HAL_GetTick() - motionStartedAt >= MOTION_TX_TIMEOUT_MS && motionTxHistoryMask != 0xFE) {
            wait = 0xFE & ~motionTxHistoryMask; failure = "TARGET_TX_TIMEOUT";
        } else if (!failure && HAL_GetTick() - motionStartedAt >= MOTION_READY_TIMEOUT_MS && motionTargetReadyMask != 0xFE) {
            wait = 0xFE & ~motionTargetReadyMask; failure = "TARGET_READY_TIMEOUT";
        }
        if (failure) {
            FailMotionDispatch(motionTaskEpoch, motionCompletionGeneration, failure, wait);
            taskEXIT_CRITICAL(); return;
        }
        motionDispatchPending = (motionTargetReadyMask & 0xFE) != 0xFE;
        motionPhase = motionDispatchPending ? MOTION_DISPATCHING : MOTION_RUNNING;
        if (!motionDispatchPending && motionRoundSeenMask == 0xFE && motionRoundGoodMask == 0xFE &&
            FreshMotionCompletionMask() == 0xFE) {
            motionPhase = MOTION_COMPLETED; motionCompletedAt = HAL_GetTick();
            motionActive = motionDispatchPending = false; motionProgressMask = jointsStateFlag = 0xFE;
            for (uint8_t n = 1; n <= 7; ++n) motionQueryNonce[n] = 0;
        } else if (motionRoundSeenMask == 0xFE ||
                   HAL_GetTick() - motionRoundAt >= MotionCompletionProtocol::FEEDBACK_FRESH_MS) BeginMotionRound();
    }
    taskEXIT_CRITICAL();
}

void DummyRobot::FailMotionDispatch(uint32_t epoch, uint32_t generation, const char* reason, uint16_t wait)
{
    taskENTER_CRITICAL();
    if (CanSendMotion(epoch, generation)) {
        motionFailureReason = reason;
        motionFailureNotice = {motionTaskId, reason, wait, motionTxOkMask, motionTargetReadyMask};
        for (uint8_t n = 1; n <= 7; ++n) motionFailureNotice.flags[n] = motionReplyFlags[n];
        motionFailurePrintPending = true;
        motionPhase = MOTION_FAILED; motionCompletedAt = HAL_GetTick(); motionActive = false;
        // Internal failure preserves confirmed or in-flight local protection.
        // User STOP/DISABLE still cancel it through RequestSoftStop/Disable.
        if (protection) protection->MotionDispatchFailed();
        else RequestSoftStop();
    }
    taskEXIT_CRITICAL();
}

void DummyRobot::CancelMotion()
{
    taskENTER_CRITICAL();
    if (motionActive) { motionPhase = MOTION_CANCELLED; motionCompletedAt = HAL_GetTick(); }
    motionActive = motionDispatchPending = false;
    ++motionCompletionGeneration;
    motionTxPendingMask = 0;
    motionConfirmedMask = motionRoundSeenMask = motionRoundGoodMask = 0;
    for (uint8_t n = 1; n <= 7; ++n) motionQueryNonce[n] = 0;
    PollCanMotionTransmit(get_can_ctx(hcan));
    taskEXIT_CRITICAL();
}

bool DummyRobot::WaitMotionCompletion(uint32_t id, uint32_t epoch)
{
    while (epoch == motionEpoch && !IsStopLocked() && isEnabled && motionTaskId == id && motionActive) osDelay(5);
    return epoch == motionEpoch && !IsStopLocked() && isEnabled &&
           motionTaskId == id && motionPhase == MOTION_COMPLETED;
}

bool DummyRobot::TrySetCommandMode(uint32_t mode)
{
    taskENTER_CRITICAL();
    const bool allowed = !IsStopLocked() && !motionActive && !commandHandler.IsBusy() &&
        mode >= COMMAND_TARGET_POINT_SEQUENTIAL && mode <= COMMAND_MOTOR_TUNING;
    if (allowed) commandMode = static_cast<CommandMode>(mode);
    taskEXIT_CRITICAL();
    return allowed;
}

void DummyRobot::PrintMotionStatus(StreamSink& response)
{
    uint32_t id, elapsed, rejected, sends, queries, ages[8];
    uint32_t tx[8], readyAt[8], planner[8], position[8], settled[8];
    uint16_t ready, live, progress, txok, pending, moving; MotionPhase phase; CommandMode mode;
    float goals[8], actual[8], speeds[8], acceleration[8], distance[8], planMs;
    uint8_t flags[8], attempts[8], failures[8]; const char* kind;
    taskENTER_CRITICAL();
    id = motionTaskId; kind = motionKind; phase = motionPhase; mode = motionMode;
    elapsed = (phase == MOTION_COMPLETED || phase == MOTION_CANCELLED || phase == MOTION_FAILED ?
               motionCompletedAt : HAL_GetTick()) - motionStartedAt;
    ready = motionTargetReadyMask; live = motionConfirmedMask; progress = motionProgressMask;
    txok = motionTxOkMask; pending = motionTxPendingMask; moving = motionMovingMask; planMs = motionPlanMs;
    rejected = motionRejectedReplies; sends = motionSendCount; queries = motionQueryCount;
    for (uint8_t n = 1; n <= 7; ++n) {
        goals[n] = motionMotorTargets[n];
        actual[n] = motorJ[n]->angle / 360.0f * motorJ[n]->reduction *
                    (motorJ[n]->inverseDirection ? -1.0f : 1.0f);
        flags[n] = motionReplyFlags[n]; ages[n] = HAL_GetTick() - motionReplyAt[n];
        tx[n] = motionFirstTxMs[n]; readyAt[n] = motionFirstReadyMs[n];
        planner[n] = motionFirstPlannerMs[n]; position[n] = motionFirstPositionMs[n]; settled[n] = motionFirstSettledMs[n];
        speeds[n] = motionMotorSpeeds[n]; acceleration[n] = motionPlanAcceleration[n]; distance[n] = motionPlanDistance[n];
        attempts[n] = motionTxAttempts[n]; failures[n] = motionTxFailures[n];
    }
    taskEXIT_CRITICAL();
    Respond(response, "MOTION id=%lu kind=%s phase=%u mode=%u ms=%lu", (unsigned long)id, kind,
            unsigned(phase), unsigned(mode), (unsigned long)elapsed);
    Respond(response, "MOTION ready=%03X confirmed=%03X progress=%03X", ready, live, progress);
    Respond(response, "MOTION send=%lu query=%lu reject=%lu", (unsigned long)sends,
            (unsigned long)queries, (unsigned long)rejected);
    Respond(response, "MOTION plan_ms=%.1f moving=%03X txok=%03X pending=%03X", double(planMs), moving, txok, pending);
    const auto spread = [&](const uint32_t* times) -> uint32_t {
        uint32_t first = MOTION_TIME_MISSING, last = 0;
        if (!moving) return 0;
        for (uint8_t n = 1; n <= 7; ++n) if (moving & (1U << n)) {
            if (times[n] == MOTION_TIME_MISSING) return MOTION_TIME_MISSING;
            if (times[n] < first) first = times[n];
            if (times[n] > last) last = times[n];
        }
        return last - first;
    };
    Respond(response, "MOTION spread tx=%ld settled_obs=%ld", (long)spread(tx), (long)spread(settled));
    CAN_context* ctx = get_can_ctx(hcan);
    if (ctx) Respond(response, "MOTION CAN alst=%lu terr=%lu busy=%lu", (unsigned long)ctx->tx_arbitration_losses,
                     (unsigned long)ctx->tx_transmission_errors, (unsigned long)ctx->tx_busy_errors);
    if (commandHandler.CancellationNoticeLosses())
        Respond(response, "MOTION cancelled_drop=%lu", (unsigned long)commandHandler.CancellationNoticeLosses());
    for (uint8_t n = 1; n <= 7; ++n) {
        Respond(response, "MOTION J%u goal=%.4f real=%.4f f=%02X age=%lu", n,
                double(goals[n]), double(actual[n]), flags[n], (unsigned long)ages[n]);
        Respond(response, "MOTION J%u tx=%ld ready=%ld try=%u fail=%u", n,
                (long)tx[n], (long)readyAt[n], attempts[n], failures[n]);
        Respond(response, "MOTION J%u plan=%ld pos=%ld settled=%ld", n,
                (long)planner[n], (long)position[n], (long)settled[n]);
        Respond(response, "MOTION J%u v=%.5f a=%.3f d=%.4f", n,
                double(speeds[n]), double(acceleration[n]), double(distance[n]));
    }
}

bool DummyRobot::CommandHandler::IsAnnounced(uint32_t id, StreamSink* response) const
{
    const uint8_t slot = response == usbStreamOutputPtr ? 0 : response == uart4StreamOutputPtr ? 1 : 2;
    return static_cast<uint32_t>(announcedId[slot] - id) < 0x80000000U;
}
void DummyRobot::CommandHandler::Announce(const QueueItem& item)
{
    if (!item.response) return;
    static const char* const modes[] = {"?", "SEQ", "INT", "TRJ", "TUN"};
    Respond(*item.response, "queued %s id=%lu mode=%s", Name(item.command), (unsigned long)item.id, modes[item.mode]);
    const uint8_t slot = item.response == usbStreamOutputPtr ? 0 : item.response == uart4StreamOutputPtr ? 1 : 2;
    taskENTER_CRITICAL(); announcedId[slot] = item.id; taskEXIT_CRITICAL();
}
void DummyRobot::CommandHandler::PollCancelled()
{
    CancelledItem item{};
    if (osMessageQueueGet(cancelledFifo, &item, nullptr, 0) != osOK) return;
    if (!IsAnnounced(item.id, item.response)) {
        if (osMessageQueuePut(cancelledFifo, &item, 0U, 0U) != osOK) ++droppedCancellationNotices;
        return;
    }
    Respond(*item.response, "error %s id=%lu reason=CANCELLED", item.kind, (unsigned long)item.id);
}
