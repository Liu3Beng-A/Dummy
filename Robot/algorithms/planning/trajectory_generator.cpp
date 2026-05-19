/**
 * @file trajectory_generator.cpp
 * @brief 轨迹生成器实现：MoveJ 关节空间 + MoveL 笛卡尔空间 S 型插补
 */

#include "algorithms/planning/trajectory_generator.h"
#include <cmath>

namespace {

constexpr float kMinDelta = 1.0e-6f;
constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;
constexpr float kPureRotMinDist = 10.0f;
constexpr float kOrientationChangeDotThreshold = 0.9999f;
/** 单帧最大关节角跳变 (度)，超限视为奇异点/IK 翻转并急停；放宽至 12 避免物理滞后与 IK 多解切换误触 */
constexpr float kMaxJointJumpPerTick = 12.0f;
/** 单帧推荐跳变阈值 (度)，超过此值但未达急停阈值时触发软锁定（保上一个解） */
constexpr float kSoftLockJointJumpPerTick = 1.0f;
/** IK 多解切换时允许的最大单轴跳变 (度)，超过则强制锁定上一解防止奇异翻转 */
constexpr float kMaxAxisJumpForSolutionSwitch = 5.0f;
/** IK 多解筛选时预选集的大小上限 */
constexpr int kIKPreSelectCount = 3;

float maxJointDelta(const DOF6Kinematic::Joint6D_t& start,
                   const DOF6Kinematic::Joint6D_t& target) {
    float maxD = 0.0f;
    for (int i = 0; i < 6; ++i) {
        float d = fabsf(target.a[i] - start.a[i]);
        if (d > maxD) maxD = d;
    }
    return maxD;
}

float jointDeltaSum(const DOF6Kinematic::Joint6D_t& from,
                    const DOF6Kinematic::Joint6D_t& to) {
    float sum = 0.0f;
    for (int i = 0; i < 6; ++i)
        sum += fabsf(to.a[i] - from.a[i]);
    return sum;
}

float jointDeltaMax(const DOF6Kinematic::Joint6D_t& from,
                    const DOF6Kinematic::Joint6D_t& to) {
    float maxD = 0.0f;
    for (int i = 0; i < 6; ++i) {
        float d = fabsf(to.a[i] - from.a[i]);
        if (d > maxD) maxD = d;
    }
    return maxD;
}

float quatDot(const Quaternion_t& a, const Quaternion_t& b) {
    return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
}

}  // namespace

void TrajectoryGenerator::SetKinematicSolver(DOF6Kinematic* solver) {
    solver_ = solver;
}

bool TrajectoryGenerator::StartMoveJ(const DOF6Kinematic::Joint6D_t& start,
                                     const DOF6Kinematic::Joint6D_t& target,
                                     float v_max, float a_max, float j_max) {
    float max_delta = maxJointDelta(start, target);
    if (max_delta < kMinDelta)
        return false;

    if (max_delta < 0.001f) {
        currentState_ = IDLE;
        return true;
    }

    float j_calc = (j_max > 0.0f) ? j_max : (a_max * 15.0f);

    if (!planner_.generate(max_delta, v_max, a_max, j_calc))
        return false;
    startJoints_ = start;
    targetJoints_ = target;
    currentTime_ = 0.0f;
    currentState_ = MOVE_J;
    return true;
}

bool TrajectoryGenerator::StartMoveL(const DOF6Kinematic::Pose6D_t& start,
                                     const DOF6Kinematic::Pose6D_t& target,
                                     float v_max, float a_max, float j_max) {
    if (solver_ == nullptr)
        return false;

    float dx = target.X - start.X;
    float dy = target.Y - start.Y;
    float dz = target.Z - start.Z;
    float dist = sqrtf(dx * dx + dy * dy + dz * dz);

    float rx0 = start.A * kDegToRad;
    float ry0 = start.B * kDegToRad;
    float rz0 = start.C * kDegToRad;
    float rx1 = target.A * kDegToRad;
    float ry1 = target.B * kDegToRad;
    float rz1 = target.C * kDegToRad;
    startQ_ = EulerToQuaternion(rx0, ry0, rz0);
    targetQ_ = EulerToQuaternion(rx1, ry1, rz1);

    if (dist < 1.0f && quatDot(startQ_, targetQ_) < kOrientationChangeDotThreshold)
        dist = kPureRotMinDist;

    if (dist < 0.001f && fabsf(acosf(quatDot(startQ_, targetQ_))) < 0.001f) {
        currentState_ = IDLE;
        return true;
    }

    float j_calc = (j_max > 0.0f) ? j_max : (a_max * 15.0f);

    if (!planner_.generate(dist, v_max, a_max, j_calc))
        return false;

    startPose_ = start;
    targetPose_ = target;
    currentTime_ = 0.0f;
    currentState_ = MOVE_L;
    return true;
}

bool TrajectoryGenerator::Update(float dt_seconds, DOF6Kinematic::Joint6D_t& out_joints) {
    if (currentState_ == IDLE)
        return false;

    currentTime_ += dt_seconds;
    float s = planner_.getS(currentTime_);

    if (currentState_ == MOVE_J) {
        for (int i = 0; i < 6; ++i)
            out_joints.a[i] = startJoints_.a[i] + s * (targetJoints_.a[i] - startJoints_.a[i]);
    } else if (currentState_ == MOVE_L && solver_ != nullptr) {
        float X = startPose_.X + s * (targetPose_.X - startPose_.X);
        float Y = startPose_.Y + s * (targetPose_.Y - startPose_.Y);
        float Z = startPose_.Z + s * (targetPose_.Z - startPose_.Z);
        Quaternion_t currQ = Slerp(startQ_, targetQ_, s);

        DOF6Kinematic::Pose6D_t currPose;
        currPose.X = X;
        currPose.Y = Y;
        currPose.Z = Z;
        QuaternionToRotMat(currQ, currPose.R);
        currPose.hasR = true;

        DOF6Kinematic::IKSolves_t ikSolves;
        solver_->SolveIK(currPose, out_joints, ikSolves);

        DOF6Kinematic::Joint6D_t prev_joints = out_joints;

        int preSelectIdx[kIKPreSelectCount] = {0, 0, 0};
        float preSelectSum[kIKPreSelectCount] = {1e30f, 1e30f, 1e30f};

        for (int k = 0; k < 8; ++k) {
            float sum = jointDeltaSum(prev_joints, ikSolves.config[k]);
            if (sum < preSelectSum[0]) {
                preSelectSum[2] = preSelectSum[1];
                preSelectIdx[2] = preSelectIdx[1];
                preSelectSum[1] = preSelectSum[0];
                preSelectIdx[1] = preSelectIdx[0];
                preSelectSum[0] = sum;
                preSelectIdx[0] = k;
            } else if (sum < preSelectSum[1]) {
                preSelectSum[2] = preSelectSum[1];
                preSelectIdx[2] = preSelectIdx[1];
                preSelectSum[1] = sum;
                preSelectIdx[1] = k;
            } else if (sum < preSelectSum[2]) {
                preSelectSum[2] = sum;
                preSelectIdx[2] = k;
            }
        }

        int bestIdx = preSelectIdx[0];
        float bestSum = preSelectSum[0];
        bool lockToPrev = false;

        if (preSelectSum[1] < 1e29f) {
            float ratio = preSelectSum[0] / (preSelectSum[1] + 1e-6f);
            if (ratio > 0.3f) {
                lockToPrev = true;
            }
        }

        if (!lockToPrev) {
            for (int pi = 0; pi < kIKPreSelectCount && preSelectSum[pi] < 1e29f; ++pi) {
                int candIdx = preSelectIdx[pi];
                float maxAxisJump = jointDeltaMax(prev_joints, ikSolves.config[candIdx]);
                if (maxAxisJump <= kMaxAxisJumpForSolutionSwitch) {
                    bestIdx = candIdx;
                    bestSum = preSelectSum[pi];
                    break;
                }
            }
        } else {
            bestIdx = preSelectIdx[0];
            bestSum = preSelectSum[0];
        }

        DOF6Kinematic::Joint6D_t candidate = ikSolves.config[bestIdx];

        float max_jump = jointDeltaMax(prev_joints, candidate);

        if (max_jump > kMaxJointJumpPerTick) {
            currentState_ = IDLE;
            return false;
        }

        out_joints = candidate;
    }

    if (currentTime_ >= planner_.getTotalTime())
        currentState_ = IDLE;

    return true;
}

bool TrajectoryGenerator::IsMoving() const {
    return currentState_ != IDLE;
}

void TrajectoryGenerator::Stop() {
    currentState_ = IDLE;
}
