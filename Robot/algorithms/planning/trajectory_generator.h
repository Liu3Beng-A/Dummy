/**
 * @file trajectory_generator.h
 * @brief 轨迹生成器引擎：基于 SCurvePlanner 的关节空间/笛卡尔空间插补 (MoveJ / MoveL)
 *
 * 纯逻辑层，不直接包含硬件头文件；依赖 s_curve_planner、6dof_kinematic、quaternion_math。
 */

#ifndef ROBOT_ALGORITHMS_PLANNING_TRAJECTORY_GENERATOR_H
#define ROBOT_ALGORITHMS_PLANNING_TRAJECTORY_GENERATOR_H

#include "algorithms/planning/s_curve_planner.h"
#include "algorithms/kinematic/6dof_kinematic.h"
#include "algorithms/kinematic/quaternion_math.h"

class TrajectoryGenerator {
public:
    enum State {
        IDLE,
        MOVE_J,
        MOVE_L
    };

    TrajectoryGenerator() = default;

    /** 设置逆运动学求解器，MoveL 前必须调用 */
    void SetKinematicSolver(DOF6Kinematic* solver);

    /**
     * 启动关节空间直线插补 (MoveJ)。
     * 以 6 轴最大关节差作为规划距离，生成 S 型时间参数化。
     * @return 若 max_delta 接近 0 或规划失败则 false
     */
    bool StartMoveJ(const DOF6Kinematic::Joint6D_t& start,
                    const DOF6Kinematic::Joint6D_t& target,
                    float v_max, float a_max, float j_max);

    /**
     * 启动笛卡尔空间直线插补 (MoveL)。
     * 以 XYZ 欧氏距离规划 S 型；姿态用四元数 Slerp；纯旋转时强制等效距离使曲线可跑。
     * @return solver_ 为空或规划失败则 false
     */
    bool StartMoveL(const DOF6Kinematic::Pose6D_t& start,
                    const DOF6Kinematic::Pose6D_t& target,
                    float v_max, float a_max, float j_max);

    /**
     * 核心更新：由 RTOS 定时器周期性调用。
     * @param dt_seconds 本周期时间步长（秒）
     * @param out_joints 输出当前插补关节角（MoveL 时也作为 IK 参考前馈）
     * @return true 表示正在运动且 out_joints 有效；IDLE 时返回 false
     */
    bool Update(float dt_seconds, DOF6Kinematic::Joint6D_t& out_joints);

    /** 当前是否处于运动状态（非 IDLE） */
    bool IsMoving() const;

    /** 强行停止，状态切回 IDLE */
    void Stop();

private:
    SCurvePlanner planner_;
    volatile State currentState_{IDLE};
    float currentTime_{0.0f};
    DOF6Kinematic::Joint6D_t startJoints_{};
    DOF6Kinematic::Joint6D_t targetJoints_{};

    DOF6Kinematic* solver_{nullptr};
    DOF6Kinematic::Pose6D_t startPose_{};
    DOF6Kinematic::Pose6D_t targetPose_{};
    Quaternion_t startQ_{};
    Quaternion_t targetQ_{};
};

#endif /* ROBOT_ALGORITHMS_PLANNING_TRAJECTORY_GENERATOR_H */
