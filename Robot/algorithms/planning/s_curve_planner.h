/**
 * @file s_curve_planner.h
 * @brief 归一化 S 型速度规划器 (S-Curve Profile Planner)
 *
 * 纯数学规划器，7 段式 Jerk 限制曲线。
 * 不依赖 FreeRTOS、HAL 或 STM32 头文件，仅依赖 <cmath>。
 * 所有计算与成员均为 float（单精度）。
 */

#ifndef ROBOT_ALGORITHMS_PLANNING_S_CURVE_PLANNER_H
#define ROBOT_ALGORITHMS_PLANNING_S_CURVE_PLANNER_H

class SCurvePlanner {
public:
    SCurvePlanner() = default;

    /**
     * 根据给定运动参数计算 S 型曲线各段时间（加速段、匀速段、减速段等）。
     * @param distance 需要规划的总距离（取绝对值参与计算）
     * @param v_max    允许的最大速度
     * @param a_max    允许的最大加速度
     * @param j_max    允许的最大加加速度 (Jerk)
     * @return 规划是否成功（参数不合理时返回 false）
     */
    bool generate(float distance, float v_max, float a_max, float j_max);

    /** 获取总运动时间 T */
    float getTotalTime() const;

    /**
     * 核心：输入当前时间 t (0 <= t <= T)，返回归一化位置 s in [0, 1]。
     * t < 0 返回 0.0；t > T 返回 1.0。
     */
    float getS(float t) const;

    /** 当前时间 t 对应的速度 v（用于前馈等） */
    float getV(float t) const;

    /** 当前时间 t 对应的加速度 a（用于前馈等） */
    float getA(float t) const;

private:
    /** 总运动时间 */
    float T_{0.0f};

    /** 总距离（用于归一化 s = 实际位移 / distance） */
    float distance_{0.0f};

    /** 7 段式时间：T1~T7（部分可能为 0，用于短距离退化） */
    float T1_{0.0f}, T2_{0.0f}, T3_{0.0f}, T4_{0.0f}, T5_{0.0f}, T6_{0.0f}, T7_{0.0f};

    /** 规划参数（生成时缓存，用于 getV/getA） */
    float v_max_{0.0f}, a_max_{0.0f}, j_max_{0.0f};

    /** 峰值速度（可能 < v_max，短距离退化） */
    float v_peak_{0.0f};

    /** 各段结束时刻的累积位移（用于 getS 分段求 s） */
    float s1_{0.0f}, s2_{0.0f}, s3_{0.0f}, s4_{0.0f}, s5_{0.0f}, s6_{0.0f};

    /** 各段结束时刻的速度 */
    float v1_{0.0f}, v2_{0.0f}, v3_{0.0f}, v4_{0.0f}, v5_{0.0f}, v6_{0.0f};
};

#endif /* ROBOT_ALGORITHMS_PLANNING_S_CURVE_PLANNER_H */
