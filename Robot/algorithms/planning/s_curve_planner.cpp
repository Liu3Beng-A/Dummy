/**
 * @file s_curve_planner.cpp
 * @brief 归一化 S 型速度规划器实现（7 段式，支持短距离退化）
 */

#include "algorithms/planning/s_curve_planner.h"
#include <cmath>

namespace {

constexpr float kEpsilon = 1.0e-9f;
/** 最小段时长，避免 v_peak 很小时 sqrt 下溢导致 T_=0、total_s=0 引发卡死或速度爆炸 */
constexpr float kMinSegmentTime = 1e-5f;

inline bool is_positive(float x) { return x > kEpsilon; }
inline bool is_non_negative(float x) { return x >= -kEpsilon; }

/** 段 1：j+，a = j*t, v = j*t^2/2, s = j*t^3/6 */
inline void eval_jerk_up(float j, float t, float& s, float& v, float& a) {
    a = j * t;
    v = 0.5f * j * t * t;
    s = (1.0f / 6.0f) * j * t * t * t;
}

/** 段 2：恒 a，v = v0 + a*t, s = s0 + v0*t + 0.5*a*t^2 */
inline void eval_const_a(float a_val, float v0, float s0, float t, float& s, float& v, float& a) {
    a = a_val;
    v = v0 + a_val * t;
    s = s0 + v0 * t + 0.5f * a_val * t * t;
}

/** 段 3：j-，a 从 a_max 线性减到 0，时长 T3 */
inline void eval_jerk_down_from_a(float j_max, float a_max, float T3, float v0, float s0, float t,
                                  float& s, float& v, float& a) {
    a = a_max - j_max * t;
    v = v0 + a_max * t - 0.5f * j_max * t * t;
    s = s0 + v0 * t + 0.5f * a_max * t * t - (1.0f / 6.0f) * j_max * t * t * t;
}

/** 段 4：恒速 v_peak */
inline void eval_const_v(float v_peak, float s0, float t, float& s, float& v, float& a) {
    a = 0.0f;
    v = v_peak;
    s = s0 + v_peak * t;
}

/** 段 5：j-，a 从 0 到 -a_max */
inline void eval_jerk_down_to_neg_a(float j_max, float v0, float s0, float t, float& s, float& v, float& a) {
    a = -j_max * t;
    v = v0 - 0.5f * j_max * t * t;
    s = s0 + v0 * t - (1.0f / 6.0f) * j_max * t * t * t;
}

/** 段 6：恒 -a_max */
inline void eval_const_neg_a(float a_max, float v0, float s0, float t, float& s, float& v, float& a) {
    a = -a_max;
    v = v0 - a_max * t;
    s = s0 + v0 * t - 0.5f * a_max * t * t;
}

/** 段 7：j+，a 从 -a_max 回到 0 */
inline void eval_jerk_up_from_neg_a(float j_max, float a_max, float v0, float s0, float t,
                                   float& s, float& v, float& a) {
    a = -a_max + j_max * t;
    v = v0 - a_max * t + 0.5f * j_max * t * t;
    s = s0 + v0 * t - 0.5f * a_max * t * t + (1.0f / 6.0f) * j_max * t * t * t;
}

}  // namespace

bool SCurvePlanner::generate(float distance, float v_max, float a_max, float j_max) {
    T1_ = T2_ = T3_ = T4_ = T5_ = T6_ = T7_ = 0.0f;
    s1_ = s2_ = s3_ = s4_ = s5_ = s6_ = 0.0f;
    v1_ = v2_ = v3_ = v4_ = v5_ = v6_ = 0.0f;
    T_ = 0.0f;
    distance_ = 0.0f;
    v_peak_ = 0.0f;
    v_max_ = v_max;
    a_max_ = a_max;
    j_max_ = j_max;

    distance = fabsf(distance);
    if (distance < kEpsilon) {
        distance_ = 0.0f;
        T_ = 0.0f;
        return true;
    }
    if (!is_positive(j_max) || !is_positive(a_max) || !is_non_negative(v_max)) {
        return false;
    }

    float Tj = a_max / j_max;
    bool has_const_a = (v_max > a_max * Tj);  // v_max > a_max^2/j_max

    float Ta;   // 加速段总时间
    float s_ramp;  // 单侧 ramp 位移

    if (has_const_a) {
        Ta = v_max / a_max + Tj;
        float T2 = Ta - 2.0f * Tj;
        if (T2 < 0.0f) T2 = 0.0f;
        float v1 = 0.5f * a_max * Tj;
        float s1 = (1.0f / 6.0f) * j_max * Tj * Tj * Tj;
        s_ramp = s1 + v1 * T2 + 0.5f * a_max * T2 * T2 + s1;
    } else {
        float Tj_prime = fmaxf(sqrtf(v_max / j_max), kMinSegmentTime);
        Ta = 2.0f * Tj_prime;
        s_ramp = 2.0f * (1.0f / 6.0f) * j_max * Tj_prime * Tj_prime * Tj_prime;
    }

    float two_s_ramp = 2.0f * s_ramp;
    if (distance >= two_s_ramp - kEpsilon) {
        v_peak_ = v_max;
        float T4 = (distance - two_s_ramp) / v_max;
        if (T4 < 0.0f) T4 = 0.0f;

        if (has_const_a) {
            T1_ = T3_ = Tj;
            T2_ = Ta - 2.0f * Tj;
            if (T2_ < 0.0f) T2_ = 0.0f;
            T5_ = T7_ = Tj;
            T6_ = T2_;
        } else {
            float Tj_prime = fmaxf(sqrtf(v_max / j_max), kMinSegmentTime);
            T1_ = T3_ = Tj_prime;
            T2_ = 0.0f;
            T5_ = T7_ = Tj_prime;
            T6_ = 0.0f;
        }
        T4_ = T4;
    } else {
        // 短距离退化：达到最大加速度所需的最小位移（单侧 ramp 的积分下限）
        float s_const_a_min = 2.0f * (a_max * a_max * a_max) / (j_max * j_max);

        if (distance >= s_const_a_min) {
            // 情况 A：距离能达到恒定加速度，但达不到最大速度
            // 梯形加速度剖面：单侧 S_ramp = v_peak^2/a_max + v_peak*(a_max/j_max)，总位移 distance = 2*S_ramp
            // 解 v_peak 的一元二次方程正根
            v_peak_ = 0.5f * a_max * (-(a_max / j_max) + sqrtf((a_max / j_max) * (a_max / j_max) + 4.0f * distance / a_max));
        } else {
            // 情况 B：距离极短，仅纯 Jerk 三角加速剖面，无法达到 a_max
            // S = 2 * v_peak^(3/2) / sqrt(j_max) 反解 v_peak（此处 S 为单侧位移，distance = 2*S）
            float tmp = 0.5f * distance * sqrtf(j_max);
            v_peak_ = (tmp > 0.0f) ? powf(tmp, 2.0f / 3.0f) : 0.0f;
        }

        if (v_peak_ > v_max)
            v_peak_ = v_max;

        if (v_peak_ < kEpsilon) {
            distance_ = distance;
            T_ = 0.0f;
            return false;
        }

        if (v_peak_ > a_max * Tj) {
            Ta = v_peak_ / a_max + Tj;
            float T2 = Ta - 2.0f * Tj;
            if (T2 < 0.0f) T2 = 0.0f;
            T1_ = T3_ = Tj;
            T2_ = T2;
            float v1 = 0.5f * a_max * Tj;
            float s1 = (1.0f / 6.0f) * j_max * Tj * Tj * Tj;
            s_ramp = s1 + v1 * T2 + 0.5f * a_max * T2 * T2 + s1;
            T5_ = T7_ = Tj;
            T6_ = T2;
            T4_ = 0.0f;
        } else {
            float Tj_prime = fmaxf(sqrtf(v_peak_ / j_max), kMinSegmentTime);
            Ta = 2.0f * Tj_prime;
            s_ramp = 2.0f * (1.0f / 6.0f) * j_max * Tj_prime * Tj_prime * Tj_prime;
            T1_ = T3_ = Tj_prime;
            T2_ = 0.0f;
            T5_ = T7_ = Tj_prime;
            T6_ = 0.0f;
            T4_ = 0.0f;
        }
    }

    float t = 0.0f;
    float s = 0.0f, v = 0.0f, a = 0.0f;

    if (is_positive(T1_)) {
        eval_jerk_up(j_max_, T1_, s, v, a);
        s1_ = s; v1_ = v; t += T1_;
    } else { s1_ = 0.0f; v1_ = 0.0f; }
    if (is_positive(T2_)) {
        eval_const_a(a_max_, v1_, s1_, T2_, s, v, a);
        s2_ = s; v2_ = v; t += T2_;
    } else { s2_ = s1_; v2_ = v1_; }
    if (is_positive(T3_)) {
        eval_jerk_down_from_a(j_max_, a_max_, T3_, v2_, s2_, T3_, s, v, a);
        s3_ = s; v3_ = v; t += T3_;
    } else { s3_ = s2_; v3_ = v2_; }
    if (is_positive(T4_)) {
        eval_const_v(v_peak_, s3_, T4_, s, v, a);
        s4_ = s; v4_ = v; t += T4_;
    } else { s4_ = s3_; v4_ = v3_; }
    if (is_positive(T5_)) {
        eval_jerk_down_to_neg_a(j_max_, v4_, s4_, T5_, s, v, a);
        s5_ = s; v5_ = v; t += T5_;
    } else { s5_ = s4_; v5_ = v4_; }
    if (is_positive(T6_)) {
        eval_const_neg_a(a_max_, v5_, s5_, T6_, s, v, a);
        s6_ = s; v6_ = v; t += T6_;
    } else { s6_ = s5_; v6_ = v5_; }
    if (is_positive(T7_)) {
        eval_jerk_up_from_neg_a(j_max_, a_max_, v6_, s6_, T7_, s, v, a);
        t += T7_;
    }

    float total_s = s;
    if (total_s < kEpsilon || t < kEpsilon) {
        distance_ = distance;
        T_ = 0.0f;
        return false;
    }
    distance_ = total_s;
    T_ = t;
    return true;
}

float SCurvePlanner::getTotalTime() const {
    return T_;
}

float SCurvePlanner::getS(float t) const {
    if (t <= 0.0f) return 0.0f;
    if (distance_ <= 0.0f || T_ <= 0.0f) return 1.0f;
    if (t >= T_) return 1.0f;

    float acc = 0.0f, vel = 0.0f, pos = 0.0f;
    float t_cur = 0.0f;

    if (is_positive(T1_) && t_cur + T1_ <= t) {
        t_cur += T1_; pos = s1_; vel = v1_;
    } else if (is_positive(T1_)) {
        eval_jerk_up(j_max_, t - t_cur, pos, vel, acc);
        return pos / distance_;
    } else { pos = 0.0f; vel = 0.0f; }

    if (is_positive(T2_) && t_cur + T2_ <= t) {
        t_cur += T2_; pos = s2_; vel = v2_;
    } else if (is_positive(T2_)) {
        eval_const_a(a_max_, v1_, s1_, t - t_cur, pos, vel, acc);
        return pos / distance_;
    }

    if (is_positive(T3_) && t_cur + T3_ <= t) {
        t_cur += T3_; pos = s3_; vel = v3_;
    } else if (is_positive(T3_)) {
        eval_jerk_down_from_a(j_max_, a_max_, T3_, v2_, s2_, t - t_cur, pos, vel, acc);
        return pos / distance_;
    }

    if (is_positive(T4_) && t_cur + T4_ <= t) {
        t_cur += T4_; pos = s4_; vel = v4_;
    } else if (is_positive(T4_)) {
        eval_const_v(v_peak_, s3_, t - t_cur, pos, vel, acc);
        return pos / distance_;
    }

    if (is_positive(T5_) && t_cur + T5_ <= t) {
        t_cur += T5_; pos = s5_; vel = v5_;
    } else if (is_positive(T5_)) {
        eval_jerk_down_to_neg_a(j_max_, v4_, s4_, t - t_cur, pos, vel, acc);
        return pos / distance_;
    }

    if (is_positive(T6_) && t_cur + T6_ <= t) {
        t_cur += T6_; pos = s6_; vel = v6_;
    } else if (is_positive(T6_)) {
        eval_const_neg_a(a_max_, v5_, s5_, t - t_cur, pos, vel, acc);
        return pos / distance_;
    }

    if (is_positive(T7_)) {
        eval_jerk_up_from_neg_a(j_max_, a_max_, v6_, s6_, t - t_cur, pos, vel, acc);
    }
    return pos / distance_;
}

float SCurvePlanner::getV(float t) const {
    if (t <= 0.0f) return 0.0f;
    if (T_ <= 0.0f || t >= T_) return 0.0f;

    float acc = 0.0f, vel = 0.0f, pos = 0.0f;
    float t_cur = 0.0f;

    if (is_positive(T1_) && t_cur + T1_ <= t) { t_cur += T1_; vel = v1_; }
    else if (is_positive(T1_)) { eval_jerk_up(j_max_, t - t_cur, pos, vel, acc); return vel; }
    else vel = 0.0f;

    if (is_positive(T2_) && t_cur + T2_ <= t) { t_cur += T2_; vel = v2_; }
    else if (is_positive(T2_)) { eval_const_a(a_max_, v1_, s1_, t - t_cur, pos, vel, acc); return vel; }

    if (is_positive(T3_) && t_cur + T3_ <= t) { t_cur += T3_; vel = v3_; }
    else if (is_positive(T3_)) { eval_jerk_down_from_a(j_max_, a_max_, T3_, v2_, s2_, t - t_cur, pos, vel, acc); return vel; }

    if (is_positive(T4_) && t_cur + T4_ <= t) { t_cur += T4_; vel = v4_; }
    else if (is_positive(T4_)) { return v_peak_; }

    if (is_positive(T5_) && t_cur + T5_ <= t) { t_cur += T5_; vel = v5_; }
    else if (is_positive(T5_)) { eval_jerk_down_to_neg_a(j_max_, v4_, s4_, t - t_cur, pos, vel, acc); return vel; }

    if (is_positive(T6_) && t_cur + T6_ <= t) { t_cur += T6_; vel = v6_; }
    else if (is_positive(T6_)) { eval_const_neg_a(a_max_, v5_, s5_, t - t_cur, pos, vel, acc); return vel; }

    if (is_positive(T7_)) eval_jerk_up_from_neg_a(j_max_, a_max_, v6_, s6_, t - t_cur, pos, vel, acc);
    return vel;
}

float SCurvePlanner::getA(float t) const {
    if (t <= 0.0f || t >= T_ || T_ <= 0.0f) return 0.0f;

    float acc = 0.0f, vel = 0.0f, pos = 0.0f;
    float t_cur = 0.0f;

    if (is_positive(T1_) && t_cur + T1_ <= t) { t_cur += T1_; }
    else if (is_positive(T1_)) { eval_jerk_up(j_max_, t - t_cur, pos, vel, acc); return acc; }

    if (is_positive(T2_) && t_cur + T2_ <= t) { t_cur += T2_; acc = a_max_; }
    else if (is_positive(T2_)) { return a_max_; }

    if (is_positive(T3_) && t_cur + T3_ <= t) { t_cur += T3_; }
    else if (is_positive(T3_)) { eval_jerk_down_from_a(j_max_, a_max_, T3_, v2_, s2_, t - t_cur, pos, vel, acc); return acc; }

    if (is_positive(T4_) && t_cur + T4_ <= t) { t_cur += T4_; }
    else if (is_positive(T4_)) { return 0.0f; }

    if (is_positive(T5_) && t_cur + T5_ <= t) { t_cur += T5_; }
    else if (is_positive(T5_)) { eval_jerk_down_to_neg_a(j_max_, v4_, s4_, t - t_cur, pos, vel, acc); return acc; }

    if (is_positive(T6_) && t_cur + T6_ <= t) { t_cur += T6_; acc = -a_max_; }
    else if (is_positive(T6_)) { return -a_max_; }

    if (is_positive(T7_)) eval_jerk_up_from_neg_a(j_max_, a_max_, v6_, s6_, t - t_cur, pos, vel, acc);
    return acc;
}
