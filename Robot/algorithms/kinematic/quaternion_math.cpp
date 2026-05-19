/**
 * @file quaternion_math.cpp
 * @brief 四元数数学库实现：欧拉角/旋转矩阵/Slerp，与 6dof_kinematic 欧拉顺序一致。
 */

#include "algorithms/kinematic/quaternion_math.h"
#include <cmath>

namespace {

constexpr float kEpsilon = 1.0e-6f;
constexpr float kSlerpLerpThreshold = 0.9995f;

inline float dot(const Quaternion_t& a, const Quaternion_t& b) {
    return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
}

Quaternion_t normalize(const Quaternion_t& q) {
    float n = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (n < kEpsilon) return Quaternion_t{1.0f, 0.0f, 0.0f, 0.0f};
    n = 1.0f / n;
    return Quaternion_t{q.w * n, q.x * n, q.y * n, q.z * n};
}

/** 四元数乘法: p * q */
Quaternion_t mul(const Quaternion_t& p, const Quaternion_t& q) {
    return Quaternion_t{
        p.w * q.w - p.x * q.x - p.y * q.y - p.z * q.z,
        p.w * q.x + p.x * q.w + p.y * q.z - p.z * q.y,
        p.w * q.y - p.x * q.z + p.y * q.w + p.z * q.x,
        p.w * q.z + p.x * q.y - p.y * q.x + p.z * q.w
    };
}

}  // namespace

Quaternion_t EulerToQuaternion(float rx, float ry, float rz) {
    float cx = cosf(rx * 0.5f), sx = sinf(rx * 0.5f);
    float cy = cosf(ry * 0.5f), sy = sinf(ry * 0.5f);
    float cz = cosf(rz * 0.5f), sz = sinf(rz * 0.5f);
    /* ZYX 顺序：Q = Qx(rx) * Qy(ry) * Qz(rz)，与 6dof_kinematic EulerAngleToRotMat 一致 */
    Quaternion_t qx{cx, sx, 0.0f, 0.0f};
    Quaternion_t qy{cy, 0.0f, sy, 0.0f};
    Quaternion_t qz{cz, 0.0f, 0.0f, sz};
    return normalize(mul(mul(qx, qy), qz));
}

void QuaternionToRotMat(const Quaternion_t& q, float* rotMat9) {
    float w = q.w, x = q.x, y = q.y, z = q.z;
    float w2 = w * w, x2 = x * x, y2 = y * y, z2 = z * z;
    float wx = w * x, wy = w * y, wz = w * z, xy = x * y, xz = x * z, yz = y * z;
    /* 行主序 [R00 R01 R02 R10 R11 R12 R20 R21 R22] */
    rotMat9[0] = 1.0f - 2.0f * (y2 + z2);
    rotMat9[1] = 2.0f * (xy - wz);
    rotMat9[2] = 2.0f * (xz + wy);
    rotMat9[3] = 2.0f * (xy + wz);
    rotMat9[4] = 1.0f - 2.0f * (x2 + z2);
    rotMat9[5] = 2.0f * (yz - wx);
    rotMat9[6] = 2.0f * (xz - wy);
    rotMat9[7] = 2.0f * (yz + wx);
    rotMat9[8] = 1.0f - 2.0f * (x2 + y2);
}

Quaternion_t Slerp(Quaternion_t q1, Quaternion_t q2, float t) {
    if (t <= 0.0f) return q1;
    if (t >= 1.0f) return q2;

    float d = dot(q1, q2);
    if (d < 0.0f) {
        q2.w = -q2.w;
        q2.x = -q2.x;
        q2.y = -q2.y;
        q2.z = -q2.z;
        d = -d;
    }

    if (d > kSlerpLerpThreshold) {
        /* 姿态极其接近，Lerp + 归一化，避免 sin(theta) 除零 */
        Quaternion_t r;
        r.w = q1.w + t * (q2.w - q1.w);
        r.x = q1.x + t * (q2.x - q1.x);
        r.y = q1.y + t * (q2.y - q1.y);
        r.z = q1.z + t * (q2.z - q1.z);
        return normalize(r);
    }

    float theta0 = acosf(d);
    float theta = theta0 * t;
    float sin_theta0 = sinf(theta0);
    if (sin_theta0 < kEpsilon) return q1;
    float k0 = cosf(theta) - d * sinf(theta) / sin_theta0;
    float k1 = sinf(theta) / sin_theta0;
    return Quaternion_t{
        k0 * q1.w + k1 * q2.w,
        k0 * q1.x + k1 * q2.x,
        k0 * q1.y + k1 * q2.y,
        k0 * q1.z + k1 * q2.z
    };
}
