/**
 * @file quaternion_math.h
 * @brief 四元数数学库，用于笛卡尔空间姿态平滑（Slerp）及与欧拉角/旋转矩阵转换。
 *
 * 无硬件/RTOS 依赖，全单精度 float，数学函数带 f 后缀。
 */

#ifndef ROBOT_ALGORITHMS_KINEMATIC_QUATERNION_MATH_H
#define ROBOT_ALGORITHMS_KINEMATIC_QUATERNION_MATH_H

/** 四元数 q = w + x*i + y*j + z*k，默认单位四元数 */
struct Quaternion_t {
    float w{1.0f};
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
};

/**
 * 欧拉角 (ZYX 顺序，与 6dof_kinematic RotMatToEulerAngle 一致) 转四元数。
 * @param rx 绕 X 轴弧度 (roll)
 * @param ry 绕 Y 轴弧度 (pitch)
 * @param rz 绕 Z 轴弧度 (yaw)
 */
Quaternion_t EulerToQuaternion(float rx, float ry, float rz);

/**
 * 四元数转 3x3 旋转矩阵，行主序一维数组 rotMat9[0..8]。
 * 与 6dof_kinematic 中 R[9] 布局一致，供 IK 使用。
 */
void QuaternionToRotMat(const Quaternion_t& q, float* rotMat9);

/**
 * 四元数球面线性插值 (Slerp)。
 * 已做最短路径处理与 |dot|≈1 时的 Lerp 降级+归一化。
 * @param t 进度 [0.0f, 1.0f]，可由 SCurvePlanner::getS(t) 提供
 */
Quaternion_t Slerp(Quaternion_t q1, Quaternion_t q2, float t);

#endif /* ROBOT_ALGORITHMS_KINEMATIC_QUATERNION_MATH_H */
