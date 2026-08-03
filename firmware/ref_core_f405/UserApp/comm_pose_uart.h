#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 初始化（若需要，当前为空实现）
void PoseUART_Init(void);

// 发送位姿：CSV 格式 "POS,X(mm),Y(mm),Z(mm),A(deg),B(deg),C(deg)\r\n"
void PoseUART_SendPose(const void *pose /* pointer to DOF6Kinematic::Pose6D_t - opaque in C */);

#ifdef __cplusplus
}
#endif