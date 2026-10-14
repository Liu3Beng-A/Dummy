#pragma once
#include "../algorithms/kinematic/6dof_kinematic.h"
#include <stdint.h>

// One flange-local TCP definition. The wrist link belongs to the robot model;
// the nominal gripper-center +Z 114 mm is an independent tool transform.
// The robot owns the idle/settings gate and disabled-only shared Flash lease.
namespace TcpTool {
struct Status {
    uint32_t revision;
    uint8_t dirty, storage; // 0=erased/default, 1=valid, 2=invalid/write failed.
};
void Load();
bool Set(float xMm, float yMm, float zMm, float aDeg, float bDeg, float cDeg);
void Reset();
bool Save();
// XYZ is metres; ABC remains degrees. R is Rz(C)*Ry(B)*Rx(A), row-major.
DOF6Kinematic::Pose6D_t Get();
Status GetStatus();

// Shared-sector storage callback; never replaces an unsaved RAM definition.
void RefreshStorageProof();
}
