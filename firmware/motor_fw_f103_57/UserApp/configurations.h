#ifndef CONFIGURATIONS_H
#define CONFIGURATIONS_H

#ifdef __cplusplus
extern "C" {
#endif
/*---------------------------- C Scope ---------------------------*/
#include <stdbool.h>
#include "stdint-gcc.h"

/* 地轨电机固定 CAN ID=9（不依赖 DIP 开关拨码） */
#define RAIL_FIXED_NODE_ID 9

typedef enum configStatus_t
{
    CONFIG_RESTORE = 0,
    CONFIG_OK,
    CONFIG_COMMIT
} configStatus_t;


typedef struct Config_t
{
    configStatus_t configStatus;
    uint32_t canNodeId;
    int32_t encoderHomeOffset;
    uint32_t defaultMode;
    int32_t currentLimit;
    int32_t velocityLimit;
    int32_t velocityAcc;
    int32_t calibrationCurrent;
    int32_t dce_kp;
    int32_t dce_kv;
    int32_t dce_ki;
    int32_t dce_kd;
    float motor_temperature;
    bool enableMotorOnBoot;
    bool enableStallProtect;
    bool enableTempWatch;
    // 重构阶段2.4 (2026-08-23): 堵转保护可调参数（EEPROM 持久化）
    int32_t stallCurrentThreshold;   // 堵转检测电流阈值（mA），默认 ratedCurrent * 95 / 100
    int32_t stallRetreatSteps;       // 回退步数（35/42=711=5°, 57=40960=5mm）
    uint16_t stallDetectTimeMs;      // 触发延迟（ms），默认 200
    uint16_t stallRetreatTimeMs;     // 回退超时（ms），默认 2000
} BoardConfig_t;

extern BoardConfig_t boardConfig;


#ifdef __cplusplus
}
/*---------------------------- C++ Scope ---------------------------*/

#include <Platform/Memory/eeprom_interface.h>
#include "Motor/motor.h"


#endif
#endif
