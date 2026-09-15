#ifndef CONFIGURATIONS_H
#define CONFIGURATIONS_H

#ifdef __cplusplus
extern "C" {
#endif
/*---------------------------- C Scope ---------------------------*/
#include <stdbool.h>
#include "stdint-gcc.h"

/* [v3.1] 地轨电机固定 CAN ID=7（不依赖 DIP 开关拨码，node ID 9 已废弃） */
#define RAIL_FIXED_NODE_ID 7

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
} BoardConfig_t;

extern BoardConfig_t boardConfig;


#ifdef __cplusplus
}
/*---------------------------- C++ Scope ---------------------------*/

#include <Platform/Memory/eeprom_interface.h>
#include "Motor/motor.h"


#endif
#endif
