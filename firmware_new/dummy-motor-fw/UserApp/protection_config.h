#ifndef MOTOR_PROTECTION_CONFIG_H
#define MOTOR_PROTECTION_CONFIG_H
#include "../../protection_protocol.h"
#include <stdint.h>

struct ProtectionSettings {
    float value[ProtectionProtocol::PARAMETER_COUNT + 1]{};
    float Get(uint8_t id) const { return id > 0 && id <= ProtectionProtocol::PARAMETER_COUNT ? value[id] : 0; }
    void Defaults(uint8_t node, int32_t currentMa);
    bool Set(uint8_t id, float v, uint8_t node, int32_t currentMa, int32_t velocity, int32_t acceleration);
    bool Validate(uint8_t node, int32_t currentMa, int32_t velocity, int32_t acceleration) const;
    static bool Finite(float v);
};

// Legacy BoardConfig remains at byte zero. This versioned extension starts at
// byte 128, entirely inside the existing 1KB user page; no calibration-table
// repartition and no incompatible insertion into BoardConfig.
bool LoadProtectionSettings(ProtectionSettings& out, uint8_t node, int32_t currentMa,
                            int32_t velocity, int32_t acceleration);
bool ProtectionStorageIsErased();
bool SaveProtectionSettings(const ProtectionSettings& settings, uint16_t task,
                            uint8_t coordinateVersion, int32_t runtimeOffset,
                            bool coordinateMeasured);
#endif
