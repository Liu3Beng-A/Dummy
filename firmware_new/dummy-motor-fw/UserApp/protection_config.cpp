#include "protection_config.h"
#include "configurations.h"
#include <cstring>
#include <cmath>

using namespace ProtectionProtocol;
namespace {
constexpr uint16_t STORAGE_OFFSET = 128;
constexpr uint32_t STORAGE_MAGIC = 0x50525431;
struct StoredProtection {
    uint32_t magic;
    uint16_t version, length;
    uint16_t task;
    uint8_t coordinateVersion, measured, node, reserved;
    int32_t homeOffset;
    ProtectionSettings settings;
    uint32_t crc;
};
static_assert(sizeof(BoardConfig_t) <= STORAGE_OFFSET, "Legacy EEPROM overlap");
static_assert(STORAGE_OFFSET + sizeof(StoredProtection) <= 1024, "Protection EEPROM overflow");
uint32_t Crc(const uint8_t* p, uint32_t length) {
    uint32_t crc = 0xFFFFFFFF;
    while (length--) {
        crc ^= *p++;
        for (unsigned b = 0; b < 8; ++b) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}
bool Integral(float f) { return f == static_cast<float>(static_cast<uint32_t>(f)); }
}
bool ProtectionSettings::Finite(float v) {
    uint32_t bits; memcpy(&bits, &v, sizeof(bits));
    // Bit test remains valid with the existing -Ofast build configuration.
    return (bits & 0x7F800000u) != 0x7F800000u;
}
void ProtectionSettings::Defaults(uint8_t node, int32_t currentMa) {
    memset(value, 0, sizeof(value));
    const float current = currentMa / 1000.0f;
    value[ProtectionProtocol::ENABLE] = node >= 1 && node <= 7 ? 1 : 0;
    value[CURRENT_A] = current * 0.9f; value[SPEED_RPS] = .02f;
    value[ERROR_R] = node == 7 ? .1f : .5f / 360.0f * (node == 6 ? 30.0f : 50.0f);
    value[TRIGGER_MS] = 300; value[STARTUP_MS] = 200;
    value[CURRENT_FILTER_MS] = 10; value[SPEED_FILTER_MS] = 20;
    value[BACK_DISTANCE_R] = node == 7 ? 1.0f : 5.0f / 360.0f * (node == 6 ? 30.0f : 50.0f);
    value[BACK_RPS] = node == 7 ? .5f : 3.0f; value[BACK_ACCEL] = 5;
    value[BACK_CURRENT_A] = value[HOLD_CURRENT_A] = current;
    value[BACK_POS_TOL_R] = .001f; value[BACK_TIMEOUT_MS] = 5000;
    value[REBLOCK_MS] = 100; value[GRANT_WAIT_MS] = 500;
    value[MIN_POSITION_R] = -100; value[MAX_POSITION_R] = 100; value[DIRECTION_MASK] = 3;
    value[CAL_SEEK_RPS] = value[CAL_RETURN_RPS] = .5f; value[CAL_RETOUCH_RPS] = .25f;
    value[CAL_ACCEL] = 5; value[CAL_CURRENT_A] = node == 8 ? fminf(1, current) : current;
    value[CAL_CONTACT_RATIO] = .8f; value[CAL_SPEED_RPS] = value[CAL_ERROR_R] = .02f;
    value[CAL_TRIGGER_MS] = 150; value[CAL_STARTUP_MS] = 100;
    value[CAL_MAX_TRAVEL_R] = node == 7 ? 101.0f : 105.0f * 16.0f / 360.0f;
    value[CAL_SEEK_TIMEOUT_MS] = node == 7 ? 220000 : 15000;
    value[CAL_RETURN_TIMEOUT_MS] = node == 7 ? 120000 : 15000;
    value[CAL_RETOUCH_TIMEOUT_MS] = 5000;
    value[CAL_BACKOFF_R] = node == 7 ? .2f : 2.0f * 16.0f / 360.0f;
    value[CAL_REPEAT_TOL_R] = .01f; value[CAL_POSITION_TOL_R] = .001f;
    value[CAL_CONTACT_WINDOW_R] = node == 7 ? 1.0f : 5.0f * 16.0f / 360.0f;
}
bool ProtectionSettings::Set(uint8_t id, float v, uint8_t node, int32_t currentMa,
                             int32_t velocity, int32_t acceleration) {
    if (id == 0 || id > PARAMETER_COUNT || !Finite(v)) return false;
    if (node == 8 && id <= DIRECTION_MASK) return false;
    if (id >= CAL_SEEK_RPS && node != 7 && node != 8) return false;
    float lo = 0, hi = 0;
    switch (id) {
        case ProtectionProtocol::ENABLE: lo = 0; hi = 1; break;
        case CURRENT_A: lo = .1f; hi = currentMa / 1000.0f; break;
        case SPEED_RPS: lo = .001f; hi = .2f; break;
        case ERROR_R: lo = .0002f; hi = 1; break;
        case TRIGGER_MS: lo = 20; hi = 2000; break;
        case STARTUP_MS: case CAL_STARTUP_MS: lo = 0; hi = 500; break;
        case CURRENT_FILTER_MS: case SPEED_FILTER_MS: lo = 1; hi = 100; break;
        case BACK_DISTANCE_R: lo = .0002f; hi = 100; break;
        case BACK_RPS: case CAL_SEEK_RPS: case CAL_RETURN_RPS: case CAL_RETOUCH_RPS:
            lo = .001f; hi = velocity / 51200.0f; break;
        case BACK_ACCEL: case CAL_ACCEL: lo = .1f; hi = fminf(100, acceleration / 51200.0f); break;
        case BACK_CURRENT_A: case HOLD_CURRENT_A: case CAL_CURRENT_A:
            lo = .1f; hi = currentMa / 1000.0f; break;
        case BACK_POS_TOL_R: case CAL_POSITION_TOL_R: lo = .0002f; hi = .01f; break;
        case BACK_TIMEOUT_MS: lo = 250; hi = 30000; break;
        case REBLOCK_MS: lo = 20; hi = 500; break;
        case GRANT_WAIT_MS: lo = 100; hi = 2000; break;
        case MIN_POSITION_R: case MAX_POSITION_R: lo = -10000; hi = 10000; break;
        case DIRECTION_MASK: lo = 1; hi = 3; break;
        case CAL_CONTACT_RATIO: lo = .1f; hi = 1; break;
        case CAL_SPEED_RPS: lo = .001f; hi = .2f; break;
        case CAL_ERROR_R: lo = .0002f; hi = 1; break;
        case CAL_TRIGGER_MS: lo = 20; hi = 2000; break;
        case CAL_MAX_TRAVEL_R: lo = .001f; hi = 200; break;
        case CAL_SEEK_TIMEOUT_MS: case CAL_RETURN_TIMEOUT_MS:
            lo = 1000; hi = 600000; break;
        case CAL_RETOUCH_TIMEOUT_MS: lo = 250; hi = 30000; break;
        case CAL_BACKOFF_R: lo = .001f; hi = 2; break;
        case CAL_REPEAT_TOL_R: lo = .0002f; hi = .1f; break;
        case CAL_CONTACT_WINDOW_R: lo = .001f; hi = 10; break;
        default: return false;
    }
    if (v < lo || v > hi) return false;
    if ((id == ProtectionProtocol::ENABLE || id == DIRECTION_MASK || id == TRIGGER_MS || id == STARTUP_MS ||
         id == CURRENT_FILTER_MS || id == SPEED_FILTER_MS || id == BACK_TIMEOUT_MS ||
         id == REBLOCK_MS || id == GRANT_WAIT_MS || id == CAL_TRIGGER_MS ||
         id == CAL_STARTUP_MS || id == CAL_SEEK_TIMEOUT_MS || id == CAL_RETURN_TIMEOUT_MS ||
         id == CAL_RETOUCH_TIMEOUT_MS) && !Integral(v)) return false;
    value[id] = v;
    return true;
}
bool ProtectionSettings::Validate(uint8_t node, int32_t currentMa, int32_t velocity,
                                  int32_t acceleration) const {
    ProtectionSettings checked = *this;
    for (uint8_t id = 1; id <= PARAMETER_COUNT; ++id) {
        if (node == 8 && id <= DIRECTION_MASK) continue;
        if (id >= CAL_SEEK_RPS && node != 7 && node != 8) continue;
        if (!checked.Set(id, value[id], node, currentMa, velocity, acceleration)) return false;
    }
    return node == 8 || value[MIN_POSITION_R] < value[MAX_POSITION_R];
}
bool LoadProtectionSettings(ProtectionSettings& out, uint8_t node, int32_t currentMa,
                            int32_t velocity, int32_t acceleration) {
    EEPROM eeprom; StoredProtection record{}; eeprom.get(STORAGE_OFFSET, record);
    if (record.magic != STORAGE_MAGIC || record.version != VERSION || record.length != sizeof(record) || record.node != node ||
        record.crc != Crc(reinterpret_cast<const uint8_t*>(&record), sizeof(record) - 4) ||
        !record.settings.Validate(node, currentMa, velocity, acceleration)) return false;
    out = record.settings;
    if (node == 8) out.value[ProtectionProtocol::ENABLE] = 0;
    return true;
}
bool ProtectionStorageIsErased() {
    for (uint32_t i = 0; i < sizeof(StoredProtection); ++i)
        if (eeprom_flash_read_byte(STORAGE_OFFSET + i) != 0xFF) return false;
    return true;
}
bool SaveProtectionSettings(const ProtectionSettings& settings, uint16_t task,
                            uint8_t coordinateVersion, int32_t runtimeOffset, bool measured) {
    StoredProtection record{};
    record.magic = STORAGE_MAGIC; record.version = VERSION; record.length = sizeof(record);
    record.task = task; record.coordinateVersion = coordinateVersion; record.measured = measured;
    record.node = static_cast<uint8_t>(boardConfig.canNodeId);
    record.homeOffset = runtimeOffset; record.settings = settings;
    record.crc = Crc(reinterpret_cast<const uint8_t*>(&record), sizeof(record) - 4);
    // Populate both records before one erase/program, preserving the remainder
    // of the existing user page. The old struct layout remains unchanged.
    EEPROM eeprom; eeprom.setCommitASAP(false);
    // The legacy boot unwrap is single-turn. Keep its established modulo
    // offset; the extension records the full runtime value for this task,
    // without claiming to recover turns moved while power was off.
    boardConfig.encoderHomeOffset = runtimeOffset % Motor::MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
    boardConfig.configStatus = CONFIG_OK;
    eeprom.put(0, boardConfig); eeprom.put(STORAGE_OFFSET, record);
    eeprom.commit();
    // Direct flash read, deliberately bypassing EEPROM's RAM cache.
    StoredProtection actual{};
    for (uint32_t i = 0; i < sizeof(actual); ++i)
        reinterpret_cast<uint8_t*>(&actual)[i] = eeprom_flash_read_byte(STORAGE_OFFSET + i);
    BoardConfig_t actualBoard{};
    for (uint32_t i = 0; i < sizeof(actualBoard); ++i)
        reinterpret_cast<uint8_t*>(&actualBoard)[i] = eeprom_flash_read_byte(i);
    return memcmp(&actual, &record, sizeof(record)) == 0 &&
           actual.crc == Crc(reinterpret_cast<const uint8_t*>(&actual), sizeof(actual) - 4) &&
           memcmp(&actualBoard, &boardConfig, sizeof(boardConfig)) == 0;
}
