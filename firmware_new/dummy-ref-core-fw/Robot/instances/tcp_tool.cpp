#include "tcp_tool.h"
#include "../../Bsp/gpio/rgb.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include <cmath>
#include <cstring>
#include <stddef.h>

extern RGB rgb;

namespace {
constexpr uint32_t TOOL_ADDRESS = 0x080FFB00UL, TOOL_MAGIC = 0x54435031UL;
constexpr uint16_t TOOL_VERSION = 1;
constexpr float DEG_TO_RAD = 0.01745329251994329577f;
constexpr float DEFAULT_VALUES[6] = {0, 0, 114, 0, 0, 0};
struct Record {
    uint32_t magic;
    uint16_t version, size;
    float values[6]; // Boundary units: mm and degrees.
    uint32_t crc;
};
static_assert(sizeof(Record) <= 256 && sizeof(Record) % 4 == 0, "TCP record must fit its separate slot");

bool Finite(float value) {
    uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    return (bits & 0x7f800000U) != 0x7f800000U;
}
bool Valid(const float* values) {
    for (unsigned i = 0; i < 6; ++i) {
        if (!Finite(values[i])) return false;
        if (i >= 3 && (values[i] < -180 || values[i] > 180)) return false;
    }
    return true;
}
uint32_t Crc(const void* data, size_t length) {
    uint32_t crc = 0xFFFFFFFFUL;
    const auto* bytes = static_cast<const uint8_t*>(data);
    while (length--) {
        crc ^= *bytes++;
        for (unsigned bit = 0; bit < 8; ++bit) crc = (crc >> 1) ^ ((crc & 1) ? 0xEDB88320UL : 0);
    }
    return ~crc;
}
DOF6Kinematic::Pose6D_t DefaultPose() {
    DOF6Kinematic::Pose6D_t pose(0, 0, .114f, 0, 0, 0);
    pose.R[0] = pose.R[4] = pose.R[8] = 1;
    pose.hasR = true;
    return pose;
}
DOF6Kinematic::Pose6D_t ToPose(const float* values) {
    DOF6Kinematic::Pose6D_t pose(values[0] * .001f, values[1] * .001f, values[2] * .001f,
                               values[3], values[4], values[5]);
    const float a = values[3] * DEG_TO_RAD, b = values[4] * DEG_TO_RAD, c = values[5] * DEG_TO_RAD;
    const float ca = std::cos(a), cb = std::cos(b), cc = std::cos(c);
    const float sa = std::sin(a), sb = std::sin(b), sc = std::sin(c);
    pose.R[0] = cc * cb; pose.R[1] = cc * sb * sa - sc * ca; pose.R[2] = cc * sb * ca + sc * sa;
    pose.R[3] = sc * cb; pose.R[4] = sc * sb * sa + cc * ca; pose.R[5] = sc * sb * ca - cc * sa;
    pose.R[6] = -sb; pose.R[7] = cb * sa; pose.R[8] = cb * ca;
    pose.hasR = true;
    return pose;
}
float currentValues[6] = {0, 0, 114, 0, 0, 0}, savedValues[6]{};
DOF6Kinematic::Pose6D_t currentPose = DefaultPose();
uint32_t revision = 1;
bool savedValid = false;
uint8_t storage = 0;

bool Read(Record& record) {
    std::memcpy(&record, reinterpret_cast<const void*>(TOOL_ADDRESS), sizeof(record));
    return record.magic == TOOL_MAGIC && record.version == TOOL_VERSION && record.size == sizeof(record.values) &&
           record.crc == Crc(&record, offsetof(Record, crc)) && Valid(record.values);
}
bool Erased(const Record& record) {
    const auto* bytes = reinterpret_cast<const uint8_t*>(&record);
    for (unsigned i = 0; i < sizeof(record); ++i) if (bytes[i] != 0xFF) return false;
    return true;
}
void Publish(const float* values, const DOF6Kinematic::Pose6D_t& pose) {
    taskENTER_CRITICAL();
    if (std::memcmp(values, currentValues, sizeof(currentValues))) {
        std::memcpy(currentValues, values, sizeof(currentValues)); currentPose = pose;
        if (++revision == 0) ++revision;
    }
    taskEXIT_CRITICAL();
}
}

namespace TcpTool {
void Load() {
    Record record{};
    const bool valid = Read(record);
    const float* values = valid ? record.values : DEFAULT_VALUES;
    const auto pose = valid ? ToPose(values) : DefaultPose();
    Publish(values, pose);
    taskENTER_CRITICAL();
    savedValid = valid;
    if (valid) std::memcpy(savedValues, record.values, sizeof(savedValues));
    storage = valid ? 1 : Erased(record) ? 0 : 2;
    taskEXIT_CRITICAL();
}
bool Set(float xMm, float yMm, float zMm, float aDeg, float bDeg, float cDeg) {
    float values[6] = {xMm, yMm, zMm, aDeg, bDeg, cDeg};
    for (float value : values) if (!Finite(value)) return false;
    for (unsigned i = 3; i < 6; ++i) {
        values[i] = std::fmod(values[i], 360.0f);
        if (values[i] > 180) values[i] -= 360;
        else if (values[i] < -180) values[i] += 360;
        if (values[i] == 0) values[i] = 0; // Canonicalize negative zero.
    }
    const auto pose = ToPose(values);
    if (!Valid(values)) return false;
    for (float value : pose.R) if (!Finite(value)) return false;
    Publish(values, pose);
    return true;
}
void Reset() { Publish(DEFAULT_VALUES, DefaultPose()); }
DOF6Kinematic::Pose6D_t Get() {
    taskENTER_CRITICAL(); const auto pose = currentPose; taskEXIT_CRITICAL(); return pose;
}
Status GetStatus() {
    taskENTER_CRITICAL();
    Status result{revision, uint8_t(!savedValid || std::memcmp(currentValues, savedValues, sizeof(currentValues)) != 0), storage};
    taskEXIT_CRITICAL(); return result;
}
void RefreshStorageProof() {
    Record record{};
    const bool valid = Read(record);
    taskENTER_CRITICAL();
    savedValid = valid;
    if (valid) std::memcpy(savedValues, record.values, sizeof(savedValues));
    storage = valid ? 1 : Erased(record) ? 0 : 2;
    taskEXIT_CRITICAL();
}
bool Save() {
    Record record{};
    record.magic = TOOL_MAGIC; record.version = TOOL_VERSION; record.size = sizeof(record.values);
    taskENTER_CRITICAL(); std::memcpy(record.values, currentValues, sizeof(currentValues)); taskEXIT_CRITICAL();
    record.crc = Crc(&record, offsetof(Record, crc));
    const bool ok = rgb.SaveToolRecord(&record, sizeof(record));
    RefreshStorageProof();
    taskENTER_CRITICAL();
    const bool verified = ok && savedValid && !std::memcmp(record.values, savedValues, sizeof(savedValues));
    if (!verified) { savedValid = false; storage = 2; }
    taskEXIT_CRITICAL();
    return verified;
}
}
