#ifndef DUMMY_MOTION_COMPLETION_PROTOCOL_H
#define DUMMY_MOTION_COMPLETION_PROTOCOL_H

#include <cstdint>
#include <cstring>

// Extended 0x23 query: expected motor revolutions [0..3], marker [4],
// nonzero query nonce [5..7]. Reply: actual revolutions [0..3], flags [4],
// echoed nonce [5..7]. Legacy position replies cannot confirm a new motion.
namespace MotionCompletionProtocol {
constexpr uint8_t COMMAND = 0x23;
constexpr uint8_t QUERY_MARKER = 0xA5;
constexpr uint8_t REPLY_MARKER = 0x80;
constexpr uint8_t TARGET_READY = 0x02;
constexpr uint8_t SETTLED = 0x01;
constexpr uint32_t NONCE_MASK = 0xFFFFFF;
constexpr uint32_t FEEDBACK_FRESH_MS = 300;
constexpr uint32_t STABLE_TICKS = 1000; // 50ms at 20kHz.
constexpr int32_t STABLE_RANGE_STEPS = 8;
constexpr int32_t STABLE_SPEED_STEPS = 512; // 0.01 motor r/s.
constexpr int32_t STEPS_PER_REV = 51200;

// 0.1 joint degree, 0.1 rail mm. J6 has a 30:1 gearbox; J1..5 have 50:1.
// J8 is excluded from whole-machine completion; retain its fine tolerance.
constexpr int32_t PositionToleranceSteps(uint8_t node) {
    return node == 7 ? 1024 : node == 6 ? 426 : node >= 1 && node <= 5 ? 711 : 51;
}
inline bool Finite(float value) {
    uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    return (bits & 0x7F800000U) != 0x7F800000U;
}
inline uint32_t Nonce(const uint8_t* data) {
    return data[5] | (static_cast<uint32_t>(data[6]) << 8) |
           (static_cast<uint32_t>(data[7]) << 16);
}
inline void PutNonce(uint8_t* data, uint32_t nonce) {
    data[5] = static_cast<uint8_t>(nonce);
    data[6] = static_cast<uint8_t>(nonce >> 8);
    data[7] = static_cast<uint8_t>(nonce >> 16);
}
}
#endif
