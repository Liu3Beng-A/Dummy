#pragma once
#include "stream_curve.h"
#include "../../../stream_protocol.h"

namespace MotorStream {
// One writer (main/CAN commands) and one consumer (20kHz). Callers publish
// state under short IRQ guards; Curve::Build itself must remain preemptible.
struct Slot {
    Curve curve;
    uint16_t sequence{}, crc{};
    uint8_t flags{};
    volatile uint8_t state{}; // 0 empty, 1 prepared, 2 committed
};
struct Runtime {
    Slot slots[StreamProtocol::CAPACITY];
    volatile uint32_t ticks{};
    uint32_t anchor{}, lastSyncTick{};
    uint16_t session{}, prepared{}, committed{}, completed{}, syncCounter{};
    uint16_t preparedCrc{}, committedCrc{};
    uint8_t mask{}, preparedFlags{}, reason{};
    volatile uint8_t state{StreamProtocol::STATE_IDLE};
    bool seeded{}, seedCommitted{}, armed{}, terminal{}, settled{}, syncSeen{};
    volatile bool changed{}, stopRequired{};
    Point previous{}, seed{}, staged{};
    uint16_t stagedSequence{};
    uint8_t stagedMask{};
    int32_t home{}, velocityLimit{}, accelerationLimit{}, position{}, velocity{}, phaseError{};
    uint16_t retired[8]{};
    uint8_t retiredWrite{};
    bool Retired(uint16_t token) const;
    void Clear(uint8_t why=StreamProtocol::OK);
    uint8_t Free() const;
    void Fault(uint8_t why);
    void Tick();
};
}
