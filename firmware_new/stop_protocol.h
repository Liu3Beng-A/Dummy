#ifndef DUMMY_STOP_PROTOCOL_H
#define DUMMY_STOP_PROTOCOL_H

#include <cstdint>

// CAN StdId = (node << 7) | COMMAND. Request: action[0], token[2..3].
// Reply: motor position r[0..3], phase[4], reserved[5], token[6..7].
namespace SoftStopProtocol
{
constexpr uint8_t COMMAND = 0x0A;
enum Action : uint8_t { QUERY = 0, STOP = 1, RELEASE = 2, ENABLE_HOLD = 3, DISABLE = 4 };
enum Phase : uint8_t { NORMAL = 0, DECELERATING = 1, HOLDING = 2, DISABLED = 3, FAULT = 4 };
}

#endif
