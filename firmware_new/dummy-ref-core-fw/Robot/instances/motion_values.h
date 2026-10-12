#pragma once
#include <cstddef>

namespace MotionValues {
// The ASCII motion queue holds at most 63 characters plus its terminator.
constexpr std::size_t MaxPayload = 63;
// Return the complete field count, or zero. Output is unchanged on failure.
unsigned Parse(const char* text, std::size_t length, float* values,
               unsigned minimum, unsigned maximum);
}
