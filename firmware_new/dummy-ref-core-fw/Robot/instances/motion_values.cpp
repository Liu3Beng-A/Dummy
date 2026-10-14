#include "motion_values.h"
#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <cstring>

namespace MotionValues {
namespace {
bool Space(char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; }
bool Finite(float value) {
    // Release uses -Ofast: do not let finite-math assumptions remove this check.
    std::uint32_t bits;
    static_assert(sizeof(bits) == sizeof(value), "IEEE-754 float required");
    std::memcpy(&bits, &value, sizeof(bits));
    return (bits & 0x7f800000U) != 0x7f800000U;
}

}

static unsigned ParseWithLimit(const char* text, std::size_t length, float* values,
                               unsigned minimum, unsigned maximum, std::size_t limit, unsigned fields) {
    if (!text || !values || !length || length > limit ||
        !minimum || minimum > maximum || maximum > fields) return 0;
    char buffer[160];
    // Reject embedded NULs so a valid prefix cannot hide an invalid command.
    if (std::memchr(text, '\0', length)) return 0;
    std::memcpy(buffer, text, length);
    buffer[length] = '\0';
    float parsed[14];
    unsigned count = 0;
    char* cursor = buffer;
    for (;;) {
        while (Space(*cursor)) ++cursor;
        if (!*cursor || count == maximum) return 0;
        char* end = nullptr;
        errno = 0;
        // Direct conversion avoids scanf's large frame and nested sprintf path.
        const float value = std::strtof(cursor, &end);
        if (end == cursor || errno == ERANGE || !Finite(value)) return 0;
        parsed[count++] = value;
        cursor = end;
        while (Space(*cursor)) ++cursor;
        if (!*cursor) break;
        if (*cursor++ != ',') return 0;
    }
    if (count < minimum) return 0;
    std::memcpy(values, parsed, count * sizeof(float));
    return count;
}
unsigned Parse(const char* text,std::size_t length,float* values,unsigned minimum,unsigned maximum) {
    return ParseWithLimit(text,length,values,minimum,maximum,MaxPayload,8);
}
unsigned ParseCartesian(const char* text,std::size_t length,float* values,unsigned minimum,unsigned maximum) {
    return ParseWithLimit(text,length,values,minimum,maximum,159,14);
}
}
