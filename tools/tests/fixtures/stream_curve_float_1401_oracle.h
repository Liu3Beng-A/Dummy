#ifndef FLOAT1401_STREAM_CURVE_H
#define FLOAT1401_STREAM_CURVE_H
#include <cstdint>

namespace Float1401 {
struct Point { int32_t q, v, a; };
// Frozen release14.01 software-double preparation, test oracle only.
// Each Tick has nine 64-bit additions, no multiplication, division or float.
class Curve {
public:
    static constexpr unsigned Ticks = 400;
    bool Build(const Point& from, const Point& to, int32_t velocityLimit,
               int32_t accelerationLimit, int32_t home = 0);
    void Tick(int32_t& position, int32_t& velocity);
    bool Finished() const { return tick_ == Ticks; }
    Point End() const { return to_; }
private:
    int64_t p_[6]{}, v_[5]{};
    Point to_{};
    int32_t base_{};
    uint16_t tick_{};
};
}
#endif
