// Frozen pre-fast-path production oracle from13.04; intentionally independent.
#include "stream_curve_hold_oracle.h"
#include <climits>

namespace Oracle {
namespace {
constexpr int64_t Scale = int64_t(1) << 40;
int64_t Round(double x) { return static_cast<int64_t>(x < 0 ? x - .5 : x + .5); }
int32_t Integer(int64_t x) {
    // Truncation-free nearest rounding, including negative fractional values.
    return static_cast<int32_t>(x >= 0 ? (x + Scale / 2) >> 40 : -((-x + Scale / 2) >> 40));
}
bool Within(int64_t x, int64_t bound) { return x >= -bound && x <= bound; }
}

bool Curve::Build(const Point& from, const Point& to, int32_t velocityLimit,
                  int32_t accelerationLimit, int32_t home) {
    if (velocityLimit <= 0 || velocityLimit > 1536000 || accelerationLimit <= 0 ||
        accelerationLimit > 5120000) return false;
    const int64_t d = int64_t(to.q) - from.q;
    // Exact Bernstein convex-hull bounds for a 20ms quintic. Control positions
    // are relative to q0, scaled by 50000. First differences /200 are speed;
    // second differences are acceleration. Covers the ENTIRE segment.
    const int64_t b[6] = {0, int64_t(200)*from.v, int64_t(400)*from.v + from.a,
        50000*d - int64_t(400)*to.v + to.a, 50000*d - int64_t(200)*to.v, 50000*d};
    for (unsigned i = 0; i < 6; ++i) {
        const int64_t relative = int64_t(from.q)*50000 + b[i];
        const int64_t absolute = relative + int64_t(home)*50000;
        if (relative < int64_t(INT32_MIN)*50000 || relative > int64_t(INT32_MAX)*50000 ||
            absolute < int64_t(INT32_MIN)*50000 || absolute > int64_t(INT32_MAX)*50000 ||
            !Within(b[i], int64_t(40000)*50000)) return false;
        if (i < 5 && !Within(b[i+1]-b[i], int64_t(velocityLimit)*200)) return false;
        if (i < 4 && !Within(b[i+2]-2*b[i+1]+b[i], accelerationLimit)) return false;
    }
    // Analytic forward differences avoid subtracting nearly equal polynomial
    // samples. The small software-double preparation is main-loop work only.
    const double c[6] = {0, from.v*.02, from.a*.0002,
        10.*d - (.12*from.v+.08*to.v) - (.0006*from.a-.0002*to.a),
        -15.*d + (.16*from.v+.14*to.v) + (.0006*from.a-.0004*to.a),
        6.*d - .06*(double(from.v)+to.v) - .0002*(double(from.a)-to.a)};
    static const uint8_t difference[6][6] = {
        {1,0,0,0,0,0}, {0,1,1,1,1,1}, {0,0,2,6,14,30},
        {0,0,0,6,36,150}, {0,0,0,0,24,240}, {0,0,0,0,0,120}};
    double invPower[6] = {1};
    for (unsigned i=1; i<6; ++i) invPower[i]=invPower[i-1]/Ticks;
    for (unsigned j=0; j<6; ++j) {
        double value=0;
        for (unsigned k=j; k<6; ++k) value += difference[j][k]*c[k]*invPower[k];
        p_[j]=Round(value*double(Scale));
    }
    for (unsigned j=0; j<5; ++j) {
        double value=0;
        for (unsigned k=j; k<5; ++k) value += difference[j][k]*(k+1)*50*c[k+1]*invPower[k];
        v_[j]=Round(value*double(Scale));
    }
    base_=from.q; to_=to; tick_=0;
    return true;
}

void Curve::Tick(int32_t& position, int32_t& velocity) {
    if (tick_ < Ticks) {
        for (unsigned i=0; i<5; ++i) p_[i] += p_[i+1];
        for (unsigned i=0; i<4; ++i) v_[i] += v_[i+1];
        ++tick_;
    }
    // Exact endpoints prevent accumulation over arbitrarily many segments.
    position = tick_ == Ticks ? to_.q : base_ + Integer(p_[0]);
    velocity = tick_ == Ticks ? to_.v : Integer(v_[0]);
}
}
