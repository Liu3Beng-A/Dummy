#include "stream_curve.h"
#include <climits>

namespace MotorStream {
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
    // A held reference is exactly constant after the same hard bounds above.
    // Prepare its zero differences without software-double arithmetic in main.
    if (from.q == to.q && !from.v && !from.a && !to.v && !to.a) {
        for (auto& difference : p_) difference = 0;
        for (auto& difference : v_) difference = 0;
        base_ = from.q; to_ = to; tick_ = 0;
        return true;
    }
    // Direct linear maps to Q40 forward differences. Constants include
    // 2^40, the exact 400-tick interval, and the 20ms velocity/acceleration
    // units. No run-time power recurrence or software double division.
    static constexpr double positionDifference[5][5] = {
        {171155.09099069441, 54973526.670003146, -1368.383022956544, 1364.1073615385394, 3.4188154424524799},
        {1021792.047071232, -12273.491818577921, -8162.3491228467201, 2687.2916733853694, 20.3759690973184},
        {1007695.9644057601, -12123.082063872, -8030.8372242431997, -60.923037351936003, 20.000588955647999},
        {-15307.263442944, 163.38055593984001, 142.76471291903999, 0.61332132986879995, -0.40716289966079999},
        {77.309411327999996, -0.77309411327999999, -0.77309411327999999, -0.0025769803775999999, 0.0025769803775999999},
    };
    static constexpr double velocityDifference[4][5] = {
        {10256446327.357441, -123145946.5560064, -81982979.991142407, 54359165.534994431, 204786.18815692799},
        {20307507318.620159, -244100600.79759359, -162049545.57480961, -1224611.1402065919, 404100.58797875198},
        {-306918362.97215998, 3275342.0599296, 2863025.1995136002, 12292.196401151999, -8169.0277969919998},
        {1546188.2265600001, -15461.882265599999, -15461.882265599999, -51.539607552, 51.539607552},
    };
    const double input[5]={double(d),double(from.v),double(to.v),double(from.a),double(to.a)};
    p_[0]=0;
    for(unsigned row=0;row<5;++row) {
        double value=0;
        for(unsigned column=0;column<5;++column)value+=positionDifference[row][column]*input[column];
        p_[row+1]=Round(value);
    }
    v_[0]=Round(50*(from.v*.02)*double(Scale));
    for(unsigned row=0;row<4;++row) {
        double value=0;
        for(unsigned column=0;column<5;++column)value+=velocityDifference[row][column]*input[column];
        v_[row+1]=Round(value);
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
