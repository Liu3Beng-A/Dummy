#include "stream_curve.h"
#include <climits>

namespace MotorStream {
namespace {
constexpr int64_t Scale = int64_t(1) << 40;
// Exact rational 20ms/400-tick Q40 maps. Each coefficient is split into
// an integer and a signed fractional remainder to keep every product in int64.
// The two large velocity coefficients store their high word separately below.
// Tables occupy 369 bytes before linker alignment (45 pairs + 9 sign masks).
int64_t Difference(unsigned row, const int32_t* input) {
    struct Coefficient { uint32_t whole, remainder; };
    static constexpr Coefficient map[9][5] = {
        {{171155U,555363125U}, {54973526U,4089374650U}, {1368U,2337786600U}, {1364U,655282828U}, {3U,2556246597U}},
        {{1021792U,57460000U}, {12273U,600364475U}, {8162U,426175350U}, {2687U,356046613U}, {20U,458946652U}},
        {{1007695U,235450625U}, {12123U,20035125U}, {8030U,204400450U}, {60U,225350916U}, {20U,143788U}},
        {{15307U,321585625U}, {163U,464545825U}, {142U,933487450U}, {0U,748683264U}, {0U,497025024U}},
        {{77U,377699375U}, {0U,943718400U}, {0U,943718400U}, {0U,3145728U}, {0U,3145728U}},
        {{1666511735U,698125U}, {123145946U,1085950U}, {81982979U,1935825U}, {54359165U,1044911U}, {204786U,367494U}},
        {{3127638134U,1211250U}, {244100600U,1557800U}, {162049545U,1122675U}, {1224611U,273841U}, {404100U,1148396U}},
        {{306918362U,1898750U}, {3275342U,117050U}, {2863025U,389675U}, {12292U,383596U}, {8169U,54291U}},
        {{1546188U,442500U}, {15461U,1723175U}, {15461U,1723175U}, {51U,1053921U}, {51U,1053921U}},
    };
    static constexpr uint8_t negative[9] = {4,6,14,17,14,6,14,17,14};
    const int64_t denominator = row==0 ? 6103515625LL :
        row==2 ? 244140625LL : row<5 ? 1220703125LL : 1953125LL;
    int64_t integral=0, fractional=0;
    for (unsigned column=0; column<5; ++column) {
        const int64_t value=(negative[row] & (1U<<column)) ? -int64_t(input[column]) : input[column];
        integral += int64_t(map[row][column].whole)*value;
        fractional += int64_t(map[row][column].remainder)*value;
    }
    if (row==5 || row==6)
        integral += int64_t(input[0])*(int64_t(row==5 ? 2 : 4)<<32);
    // Denominators are odd powers of five, so an exact half cannot occur.
    // Whole sum <=1.445e15, remainder sum <=2.634e16 for the accepted input
    // box; adding half a denominator and the final quotient cannot overflow.
    return integral + (fractional + (fractional<0 ? -denominator/2 : denominator/2))/denominator;
}

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
    // Hard hull checks above imply |d|<=40000, |v|<=1536000 and
    // |a|<=5120000. No floating point, wide multiply or run-time powers.
    const int32_t input[5]={int32_t(d),from.v,to.v,from.a,to.a};
    p_[0]=0;
    for (unsigned row=0; row<5; ++row) p_[row+1]=Difference(row,input);
    v_[0]=int64_t(from.v)*Scale;
    for (unsigned row=0; row<4; ++row) v_[row+1]=Difference(row+5,input);
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
