#pragma once
#include <cmath>
// Host-only boundary shim. Interpolated mode models the CMSIS 512-entry fast
// sine/cosine table; it is not a claim of ARM timing or bit-exact library identity.
inline float host_sine(float value) {
#ifdef CMSIS_INTERPOLATED
    constexpr float tau=6.2831853071795864769f;
    float phase=std::fmod(value/tau,1.0f);if(phase<0)phase+=1.0f;
    const float index=phase*512.0f;const int integer=int(index);const float fraction=index-integer;
    const float a=std::sin(float(integer)*tau/512.0f),b=std::sin(float(integer+1)*tau/512.0f);
    return (1.0f-fraction)*a+fraction*b;
#else
    return std::sin(value);
#endif
}
inline float arm_sin_f32(float value) {return host_sine(value);}
inline float arm_cos_f32(float value) {
#ifdef CMSIS_INTERPOLATED
    return host_sine(value+1.5707963267948966192f);
#else
    return std::cos(value);
#endif
}
