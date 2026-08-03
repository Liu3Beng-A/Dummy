/**
 * @file rail_motor.h
 * @brief Simplified rail motor controller for 57 stepper motor
 *
 * Features:
 * - Fixed 2.8A current output (no ramp)
 * - Velocity closed-loop control with PID
 * - No position tracking (simplified for rail application)
 * - CAN interface for velocity commands
 */

#ifndef RAIL_MOTOR_H
#define RAIL_MOTOR_H

#include "Sensor/Encoder/encoder_base.h"
#include "Driver/driver_base.h"

class RailMotor
{
public:
    static constexpr int32_t MOTOR_ONE_CIRCLE_HARD_STEPS = 200;  // for 1.8° step-motors
    static constexpr int32_t SOFT_DIVIDE_NUM = 256;
    static constexpr int32_t MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS = MOTOR_ONE_CIRCLE_HARD_STEPS * SOFT_DIVIDE_NUM;

    // Fixed current for 57 motor (mA)
    static constexpr int32_t FIXED_CURRENT_MA = 2800;

    // Control frequency (Hz)
    static constexpr uint32_t CONTROL_FREQUENCY = 20000;
    static constexpr uint32_t CONTROL_PERIOD_US = 50;  // 50us = 20kHz

    // Velocity limits (subdivide steps/s)
    static constexpr int32_t MAX_VELOCITY = 30 * MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;  // 30 r/s

    typedef enum
    {
        STATE_STOP,
        STATE_RUNNING
    } State_t;

    RailMotor();

    void Tick20kHz();

    void AttachEncoder(EncoderBase* _encoder);
    void AttachDriver(DriverBase* _driver);

    // Velocity control (in rps, revolutions per second)
    void SetVelocity(float _rps);

    // Stop motor
    void Stop();

    // Get current state
    State_t GetState() const { return state; }

    // Get current velocity (rps)
    float GetVelocity();

    // Get current (A)
    float GetCurrent() const { return (float)focCurrent / 1000.0f; }

private:
    EncoderBase* encoder = nullptr;
    DriverBase* driver = nullptr;

    State_t state = STATE_STOP;
    int32_t goalVelocity = 0;  // subdivide steps/s
    int32_t estVelocity = 0;
    int32_t estVelocityIntegral = 0;

    int32_t realPosition = 0;
    int32_t realPositionLast = 0;

    int32_t focCurrent = 0;
    int32_t focPosition = 0;

    // PID parameters for velocity control
    struct
    {
        int32_t kp = 100;
        int32_t ki = 20;
        int32_t kd = 10;
        int32_t error;
        int32_t errorLast;
        int32_t integral;
        int32_t output;
    } velocityPid;

    void InitEncoderData();
    void UpdateVelocityEstimate();
    void VelocityControlLoop();
    void CurrentToOutput();
};

#endif  // RAIL_MOTOR_H
