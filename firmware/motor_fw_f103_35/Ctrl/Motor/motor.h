#ifndef CTRL_STEP_FW_MOTOR_H
#define CTRL_STEP_FW_MOTOR_H

#include "Motor/motion_planner.h"
#include "Sensor/Encoder/encoder_base.h"
#include "Driver/driver_base.h"

class Motor
{
public:
    Motor() :
        controller(&controllerInstance)
    {
        /****************** Default Configs *******************/
        config.motionParams.encoderHomeOffset = 0;
        config.motionParams.caliCurrent = 2000;             // (mA)
        config.motionParams.ratedCurrent = 1000;            // (mA)
        config.motionParams.ratedCurrentAcc = 2 * 1000;     // (mA/s)
        config.motionParams.ratedVelocity = 30 * MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
        config.motionParams.ratedVelocityAcc = 1000 * MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;

        config.ctrlParams.stallProtectSwitch = true;
        config.ctrlParams.pid =
            Controller::PID_t{
                .kp = 5,
                .ki = 30,
                .kd = 0
            };
        config.ctrlParams.dce =
            Controller::DCE_t{
                .kp = 200,
                .kv = 80,
                .ki = 300,
                .kd = 250
            };

        /*****************************************************/


        motionPlanner.AttachConfig(&config.motionParams);
        controller->AttachConfig(&config.ctrlParams);
    }


    const int32_t MOTOR_ONE_CIRCLE_HARD_STEPS = 200; // for 1.8° step-motors
    const int32_t SOFT_DIVIDE_NUM = 256;
    const int32_t MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS = MOTOR_ONE_CIRCLE_HARD_STEPS * SOFT_DIVIDE_NUM;

    typedef enum
    {
        MODE_STOP,
        MODE_COMMAND_POSITION,
        MODE_COMMAND_VELOCITY,
        MODE_COMMAND_CURRENT,
        MODE_COMMAND_Trajectory,
        MODE_PWM_POSITION,
        MODE_PWM_VELOCITY,
        MODE_PWM_CURRENT,
        MODE_STEP_DIR,
    } Mode_t;

    typedef enum
    {
        STATE_STOP,
        STATE_FINISH,
        STATE_RUNNING,
        STATE_STALL,  // 仅用于 LED 指示（保留，不影响实际堵转逻辑）
        STATE_NO_CALIB
    } State_t;

    enum StallMode_t {
        STALL_IDLE = 0,       // 正常，检测堵转
        STALL_RETREATING = 1, // 回退中
        STALL_LOCKED = 2,     // 锁死，等 UNLOCKED
    };


    class Controller
    {
    public:
        friend Motor;

        typedef struct
        {
            bool kpValid, kiValid, kdValid;
            int32_t kp, ki, kd;
            int32_t vError, vErrorLast;
            int32_t outputKp, outputKi, outputKd;
            int32_t integralRound;
            int32_t integralRemainder;
            int32_t output;
        } PID_t;

        typedef struct
        {
            int32_t kp, kv, ki, kd;
            int32_t pError, vError;
            int32_t outputKp, outputKi, outputKd;
            int32_t integralRound;
            int32_t integralRemainder;
            int32_t output;
        } DCE_t;

        typedef struct
        {
            PID_t pid;
            DCE_t dce;

            bool stallProtectSwitch;
        } Config_t;


        explicit Controller(Motor* _context)
        {
            context = _context;

            requestMode = MODE_STOP;
            modeRunning = MODE_STOP;
        }


        Config_t* config = nullptr;
        Mode_t requestMode;
        Mode_t modeRunning;
        State_t state = STATE_STOP;
        StallMode_t stallMode = STALL_IDLE;
        uint32_t stallStartTick{};       // 堵转检测计时开始（us）
        uint32_t retreatStartTick{};     // 回退开始时刻（ms，2s超时用）
        int32_t retreatTarget{};          // 回退目标位置（触发瞬间快照）
        int32_t retreatDirection{};        // 回退方向（+1/-1，触发瞬间快照）
        int32_t stallCurrentThreshold{};   // 电流阈值（ratedCurrent*75%）
        uint32_t positionModeStartCycles{}; // 进入位置模式的起始周期（启动豁免期用）
        uint8_t stallBroadcastCmd{};       // 待发送广播命令 (0=无, 1=TRIGGER, 2=DONE, 3=TIMEOUT)
        bool stallDetectRisingEdge{};      // 堵转检测上升沿标记：true=三条件首次同时满足
        int32_t stallDetectRisingEstError{}; // 上升沿时刻的 |estError| 值（200ms 后比对）


        void Init();
        void SetCtrlMode(Mode_t _mode);
        void SetCurrentSetPoint(int32_t _cur);
        void SetVelocitySetPoint(int32_t _vel);
        void SetPositionSetPoint(int32_t _pos);
        bool SetPositionSetPointWithTime(int32_t _pos, float _time);
        // 重置所有 goal/soft 状态（用于 enable/UNLOCKED 后电机停在当前位置）：
        // 把 goalPosition=estPosition, 清零 goalVelocity/goalCurrent, 触发 softNewCurve
        void ResetGoalsToCurrentPosition();
        float GetPosition(bool _isLap = false);
        float GetVelocity();
        float GetFocCurrent();
        void AddTrajectorySetPoint(int32_t _pos, int32_t _vel);
        void SetDisable(bool _disable);
        void SetBrake(bool _brake);
        void ApplyPosAsHomeOffset();
        int32_t GetEstPosition() { return estPosition; }
        int32_t GetEstVelocity() { return estVelocity; }
        void ClearIntegral() { estVelocityIntegral = 0; estError = 0; }


    private:
        Motor* context;
        int32_t realLapPosition{};
        int32_t realLapPositionLast{};
        int32_t realPosition{};
        int32_t realPositionLast{};
        int32_t estVelocity{};
        int32_t estVelocityIntegral{};
        int32_t estLeadPosition{};
        int32_t estPosition{};
        int32_t estError{};
        int32_t focCurrent{};
        int32_t goalPosition{};
        int32_t goalVelocity{};
        int32_t goalCurrent{};
        bool goalDisable{};
        bool goalBrake{};
        int32_t softPosition{};
        int32_t softVelocity{};
        int32_t softCurrent{};
        bool softDisable{};
        bool softBrake{};
        bool softNewCurve{};
        int32_t focPosition{};


        void AttachConfig(Config_t* _config);
        void CalcCurrentToOutput(int32_t current);
        void CalcPidToOutput(int32_t _speed);
        void CalcDceToOutput(int32_t _location, int32_t _speed);
        void ClearTrajectory();

        static int32_t CompensateAdvancedAngle(int32_t _vel);
    };


    struct Config_t
    {
        MotionPlanner::Config_t motionParams{};
        Controller::Config_t ctrlParams{};
    };
    Config_t config;

    MotionPlanner motionPlanner;
    Controller* controller = nullptr;
    EncoderBase* encoder = nullptr;
    DriverBase* driver = nullptr;


    void Tick20kHz();
    void AttachEncoder(EncoderBase* _encoder);
    void AttachDriver(DriverBase* _driver);


private:
    Controller controllerInstance = Controller(this);


    void CloseLoopControlTick();
};

#endif
