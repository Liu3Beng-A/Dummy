#include "common_inc.h"
#include "configurations.h"
#include "Platform/Utils/st_hardware.h"
#include <tim.h>
#include <can.h>


/* Component Definitions -----------------------------------------------------*/
BoardConfig_t boardConfig;
Motor motor;
TB67H450 tb67H450;
MT6816 mt6816;
EncoderCalibrator encoderCalibrator(&motor);
Button button1(1, 1000), button2(2, 3000);
void OnButton1Event(Button::Event _event);
void OnButton2Event(Button::Event _event);
Led statusLed;
bool MotorDefaultsRestoreAllowed()
{
    return boardConfig.configStatus == CONFIG_OK && !encoderCalibrator.isTriggered;
}
/* Main Entry ----------------------------------------------------------------*/
void Main()
{
    uint64_t serialNum = GetSerialNumber();
    uint16_t defaultNodeID = 0;
    //ID0 is PA8 -->  Switch 4
    //ID1 is PA9 -->  Switch 3
    //ID2 is PA10 --> Switch 2
    uint16_t nodeID = !HAL_GPIO_ReadPin(GPIOA, ID0_Pin);
    nodeID |= !HAL_GPIO_ReadPin(GPIOA, ID1_Pin) << 1;
    nodeID |= !HAL_GPIO_ReadPin(GPIOA, ID2_Pin) << 2;
    defaultNodeID = nodeID;

    /*---------- Apply EEPROM Settings ----------*/
    __HAL_RCC_PWR_CLK_ENABLE(); __HAL_RCC_BKP_CLK_ENABLE(); HAL_PWR_EnableBkUpAccess();
    const bool restoreBoot = BKP->DR5 == ProtectionProtocol::RESTORE_ERASED;
    // Setting priority is EEPROM > Motor.h
    EEPROM eeprom;
    eeprom.get(0, boardConfig);
    if (boardConfig.configStatus != CONFIG_OK || restoreBoot) // use default settings
    {
        boardConfig = BoardConfig_t{
            .configStatus = CONFIG_OK,
//            .canNodeId = defaultNodeID,
            .encoderHomeOffset = 0,
            .defaultMode = Motor::MODE_COMMAND_POSITION,
#if MOTOR_TYPE == 35
            .currentLimit = 2500,                    // mA
            .velocityLimit = 30 * Motor::MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS, // r/s
            .velocityAcc = 30 * Motor::MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS,   // r/s^2
            .calibrationCurrent = 2000,
            .dce_kp = 200, .dce_kv = 80, .dce_ki = 300, .dce_kd = 250,
            .motor_temperature = 0.0,
#elif MOTOR_TYPE == 42
            .currentLimit = 2500,
            .velocityLimit = 30 * Motor::MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS, // r/s
            .velocityAcc = 30 * Motor::MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS,   // r/s^2
            .calibrationCurrent = 2000,
            .dce_kp = 200, .dce_kv = 80, .dce_ki = 300, .dce_kd = 250,
            .motor_temperature = 0.0,
#elif MOTOR_TYPE == 57
            .currentLimit = 2500,                                          // mA — 参考 firmware/motor_fw_f103_57 设为 2.5A
            .velocityLimit = 30 * Motor::MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS, // r/s
            .velocityAcc = 30 * Motor::MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS,   // r/s^2
            .calibrationCurrent = 2000,
            // User-selected J7 defaults (2026-10-08); valid EEPROM settings take precedence.
            .dce_kp = 350, .dce_kv = 320, .dce_ki = 25, .dce_kd = 50,
            .motor_temperature = 0.0,
            .enableMotorOnBoot = false,
            .enableStallProtect = true,    // 57 地轨默认开启堵转保护（覆盖 outer 块的 false）
#elif MOTOR_TYPE == 8
            .currentLimit = 2500,
            .velocityLimit = 30 * Motor::MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS, // r/s
            .velocityAcc = 30 * Motor::MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS,   // r/s^2
            .calibrationCurrent = 2000,
            .dce_kp = 200, .dce_kv = 80, .dce_ki = 300, .dce_kd = 250,
            .motor_temperature = 0.0,
#endif
            // .enableMotorOnBoot/.enableStallProtect 在 57 段内显式设为 false/true；
            // 35/42/8 此处省略，按 C++20 designated initializer 规则默认为 false
            .enableTempWatch=false,
        };
        eeprom.put(0, boardConfig);
    }
    // The restore request survives a software reset in backup registers. Only
    // acknowledge completion after physical Flash contains the new base config
    // and an erased extension, before applying runtime-only node overrides.
    if (restoreBoot)
    {
        bool verified = true;
        const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&boardConfig);
        for (uint32_t i = 0; i <= E2END; ++i)
            if (eeprom_flash_read_byte(i) != (i < sizeof(boardConfig) ? bytes[i] : 0xFF))
                verified = false;
        BKP->DR5 = verified ? ProtectionProtocol::RESTORE_READY : ProtectionProtocol::RESTORE_FAILED;
    }
    boardConfig.enableTempWatch=false;
    //depends on 3 bits switch now
    boardConfig.canNodeId = defaultNodeID;
#if MOTOR_TYPE == 57
    boardConfig.canNodeId = RAIL_FIXED_NODE_ID;
#elif MOTOR_TYPE == 8
    boardConfig.canNodeId = GRIPPER_FIXED_NODE_ID;
    // The confirmed gripper boot contract is always disabled and motionless,
    // even if an older EEPROM enabled automatic startup.
    boardConfig.enableMotorOnBoot = false;
#endif
    motor.config.motionParams.encoderHomeOffset = boardConfig.encoderHomeOffset;
    motor.config.motionParams.ratedCurrent = boardConfig.currentLimit;
#if MOTOR_TYPE == 57
    motor.config.motionParams.ratedCurrentAcc = 50 * 1000;    // mA/s — 57 电机慢爬坡
    // 强制 nodeID（不读 DIP 开关），地轨 ID 固定为 7
    boardConfig.canNodeId = RAIL_FIXED_NODE_ID;
#elif MOTOR_TYPE == 8
    motor.config.motionParams.ratedCurrentAcc = 2 * 1000;     // mA/s（与通用一致）
    boardConfig.canNodeId = GRIPPER_FIXED_NODE_ID;
#endif
    motor.config.motionParams.ratedVelocity = boardConfig.velocityLimit;
    motor.config.motionParams.ratedVelocityAcc = boardConfig.velocityAcc;
    motor.motionPlanner.velocityTracker.SetVelocityAcc(boardConfig.velocityAcc);
    motor.motionPlanner.positionTracker.SetVelocityAcc(boardConfig.velocityAcc);
    motor.config.motionParams.caliCurrent = boardConfig.calibrationCurrent;
    motor.config.ctrlParams.dce.kp = boardConfig.dce_kp;
    motor.config.ctrlParams.dce.kv = boardConfig.dce_kv;
    motor.config.ctrlParams.dce.ki = boardConfig.dce_ki;
    motor.config.ctrlParams.dce.kd = boardConfig.dce_kd;
    motor.config.ctrlParams.stallProtectSwitch = boardConfig.enableStallProtect;
#if MOTOR_TYPE == 57
    // 57 motor uses different PID gains for velocity loop
    motor.config.ctrlParams.pid.kp = 15;
    motor.config.ctrlParams.pid.ki = 80;
    motor.config.ctrlParams.pid.kd = 10;
#endif


//    printf("encoderHomeOffset[%d] ratedCurrent[%d] ratedVelocity[%d] ratedVelocityAcc[%d]\r\n" , boardConfig.encoderHomeOffset, boardConfig.currentLimit, boardConfig.velocityLimit, boardConfig.velocityAcc);
//    printf("velocityTracker[%d] positionTracker[%d] caliCurrent[%d] \r\n" , boardConfig.velocityAcc, boardConfig.velocityAcc);
//    printf("kp[%d] kv[%d] ki[%d] kd[%d]\r\n" , boardConfig.dce_kp, boardConfig.dce_kv, boardConfig.dce_ki, boardConfig.dce_kd);


    /*---------------- Init Motor ----------------*/
    motor.AttachDriver(&tb67H450);
    motor.AttachEncoder(&mt6816);
    motor.controller->Init();
    motor.controller->InitProtection();
    motor.driver->Init();
    motor.encoder->Init();


    /*------------- Init peripherals -------------*/
    button1.SetOnEventListener(OnButton1Event);
    button2.SetOnEventListener(OnButton2Event);


    /*------- Start Close-Loop Control Tick ------*/
    HAL_Delay(100);
    HAL_TIM_Base_Start_IT(&htim1);  // 100Hz
    HAL_TIM_Base_Start_IT(&htim4);  // 20kHz

    if (!restoreBoot && button1.IsPressed() && button2.IsPressed())
        encoderCalibrator.isTriggered = true;
//    printf("start motor %d\r\n", defaultNodeID);

    for (;;)
    {
        CAN_PollTransmit();
        motor.controller->PollDefaultsRestore();
        motor.controller->PollProtectionMain();
//        if ( encoderCalibrator.FlashRun() == 0)
            encoderCalibrator.TickMainLoop();

        if (boardConfig.configStatus == CONFIG_COMMIT)
        {
            boardConfig.configStatus = CONFIG_OK;
            eeprom.put(0, boardConfig);
        } else if (boardConfig.configStatus == CONFIG_RESTORE)
        {
            eeprom.put(0, boardConfig);
            HAL_NVIC_SystemReset();
        }
    }
}


/* Event Callbacks -----------------------------------------------------------*/
//let's add a global count in here for quick test
uint32_t count;
extern "C" void Tim1Callback100Hz()
{
    __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);

    button1.Tick(10);
    button2.Tick(10);
    statusLed.Tick(10, motor.controller->state);
    //1s period to collect a temperature data
    if (boardConfig.enableTempWatch)
    {
        count ++;
        if ( count >= 100)
        {
            boardConfig.motor_temperature = AdcGetChipTemperature();
            count = 0;
        }
        if (count > 50)
        {
            statusLed.Status(1, true);
        }
        else
        {
            statusLed.Status(1, false);
        }
    }
}


extern "C" void Tim4Callback20kHz()
{
    static uint32_t previousCycle;
    const uint32_t began = DWT->CYCCNT;
    const uint32_t gap = began - previousCycle;
    if (previousCycle && gap > motorCanDiagnostic[MC_CONTROL_GAP]) motorCanDiagnostic[MC_CONTROL_GAP] = gap;
    previousCycle = began;
    __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);

    if (encoderCalibrator.isTriggered)
        encoderCalibrator.Tick20kHz();
    else
        motor.Tick20kHz();
    const uint32_t elapsed = DWT->CYCCNT - began;
    if (elapsed > motorCanDiagnostic[MC_CONTROL_MAX]) motorCanDiagnostic[MC_CONTROL_MAX] = elapsed;
    if (elapsed >= SystemCoreClock / 20000U || __HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_UPDATE))
        ++motorCanDiagnostic[MC_CONTROL_LATE];
}


void OnButton1Event(Button::Event _event)
{
    if (motor.controller->IsSoftStopLocked()) return;
    switch (_event)
    {
        case ButtonBase::UP:
            break;
        case ButtonBase::DOWN:
            break;
        case ButtonBase::LONG_PRESS:
            encoderCalibrator.isTriggered = true;
            encoderCalibrator.Tick20kHz();
            encoderCalibrator.TickMainLoop();
//            HAL_NVIC_SystemReset();
//            encoderCalibrator.TestFlash();
            break;
        case ButtonBase::CLICK:
            printf("KEY1\r\n");
            if (motor.controller->modeRunning != Motor::MODE_STOP)
            {
                boardConfig.defaultMode = motor.controller->modeRunning;
                motor.controller->requestMode = Motor::MODE_STOP;
            } else
            {
                motor.controller->requestMode = static_cast<Motor::Mode_t>(boardConfig.defaultMode);
            }
            break;
    }
}


void OnButton2Event(Button::Event _event)
{
    if (motor.controller->IsSoftStopLocked()) return;
    switch (_event)
    {
        case ButtonBase::UP:
            break;
        case ButtonBase::DOWN:
            break;
        case ButtonBase::LONG_PRESS:
            switch (motor.controller->modeRunning)
            {
                case Motor::MODE_COMMAND_CURRENT:
                case Motor::MODE_PWM_CURRENT:
                    motor.controller->SetCurrentSetPoint(0);
                    break;
                case Motor::MODE_COMMAND_VELOCITY:
                case Motor::MODE_PWM_VELOCITY:
                    motor.controller->SetVelocitySetPoint(0);
                    break;
                case Motor::MODE_COMMAND_POSITION:
                case Motor::MODE_PWM_POSITION:
                    motor.controller->SetPositionSetPoint(0);
                    break;
                case Motor::MODE_COMMAND_Trajectory:
                case Motor::MODE_STEP_DIR:
                case Motor::MODE_STOP:
                    break;
            }
            break;
        case ButtonBase::CLICK:
            printf("KEY2\r\n");
            motor.controller->ClearStallFlag();
            break;
    }
}
