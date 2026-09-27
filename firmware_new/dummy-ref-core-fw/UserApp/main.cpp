#include "common_inc.h"

/* ======================================================================
 * [PID 修复] PID 合并打印 poll（由 OledTask 调用，定义见 can_protocol.cpp）
 * ====================================================================== */
extern "C" void PollPidPrint();

/* ======================================================================
 * [DIAG-2026-09-16] 4 边界心跳计数器
 * 用于定位 "发送就卡死" 故障层级 —— 详见 docs/发送就卡死—心跳定位排查方案—2026-09-16.md
 * ====================================================================== */
volatile uint32_t g_byte_in        = 0;  // ASCII 字节接收 (每字节 +1)
volatile uint32_t g_line_ready     = 0;  // ASCII 行完成 (每行 +1)
volatile uint32_t g_cmd_in         = 0;  // 命令 handler 入口 (每个命令 +1)
volatile uint32_t g_cmd_out        = 0;  // 命令 handler 出口 (每个命令 +1)
volatile uint32_t g_realtime_tick  = 0;  // 5kHz 实时环 tick
volatile uint32_t g_oled_tick      = 0;  // OLED 任务 tick
volatile uint32_t g_rgb_tick       = 0;  // RGB 任务 tick

/* ======================================================================
 * 系统硬件外设层资源静态分配实例
 * ====================================================================== */
SSD1306 oled(&hi2c0);
MPU6050 mpu6050(&hi2c1);
Timer timerCtrlLoop(&htim7, 200);
PWM pwm(21000, 21000);
RGB rgb(0);
DummyRobot dummy(&hcan1);

/* ======================================================================
 * 自由实时操作系统 (FreeRTOS) 任务流调度指引针
 * ====================================================================== */
osThreadId_t controlLoopFixUpdateHandle;
osThreadId_t ControlLoopUpdateHandle;
osThreadId_t oledTaskHandle;
osThreadId_t rgbTaskHandle;

/**
 * @brief 电机运动层微秒级伺服解析主线 (高优先级抢占式)
 * @note  配合系统 5kHz TIM7 硬中断驱动，实现多轴并行梯形加减速与位姿闭环监控
 */
void ThreadControlLoopFixUpdate(void* argument)
{
    for (;;)
    {
        g_realtime_tick++;  // [DIAG-2026-09-16] 5kHz 心跳
        static uint32_t updateCounter = 0;

        // 堵塞当前任务状态，直至 TIM7 送出调度准行号志
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (dummy.IsEnabled())
        {
            switch (dummy.commandMode)
            {
                // [常规格局模式: 序列串行, 打断重切, 滑移连接]
                case DummyRobot::COMMAND_TARGET_POINT_SEQUENTIAL:
                case DummyRobot::COMMAND_TARGET_POINT_INTERRUPTABLE:
                case DummyRobot::COMMAND_CONTINUES_TRAJECTORY:
                    // [总线节流] 50 Hz 下发 (总线 1 Mbps / 8B DLC ≈ 7700 fps,
                    //   7 电机 × 50 Hz = 350 fps, 富余 ~22 倍; 5 kHz 直发会到
                    //   35 000 fps 撞爆总线, 实时环在 sem 上死等 3.85× tick
                    //   周期, 饿死 Normal 优先级 OLED/RGB/ASCII 任务)
                    if (updateCounter % 100 == 0) {
                        dummy.MoveJoints(dummy.targetJoints);
                        // [v3.1] J7/地轨: 直接下发 (railSpeedRps 由 MoveJ 计算填入)
                        dummy.motorJ[7]->SetPositionWithMotorRps(
                            dummy.targetRailPos / 5.0f, dummy.railSpeedRps);
                    }
                    // [方案C] 单播轮询, 100 Hz / 7 轴 ≈ 14 Hz/电机
                    // 基于刚体计算正逆解析法进行空间 TCP 位置计算追踪
                    dummy.UpdateJointPose6D();
                    break;

                // [特种应用模式: 极速伺服随动层]
                case DummyRobot::COMMAND_SERVO_J:
                    // 高频下发臂关节和地轨
                    dummy.MoveJoints(dummy.targetJoints);
                    // [v3.1] J7/地轨：直接下发（railSpeedRps 默认 30 r/s）
                    dummy.motorJ[7]->SetPositionWithMotorRps(
                        dummy.targetRailPos / 5.0f, dummy.railSpeedRps);
                    dummy.UpdateJointPose6D();
                    break;

                // [工程标定模式: 单关节信号辨识发生器]
                case DummyRobot::COMMAND_MOTOR_TUNING:
                    if (updateCounter % 50 == 0) {
                        dummy.tuningHelper.Tick(10);
                    }
                    dummy.UpdateJointPose6D();
                    break;

                // [特种应用模式: 粗暴力矩纯压输入操控]
                case DummyRobot::COMMAND_TORQUE_CONTROL:
                    if (updateCounter % 50 == 0) {
                        for (int i = 1; i <= 6; i++) {
                            dummy.motorJ[i]->SetCurrentSetPoint(dummy.targetCurrents[i-1]);
                        }
                        // [v3.1] J7/地轨电流
                        dummy.motorJ[7]->SetCurrentSetPoint(dummy.targetRailCurrent);
                    }
                    dummy.UpdateJointPose6D();
                    break;
            }
        }
        else
        {
            // 防呆或休眠情况中持续刷新观测位表，确切知晓关节在外力拖拽下的漂移姿态
            dummy.UpdateJointAngles();
            dummy.UpdateJointPose6D();
        }
        updateCounter++;
    }
}

/**
 * @brief 指令栈异步流清洗消费者线程
 * @note  剥离复杂文本解析逻辑至单独队列处理空间避免系统被过高的命令阻塞挂死
 */
void ThreadControlLoopUpdate(void* argument)
{
    for (;;)
    {
        const char* _cmd = dummy.commandHandler.Pop(osWaitForever);
        if (_cmd != nullptr)
            dummy.commandHandler.ParseCommand(_cmd);
    }
}

/**
 * @brief 终端运行视窗实时绘图逻辑处理引擎
 * @note  采集重要状态参数生成 Oled 面板 UI 图表交互数据阵列
 */
void ThreadOledUpdate(void* argument)
{
    uint32_t lastMicros  = micros();
    uint32_t lastImuUpdate = 0;
    char buf[16];

    char cmdModeNames[6][4] = {"SEQ", "INT", "TRJ", "TUN", "TRQ", "SRV"};

    for (;;)
    {
        g_oled_tick++;  // [DIAG-2026-09-16] OLED 心跳 (per-iteration)
        uint32_t currentTime = HAL_GetTick();

        // 强行约束 IMU I2C 外设的数据检索轮询耗时频率在 100Hz 最大边界内
        if (currentTime - lastImuUpdate >= 10)
        {
            mpu6050.Update(true);
            lastImuUpdate = currentTime;
        }

        oled.clearBuffer();

        // 【首行顶栏信息模块】包括防翻倒监测倾角计算与图像面板执行渲染效能指示器
        oled.setFont(u8g2_font_5x8_tr);
        oled.setCursor(0, 10);
        oled.printf("IMU:%.3f/%.3f", mpu6050.data.ax, mpu6050.data.ay);

        uint32_t nowMicros = micros();
        uint32_t elapsed   = nowMicros - lastMicros;
        lastMicros = nowMicros;
        uint32_t fps = (elapsed > 0) ? (1000000u / elapsed) : 0u;

        oled.setCursor(85, 10);
        oled.printf("| FPS:%lu", fps);

        oled.drawBox(0, 15, 128, 3); // 划定内容核心显示界线

        // 【角度数字矩阵展示】实时展列机器关节旋转实值
        oled.setCursor(0, 30);
        oled.printf(">%3d|%3d|%3d|%3d|%3d|%3d",
                    (int)roundf(dummy.currentJoints.a[0]),
                    (int)roundf(dummy.currentJoints.a[1]),
                    (int)roundf(dummy.currentJoints.a[2]),
                    (int)roundf(dummy.currentJoints.a[3]),
                    (int)roundf(dummy.currentJoints.a[4]),
                    (int)roundf(dummy.currentJoints.a[5]));

        // 【世界终端坐标反显板】借助逆色覆盖高光显要体现工具原点的移动偏转坐标系数值
        oled.drawBox(40, 35, 128, 24);
        oled.setFont(u8g2_font_6x12_tr);
        oled.setDrawColor(0);

        oled.setCursor(42, 45);
        oled.printf("%4d|%4d|%4d",
                    (int)roundf(dummy.currentPose6D.X),
                    (int)roundf(dummy.currentPose6D.Y),
                    (int)roundf(dummy.currentPose6D.Z));

        oled.setCursor(42, 56);
        oled.printf("%4d|%4d|%4d",
                    (int)roundf(dummy.currentPose6D.A),
                    (int)roundf(dummy.currentPose6D.B),
                    (int)roundf(dummy.currentPose6D.C));

        oled.setDrawColor(1);
        oled.setCursor(0, 45);
        oled.printf("[XYZ]:");
        oled.setCursor(0, 56);
        oled.printf("[ABC]:");

        // 【综合状态横幅通知】提供设备使能以及指令落实执行同步标记进度显示
        // 【心跳探针面板】[DIAG-2026-09-16] 4 边界 + 3 任务探针 + 关节完成标志
        oled.setFont(u8g2_font_5x8_tr);
        oled.setCursor(0, 62);
        oled.printf("FLAG:0x%02X/%02X", dummy.jointsStateFlag, 0x7E);
        oled.setCursor(0, 70);
        oled.printf("RT:%lu OL:%lu RGB:%lu",
                    g_realtime_tick, g_oled_tick, g_rgb_tick);
        oled.setCursor(0, 78);
        oled.printf("B:%lu L:%lu I:%lu O:%lu",
                    g_byte_in, g_line_ready, g_cmd_in, g_cmd_out);
        // 旧的 cmdMode/joints 横幅已由心跳探针取代
        // （调试阶段不需要 [SRV] __*__*_ 样式，恢复方法：见文档 §四.6）

        oled.sendBuffer();

        // [PID 修复] PID 合并打印 poll（避免 ISR 内 printf 影响 CAN RX ISR）
        // 详见 can_protocol.cpp PollPidPrint()
        PollPidPrint();
    }
}

/**
 * @brief 系统时钟骨干网络接入口
 * @note  借助 ISR 触发机制以极高特权唤醒被屏蔽的解算与发送任务流水线
 */
void OnTimer7Callback()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    vTaskNotifyGiveFromISR(
            TaskHandle_t(controlLoopFixUpdateHandle),
            &xHigherPriorityTaskWoken
    );

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief 系统炫彩指示光效动态播放管家
 * @note  rgb.Run() 内部使用 WS2812_Send() 启动 DMA 传输；
 *        UpdateLoop() 在每次 Run() 前非阻塞检查 DMA 是否空闲，
 *        避免上一次 DMA 未完成时被自旋锁阻塞整个系统。
 */
void ThreadRGBUpdate(void* argument)
{
    rgb.InitSync();  // 初始化事件标志（一次性）

    for (;;)
    {
        g_rgb_tick++;  // [DIAG-2026-09-16] RGB 心跳 (per-iteration)
        // 先检查 DMA 状态：DMA 空闲才执行灯效；DMA 忙则跳过本帧
        if (rgb.UpdateLoop())
        {
            rgb.Run((RGB::Rgb_style_t)dummy.GetRGBMode());
        }
        osDelay(30);
    }
}

/**
 * @brief 挂载在 HAL 栈上的高速像素映射投递终点拦截哨
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_4);
        rgb.Interrupt(1);
    }
}

/**
 * @brief 程序执行主轴 - 一次性部署软硬件联调方案入口
 */
void Main(void)
{
    // [步骤1] 创建下底层交互硬件基站和数据出入栈道
    InitCommunication();
    Respond(*uart4StreamOutputPtr, "[sys] comm init ok\n");

    // [步骤2] 构建机组控制树及载入全局 EEPROM 常驻缓存字典
    dummy.Init();
    Respond(*uart4StreamOutputPtr, "[sys] robot init ok\n");

    // [步骤3] 洗脱 IMU 总线静滞并布置高低频融合互补滤波基准
    mpu6050.Init();
    HAL_Delay(100);
    mpu6050.InitFilter(200, 100, 50);
    Respond(*uart4StreamOutputPtr, "[sys] imu init ok\n");

    // [步骤4] 开启直观数据显示矩阵与执行末端供电控制阀
    oled.Init();
    Respond(*uart4StreamOutputPtr, "[sys] oled init ok\n");
    pwm.Start();

    // [步骤5] 精准刻画各工作包栈体深度、任务归属以及并发抢占排序
    const osThreadAttr_t controlLoopTask_attributes = {
            .name = "ControlLoopFixUpdateTask",
            .stack_size = 2000,
            .priority = (osPriority_t)osPriorityRealtime,
    };
    controlLoopFixUpdateHandle =
            osThreadNew(ThreadControlLoopFixUpdate, nullptr,
                        &controlLoopTask_attributes);

    const osThreadAttr_t ControlLoopUpdateTask_attributes = {
            .name = "ControlLoopUpdateTask",
            .stack_size = 2000,
            .priority = (osPriority_t)osPriorityNormal,
    };
    ControlLoopUpdateHandle =
            osThreadNew(ThreadControlLoopUpdate, nullptr,
                        &ControlLoopUpdateTask_attributes);

    const osThreadAttr_t oledTask_attributes = {
            .name = "OledTask",
            .stack_size = 2000, 
            .priority = (osPriority_t)osPriorityNormal,
    };
    oledTaskHandle =
            osThreadNew(ThreadOledUpdate, nullptr, &oledTask_attributes);

    const osThreadAttr_t rgbTask_attributes = {
            .name = "RGBTask",
            .stack_size = 2000,
            .priority = (osPriority_t)osPriorityNormal,
    };
    rgbTaskHandle =
            osThreadNew(ThreadRGBUpdate, nullptr, &rgbTask_attributes);

    // [步骤6] 启用命脉发生器拨动机械大脑运算生命时钟
    timerCtrlLoop.SetCallback(OnTimer7Callback);
    timerCtrlLoop.Start();

    // [步骤7] 打印核验结果并在交互终端汇报运行时资源宽裕度
    Respond(*uart4StreamOutputPtr,
            "[sys] threads active: ctrl=%d upd=%d oled=%d rgb=%d\n",
            (int)(controlLoopFixUpdateHandle != nullptr),
            (int)(ControlLoopUpdateHandle    != nullptr),
            (int)(oledTaskHandle             != nullptr),
            (int)(rgbTaskHandle              != nullptr));

    Respond(*uart4StreamOutputPtr,
            "[sys] Heap remain: %d Bytes\n",
            xPortGetMinimumEverFreeHeapSize());

    pwm.SetDuty(PWM::CH_A1, 0.5);
}
