#include "common_inc.h"
#include "instances/cartesian_benchmark.h"
#include "instances/bench_profile.h"


// On-board Screen, can choose from hi2c2 or hi2c0(soft i2c)
SSD1306 oled(&hi2c0);
// On-board Sensor, used hi2c1
MPU6050 mpu6050(&hi2c1);
// 5 User-Timers, can choose from htim7/htim10/htim11/htim13/htim14
Timer timerCtrlLoop(&htim7, 1000); // Stream mailbox pump; ordinary loop remains 200 Hz.
// 2x2-channel PWMs, used htim9 & htim12, each has 2-channel outputs
PWM pwm(21000, 21000);

RGB rgb(0);
// Robot instance
DummyRobot dummy(&hcan1);


/* Thread Definitions -----------------------------------------------------*/
osThreadId_t controlLoopFixUpdateHandle;
// Static SRAM avoids consuming additional CCM startup heap for RT headroom.
alignas(8) static StackType_t realtimeTaskStack[4096 / sizeof(StackType_t)];
static StaticTask_t realtimeTaskControlBlock;
void ThreadControlLoopFixUpdate(void* argument)
{
    // §8.2 + §8.8 — 5ms 节奏累加，每 4 次（即 20ms / 50Hz）触发一次角度查询。
    // 使能态、失能态、TUN 模式共享同一个 tick，相位一致。
    static uint32_t queryTick = 0;
    static uint32_t ordinaryTick = 0;

    for (;;)
    {
        // Suspended here until got Notification.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        CoreStreamPoll();
        PollCanMotionTransmit(get_can_ctx(&hcan1));
        if (++ordinaryTick < 5) continue;
        ordinaryTick = 0;
        queryTick++;
        dummy.PollSoftStop();
        dummy.PollProtection();
        dummy.PollMotionCompletion();

        if (dummy.IsEnabled())
        {
            // Send control command to Motors & update Joint states
            switch (dummy.commandMode)
            {
                case DummyRobot::COMMAND_TARGET_POINT_SEQUENTIAL:
                case DummyRobot::COMMAND_TARGET_POINT_INTERRUPTABLE:
                case DummyRobot::COMMAND_CONTINUES_TRAJECTORY:
                    dummy.MoveJoints(dummy.targetJoints);
                    dummy.UpdateJointPose6D();
                    if (dummy.IsMotionBusy() || (queryTick & 0x03) == 0)
                        dummy.UpdateJointAngles();
                    break;
                case DummyRobot::COMMAND_STREAM_JOINT:
                    dummy.UpdateJointPose6D();
                    dummy.UpdateJointAngles();
                    break;
                case DummyRobot::COMMAND_MOTOR_TUNING:
                    dummy.tuningHelper.Tick(10);
                    dummy.UpdateJointPose6D();
                    if ((queryTick & 0x03) == 0)
                        dummy.UpdateJointAngles();
                    break;
            }
        } else
        {
            // 失能态没有 0x07 下发，靠查询拿位置
            // 50Hz 与使能态节奏保持一致
            if ((queryTick & 0x03) == 0) {
                dummy.UpdateJointAngles();
                dummy.UpdateJointPose6D();
            }
        }
    }
}


// Keep the command stack out of the nearly full 64 KiB CCM RTOS heap.
// Size is bytes in CMSIS; FreeRTOS's StackType_t array uses words.
alignas(8) static StackType_t commandTaskStack[8192 / sizeof(StackType_t)];
static StaticTask_t commandTaskControlBlock;
osThreadId_t ControlLoopUpdateHandle;
void ThreadControlLoopUpdate(void* argument)
{
    for (;;)
    {
        if(CartesianBenchmarkActive()){CartesianBenchmarkPoll();continue;}
        // A bounded idle wait lets a USB diagnostic request reach this normal
        // priority owner. No benchmark work enters the 1 kHz realtime pump.
        dummy.commandHandler.ParseCommand(dummy.commandHandler.Pop(pdMS_TO_TICKS(20)));
    }
}


osThreadId_t oledTaskHandle;
osThreadId_t oledDisplayTaskHandle;
// The additional task uses SRAM, leaving the existing CCM task heap intact.
alignas(8) static StackType_t oledDisplayTaskStack[2048 / sizeof(StackType_t)];
static StaticTask_t oledDisplayTaskControlBlock;
namespace {
struct OledDisplayState {
    float imuX=0,imuY=0,joints[6]{},rail=0,pose[6]{};
    uint32_t jointFlags=0,commandMode=2,tick=0;
    bool enabled=false;
};
OledDisplayState oledMailbox;
OledDiagnostic::RuntimeSnapshot oledRuntime;
volatile unsigned oledRequestedMode=0,oledActiveMode=0;
bool oledDisplayBusy=false,oledMailboxPending=false,oledRestoreAllowed=true;
uint32_t oledLastRenderMicros=0;
bool oledSendMonitoring=false;
uint32_t oledLastCompletedSendTick=0;
constexpr uint32_t OledStartTimeout=1U,OledStopTimeout=2U,OledPriorityError=4U;
constexpr TickType_t OledHandoffTimeout=pdMS_TO_TICKS(500);

BenchProfile::OledToken BeginOledFrame(TickType_t planned) {
    if(!benchOledSessionActive)return {};
    taskENTER_CRITICAL();
    const auto token=BenchProfile::OledFrameBegin(DWT->CYCCNT,xTaskGetTickCount(),planned);
    taskEXIT_CRITICAL();return token;
}
void SetOledStage(const BenchProfile::OledToken& token,unsigned stage) {
    if(!token.generation)return;
    taskENTER_CRITICAL();BenchProfile::OledStageAt(token,stage,DWT->CYCCNT);taskEXIT_CRITICAL();
}
void EndOledFrame(const BenchProfile::OledToken& token) {
    if(!token.generation)return;
    taskENTER_CRITICAL();BenchProfile::OledFrameEnd(token,DWT->CYCCNT);taskEXIT_CRITICAL();
}
bool SetOledDisplayPriority(osPriority_t priority) {
    return oledDisplayTaskHandle && osThreadSetPriority(oledDisplayTaskHandle,priority)==osOK
        && osThreadGetPriority(oledDisplayTaskHandle)==priority;
}
// Called only at a worker iteration boundary, after its previous send ended.
bool CoordinateOledOwner() {
    taskENTER_CRITICAL();
    if(oledRequestedMode>=3U) {
        if(oledActiveMode==0U)oledActiveMode=oledRequestedMode;
    } else if(oledActiveMode>=3U && !oledDisplayBusy && oledRestoreAllowed) {
        oledActiveMode=0;oledMailboxPending=false;oledRuntime.restored=1;
    }
    const bool split=oledActiveMode>=3U;
    taskEXIT_CRITICAL();return split;
}
OledDisplayState CaptureOledDisplayState() {
    OledDisplayState state;
    taskENTER_CRITICAL();
    state.imuX=mpu6050.data.ax;state.imuY=mpu6050.data.ay;
    for(unsigned i=0;i<6;++i)state.joints[i]=dummy.currentJoints.a[i];
    state.rail=dummy.currentRailPos;
    state.pose[0]=dummy.currentPose6D.X;state.pose[1]=dummy.currentPose6D.Y;state.pose[2]=dummy.currentPose6D.Z;
    state.pose[3]=dummy.currentPose6D.A;state.pose[4]=dummy.currentPose6D.B;state.pose[5]=dummy.currentPose6D.C;
    state.jointFlags=dummy.jointsStateFlag;state.commandMode=unsigned(dummy.commandMode);state.enabled=dummy.IsEnabled();
    state.tick=xTaskGetTickCount();
    taskEXIT_CRITICAL();return state;
}
void PublishOledDisplayState(const OledDisplayState& state) {
    taskENTER_CRITICAL();
    const bool publish=oledActiveMode>=3U && oledRequestedMode==oledActiveMode;
    if(publish) {
        if(oledMailboxPending)++oledRuntime.overwritten;
        oledMailbox=state;oledMailboxPending=true;
    }
    taskEXIT_CRITICAL();
    if(publish)xTaskNotifyGive(TaskHandle_t(oledDisplayTaskHandle));
}
void ObserveOledSend(bool completed) {
    taskENTER_CRITICAL();
    if(oledSendMonitoring) {
        const uint32_t now=xTaskGetTickCount();
        const uint32_t gap=now-oledLastCompletedSendTick;
        if(gap>oledRuntime.maxSendGapTicks)oledRuntime.maxSendGapTicks=gap;
        if(completed)oledLastCompletedSendTick=now;
    }
    taskEXIT_CRITICAL();
}
void RenderOledDisplayState(const OledDisplayState& state,const BenchProfile::OledToken& oledFrame) {
    uint32_t& t=oledLastRenderMicros;
    char buf[16];
    const char cmdModeNames[7][4]={"SEQ","INT","TRJ","TUN","---","---","STR"};
    SetOledStage(oledFrame,BenchProfile::OLED_DRAW);
    oled.clearBuffer();
    oled.setFont(u8g2_font_5x8_tr);
    oled.setCursor(0, 10);
    oled.printf("IMU:%.3f/%.3f", state.imuX, state.imuY);
    oled.setCursor(85, 10);
    oled.printf("| FPS:%lu", 1000000 / (micros() - t));
    t = micros();

    oled.drawBox(0, 15, 128, 3);
    oled.setCursor(0, 30);
    oled.printf(">%3d|%3d|%3d|%3d|%3d|%3d|",
                (int) roundf(state.joints[0]), (int) roundf(state.joints[1]),
                (int) roundf(state.joints[2]), (int) roundf(state.joints[3]),
                (int) roundf(state.joints[4]), (int) roundf(state.joints[5]));
    oled.setCursor(17, 40);
    oled.printf("rail:%.2fmm", state.rail);

    // [FIX] 模式 + 运动状态上移到中部（10x20），紧跟 rail
    oled.setFont(u8g2_font_10x20_tr);
    oled.setCursor(0, 58);
    if (state.enabled)
    {
        for (int i = 1; i <= 7; i++)
            buf[i - 1] = (state.jointFlags & (1 << i) ? '*' : '_');
        buf[7] = 0;
        oled.printf("%s: %s", cmdModeNames[state.commandMode - 1], buf);
    } else
    {
        oled.printf("%s: %s", cmdModeNames[state.commandMode - 1], "=======");
    }

    // [FIX] XYZ/ABC 下移到屏幕底部：标签在框左外侧，6 个值在框内右侧（6x10 + 白框）
    oled.drawBox(40, 60, 128, 20);
    oled.setFont(u8g2_font_6x10_tr);
    oled.setDrawColor(0);
    oled.setCursor(42, 70);
    oled.printf("%4d|%4d|%4d", (int) roundf(state.pose[0]),
                (int) roundf(state.pose[1]), (int) roundf(state.pose[2]));
    oled.setCursor(42, 79);
    oled.printf("%4d|%4d|%4d", (int) roundf(state.pose[3]),
                (int) roundf(state.pose[4]), (int) roundf(state.pose[5]));
    oled.setDrawColor(1);
    oled.setCursor(0, 70);
    oled.printf("[XYZ]:");
    oled.setCursor(0, 79);
    oled.printf("[ABC]:");

    SetOledStage(oledFrame,BenchProfile::OLED_SEND);
    oled.sendBuffer();
    ObserveOledSend(true);
}
}
namespace OledDiagnostic {
bool Busy() {
    // FreeRTOS permits this short nested critical section at guarded callers.
    taskENTER_CRITICAL();
    const bool busy=oledRequestedMode!=0U || oledActiveMode!=0U || oledDisplayBusy || !oledRuntime.restored;
    taskEXIT_CRITICAL();return busy;
}
bool Stop() {
    ObserveOledSend(false);
    taskENTER_CRITICAL();oledRequestedMode=0;oledMailboxPending=false;oledRestoreAllowed=false;
    oledRuntime.restored=0;oledSendMonitoring=false;taskEXIT_CRITICAL();
    const bool priorityOk=SetOledDisplayPriority(osPriorityNormal);
    taskENTER_CRITICAL();
    oledRestoreAllowed=priorityOk;
    if(!priorityOk)oledRuntime.flags|=OledPriorityError;
    taskEXIT_CRITICAL();
    if(!priorityOk)return false;
    const TickType_t began=xTaskGetTickCount();
    do {
        taskENTER_CRITICAL();
        const bool stopped=oledActiveMode==0U&&!oledDisplayBusy;
        if(stopped)oledRuntime.restored=1;
        taskEXIT_CRITICAL();
        if(stopped)return true;
        osDelay(1);
    } while(TickType_t(xTaskGetTickCount()-began)<OledHandoffTimeout);
    taskENTER_CRITICAL();oledRuntime.flags|=OledStopTimeout;taskEXIT_CRITICAL();return false;
}
bool Start(unsigned mode) {
    if(mode>4U || Busy())return false;
    taskENTER_CRITICAL();oledRuntime=RuntimeSnapshot{};oledRuntime.priority=unsigned(osPriorityNormal);
    oledMailboxPending=false;oledRequestedMode=0;oledRestoreAllowed=true;
    oledLastCompletedSendTick=xTaskGetTickCount();oledSendMonitoring=true;taskEXIT_CRITICAL();
    if(!oledTaskHandle||!oledDisplayTaskHandle) {
        taskENTER_CRITICAL();oledRuntime.flags|=OledPriorityError;taskEXIT_CRITICAL();return false;
    }
    if(mode<3U)return true;
    const auto priority=mode==4U?osPriorityBelowNormal:osPriorityNormal;
    const bool priorityOk=SetOledDisplayPriority(priority);
    taskENTER_CRITICAL();
    oledRuntime.restored=0;oledRestoreAllowed=false;
    if(priorityOk){oledRuntime.priority=unsigned(priority);oledRequestedMode=mode;}
    else oledRuntime.flags|=OledPriorityError;
    taskEXIT_CRITICAL();
    if(!priorityOk){Stop();return false;}
    const TickType_t began=xTaskGetTickCount();
    do {
        taskENTER_CRITICAL();const bool started=oledActiveMode==mode&&oledRequestedMode==mode;taskEXIT_CRITICAL();
        if(started)return true;
        osDelay(1);
    } while(TickType_t(xTaskGetTickCount()-began)<OledHandoffTimeout);
    taskENTER_CRITICAL();oledRuntime.flags|=OledStartTimeout;taskEXIT_CRITICAL();Stop();return false;
}
RuntimeSnapshot Snapshot() {
    RuntimeSnapshot result;
    taskENTER_CRITICAL();result=oledRuntime;taskEXIT_CRITICAL();
    result.displayStackFree=oledDisplayTaskHandle?osThreadGetStackSpace(oledDisplayTaskHandle):0;
    return result;
}
}
void OledWorkerIteration(TickType_t planned) {
    ObserveOledSend(false);
    const bool split=CoordinateOledOwner();
    const auto oledFrame=BeginOledFrame(planned);
    mpu6050.Update(true);
    SetOledStage(oledFrame,BenchProfile::OLED_SERVICE);
    dummy.PollPidPrint();
    dummy.PollStopPrint();
    dummy.commandHandler.PollCancelled();
    dummy.PollProtectionPrint();
    dummy.PollMotorAccelerationSync();
    CoreStreamPrint();
    const auto state=CaptureOledDisplayState();
    if(split)PublishOledDisplayState(state);
    else if(!oledFrame.skip)RenderOledDisplayState(state,oledFrame);
    EndOledFrame(oledFrame);
}
void OledDisplayIteration() {
    OledDisplayState state;BenchProfile::OledToken token;
    taskENTER_CRITICAL();
    const bool ready=oledActiveMode>=3U && oledRequestedMode==oledActiveMode && oledMailboxPending && !oledDisplayBusy;
    if(ready) {
        state=oledMailbox;oledMailboxPending=false;oledDisplayBusy=true;
        const TickType_t now=xTaskGetTickCount();
        token=BenchProfile::OledDisplayFrameBegin(DWT->CYCCNT,now,state.tick);
        if(token.generation&&!token.skip) {
            const uint32_t age=now-state.tick;
            if(age>oledRuntime.maxSnapshotAgeTicks)oledRuntime.maxSnapshotAgeTicks=age;
        }
    }
    taskEXIT_CRITICAL();
    if(!ready)return;
    if(token.generation&&!token.skip)RenderOledDisplayState(state,token);
    EndOledFrame(token);
    taskENTER_CRITICAL();oledDisplayBusy=false;taskEXIT_CRITICAL();
}
void ThreadOledUpdate(void* argument) {
    TickType_t wake=xTaskGetTickCount();
    oledLastRenderMicros=micros();
    for(;;) {
        // The legacy worker/IMU/services retain their 20ms opportunities.
        vTaskDelayUntil(&wake,pdMS_TO_TICKS(20));
        OledWorkerIteration(wake);
    }
}
void ThreadOledDisplay(void* argument) {
    for(;;) {
        // Clear all pending notifications; the mailbox contains only latest.
        ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
        OledDisplayIteration();
    }
}


/* Timer Callbacks -------------------------------------------------------*/
void OnTimer7Callback()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Wake & invoke thread IMMEDIATELY.
    vTaskNotifyGiveFromISR(TaskHandle_t(controlLoopFixUpdateHandle), &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

osThreadId_t rgbTaskHandle;
void ThreadRGBUpdate(void* argument) {
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        rgb.Update(dummy.GetRGBState(), HAL_GetTick());
        if (xTaskGetTickCount() - wake >= pdMS_TO_TICKS(40)) wake = xTaskGetTickCount();
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(20));
    }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance==TIM2 && htim->Channel==HAL_TIM_ACTIVE_CHANNEL_4)
    {
        rgb.Interrupt(1);
    }
}

void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) rgb.Interrupt(2);
}

/* Default Entry -------------------------------------------------------*/
void Main(void)
{
    rgb.LoadConfig();
    // Init all communication staff, including USB-CDC/VCP/UART/CAN etc.
    InitCommunication();

    // Init Robot.
    dummy.Init();

    // Init IMU.
    do
    {
        mpu6050.Init();
        osDelay(100);
    } while (!mpu6050.testConnection());
    mpu6050.InitFilter(200, 100, 50);

    // Init OLED 128x80.
    oled.Init();
    pwm.Start();

    // Init & Run User Threads.
    const osThreadAttr_t controlLoopTask_attributes = {
        .name = "ControlLoopFixUpdateTask",
        .cb_mem = &realtimeTaskControlBlock,
        .cb_size = sizeof(realtimeTaskControlBlock),
        .stack_mem = realtimeTaskStack,
        .stack_size = sizeof(realtimeTaskStack),
        .priority = (osPriority_t) osPriorityRealtime,
    };
    controlLoopFixUpdateHandle = osThreadNew(ThreadControlLoopFixUpdate, nullptr,
                                             &controlLoopTask_attributes);

    const osThreadAttr_t ControlLoopUpdateTask_attributes = {
        .name = "ControlLoopUpdateTask",
        .cb_mem = &commandTaskControlBlock,
        .cb_size = sizeof(commandTaskControlBlock),
        .stack_mem = commandTaskStack,
        .stack_size = sizeof(commandTaskStack),
        .priority = (osPriority_t) osPriorityNormal,
    };
    ControlLoopUpdateHandle = osThreadNew(ThreadControlLoopUpdate, nullptr,
                                          &ControlLoopUpdateTask_attributes);
    configASSERT(ControlLoopUpdateHandle != nullptr);

    const osThreadAttr_t oledDisplayTask_attributes = {
        .name = "OledDisplayTask",
        .cb_mem = &oledDisplayTaskControlBlock,
        .cb_size = sizeof(oledDisplayTaskControlBlock),
        .stack_mem = oledDisplayTaskStack,
        .stack_size = sizeof(oledDisplayTaskStack),
        .priority = (osPriority_t) osPriorityNormal,
    };
    oledDisplayTaskHandle = osThreadNew(ThreadOledDisplay, nullptr, &oledDisplayTask_attributes);
    configASSERT(oledDisplayTaskHandle != nullptr);

    const osThreadAttr_t oledTask_attributes = {
        .name = "OledTask",
        .stack_size = 2000,
        .priority = (osPriority_t) osPriorityNormal,   // should >= Normal
    };
    oledTaskHandle = osThreadNew(ThreadOledUpdate, nullptr, &oledTask_attributes);

    const osThreadAttr_t rgbTask_attributes = {
            .name = "RGBTask",
            .stack_size = 2000,
            .priority = (osPriority_t) osPriorityNormal,   // should >= Normal
    };
    rgbTaskHandle = osThreadNew(ThreadRGBUpdate, nullptr, &rgbTask_attributes);

    // Start Timer Callbacks.
    timerCtrlLoop.SetCallback(OnTimer7Callback);
    timerCtrlLoop.Start();

    // System started, light switch-led up.
    Respond(*uart4StreamOutputPtr, "[sys] Heap remain: %d Bytes\n", xPortGetMinimumEverFreeHeapSize());
    pwm.SetDuty(PWM::CH_A1, 0.5);
}


// Historical minimum unused bytes, sampled only when explicitly queried.
uint32_t CommandTaskStackFreeBytes(void) {
    return ControlLoopUpdateHandle ? osThreadGetStackSpace(ControlLoopUpdateHandle) : 0;
}
uint32_t RealtimeTaskStackFreeBytes(void) {
    return controlLoopFixUpdateHandle ? osThreadGetStackSpace(controlLoopFixUpdateHandle) : 0;
}
uint32_t OledTaskStackFreeBytes(void) {
    return oledTaskHandle ? osThreadGetStackSpace(oledTaskHandle) : 0;
}
uint32_t RGBTaskStackFreeBytes(void) {
    return rgbTaskHandle ? osThreadGetStackSpace(rgbTaskHandle) : 0;
}
