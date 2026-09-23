/*
  Pose sender task: 确保先包含 main.h（提供 STM32F4xx 设备宏与 HAL 配置），
  然后在 extern "C" 中包含 C 头，最后包含 C++ 运动学头。
*/

#include "main.h"
#include "cmsis_os.h"

extern "C" void PoseTask_Create(void);

static void PoseSenderTask(void *pvParameters)
{
    // 最小循环占位，避免链接错误
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

extern "C" void PoseTask_Create(void)
{
    xTaskCreate(PoseSenderTask, "PoseTx", 1024, NULL, tskIDLE_PRIORITY + 2, NULL);
}

void PoseSenderTask_Run()
{
    // 占位实现
}


///opt/clion-2023.2/bin/cmake/linux/x64/bin/cmake -S /mnt/share/dummy/dummy/firmware/dummy-ref-core-fw -B /mnt/share/dummy/dummy/firmware/dummy-ref-core-fw/cmake-build-debug -G "Ninja"
///opt/clion-2023.2/bin/cmake/linux/x64/bin/cmake --build /mnt/share/dummy/dummy/firmware/dummy-ref-core-fw/cmake-build-debug --target Core-STM32F4-fw.elf -j6

