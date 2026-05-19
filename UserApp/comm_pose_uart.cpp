#include "comm_pose_uart.h"
#include <cstdio>
#include <cstring>

extern "C" {
#include "usart.h" // CubeMX 生成的 huart1
}

// 如果 usart.h 没有 huart1 的声明，请在此处声明：
// extern "C" UART_HandleTypeDef huart1;

void PoseUART_Init(void)
{
    // 如需 RS485 DE 引脚控制或 DMA 初始化，可在此实现
}

//
// 因为此函数以 C 接口导出，我们在 task 内部以 C++ 知识处理具体类型（调用处是 C++）
// 传入 pose 指针时实际传入 DOF6Kinematic::Pose6D_t*
void PoseUART_SendPose(const void *pose)
{
    if (pose == nullptr) return;

    // 结构体布局由 C++ 端保证，按字段读取：转换到 C++ 层做格式化
    // 为简化，这在 C++ 任务里会直接调用 C++ 层的重载。此处做最小 C 接口包装。
    // 直接使用 HAL UART 阻塞发送。实际应用请改为 DMA 非阻塞方式。
    const char *buf = (const char *)pose; // 占位：不会实际用于格式化
    (void)buf;
    // 注意：具体格式化在 C++ 任务里完成并调用 HAL_UART_Transmit 发送
}