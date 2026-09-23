/* 包含的头文件 ------------------------------------------------------------------*/

#include "communication.hpp"  // 包含通信相关的头文件
#include "common_inc.h"       // 包含通用的头文件

/* 私有宏定义 -----------------------------------------------------------*/
/* 私有类型定义 -----------------------------------------------------------*/
/* 全局常量数据 ---------------------------------------------------------*/
/* 全局变量 -------------------------------------------------------------*/
/* 私有常量数据 ----------------------------------------------------------*/
/* 私有变量 -------------------------------------------------------------*/
volatile bool endpointListValid = false;  // 标记端点列表是否有效，初始化为 false

/* 私有函数原型声明 -------------------------------------------------------*/
/* 函数实现 --------------------------------------------------------------*/
// @brief 发送一行数据到指定的输出设备

// 定义通信任务线程句柄
osThreadId_t commTaskHandle;

// 定义通信任务线程的属性
const osThreadAttr_t commTask_attributes = {
        .name = "commTask",                  // 线程名称
        /* CommitProtocol() 通过 COMMIT_PROTOCOL 宏在栈上构造 Fibre 协议树：
         *   sizeof(treeType) ≈ 11.3KB（含8个关节+夹爪的 CtrlStepMotor 协议）。
         * 当编译器未能完全应用 NRVO 时，make_protocol_member_list 的多层嵌套
         * 临时对象会在栈上同时存在，实际最大栈深远超 11.3KB（约 30~40KB）。
         *
         * 注意：不可轻易降低此值，否则 CommitProtocol() 栈溢出会导致：
         *   ① 内存静默损坏 → OLED/RGB 任务无法正常运行
         *   ② 若溢出严重则触发 HardFault → USB 无法枚举，设备管理器看不到 COM 口
         *
         * OledTask 已从 4096 恢复为 2048，为本任务节省约 2KB CCMRAM 堆空间。
         * 当前 CCMRAM 堆（64KB）使用约 61.7KB，余量约 3.8KB，安全。 */
        .stack_size = 45000,
        .priority = (osPriority_t) osPriorityNormal,  // 线程优先级
};

// 初始化通信模块
void InitCommunication(void)
{
    // 启动命令处理线程
    commTaskHandle = osThreadNew(CommunicationTask, nullptr, &commTask_attributes);

    // 等待端点列表有效，防止后续操作提前执行
    while (!endpointListValid)
        osDelay(1);  // 延时 1 毫秒
}

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;  // 声明 USB 外设句柄
osThreadId_t usbIrqTaskHandle;  // USB 中断任务线程句柄

// USB 延迟中断任务
void UsbDeferredInterruptTask(void* ctx)
{
    (void) ctx;  // 忽略未使用的参数

    for (;;)
    {
        // 等待来自 USB 中断的信号（通过 USB 中断处理器信号量）
        osStatus semaphore_status = osSemaphoreAcquire(sem_usb_irq, osWaitForever);
        if (semaphore_status == osOK)
        {
            // 获取到信号，表示有新的 USB 数据传输，需要进行处理
            HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);  // 处理 USB 外设中断
            // 允许中断再次触发（通过 NVIC 控制器启用中断）
            HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
        }
    }
}

// 通信任务线程，处理 USB 中断延迟处理、UART DMA 循环缓冲区读取命令等
void CommunicationTask(void* ctx)
{
    (void) ctx;  // 忽略未使用的参数

    CommitProtocol();  // 提交协议

    // 允许主程序继续初始化
    endpointListValid = true;  // 标记端点列表有效

    // 启动各个通信服务器
    StartUartServer();  // 启动 UART 服务器
    StartUsbServer();   // 启动 USB 服务器
    StartCanServer(CAN1);  // 启动 CAN1 服务器
    StartCanServer(CAN2);  // 启动 CAN2 服务器

    for (;;)
    {
        osDelay(1000);  // 无操作，等待 1000 毫秒
    }
}

// 重定向标准输出流（如 printf）到 USB 和 UART
extern "C" {
int _write(int file, const char* data, int len);  // 声明重定向函数
}

// @brief 重定向 printf 调用的函数
int _write(int file, const char* data, int len)
{
    // 将数据通过 USB 输出流和 UART4 输出流分别处理
    usbStreamOutputPtr->process_bytes((const uint8_t*) data, len, nullptr);
    uart4StreamOutputPtr->process_bytes((const uint8_t*) data, len, nullptr);

    return len;  // 返回处理的字节数
}
