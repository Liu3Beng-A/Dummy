///* USER CODE BEGIN Header */
///**
//  ******************************************************************************
//  * File Name          : freertos.c
//  * Description        : Code for freertos applications
//  ******************************************************************************
//  * @attention
//  *
//  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
//  * All rights reserved.</center></h2>
//  *
//  * This software component is licensed by ST under Ultimate Liberty license
//  * SLA0044, the "License"; You may not use this file except in compliance with
//  * the License. You may obtain a copy of the License at:
//  *                             www.st.com/SLA0044
//  *
//  ******************************************************************************
//  */
///* USER CODE END Header */
//
///* Includes ------------------------------------------------------------------*/
//#include "FreeRTOS.h"
//#include "task.h"
//#include "main.h"
//#include "cmsis_os.h"
//
///* Private includes ----------------------------------------------------------*/
///* USER CODE BEGIN Includes */
//#include "common_inc.h"
//#include "communication.hpp"
///* USER CODE END Includes */
//
///* Private typedef -----------------------------------------------------------*/
///* USER CODE BEGIN PTD */
//
///* USER CODE END PTD */
//
///* Private define ------------------------------------------------------------*/
///* USER CODE BEGIN PD */
//
///* USER CODE END PD */
//
///* Private macro -------------------------------------------------------------*/
///* USER CODE BEGIN PM */
//
///* USER CODE END PM */
//
///* Private variables ---------------------------------------------------------*/
///* USER CODE BEGIN Variables */
//
//// List of semaphores
//osSemaphoreId sem_usb_irq;
//osSemaphoreId sem_uart4_dma;
//osSemaphoreId sem_uart5_dma;
//osSemaphoreId sem_usb_rx;
//osSemaphoreId sem_usb_tx;
//osSemaphoreId sem_can1_tx;
//osSemaphoreId sem_can2_tx;
//
///* USER CODE END Variables */
///* Definitions for defaultTask */
//osThreadId_t defaultTaskHandle;
//const osThreadAttr_t defaultTask_attributes = {
//  .name = "defaultTask",
//  .stack_size = 500 * 4,
//  .priority = (osPriority_t) osPriorityNormal,
//};
//
///* Private function prototypes -----------------------------------------------*/
///* USER CODE BEGIN FunctionPrototypes */
//
//
///* USER CODE END FunctionPrototypes */
//
//void StartDefaultTask(void *argument);
//
//extern void MX_USB_DEVICE_Init(void);
//void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */
//
///**
//  * @brief  FreeRTOS initialization
//  * @param  None
//  * @retval None
//  */
//void MX_FREERTOS_Init(void) {
//  /* USER CODE BEGIN Init */
//
//  /* USER CODE END Init */
//
//  /* USER CODE BEGIN RTOS_MUTEX */
//    /* add mutexes, ... */
//  /* USER CODE END RTOS_MUTEX */
//
//  /* USER CODE BEGIN RTOS_SEMAPHORES */
//    // Init usb irq binary semaphore, and start with no tokens by removing the starting one.
//    osSemaphoreDef(sem_usb_irq);
//    sem_usb_irq = osSemaphoreNew(1, 0, osSemaphore(sem_usb_irq));
//
//    // Create a semaphore for UART DMA and remove a token
//    osSemaphoreDef(sem_uart4_dma);
//    sem_uart4_dma = osSemaphoreNew(1, 1, osSemaphore(sem_uart4_dma));
//    osSemaphoreDef(sem_uart5_dma);
//    sem_uart5_dma = osSemaphoreNew(1, 1, osSemaphore(sem_uart5_dma));
//
//    // Create a semaphore for USB RX, and start with no tokens by removing the starting one.
//    osSemaphoreDef(sem_usb_rx);
//    sem_usb_rx = osSemaphoreNew(1, 0, osSemaphore(sem_usb_rx));
//
//    // Create a semaphore for USB TX
//    osSemaphoreDef(sem_usb_tx);
//    sem_usb_tx = osSemaphoreNew(1, 1, osSemaphore(sem_usb_tx));
//
//    // Create a semaphore for CAN TX
//    osSemaphoreDef(sem_can1_tx);
//    sem_can1_tx = osSemaphoreNew(1, 1, osSemaphore(sem_can1_tx));
//    osSemaphoreDef(sem_can2_tx);
//    sem_can2_tx = osSemaphoreNew(1, 1, osSemaphore(sem_can2_tx));
//
//  /* USER CODE END RTOS_SEMAPHORES */
//
//  /* USER CODE BEGIN RTOS_TIMERS */
//
//  /* USER CODE END RTOS_TIMERS */
//
//  /* USER CODE BEGIN RTOS_QUEUES */
//    // This Task must run before MX_USB_DEVICE_Init(), so have to put it here.
//    const osThreadAttr_t usbIrqTask_attributes = {
//        .name = "usbIrqTask",
//        .stack_size = 500,
//        .priority = (osPriority_t) osPriorityAboveNormal,
//    };
//    usbIrqTaskHandle = osThreadNew(UsbDeferredInterruptTask, NULL, &usbIrqTask_attributes);
//
//  /* USER CODE END RTOS_QUEUES */
//
//  /* Create the thread(s) */
//  /* creation of defaultTask */
//  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
//
//  /* USER CODE BEGIN RTOS_THREADS */
//    /* add threads, ... */
//  /* USER CODE END RTOS_THREADS */
//
//  /* USER CODE BEGIN RTOS_EVENTS */
//    /* add events, ... */
//  /* USER CODE END RTOS_EVENTS */
//
//}
//
///* USER CODE BEGIN Header_StartDefaultTask */
///**
//  * @brief  Function implementing the defaultTask thread.
//  * @param  argument: Not used
//  * @retval None
//  */
///* USER CODE END Header_StartDefaultTask */
//void StartDefaultTask(void *argument)
//{
//  /* init code for USB_DEVICE */
//  MX_USB_DEVICE_Init();
//  /* USER CODE BEGIN StartDefaultTask */
//
//    // Invoke cpp-version main().
//    Main();
//
//    vTaskDelete(defaultTaskHandle);
//  /* USER CODE END StartDefaultTask */
//}
//
///* Private application code --------------------------------------------------*/
///* USER CODE BEGIN Application */
//
///* USER CODE END Application */
//
///************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/




/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : FreeRTOS 应用层代码
  ******************************************************************************
  * @attention
  *
  * 本文件用于：
  *  - 初始化 FreeRTOS 内核对象（任务、信号量等）
  *  - 创建系统基础任务
  *  - 启动 USB、通信与 C++ 主逻辑
  *
  * <h2><center>&copy; STMicroelectronics 版权所有</center></h2>
  *
  * 本软件组件遵循 ST Ultimate Liberty license (SLA0044)
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* ======================= 头文件包含 ======================= */
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* ======================= 用户自定义头文件 ======================= */
/* USER CODE BEGIN Includes */
#include "common_inc.h"          // 公共配置、工具函数
#include "communication.hpp"     // 通信模块（USB / UART / CAN）
/* USER CODE END Includes */

/* ======================= 类型定义 ======================= */
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* ======================= 宏定义 ======================= */
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* ======================= 宏函数 ======================= */
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* ======================= 全局变量 ======================= */
/* USER CODE BEGIN Variables */

/**
 * FreeRTOS 信号量列表
 *
 * 所有信号量均用于：
 *  - 中断与任务之间的同步
 *  - DMA / USB / CAN 发送资源互斥
 */

// USB 中断二值信号量（用于中断延迟处理）
osSemaphoreId sem_usb_irq;

// UART4 DMA 发送完成信号量
osSemaphoreId sem_uart4_dma;

// UART5 DMA 发送完成信号量
osSemaphoreId sem_uart5_dma;

// USB 接收完成信号量
osSemaphoreId sem_usb_rx;

// USB 发送完成信号量
osSemaphoreId sem_usb_tx;

// CAN1 发送信号量
osSemaphoreId sem_can1_tx;

// CAN2 发送信号量
osSemaphoreId sem_can2_tx;

/* USER CODE END Variables */

/* ======================= 默认任务定义 ======================= */

/**
 * defaultTask：
 *  - CubeMX 自动生成的默认任务
 *  - 在本工程中用于：
 *      1. 初始化 USB
 *      2. 调用 C++ 版本的 Main()
 */
osThreadId_t defaultTaskHandle;

const osThreadAttr_t defaultTask_attributes = {
        .name = "defaultTask",
        .stack_size = 500 * 4,                 // 栈大小（字节）
        .priority = (osPriority_t)osPriorityNormal,
};

/* ======================= 函数声明 ======================= */
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void);   // FreeRTOS 初始化入口

/**
  * @brief  FreeRTOS 初始化函数
  * @retval None
  */
void MX_FREERTOS_Init(void)
{
    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* ======================= RTOS 互斥量 ======================= */
    /* USER CODE BEGIN RTOS_MUTEX */
    // 本工程未使用 mutex（互斥锁）
    /* USER CODE END RTOS_MUTEX */

    /* ======================= RTOS 信号量 ======================= */
    /* USER CODE BEGIN RTOS_SEMAPHORES */

    /**
     * USB 中断信号量
     *  - 用于中断 → 任务的同步
     *  - 初始值为 0（上电后阻塞）
     */
    osSemaphoreDef(sem_usb_irq);
    sem_usb_irq = osSemaphoreNew(1, 0, osSemaphore(sem_usb_irq));

    /**
     * UART DMA 信号量
     *  - 控制 DMA 发送资源
     *  - 初始值为 1，表示可用
     */
    osSemaphoreDef(sem_uart4_dma);
    sem_uart4_dma = osSemaphoreNew(1, 1, osSemaphore(sem_uart4_dma));

    osSemaphoreDef(sem_uart5_dma);
    sem_uart5_dma = osSemaphoreNew(1, 1, osSemaphore(sem_uart5_dma));

    /**
     * USB 接收信号量
     *  - USB 中断中释放
     *  - 接收任务中获取
     */
    osSemaphoreDef(sem_usb_rx);
    sem_usb_rx = osSemaphoreNew(1, 0, osSemaphore(sem_usb_rx));

    /**
     * USB 发送信号量
     *  - 控制 USB CDC 发送互斥
     */
    osSemaphoreDef(sem_usb_tx);
    sem_usb_tx = osSemaphoreNew(1, 1, osSemaphore(sem_usb_tx));

    /**
     * CAN 总线发送信号量
     *  - 保证同一时间只有一个 CAN TX
     */
    osSemaphoreDef(sem_can1_tx);
    sem_can1_tx = osSemaphoreNew(1, 1, osSemaphore(sem_can1_tx));

    osSemaphoreDef(sem_can2_tx);
    sem_can2_tx = osSemaphoreNew(1, 1, osSemaphore(sem_can2_tx));

    /* USER CODE END RTOS_SEMAPHORES */

    /* ======================= RTOS 软件定时器 ======================= */
    /* USER CODE BEGIN RTOS_TIMERS */
    /* USER CODE END RTOS_TIMERS */

    /* ======================= RTOS 队列 / 特殊任务 ======================= */
    /* USER CODE BEGIN RTOS_QUEUES */

    /**
     * USB 延迟中断处理任务（Deferred Interrupt Task）
     *
     * 设计原因：
     *  - USB 中断中不能执行复杂逻辑
     *  - 中断中仅释放信号量
     *  - 由该任务完成实际处理
     *
     * 注意：
     *  - 该任务必须在 MX_USB_DEVICE_Init() 之前创建
     */
    const osThreadAttr_t usbIrqTask_attributes = {
            .name = "usbIrqTask",
            .stack_size = 500,
            .priority = (osPriority_t)osPriorityAboveNormal,
    };
    usbIrqTaskHandle =
            osThreadNew(UsbDeferredInterruptTask, NULL,
                        &usbIrqTask_attributes);

    /* USER CODE END RTOS_QUEUES */

    /* ======================= 创建默认任务 ======================= */

    defaultTaskHandle =
            osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_EVENTS */
    /* USER CODE END RTOS_EVENTS */
}

/* ======================= defaultTask 实现 ======================= */
/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  默认任务函数
  * @param  argument 未使用
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
    /**
     * 初始化 USB 设备（CDC / HID 等）
     * 必须在 RTOS 启动后执行
     */
    MX_USB_DEVICE_Init();

    /* USER CODE BEGIN StartDefaultTask */

    /**
     * 调用 C++ 版本的 Main()
     *
     * 设计说明：
     *  - STM32CubeMX 生成的是 C 入口
     *  - 项目主逻辑使用 C++
     *  - 通过 defaultTask 作为桥接
     */
    Main();

    // 主逻辑结束后删除自身任务
    vTaskDelete(defaultTaskHandle);

    /* USER CODE END StartDefaultTask */
}

/* ======================= 应用层扩展代码 ======================= */
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
