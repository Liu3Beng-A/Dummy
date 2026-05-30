///* USER CODE BEGIN Header */
///**
//  ******************************************************************************
//  * @file           : main.c
//  * @brief          : Main program body
//  ******************************************************************************
//  * @attention
//  *
//  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
//  * All rights reserved.</center></h2>
//  *
//  * This software component is licensed by ST under BSD 3-Clause license,
//  * the "License"; You may not use this file except in compliance with the
//  * License. You may obtain a copy of the License at:
//  *                        opensource.org/licenses/BSD-3-Clause
//  *
//  ******************************************************************************
//  */
///* USER CODE END Header */
///* Includes ------------------------------------------------------------------*/
//#include "main.h"
//#include "cmsis_os.h"
//#include "adc.h"
//#include "can.h"
//#include "dma.h"
//#include "i2c.h"
//#include "spi.h"
//#include "tim.h"
//#include "usart.h"
//#include "usb_device.h"
//#include "gpio.h"
//
///* Private includes ----------------------------------------------------------*/
///* USER CODE BEGIN Includes */
//
///* USER CODE END Includes */
//
///* Private typedef -----------------------------------------------------------*/
///* USER CODE BEGIN PTD */
//
///* USER CODE END PTD */
//
///* Private define ------------------------------------------------------------*/
///* USER CODE BEGIN PD */
///* USER CODE END PD */
//
///* Private macro -------------------------------------------------------------*/
///* USER CODE BEGIN PM */
//
///* USER CODE END PM */
//
///* Private variables ---------------------------------------------------------*/
//
///* USER CODE BEGIN PV */
//uint64_t serialNumber;
//uint64_t myserial = 8;
//char serialNumberStr[13];
//__attribute__((section(".ccmram"))) uint8_t ucHeap[configTOTAL_HEAP_SIZE];
///* USER CODE END PV */
//
///* Private function prototypes -----------------------------------------------*/
//void SystemClock_Config(void);
//void MX_FREERTOS_Init(void);
///* USER CODE BEGIN PFP */
//
///* USER CODE END PFP */
//
///* Private user code ---------------------------------------------------------*/
///* USER CODE BEGIN 0 */
//
//
///* USER CODE END 0 */
//
///**
//  * @brief  The application entry point.
//  * @retval int
//  */
//int main(void)
//{
//  /* USER CODE BEGIN 1 */
//
//    // This procedure of building a USB serial number should be identical
//    // to the way the STM's built-in USB bootloader does it. This means
//    // that the device will have the same serial number in normal and DFU mode.
//    uint32_t uuid0 = *(uint32_t *) (UID_BASE + 0);
//    uint32_t uuid1 = *(uint32_t *) (UID_BASE + 4);
//    uint32_t uuid2 = *(uint32_t *) (UID_BASE + 8);
//    uint32_t uuid_mixed_part = uuid0 + uuid2;
//    serialNumber = ((uint64_t) uuid_mixed_part << 16) | (uint64_t) (uuid1 >> 16);
//
//    uint64_t val = serialNumber;
//    for (size_t i = 0; i < 12; ++i)
//    {
//        serialNumberStr[i] = "0123456789ABCDEF"[(val >> (48 - 4)) & 0xf];
//        val <<= 4;
//    }
//    serialNumberStr[12] = 0;
//
//  /* USER CODE END 1 */
//
//  /* MCU Configuration--------------------------------------------------------*/
//
//  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//  HAL_Init();
//
//  /* USER CODE BEGIN Init */
//    HAL_RCC_DeInit();
//
//  /* USER CODE END Init */
//
//  /* Configure the system clock */
//  SystemClock_Config();
//
//  /* USER CODE BEGIN SysInit */
//
//  /* USER CODE END SysInit */
//
//  /* Initialize all configured peripherals */
//  MX_GPIO_Init();
//  MX_DMA_Init();
//  MX_I2C1_Init();
//  MX_I2C2_Init();
//  MX_CAN1_Init();
//  MX_CAN2_Init();
//  MX_USART1_UART_Init();
//  MX_I2C3_Init();
//  MX_SPI1_Init();
//  MX_SPI3_Init();
//  MX_UART4_Init();
//  MX_ADC1_Init();
//  MX_UART5_Init();
//  MX_TIM2_Init();
//  MX_TIM3_Init();
//  /* USER CODE BEGIN 2 */
//
//  /* USER CODE END 2 */
//
//  /* Init scheduler */
//  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
//  MX_FREERTOS_Init();
//  /* Start scheduler */
//  osKernelStart();
//
//  /* We should never get here as control is now taken by the scheduler */
//  /* Infinite loop */
//  /* USER CODE BEGIN WHILE */
//
//    while (1)
//    {
//    /* USER CODE END WHILE */
//
//    /* USER CODE BEGIN 3 */
//    }
//  /* USER CODE END 3 */
//}
//
///**
//  * @brief System Clock Configuration
//  * @retval None
//  */
//void SystemClock_Config(void)
//{
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//  /** Configure the main internal regulator output voltage
//  */
//  __HAL_RCC_PWR_CLK_ENABLE();
//  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
//  /** Initializes the RCC Oscillators according to the specified parameters
//  * in the RCC_OscInitTypeDef structure.
//  */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//  RCC_OscInitStruct.PLL.PLLM = 4;
//  RCC_OscInitStruct.PLL.PLLN = 168;
//  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
//  RCC_OscInitStruct.PLL.PLLQ = 7;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Initializes the CPU, AHB and APB buses clocks
//  */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
//
//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}
//
///* USER CODE BEGIN 4 */
//void OnTimerCallback(TIM_TypeDef *timInstance);
///* USER CODE END 4 */
//
// /**
//  * @brief  Period elapsed callback in non blocking mode
//  * @note   This function is called  when TIM6 interrupt took place, inside
//  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
//  * a global variable "uwTick" used as application time base.
//  * @param  htim : TIM handle
//  * @retval None
//  */
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//  /* USER CODE BEGIN Callback 0 */
//
//  /* USER CODE END Callback 0 */
//  if (htim->Instance == TIM6) {
//    HAL_IncTick();
//  }
//  /* USER CODE BEGIN Callback 1 */
//  else
//  {
//      OnTimerCallback(htim->Instance);
//  }
//  /* USER CODE END Callback 1 */
//}
//
///**
//  * @brief  This function is executed in case of error occurrence.
//  * @retval None
//  */
//void Error_Handler(void)
//{
//  /* USER CODE BEGIN Error_Handler_Debug */
//    /* User can add his own implementation to report the HAL error return state */
//    __disable_irq();
//    while (1)
//    {
//    }
//  /* USER CODE END Error_Handler_Debug */
//}
//
//#ifdef  USE_FULL_ASSERT
///**
//  * @brief  Reports the name of the source file and the source line number
//  *         where the assert_param error has occurred.
//  * @param  file: pointer to the source file name
//  * @param  line: assert_param error line source number
//  * @retval None
//  */
//void assert_failed(uint8_t *file, uint32_t line)
//{
//  /* USER CODE BEGIN 6 */
//  /* User can add his own implementation to report the file name and line number,
//     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//  /* USER CODE END 6 */
//}
//#endif /* USE_FULL_ASSERT */
//
///************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/





/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 主程序入口文件
  ******************************************************************************
  * @attention
  *
  * 本文件基于 STM32CubeMX 自动生成，并在 USER CODE 区域中
  * 添加了用户自定义代码。
  *
  * <h2><center>&copy; STMicroelectronics 版权所有</center></h2>
  *
  * 本软件遵循 BSD 3-Clause License 许可协议。
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* ======================= 头文件包含 ======================= */
#include "main.h"
#include "cmsis_os.h"     // CMSIS-RTOS (FreeRTOS) 接口
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* ======================= 用户自定义头文件 ======================= */
/* USER CODE BEGIN Includes */

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
/* USER CODE BEGIN PV */

// STM32 唯一 ID 生成的 USB 序列号（64 位）
uint64_t serialNumber;

// 自定义设备序列号（当前未使用，保留接口）
uint64_t myserial = 8;

// USB 序列号字符串（12 字符 + 结束符）
char serialNumberStr[13];

// FreeRTOS 堆空间，放置在 CCMRAM（高速核心耦合存储器）中
// 目的：提高 RTOS 动态内存分配效率
__attribute__((section(".ccmram"))) uint8_t ucHeap[configTOTAL_HEAP_SIZE];

/* USER CODE END PV */

/* ======================= 函数声明 ======================= */
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* ======================= 用户代码区 ======================= */
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  程序入口函数
  * @retval int（理论上不会返回）
  */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /**
     * 以下代码用于生成 USB 设备序列号
     *
     * 目的：
     *  - 保证 USB 正常模式 和 DFU（下载）模式下
     *    设备序列号一致
     *  - 与 STM32 内置 USB BootLoader 行为保持一致
     */

    // 读取 STM32 芯片的唯一 ID（96 位）
    uint32_t uuid0 = *(uint32_t *)(UID_BASE + 0);
    uint32_t uuid1 = *(uint32_t *)(UID_BASE + 4);
    uint32_t uuid2 = *(uint32_t *)(UID_BASE + 8);

    // 按照 ST 官方算法混合 UID
    uint32_t uuid_mixed_part = uuid0 + uuid2;

    // 生成 64 位序列号
    serialNumber = ((uint64_t)uuid_mixed_part << 16)
                   | (uint64_t)(uuid1 >> 16);

    // 转换为 12 位十六进制字符串
    uint64_t val = serialNumber;
    for (size_t i = 0; i < 12; ++i)
    {
        serialNumberStr[i] =
                "0123456789ABCDEF"[(val >> (48 - 4)) & 0xF];
        val <<= 4;
    }
    serialNumberStr[12] = 0; // 字符串结束符

    /* USER CODE END 1 */

    /* ======================= MCU 初始化 ======================= */

    /**
     * 复位所有外设
     * 初始化 Flash 接口
     * 初始化 SysTick
     */
    HAL_Init();

    /* USER CODE BEGIN Init */

    // 复位 RCC 配置（在部分复杂工程中用于重新初始化时钟）
    HAL_RCC_DeInit();

    /* USER CODE END Init */

    /* ======================= 系统时钟配置 ======================= */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* ======================= 外设初始化 ======================= */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_I2C1_Init();
    MX_I2C2_Init();
    MX_CAN1_Init();
    MX_CAN2_Init();
    MX_USART1_UART_Init();
    MX_I2C3_Init();
    MX_SPI1_Init();
    MX_SPI3_Init();
    MX_UART4_Init();
    MX_ADC1_Init();
    MX_UART5_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();

    /* USER CODE BEGIN 2 */

    /* USER CODE END 2 */

    /* ======================= FreeRTOS 初始化 ======================= */

    // 初始化 RTOS 内核
    osKernelInitialize();

    // 创建线程、消息队列、定时器等（freertos.c）
    MX_FREERTOS_Init();

    // 启动 RTOS 调度器
    osKernelStart();

    /**
     * 理论上程序不会执行到这里
     * 因为控制权已交由 FreeRTOS
     */
    while (1)
    {
    }
}

/**
  * @brief 系统时钟配置函数
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /**
     * 启用电源控制时钟
     * 设置电压调节器等级（支持 168MHz）
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**
     * 使用 HSE 外部晶振
     * PLL 配置：
     * HSE = 8MHz
     * PLLM = 4
     * PLLN = 168
     * SYSCLK = 168MHz
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 4;
    RCC_OscInitStruct.PLL.PLLN       = 168;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = 7;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /**
     * 时钟分频配置：
     * AHB  = 168MHz
     * APB1 = 42MHz
     * APB2 = 84MHz
     */
    RCC_ClkInitStruct.ClockType =
            RCC_CLOCKTYPE_HCLK  |
            RCC_CLOCKTYPE_SYSCLK |
            RCC_CLOCKTYPE_PCLK1 |
            RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct,
                            FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

// 用户自定义定时器回调函数声明
void OnTimerCallback(TIM_TypeDef *timInstance);

/* USER CODE END 4 */

/**
  * @brief 定时器周期中断回调函数
  *
  * TIM6：
  *   - 系统时基（HAL_IncTick）
  *
  * 其他定时器：
  *   - 转交给用户自定义回调函数处理
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        // 系统节拍计数
        HAL_IncTick();
    }
    else
    {
        // 用户定时器回调（如控制环、任务调度等）
        OnTimerCallback(htim->Instance);
    }
}

/**
  * @brief 错误处理函数
  *
  * 当系统发生严重错误时进入死循环
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
/**
  * @brief assert 错误回调
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  // 可在此输出错误文件名和行号
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
