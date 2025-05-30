/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 主程序文件 - STM32平衡车控制系统
  *                   实现平衡车的主控制逻辑，包括传感器数据采集、
  *                   平衡控制算法、电机驱动和蓝牙通信等功能
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
/* STM32 HAL库头文件 */
#include "main.h"        // 主头文件，包含系统配置和函数声明
#include "dma.h"         // DMA配置和控制函数
#include "i2c.h"         // I2C通信接口（用于MPU6050传感器）
#include "tim.h"         // 定时器配置（用于PWM和编码器）
#include "usart.h"       // 串口通信配置（蓝牙和调试）
#include "gpio.h"        // GPIO引脚配置

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* 标准C库 */
#include <string.h>       // 字符串处理函数
#include <stdio.h>        // 标准输入输出函数（sprintf等）
#include <stdlib.h>       // 标准库函数

/* 自定义模块头文件 */
#include "mpu6050.h"      // MPU6050六轴传感器驱动
#include "TB6612.h"       // TB6612电机驱动模块
#include "motorencoder.h" // 电机编码器处理模块
#include "ultrasonic.h"   // 超声波测距模块
#include "balance_control.h" // 平衡控制算法模块
#include "bluetooth.h"    // 蓝牙通信模块
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 缓冲区大小定义 */
#define RX_BUFFER_SIZE 10             // 串口接收缓冲区大小
#define TX_BUFFER_SIZE 20             // 串口发送缓冲区大小

/* 系统任务更新间隔定义（毫秒） */
#define BALANCE_CONTROL_INTERVAL 5    // 平衡控制更新间隔(毫秒) - 高频率保证控制精度
#define ENCODER_UPDATE_INTERVAL 10    // 编码器更新间隔(毫秒) - 获取电机速度反馈
#define ULTRASONIC_UPDATE_INTERVAL 50 // 超声波更新间隔(毫秒) - 障碍物检测
#define BLUETOOTH_UPDATE_INTERVAL 20  // 蓝牙更新间隔(毫秒) - 处理遥控指令
#define STATUS_SEND_INTERVAL 1000     // 状态发送间隔(毫秒) - 调试信息输出
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* 串口通信缓冲区 */
uint8_t rx_buffer[RX_BUFFER_SIZE];    // 串口接收数据缓冲区
uint8_t tx_buffer[TX_BUFFER_SIZE];    // 串口发送数据缓冲区
volatile uint8_t rx_data_ready = 0;   // 接收数据就绪标志

/* 任务调度时间变量 - 记录各模块上次更新时间 */
uint32_t last_balance_update = 0;     // 平衡控制上次更新时间
uint32_t last_encoder_update = 0;     // 编码器上次更新时间
uint32_t last_ultrasonic_update = 0;  // 超声波上次更新时间
uint32_t last_bluetooth_update = 0;   // 蓝牙通信上次更新时间
uint32_t last_status_send = 0;        // 状态信息上次发送时间

/* 系统状态标志 */
uint8_t system_initialized = 0;       // 系统初始化完成标志
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);        // 系统时钟配置函数
/* USER CODE BEGIN PFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart); // 串口接收完成回调函数
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief 串口接收完成回调函数
 * @param huart: 串口句柄指针
 * @note 当串口接收到数据时，HAL库会自动调用此函数
 *       用于处理不同串口的数据接收
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART3)
  {
    // 蓝牙数据接收处理 (HC-05使用UART3: PB10-TX, PB11-RX)
    Bluetooth_HandleRxData(Bluetooth.rx_buffer[0]);
    // 重新启动接收 - 保持持续接收状态
    HAL_UART_Receive_IT(&huart3, &Bluetooth.rx_buffer[0], 1);
  }
  else if(huart->Instance == USART2)
  {
    // USART2预留给摄像头使用 (PA2-TX, PA3-RX)
    // 暂时不处理，以后会用到
  }
}
/* USER CODE END 0 */

/**
  * @brief  应用程序入口点 - 平衡车主控制程序
  * @retval int 程序返回值（正常情况下不会返回）
  * @note   实现平衡车的完整控制流程：
  *         1. 系统初始化（时钟、外设、传感器等）
  *         2. 进入主循环，执行多任务调度
  *         3. 实时处理平衡控制、传感器数据和通信
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* 用户自定义初始化代码区域1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  /* 微控制器配置 */

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();                    // 初始化HAL库，复位所有外设，初始化Flash接口和SysTick

  /* USER CODE BEGIN Init */
  /* 用户自定义初始化代码区域2 */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();          // 配置系统时钟（72MHz）

  /* USER CODE BEGIN SysInit */
  /* 用户自定义系统初始化代码区域 */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* 初始化所有配置的外设 */
  MX_GPIO_Init();                // 初始化GPIO引脚（电机控制、传感器触发等）
  MX_DMA_Init();                 // 初始化DMA（用于串口高效数据传输）
  MX_USART3_UART_Init();         // 初始化UART3（蓝牙通信：PB10-TX, PB11-RX）
  MX_I2C1_Init();                // 初始化I2C1（MPU6050传感器通信）
  MX_TIM1_Init();                // 初始化定时器1（电机PWM控制）
  MX_TIM2_Init();                // 初始化定时器2
  MX_TIM3_Init();                // 初始化定时器3
  MX_USART2_UART_Init();         // 初始化UART2（预留摄像头通信：PA2-TX, PA3-RX）
  /* USER CODE BEGIN 2 */
  /* 用户自定义初始化代码区域3 - 传感器和模块初始化 */

  // 初始化UART接收 - 启动DMA接收模式
  HAL_UART_Receive_DMA(&huart3, rx_buffer, 1); // 启动DMA接收1字节数据（蓝牙通信）

  uint8_t init_msg[200];         // 初始化信息缓冲区，用于串口输出调试信息

  // 延时等待系统稳定 - 确保电源和时钟稳定
  HAL_Delay(100);

  // 初始化MPU6050六轴传感器（加速度计+陀螺仪）
  if (MPU6050_Init(&hi2c1) != 0)
  {
    sprintf((char*)init_msg, "MPU6050 initialization failed!\r\n");
    HAL_UART_Transmit(&huart3, init_msg, strlen((char*)init_msg), 1000);
    Error_Handler();             // 初始化失败，进入错误处理
  }
  else
  {
    sprintf((char*)init_msg, "MPU6050 initialization successful!\r\n");
    HAL_UART_Transmit(&huart3, init_msg, strlen((char*)init_msg), 1000);
    Kalman_Init();               // 初始化卡尔曼滤波器，用于姿态角度融合
  }

  // 初始化TB6612电机驱动模块
  TB6612_Init();               // 配置电机驱动引脚和PWM输出
  sprintf((char*)init_msg, "TB6612 Motor Driver initialized!\r\n");
  HAL_UART_Transmit(&huart3, init_msg, strlen((char*)init_msg), 1000);

  // 初始化电机编码器
  MotorEncoder_Init();         // 配置编码器输入捕获，用于测量电机转速
  sprintf((char*)init_msg, "Motor Encoders initialized!\r\n");
  HAL_UART_Transmit(&huart3, init_msg, strlen((char*)init_msg), 1000);

  // 初始化超声波测距传感器
  Ultrasonic_Init();           // 配置超声波触发和回响引脚，用于障碍物检测
  sprintf((char*)init_msg, "Ultrasonic sensor initialized!\r\n");
  HAL_UART_Transmit(&huart3, init_msg, strlen((char*)init_msg), 1000);

  // 初始化平衡控制系统
  BalanceControl_Init();       // 初始化PID控制器和平衡算法参数
  sprintf((char*)init_msg, "Balance Control initialized!\r\n");
  HAL_UART_Transmit(&huart3, init_msg, strlen((char*)init_msg), 1000);

  // 初始化蓝牙通信模块
  Bluetooth_Init();            // 配置蓝牙通信协议和命令解析
  sprintf((char*)init_msg, "Bluetooth Communication initialized!\r\n");
  HAL_UART_Transmit(&huart3, init_msg, strlen((char*)init_msg), 1000);

  // 系统初始化完成标志
  system_initialized = 1;      // 设置系统初始化完成标志，允许主循环开始工作
  sprintf((char*)init_msg, "\r\n=== Balance Robot System Ready ===\r\n");
  HAL_UART_Transmit(&huart3, init_msg, strlen((char*)init_msg), 1000);

  // 初始化任务调度时间变量 - 记录当前时间作为各模块的起始时间
  last_balance_update = HAL_GetTick();    // 平衡控制任务时间基准
  last_encoder_update = HAL_GetTick();    // 编码器更新任务时间基准
  last_ultrasonic_update = HAL_GetTick(); // 超声波测距任务时间基准
  last_bluetooth_update = HAL_GetTick();  // 蓝牙通信任务时间基准
  last_status_send = HAL_GetTick();       // 状态发送任务时间基准
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* 主程序无限循环 - 系统核心运行逻辑 */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 用户自定义主循环代码区域 - 任务调度和控制逻辑 */

    // 只有在系统初始化完成后才执行主循环逻辑
    if(!system_initialized) continue;

    uint32_t current_time = HAL_GetTick();  // 获取当前系统时间戳（毫秒）

    /* 任务调度系统 - 基于时间片的多任务调度 */

    // 平衡控制更新任务 (每5毫秒) - 最高优先级
    if (current_time - last_balance_update >= BALANCE_CONTROL_INTERVAL)
    {
      last_balance_update = current_time;
      BalanceControl_Update();           // 执行平衡控制算法（PID计算、电机输出）
    }

    // 编码器更新任务 (每10毫秒) - 速度反馈
    if (current_time - last_encoder_update >= ENCODER_UPDATE_INTERVAL)
    {
      last_encoder_update = current_time;
      MotorEncoder_Update();             // 更新电机编码器数据，计算实际转速
    }

    // 超声波更新任务 (每50毫秒) - 障碍物检测
    if (current_time - last_ultrasonic_update >= ULTRASONIC_UPDATE_INTERVAL)
    {
      last_ultrasonic_update = current_time;
      Ultrasonic_Update();               // 执行超声波测距，更新障碍物距离信息
    }

    // 蓝牙通信更新任务 (每20毫秒) - 指令处理
    if (current_time - last_bluetooth_update >= BLUETOOTH_UPDATE_INTERVAL)
    {
      last_bluetooth_update = current_time;
      Bluetooth_Update();                // 处理蓝牙接收的控制指令
    }

    // 状态信息发送任务 (每1秒) - 调试和监控
    if (current_time - last_status_send >= STATUS_SEND_INTERVAL)
    {
      last_status_send = current_time;

      // 发送系统状态信息到串口（用于调试和监控）
      BalanceState_t* state = BalanceControl_GetState();  // 获取当前平衡车状态
      uint8_t status_msg[200];
      sprintf((char*)status_msg,
              "[DEBUG] Pitch:%.2f Roll:%.2f Speed:%.2f Dist:%.1fcm Enable:%d\r\n",
              state->pitch, state->roll,
              (state->left_speed + state->right_speed) / 2.0f,
              state->distance_front,
              state->balance_enabled);
      HAL_UART_Transmit(&huart3, status_msg, strlen((char*)status_msg), 100);
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief 系统时钟配置函数
  * @retval None
  * @note 配置系统主时钟为72MHz，使用外部HSE晶振和PLL倍频
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};  // 振荡器初始化结构体
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};  // 时钟初始化结构体

  /** 根据RCC_OscInitTypeDef结构体中指定的参数初始化RCC振荡器 */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;       // 使用外部高速振荡器(HSE)
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                        // 开启HSE
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;          // HSE预分频值为1
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                        // 保持HSI开启作为备用
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                    // 开启PLL锁相环
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;            // PLL时钟源选择HSE
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;                    // PLL倍频系数为9 (8MHz*9 = 72MHz)
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();                                               // 振荡器配置失败，进入错误处理
  }

  /** 初始化CPU、AHB和APB总线时钟 */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;  // 配置所有时钟类型
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;       // 系统时钟源选择PLL输出
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;              // AHB时钟不分频 (72MHz)
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;               // APB1时钟2分频 (36MHz)
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;               // APB2时钟不分频 (72MHz)

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();                                               // 时钟配置失败，进入错误处理
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  系统错误处理函数
  * @retval None
  * @note   当系统发生严重错误时调用此函数，禁用中断并进入死循环
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* 用户可以在此添加自己的错误处理实现来报告HAL错误状态 */

  __disable_irq();                    // 禁用所有中断，防止系统进一步损坏

  // 可选：在此添加错误指示（如LED闪烁、串口输出错误信息等）
  // 例如：点亮错误指示LED，发送错误信息到串口

  while (1)                           // 进入死循环，等待系统复位
  {
    // 系统挂起，等待看门狗复位或手动复位
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  断言失败报告函数
  * @param  file: 指向发生断言错误的源文件名的指针
  * @param  line: 发生断言错误的源代码行号
  * @retval None
  * @note   当USE_FULL_ASSERT宏被定义时，此函数用于报告断言失败的位置
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* 用户可以在此添加自己的实现来报告文件名和行号，
     例如: printf("参数错误: 文件 %s 第 %d 行\r\n", file, line) */

  // 可选：通过串口输出断言失败信息，便于调试
  // HAL_UART_Transmit(&huart3, (uint8_t*)"Assert Failed!\r\n", 16, 1000);

  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
