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
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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
#include "k230_vision.h"  // K230视觉模块
#include "motor_test.h"   // 电机测试模块
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
#define STATUS_SEND_INTERVAL 3000     // 状态发送间隔(毫秒) - 调试信息输出
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
uint32_t last_vision_update = 0;      // K230视觉模块上次更新时间

/* 系统状态标志 */
uint8_t system_initialized = 0;       // 系统初始化完成标志
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  static uint8_t bt_rx_byte;  // 蓝牙接收字节缓冲
  static uint8_t vision_rx_byte;  // 视觉模块接收字节缓冲
  
  if(huart->Instance == USART3)
  {
    // 蓝牙数据接收处理 (HC-05使用UART3: PB10-TX, PB11-RX)
    Bluetooth_HandleRxData(bt_rx_byte);
    // 重新启动接收 - 保持持续接收状态
    HAL_UART_Receive_IT(&huart3, &bt_rx_byte, 1);
  }
  else if(huart->Instance == USART2)
  {
    // K230视觉模块数据接收处理 (PA2-TX, PA3-RX)
    K230_Vision_ReceiveData(vision_rx_byte);
    // 重新启动接收 - 保持持续接收状态
    HAL_UART_Receive_IT(&huart2, &vision_rx_byte, 1);
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* 用户自定义初始化代码区域1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* 用户自定义初始化代码区域2 */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* 用户自定义系统初始化代码区域 */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  /* 用户自定义初始化代码区域3 - 传感器和模块初始化 */


  uint8_t init_msg[200];         // 初始化信息缓冲区，用于串口输出调试信息

  // 延时等待系统稳定 - 确保电源和时钟稳定
  HAL_Delay(100);

  // 初始化蓝牙通信模块
  Bluetooth_Init();            // 配置蓝牙通信协议和命令解析
  sprintf((char*)init_msg, "Bluetooth Communication initialized!\r\n");
  Bluetooth_SendMessage((char*)init_msg);
  HAL_Delay(100);
  
  // 初始化MPU6050六轴传感器（加速度计+陀螺仪）
  if (MPU6050_Init(&hi2c1) != 0)
  {
    sprintf((char*)init_msg, "MPU6050 initialization failed!\r\n");
    Bluetooth_SendMessage((char*)init_msg);
    HAL_Delay(100);
  }
  else
  {
    sprintf((char*)init_msg, "MPU6050 initialization successful!\r\n");
    Bluetooth_SendMessage((char*)init_msg);
    HAL_Delay(100);
    Kalman_Init();               // 初始化卡尔曼滤波器，用于姿态角度融合
  }

  // 初始化TB6612电机驱动模块
  TB6612_Init();               // 配置电机驱动引脚和PWM输出
  sprintf((char*)init_msg, "TB6612 Motor Driver initialized!\r\n");
  Bluetooth_SendMessage((char*)init_msg);
  HAL_Delay(100);

  // 初始化电机编码器
  MotorEncoder_Init();         // 配置编码器输入捕获，用于测量电机转速
  sprintf((char*)init_msg, "Motor Encoders initialized!\r\n");
  Bluetooth_SendMessage((char*)init_msg);
  HAL_Delay(100);

  // 初始化超声波测距传感器
  Ultrasonic_Init();           // 配置超声波触发和回响引脚，用于障碍物检测
  sprintf((char*)init_msg, "Ultrasonic sensor initialized!\r\n");
  Bluetooth_SendMessage((char*)init_msg);
  HAL_Delay(100);

  // 初始化平衡控制系统
  BalanceControl_Init();       // 初始化PID控制器和平衡算法参数
  sprintf((char*)init_msg, "Balance Control initialized!\r\n");
  Bluetooth_SendMessage((char*)init_msg);
  HAL_Delay(100);

  // 初始化K230视觉模块
  K230_Vision_Init(&huart2);   // 配置K230视觉模块通信
  sprintf((char*)init_msg, "K230 Vision Module initialized!\r\n");
  Bluetooth_SendMessage((char*)init_msg);
  HAL_Delay(100);

  /*
  // 初始化电机测试模块
  MotorTest_Init();            // 初始化电机测试功能
  sprintf((char*)init_msg, "Motor Test Module initialized!\r\n");
  Bluetooth_SendMessage((char*)init_msg);
  while(1){
    MotorTest_BothForward(500);
    for(int i=0;i<5;i++){
      MotorEncoder_Update();
      BalanceState.left_speed = MotorEncoder_GetSpeedMPS(&EncoderA);
      BalanceState.right_speed = MotorEncoder_GetSpeedMPS(&EncoderB);
      sprintf((char*)init_msg, "Forward_Speed: %f, %f\r\n", BalanceState.left_speed, BalanceState.right_speed);
      Bluetooth_SendStatus();
      HAL_Delay(1000);
    }
    MotorTest_BothBackward(500);
    for(int i=0;i<5;i++){
      MotorEncoder_Update();
      BalanceState.left_speed = MotorEncoder_GetSpeedMPS(&EncoderA);
      BalanceState.right_speed = MotorEncoder_GetSpeedMPS(&EncoderB);
      sprintf((char*)init_msg, "Backward_Speed: %f, %f\r\n", BalanceState.left_speed, BalanceState.right_speed);
      Bluetooth_SendStatus();
      HAL_Delay(1000);
    }
  }
  HAL_Delay(100);*/

  // 系统初始化完成标志
  system_initialized = 1;      // 设置系统初始化完成标志，允许主循环开始工作
  sprintf((char*)init_msg, "\r\n=== Balance Robot System Ready ===\r\n");
  Bluetooth_SendMessage((char*)init_msg);
  HAL_Delay(100);

  // 初始化任务调度时间变量 - 记录当前时间作为各模块的起始时间
  last_balance_update = HAL_GetTick();    // 平衡控制任务时间基准
  last_encoder_update = HAL_GetTick();    // 编码器更新任务时间基准
  last_ultrasonic_update = HAL_GetTick(); // 超声波测距任务时间基准
  last_bluetooth_update = HAL_GetTick();  // 蓝牙通信任务时间基准
  last_status_send = HAL_GetTick();       // 状态发送任务时间基准
  last_vision_update = HAL_GetTick();     // K230视觉模块任务时间基准
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

    // K230视觉数据更新任务 (每20毫秒) - 视觉处理
    if (BalanceState.vision_mode != 0)
    { 
      if(current_time - last_vision_update >= 20){
        last_vision_update = current_time;
        K230_Vision_Update();
      }
                    // 更新K230视觉模块数据
    }

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
      Ultrasonic_Update();              // 执行超声波测距，更新障碍物距离信息
    }

    // 蓝牙通信更新任务 (每20毫秒) - 指令处理
    if (current_time - last_bluetooth_update >= BLUETOOTH_UPDATE_INTERVAL)
    {
      last_bluetooth_update = current_time;
      Bluetooth_Update();                // 处理蓝牙接收的控制指令
    }

    // 状态信息发送任务 (每10秒) - 调试和监控
    if (current_time - last_status_send >= STATUS_SEND_INTERVAL)
    {
      last_status_send = current_time;

      // 发送系统状态信息到串口（用于调试和监控）
      Bluetooth_SendStatus();
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
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
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
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
