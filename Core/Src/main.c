/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "mpu6050.h"
#include "TB6612.h"
#include "motorencoder.h"
#include "ultrasonic.h"
#include "balance_control.h"
#include "bluetooth.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 10
#define TX_BUFFER_SIZE 20
#define BALANCE_CONTROL_INTERVAL 5   // 平衡控制更新间隔(毫秒)
#define ENCODER_UPDATE_INTERVAL 10   // 编码器更新间隔(毫秒)
#define ULTRASONIC_UPDATE_INTERVAL 50 // 超声波更新间隔(毫秒)
#define BLUETOOTH_UPDATE_INTERVAL 20  // 蓝牙更新间隔(毫秒)
#define STATUS_SEND_INTERVAL 1000     // 状态发送间隔(毫秒)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t tx_buffer[TX_BUFFER_SIZE];
volatile uint8_t rx_data_ready = 0;

// 时间变量
uint32_t last_balance_update = 0;
uint32_t last_encoder_update = 0;
uint32_t last_ultrasonic_update = 0;
uint32_t last_bluetooth_update = 0;
uint32_t last_status_send = 0;

// 系统状态
uint8_t system_initialized = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART3)
  {
    rx_data_ready = 1;
    // Stop DMA reception to process data, will be restarted in main loop
    HAL_UART_DMAStop(&huart3);
  }
  else if(huart->Instance == USART2)
  {
    // 蓝牙数据接收处理
    Bluetooth_HandleRxData(Bluetooth.rx_buffer[0]);
    // 重新启动接收
    HAL_UART_Receive_IT(&huart2, &Bluetooth.rx_buffer[0], 1);
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

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

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
  /* USER CODE BEGIN 2 */
  // 初始化UART接收
  HAL_UART_Receive_DMA(&huart3, rx_buffer, 1); // Start DMA reception for 1 byte
  
  uint8_t init_msg[200];
  
  // 延时等待系统稳定
  HAL_Delay(100);
  
  // 初始化MPU6050
  if (MPU6050_Init(&hi2c1) != 0)
  {
    sprintf((char*)init_msg, "MPU6050 initialization failed!\r\n");
    HAL_UART_Transmit(&huart3, init_msg, strlen((char*)init_msg), 1000);
    Error_Handler();
  }
  else
  {
    sprintf((char*)init_msg, "MPU6050 initialization successful!\r\n");
    HAL_UART_Transmit(&huart3, init_msg, strlen((char*)init_msg), 1000);
    Kalman_Init(); // Initialize Kalman filters
  }
  
  // 初始化TB6612电机驱动
  TB6612_Init();
  sprintf((char*)init_msg, "TB6612 Motor Driver initialized!\r\n");
  HAL_UART_Transmit(&huart3, init_msg, strlen((char*)init_msg), 1000);
  
  // 初始化编码器
  MotorEncoder_Init();
  sprintf((char*)init_msg, "Motor Encoders initialized!\r\n");
  HAL_UART_Transmit(&huart3, init_msg, strlen((char*)init_msg), 1000);
  
  // 初始化超声波
  Ultrasonic_Init();
  sprintf((char*)init_msg, "Ultrasonic sensor initialized!\r\n");
  HAL_UART_Transmit(&huart3, init_msg, strlen((char*)init_msg), 1000);
  
  // 初始化平衡控制
  BalanceControl_Init();
  sprintf((char*)init_msg, "Balance Control initialized!\r\n");
  HAL_UART_Transmit(&huart3, init_msg, strlen((char*)init_msg), 1000);
  
  // 初始化蓝牙通信
  Bluetooth_Init();
  sprintf((char*)init_msg, "Bluetooth Communication initialized!\r\n");
  HAL_UART_Transmit(&huart3, init_msg, strlen((char*)init_msg), 1000);
  
  // 系统初始化完成
  system_initialized = 1;
  sprintf((char*)init_msg, "\r\n=== Balance Robot System Ready ===\r\n");
  HAL_UART_Transmit(&huart3, init_msg, strlen((char*)init_msg), 1000);
  
  // 初始化时间变量
  last_balance_update = HAL_GetTick();
  last_encoder_update = HAL_GetTick();
  last_ultrasonic_update = HAL_GetTick();
  last_bluetooth_update = HAL_GetTick();
  last_status_send = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if(!system_initialized) continue;
    
    uint32_t current_time = HAL_GetTick();
    
    // 高频率平衡控制更新 (每5毫秒)
    if (current_time - last_balance_update >= BALANCE_CONTROL_INTERVAL)
    {
      last_balance_update = current_time;
      BalanceControl_Update();
    }
    
    // 编码器数据更新 (每10毫秒)
    if (current_time - last_encoder_update >= ENCODER_UPDATE_INTERVAL)
    {
      last_encoder_update = current_time;
      MotorEncoder_Update();
    }
    
    // 超声波测距更新 (每50毫秒)
    if (current_time - last_ultrasonic_update >= ULTRASONIC_UPDATE_INTERVAL)
    {
      last_ultrasonic_update = current_time;
      Ultrasonic_Update();
    }
    
    // 蓝牙通信更新 (每20毫秒)
    if (current_time - last_bluetooth_update >= BLUETOOTH_UPDATE_INTERVAL)
    {
      last_bluetooth_update = current_time;
      Bluetooth_Update();
    }
    
    // 状态信息发送 (每1秒)
    if (current_time - last_status_send >= STATUS_SEND_INTERVAL)
    {
      last_status_send = current_time;
      
      // 发送系统状态到调试串口
      BalanceState_t* state = BalanceControl_GetState();
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
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
