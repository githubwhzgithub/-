/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : 主程序头文件
  * @description    : 包含两轮自平衡车应用程序的通用定义，包括：
  *                   - GPIO引脚定义（电机控制、传感器接口、通信接口）
  *                   - 系统配置宏定义
  *                   - 外部函数声明
  *                   - 硬件抽象层接口
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

/* 防止头文件重复包含 -------------------------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* 系统头文件包含 -----------------------------------------------------------*/
#include "stm32f1xx_hal.h"      // STM32F1xx系列HAL库头文件

/* 用户自定义头文件包含 -----------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* 导出类型定义 -------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* 导出常量定义 -------------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* 导出宏定义 ---------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* 导出函数原型声明 ---------------------------------------------------------*/
void Error_Handler(void);       // 系统错误处理函数

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* GPIO引脚定义 -------------------------------------------------------------*/

/* 摄像头通信接口（预留，用于图像识别扩展功能） */
#define Camera_TX_Pin GPIO_PIN_2        // 摄像头串口发送引脚 (PA2)
#define Camera_TX_GPIO_Port GPIOA
#define Camera_RX_Pin GPIO_PIN_3        // 摄像头串口接收引脚 (PA3)
#define Camera_RX_GPIO_Port GPIOA

/* TB6612电机驱动PWM控制引脚 */
#define PWMA_Pin GPIO_PIN_6             // 电机A的PWM控制引脚 (PA6) - 左轮速度控制
#define PWMA_GPIO_Port GPIOA
#define PWMB_Pin GPIO_PIN_7             // 电机B的PWM控制引脚 (PA7) - 右轮速度控制
#define PWMB_GPIO_Port GPIOA

/* 蓝牙通信接口 */
#define BT_TX_Pin GPIO_PIN_10           // 蓝牙模块串口发送引脚 (PB10)
#define BT_TX_GPIO_Port GPIOB
#define BT_RX_Pin GPIO_PIN_11           // 蓝牙模块串口接收引脚 (PB11)
#define BT_RX_GPIO_Port GPIOB

/* TB6612电机驱动方向控制引脚 */
#define AIN2_Pin GPIO_PIN_12            // 电机A方向控制引脚2 (PB12) - 左轮方向
#define AIN2_GPIO_Port GPIOB
#define AIN1_Pin GPIO_PIN_13            // 电机A方向控制引脚1 (PB13) - 左轮方向
#define AIN1_GPIO_Port GPIOB
#define BIN1_Pin GPIO_PIN_14            // 电机B方向控制引脚1 (PB14) - 右轮方向
#define BIN1_GPIO_Port GPIOB
#define BIN2_Pin GPIO_PIN_15            // 电机B方向控制引脚2 (PB15) - 右轮方向
#define BIN2_GPIO_Port GPIOB

/* 超声波测距传感器接口 */
#define ECHO_Pin GPIO_PIN_10            // 超声波回响信号接收引脚 (PA10) - 测距反馈
#define ECHO_GPIO_Port GPIOA
#define TRIG_Pin GPIO_PIN_12            // 超声波触发信号发送引脚 (PA12) - 测距触发
#define TRIG_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
