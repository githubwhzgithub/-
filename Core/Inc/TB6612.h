/**
 * @file    TB6612.h
 * @brief   TB6612FNG双电机驱动模块头文件
 * @details 该文件定义了TB6612FNG电机驱动芯片的控制接口，包括电机方向控制、
 *          PWM速度控制等功能。TB6612FNG是一款双路直流电机驱动芯片，
 *          支持正反转、制动等多种控制模式。
 * @author  平衡车项目组
 * @date    2025
 */

#ifndef __TB6612_H
#define __TB6612_H

#include "main.h"    // 包含STM32主头文件，提供GPIO、系统等基础定义
#include "tim.h"     // 包含定时器头文件，用于PWM信号生成

/*==============================================================================
                            TB6612电机驱动模块硬件引脚定义
==============================================================================*/

/**
 * @brief 电机A控制引脚定义
 * @note  TB6612芯片通过IN1和IN2引脚的高低电平组合来控制电机转向：
 *        IN1=1, IN2=0: 正转
 *        IN1=0, IN2=1: 反转
 *        IN1=0, IN2=0: 停止
 *        IN1=1, IN2=1: 制动
 */
#define MOTOR_A_IN1_PORT    GPIOA        // 电机A方向控制引脚1的GPIO端口
#define MOTOR_A_IN1_PIN     GPIO_PIN_0   // 电机A方向控制引脚1的GPIO引脚号
#define MOTOR_A_IN2_PORT    GPIOA        // 电机A方向控制引脚2的GPIO端口
#define MOTOR_A_IN2_PIN     GPIO_PIN_1   // 电机A方向控制引脚2的GPIO引脚号

/**
 * @brief 电机B控制引脚定义
 * @note  控制逻辑与电机A相同
 */
#define MOTOR_B_IN1_PORT    GPIOA        // 电机B方向控制引脚1的GPIO端口
#define MOTOR_B_IN1_PIN     GPIO_PIN_2   // 电机B方向控制引脚1的GPIO引脚号
#define MOTOR_B_IN2_PORT    GPIOA        // 电机B方向控制引脚2的GPIO端口
#define MOTOR_B_IN2_PIN     GPIO_PIN_3   // 电机B方向控制引脚2的GPIO引脚号

/**
 * @brief PWM定时器和通道定义
 * @note  TB6612芯片的PWMA和PWMB引脚用于控制电机转速，
 *        PWM占空比越大，电机转速越快
 */
#define MOTOR_PWM_TIMER     &htim3           // 电机PWM控制使用的定时器句柄
#define MOTOR_A_PWM_CHANNEL TIM_CHANNEL_1    // 电机A的PWM输出通道(PWMA)
#define MOTOR_B_PWM_CHANNEL TIM_CHANNEL_2    // 电机B的PWM输出通道(PWMB)

/*==============================================================================
                                数据类型定义
==============================================================================*/

/**
 * @brief 电机方向枚举类型
 * @note  定义了电机的四种工作状态，对应TB6612芯片的控制逻辑
 */
typedef enum {
    MOTOR_FORWARD = 0,    // 电机正转：IN1=1, IN2=0
    MOTOR_BACKWARD,       // 电机反转：IN1=0, IN2=1
    MOTOR_STOP,          // 电机停止：IN1=0, IN2=0（自由滑行）
    MOTOR_BRAKE          // 电机制动：IN1=1, IN2=1（短路制动）
} MotorDirection_t;

/**
 * @brief 电机控制结构体
 * @note  封装了单个电机的所有控制参数，包括GPIO引脚配置和当前状态
 */
typedef struct {
    GPIO_TypeDef* IN1_Port;    // IN1引脚的GPIO端口指针
    uint16_t IN1_Pin;          // IN1引脚的GPIO引脚号
    GPIO_TypeDef* IN2_Port;    // IN2引脚的GPIO端口指针
    uint16_t IN2_Pin;          // IN2引脚的GPIO引脚号
    uint32_t PWM_Channel;      // PWM输出通道号
    int16_t current_speed;     // 当前电机速度值 (范围: -1000 到 1000)
                              // 正值表示正转，负值表示反转，绝对值表示速度大小
} Motor_t;

/*==============================================================================
                                函数声明
==============================================================================*/

/**
 * @brief 初始化TB6612电机驱动模块
 * @note  启动PWM定时器，初始化电机为停止状态
 */
void TB6612_Init(void);

/**
 * @brief 设置单个电机的速度
 * @param motor 电机结构体指针
 * @param speed 速度值 (-1000 到 1000)
 *              正值：正转，负值：反转，0：停止
 * @note  函数内部会自动根据速度值的正负设置电机方向
 */
void TB6612_SetMotorSpeed(Motor_t* motor, int16_t speed);

/**
 * @brief 设置单个电机的转向
 * @param motor 电机结构体指针
 * @param direction 电机方向（正转/反转/停止/制动）
 * @note  该函数只控制方向，不控制速度
 */
void TB6612_SetMotorDirection(Motor_t* motor, MotorDirection_t direction);

/**
 * @brief 同时设置两个电机的速度
 * @param speedA 电机A的速度值 (-1000 到 1000)
 * @param speedB 电机B的速度值 (-1000 到 1000)
 * @note  这是最常用的函数，用于平衡车的运动控制
 */
void TB6612_SetBothMotors(int16_t speedA, int16_t speedB);

/**
 * @brief 停止所有电机
 * @note  将两个电机都设置为停止状态（自由滑行）
 */
void TB6612_StopAllMotors(void);

/**
 * @brief 制动所有电机
 * @note  将两个电机都设置为制动状态（短路制动，快速停止）
 */
void TB6612_BrakeAllMotors(void);

/*==============================================================================
                                全局变量声明
==============================================================================*/

/**
 * @brief 电机实例声明
 * @note  这两个变量在TB6612.c中定义，包含了电机A和电机B的所有配置信息
 */
extern Motor_t MotorA;    // 电机A实例（通常为左轮电机）
extern Motor_t MotorB;    // 电机B实例（通常为右轮电机）

#endif /* __TB6612_H */
