#ifndef __TB6612_H
#define __TB6612_H

#include "main.h"
#include "tim.h"
#include "gpio.h"

// TB6612电机驱动模块定义
// 电机A控制引脚
#define MOTOR_A_IN1_PORT    GPIOB
#define MOTOR_A_IN1_PIN     GPIO_PIN_13
#define MOTOR_A_IN2_PORT    GPIOB
#define MOTOR_A_IN2_PIN     GPIO_PIN_12

// 电机B控制引脚
#define MOTOR_B_IN1_PORT    GPIOB
#define MOTOR_B_IN1_PIN     GPIO_PIN_14
#define MOTOR_B_IN2_PORT    GPIOB
#define MOTOR_B_IN2_PIN     GPIO_PIN_15

// PWM定时器和通道定义
#define MOTOR_PWM_TIMER     &htim3
#define MOTOR_A_PWM_CHANNEL TIM_CHANNEL_1  // PWMA
#define MOTOR_B_PWM_CHANNEL TIM_CHANNEL_2  // PWMB

// 电机最大速度
#define MOTOR_MAX_SPEED     950

// 电机方向枚举
typedef enum {
    MOTOR_FORWARD = 0,
    MOTOR_BACKWARD,
    MOTOR_STOP,
    MOTOR_BRAKE
} MotorDirection_t;

// 电机结构体
typedef struct {
    GPIO_TypeDef* IN1_Port;
    uint16_t IN1_Pin;
    GPIO_TypeDef* IN2_Port;
    uint16_t IN2_Pin;
    uint32_t PWM_Channel;
    int16_t current_speed;  // 当前速度 (-1000 到 1000)
} Motor_t;

// 函数声明
void TB6612_Init(void);
void TB6612_SetMotorSpeed(Motor_t* motor, int16_t speed);
void TB6612_SetMotorDirection(Motor_t* motor, MotorDirection_t direction);
void TB6612_SetBothMotors(int16_t speedA, int16_t speedB);
void TB6612_StopAllMotors(void);
void TB6612_BrakeAllMotors(void);

// 电机实例声明
extern Motor_t MotorA;
extern Motor_t MotorB;

#endif /* __TB6612_H */
