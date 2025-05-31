#include "TB6612.h"

// 电机实例定义
Motor_t MotorA = {
    .IN1_Port = MOTOR_A_IN1_PORT,
    .IN1_Pin = MOTOR_A_IN1_PIN,
    .IN2_Port = MOTOR_A_IN2_PORT,
    .IN2_Pin = MOTOR_A_IN2_PIN,
    .PWM_Channel = MOTOR_A_PWM_CHANNEL,
    .current_speed = 0
};

Motor_t MotorB = {
    .IN1_Port = MOTOR_B_IN1_PORT,
    .IN1_Pin = MOTOR_B_IN1_PIN,
    .IN2_Port = MOTOR_B_IN2_PORT,
    .IN2_Pin = MOTOR_B_IN2_PIN,
    .PWM_Channel = MOTOR_B_PWM_CHANNEL,
    .current_speed = 0
};

/**
 * @brief 初始化TB6612电机驱动模块
 */
void TB6612_Init(void)
{
    // 启动PWM定时器
    HAL_TIM_PWM_Start(MOTOR_PWM_TIMER, MOTOR_A_PWM_CHANNEL);
    HAL_TIM_PWM_Start(MOTOR_PWM_TIMER, MOTOR_B_PWM_CHANNEL);

    // 初始化电机为停止状态
    TB6612_StopAllMotors();
}

/**
 * @brief 设置电机方向
 * @param motor: 电机结构体指针
 * @param direction: 电机方向
 */
void TB6612_SetMotorDirection(Motor_t* motor, MotorDirection_t direction)
{
    switch(direction)
    {
        case MOTOR_FORWARD:
            HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_RESET);
            break;

        case MOTOR_BACKWARD:
            HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_SET);
            break;

        case MOTOR_STOP:
            HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_RESET);
            break;

        case MOTOR_BRAKE:
            HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_SET);
            break;
    }
}

/**
 * @brief 设置电机速度
 * @param motor: 电机结构体指针
 * @param speed: 速度值 (-1000 到 1000, 负值为反转)
 */
void TB6612_SetMotorSpeed(Motor_t* motor, int16_t speed)
{
    // 限制速度范围
    if(speed > 1000) speed = 1000;
    if(speed < -1000) speed = -1000;

    motor->current_speed = speed;

    // 根据速度符号设置方向
    if(speed > 0)
    {
        TB6612_SetMotorDirection(motor, MOTOR_FORWARD);
        __HAL_TIM_SET_COMPARE(MOTOR_PWM_TIMER, motor->PWM_Channel, speed);
    }
    else if(speed < 0)
    {
        TB6612_SetMotorDirection(motor, MOTOR_BACKWARD);
        __HAL_TIM_SET_COMPARE(MOTOR_PWM_TIMER, motor->PWM_Channel, -speed);
    }
    else
    {
        TB6612_SetMotorDirection(motor, MOTOR_STOP);
        __HAL_TIM_SET_COMPARE(MOTOR_PWM_TIMER, motor->PWM_Channel, 0);
    }
}

/**
 * @brief 同时设置两个电机的速度
 * @param speedA: 电机A速度 (-1000 到 1000)
 * @param speedB: 电机B速度 (-1000 到 1000)
 */
void TB6612_SetBothMotors(int16_t speedA, int16_t speedB)
{
    TB6612_SetMotorSpeed(&MotorA, speedA);
    TB6612_SetMotorSpeed(&MotorB, speedB);
}

/**
 * @brief 停止所有电机
 */
void TB6612_StopAllMotors(void)
{
    TB6612_SetMotorSpeed(&MotorA, 0);
    TB6612_SetMotorSpeed(&MotorB, 0);
}

/**
 * @brief 刹车所有电机
 */
void TB6612_BrakeAllMotors(void)
{
    TB6612_SetMotorDirection(&MotorA, MOTOR_BRAKE);
    TB6612_SetMotorDirection(&MotorB, MOTOR_BRAKE);
    __HAL_TIM_SET_COMPARE(MOTOR_PWM_TIMER, MOTOR_A_PWM_CHANNEL, 1000);
    __HAL_TIM_SET_COMPARE(MOTOR_PWM_TIMER, MOTOR_B_PWM_CHANNEL, 1000);
}
