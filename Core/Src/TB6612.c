/**
 * @file    TB6612.c
 * @brief   TB6612FNG双电机驱动模块实现文件
 * @details 该文件实现了TB6612FNG双电机驱动芯片的控制功能。
 *          TB6612FNG是一款高性能的双路直流电机驱动芯片，
 *          支持PWM调速和正反转控制。本模块为平衡车提供
 *          精确的电机控制功能，包括速度控制、方向控制、
 *          制动等功能。
 *
 * @author  STM32平衡车项目组
 * @version 1.0
 * @date    2024
 *
 * @note    使用PWM控制电机速度，GPIO控制电机方向
 */

#include "TB6612.h"

/*==============================================================================
                                全局变量定义
==============================================================================*/

/**
 * @brief 电机A实例定义
 * @details 左侧电机的控制参数，包括方向控制引脚、PWM通道和当前速度
 */
Motor_t MotorA = {
    .IN1_Port = MOTOR_A_IN1_PORT,       // 电机A方向控制引脚1端口
    .IN1_Pin = MOTOR_A_IN1_PIN,         // 电机A方向控制引脚1
    .IN2_Port = MOTOR_A_IN2_PORT,       // 电机A方向控制引脚2端口
    .IN2_Pin = MOTOR_A_IN2_PIN,         // 电机A方向控制引脚2
    .PWM_Channel = MOTOR_A_PWM_CHANNEL, // 电机A的PWM通道
    .current_speed = 0                  // 电机A当前速度，初始为0
};

/**
 * @brief 电机B实例定义
 * @details 右侧电机的控制参数，包括方向控制引脚、PWM通道和当前速度
 */
Motor_t MotorB = {
    .IN1_Port = MOTOR_B_IN1_PORT,       // 电机B方向控制引脚1端口
    .IN1_Pin = MOTOR_B_IN1_PIN,         // 电机B方向控制引脚1
    .IN2_Port = MOTOR_B_IN2_PORT,       // 电机B方向控制引脚2端口
    .IN2_Pin = MOTOR_B_IN2_PIN,         // 电机B方向控制引脚2
    .PWM_Channel = MOTOR_B_PWM_CHANNEL, // 电机B的PWM通道
    .current_speed = 0                  // 电机B当前速度，初始为0
};

/*==============================================================================
                                函数实现
==============================================================================*/

/**
 * @brief  TB6612电机驱动模块初始化
 * @details 初始化TB6612双电机驱动模块，设置电机的初始状态。
 *          该函数会将两个电机设置为停止状态，并将PWM输出设置为0。
 *          注意：GPIO引脚的初始化应该在HAL_Init()或main.c中完成。
 *
 * @param  无
 * @retval 无
 *
 * @note   调用此函数前，确保相关的GPIO和PWM定时器已经初始化
 * @note   初始化后电机处于停止状态，速度为0
 */
void TB6612_Init(void)
{
    // 启动PWM定时器
    HAL_TIM_PWM_Start(MOTOR_PWM_TIMER, MOTOR_A_PWM_CHANNEL);
    HAL_TIM_PWM_Start(MOTOR_PWM_TIMER, MOTOR_B_PWM_CHANNEL);

    // 设置电机A和电机B的初始状态为停止
    // 通过设置IN1和IN2引脚的状态来控制电机方向
    TB6612_SetMotorDirection(&MotorA, MOTOR_STOP);  // 电机A停止
    TB6612_SetMotorDirection(&MotorB, MOTOR_STOP);  // 电机B停止

    // 设置初始PWM占空比为0，确保电机完全停止
    // PWM值范围通常为0-1000或0-100，具体取决于定时器配置
    TB6612_SetMotorSpeed(&MotorA, 0);  // 电机A速度设为0
    TB6612_SetMotorSpeed(&MotorB, 0);  // 电机B速度设为0
}

/**
 * @brief  设置电机旋转方向
 * @details 通过控制TB6612的IN1和IN2引脚来设置电机的旋转方向。
 *          TB6612的方向控制逻辑：
 *          - 正转：IN1=1, IN2=0
 *          - 反转：IN1=0, IN2=1
 *          - 停止：IN1=0, IN2=0
 *          - 制动：IN1=1, IN2=1
 *
 * @param  motor: 指向电机结构体的指针，包含控制引脚信息
 * @param  direction: 电机方向枚举值
 *         - MOTOR_FORWARD: 电机正转
 *         - MOTOR_BACKWARD: 电机反转
 *         - MOTOR_STOP: 电机停止（自由滑行）
 *         - MOTOR_BRAKE: 电机制动（快速停止）
 * @retval 无
 *
 * @note   制动模式会产生较大的制动力，适用于需要快速停止的场合
 * @note   停止模式电机会自由滑行，制动模式电机会快速停止
 */
void TB6612_SetMotorDirection(Motor_t* motor, MotorDirection_t direction)
{
    switch(direction)
    {
        case MOTOR_FORWARD:  // 电机正转：IN1=高电平，IN2=低电平
            HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_SET);    // IN1置高
            HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_RESET);  // IN2置低
            break;

        case MOTOR_BACKWARD: // 电机反转：IN1=低电平，IN2=高电平
            HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_RESET);  // IN1置低
            HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_SET);    // IN2置高
            break;

        case MOTOR_STOP:     // 电机停止：IN1=低电平，IN2=低电平（自由滑行）
            HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_RESET);  // IN1置低
            HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_RESET);  // IN2置低
            break;

        case MOTOR_BRAKE:    // 电机制动：IN1=高电平，IN2=高电平（快速停止）
            HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_SET);    // IN1置高
            HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_SET);    // IN2置高
            break;
    }
}

/**
 * @brief  设置电机转速
 * @details 通过PWM信号控制电机的转速，同时根据速度值的正负自动设置电机方向。
 *          该函数集成了方向控制和速度控制，简化了电机控制操作。
 *          PWM占空比与电机转速成正比，占空比越大转速越快。
 *
 * @param  motor: 指向电机结构体的指针，包含PWM通道等控制信息
 * @param  speed: 电机速度值，范围为-1000到1000
 *         - 正值：电机正转，数值越大转速越快
 *         - 负值：电机反转，绝对值越大转速越快
 *         - 零值：电机停止
 *         - 1000：最大正转速度
 *         - -1000：最大反转速度
 * @retval 无
 *
 * @note   速度值会被自动限制在[-1000, 1000]范围内
 * @note   PWM定时器的ARR值决定了速度分辨率，通常设置为1000
 * @note   该函数会自动更新电机结构体中的current_speed字段
 */
void TB6612_SetMotorSpeed(Motor_t* motor, int16_t speed)
{
    // 限制速度范围在[-1000, 1000]之间，防止溢出
    if(speed > 1000) speed = 1000;   // 限制最大正转速度
    if(speed < -1000) speed = -1000; // 限制最大反转速度

    // 更新电机当前速度记录
    motor->current_speed = speed;

    // 根据速度值的正负号设置电机方向和PWM占空比
    if(speed > 0)  // 正值：电机正转
    {
        TB6612_SetMotorDirection(motor, MOTOR_FORWARD);  // 设置正转方向
        // 设置PWM占空比，speed值直接对应占空比
        __HAL_TIM_SET_COMPARE(MOTOR_PWM_TIMER, motor->PWM_Channel, speed);
    }
    else if(speed < 0)  // 负值：电机反转
    {
        TB6612_SetMotorDirection(motor, MOTOR_BACKWARD); // 设置反转方向
        // 设置PWM占空比，使用speed的绝对值
        __HAL_TIM_SET_COMPARE(MOTOR_PWM_TIMER, motor->PWM_Channel, -speed);
    }
    else  // 零值：电机停止
    {
        TB6612_SetMotorDirection(motor, MOTOR_STOP);     // 设置停止状态
        // PWM占空比设为0，完全停止
        __HAL_TIM_SET_COMPARE(MOTOR_PWM_TIMER, motor->PWM_Channel, 0);
    }
}

/**
 * @brief  同时设置两个电机的转速
 * @details 该函数提供了同时控制左右两个电机的便捷接口，
 *          常用于平衡车的运动控制，如前进、后退、转弯等。
 *          通过分别设置两个电机的速度，可以实现差速转向。
 *
 * @param  speedA: 电机A（左电机）的速度值，范围为-1000到1000
 *         - 正值：电机A正转
 *         - 负值：电机A反转
 *         - 零值：电机A停止
 * @param  speedB: 电机B（右电机）的速度值，范围为-1000到1000
 *         - 正值：电机B正转
 *         - 负值：电机B反转
 *         - 零值：电机B停止
 * @retval 无
 *
 * @note   该函数内部调用TB6612_SetMotorSpeed()来分别控制两个电机
 * @note   适用于需要同时控制两个电机的场合，如平衡车运动控制
 *
 * @example
 *         TB6612_SetBothMotors(500, 500);   // 两电机同速前进
 *         TB6612_SetBothMotors(500, -500);  // 原地右转
 *         TB6612_SetBothMotors(-500, 500);  // 原地左转
 */
void TB6612_SetBothMotors(int16_t speedA, int16_t speedB)
{
    TB6612_SetMotorSpeed(&MotorA, speedA);  // 设置电机A速度
    TB6612_SetMotorSpeed(&MotorB, speedB);  // 设置电机B速度
}

/**
 * @brief  停止所有电机
 * @details 将两个电机的速度都设置为0，使电机进入停止状态。
 *          电机会自由滑行直到完全停止，不会产生制动力。
 *          这是一种"软停止"方式，适用于正常的停车操作。
 *
 * @param  无
 * @retval 无
 *
 * @note   电机停止后会自由滑行，如需快速停止请使用TB6612_BrakeAllMotors()
 * @note   该函数等效于调用TB6612_SetBothMotors(0, 0)
 */
void TB6612_StopAllMotors(void)
{
    TB6612_SetMotorSpeed(&MotorA, 0);  // 电机A停止
    TB6612_SetMotorSpeed(&MotorB, 0);  // 电机B停止
}

/**
 * @brief  制动所有电机
 * @details 将两个电机都设置为制动状态，通过短路电机绕组产生制动力，
 *          使电机快速停止。这是一种"硬停止"方式，制动效果明显。
 *          制动时会给PWM通道设置最大占空比以增强制动效果。
 *
 * @param  无
 * @retval 无
 *
 * @note   制动模式会产生较大的制动力，适用于紧急停车
 * @note   长时间制动可能导致电机发热，建议制动后及时切换到停止状态
 * @note   制动时PWM占空比设为1000（最大值）以获得最佳制动效果
 */
void TB6612_BrakeAllMotors(void)
{
    // 设置两个电机为制动状态
    TB6612_SetMotorDirection(&MotorA, MOTOR_BRAKE);  // 电机A制动
    TB6612_SetMotorDirection(&MotorB, MOTOR_BRAKE);  // 电机B制动

    // 设置最大PWM占空比以增强制动效果
    __HAL_TIM_SET_COMPARE(MOTOR_PWM_TIMER, MOTOR_A_PWM_CHANNEL, 1000);  // 电机A最大制动力
    __HAL_TIM_SET_COMPARE(MOTOR_PWM_TIMER, MOTOR_B_PWM_CHANNEL, 1000);  // 电机B最大制动力
}
