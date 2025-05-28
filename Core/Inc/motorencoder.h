/**
 * @file    motorencoder.h
 * @brief   电机编码器模块头文件
 * @details 该文件定义了电机编码器的接口函数、数据结构和配置参数。
 *          编码器模块用于测量电机的转速和位置，为平衡车的速度控制
 *          和位置反馈提供精确的测量数据。支持增量式编码器，
 *          通过定时器的编码器模式读取编码器脉冲，计算电机转速、
 *          行驶距离等运动参数。
 *
 * @author  STM32平衡车项目组
 * @version 1.0
 * @date    2024
 *
 * @note    使用STM32的定时器编码器模式，支持正反转检测和计数
 */

#ifndef __MOTORENCODER_H
#define __MOTORENCODER_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/*==============================================================================
                                定时器配置定义
==============================================================================*/

/**
 * @brief 编码器定时器分配
 * @note  为左右两个电机分别分配不同的定时器，避免冲突
 */
#define ENCODER_LEFT_TIMER      TIM2        // 左电机编码器使用TIM2
#define ENCODER_RIGHT_TIMER     TIM3        // 右电机编码器使用TIM3
#define ENCODER_TIMER_PERIOD    65535       // 定时器计数周期：16位定时器最大值

/*==============================================================================
                                编码器硬件参数
==============================================================================*/

/**
 * @brief 编码器和机械系统参数
 * @details 这些参数定义了编码器的分辨率和机械传动系统的特性，
 *          用于将编码器脉冲转换为实际的速度和距离值。
 */
#define ENCODER_PPR             1000    // 编码器每转脉冲数：编码器一圈产生的脉冲数
#define WHEEL_DIAMETER          0.065f  // 轮子直径(m)：平衡车车轮的实际直径
#define WHEEL_CIRCUMFERENCE     (3.14159f * WHEEL_DIAMETER)  // 轮子周长(m)：计算行驶距离用
#define GEAR_RATIO              30      // 减速比：电机到车轮的齿轮减速比
#define ENCODER_RESOLUTION      (ENCODER_PPR * GEAR_RATIO)   // 编码器总分辨率：车轮转一圈的脉冲数

/*==============================================================================
                                数据类型定义
==============================================================================*/

/**
 * @brief 电机编码器数据结构
 * @details 包含单个电机编码器的所有状态信息，包括计数值、速度、
 *          距离等运动参数。该结构体用于管理每个电机的编码器数据。
 */
typedef struct {
    int32_t count;          // 编码器当前计数值：累计脉冲数，可正可负
    int32_t last_count;     // 上次计数值：用于计算速度的历史数据
    float speed;            // 当前速度(m/s)：根据脉冲变化计算的线速度
    float distance;         // 累计距离(m)：从启动开始的总行驶距离
    uint32_t last_time;     // 上次更新时间(ms)：用于计算时间间隔
} MotorEncoder_t;

/*==============================================================================
                                函数声明
==============================================================================*/

/**
 * @brief 初始化电机编码器模块
 * @note  配置定时器编码器模式，初始化编码器数据结构
 */
void MotorEncoder_Init(void);

/**
 * @brief 更新编码器数据
 * @note  读取定时器计数值，计算速度和距离，应定期调用
 */
void MotorEncoder_Update(void);

/**
 * @brief 获取指定电机的编码器计数值
 * @param motor_id 电机ID：0-左电机，1-右电机
 * @return 编码器累计计数值
 * @note  返回值可能为负数，表示反向旋转
 */
int32_t MotorEncoder_GetCount(uint8_t motor_id);

/**
 * @brief 获取指定电机的当前速度
 * @param motor_id 电机ID：0-左电机，1-右电机
 * @return 当前线速度(m/s)
 * @note  正值表示前进方向，负值表示后退方向
 */
float MotorEncoder_GetSpeed(uint8_t motor_id);

/**
 * @brief 获取指定电机的累计行驶距离
 * @param motor_id 电机ID：0-左电机，1-右电机
 * @return 累计行驶距离(m)
 * @note  距离为绝对值，不区分方向
 */
float MotorEncoder_GetDistance(uint8_t motor_id);

/**
 * @brief 重置指定电机的编码器数据
 * @param motor_id 电机ID：0-左电机，1-右电机
 * @note  清零计数值、速度和距离，用于重新开始测量
 */
void MotorEncoder_Reset(uint8_t motor_id);

/**
 * @brief 重置所有电机的编码器数据
 * @note  同时重置左右两个电机的编码器数据
 */
void MotorEncoder_ResetAll(void);

/*==============================================================================
                                全局变量声明
==============================================================================*/

/**
 * @brief 左电机编码器数据实例
 * @note  存储左电机的编码器计数、速度、距离等信息
 */
extern MotorEncoder_t LeftEncoder;

/**
 * @brief 右电机编码器数据实例
 * @note  存储右电机的编码器计数、速度、距离等信息
 */
extern MotorEncoder_t RightEncoder;

#endif /* __MOTORENCODER_H */
