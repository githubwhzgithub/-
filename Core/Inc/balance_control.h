/**
 * @file    balance_control.h
 * @brief   平衡车控制系统头文件
 * @details 该文件定义了平衡车的核心控制算法，包括PID控制器、状态管理、
 *          传感器数据融合等功能。平衡车采用双环PID控制策略：
 *          内环为角度环，外环为速度环，同时包含转向控制。
 * @author  平衡车项目组
 * @date    2025
 */

#ifndef __BALANCE_CONTROL_H
#define __BALANCE_CONTROL_H

#include "main.h"         // STM32主头文件，提供基础数据类型定义
#include "mpu6050.h"      // MPU6050陀螺仪加速度计驱动，用于姿态检测
#include "TB6612.h"       // TB6612电机驱动，用于电机控制
#include "motorencoder.h" // 电机编码器驱动，用于速度反馈
#include "ultrasonic.h"   // 超声波传感器驱动，用于障碍物检测

/*==============================================================================
                                数据结构定义
==============================================================================*/

/**
 * @brief PID控制器结构体
 * @details PID控制器是平衡车控制的核心算法，通过比例(P)、积分(I)、微分(D)
 *          三个环节的组合来实现精确的闭环控制。
 *          - 比例环节：响应当前误差，提供主要的控制力
 *          - 积分环节：消除稳态误差，提高控制精度
 *          - 微分环节：预测误差变化趋势，提高系统稳定性
 */
typedef struct {
    float Kp;           // 比例系数：决定系统对当前误差的响应强度
    float Ki;           // 积分系数：决定系统对历史误差累积的响应强度
    float Kd;           // 微分系数：决定系统对误差变化率的响应强度
    float setpoint;     // 目标值：控制器期望达到的目标状态
    float integral;     // 积分累积值：历史误差的累积，用于消除稳态误差
    float last_error;   // 上次误差值：用于计算微分项
    float output;       // 控制器输出值：经过PID计算后的控制量
    float max_output;   // 最大输出限制：防止控制量过大导致系统不稳定
    float min_output;   // 最小输出限制：防止控制量过小导致响应迟缓
    float max_integral; // 积分限幅：防止积分饱和现象
} PID_Controller_t;

/**
 * @brief 平衡车状态结构体
 * @details 该结构体包含了平衡车运行过程中的所有关键状态信息，
 *          包括姿态角度、运动速度、目标参数、传感器数据等。
 */
typedef struct {
    float pitch;                // 俯仰角(度)：车体前后倾斜角度，正值表示前倾
    float roll;                 // 横滚角(度)：车体左右倾斜角度，正值表示右倾
    float pitch_rate;           // 俯仰角速度(度/秒)：俯仰角的变化率
    float roll_rate;            // 横滚角速度(度/秒)：横滚角的变化率
    float left_speed;           // 左轮速度(m/s)：左侧车轮的线速度
    float right_speed;          // 右轮速度(m/s)：右侧车轮的线速度
    float target_speed;         // 目标速度(m/s)：期望的前进/后退速度
    float target_angle;         // 目标角度(度)：期望的平衡角度
    float distance_front;       // 前方距离(cm)：超声波测得的前方障碍物距离
    uint8_t balance_enabled;    // 平衡使能标志：1-启用平衡控制，0-禁用
    uint8_t obstacle_detected;  // 障碍物检测标志：1-检测到障碍物，0-前方无障碍
} BalanceState_t;

/*==============================================================================
                                控制参数定义
==============================================================================*/

/**
 * @brief 平衡控制基础参数
 * @note  这些参数定义了平衡车的基本控制特性和安全限制
 */
#define BALANCE_TARGET_ANGLE    0.0f    // 目标平衡角度(度)：理想的直立状态角度
#define MAX_TILT_ANGLE         45.0f    // 最大倾斜角度(度)：超过此角度将触发紧急停止
#define MIN_OBSTACLE_DISTANCE  20.0f    // 最小障碍物距离(cm)：小于此距离将停止前进
#define CONTROL_FREQUENCY      100      // 控制频率(Hz)：控制算法的执行频率
#define CONTROL_PERIOD_MS      10       // 控制周期(ms)：两次控制更新之间的时间间隔

/**
 * @brief 角度环PID参数定义
 * @details 角度环是平衡车控制的内环，负责维持车体的直立状态。
 *          该环路的响应速度要快，以确保车体不会倾倒。
 */
#define ANGLE_PID_KP           80.0f    // 角度环比例系数：决定对倾斜角度的响应强度
#define ANGLE_PID_KI           0.5f     // 角度环积分系数：消除角度稳态误差
#define ANGLE_PID_KD           2.0f     // 角度环微分系数：抑制角度振荡
#define ANGLE_PID_MAX_OUTPUT   1000.0f  // 角度环最大输出：限制电机最大驱动力
#define ANGLE_PID_MAX_INTEGRAL 500.0f   // 角度环积分限幅：防止积分饱和

/**
 * @brief 速度环PID参数定义
 * @details 速度环是平衡车控制的外环，负责控制车体的前进后退速度。
 *          该环路的响应可以相对较慢，主要用于位置控制。
 */
#define SPEED_PID_KP           5.0f     // 速度环比例系数：决定对速度误差的响应强度
#define SPEED_PID_KI           0.1f     // 速度环积分系数：消除速度稳态误差
#define SPEED_PID_KD           0.0f     // 速度环微分系数：通常设为0避免噪声放大
#define SPEED_PID_MAX_OUTPUT   10.0f    // 速度环最大输出：限制角度偏移量
#define SPEED_PID_MAX_INTEGRAL 50.0f    // 速度环积分限幅：防止积分饱和

/**
 * @brief 转向PID参数定义
 * @details 转向环负责控制平衡车的左右转向，通过控制两轮差速实现转向。
 */
#define TURN_PID_KP            20.0f    // 转向环比例系数：决定对转向误差的响应强度
#define TURN_PID_KI            0.0f     // 转向环积分系数：通常设为0避免转向漂移
#define TURN_PID_KD            1.0f     // 转向环微分系数：提高转向稳定性
#define TURN_PID_MAX_OUTPUT    500.0f   // 转向环最大输出：限制转向力矩
#define TURN_PID_MAX_INTEGRAL  100.0f   // 转向环积分限幅：防止积分饱和

/*==============================================================================
                                函数声明
==============================================================================*/

/**
 * @brief 初始化平衡控制系统
 * @note  初始化所有传感器、电机驱动和PID控制器
 */
void BalanceControl_Init(void);

/**
 * @brief 平衡控制主更新函数
 * @note  该函数应在主循环中定期调用，执行完整的控制算法
 */
void BalanceControl_Update(void);

/**
 * @brief 使能/禁用平衡控制
 * @param enable 1-使能平衡控制，0-禁用平衡控制
 * @note  禁用时会停止所有电机并重置PID控制器
 */
void BalanceControl_Enable(uint8_t enable);

/**
 * @brief 设置目标速度
 * @param speed 目标速度值(m/s)，正值前进，负值后退
 * @note  该函数用于外部控制平衡车的前进后退
 */
void BalanceControl_SetTargetSpeed(float speed);

/**
 * @brief 设置目标角度
 * @param angle 目标角度值(度)
 * @note  通常用于微调平衡车的直立角度
 */
void BalanceControl_SetTargetAngle(float angle);

/**
 * @brief PID控制器更新函数
 * @param pid PID控制器指针
 * @param current_value 当前测量值
 * @param dt 时间间隔(秒)
 * @return PID控制器输出值
 * @note  通用的PID计算函数，被各个控制环路调用
 */
float BalanceControl_PID_Update(PID_Controller_t* pid, float current_value, float dt);

/**
 * @brief 重置PID控制器
 * @param pid PID控制器指针
 * @note  清零积分项和历史误差，用于控制器重新启动
 */
void BalanceControl_PID_Reset(PID_Controller_t* pid);

/**
 * @brief 紧急停止函数
 * @note  立即制动所有电机并禁用平衡控制，用于安全保护
 */
void BalanceControl_EmergencyStop(void);

/**
 * @brief 障碍物避让函数
 * @note  根据超声波传感器数据判断是否需要停止前进
 */
void BalanceControl_ObstacleAvoidance(void);

/**
 * @brief 获取平衡车状态
 * @return 平衡车状态结构体指针
 * @note  用于外部模块获取平衡车的实时状态信息
 */
BalanceState_t* BalanceControl_GetState(void);

/*==============================================================================
                                全局变量声明
==============================================================================*/

/**
 * @brief PID控制器实例声明
 * @note  这些控制器在balance_control.c中定义和初始化
 */
extern PID_Controller_t AnglePID;    // 角度环PID控制器：维持车体直立
extern PID_Controller_t SpeedPID;    // 速度环PID控制器：控制前进后退速度
extern PID_Controller_t TurnPID;     // 转向环PID控制器：控制左右转向

/**
 * @brief 平衡车状态实例声明
 * @note  包含平衡车运行过程中的所有状态信息
 */
extern BalanceState_t BalanceState;  // 平衡车全局状态变量

#endif /* __BALANCE_CONTROL_H */
