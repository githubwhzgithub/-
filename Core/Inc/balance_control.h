/**
  ******************************************************************************
  * @file           : balance_control.h
  * @brief          : 平衡车控制系统头文件
  * @description    : 定义两轮自平衡车控制系统的数据结构、常量和函数接口，包括：
  *                   - PID控制器结构体定义
  *                   - 平衡车状态结构体定义
  *                   - 控制参数宏定义
  *                   - 控制函数接口声明
  ******************************************************************************
  */

#ifndef __BALANCE_CONTROL_H
#define __BALANCE_CONTROL_H

/* 系统头文件包含 */
#include "main.h"           // 主程序头文件
#include "mpu6050.h"        // MPU6050六轴传感器驱动
#include "TB6612.h"         // TB6612电机驱动模块
#include "motorencoder.h"   // 电机编码器模块
#include "ultrasonic.h"     // 超声波测距模块

/* 数据结构定义 */

/**
 * @brief PID控制器结构体
 * @note  实现标准的PID控制算法，包含比例、积分、微分三个控制环节
 */
typedef struct {
    float Kp;           // 比例系数 - 控制响应速度，值越大响应越快
    float Ki;           // 积分系数 - 消除稳态误差，值越大积分作用越强
    float Kd;           // 微分系数 - 预测误差变化，提高系统稳定性
    float setpoint;     // 目标值 - PID控制器的期望输出值
    float integral;     // 积分累积 - 历史误差的累积值
    float last_error;   // 上次误差 - 用于计算微分项
    float output;       // 输出值 - PID控制器的最终输出
    float max_output;   // 最大输出限制 - 防止输出过大
    float min_output;   // 最小输出限制 - 防止输出过小
    float max_integral; // 积分限幅 - 防止积分饱和
} PID_Controller_t;

/**
 * @brief 平衡车状态结构体
 * @note  存储平衡车的所有状态信息，包括传感器数据、控制目标和系统标志
 */
typedef struct {
    float pitch;                // 俯仰角 - 前后倾斜角度（度），正值表示前倾
    float roll;                 // 横滚角 - 左右倾斜角度（度），正值表示右倾
    float pitch_rate;           // 俯仰角速度 - 前后倾斜的角速度（度/秒）
    float roll_rate;            // 横滚角速度 - 左右倾斜的角速度（度/秒）
    float left_speed;           // 左轮速度 - 左电机的实际转速（编码器计数/秒）
    float right_speed;          // 右轮速度 - 右电机的实际转速（编码器计数/秒）
    float target_speed;         // 目标速度 - 期望的前进后退速度
    float target_angle;         // 目标角度 - 期望的平衡角度（通常为0度）
    float distance_front;       // 前方距离 - 超声波测得的前方障碍物距离（厘米）
    uint8_t balance_enabled;    // 平衡使能标志 - 1:启用平衡控制, 0:禁用
    uint8_t obstacle_detected;  // 障碍物检测标志 - 1:检测到障碍物, 0:无障碍物
} BalanceState_t;

/* 平衡控制系统参数定义 */

/**
 * @brief 平衡控制基本参数
 */
#define BALANCE_TARGET_ANGLE    0.0f    // 目标平衡角度（度）- 平衡车的理想直立角度
#define MAX_TILT_ANGLE         45.0f    // 最大倾斜角度（度）- 超过此角度触发保护
#define MIN_OBSTACLE_DISTANCE  20.0f    // 最小障碍物距离（厘米）- 触发避障的距离阈值
#define CONTROL_FREQUENCY      100      // 控制频率（Hz）- 控制算法的执行频率
#define CONTROL_PERIOD_MS      10       // 控制周期（毫秒）- 每次控制更新的时间间隔

/**
 * @brief PID控制器参数定义
 * @note  三个PID控制器分别控制角度、速度和转向，参数需要根据实际硬件调优
 */

/* 角度环PID参数 - 控制平衡车的俯仰角度，维持直立状态 */
#define ANGLE_PID_KP           80.0f    // 角度比例系数 - 主要控制力，影响平衡响应速度
#define ANGLE_PID_KI           0.5f     // 角度积分系数 - 消除静态误差，防止长期偏移
#define ANGLE_PID_KD           2.0f     // 角度微分系数 - 阻尼作用，减少震荡
#define ANGLE_PID_MAX_OUTPUT   1000.0f  // 角度PID最大输出 - 限制电机最大驱动力
#define ANGLE_PID_MAX_INTEGRAL 500.0f   // 角度PID积分限幅 - 防止积分饱和

/* 速度环PID参数 - 控制平衡车的前进后退速度 */
#define SPEED_PID_KP           5.0f     // 速度比例系数 - 速度跟踪响应强度
#define SPEED_PID_KI           0.1f     // 速度积分系数 - 消除速度稳态误差
#define SPEED_PID_KD           0.0f     // 速度微分系数 - 通常设为0，避免噪声放大
#define SPEED_PID_MAX_OUTPUT   10.0f    // 速度PID最大输出 - 限制角度偏移量
#define SPEED_PID_MAX_INTEGRAL 50.0f    // 速度PID积分限幅 - 防止积分饱和

/* 转向PID参数 - 控制平衡车的左右转向 */
#define TURN_PID_KP            20.0f    // 转向比例系数 - 转向响应强度
#define TURN_PID_KI            0.0f     // 转向积分系数 - 通常设为0，避免转向漂移
#define TURN_PID_KD            1.0f     // 转向微分系数 - 提高转向稳定性
#define TURN_PID_MAX_OUTPUT    500.0f   // 转向PID最大输出 - 限制转向力矩
#define TURN_PID_MAX_INTEGRAL  100.0f   // 转向PID积分限幅 - 防止积分饱和

/* 函数接口声明 */

/**
 * @brief 平衡控制系统初始化
 */
void BalanceControl_Init(void);

/**
 * @brief 平衡控制主更新函数（需要周期性调用）
 */
void BalanceControl_Update(void);

/**
 * @brief 启用/禁用平衡控制
 * @param enable 1:启用, 0:禁用
 */
void BalanceControl_Enable(uint8_t enable);

/**
 * @brief 设置目标速度
 * @param speed 目标速度值
 */
void BalanceControl_SetTargetSpeed(float speed);

/**
 * @brief 设置目标角度
 * @param angle 目标角度值（度）
 */
void BalanceControl_SetTargetAngle(float angle);

/**
 * @brief PID控制器更新计算
 * @param pid PID控制器指针
 * @param current_value 当前测量值
 * @param dt 时间间隔（秒）
 * @return PID输出值
 */
float BalanceControl_PID_Update(PID_Controller_t* pid, float current_value, float dt);

/**
 * @brief 重置PID控制器状态
 * @param pid PID控制器指针
 */
void BalanceControl_PID_Reset(PID_Controller_t* pid);

/**
 * @brief 紧急停止功能
 */
void BalanceControl_EmergencyStop(void);

/**
 * @brief 障碍物避障处理
 */
void BalanceControl_ObstacleAvoidance(void);

/**
 * @brief 获取平衡车状态
 * @return 平衡车状态结构体指针
 */
BalanceState_t* BalanceControl_GetState(void);

/* 全局变量外部声明 */

/**
 * @brief PID控制器实例
 */
extern PID_Controller_t AnglePID;    // 角度PID控制器实例
extern PID_Controller_t SpeedPID;    // 速度PID控制器实例
extern PID_Controller_t TurnPID;     // 转向PID控制器实例

/**
 * @brief 平衡车状态实例
 */
extern BalanceState_t BalanceState;  // 全局平衡车状态变量

#endif /* __BALANCE_CONTROL_H */
