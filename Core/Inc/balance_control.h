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
#include "k230_vision.h"    // K230视觉模块

/* 数据结构定义 */

/**
 * @brief PID控制器结构体
 * @note  实现标准的PID控制算法，包含比例、积分、微分三个控制环节
 */
typedef struct {
    float Kp;                    // 比例系数 - 控制响应速度，值越大响应越快
    float Ki;                    // 积分系数 - 消除稳态误差，值越大积分作用越强
    float Kd;                    // 微分系数 - 预测误差变化，提高系统稳定性
    float setpoint;              // 目标值 - PID控制器的期望输出值
    float integral;              // 积分累积 - 历史误差的累积值
    float last_error;            // 上次误差 - 用于计算微分项
    float output;                // 输出值 - PID控制器的最终输出
    float max_output;            // 最大输出限制 - 防止输出过大
    float min_output;            // 最小输出限制 - 防止输出过小
    float max_integral;          // 积分限幅 - 防止积分饱和
    float filtered_derivative;   // 微分项滤波器状态 - 用于低通滤波减少噪声
} PID_Controller_t;

/**
 * @brief 平衡车状态结构体
 * @note  存储平衡车的所有状态信息，包括传感器数据、控制目标和系统标志
 */
typedef struct {
    float pitch;                // 俯仰角 - 前后倾斜角度（度），正值表示前倾（辅助监控）
    float roll;                 // 横滚角 - 左右倾斜角度（度），正值表示右倾（主平衡控制轴）
    float pitch_rate;           // 俯仰角速度 - 前后倾斜的角速度（度/秒）
    float roll_rate;            // 横滚角速度 - 左右倾斜的角速度（度/秒）
    float yaw_rate;             // 偏航角速度 - Z轴角速度（度/秒），用于转向控制
    float left_speed;           // 左轮速度 - 左电机的实际转速（编码器计数/秒）
    float right_speed;          // 右轮速度 - 右电机的实际转速（编码器计数/秒）
    float target_speed;         // 目标速度 - 期望的前进后退速度
    float target_angle;         // 目标角度 - 期望的平衡角度（通常为0度）
    float target_yaw_rate;      // 目标偏航角速度 - 期望的转向角速度（度/秒）
    float distance_front;       // 前方距离 - 超声波测得的前方障碍物距离（厘米）
    uint8_t balance_enabled;    // 平衡使能标志 - 1:启用平衡控制, 0:禁用
    uint8_t obstacle_detected;  // 障碍物检测标志 - 1:检测到障碍物, 0:无障碍物
    uint8_t vision_mode;        // 视觉模式 - 0:关闭, 1:循迹, 2:物体追踪
    float vision_error_x;       // 视觉X轴误差 - 用于转向控制
    float vision_error_y;       // 视觉Y轴误差 - 用于速度控制
} BalanceState_t;

/* 平衡控制系统参数定义 */

/**
 * @brief 平衡控制基本参数
 */
#define BALANCE_TARGET_ANGLE   -5.2f    // 目标平衡角度（度）- 平衡车的理想直立角度
#define MAX_TILT_ANGLE         45.0f    // 最大倾斜角度（度）- 超过此角度触发保护
#define MIN_OBSTACLE_DISTANCE  15.0f    // 最小障碍物距离（厘米）- 触发避障的距离阈值

/**
 * @brief PID控制器参数定义
 * @note  三个PID控制器分别控制角度、速度和转向，参数需要根据实际硬件调优
 */

/* 角度环PID参数 - 控制平衡车的俯仰角度，维持直立状态 */
#define ANGLE_PID_KP           42.0f    // 角度比例系数 - 主要控制力，影响平衡响应速度
#define ANGLE_PID_KI           0.0f     // 角度积分系数 - 消除静态误差，防止长期偏移
#define ANGLE_PID_KD           1.5f     // 角度微分系数 - 阻尼作用，减少震荡
#define ANGLE_PID_MAX_OUTPUT   900.0f  // 角度PID最大输出 - 限制电机最大驱动力
#define ANGLE_PID_MAX_INTEGRAL 100.0f   // 角度PID积分限幅 - 防止积分饱和

/* 速度环PID参数 - 控制平衡车的前进后退速度 */
#define SPEED_PID_KP           -2000.0f     // 速度比例系数 - 速度跟踪响应强度
#define SPEED_PID_KI           SPEED_PID_KP / 200.0f     // 速度积分系数 - 消除速度稳态误差
#define SPEED_PID_KD           0.0f     // 速度微分系数 - 通常设为0，避免噪声放大
#define SPEED_PID_MAX_OUTPUT   500.0f    // 速度PID最大输出 
#define SPEED_PID_MAX_INTEGRAL 50.0f    // 速度PID积分限幅 

/* 转向PID参数 - 控制平衡车的左右转向 */
#define TURN_PID_KP            2.0f    // 转向比例系数 - 转向响应强度
#define TURN_PID_KI            0.0f     // 转向积分系数 - 通常设为0，避免转向漂移
#define TURN_PID_KD            0.01f     // 转向微分系数 - 提高转向稳定性
#define TURN_PID_MAX_OUTPUT    500.0f   // 转向PID最大输出 - 限制转向力矩
#define TURN_PID_MAX_INTEGRAL  100.0f   // 转向PID积分限幅 - 防止积分饱和

/*视觉PID参数 - 控制平衡车的循迹左右转向*/
#define VISION_KP              -100.0f    //误差比例系数

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
 * @brief 设置目标偏航角速度（用于转向控制）
 * @param yaw_rate 目标偏航角速度值（度/秒）
 */
void BalanceControl_SetTargetYawRate(float yaw_rate);

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

/**
 * @brief 设置视觉控制模式
 * @param mode 视觉模式 - 0:关闭, 1:循迹, 2:物体追踪
 */
void BalanceControl_SetVisionMode(uint8_t mode);

/**
 * @brief 视觉控制更新
 * @note 根据视觉数据调整平衡车的运动
 */
void BalanceControl_VisionUpdate(void);

/**
 * @brief 循迹控制
 * @note 根据线条位置调整转向
 */
void BalanceControl_LineTracking(void);

/**
 * @brief 物体追踪控制
 * @note 根据物体位置调整运动
 */
void BalanceControl_ObjectTracking(void);

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
