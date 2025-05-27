#ifndef __BALANCE_CONTROL_H
#define __BALANCE_CONTROL_H

#include "main.h"
#include "mpu6050.h"
#include "TB6612.h"
#include "motorencoder.h"
#include "ultrasonic.h"

// PID控制器结构体
typedef struct {
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数
    float setpoint;     // 目标值
    float integral;     // 积分累积
    float last_error;   // 上次误差
    float output;       // 输出值
    float max_output;   // 最大输出限制
    float min_output;   // 最小输出限制
    float max_integral; // 积分限幅
} PID_Controller_t;

// 平衡车状态结构体
typedef struct {
    float pitch;                // 俯仰角
    float roll;                 // 横滚角
    float pitch_rate;           // 俯仰角速度
    float roll_rate;            // 横滚角速度
    float left_speed;           // 左轮速度
    float right_speed;          // 右轮速度
    float target_speed;         // 目标速度
    float target_angle;         // 目标角度
    float distance_front;       // 前方距离
    uint8_t balance_enabled;    // 平衡使能标志
    uint8_t obstacle_detected;  // 障碍物检测标志
} BalanceState_t;

// 平衡控制参数
#define BALANCE_TARGET_ANGLE    0.0f    // 目标平衡角度
#define MAX_TILT_ANGLE         45.0f    // 最大倾斜角度
#define MIN_OBSTACLE_DISTANCE  20.0f    // 最小障碍物距离 (cm)
#define CONTROL_FREQUENCY      100      // 控制频率 (Hz)
#define CONTROL_PERIOD_MS      10       // 控制周期 (ms)

// PID参数定义
// 角度环PID参数
#define ANGLE_PID_KP           80.0f
#define ANGLE_PID_KI           0.5f
#define ANGLE_PID_KD           2.0f
#define ANGLE_PID_MAX_OUTPUT   1000.0f
#define ANGLE_PID_MAX_INTEGRAL 500.0f

// 速度环PID参数
#define SPEED_PID_KP           5.0f
#define SPEED_PID_KI           0.1f
#define SPEED_PID_KD           0.0f
#define SPEED_PID_MAX_OUTPUT   10.0f
#define SPEED_PID_MAX_INTEGRAL 50.0f

// 转向PID参数
#define TURN_PID_KP            20.0f
#define TURN_PID_KI            0.0f
#define TURN_PID_KD            1.0f
#define TURN_PID_MAX_OUTPUT    500.0f
#define TURN_PID_MAX_INTEGRAL  100.0f

// 函数声明
void BalanceControl_Init(void);
void BalanceControl_Update(void);
void BalanceControl_Enable(uint8_t enable);
void BalanceControl_SetTargetSpeed(float speed);
void BalanceControl_SetTargetAngle(float angle);
float BalanceControl_PID_Update(PID_Controller_t* pid, float current_value, float dt);
void BalanceControl_PID_Reset(PID_Controller_t* pid);
void BalanceControl_EmergencyStop(void);
void BalanceControl_ObstacleAvoidance(void);
BalanceState_t* BalanceControl_GetState(void);

// 控制器实例声明
extern PID_Controller_t AnglePID;
extern PID_Controller_t SpeedPID;
extern PID_Controller_t TurnPID;
extern BalanceState_t BalanceState;

#endif /* __BALANCE_CONTROL_H */