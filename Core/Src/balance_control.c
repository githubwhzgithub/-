#include "balance_control.h"
#include <math.h>

// PID控制器实例
PID_Controller_t AnglePID = {
    .Kp = ANGLE_PID_KP,
    .Ki = ANGLE_PID_KI,
    .Kd = ANGLE_PID_KD,
    .setpoint = BALANCE_TARGET_ANGLE,
    .integral = 0.0f,
    .last_error = 0.0f,
    .output = 0.0f,
    .max_output = ANGLE_PID_MAX_OUTPUT,
    .min_output = -ANGLE_PID_MAX_OUTPUT,
    .max_integral = ANGLE_PID_MAX_INTEGRAL
};

PID_Controller_t SpeedPID = {
    .Kp = SPEED_PID_KP,
    .Ki = SPEED_PID_KI,
    .Kd = SPEED_PID_KD,
    .setpoint = 0.0f,
    .integral = 0.0f,
    .last_error = 0.0f,
    .output = 0.0f,
    .max_output = SPEED_PID_MAX_OUTPUT,
    .min_output = -SPEED_PID_MAX_OUTPUT,
    .max_integral = SPEED_PID_MAX_INTEGRAL
};

PID_Controller_t TurnPID = {
    .Kp = TURN_PID_KP,
    .Ki = TURN_PID_KI,
    .Kd = TURN_PID_KD,
    .setpoint = 0.0f,
    .integral = 0.0f,
    .last_error = 0.0f,
    .output = 0.0f,
    .max_output = TURN_PID_MAX_OUTPUT,
    .min_output = -TURN_PID_MAX_OUTPUT,
    .max_integral = TURN_PID_MAX_INTEGRAL
};

// 平衡车状态实例
BalanceState_t BalanceState = {
    .pitch = 0.0f,
    .roll = 0.0f,
    .pitch_rate = 0.0f,
    .roll_rate = 0.0f,
    .left_speed = 0.0f,
    .right_speed = 0.0f,
    .target_speed = 0.0f,
    .target_angle = BALANCE_TARGET_ANGLE,
    .distance_front = 0.0f,
    .balance_enabled = 0,
    .obstacle_detected = 0
};

// 私有变量
static uint32_t last_control_time = 0;

/**
 * @brief 初始化平衡控制系统
 */
void BalanceControl_Init(void)
{
    // 初始化各个模块
    TB6612_Init();
    MotorEncoder_Init();
    Ultrasonic_Init();
    
    // 重置PID控制器
    BalanceControl_PID_Reset(&AnglePID);
    BalanceControl_PID_Reset(&SpeedPID);
    BalanceControl_PID_Reset(&TurnPID);
    
    // 初始化时间戳
    last_control_time = HAL_GetTick();
    
    // 停止电机
    TB6612_StopAllMotors();
}

/**
 * @brief PID控制器更新
 * @param pid: PID控制器指针
 * @param current_value: 当前值
 * @param dt: 时间间隔
 * @return PID输出值
 */
float BalanceControl_PID_Update(PID_Controller_t* pid, float current_value, float dt)
{
    // 计算误差
    float error = pid->setpoint - current_value;
    
    // 比例项
    float proportional = pid->Kp * error;
    
    // 积分项
    pid->integral += error * dt;
    
    // 积分限幅
    if(pid->integral > pid->max_integral) {
        pid->integral = pid->max_integral;
    } else if(pid->integral < -pid->max_integral) {
        pid->integral = -pid->max_integral;
    }
    
    float integral = pid->Ki * pid->integral;
    
    // 微分项
    float derivative = 0.0f;
    if(dt > 0) {
        derivative = pid->Kd * (error - pid->last_error) / dt;
    }
    
    // 计算输出
    pid->output = proportional + integral + derivative;
    
    // 输出限幅
    if(pid->output > pid->max_output) {
        pid->output = pid->max_output;
    } else if(pid->output < pid->min_output) {
        pid->output = pid->min_output;
    }
    
    // 保存当前误差
    pid->last_error = error;
    
    return pid->output;
}

/**
 * @brief 重置PID控制器
 * @param pid: PID控制器指针
 */
void BalanceControl_PID_Reset(PID_Controller_t* pid)
{
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->output = 0.0f;
}

/**
 * @brief 平衡控制主更新函数
 */
void BalanceControl_Update(void)
{
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - last_control_time) / 1000.0f; // 转换为秒
    
    // 控制频率限制
    if(dt < (CONTROL_PERIOD_MS / 1000.0f)) {
        return;
    }
    
    last_control_time = current_time;
    
    // 更新传感器数据
    MPU_Get_Filtered_Angles(&BalanceState.pitch, &BalanceState.roll);
    
    float gx, gy, gz;
    if(MPU_Get_Gyroscope(&gx, &gy, &gz) == 0) {
        BalanceState.pitch_rate = gy;
        BalanceState.roll_rate = gx;
    }
    
    // 更新编码器数据
    MotorEncoder_Update();
    BalanceState.left_speed = MotorEncoder_GetSpeedMPS(&EncoderA);
    BalanceState.right_speed = MotorEncoder_GetSpeedMPS(&EncoderB);
    
    // 更新超声波数据
    Ultrasonic_Update();
    BalanceState.distance_front = Ultrasonic_GetFilteredDistance();
    
    // 障碍物检测
    BalanceControl_ObstacleAvoidance();
    
    // 检查是否需要紧急停止
    if(fabs(BalanceState.pitch) > MAX_TILT_ANGLE || 
       fabs(BalanceState.roll) > MAX_TILT_ANGLE ||
       BalanceState.obstacle_detected) {
        BalanceControl_EmergencyStop();
        return;
    }
    
    // 如果平衡控制未使能，停止电机
    if(!BalanceState.balance_enabled) {
        TB6612_StopAllMotors();
        return;
    }
    
    // 速度环控制
    float average_speed = (BalanceState.left_speed + BalanceState.right_speed) / 2.0f;
    SpeedPID.setpoint = BalanceState.target_speed;
    float speed_output = BalanceControl_PID_Update(&SpeedPID, average_speed, dt);
    
    // 角度环控制
    AnglePID.setpoint = BalanceState.target_angle + speed_output;
    float angle_output = BalanceControl_PID_Update(&AnglePID, BalanceState.pitch, dt);
    
    // 转向控制 (基于横滚角)
    TurnPID.setpoint = 0.0f; // 保持直行
    float turn_output = BalanceControl_PID_Update(&TurnPID, BalanceState.roll, dt);
    
    // 计算左右电机输出
    int16_t left_motor_output = (int16_t)(angle_output - turn_output);
    int16_t right_motor_output = (int16_t)(angle_output + turn_output);
    
    // 限制电机输出范围
    if(left_motor_output > 1000) left_motor_output = 1000;
    if(left_motor_output < -1000) left_motor_output = -1000;
    if(right_motor_output > 1000) right_motor_output = 1000;
    if(right_motor_output < -1000) right_motor_output = -1000;
    
    // 设置电机速度
    TB6612_SetBothMotors(left_motor_output, right_motor_output);
}

/**
 * @brief 使能/禁用平衡控制
 * @param enable: 1-使能, 0-禁用
 */
void BalanceControl_Enable(uint8_t enable)
{
    BalanceState.balance_enabled = enable;
    
    if(!enable) {
        TB6612_StopAllMotors();
        BalanceControl_PID_Reset(&AnglePID);
        BalanceControl_PID_Reset(&SpeedPID);
        BalanceControl_PID_Reset(&TurnPID);
    }
}

/**
 * @brief 设置目标速度
 * @param speed: 目标速度 (m/s)
 */
void BalanceControl_SetTargetSpeed(float speed)
{
    BalanceState.target_speed = speed;
}

/**
 * @brief 设置目标角度
 * @param angle: 目标角度 (度)
 */
void BalanceControl_SetTargetAngle(float angle)
{
    BalanceState.target_angle = angle;
}

/**
 * @brief 紧急停止
 */
void BalanceControl_EmergencyStop(void)
{
    TB6612_BrakeAllMotors();
    BalanceState.balance_enabled = 0;
    
    // 重置PID控制器
    BalanceControl_PID_Reset(&AnglePID);
    BalanceControl_PID_Reset(&SpeedPID);
    BalanceControl_PID_Reset(&TurnPID);
}

/**
 * @brief 障碍物避障
 */
void BalanceControl_ObstacleAvoidance(void)
{
    // 启动超声波测距
    if(Ultrasonic_GetState() == ULTRASONIC_IDLE) {
        Ultrasonic_StartMeasurement();
    }
    
    // 检查前方障碍物
    if(BalanceState.distance_front < MIN_OBSTACLE_DISTANCE && 
       BalanceState.distance_front > MIN_DISTANCE) {
        BalanceState.obstacle_detected = 1;
        
        // 如果正在前进，则停止
        if(BalanceState.target_speed > 0) {
            BalanceState.target_speed = 0;
        }
    } else {
        BalanceState.obstacle_detected = 0;
    }
}

/**
 * @brief 获取平衡车状态
 * @return 平衡车状态指针
 */
BalanceState_t* BalanceControl_GetState(void)
{
    return &BalanceState;
}