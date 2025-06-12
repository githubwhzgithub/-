#include "balance_control.h"
#include "bluetooth.h"
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
    .max_integral = ANGLE_PID_MAX_INTEGRAL,
    .filtered_derivative = 0.0f
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
    .max_integral = SPEED_PID_MAX_INTEGRAL,
    .filtered_derivative = 0.0f
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
    .max_integral = TURN_PID_MAX_INTEGRAL,
    .filtered_derivative = 0.0f
};

// 平衡车状态实例
BalanceState_t BalanceState = {
    .pitch = 0.0f,
    .roll = 0.0f,
    .pitch_rate = 0.0f,
    .roll_rate = 0.0f,
    .yaw_rate = 0.0f,
    .left_speed = 0.0f,
    .right_speed = 0.0f,
    .target_speed = 0.0f,
    .target_angle = BALANCE_TARGET_ANGLE,
    .target_yaw_rate = 0.0f,
    .distance_front = 0.0f,
    .balance_enabled = 1,
    .obstacle_detected = 0,
    .vision_mode = 0,
    .vision_error_x = 0.0f,
    .vision_error_y = 0.0f
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
    // 参数有效性检查
    if(pid == NULL || dt <= 0.0f || dt > 1.0f) {
        return pid ? pid->output : 0.0f;
    }

    // 计算误差
    float error = pid->setpoint - current_value;
    
    // 死区处理 - 减少小误差时的抖动
    const float deadzone = 0.01f;
    if(fabsf(error) < deadzone) {
        error = 0.0f;
    }

    // 比例项
    float proportional = pid->Kp * error;

    // 积分项计算（改进的抗饱和机制）
    float integral_term = pid->Ki * pid->integral;
    float pre_output = proportional + integral_term;
    
    // 只有在输出未饱和时才累积积分
    int output_saturated = (pre_output > pid->max_output) || (pre_output < pid->min_output);
    int integral_same_sign = (error * pid->integral) > 0.0f;
    
    if(!output_saturated || !integral_same_sign) {
        pid->integral += error * dt;
        
        // 积分限幅
        if(pid->integral > pid->max_integral) {
            pid->integral = pid->max_integral;
        } else if(pid->integral < -pid->max_integral) {
            pid->integral = -pid->max_integral;
        }
    }
    
    integral_term = pid->Ki * pid->integral;

    // 微分项计算（带低通滤波减少噪声）
     float derivative = 0.0f;
     if(pid->Kd > 0.0f) {
         float raw_derivative = (error - pid->last_error) / dt;
         
         // 一阶低通滤波器，截止频率约为采样频率的1/10
         const float alpha = 0.9f;  // 滤波系数
         pid->filtered_derivative = alpha * raw_derivative + (1.0f - alpha) * pid->filtered_derivative;
         
         derivative = pid->Kd * pid->filtered_derivative;
         
         // 微分项限幅，防止噪声导致的过大输出
         const float max_derivative = pid->max_output * 0.3f;
         if(derivative > max_derivative) {
             derivative = max_derivative;
         } else if(derivative < -max_derivative) {
             derivative = -max_derivative;
         }
     }

    // 计算最终输出
    pid->output = proportional + integral_term + derivative;

    // 输出限幅
    if(pid->output > pid->max_output) {
        pid->output = pid->max_output;
    } else if(pid->output < pid->min_output) {
        pid->output = pid->min_output;
    }

    // 保存当前误差用于下次微分计算
    pid->last_error = error;

    return pid->output;
}

/**
 * @brief 重置PID控制器
 * @param pid: PID控制器指针
 */
void BalanceControl_PID_Reset(PID_Controller_t* pid)
{
    if(pid != NULL) {
        pid->integral = 0.0f;
        pid->last_error = 0.0f;
        pid->output = 0.0f;
        pid->filtered_derivative = 0.0f;
    }
}

/**
 * @brief 平衡控制主更新函数
 */
void BalanceControl_Update(void)
{
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - last_control_time) / 1000.0f; // 转换为秒


    last_control_time = current_time;

    // 更新传感器数据
    MPU_Get_Filtered_Angles(&BalanceState.pitch, &BalanceState.roll);

    float gx, gy, gz;
    if(MPU_Get_Gyroscope(&gx, &gy, &gz) == 0) {
        BalanceState.pitch_rate = gy;
        BalanceState.roll_rate = gx;
        BalanceState.yaw_rate = gz;  // Z轴角速度用于转向控制
    }

    // 获取编码器数据（编码器在main.c中每10ms更新一次）
    BalanceState.left_speed = MotorEncoder_GetSpeedMPS(&EncoderA);
    BalanceState.right_speed = MotorEncoder_GetSpeedMPS(&EncoderB);
    

    // 获取超声波数据
    BalanceState.distance_front = Ultrasonic_GetDistance();

    // 障碍物检测
    BalanceControl_ObstacleAvoidance();
    
    // 视觉控制更新
    BalanceControl_VisionUpdate();

    // 检查是否需要紧急停止 - 主要检查roll值（平衡控制轴）
    if(fabs(BalanceState.roll) > MAX_TILT_ANGLE ||
       fabs(BalanceState.pitch) > MAX_TILT_ANGLE ) {
        //停止前进
        TB6612_StopAllMotors();
    }

    // 障碍物检测
    if(BalanceState.obstacle_detected) {
        BalanceState.target_speed = 0.0f;           //停止前进
    }

    // 如果平衡控制未使能，停止电机
    if(!BalanceState.balance_enabled) {
        TB6612_StopAllMotors();
        return;
    }

    // 角度环控制 - 使用roll值进行平衡控制
    BalanceState.target_angle = BALANCE_TARGET_ANGLE;
    AnglePID.setpoint = BalanceState.target_angle ;
    float angle_output = BalanceControl_PID_Update(&AnglePID, BalanceState.roll, dt);

    // 速度环控制
    float speed_output = 0.0f;
    float average_speed = (BalanceState.left_speed + BalanceState.right_speed) / 2.0f;
    if(BalanceState.vision_mode == 0){
        SpeedPID.setpoint = BalanceState.target_speed;
        speed_output = BalanceControl_PID_Update(&SpeedPID, average_speed, dt);
    }
    else{
        // 设置追踪速度
        if(K230_Vision_IsObjectDetected() || K230_Vision_IsLineDetected()){
            BalanceState.target_speed = 0.04f;
        }else{
            BalanceState.target_speed = 0.0f;
        }
        SpeedPID.setpoint = BalanceState.target_speed;
        speed_output = BalanceControl_PID_Update(&SpeedPID, average_speed, dt);
    }
    

    // 转向控制
    float turn_output = 0.0f;
    if(BalanceState.vision_mode > 0) {
        // 视觉模式下直接使用视觉误差进行转向控制
        // vision_error_x范围为[-1.0, 1.0]，需要转换为合适的转向输出
        TurnPID.setpoint = VISION_KP * BalanceState.vision_error_x; // 使用目标偏航角速度
        turn_output = BalanceControl_PID_Update(&TurnPID, BalanceState.yaw_rate, dt);
    } else {
        // 非视觉模式下，基于Z轴角速度进行转向控制
        TurnPID.setpoint = BalanceState.target_yaw_rate; // 使用目标偏航角速度
        turn_output = BalanceControl_PID_Update(&TurnPID, BalanceState.yaw_rate, dt);
        
    }

    // 计算左右电机输出
    int16_t left_motor_output = (int16_t)(speed_output+angle_output - turn_output);
    int16_t right_motor_output = (int16_t)(speed_output+angle_output + turn_output);

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
 * @brief 设置目标偏航角速度
 * @param yaw_rate: 目标偏航角速度 (度/秒)
 */
void BalanceControl_SetTargetYawRate(float yaw_rate)
{
    BalanceState.target_yaw_rate = yaw_rate;
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
    if(BalanceState.distance_front < MIN_OBSTACLE_DISTANCE) {
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

/**
 * @brief 设置视觉控制模式
 * @param mode 视觉模式 - 0:关闭, 1:循迹, 2:物体追踪
 */
void BalanceControl_SetVisionMode(uint8_t mode)
{
    BalanceState.vision_mode = mode;
    
    // 切换模式时重置视觉误差
    BalanceState.vision_error_x = 0.0f;
    BalanceState.vision_error_y = 0.0f;
    
    // 设置K230视觉模块的工作模式
    if(mode == 1) {
        K230_SetMode(K230_MODE_LINE_TRACK);
    } else if(mode == 2) {
        K230_SetMode(K230_MODE_OBJECT_TRACK);
    } else {
        K230_SetMode(K230_MODE_IDLE);
    }
}

/**
 * @brief 视觉控制更新
 * @note 根据视觉数据调整平衡车的运动
 */
void BalanceControl_VisionUpdate(void)
{
    if(BalanceState.vision_mode == 1) {
        // 循迹模式
        BalanceControl_LineTracking();
    } else if(BalanceState.vision_mode == 2) {
        // 物体追踪模式
        BalanceControl_ObjectTracking();
    }
}

/**
 * @brief 循迹控制
 * @note 根据线条位置调整转向和速度
 */
void BalanceControl_LineTracking(void)
{
    K230_Vision_t* vision_data = K230_GetVisionData();
    
    if(K230_Vision_IsLineDetected()) {
        // 计算线条中心相对于图像中心的偏移
        float image_center_x = 320.0f; // OpenMV图像宽度为640像素，中心为320
        float line_center_x = (float)vision_data->line_track.line_x;
        
        // 计算X轴误差 (转向控制)
        float raw_error_x = (line_center_x - image_center_x) / image_center_x;
        
        // 对转向误差进行低通滤波，减少抖动
        static float filtered_error_x = 0.0f;
        float filter_alpha = 0.7f; // 滤波系数，越小越平滑
        filtered_error_x = filter_alpha * raw_error_x + (1.0f - filter_alpha) * filtered_error_x;
        
        BalanceState.vision_error_x = filtered_error_x;
        
        // 限制误差范围
        if(BalanceState.vision_error_x > 1.0f) BalanceState.vision_error_x = 1.0f;
        if(BalanceState.vision_error_x < -1.0f) BalanceState.vision_error_x = -1.0f;
        
        // 使用vision_tracker.py发送的角度信息进行转向控制
        float line_angle = (float)vision_data->line_track.line_angle;
        
        // 根据线条角度调整速度（角度越大，速度越慢）
        float angle_speed_factor = 1.0f - 0.3f * (fabs(line_angle) / 90.0f);
        if(angle_speed_factor < 0.4f) angle_speed_factor = 0.4f;
        if(angle_speed_factor > 1.0f) angle_speed_factor = 1.0f;
        
        // 根据转向误差进一步调整速度（转向越大，速度越慢）
        float turn_speed_factor = 1.0f - 0.5f * fabs(BalanceState.vision_error_x);
        if(turn_speed_factor < 0.5f) turn_speed_factor = 0.5f;
        
        // 设置前进速度（基础速度 * 角度速度因子 * 转向速度因子）
        BalanceState.target_speed = 0.25f * angle_speed_factor * turn_speed_factor;
        
    } 
}

/**
 * @brief 物体追踪控制
 * @note 根据物体位置调整运动
 */
void BalanceControl_ObjectTracking(void)
{
    K230_Vision_t* vision_data = K230_GetVisionData();

    
    if(K230_Vision_IsObjectDetected()) {
        // 计算物体中心相对于图像中心的偏移
        float image_center_x = 320.0f; // vision_tracker.py图像宽度为640像素，中心为320
        float image_center_y = 240.0f; // vision_tracker.py图像高度为480像素，中心为240
        
        // vision_tracker.py直接发送物体中心坐标
        float obj_center_x = (float)vision_data->object_track.obj_x;
        float obj_center_y = (float)vision_data->object_track.obj_y;
        
        // 计算X轴误差 (转向控制)
        float raw_error_x = (obj_center_x - image_center_x) / image_center_x;
        
        // 对转向误差进行低通滤波，减少抖动
        static float filtered_error_x = 0.0f;
        float filter_alpha = 0.6f; // 滤波系数
        filtered_error_x = filter_alpha * raw_error_x + (1.0f - filter_alpha) * filtered_error_x;
        
        BalanceState.vision_error_x = filtered_error_x;
        
        // 计算Y轴误差 (距离控制)
        BalanceState.vision_error_y = (obj_center_y - image_center_y) / image_center_y;
        
        // 限制误差范围
        if(BalanceState.vision_error_x > 1.0f) BalanceState.vision_error_x = 1.0f;
        if(BalanceState.vision_error_x < -1.0f) BalanceState.vision_error_x = -1.0f;
        if(BalanceState.vision_error_y > 1.0f) BalanceState.vision_error_y = 1.0f;
        if(BalanceState.vision_error_y < -1.0f) BalanceState.vision_error_y = -1.0f;
        
        
        // 根据转向误差调整速度（转向越大，速度越慢）
        float turn_speed_factor = 1.0f - 0.4f * fabs(BalanceState.vision_error_x);
        if(turn_speed_factor < 0.3f) turn_speed_factor = 0.3f;
    }
}
