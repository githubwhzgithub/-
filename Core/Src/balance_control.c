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

    // 获取超声波数据
    BalanceState.distance_front = Ultrasonic_GetDistance();

    // 障碍物检测
    BalanceControl_ObstacleAvoidance();
    
    // 视觉控制更新
    BalanceControl_VisionUpdate();

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

    // 转向控制 (基于横滚角和视觉误差)
    float turn_setpoint = 0.0f;
    if(BalanceState.vision_mode > 0) {
        // 视觉模式下使用视觉误差作为转向目标
        turn_setpoint = BalanceState.vision_error_x;
    }
    TurnPID.setpoint = turn_setpoint;
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
    /*if(mode == 1) {
        K230_Vision_SetMode(K230_MODE_LINE_TRACKING);
    } else if(mode == 2) {
        K230_Vision_SetMode(K230_MODE_OBJECT_TRACKING);
    } else {
        K230_Vision_SetMode(K230_MODE_IDLE);
    }*/
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
 * @note 根据线条位置调整转向
 */
void BalanceControl_LineTracking(void)
{
    /*K230_LineTrack_t* line_data = K230_Vision_GetLineTrackingData();
    
    if(K230_Vision_IsLineDetected()) {
        // 计算线条中心相对于图像中心的偏移
        float image_center_x = 160.0f; // 假设图像宽度为320像素
        float line_center_x = line_data->line_x + line_data->line_w / 2.0f;
        
        // 计算X轴误差 (转向控制)
        BalanceState.vision_error_x = (line_center_x - image_center_x) / image_center_x;
        
        // 限制误差范围
        if(BalanceState.vision_error_x > 1.0f) BalanceState.vision_error_x = 1.0f;
        if(BalanceState.vision_error_x < -1.0f) BalanceState.vision_error_x = -1.0f;
        
        // 根据线条宽度调整速度 (线条越宽说明越近，速度可以快一些)
        float speed_factor = line_data->w / 100.0f; // 归一化线条宽度
        if(speed_factor > 1.0f) speed_factor = 1.0f;
        if(speed_factor < 0.3f) speed_factor = 0.3f; // 最小速度因子
        
        // 设置前进速度
        BalanceState.target_speed = 0.2f * speed_factor; // 基础速度0.2m/s
        
    } else {
        // 没有检测到线条，停止前进
        BalanceState.target_speed = 0.0f;
        BalanceState.vision_error_x = 0.0f;
    } */
}

/**
 * @brief 物体追踪控制
 * @note 根据物体位置调整运动
 */
void BalanceControl_ObjectTracking(void)
{
    /*K230_ObjectTrack_t* obj_data = K230_Vision_GetObjectTrackingData();
    
    if(K230_Vision_IsObjectDetected()) {
        // 计算物体中心相对于图像中心的偏移
        float image_center_x = 160.0f; // 假设图像宽度为320像素
        float image_center_y = 120.0f; // 假设图像高度为240像素
        
        float obj_center_x = obj_data->obj_x + obj_data->obj_w / 2.0f;
        float obj_center_y = obj_data->obj_y + obj_data->obj_h / 2.0f;
        
        // 计算X轴误差 (转向控制)
        BalanceState.vision_error_x = (obj_center_x - image_center_x) / image_center_x;
        
        // 计算Y轴误差 (距离控制)
        BalanceState.vision_error_y = (obj_center_y - image_center_y) / image_center_y;
        
        // 限制误差范围
        if(BalanceState.vision_error_x > 1.0f) BalanceState.vision_error_x = 1.0f;
        if(BalanceState.vision_error_x < -1.0f) BalanceState.vision_error_x = -1.0f;
        if(BalanceState.vision_error_y > 1.0f) BalanceState.vision_error_y = 1.0f;
        if(BalanceState.vision_error_y < -1.0f) BalanceState.vision_error_y = -1.0f;
        
        // 根据物体大小调整速度 (物体越大说明越近)
        float obj_size = obj_data->obj_w * obj_data->obj_h;
        float distance_factor = 1.0f - (obj_size / (320.0f * 240.0f)); // 归一化距离因子
        if(distance_factor < 0.1f) distance_factor = 0.1f; // 最小距离因子
        if(distance_factor > 1.0f) distance_factor = 1.0f;
        
        // 设置追踪速度
        if(distance_factor > 0.5f) {
            // 物体较远，前进追踪
            BalanceState.target_speed = 0.3f * distance_factor;
        } else {
            // 物体较近，减速或停止
            BalanceState.target_speed = 0.1f;
        }
        
    } else {
        // 没有检测到物体，停止运动
        BalanceState.target_speed = 0.0f;
        BalanceState.vision_error_x = 0.0f;
        BalanceState.vision_error_y = 0.0f;
    } */
}
