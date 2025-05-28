/**
 * @file    balance_control.c
 * @brief   平衡车控制算法实现文件
 * @details 该文件实现了平衡车的核心控制算法，包括角度PID控制、
 *          速度PID控制和转向PID控制。通过多级PID控制器的协调工作，
 *          实现平衡车的自平衡、速度控制和转向控制功能。
 *
 *          控制策略说明：
 *          1. 角度环（内环）：维持车体直立，响应速度最快
 *          2. 速度环（外环）：控制前进后退速度，调节角度环的目标值
 *          3. 转向环（独立）：控制左右转向，通过差速实现
 *
 * @author  STM32平衡车项目组
 * @version 1.0
 * @date    2024
 *
 * @note    控制周期建议为5-10ms，过快或过慢都会影响控制效果
 */

#include "balance_control.h"
#include "TB6612.h"
#include "mpu6050.h"
#include "motorencoder.h"

/*==============================================================================
                                全局变量定义
==============================================================================*/

/**
 * @brief 角度PID控制器实例
 * @details 角度环是平衡车控制的核心，负责维持车体的直立状态。
 *          该PID控制器以车体倾斜角度为输入，输出基础电机控制量。
 *          角度环响应速度最快，是整个控制系统的基础。
 */
PID_Controller_t AnglePID = {
    .Kp = ANGLE_KP,         // 角度比例系数，影响响应速度
    .Ki = ANGLE_KI,         // 角度积分系数，消除静态误差
    .Kd = ANGLE_KD,         // 角度微分系数，提高系统稳定性
    .integral = 0.0f,       // 积分累积值，初始为0
    .prev_error = 0.0f,     // 上次误差值，用于微分计算
    .output = 0.0f          // PID输出值，即电机控制量
};

/**
 * @brief 速度PID控制器实例
 * @details 速度环控制平衡车的前进后退速度，通过调节角度环的目标角度
 *          来实现速度控制。当需要前进时，目标角度略微前倾；
 *          当需要后退时，目标角度略微后倾。
 */
PID_Controller_t SpeedPID = {
    .Kp = SPEED_KP,         // 速度比例系数，影响速度响应
    .Ki = SPEED_KI,         // 速度积分系数，消除速度稳态误差
    .Kd = SPEED_KD,         // 速度微分系数，抑制速度超调
    .integral = 0.0f,       // 积分累积值，初始为0
    .prev_error = 0.0f,     // 上次误差值，用于微分计算
    .output = 0.0f          // PID输出值，作为角度环的目标角度修正
};

/**
 * @brief 转向PID控制器实例
 * @details 转向环控制平衡车的左右转向，通过在左右电机输出上叠加
 *          差值来实现转向。转向控制独立于角度和速度控制，
 *          可以在保持平衡和速度的同时进行转向操作。
 */
PID_Controller_t TurnPID = {
    .Kp = TURN_KP,          // 转向比例系数，影响转向响应速度
    .Ki = TURN_KI,          // 转向积分系数，消除转向稳态误差
    .Kd = TURN_KD,          // 转向微分系数，提高转向稳定性
    .integral = 0.0f,       // 积分累积值，初始为0
    .prev_error = 0.0f,     // 上次误差值，用于微分计算
    .output = 0.0f          // PID输出值，即左右电机的差值
};

/**
 * @brief 平衡车状态实例
 * @details 存储平衡车当前的所有状态信息，包括传感器数据、
 *          目标值、控制输出等。该结构体是整个控制系统的
 *          数据中心，所有控制算法都基于这些状态数据。
 */
BalanceState_t BalanceState = {
    .angle = 0.0f,              // 当前车体倾斜角度（度）
    .angular_velocity = 0.0f,   // 当前角速度（度/秒）
    .speed = 0.0f,              // 当前移动速度（编码器计算得出）
    .target_angle = TARGET_ANGLE,   // 目标角度，通常为0度（直立）
    .target_speed = 0.0f,       // 目标速度，由用户指令设定
    .target_turn = 0.0f,        // 目标转向速度，由用户指令设定
    .motor_output_left = 0,     // 左电机输出值
    .motor_output_right = 0,    // 右电机输出值
    .is_balanced = false        // 平衡状态标志，true表示已平衡
};

/*==============================================================================
                                私有变量定义
==============================================================================*/

/**
 * @brief 上次控制周期的时间戳
 * @details 用于计算控制周期的时间间隔，确保PID控制器的微分和积分
 *          计算基于准确的时间间隔。时间戳以毫秒为单位。
 */
static uint32_t last_control_time = 0;

/*==============================================================================
                                函数实现
==============================================================================*/

/**
 * @brief  平衡控制系统初始化
 * @details 初始化平衡车控制系统的所有组件，包括电机驱动、编码器、
 *          超声波传感器和PID控制器。该函数必须在开始平衡控制前调用。
 *          初始化完成后，系统处于安全状态（电机停止）。
 *
 * @param  无
 * @retval 无
 *
 * @note   该函数会停止所有电机，确保初始化过程的安全性
 * @note   PID控制器的积分项和微分项会被重置为0
 * @note   建议在主函数中调用，在开始控制循环之前
 */
void BalanceControl_Init(void)
{
    // 初始化硬件模块
    TB6612_Init();          // 初始化电机驱动模块
    MotorEncoder_Init();    // 初始化电机编码器模块
    Ultrasonic_Init();      // 初始化超声波传感器模块

    // 重置所有PID控制器，清除历史数据
    BalanceControl_PID_Reset(&AnglePID);  // 重置角度PID控制器
    BalanceControl_PID_Reset(&SpeedPID);  // 重置速度PID控制器
    BalanceControl_PID_Reset(&TurnPID);   // 重置转向PID控制器

    // 初始化控制时间戳，为后续时间间隔计算做准备
    last_control_time = HAL_GetTick();

    // 确保电机处于停止状态，保证初始化安全
    TB6612_StopAllMotors();
}

/**
 * @brief  PID控制器更新计算
 * @details 根据当前值和目标值计算PID控制器的输出。该函数实现了
 *          标准的PID控制算法，包括比例项、积分项和微分项的计算。
 *          积分项具有限幅功能，防止积分饱和；微分项基于误差变化率计算。
 *
 * @param  pid: 指向PID控制器结构体的指针
 * @param  current_value: 当前测量值（反馈值）
 * @param  dt: 控制周期时间间隔（秒），用于积分和微分计算
 * @retval PID控制器的输出值
 *
 * @note   时间间隔dt必须大于0，否则微分项将为0
 * @note   积分项具有上下限限制，防止积分饱和现象
 * @note   该函数会更新PID控制器内部状态（积分值、上次误差等）
 *
 * @algorithm PID算法公式：
 *            output = Kp*error + Ki*∫error*dt + Kd*d(error)/dt
 *            其中：error = setpoint - current_value
 */
float BalanceControl_PID_Update(PID_Controller_t* pid, float current_value, float dt)
{
    // 计算当前误差：目标值减去当前值
    float error = pid->setpoint - current_value;

    // 比例项计算：Kp * error
    // 比例项提供主要的控制作用，误差越大输出越大
    float proportional = pid->Kp * error;

    // 积分项计算：Ki * ∫error*dt
    // 积分项用于消除稳态误差，累积历史误差
    pid->integral += error * dt;

    // 积分限幅，防止积分饱和导致系统不稳定
    if(pid->integral > pid->max_integral) {
        pid->integral = pid->max_integral;      // 限制积分上限
    } else if(pid->integral < -pid->max_integral) {
        pid->integral = -pid->max_integral;     // 限制积分下限
    }

    // 计算积分项输出
    float integral = pid->Ki * pid->integral;

    // 微分项计算：Kd * d(error)/dt
    // 微分项用于预测误差变化趋势，提高系统稳定性
    float derivative = 0.0f;
    if(dt > 0) {  // 确保时间间隔有效
        derivative = pid->Kd * (error - pid->last_error) / dt;
    }

    // 计算PID总输出：比例项 + 积分项 + 微分项
    pid->output = proportional + integral + derivative;

    // 输出限幅，防止控制量过大导致系统不稳定
    if(pid->output > pid->max_output) {
        pid->output = pid->max_output;      // 限制输出上限
    } else if(pid->output < pid->min_output) {
        pid->output = pid->min_output;      // 限制输出下限
    }

    // 保存当前误差值，供下次微分计算使用
    pid->last_error = error;

    return pid->output;  // 返回PID控制器输出值
}

/**
 * @brief  重置PID控制器状态
 * @details 清除PID控制器的所有历史状态，包括积分累积值、
 *          上次误差值和输出值。通常在系统启动、模式切换
 *          或检测到异常时调用，以避免历史数据影响新的控制过程。
 *
 * @param  pid: 指向要重置的PID控制器结构体的指针
 * @retval 无
 *
 * @note   重置后PID控制器从"干净"状态开始工作
 * @note   建议在系统初始化或重新启动控制时调用
 * @note   不会改变PID参数（Kp、Ki、Kd），只清除状态变量
 */
void BalanceControl_PID_Reset(PID_Controller_t* pid)
{
    pid->integral = 0.0f;     // 清除积分累积值
    pid->last_error = 0.0f;   // 清除上次误差记录
    pid->output = 0.0f;       // 清除输出值
}

/**
 * @brief  平衡控制系统主更新函数
 * @details 这是平衡车控制系统的核心函数，负责周期性地执行以下任务：
 *          1. 控制频率管理，确保固定周期执行
 *          2. 传感器数据更新（MPU6050、编码器、超声波）
 *          3. 障碍物检测和避障处理
 *          4. 多级PID控制计算
 *          5. 电机输出控制
 *
 *          该函数应该在主循环中周期性调用，建议调用频率为100-200Hz。
 *
 * @param  无
 * @retval 无
 *
 * @note   函数内部有频率控制，即使高频调用也只会在达到控制周期时执行
 * @note   控制周期由CONTROL_PERIOD_MS宏定义，通常设置为5-10ms
 * @note   传感器数据更新失败不会影响控制流程的继续执行
 */
void BalanceControl_Update(void)
{
    // 获取当前时间戳（毫秒）
    uint32_t current_time = HAL_GetTick();
    // 计算距离上次控制的时间间隔，转换为秒
    float dt = (current_time - last_control_time) / 1000.0f;

    // 控制频率限制：如果时间间隔小于设定的控制周期，则跳过本次执行
    // 这确保了控制算法以固定频率运行，避免因主循环频率变化影响控制效果
    if(dt < (CONTROL_PERIOD_MS / 1000.0f)) {
        return;  // 未到控制周期，直接返回
    }

    // 更新时间戳，记录本次控制时间
    last_control_time = current_time;

    /*--------------------------------------------------------------------------
                                传感器数据更新
    --------------------------------------------------------------------------*/

    // 更新MPU6050姿态角数据（俯仰角和横滚角）
    // 使用滤波后的角度数据，提高控制稳定性
    MPU_Get_Filtered_Angles(&BalanceState.pitch, &BalanceState.roll);

    // 更新MPU6050角速度数据
    float gx, gy, gz;  // 三轴角速度临时变量
    if(MPU_Get_Gyroscope(&gx, &gy, &gz) == 0) {  // 成功读取角速度数据
        BalanceState.pitch_rate = gy;  // Y轴角速度对应俯仰角速度
        BalanceState.roll_rate = gx;   // X轴角速度对应横滚角速度
    }

    // 更新电机编码器数据，获取当前车轮转速
    MotorEncoder_Update();  // 更新编码器计数和速度计算
    BalanceState.left_speed = MotorEncoder_GetSpeedMPS(&EncoderA);   // 左轮速度（米/秒）
    BalanceState.right_speed = MotorEncoder_GetSpeedMPS(&EncoderB);  // 右轮速度（米/秒）

    // 更新超声波传感器数据，用于障碍物检测
    Ultrasonic_Update();  // 触发超声波测距
    BalanceState.distance_front = Ultrasonic_GetFilteredDistance();  // 获取滤波后的距离

    /*--------------------------------------------------------------------------
                                障碍物检测与避障
    --------------------------------------------------------------------------*/

    // 执行障碍物检测和避障逻辑
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
