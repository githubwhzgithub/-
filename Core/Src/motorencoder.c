#include "motorencoder.h"
#include <math.h>
#define M_PI 3.14159265358979323846

// 编码器实例定义
Encoder_t EncoderA = {
    .timer = ENCODER_A_TIMER,
    .last_count = 0,
    .total_count = 0,
    .speed_rpm = 0.0f,
    .speed_mps = 0.0f,
    .distance_mm = 0.0f,
    .last_update_time = 0
};

Encoder_t EncoderB = {
    .timer = ENCODER_B_TIMER,
    .last_count = 0,
    .total_count = 0,
    .speed_rpm = 0.0f,
    .speed_mps = 0.0f,
    .distance_mm = 0.0f,
    .last_update_time = 0
};

/**
 * @brief 初始化电机编码器
 */
void MotorEncoder_Init(void)
{
    // 启动编码器定时器
    HAL_TIM_Encoder_Start(ENCODER_A_TIMER, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(ENCODER_B_TIMER, TIM_CHANNEL_ALL);

    // 设置初始计数值为0
    __HAL_TIM_SET_COUNTER(ENCODER_A_TIMER, 0);
    __HAL_TIM_SET_COUNTER(ENCODER_B_TIMER, 0);

    // 初始化时间戳
    EncoderA.last_update_time = HAL_GetTick();
    EncoderB.last_update_time = HAL_GetTick();
}

/**
 * @brief 更新编码器数据
 */
void MotorEncoder_Update(void)
{
    uint32_t current_time = HAL_GetTick();

    // 更新编码器A
    int32_t current_count_A = (int32_t)__HAL_TIM_GET_COUNTER(ENCODER_A_TIMER);
    int32_t delta_count_A = current_count_A - EncoderA.last_count;

    
    // 处理定时器溢出
    if(delta_count_A > 32767) {
        delta_count_A -= 65536;
    } else if(delta_count_A < -32767) {
        delta_count_A += 65536;
    }


    EncoderA.total_count += delta_count_A;
    EncoderA.last_count = current_count_A;

    // 计算时间差
    float dt_A = (current_time - EncoderA.last_update_time) / 1000.0f; // 转换为秒
    if(dt_A > 0) {
        // 计算转速 (RPM)
        float pulses_per_rev = ENCODER_RESOLUTION * 4; // 4倍频
        EncoderA.speed_rpm = (delta_count_A / pulses_per_rev) * (60.0f / dt_A) / GEAR_RATIO;

        // 计算线速度 (m/s)
        float wheel_circumference = M_PI * WHEEL_DIAMETER / 1000.0f; // 转换为米
        EncoderA.speed_mps = EncoderA.speed_rpm * wheel_circumference / 60.0f;

        // 计算累计距离 (mm)
        float distance_delta = (delta_count_A / pulses_per_rev) * (M_PI * WHEEL_DIAMETER) / GEAR_RATIO;
        EncoderA.distance_mm += distance_delta;

        EncoderA.last_update_time = current_time;
    }

    // 更新编码器B
    int32_t current_count_B = (int32_t)__HAL_TIM_GET_COUNTER(ENCODER_B_TIMER);
    int32_t delta_count_B = current_count_B - EncoderB.last_count;

    // 处理定时器溢出
    if(delta_count_B > 32767) {
        delta_count_B -= 65536;
    } else if(delta_count_B < -32767) {
        delta_count_B += 65536;
    }

    delta_count_B = -delta_count_B;

    EncoderB.total_count += delta_count_B;
    EncoderB.last_count = current_count_B;

    // 计算时间差
    float dt_B = (current_time - EncoderB.last_update_time) / 1000.0f; // 转换为秒
    if(dt_B > 0) {
        // 计算转速 (RPM)
        float pulses_per_rev = ENCODER_RESOLUTION * 4; // 4倍频
        EncoderB.speed_rpm = (delta_count_B / pulses_per_rev) * (60.0f / dt_B) / GEAR_RATIO;

        // 计算线速度 (m/s)
        float wheel_circumference = M_PI * WHEEL_DIAMETER / 1000.0f; // 转换为米
        EncoderB.speed_mps = EncoderB.speed_rpm * wheel_circumference / 60.0f;

        // 计算累计距离 (mm)
        float distance_delta = (delta_count_B / pulses_per_rev) * (M_PI * WHEEL_DIAMETER) / GEAR_RATIO;
        EncoderB.distance_mm += distance_delta;

        EncoderB.last_update_time = current_time;
    }
}

/**
 * @brief 重置编码器数据
 */
void MotorEncoder_Reset(void)
{
    // 重置计数器
    __HAL_TIM_SET_COUNTER(ENCODER_A_TIMER, 0);
    __HAL_TIM_SET_COUNTER(ENCODER_B_TIMER, 0);

    // 重置编码器A数据
    EncoderA.last_count = 0;
    EncoderA.total_count = 0;
    EncoderA.speed_rpm = 0.0f;
    EncoderA.speed_mps = 0.0f;
    EncoderA.distance_mm = 0.0f;
    EncoderA.last_update_time = HAL_GetTick();

    // 重置编码器B数据
    EncoderB.last_count = 0;
    EncoderB.total_count = 0;
    EncoderB.speed_rpm = 0.0f;
    EncoderB.speed_mps = 0.0f;
    EncoderB.distance_mm = 0.0f;
    EncoderB.last_update_time = HAL_GetTick();
}

/**
 * @brief 获取编码器转速 (RPM)
 * @param encoder: 编码器结构体指针
 * @return 转速值
 */
float MotorEncoder_GetSpeedRPM(Encoder_t* encoder)
{
    return encoder->speed_rpm;
}

/**
 * @brief 获取编码器线速度 (m/s)
 * @param encoder: 编码器结构体指针
 * @return 线速度值
 */
float MotorEncoder_GetSpeedMPS(Encoder_t* encoder)
{
    return encoder->speed_mps;
}

/**
 * @brief 获取编码器累计距离 (mm)
 * @param encoder: 编码器结构体指针
 * @return 累计距离值
 */
float MotorEncoder_GetDistance(Encoder_t* encoder)
{
    return encoder->distance_mm;
}

/**
 * @brief 获取编码器计数值
 * @param encoder: 编码器结构体指针
 * @return 总计数值
 */
int32_t MotorEncoder_GetCount(Encoder_t* encoder)
{
    return encoder->total_count;
}

/**
 * @brief 重置编码器距离
 * @param encoder: 编码器结构体指针
 */
void MotorEncoder_ResetDistance(Encoder_t* encoder)
{
    encoder->distance_mm = 0.0f;
    encoder->total_count = 0;
}
