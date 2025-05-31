#ifndef __MOTORENCODER_H
#define __MOTORENCODER_H

#include "main.h"
#include "tim.h"

// 编码器定时器定义
#define ENCODER_A_TIMER     &htim1  // 电机A编码器使用TIM1
#define ENCODER_B_TIMER     &htim2  // 电机B编码器使用TIM2

// 编码器参数
#define ENCODER_RESOLUTION  1024    // 编码器分辨率 (每转脉冲数)
#define WHEEL_DIAMETER      65.0f   // 轮子直径 (mm)
#define GEAR_RATIO          30.0f   // 减速比
#define ENCODER_SAMPLE_TIME 10      // 编码器采样时间 (ms)

// 编码器结构体
typedef struct {
    TIM_HandleTypeDef* timer;
    int32_t last_count;         // 上次计数值
    int32_t total_count;        // 总计数值
    float speed_rpm;            // 转速 (RPM)
    float speed_mps;            // 线速度 (m/s)
    float distance_mm;          // 累计距离 (mm)
    uint32_t last_update_time;  // 上次更新时间
} Encoder_t;

// 函数声明
void MotorEncoder_Init(void);
void MotorEncoder_Update(void);
void MotorEncoder_Reset(void);
float MotorEncoder_GetSpeedRPM(Encoder_t* encoder);
float MotorEncoder_GetSpeedMPS(Encoder_t* encoder);
float MotorEncoder_GetDistance(Encoder_t* encoder);
int32_t MotorEncoder_GetCount(Encoder_t* encoder);
void MotorEncoder_ResetDistance(Encoder_t* encoder);

// 编码器实例声明
extern Encoder_t EncoderA;
extern Encoder_t EncoderB;

#endif /* __MOTORENCODER_H */
