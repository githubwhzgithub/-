#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "usart.h"

// HC-SR04超声波模块引脚定义
#define TRIG_PORT       GPIOA
#define TRIG_PIN        GPIO_PIN_12
#define ECHO_PORT       GPIOA
#define ECHO_PIN        GPIO_PIN_10

// 超声波测距参数
#define SOUND_SPEED     340.0f      // 声速 (m/s)
#define MAX_DISTANCE    400.0f      // 最大测距距离 (cm)
#define MIN_DISTANCE    2.0f        // 最小测距距离 (cm)
#define TIMEOUT_US      30000       // 超时时间 (微秒)

// 超声波状态枚举
typedef enum {
    ULTRASONIC_IDLE = 0,
    ULTRASONIC_MEASURING,
    ULTRASONIC_TIMEOUT,
    ULTRASONIC_ERROR
} UltrasonicState_t;

// 超声波结构体
typedef struct {
    float distance_cm;              // 测量距离 (cm)
    UltrasonicState_t state;        // 当前状态
    uint32_t start_capture;         // 上升沿捕获值
    uint32_t end_capture;           // 下降沿捕获值
    uint32_t pulse_width;           // 脉冲宽度 (定时器计数值)
    uint8_t measurement_valid;      // 测量有效标志
    float filtered_distance;        // 滤波后距离
    float distance_buffer[5];       // 距离缓冲区用于滤波
    uint8_t buffer_index;           // 缓冲区索引
    uint8_t capture_done;           // 捕获完成标志
    uint32_t measurement_start_tick; // 测量开始时间戳
} Ultrasonic_t;

// 函数声明
void Ultrasonic_Init(void);
void Ultrasonic_StartMeasurement(void);
float Ultrasonic_GetDistance(void);
float Ultrasonic_GetFilteredDistance(void);
UltrasonicState_t Ultrasonic_GetState(void);
void Ultrasonic_Update(void);
void Ultrasonic_ECHO_IRQHandler(void);  // 保留兼容性
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);  // TIM1输入捕获回调

// 超声波实例声明
extern Ultrasonic_t Ultrasonic;

#endif /* __ULTRASONIC_H */
