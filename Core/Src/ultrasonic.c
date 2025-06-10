#include "ultrasonic.h"

// 超声波实例定义
Ultrasonic_t Ultrasonic = {
    .distance_cm = 0.0f,
    .state = ULTRASONIC_IDLE,
    .start_capture = 0,
    .end_capture = 0,
    .pulse_width = 0,
    .measurement_valid = 0,
    .filtered_distance = 0.0f,
    .distance_buffer = {0},
    .buffer_index = 0,
    .capture_done = 0
};

// 外部定时器句柄声明
extern TIM_HandleTypeDef htim1;

// 私有函数声明
static void Ultrasonic_DelayUs(uint32_t us);

/**
 * @brief 微秒延时函数
 * @param us: 延时微秒数
 */
static void Ultrasonic_DelayUs(uint32_t us)
{
    uint32_t start = HAL_GetTick() * 1000 + (SysTick->LOAD - SysTick->VAL) / (SystemCoreClock / 1000000);
    uint32_t end = start + us;
    
    while((HAL_GetTick() * 1000 + (SysTick->LOAD - SysTick->VAL) / (SystemCoreClock / 1000000)) < end);
}

/**
 * @brief 初始化超声波模块
 */
void Ultrasonic_Init(void)
{
    // 初始化TRIG引脚为低电平
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
    
    // 初始化距离缓冲区
    for(int i = 0; i < 5; i++) {
        Ultrasonic.distance_buffer[i] = MAX_DISTANCE;
    }
    
    // 启动TIM1输入捕获功能 (通道3和通道4)
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);  // 上升沿捕获
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);  // 下降沿捕获
    
    Ultrasonic.state = ULTRASONIC_IDLE;
}

/**
 * @brief 开始超声波测距
 */
void Ultrasonic_StartMeasurement(void)
{
    if(Ultrasonic.state != ULTRASONIC_IDLE) {
        return; // 如果不是空闲状态，不开始新的测量
    }
    
    Ultrasonic.state = ULTRASONIC_MEASURING;
    Ultrasonic.measurement_valid = 0;
    Ultrasonic.capture_done = 0;
    Ultrasonic.measurement_start_tick = HAL_GetTick(); // 记录测量开始时间
    
    // 重置定时器计数器
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    
    // 发送触发脉冲
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
    Ultrasonic_DelayUs(10); // 10微秒高电平
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
}

/**
 * @brief 更新超声波测量状态
 */
void Ultrasonic_Update(void)
{
    // 只有在空闲状态时才开始新的测量
    if(Ultrasonic.state == ULTRASONIC_IDLE) {
        Ultrasonic_StartMeasurement();
        return; // 开始测量后直接返回，等待下次调用处理结果
    }
    
    // 处理测量中的状态
    if(Ultrasonic.state == ULTRASONIC_MEASURING) {
        // 检查是否捕获完成
        if(Ultrasonic.capture_done) {
            // 计算脉冲宽度对应的时间 (微秒)
            // TIM1预分频器为71，所以定时器频率为72MHz/(71+1) = 1MHz
            // 每个计数值对应1微秒
            float pulse_time_us = (float)Ultrasonic.pulse_width;
            
            // 计算距离: 距离 = (脉冲时间 * 声速) / 2
            // 声速340m/s = 0.034cm/us
            float distance = (pulse_time_us * 0.034f) / 2.0f;
            
            // 检查距离是否在有效范围内
            if(distance >= MIN_DISTANCE && distance <= MAX_DISTANCE) {
                Ultrasonic.distance_cm = distance;
                Ultrasonic.measurement_valid = 1;
                Ultrasonic.state = ULTRASONIC_IDLE; // 测量成功，回到空闲状态
            } else {
                // 距离超出范围，标记为错误
                Ultrasonic.measurement_valid = 0;
                Ultrasonic.state = ULTRASONIC_ERROR;
            }
            
            // 重置捕获完成标志
            Ultrasonic.capture_done = 0;
        }
        else {
            // 检查是否超时（使用结构体中的时间戳）
            uint32_t current_tick = HAL_GetTick();
            uint32_t elapsed_time = current_tick - Ultrasonic.measurement_start_tick;
            
            // 超时时间设置为100ms（根据HC-SR04规格，最大测距时间约为38ms）
            if(elapsed_time > 50) {
                Ultrasonic.state = ULTRASONIC_TIMEOUT;
                Ultrasonic.distance_cm = MAX_DISTANCE;
                Ultrasonic.measurement_valid = 0;
                Ultrasonic.capture_done = 0; // 重置捕获标志
            }
        }
    }
    
    // 错误或超时状态自动恢复到空闲状态
    if(Ultrasonic.state == ULTRASONIC_ERROR || Ultrasonic.state == ULTRASONIC_TIMEOUT) {
        Ultrasonic.state = ULTRASONIC_IDLE;
    }
}

/**
 * @brief 获取测量距离
 * @return 距离值 (cm)
 */
float Ultrasonic_GetDistance(void)
{
    return Ultrasonic.distance_cm;
}

/**
 * @brief 获取滤波后的距离
 * @return 滤波后的距离值 (cm)
 */
float Ultrasonic_GetFilteredDistance(void)
{
    return Ultrasonic.filtered_distance;
}

/**
 * @brief 获取超声波状态
 * @return 当前状态
 */
UltrasonicState_t Ultrasonic_GetState(void)
{
    return Ultrasonic.state;
}

/**
 * @brief TIM1输入捕获回调函数
 * @param htim: 定时器句柄
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM1 && Ultrasonic.state == ULTRASONIC_MEASURING) {
        if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
            // 通道3捕获到上升沿
            Ultrasonic.start_capture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
        }
        else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
            // 通道4捕获到下降沿
            Ultrasonic.end_capture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
            
            // 计算脉冲宽度
            if(Ultrasonic.end_capture >= Ultrasonic.start_capture) {
                Ultrasonic.pulse_width = Ultrasonic.end_capture - Ultrasonic.start_capture; 
            } else {
                // 处理定时器溢出情况
                Ultrasonic.pulse_width = (0xFFFF - Ultrasonic.start_capture) + Ultrasonic.end_capture + 1;
            }

            // 标记捕获完成
            Ultrasonic.capture_done = 1;
        }
    }
}
