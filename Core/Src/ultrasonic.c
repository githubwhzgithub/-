#include "ultrasonic.h"

// 超声波实例定义
Ultrasonic_t Ultrasonic = {
    .distance_cm = 0.0f,
    .state = ULTRASONIC_IDLE,
    .start_time = 0,
    .end_time = 0,
    .measurement_valid = 0,
    .filtered_distance = 0.0f,
    .distance_buffer = {0},
    .buffer_index = 0
};

// 私有函数声明
static void Ultrasonic_DelayUs(uint32_t us);
static float Ultrasonic_MedianFilter(float new_value);

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
 * @brief 中值滤波函数
 * @param new_value: 新的测量值
 * @return 滤波后的值
 */
static float Ultrasonic_MedianFilter(float new_value)
{
    // 将新值添加到缓冲区
    Ultrasonic.distance_buffer[Ultrasonic.buffer_index] = new_value;
    Ultrasonic.buffer_index = (Ultrasonic.buffer_index + 1) % 5;
    
    // 复制缓冲区用于排序
    float temp_buffer[5];
    for(int i = 0; i < 5; i++) {
        temp_buffer[i] = Ultrasonic.distance_buffer[i];
    }
    
    // 简单冒泡排序
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4 - i; j++) {
            if(temp_buffer[j] > temp_buffer[j + 1]) {
                float temp = temp_buffer[j];
                temp_buffer[j] = temp_buffer[j + 1];
                temp_buffer[j + 1] = temp;
            }
        }
    }
    
    // 返回中值
    return temp_buffer[2];
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
    
    // 发送触发脉冲
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
    Ultrasonic_DelayUs(10); // 10微秒高电平
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
    
    // 记录开始时间
    Ultrasonic.start_time = HAL_GetTick() * 1000 + (SysTick->LOAD - SysTick->VAL) / (SystemCoreClock / 1000000);
}

/**
 * @brief 更新超声波测量状态
 */
void Ultrasonic_Update(void)
{
    if(Ultrasonic.state == ULTRASONIC_MEASURING) {
        uint32_t current_time = HAL_GetTick() * 1000 + (SysTick->LOAD - SysTick->VAL) / (SystemCoreClock / 1000000);
        
        // 检查是否超时
        if((current_time - Ultrasonic.start_time) > TIMEOUT_US) {
            Ultrasonic.state = ULTRASONIC_TIMEOUT;
            Ultrasonic.distance_cm = MAX_DISTANCE; // 超时时设置为最大距离
        }
        
        // 检查ECHO引脚状态
        if(HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_SET && Ultrasonic.start_time != 0) {
            // ECHO上升沿，开始计时
            Ultrasonic.start_time = current_time;
        }
        else if(HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_RESET && Ultrasonic.start_time != 0) {
            // ECHO下降沿，结束计时
            Ultrasonic.end_time = current_time;
            
            // 计算距离
            uint32_t pulse_duration = Ultrasonic.end_time - Ultrasonic.start_time;
            float distance = (pulse_duration * SOUND_SPEED) / (2.0f * 10000.0f); // 转换为cm
            
            // 检查距离是否在有效范围内
            if(distance >= MIN_DISTANCE && distance <= MAX_DISTANCE) {
                Ultrasonic.distance_cm = distance;
                Ultrasonic.filtered_distance = Ultrasonic_MedianFilter(distance);
                Ultrasonic.measurement_valid = 1;
            } else {
                Ultrasonic.state = ULTRASONIC_ERROR;
            }
            
            Ultrasonic.state = ULTRASONIC_IDLE;
        }
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
 * @brief ECHO引脚中断处理函数
 */
void Ultrasonic_ECHO_IRQHandler(void)
{
    uint32_t current_time = HAL_GetTick() * 1000 + (SysTick->LOAD - SysTick->VAL) / (SystemCoreClock / 1000000);
    
    if(Ultrasonic.state == ULTRASONIC_MEASURING) {
        if(HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_SET) {
            // ECHO上升沿
            Ultrasonic.start_time = current_time;
        } else {
            // ECHO下降沿
            Ultrasonic.end_time = current_time;
            
            // 计算距离
            uint32_t pulse_duration = Ultrasonic.end_time - Ultrasonic.start_time;
            float distance = (pulse_duration * SOUND_SPEED) / (2.0f * 10000.0f); // 转换为cm
            
            // 检查距离是否在有效范围内
            if(distance >= MIN_DISTANCE && distance <= MAX_DISTANCE) {
                Ultrasonic.distance_cm = distance;
                Ultrasonic.filtered_distance = Ultrasonic_MedianFilter(distance);
                Ultrasonic.measurement_valid = 1;
                Ultrasonic.state = ULTRASONIC_IDLE;
            } else {
                Ultrasonic.state = ULTRASONIC_ERROR;
            }
        }
    }
}