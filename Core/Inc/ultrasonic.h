/**
 * @file    ultrasonic.h
 * @brief   超声波测距传感器模块头文件
 * @details 该文件定义了超声波测距传感器的接口函数、数据结构和配置参数。
 *          超声波传感器用于检测平衡车前方的障碍物距离，为避障功能
 *          提供距离信息。模块采用HC-SR04或类似的超声波传感器，
 *          通过发送超声波脉冲并测量回波时间来计算距离。集成了
 *          滑动平均滤波算法，提高测距精度和稳定性。
 *
 * @author  STM32平衡车项目组
 * @version 1.0
 * @date    2024
 *
 * @note    使用GPIO和定时器实现，支持中断方式测量和滤波处理
 */

#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/*==============================================================================
                                GPIO引脚定义
==============================================================================*/

/**
 * @brief 超声波传感器GPIO引脚配置
 * @note  TRIG引脚用于发送触发脉冲，ECHO引脚用于接收回波信号
 */
#define TRIG_PIN        GPIO_PIN_8      // 触发引脚：发送10us高电平触发测距
#define TRIG_PORT       GPIOB           // 触发引脚端口
#define ECHO_PIN        GPIO_PIN_9      // 回波引脚：接收超声波反射信号
#define ECHO_PORT       GPIOB           // 回波引脚端口

/*==============================================================================
                                传感器参数定义
==============================================================================*/

/**
 * @brief 超声波传感器物理参数和限制
 * @details 定义了超声波测距的物理参数和系统限制，用于距离计算和有效性检查
 */
#define SOUND_SPEED     340.0f      // 声速(m/s)：20°C时空气中的声速
#define TIMEOUT_US      30000       // 超时时间(us)：最大等待回波时间，对应约5m距离
#define MIN_DISTANCE    2.0f        // 最小测量距离(cm)：传感器盲区
#define MAX_DISTANCE    400.0f      // 最大测量距离(cm)：传感器有效测量范围
#define FILTER_SIZE     5           // 滤波器大小：滑动平均滤波的采样点数

/*==============================================================================
                                数据类型定义
==============================================================================*/

/**
 * @brief 超声波传感器状态枚举
 * @details 定义了超声波测距过程中的各种状态，用于状态机管理
 */
typedef enum {
    ULTRASONIC_IDLE = 0,        // 空闲状态：等待开始新的测量
    ULTRASONIC_TRIGGER,         // 触发状态：正在发送触发脉冲
    ULTRASONIC_WAITING,         // 等待状态：等待回波信号上升沿
    ULTRASONIC_MEASURING,       // 测量状态：正在测量回波持续时间
    ULTRASONIC_TIMEOUT          // 超时状态：测量超时，可能无障碍物
} UltrasonicState_t;

/**
 * @brief 超声波传感器数据结构
 * @details 包含超声波测距的所有状态信息和测量数据，
 *          管理整个测距过程和数据滤波。
 */
typedef struct {
    float distance;                         // 当前测量距离(cm)：最新一次测量结果
    UltrasonicState_t state;                // 当前工作状态：测距状态机的当前状态
    uint32_t start_time;                    // 回波开始时间(us)：ECHO信号上升沿时间
    uint32_t end_time;                      // 回波结束时间(us)：ECHO信号下降沿时间
    float filtered_distance;                // 滤波后距离(cm)：经过滑动平均滤波的距离
    float distance_buffer[FILTER_SIZE];     // 距离缓冲区：存储最近几次测量结果
    uint8_t buffer_index;                   // 缓冲区索引：当前写入位置的循环索引
} UltrasonicData_t;

/*==============================================================================
                                函数声明
==============================================================================*/

/**
 * @brief 初始化超声波传感器
 * @note  配置GPIO引脚、定时器和中断，初始化数据结构
 */
void Ultrasonic_Init(void);

/**
 * @brief 获取当前测量距离
 * @return 当前距离值(cm)
 * @note  返回最新一次测量的原始距离，未经滤波
 */
float Ultrasonic_GetDistance(void);

/**
 * @brief 开始一次超声波测距
 * @note  发送触发脉冲，启动测距过程，非阻塞式
 */
void Ultrasonic_StartMeasurement(void);

/**
 * @brief 更新超声波传感器状态
 * @note  处理状态机转换和数据更新，应定期调用
 */
void Ultrasonic_Update(void);

/**
 * @brief 超声波传感器中断处理函数
 * @note  在ECHO引脚中断中调用，处理上升沿和下降沿事件
 */
void Ultrasonic_IRQHandler(void);

/**
 * @brief 获取滤波后的距离
 * @return 滤波后的距离值(cm)
 * @note  返回经过滑动平均滤波的稳定距离值
 */
float Ultrasonic_GetFilteredDistance(void);

/**
 * @brief 获取超声波传感器当前状态
 * @return 当前工作状态
 * @note  用于外部模块查询传感器的工作状态
 */
UltrasonicState_t Ultrasonic_GetState(void);

/*==============================================================================
                                全局变量声明
==============================================================================*/

/**
 * @brief 超声波传感器数据实例
 * @note  包含所有超声波测距相关的状态和测量数据
 */
extern UltrasonicData_t UltrasonicData;

#endif /* __ULTRASONIC_H */
