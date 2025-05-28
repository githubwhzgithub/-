/**
 * @file    bluetooth.h
 * @brief   蓝牙通信模块头文件
 * @details 该文件定义了蓝牙通信模块的所有接口函数、数据结构和配置参数。
 *          蓝牙模块用于接收手机APP或其他蓝牙设备发送的控制命令，
 *          实现对平衡车的无线遥控功能。支持前进、后退、左转、右转、
 *          停止、调速等基本控制命令，同时可以向上位机发送平衡车的
 *          状态信息用于监控和调试。
 *
 * @author  STM32平衡车项目组
 * @version 1.0
 * @date    2024
 *
 * @note    使用UART接口与蓝牙模块通信，支持中断接收和DMA传输
 */

#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <string.h>

/*==============================================================================
                                UART通信参数配置
==============================================================================*/

/**
 * @brief UART通信基础参数
 * @note  这些参数定义了与蓝牙模块通信的串口配置
 */
#define BT_UART_BAUDRATE    9600                // 波特率：9600bps，适合蓝牙模块的标准速率
#define BT_UART_DATABITS    8                   // 数据位：8位数据位
#define BT_UART_STOPBITS    1                   // 停止位：1位停止位
#define BT_UART_PARITY      UART_PARITY_NONE    // 校验位：无校验
#define BT_UART_FLOWCONTROL UART_HWCONTROL_NONE // 流控制：无硬件流控制

/*==============================================================================
                                缓冲区大小定义
==============================================================================*/

/**
 * @brief 通信缓冲区大小配置
 * @note  根据实际应用需求配置缓冲区大小，确保能够处理完整的命令和数据
 */
#define BT_RX_BUFFER_SIZE   128    // 接收缓冲区大小：存储从蓝牙接收的数据
#define BT_TX_BUFFER_SIZE   128    // 发送缓冲区大小：存储待发送到蓝牙的数据
#define BT_CMD_BUFFER_SIZE  32     // 命令缓冲区大小：存储解析后的完整命令

/*==============================================================================
                                控制命令定义
==============================================================================*/

/**
 * @brief 蓝牙遥控命令字符定义
 * @details 定义了手机APP或遥控器发送的各种控制命令对应的字符。
 *          这些命令字符应与上位机软件保持一致。
 */
#define BT_CMD_FORWARD      'W'    // 前进命令：控制平衡车向前移动
#define BT_CMD_BACKWARD     'S'    // 后退命令：控制平衡车向后移动
#define BT_CMD_LEFT         'A'    // 左转命令：控制平衡车向左转向
#define BT_CMD_RIGHT        'D'    // 右转命令：控制平衡车向右转向
#define BT_CMD_STOP         'X'    // 停止命令：立即停止平衡车运动
#define BT_CMD_SPEED_UP     '+'    // 加速命令：增加平衡车运动速度
#define BT_CMD_SPEED_DOWN   '-'    // 减速命令：降低平衡车运动速度
#define BT_CMD_STATUS       '?'    // 状态查询：请求平衡车发送当前状态信息

/*==============================================================================
                                数据类型定义
==============================================================================*/

/**
 * @brief 蓝牙连接状态枚举
 * @details 定义了蓝牙模块可能的工作状态，用于状态机管理和错误处理
 */
typedef enum {
    BT_STATE_DISCONNECTED = 0,    // 断开连接：蓝牙模块未连接或连接丢失
    BT_STATE_CONNECTED,           // 已连接：蓝牙模块已成功连接到设备
    BT_STATE_RECEIVING,           // 接收中：正在接收数据，用于流控制
    BT_STATE_TRANSMITTING,        // 发送中：正在发送数据，用于流控制
    BT_STATE_ERROR                // 错误状态：通信出现错误，需要重新初始化
} BluetoothState_t;

/**
 * @brief 蓝牙通信数据结构
 * @details 包含蓝牙通信所需的所有缓冲区、索引和状态信息。
 *          该结构体管理整个蓝牙通信的数据流和状态。
 */
typedef struct {
    uint8_t rx_buffer[BT_RX_BUFFER_SIZE];    // 接收缓冲区：存储从蓝牙模块接收的原始数据
    uint8_t tx_buffer[BT_TX_BUFFER_SIZE];    // 发送缓冲区：存储待发送到蓝牙模块的数据
    uint8_t cmd_buffer[BT_CMD_BUFFER_SIZE];  // 命令缓冲区：存储解析完成的完整命令
    uint16_t rx_index;                       // 接收索引：当前接收缓冲区的写入位置
    uint16_t tx_index;                       // 发送索引：当前发送缓冲区的读取位置
    uint16_t cmd_length;                     // 命令长度：当前命令缓冲区中有效数据的长度
    BluetoothState_t state;                  // 连接状态：当前蓝牙模块的工作状态
    uint8_t connected;                       // 连接标志：1-已连接，0-未连接
    uint32_t last_receive_time;              // 最后接收时间：用于连接超时检测(ms)
} BluetoothData_t;

/*==============================================================================
                                函数声明
==============================================================================*/

/**
 * @brief 初始化蓝牙通信模块
 * @note  配置UART参数，初始化缓冲区，启动接收中断
 */
void Bluetooth_Init(void);

/**
 * @brief 发送数据到蓝牙模块
 * @param data 待发送的数据指针
 * @param length 数据长度
 * @note  使用DMA或中断方式发送数据，非阻塞式
 */
void Bluetooth_SendData(uint8_t* data, uint16_t length);

/**
 * @brief 发送字符串到蓝牙模块
 * @param str 待发送的字符串指针
 * @note  自动计算字符串长度并发送，以'\0'结尾
 */
void Bluetooth_SendString(char* str);

/**
 * @brief 处理接收到的蓝牙命令
 * @note  解析接收缓冲区中的数据，提取有效命令并执行相应动作
 */
void Bluetooth_ProcessCommand(void);

/**
 * @brief 蓝牙UART中断处理函数
 * @note  在UART中断服务程序中调用，处理接收完成和发送完成事件
 */
void Bluetooth_IRQHandler(void);

/**
 * @brief 获取蓝牙模块当前状态
 * @return 当前蓝牙连接状态
 * @note  用于外部模块查询蓝牙模块的工作状态
 */
BluetoothState_t Bluetooth_GetState(void);

/**
 * @brief 检查蓝牙是否已连接
 * @return 1-已连接，0-未连接
 * @note  简化的连接状态查询函数
 */
uint8_t Bluetooth_IsConnected(void);

/**
 * @brief 发送平衡车状态信息
 * @note  将平衡车的角度、速度、电池电压等信息发送给上位机
 */
void Bluetooth_SendStatus(void);

/**
 * @brief 清空所有通信缓冲区
 * @note  用于错误恢复或重新初始化时清理缓冲区数据
 */
void Bluetooth_ClearBuffer(void);

/*==============================================================================
                                全局变量声明
==============================================================================*/

/**
 * @brief 蓝牙通信数据实例
 * @note  包含所有蓝牙通信相关的缓冲区和状态信息
 */
extern BluetoothData_t BluetoothData;

/**
 * @brief 蓝牙UART句柄
 * @note  HAL库UART句柄，用于底层UART通信操作
 */
extern UART_HandleTypeDef huart_bluetooth;

#endif /* __BLUETOOTH_H */
