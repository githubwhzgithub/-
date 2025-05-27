#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include "main.h"
#include "usart.h"
#include "balance_control.h"

// 蓝牙通信参数
#define BT_UART_HANDLE      &huart2     // HC-05使用UART2
#define BT_RX_BUFFER_SIZE   64          // 接收缓冲区大小
#define BT_TX_BUFFER_SIZE   128         // 发送缓冲区大小
#define BT_COMMAND_MAX_LEN  32          // 最大命令长度

// 蓝牙命令定义
#define BT_CMD_START        "START"     // 开始平衡
#define BT_CMD_STOP         "STOP"      // 停止平衡
#define BT_CMD_FORWARD      "FORWARD"   // 前进
#define BT_CMD_BACKWARD     "BACKWARD"  // 后退
#define BT_CMD_LEFT         "LEFT"      // 左转
#define BT_CMD_RIGHT        "RIGHT"     // 右转
#define BT_CMD_STATUS       "STATUS"    // 获取状态
#define BT_CMD_RESET        "RESET"     // 重置系统
#define BT_CMD_SPEED        "SPEED"     // 设置速度
#define BT_CMD_ANGLE        "ANGLE"     // 设置角度
#define BT_CMD_PID          "PID"       // 设置PID参数

// 蓝牙状态枚举
typedef enum {
    BT_STATE_IDLE = 0,
    BT_STATE_RECEIVING,
    BT_STATE_PROCESSING,
    BT_STATE_SENDING
} BluetoothState_t;

// 蓝牙数据结构体
typedef struct {
    uint8_t rx_buffer[BT_RX_BUFFER_SIZE];   // 接收缓冲区
    uint8_t tx_buffer[BT_TX_BUFFER_SIZE];   // 发送缓冲区
    uint8_t command_buffer[BT_COMMAND_MAX_LEN]; // 命令缓冲区
    uint16_t rx_index;                      // 接收索引
    uint16_t tx_length;                     // 发送长度
    BluetoothState_t state;                 // 当前状态
    uint8_t command_ready;                  // 命令就绪标志
    uint8_t connected;                      // 连接状态
    uint32_t last_heartbeat;                // 上次心跳时间
} Bluetooth_t;

// 函数声明
void Bluetooth_Init(void);
void Bluetooth_Update(void);
void Bluetooth_ProcessCommand(void);
void Bluetooth_SendStatus(void);
void Bluetooth_SendMessage(const char* message);
void Bluetooth_SendFloat(const char* name, float value);
void Bluetooth_SendInt(const char* name, int value);
void Bluetooth_HandleRxData(uint8_t data);
void Bluetooth_RxCallback(void);
BluetoothState_t Bluetooth_GetState(void);
uint8_t Bluetooth_IsConnected(void);

// 蓝牙实例声明
extern Bluetooth_t Bluetooth;

#endif /* __BLUETOOTH_H */