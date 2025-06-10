#include "bluetooth.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "motor_test.h"

// 蓝牙实例定义
Bluetooth_t Bluetooth = {
    .rx_buffer = {0},
    .tx_buffer = {0},
    .command_buffer = {0},
    .rx_index = 0,
    .tx_length = 0,
    .state = BT_STATE_IDLE,
    .command_ready = 0,
    .connected = 0,
    .last_heartbeat = 0
};

// 私有函数声明
static void Bluetooth_ExecuteCommand(const char* cmd, const char* param);

/**
 * @brief 初始化蓝牙模块
 */
void Bluetooth_Init(void)
{
    // 初始化状态
    Bluetooth.state = BT_STATE_IDLE;
    Bluetooth.rx_index = 0;
    Bluetooth.command_ready = 0;
    Bluetooth.connected = 0;
    Bluetooth.last_heartbeat = HAL_GetTick();
    
    // 清空缓冲区
    memset(Bluetooth.rx_buffer, 0, BT_RX_BUFFER_SIZE);
    memset(Bluetooth.tx_buffer, 0, BT_TX_BUFFER_SIZE);
    memset(Bluetooth.command_buffer, 0, BT_COMMAND_MAX_LEN);

    // 启动UART接收中断 - 注意：实际接收在main.c的HAL_UART_RxCpltCallback中处理
    // 这里只是为了兼容原有的GO命令等待逻辑
    uint8_t temp_rx_byte;
    HAL_UART_Receive_IT(BT_UART_HANDLE, &temp_rx_byte, 1);

    while(1){
        // 等待蓝牙模块连接
        if(strncmp((char*)Bluetooth.rx_buffer, BT_CMD_GO, 2) == 0) {
            Bluetooth.connected = 1;
            memset(Bluetooth.rx_buffer, 0, BT_RX_BUFFER_SIZE);
            break; // 连接成功，跳出循环
        }
        HAL_Delay(10);
    }
    
}

/**
 * @brief 蓝牙模块更新函数
 */
void Bluetooth_Update(void)
{
    uint32_t current_time = HAL_GetTick();

    // 处理接收到的命令
    if(Bluetooth.command_ready) {
        Bluetooth_ProcessCommand();
        Bluetooth.command_ready = 0;
    }
    
}

/**
 * @brief 处理接收到的数据
 * @param data: 接收到的字节
 */
void Bluetooth_HandleRxData(uint8_t data)
{
    Bluetooth.last_heartbeat = HAL_GetTick();
    Bluetooth.connected = 1;

    // 检查是否为命令结束符
    if(data == '\r' || data == '\n') {
        if(Bluetooth.rx_index > 0) {
            // 复制命令到命令缓冲区
            memcpy(Bluetooth.command_buffer, Bluetooth.rx_buffer, Bluetooth.rx_index);
            Bluetooth.command_buffer[Bluetooth.rx_index] = '\0';
            Bluetooth.command_ready = 1;
            Bluetooth.rx_index = 0;
        }
    }
    // 检查缓冲区是否溢出
    else if(Bluetooth.rx_index < BT_RX_BUFFER_SIZE - 1) {
        Bluetooth.rx_buffer[Bluetooth.rx_index++] = data;
    }
    else {
        // 缓冲区溢出，重置
        Bluetooth.rx_index = 0;
    }
}

/**
 * @brief UART接收完成回调函数
 */
void Bluetooth_RxCallback(void)
{
    // 这个函数由main.c中的HAL_UART_RxCpltCallback调用
    // 重新启动接收在main.c中处理
}

/**
 * @brief 处理命令
 */
void Bluetooth_ProcessCommand(void)
{
    char* cmd = (char*)Bluetooth.command_buffer;
    char* param = NULL;

    // 查找空格分隔符
    char* space_pos = strchr(cmd, ' ');
    if(space_pos != NULL) {
        *space_pos = '\0';
        param = space_pos + 1;
    }

    // 执行命令
    Bluetooth_ExecuteCommand(cmd, param);
}

/**
 * @brief 执行具体命令
 * @param cmd: 命令字符串
 * @param param: 参数字符串
 */
static void Bluetooth_ExecuteCommand(const char* cmd, const char* param)
{
    BalanceState_t* state = BalanceControl_GetState();

    if(strcmp(cmd, BT_CMD_START) == 0) {
        BalanceControl_Enable(1);
        Bluetooth_SendMessage("Balance Started\r\n");
    }
    else if(strcmp(cmd, BT_CMD_STOP) == 0) {
        BalanceControl_Enable(0);
        Bluetooth_SendMessage("Balance Stopped\r\n");
    }
    else if(strcmp(cmd, BT_CMD_FORWARD) == 0) {
        float speed = 0.2f; // 默认速度
        if(param != NULL) {
            speed = atof(param);
        }
        BalanceControl_SetTargetSpeed(speed);
        Bluetooth_SendMessage("Moving Forward\r\n");
    }
    else if(strcmp(cmd, BT_CMD_BACKWARD) == 0) {
        float speed = -0.2f; // 默认速度
        if(param != NULL) {
            speed = -atof(param);
        }
        BalanceControl_SetTargetSpeed(speed);
        Bluetooth_SendMessage("Moving Backward\r\n");
    }
    else if(strcmp(cmd, BT_CMD_LEFT) == 0) {
        float L_YawRate = 30.0f; // 默认角速度
        if(param!= NULL) {
            L_YawRate = abs(atof(param));    
        }
        BalanceControl_SetTargetYawRate(L_YawRate);  // 设置左转角速度目标
        Bluetooth_SendMessage("Turning Left\r\n");
    }
    else if(strcmp(cmd, BT_CMD_RIGHT) == 0) {
        float R_YawRate = -30.0f; // 默认角速度
        if(param!= NULL) {
            R_YawRate = -abs(atof(param));
        }
        BalanceControl_SetTargetYawRate(R_YawRate);   // 设置右转角速度目标
        Bluetooth_SendMessage("Turning Right\r\n");
    }
    else if(strcmp(cmd, BT_CMD_STATUS) == 0) {
        Bluetooth_SendStatus();
    }
    else if(strcmp(cmd, BT_CMD_RESET) == 0) {
        BalanceControl_Enable(0);
        BalanceControl_SetTargetSpeed(0);
        BalanceControl_SetTargetAngle(0);
        BalanceControl_SetTargetYawRate(0);
        MotorEncoder_Reset();
        Bluetooth_SendMessage("System Reset\r\n");
    }
    else if(strcmp(cmd, BT_CMD_SPEED) == 0) {
        if(param != NULL) {
            float speed = atof(param);
            BalanceControl_SetTargetSpeed(speed);
            snprintf((char*)Bluetooth.tx_buffer, BT_TX_BUFFER_SIZE, "Speed set to %.2f\r\n", speed);
            Bluetooth_SendMessage((char*)Bluetooth.tx_buffer);
        }
    }
    else if(strcmp(cmd, BT_CMD_ANGLE) == 0) {
        if(param != NULL) {
            float angle = atof(param);
            BalanceControl_SetTargetAngle(angle);
            snprintf((char*)Bluetooth.tx_buffer, BT_TX_BUFFER_SIZE, "Angle set to %.2f\r\n", angle);
            Bluetooth_SendMessage((char*)Bluetooth.tx_buffer);
        }
    }
    else if(strcmp(cmd, BT_CMD_PID) == 0) {
        // PID参数设置 (格式: PID KP KI KD)
        if(param != NULL) {
            float kp, ki, kd;
            if(sscanf(param, "%f %f %f", &kp, &ki, &kd) == 3) {
                AnglePID.Kp = kp;
                AnglePID.Ki = ki;
                AnglePID.Kd = kd;
                snprintf((char*)Bluetooth.tx_buffer, BT_TX_BUFFER_SIZE, "PID set: Kp=%.2f Ki=%.2f Kd=%.2f\r\n", kp, ki, kd);
                Bluetooth_SendMessage((char*)Bluetooth.tx_buffer);
            }
        }
    }
    else if(strcmp(cmd, BT_CMD_VISION) == 0) {
        // 视觉模式设置 (格式: VISION 0/1/2)
        if(param != NULL) {
            int mode = atoi(param);
            if(mode >= 0 && mode <= 2) {
                BalanceControl_SetVisionMode((uint8_t)mode);
                const char* mode_names[] = {"OFF", "LINE_TRACKING", "OBJECT_TRACKING"};
                snprintf((char*)Bluetooth.tx_buffer, BT_TX_BUFFER_SIZE, "Vision mode set to %s\r\n", mode_names[mode]);
                Bluetooth_SendMessage((char*)Bluetooth.tx_buffer);
            } else {
                Bluetooth_SendMessage("Invalid vision mode (0-2)\r\n");
            }
        } else {
            Bluetooth_SendMessage("Vision mode parameter required\r\n");
        }
    }
    else if(strcmp(cmd, BT_CMD_LINE) == 0) {
        // 启动循迹模式
        BalanceControl_SetVisionMode(1);
        Bluetooth_SendMessage("Line tracking mode enabled\r\n");
    }
    else if(strcmp(cmd, BT_CMD_TRACK) == 0) {
        // 启动物体追踪模式
        BalanceControl_SetVisionMode(2);
        Bluetooth_SendMessage("Object tracking mode enabled\r\n");
    }
    else if(strcmp(cmd, BT_CMD_VISION_OFF) == 0) {
        // 关闭视觉模式
        BalanceControl_SetVisionMode(0);
        Bluetooth_SendMessage("Vision mode disabled\r\n");
    }
    else if(strcmp(cmd, BT_CMD_COLOR) == 0) {
        // 设置追踪颜色
        // 参数格式: COLOR RED 或 COLOR GREEN 或 COLOR BLUE
        if(param != NULL) {
            char color_cmd[32];
            snprintf(color_cmd, sizeof(color_cmd), "COLOR_%s\r\n", param);
            // 通过UART2发送颜色设置命令给vision_tracker.py
            HAL_UART_Transmit(&huart2, (uint8_t*)color_cmd, strlen(color_cmd), 100);
            
            char response[64];
            snprintf(response, sizeof(response), "Color set to %s\r\n", param);
            Bluetooth_SendMessage(response);
        } else {
            Bluetooth_SendMessage("Usage: COLOR <RED|GREEN|BLUE>\r\n");
        }
    }
    // 电机测试命令处理
    else if(strncmp(cmd, "M", 1) == 0) {
        // 所有以M开头的命令都转发给电机测试模块处理
        MotorTest_ProcessCommand(cmd, param);
    }
    else {
        Bluetooth_SendMessage("Unknown Command\r\n");
    }
}

/**
 * @brief 发送状态信息
 */
void Bluetooth_SendStatus(void)
{
    BalanceState_t* state = BalanceControl_GetState();
    
    // 获取视觉数据状态
    const char* vision_modes[] = {"OFF", "LINE", "TRACK"};
    const char* vision_mode_name = (state->vision_mode <= 2) ? vision_modes[state->vision_mode] : "UNKNOWN";
    
    // 发送基本状态信息
    snprintf((char*)Bluetooth.tx_buffer, BT_TX_BUFFER_SIZE,
            "STATUS: Pitch=%.2f Roll=%.2f Speed=%.2f Distance=%.1fcm Enabled=%d Vision=%s\r\n",
            state->pitch, state->roll,
            (state->left_speed + state->right_speed) / 2.0f,
            state->distance_front,
            state->balance_enabled,
            vision_mode_name);
    Bluetooth_SendMessage((char*)Bluetooth.tx_buffer);
    
    // 如果视觉模式开启，发送视觉数据
    if(state->vision_mode > 0) {
        snprintf((char*)Bluetooth.tx_buffer, BT_TX_BUFFER_SIZE,
                "VISION: ErrorX=%.3f ErrorY=%.3f LineDetected=%d ObjectDetected=%d\r\n",
                state->vision_error_x, state->vision_error_y,
                K230_Vision_IsLineDetected(), K230_Vision_IsObjectDetected());
        Bluetooth_SendMessage((char*)Bluetooth.tx_buffer);
    }
}

/**
 * @brief 发送消息
 * @param message: 要发送的消息
 */
void Bluetooth_SendMessage(const char* message)
{
    if(message == NULL || strlen(message) == 0) {
        return;
    }
    
    HAL_UART_Transmit_DMA(BT_UART_HANDLE, (uint8_t*)message, strlen(message));
}

/**
 * @brief 发送浮点数值
 * @param name: 数值名称
 * @param value: 浮点数值
 */
void Bluetooth_SendFloat(const char* name, float value)
{
    snprintf((char*)Bluetooth.tx_buffer, BT_TX_BUFFER_SIZE, "%s: %.3f\r\n", name, value);
    Bluetooth_SendMessage((char*)Bluetooth.tx_buffer);
}

/**
 * @brief 发送整数值
 * @param name: 数值名称
 * @param value: 整数值
 */
void Bluetooth_SendInt(const char* name, int value)
{
    snprintf((char*)Bluetooth.tx_buffer, BT_TX_BUFFER_SIZE, "%s: %d\r\n", name, value);
    Bluetooth_SendMessage((char*)Bluetooth.tx_buffer);
}


/**
 * @brief 获取蓝牙状态
 * @return 当前状态
 */
BluetoothState_t Bluetooth_GetState(void)
{
    return Bluetooth.state;
}

/**
 * @brief 检查蓝牙是否连接
 * @return 1-已连接, 0-未连接
 */
uint8_t Bluetooth_IsConnected(void)
{
    return Bluetooth.connected;
}
