/**
 * @file    bluetooth.c
 * @brief   蓝牙通信模块实现文件
 * @details 该文件实现了平衡车与手机APP或上位机的蓝牙通信功能。
 *          支持命令解析、状态反馈、参数设置等功能。通信协议基于
 *          文本命令格式，便于调试和扩展。主要功能包括：
 *          - 接收和解析控制命令
 *          - 发送平衡车状态信息
 *          - 参数在线调节
 *          - 连接状态管理
 *
 * @author  STM32平衡车项目组
 * @version 1.0
 * @date    2024
 *
 * @note    使用UART中断方式接收数据，确保实时性
 * @note    支持心跳检测，自动判断连接状态
 */

#include "bluetooth.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/*==============================================================================
                                全局变量定义
==============================================================================*/

/**
 * @brief 蓝牙通信实例定义
 * @details 存储蓝牙通信的所有状态信息，包括接收缓冲区、发送缓冲区、
 *          命令缓冲区、连接状态等。该结构体是蓝牙通信模块的核心数据结构。
 */
Bluetooth_t Bluetooth = {
    .rx_buffer = {0},           // 接收缓冲区，存储从UART接收的原始数据
    .tx_buffer = {0},           // 发送缓冲区，存储待发送的数据
    .command_buffer = {0},      // 命令缓冲区，存储完整的命令字符串
    .rx_index = 0,              // 接收缓冲区索引，指向下一个写入位置
    .tx_length = 0,             // 发送数据长度
    .state = BT_STATE_IDLE,     // 蓝牙通信状态，初始为空闲状态
    .command_ready = 0,         // 命令就绪标志，1表示有完整命令待处理
    .connected = 0,             // 连接状态标志，1表示已连接
    .last_heartbeat = 0         // 最后一次通信时间戳，用于连接检测
};

/*==============================================================================
                                私有函数声明
==============================================================================*/

/**
 * @brief 解析接收到的命令字符串
 * @details 将命令缓冲区中的字符串解析为命令和参数
 */
static void Bluetooth_ParseCommand(void);

/**
 * @brief 执行解析后的命令
 * @param cmd: 命令字符串
 * @param param: 参数字符串
 */
static void Bluetooth_ExecuteCommand(const char* cmd, const char* param);

/**
 * @brief 发送响应消息
 * @param response: 响应字符串
 */
static void Bluetooth_SendResponse(const char* response);

/*==============================================================================
                                函数实现
==============================================================================*/

/**
 * @brief  蓝牙通信模块初始化
 * @details 初始化蓝牙通信模块的所有参数和状态，启动UART接收中断，
 *          并发送初始化完成消息。该函数必须在使用蓝牙功能前调用。
 *
 * @param  无
 * @retval 无
 *
 * @note   该函数会启动UART接收中断，确保能够实时接收数据
 * @note   初始化后会发送"Balance Robot Ready"消息，表示系统就绪
 * @note   所有状态变量都会被重置为初始值
 */
void Bluetooth_Init(void)
{
    // 启动UART接收中断，开始接收蓝牙数据
    // 使用单字节接收模式，每接收一个字节就触发中断
    HAL_UART_Receive_IT(BT_UART_HANDLE, &Bluetooth.rx_buffer[0], 1);

    // 初始化蓝牙通信状态变量
    Bluetooth.state = BT_STATE_IDLE;        // 设置为空闲状态
    Bluetooth.rx_index = 0;                 // 接收缓冲区索引清零
    Bluetooth.command_ready = 0;            // 清除命令就绪标志
    Bluetooth.connected = 0;                // 设置为未连接状态
    Bluetooth.last_heartbeat = HAL_GetTick(); // 初始化心跳时间戳

    // 发送初始化完成消息，通知上位机系统已就绪
    Bluetooth_SendMessage("Balance Robot Ready\r\n");
}

/**
 * @brief  蓝牙通信模块更新函数
 * @details 该函数负责蓝牙通信的周期性维护任务，包括连接状态检测、
 *          命令处理和状态信息发送。应该在主循环中周期性调用。
 *
 *          主要功能：
 *          1. 连接状态监控：超过5秒无通信则判断为断开
 *          2. 命令处理：处理接收到的完整命令
 *          3. 状态反馈：定期发送平衡车状态信息
 *
 * @param  无
 * @retval 无
 *
 * @note   建议调用频率为10-50Hz，过低会影响响应速度
 * @note   连接超时时间为5秒，可根据实际需要调整
 * @note   状态信息发送间隔为2秒，避免数据量过大
 */
void Bluetooth_Update(void)
{
    // 获取当前时间戳
    uint32_t current_time = HAL_GetTick();

    // 连接状态检测：如果超过5秒没有收到数据，则认为连接断开
    // 这是一种简单的心跳检测机制，确保连接状态的准确性
    if(current_time - Bluetooth.last_heartbeat > 5000) {
        Bluetooth.connected = 0;  // 标记为断开状态
    }

    // 处理接收到的完整命令
    // 当command_ready标志为1时，表示有完整的命令等待处理
    if(Bluetooth.command_ready) {
        Bluetooth_ProcessCommand();     // 解析并执行命令
        Bluetooth.command_ready = 0;    // 清除命令就绪标志
    }

    // 定期发送平衡车状态信息（每2秒发送一次）
    // 只有在连接状态下才发送，避免无效传输
    static uint32_t last_status_time = 0;  // 静态变量记录上次发送时间
    if(current_time - last_status_time > 2000 && Bluetooth.connected) {
        Bluetooth_SendStatus();         // 发送状态信息
        last_status_time = current_time; // 更新发送时间戳
    }
}

/**
 * @brief  处理UART接收到的单个字节数据
 * @details 该函数在UART接收中断中被调用，负责接收和缓存数据，
 *          检测命令结束符，组装完整的命令字符串。使用状态机
 *          方式处理数据流，确保命令的完整性和正确性。
 *
 * @param  data: 从UART接收到的单个字节数据
 * @retval 无
 *
 * @note   该函数在中断上下文中执行，应保持简洁高效
 * @note   支持\r和\n作为命令结束符，兼容不同终端
 * @note   自动更新心跳时间戳和连接状态
 */
void Bluetooth_HandleRxData(uint8_t data)
{
    // 更新心跳时间戳，表示收到了数据
    Bluetooth.last_heartbeat = HAL_GetTick();
    // 标记为已连接状态
    Bluetooth.connected = 1;

    // 检查是否为命令结束符（回车或换行）
    if(data == '\r' || data == '\n') {
        // 如果接收缓冲区中有数据，则处理命令
        if(Bluetooth.rx_index > 0) {
            // 将接收缓冲区的数据复制到命令缓冲区
            memcpy(Bluetooth.command_buffer, Bluetooth.rx_buffer, Bluetooth.rx_index);
            // 添加字符串结束符，确保是有效的C字符串
            Bluetooth.command_buffer[Bluetooth.rx_index] = '\0';
            // 设置命令就绪标志，通知主循环有命令需要处理
            Bluetooth.command_ready = 1;
            // 重置接收缓冲区索引，准备接收下一个命令
            Bluetooth.rx_index = 0;
        }
    }
    // 检查接收缓冲区是否还有空间（预留1字节给字符串结束符）
    else if(Bluetooth.rx_index < BT_RX_BUFFER_SIZE - 1) {
        // 将数据存入接收缓冲区，并递增索引
        Bluetooth.rx_buffer[Bluetooth.rx_index++] = data;
    }
    else {
        // 接收缓冲区溢出，重置索引以避免内存越界
        // 这会丢弃当前正在接收的命令，但保证系统稳定性
        Bluetooth.rx_index = 0;
    }
}

/**
 * @brief  UART接收完成回调函数
 * @details 该函数在UART接收中断完成后被HAL库调用，用于重新启动
 *          下一次的单字节接收。确保能够连续接收数据流。
 *
 * @param  无
 * @retval 无
 *
 * @note   该函数在中断上下文中执行
 * @note   必须重新启动接收，否则只能接收一个字节
 */
void Bluetooth_RxCallback(void)
{
    // 重新启动UART接收中断，准备接收下一个字节
    // 使用当前接收缓冲区的位置作为接收地址
    HAL_UART_Receive_IT(BT_UART_HANDLE, &Bluetooth.rx_buffer[Bluetooth.rx_index], 1);
}

/**
 * @brief  蓝牙命令预处理函数
 * @details 解析接收到的完整命令字符串，分离命令和参数部分，
 *          然后调用具体的命令执行函数。支持带参数和不带参数的命令。
 *
 *          命令格式："COMMAND [PARAMETER]"
 *          例如："START"、"FORWARD 0.5"、"PID_P 1.2"
 *
 * @param  无
 * @retval 无
 *
 * @note   使用空格作为命令和参数的分隔符
 * @note   命令字符串会被修改（插入\0分隔符）
 */
void Bluetooth_ProcessCommand(void)
{
    // 获取命令字符串指针
    char* cmd = (char*)Bluetooth.command_buffer;
    char* param = NULL;

    // 查找空格分隔符，用于分离命令和参数
    char* space_pos = strchr(cmd, ' ');
    if(space_pos != NULL) {
        // 将空格替换为字符串结束符，分离命令部分
        *space_pos = '\0';
        // 参数部分从空格后开始
        param = space_pos + 1;
    }

    // 调用具体的命令执行函数
    Bluetooth_ExecuteCommand(cmd, param);
}

/**
 * @brief  蓝牙命令执行函数
 * @details 根据解析出的命令字符串执行相应的控制操作。支持平衡车的
 *          基本运动控制、PID参数调节、状态查询等功能。
 *
 *          支持的命令类型：
 *          - 运动控制：START、STOP、FORWARD、BACKWARD、LEFT、RIGHT
 *          - PID调节：PID_P、PID_I、PID_D（角度、速度、转向PID）
 *          - 状态查询：STATUS、GET_ANGLE、GET_SPEED
 *          - 系统控制：RESET、CALIBRATE
 *
 * @param  cmd: 命令字符串（不含参数）
 * @param  param: 参数字符串（可为NULL）
 * @retval 无
 *
 * @note   所有命令执行后都会发送响应消息
 * @note   参数解析失败时使用默认值
 * @note   无效命令会返回错误提示
 */
static void Bluetooth_ExecuteCommand(const char* cmd, const char* param)
{
    // 获取平衡控制状态结构体指针，用于状态查询和参数设置
    BalanceState_t* state = BalanceControl_GetState();

    // 启动平衡控制命令
    if(strcmp(cmd, BT_CMD_START) == 0) {
        BalanceControl_Enable(1);                    // 启用平衡控制
        Bluetooth_SendResponse("Balance Started\r\n"); // 发送确认消息
    }
    // 停止平衡控制命令
    else if(strcmp(cmd, BT_CMD_STOP) == 0) {
        BalanceControl_Enable(0);                   // 禁用平衡控制
        Bluetooth_SendResponse("Balance Stopped\r\n"); // 发送确认消息
    }
    // 前进运动命令
    else if(strcmp(cmd, BT_CMD_FORWARD) == 0) {
        float speed = 0.2f; // 默认前进速度（m/s）
        if(param != NULL) {
            speed = atof(param); // 解析参数中的速度值
        }
        BalanceControl_SetTargetSpeed(speed);        // 设置目标速度
        Bluetooth_SendResponse("Moving Forward\r\n"); // 发送确认消息
    }
    // 后退运动命令
    else if(strcmp(cmd, BT_CMD_BACKWARD) == 0) {
        float speed = -0.2f; // 默认后退速度（负值表示后退）
        if(param != NULL) {
            speed = -atof(param); // 解析参数并取负值
        }
        BalanceControl_SetTargetSpeed(speed);         // 设置目标速度
        Bluetooth_SendResponse("Moving Backward\r\n"); // 发送确认消息
    }
    // 左转命令
    else if(strcmp(cmd, BT_CMD_LEFT) == 0) {
        BalanceControl_SetTargetAngle(-5.0f);       // 设置左转角度（负值）
        Bluetooth_SendResponse("Turning Left\r\n");  // 发送确认消息
    }
    // 右转命令
    else if(strcmp(cmd, BT_CMD_RIGHT) == 0) {
        BalanceControl_SetTargetAngle(5.0f);        // 设置右转角度（正值）
        Bluetooth_SendResponse("Turning Right\r\n"); // 发送确认消息
    }
    // 状态查询命令
    else if(strcmp(cmd, BT_CMD_STATUS) == 0) {
        Bluetooth_SendStatus();                     // 发送详细状态信息
    }
    // 系统复位命令
    else if(strcmp(cmd, BT_CMD_RESET) == 0) {
        BalanceControl_Enable(0);                   // 禁用平衡控制
        BalanceControl_SetTargetSpeed(0);           // 清零目标速度
        BalanceControl_SetTargetAngle(0);           // 清零目标角度
        MotorEncoder_Reset();                       // 重置编码器计数
        Bluetooth_SendResponse("System Reset\r\n"); // 发送确认消息
    }
    // 速度设置命令（带参数）
    else if(strcmp(cmd, BT_CMD_SPEED) == 0) {
        if(param != NULL) {
            float speed = atof(param);              // 解析速度参数
            BalanceControl_SetTargetSpeed(speed);   // 设置目标速度
            // 格式化并发送确认消息
            sprintf((char*)Bluetooth.tx_buffer, "Speed set to %.2f\r\n", speed);
            Bluetooth_SendResponse((char*)Bluetooth.tx_buffer);
        }
    }
    // 角度设置命令（带参数）
    else if(strcmp(cmd, BT_CMD_ANGLE) == 0) {
        if(param != NULL) {
            float angle = atof(param);              // 解析角度参数
            BalanceControl_SetTargetAngle(angle);   // 设置目标角度
            // 格式化并发送确认消息
            sprintf((char*)Bluetooth.tx_buffer, "Angle set to %.2f\r\n", angle);
            Bluetooth_SendResponse((char*)Bluetooth.tx_buffer);
        }
    }
    // PID参数设置命令（格式: PID KP KI KD）
    else if(strcmp(cmd, BT_CMD_PID) == 0) {
        if(param != NULL) {
            float kp, ki, kd;
            // 解析三个PID参数（比例、积分、微分）
            if(sscanf(param, "%f %f %f", &kp, &ki, &kd) == 3) {
                AnglePID.Kp = kp;                   // 设置比例系数
                AnglePID.Ki = ki;                   // 设置积分系数
                AnglePID.Kd = kd;                   // 设置微分系数
                // 格式化并发送确认消息
                sprintf((char*)Bluetooth.tx_buffer, "PID set: Kp=%.2f Ki=%.2f Kd=%.2f\r\n", kp, ki, kd);
                Bluetooth_SendResponse((char*)Bluetooth.tx_buffer);
            }
        }
    }
    // 未知命令处理
    else {
        Bluetooth_SendResponse("Unknown Command\r\n"); // 发送错误提示
    }
}

/**
 * @brief  发送平衡车状态信息
 * @details 收集并发送平衡车的实时状态信息，包括姿态角度、运动速度、
 *          障碍物距离和控制使能状态。用于上位机监控和调试。
 *
 *          发送的状态信息包括：
 *          - Pitch: 俯仰角（度）
 *          - Roll: 横滚角（度）
 *          - Speed: 平均速度（m/s）
 *          - Distance: 前方障碍物距离（cm）
 *          - Enabled: 平衡控制使能状态（0/1）
 *
 * @param  无
 * @retval 无
 *
 * @note   该函数通常在状态查询命令或定期状态更新时调用
 * @note   使用格式化字符串确保数据的可读性
 */
void Bluetooth_SendStatus(void)
{
    // 获取平衡控制状态结构体指针
    BalanceState_t* state = BalanceControl_GetState();

    // 格式化状态信息字符串
    // 包含俯仰角、横滚角、平均速度、前方距离和使能状态
    sprintf((char*)Bluetooth.tx_buffer,
            "STATUS: Pitch=%.2f Roll=%.2f Speed=%.2f Distance=%.1fcm Enabled=%d\r\n",
            state->pitch,                                    // 俯仰角（度）
            state->roll,                                     // 横滚角（度）
            (state->left_speed + state->right_speed) / 2.0f, // 左右轮平均速度
            state->distance_front,                           // 前方超声波距离
            state->balance_enabled);                         // 平衡控制使能状态

    // 发送格式化后的状态信息
    Bluetooth_SendResponse((char*)Bluetooth.tx_buffer);
}

/**
 * @brief  通过蓝牙发送文本消息
 * @details 使用UART接口发送字符串消息到蓝牙模块。该函数是底层的
 *          消息发送接口，被其他发送函数调用。
 *
 * @param  message: 要发送的消息字符串（以\0结尾）
 * @retval 无
 *
 * @note   发送超时时间为1000ms，适合大部分应用场景
 * @note   该函数是阻塞式发送，会等待发送完成
 * @note   确保消息字符串以\0结尾，否则可能发送异常数据
 */
void Bluetooth_SendMessage(const char* message)
{
    // 使用HAL库的UART发送函数，阻塞式发送
    // 参数：UART句柄、数据指针、数据长度、超时时间
    HAL_UART_Transmit(BT_UART_HANDLE, (uint8_t*)message, strlen(message), 1000);
}

/**
 * @brief  发送格式化的浮点数值
 * @details 将浮点数值格式化为"名称: 数值"的形式并通过蓝牙发送。
 *          主要用于调试和数据监控，方便上位机解析数值信息。
 *
 * @param  name: 数值的名称标识符（如"Angle"、"Speed"等）
 * @param  value: 要发送的浮点数值
 * @retval 无
 *
 * @note   浮点数精度为3位小数
 * @note   发送格式为"name: value\r\n"
 * @note   适用于实时数据监控和参数调试
 */
void Bluetooth_SendFloat(const char* name, float value)
{
    // 格式化浮点数为字符串（保留3位小数）
    sprintf((char*)Bluetooth.tx_buffer, "%s: %.3f\r\n", name, value);
    // 发送格式化后的字符串
    Bluetooth_SendMessage((char*)Bluetooth.tx_buffer);
}

/**
 * @brief  发送格式化的整数值
 * @details 将整数值格式化为"名称: 数值"的形式并通过蓝牙发送。
 *          主要用于发送计数器、状态码、配置参数等整数类型数据。
 *
 * @param  name: 数值的名称标识符（如"Count"、"Mode"等）
 * @param  value: 要发送的整数值
 * @retval 无
 *
 * @note   发送格式为"name: value\r\n"
 * @note   适用于状态信息和计数数据的发送
 */
void Bluetooth_SendInt(const char* name, int value)
{
    // 格式化整数为字符串
    sprintf((char*)Bluetooth.tx_buffer, "%s: %d\r\n", name, value);
    // 发送格式化后的字符串
    Bluetooth_SendMessage((char*)Bluetooth.tx_buffer);
}

/**
 * @brief  发送命令响应消息（私有函数）
 * @details 用于向上位机发送命令执行的响应消息。这是一个内部函数，
 *          封装了响应消息的发送逻辑，统一管理响应格式。
 *
 * @param  response: 要发送的响应字符串
 * @retval 无
 *
 * @note   该函数为静态函数，仅在本模块内部使用
 * @note   所有命令执行后的确认消息都通过此函数发送
 */
static void Bluetooth_SendResponse(const char* response)
{
    // 直接调用消息发送函数
    Bluetooth_SendMessage(response);
}

/**
 * @brief  获取蓝牙通信模块当前状态
 * @details 返回蓝牙通信模块的内部状态，用于外部模块查询蓝牙
 *          模块的工作状态（空闲、接收、处理等）。
 *
 * @param  无
 * @retval BluetoothState_t: 当前蓝牙模块状态
 *         - BT_STATE_IDLE: 空闲状态
 *         - BT_STATE_RECEIVING: 正在接收数据
 *         - BT_STATE_PROCESSING: 正在处理命令
 *
 * @note   可用于状态机管理和调试
 */
BluetoothState_t Bluetooth_GetState(void)
{
    return Bluetooth.state;  // 返回当前状态
}

/**
 * @brief  检查蓝牙连接状态
 * @details 检查蓝牙模块是否与上位机建立了有效连接。连接状态基于
 *          心跳检测机制，如果超过设定时间未收到数据则认为断开。
 *
 * @param  无
 * @retval uint8_t: 连接状态
 *         - 1: 已连接（最近5秒内有数据通信）
 *         - 0: 未连接（超过5秒无数据通信）
 *
 * @note   连接状态影响状态信息的定期发送
 * @note   可用于LED指示灯控制和系统状态显示
 */
uint8_t Bluetooth_IsConnected(void)
{
    return Bluetooth.connected;  // 返回连接状态
}
