/**
  ******************************************************************************
  * @file    k230_vision.c
  * @brief   K230视觉模块通讯协议实现
  *          支持循迹和物体追踪功能
  ******************************************************************************
  */

#include "k230_vision.h"

/* 私有变量 */
static uint8_t k230_rx_buffer[K230_BUF_LEN_MAX];  // 接收缓冲区
static uint8_t k230_rx_index = 0;                 // 接收数据索引
static uint8_t k230_rx_flag = 0;                  // 接收状态机标志
static uint8_t k230_new_data_flag = 0;            // 新数据标志
static uint8_t k230_data_length = 0;              // 数据长度

static K230_Vision_t k230_vision_data;            // 视觉数据结构

/* 图像中心坐标定义 (假设640*480分辨率) */
#define IMAGE_CENTER_X  320
#define IMAGE_CENTER_Y  240
#define IMAGE_WIDTH     640
#define IMAGE_HEIGHT    480

/* 私有函数声明 */
static void K230_ParseData(uint8_t *data_buf, uint8_t length);
static int K230_CharToInt(char* data);
static void K230_ClearBuffer(void);

/**
 * @brief 初始化K230视觉模块
 * @param huart: 串口句柄
 */
void K230_Vision_Init(UART_HandleTypeDef *huart)
{
    // 清空接收缓冲区
    K230_ClearBuffer();
    
    // 初始化视觉数据结构
    memset(&k230_vision_data, 0, sizeof(K230_Vision_t));
    k230_vision_data.current_mode = K230_MODE_IDLE;
    k230_vision_data.communication_ok = 0;
    
    // 启动UART2接收中断 (K230连接到UART2)
    HAL_UART_Receive_IT(&huart2, &k230_rx_buffer[0], 1);
}

/**
 * @brief 接收数据处理
 * @param rx_data: 接收到的数据
 */
void K230_Vision_ReceiveData(uint8_t rx_data)
{
    switch (k230_rx_flag)
    {
    case 0:
        if (rx_data == K230_HEAD)
        {
            k230_rx_buffer[0] = K230_HEAD;
            k230_rx_flag = 1;
            k230_rx_index = 1;
        }
        break;

    case 1:
        k230_rx_buffer[k230_rx_index] = rx_data;
        k230_rx_index++;
        
        if (rx_data == K230_TAIL)
        {
            k230_new_data_flag = 1;
            k230_data_length = k230_rx_index;
            k230_rx_flag = 0;
            k230_rx_index = 0;
        }
        else if (k230_rx_index >= K230_BUF_LEN_MAX)
        {
            // 缓冲区溢出，重置状态
            k230_new_data_flag = 0;
            k230_data_length = 0;
            k230_rx_flag = 0;
            k230_rx_index = 0;
            K230_ClearBuffer();
        }
        break;

    default:
        k230_rx_flag = 0;
        break;
    }
}

/**
 * @brief 更新K230视觉模块数据
 */
void K230_Vision_Update(void)
{
    if (k230_new_data_flag)
    {
        K230_ParseData(k230_rx_buffer, k230_data_length);
        k230_new_data_flag = 0;
        K230_ClearBuffer();
        k230_vision_data.communication_ok = 1;
    }
    
    // 检查通讯超时 (1秒无数据认为通讯异常)
    uint32_t current_time = HAL_GetTick();
    if (k230_vision_data.line_track.valid && 
        (current_time - k230_vision_data.line_track.last_update > 1000))
    {
        k230_vision_data.line_track.valid = 0;
        k230_vision_data.line_track.line_detected = 0;
    }
    
    if (k230_vision_data.object_track.valid && 
        (current_time - k230_vision_data.object_track.last_update > 1000))
    {
        k230_vision_data.object_track.valid = 0;
        k230_vision_data.object_track.obj_detected = 0;
    }
    
    // 检查整体通讯状态
    if ((current_time - k230_vision_data.line_track.last_update > 2000) &&
        (current_time - k230_vision_data.object_track.last_update > 2000))
    {
        k230_vision_data.communication_ok = 0;
    }
}

/**
 * @brief 设置K230工作模式
 * @param mode: 工作模式
 */
void K230_SetMode(K230_Mode_t mode)
{
    k230_vision_data.current_mode = mode;
    // 发送模式切换命令给vision_tracker.py
    // 格式: MODE_1 (循迹模式) 或 MODE_2 (物体检测模式)
    char cmd[20];
    if (mode == K230_MODE_LINE_TRACK) {
        sprintf(cmd, "MODE_1\r\n");
    } else if (mode == K230_MODE_OBJECT_TRACK) {
        sprintf(cmd, "MODE_2\r\n");
    } else {
        sprintf(cmd, "MODE_0\r\n");  // 空闲模式
    }
    HAL_UART_Transmit(&huart2, (uint8_t*)cmd, strlen(cmd), 100);
}

/**
 * @brief 获取视觉数据
 * @return 视觉数据结构指针
 */
K230_Vision_t* K230_GetVisionData(void)
{
    return &k230_vision_data;
}

/**
 * @brief 清空数据
 */
void K230_ClearData(void)
{
    k230_vision_data.line_track.valid = 0;
    k230_vision_data.line_track.line_detected = 0;
    k230_vision_data.object_track.valid = 0;
    k230_vision_data.object_track.obj_detected = 0;
}

/**
 * @brief 获取线条角度偏差
 * @return 角度偏差 (-90 到 90度)
 */
float K230_GetLineAngleError(void)
{
    if (!k230_vision_data.line_track.valid || !k230_vision_data.line_track.line_detected)
        return 0.0f;
    
    return (float)k230_vision_data.line_track.line_angle;
}

/**
 * @brief 获取线条位置偏差
 * @return 位置偏差 (-1.0 到 1.0, 负值表示左偏，正值表示右偏)
 */
float K230_GetLinePositionError(void)
{
    if (!k230_vision_data.line_track.valid || !k230_vision_data.line_track.line_detected)
        return 0.0f;
    
    float error = (float)(k230_vision_data.line_track.line_x - IMAGE_CENTER_X) / (IMAGE_WIDTH / 2.0f);
    // 限制在 -1.0 到 1.0 范围内
    if (error > 1.0f) error = 1.0f;
    if (error < -1.0f) error = -1.0f;
    
    return error;
}

/**
 * @brief 检查是否检测到线条
 * @return 1: 检测到, 0: 未检测到
 */
uint8_t K230_Vision_IsLineDetected(void)
{
    return (k230_vision_data.line_track.valid && k230_vision_data.line_track.line_detected);
}

/**
 * @brief 获取物体X方向偏差
 * @return X方向偏差 (-1.0 到 1.0)
 */
float K230_GetObjectXError(void)
{
    if (!k230_vision_data.object_track.valid || !k230_vision_data.object_track.obj_detected)
        return 0.0f;
    
    float error = (float)(k230_vision_data.object_track.obj_x - IMAGE_CENTER_X) / (IMAGE_WIDTH / 2.0f);
    // 限制在 -1.0 到 1.0 范围内
    if (error > 1.0f) error = 1.0f;
    if (error < -1.0f) error = -1.0f;
    
    return error;
}

/**
 * @brief 获取物体Y方向偏差
 * @return Y方向偏差 (-1.0 到 1.0)
 */
float K230_GetObjectYError(void)
{
    if (!k230_vision_data.object_track.valid || !k230_vision_data.object_track.obj_detected)
        return 0.0f;
    
    float error = (float)(k230_vision_data.object_track.obj_y - IMAGE_CENTER_Y) / (IMAGE_HEIGHT / 2.0f);
    // 限制在 -1.0 到 1.0 范围内
    if (error > 1.0f) error = 1.0f;
    if (error < -1.0f) error = -1.0f;
    
    return error;
}

/**
 * @brief 检查是否检测到物体
 * @return 1: 检测到, 0: 未检测到
 */
uint8_t K230_Vision_IsObjectDetected(void)
{
    return (k230_vision_data.object_track.valid && k230_vision_data.object_track.obj_detected);
}

/**
 * @brief 根据物体大小估算距离
 * @return 估算距离 (相对值)
 */
float K230_GetObjectDistance(void)
{
    if (!k230_vision_data.object_track.valid || !k230_vision_data.object_track.obj_detected)
        return 0.0f;
    
    // 简单的距离估算：物体越大，距离越近
    float object_size = (float)(k230_vision_data.object_track.obj_w * k230_vision_data.object_track.obj_h);
    float max_size = IMAGE_WIDTH * IMAGE_HEIGHT;
    
    return object_size / max_size;  // 返回0-1之间的值，1表示最近
}

/* 私有函数实现 */

/**
 * @brief 解析接收到的数据
 * @param data_buf: 数据缓冲区
 * @param length: 数据长度
 */
static void K230_ParseData(uint8_t *data_buf, uint8_t length)
{
    uint8_t pto_head = data_buf[0];
    uint8_t pto_tail = data_buf[length-1];
    
    // 检查协议头尾
    if (!(pto_head == K230_HEAD && pto_tail == K230_TAIL))
    {
        return;
    }
    
    // 解析字段
    uint8_t data_index = 0;
    uint8_t field_start[K230_BUF_LEN_MAX] = {1}; // 第一个字段从索引1开始
    int values[K230_BUF_LEN_MAX] = {0};
    
    // 查找逗号分隔符，记录每个字段的起始位置
    for (int i = 1; i < length-1; i++)
    {
        if (data_buf[i] == ',')
        {
            data_buf[i] = 0;  // 替换为字符串结束符
            data_index++;
            if (data_index < K230_BUF_LEN_MAX)
            {
                field_start[data_index] = i + 1;
            }
        }
    }
    data_index++; // 包含最后一个字段
    
    // 转换字符串为数值
    for (int i = 0; i < data_index && i < K230_BUF_LEN_MAX; i++)
    {
        values[i] = K230_CharToInt((char*)data_buf + field_start[i]);
    }
    
    // 检查数据长度
    if (data_index < 2)
    {
        return;
    }
    
    uint8_t pto_len = values[0];
    if (pto_len != length)
    {
        return;
    }
    
    // 检查功能ID并解析对应数据
    uint8_t func_id = values[1];
    uint32_t current_time = HAL_GetTick();
    
    // 新的vision_tracker.py使用YbProtocol发送数据
    // 循迹模式数据格式: $length,1,center_x,angle,speed_percentage,found#
    // 物体检测模式数据格式: $length,1,center_x,center_y,width,found#
    if (func_id == K230_MODE_LINE_TRACKING && data_index >= 6)
    {
        k230_vision_data.line_track.line_x = values[2];        // 线条中心X坐标
        k230_vision_data.line_track.line_y = 240;              // 固定Y坐标(图像中心)
        k230_vision_data.line_track.line_angle = values[3];    // 线条角度
        // values[4] 是速度百分比 (0-100)
        // values[5] 是found标志
        k230_vision_data.line_track.line_detected = (values[5] > 0) ? 1 : 0;
        k230_vision_data.line_track.valid = 1;
        k230_vision_data.line_track.last_update = current_time;
    }
    else if (func_id == K230_MODE_OBJECT_TRACKING && data_index >= 6)
    {
        // 物体追踪数据格式: $length,1,center_x,center_y,width,found#
        k230_vision_data.object_track.obj_x = values[2];       // 物体中心X坐标
        k230_vision_data.object_track.obj_y = values[3];       // 物体中心Y坐标
        k230_vision_data.object_track.obj_w = values[4];       // 物体宽度
        k230_vision_data.object_track.obj_h = values[4];       // 物体高度(使用宽度近似)
        k230_vision_data.object_track.obj_detected = (values[5] > 0) ? 1 : 0;
        k230_vision_data.object_track.valid = 1;
        k230_vision_data.object_track.last_update = current_time;
    }
}

/**
 * @brief 字符串转整数
 * @param data: 字符串指针
 * @return 转换后的整数
 */
static int K230_CharToInt(char* data)
{
    return atoi(data);
}

/**
 * @brief 清空接收缓冲区
 */
static void K230_ClearBuffer(void)
{
    memset(k230_rx_buffer, 0, K230_BUF_LEN_MAX);
    k230_data_length = 0;
    k230_new_data_flag = 0;
}
