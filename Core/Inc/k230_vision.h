#ifndef __K230_VISION_H__
#define __K230_VISION_H__

#include "main.h"
#include "usart.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* K230视觉模块通讯协议定义 */
#define K230_BUF_LEN_MAX           (200)    // 接收缓冲区最大长度
#define K230_HEAD                  (0x24)  // 协议头 '$'
#define K230_TAIL                  (0x23)  // 协议尾 '#'

/* 图像尺寸定义 (与vision_tracker.py中的Config.DISPLAY_WIDTH/HEIGHT对应) */
#define IMAGE_WIDTH                (640)    // 图像宽度
#define IMAGE_HEIGHT               (480)    // 图像高度
#define IMAGE_CENTER_X             (320)    // 图像中心X坐标
#define IMAGE_CENTER_Y             (240)    // 图像中心Y坐标


/* 循迹数据结构 */
typedef struct {
    uint8_t valid;          // 数据有效标志
    int16_t line_x;         // 线条中心X坐标
    int16_t line_y;         // 线条中心Y坐标
    int16_t line_angle;     // 线条角度 (-90 到 90度)
    uint8_t line_detected;  // 线条检测标志
    uint32_t last_update;   // 最后更新时间
} K230_LineTrack_t;

/* 物体追踪数据结构 */
typedef struct {
    uint8_t valid;          // 数据有效标志
    int16_t obj_x;          // 物体中心X坐标
    int16_t obj_y;          // 物体中心Y坐标
    int16_t obj_w;          // 物体宽度
    int16_t obj_h;          // 物体高度
    uint8_t obj_detected;   // 物体检测标志
    uint32_t last_update;   // 最后更新时间
} K230_ObjectTrack_t;

/* K230视觉模块状态结构 */
typedef struct {
    K230_LineTrack_t line_track;      // 循迹数据
    K230_ObjectTrack_t object_track;  // 物体追踪数据
    uint8_t current_mode;              // 当前工作模式
    uint8_t communication_ok;          // 通讯状态
} K230_Vision_t;

/* 工作模式定义 */
typedef enum {
    K230_MODE_IDLE = 0,               // Idle mode
    K230_MODE_LINE_TRACK = 1,      // Line tracking mode (mode 1 in vision_tracker.py)
    K230_MODE_OBJECT_TRACK = 2,    // Object tracking mode (mode 2 in vision_tracker.py)
} K230_Mode_t;


/* 函数声明 */
void K230_Vision_Init(UART_HandleTypeDef *huart);
void K230_Vision_ReceiveData(uint8_t rx_data);
void K230_Vision_Update(void);
void K230_SetMode(K230_Mode_t mode);
K230_Vision_t* K230_GetVisionData(void);
void K230_ClearData(void);

/* 循迹相关函数 */
float K230_GetLineAngleError(void);     // 获取线条角度偏差
float K230_GetLinePositionError(void);  // 获取线条位置偏差
uint8_t K230_Vision_IsLineDetected(void);      // 检查是否检测到线条

/* 物体追踪相关函数 */
float K230_GetObjectXError(void);       // 获取物体X方向偏差
float K230_GetObjectYError(void);       // 获取物体Y方向偏差
uint8_t K230_Vision_IsObjectDetected(void);    // 检查是否检测到物体
float K230_GetObjectDistance(void);     // 根据物体大小估算距离

/* 注意: UART接收缓冲区现在使用main.c中HAL_UART_RxCpltCallback函数的静态局部变量 */

#endif /* __K230_VISION_H__ */

