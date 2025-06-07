/**
  ******************************************************************************
  * @file           : vision_demo.c
  * @brief          : K230视觉模块使用示例
  * @description    : 演示如何使用K230视觉模块进行循迹和物体追踪
  *                   包含基本的使用方法和测试函数
  ******************************************************************************
  */

#include "vision_demo.h"
#include "k230_vision.h"
#include "balance_control.h"
#include "bluetooth.h"
#include <stdio.h>

/**
 * @brief 视觉功能演示初始化
 */
void VisionDemo_Init(void)
{
    // 发送演示开始信息
    Bluetooth_SendMessage("\r\n=== K230 Vision Demo Started ===\r\n");
    Bluetooth_SendMessage("Available Commands:\r\n");
    Bluetooth_SendMessage("  LINE     - Start line tracking\r\n");
    Bluetooth_SendMessage("  TRACK    - Start object tracking\r\n");
    Bluetooth_SendMessage("  VOFF     - Turn off vision\r\n");
    Bluetooth_SendMessage("  STATUS   - Get system status\r\n");
    Bluetooth_SendMessage("================================\r\n\r\n");
}

/**
 * @brief 循迹演示
 */
void VisionDemo_LineTracking(void)
{
    Bluetooth_SendMessage("Starting Line Tracking Demo...\r\n");
    
    // 启用平衡控制
    BalanceControl_Enable(1);
    
    // 设置循迹模式
    BalanceControl_SetVisionMode(1);
    
    Bluetooth_SendMessage("Line tracking enabled. Place robot on a line.\r\n");
}

/**
 * @brief 物体追踪演示
 */
void VisionDemo_ObjectTracking(void)
{
    Bluetooth_SendMessage("Starting Object Tracking Demo...\r\n");
    
    // 启用平衡控制
    BalanceControl_Enable(1);
    
    // 设置物体追踪模式
    BalanceControl_SetVisionMode(2);
    
    Bluetooth_SendMessage("Object tracking enabled. Show an object to the camera.\r\n");
}

/**
 * @brief 停止视觉演示
 */
void VisionDemo_Stop(void)
{
    Bluetooth_SendMessage("Stopping Vision Demo...\r\n");
    
    // 关闭视觉模式
    BalanceControl_SetVisionMode(0);
    
    // 停止平衡控制
    BalanceControl_Enable(0);
    
    Bluetooth_SendMessage("Vision demo stopped.\r\n");
}

/**
 * @brief 视觉数据测试
 */
void VisionDemo_TestVisionData(void)
{
    char msg[200];
    BalanceState_t* state = BalanceControl_GetState();
    
    // 显示当前视觉模式
    const char* mode_names[] = {"OFF", "LINE_TRACKING", "OBJECT_TRACKING"};
    sprintf(msg, "Current Vision Mode: %s\r\n", mode_names[state->vision_mode]);
    Bluetooth_SendMessage(msg);
    
    // 显示视觉误差
    sprintf(msg, "Vision Errors - X: %.3f, Y: %.3f\r\n", 
            state->vision_error_x, state->vision_error_y);
    Bluetooth_SendMessage(msg);
    
    // 显示检测状态
    if(state->vision_mode == 1) {
        // 循迹模式
        K230_LineTrackingData_t* line_data = K230_Vision_GetLineTrackingData();
        sprintf(msg, "Line Detection: %s\r\n", 
                K230_Vision_IsLineDetected() ? "YES" : "NO");
        Bluetooth_SendMessage(msg);
        
        if(K230_Vision_IsLineDetected()) {
            sprintf(msg, "Line Position: X=%d, Y=%d, W=%d, H=%d\r\n",
                    line_data->x, line_data->y, line_data->w, line_data->h);
            Bluetooth_SendMessage(msg);
        }
    }
    else if(state->vision_mode == 2) {
        // 物体追踪模式
        K230_ObjectTrackingData_t* obj_data = K230_Vision_GetObjectTrackingData();
        sprintf(msg, "Object Detection: %s\r\n", 
                K230_Vision_IsObjectDetected() ? "YES" : "NO");
        Bluetooth_SendMessage(msg);
        
        if(K230_Vision_IsObjectDetected()) {
            sprintf(msg, "Object Position: X=%d, Y=%d, W=%d, H=%d\r\n",
                    obj_data->x, obj_data->y, obj_data->w, obj_data->h);
            Bluetooth_SendMessage(msg);
            
            float distance = K230_Vision_GetObjectDistance();
            sprintf(msg, "Estimated Distance: %.2f\r\n", distance);
            Bluetooth_SendMessage(msg);
        }
    }
}

/**
 * @brief 视觉通信测试
 */
void VisionDemo_TestCommunication(void)
{
    Bluetooth_SendMessage("Testing K230 Vision Communication...\r\n");
    
    // 测试通信状态
    if(K230_Vision_IsConnected()) {
        Bluetooth_SendMessage("K230 Communication: OK\r\n");
        
        // 获取通信统计
        char msg[100];
        sprintf(msg, "Communication Error Rate: %.2f%%\r\n", 
                K230_Vision_GetErrorRate());
        Bluetooth_SendMessage(msg);
    } else {
        Bluetooth_SendMessage("K230 Communication: FAILED\r\n");
        Bluetooth_SendMessage("Please check K230 connection and power.\r\n");
    }
}

/**
 * @brief 自动演示模式
 */
void VisionDemo_AutoDemo(void)
{
    static uint32_t demo_start_time = 0;
    static uint8_t demo_stage = 0;
    uint32_t current_time = HAL_GetTick();
    
    if(demo_start_time == 0) {
        demo_start_time = current_time;
        Bluetooth_SendMessage("Starting Auto Demo...\r\n");
    }
    
    uint32_t elapsed = current_time - demo_start_time;
    
    switch(demo_stage) {
        case 0: // 初始化阶段 (0-2秒)
            if(elapsed > 2000) {
                VisionDemo_LineTracking();
                demo_stage = 1;
            }
            break;
            
        case 1: // 循迹演示 (2-12秒)
            if(elapsed > 12000) {
                VisionDemo_ObjectTracking();
                demo_stage = 2;
            }
            break;
            
        case 2: // 物体追踪演示 (12-22秒)
            if(elapsed > 22000) {
                VisionDemo_Stop();
                demo_stage = 3;
            }
            break;
            
        case 3: // 演示结束
            Bluetooth_SendMessage("Auto Demo Completed!\r\n");
            demo_start_time = 0;
            demo_stage = 0;
            break;
    }
}