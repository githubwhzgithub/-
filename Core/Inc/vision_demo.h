/**
  ******************************************************************************
  * @file           : vision_demo.h
  * @brief          : K230视觉模块使用示例头文件
  * @description    : 定义视觉演示功能的接口和常量
  ******************************************************************************
  */

#ifndef __VISION_DEMO_H
#define __VISION_DEMO_H

#include "main.h"

/* 演示模式定义 */
#define DEMO_MODE_MANUAL    0   // 手动模式
#define DEMO_MODE_AUTO      1   // 自动演示模式

/* 函数声明 */

/**
 * @brief 视觉功能演示初始化
 */
void VisionDemo_Init(void);

/**
 * @brief 循迹演示
 */
void VisionDemo_LineTracking(void);

/**
 * @brief 物体追踪演示
 */
void VisionDemo_ObjectTracking(void);

/**
 * @brief 停止视觉演示
 */
void VisionDemo_Stop(void);

/**
 * @brief 视觉数据测试
 */
void VisionDemo_TestVisionData(void);

/**
 * @brief 视觉通信测试
 */
void VisionDemo_TestCommunication(void);

/**
 * @brief 自动演示模式
 */
void VisionDemo_AutoDemo(void);

#endif /* __VISION_DEMO_H */