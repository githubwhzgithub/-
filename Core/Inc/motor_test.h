/**
 * @file motor_test.h
 * @brief 电机测试程序头文件 - 独立测试H4(左电机)和H5(右电机)功能
 * @author Balance Car Project
 * @date 2024
 * 
 * 使用说明：
 * 1. 蓝牙命令控制：
 *    - MTEST: 开始自动测试序列
 *    - MTEST STOP: 停止测试
 *    - MTEST STATUS: 查看测试状态
 *    - MLEFT [speed]: 测试左电机 (H4)
 *    - MRIGHT [speed]: 测试右电机 (H5)
 *    - MBOTH [speed]: 测试双电机
 *    - MSTOP: 停止所有电机
 *    - MSPEED [value]: 设置测试速度 (0-1000)
 * 
 * 2. 自动测试序列：
 *    - 左电机正转 -> 左电机反转
 *    - 右电机正转 -> 右电机反转
 *    - 双电机正转 -> 双电机反转
 *    - 速度渐变测试 -> 方向切换测试
 *    - 刹车测试 -> 完成
 */

#ifndef __MOTOR_TEST_H
#define __MOTOR_TEST_H

#include "main.h"
#include <stdint.h>

// 函数声明

/**
 * @brief 初始化电机测试模块
 */
void MotorTest_Init(void);

/**
 * @brief 开始自动测试序列
 * @note 测试序列包括单电机测试、双电机测试、速度渐变、方向切换等
 */
void MotorTest_StartAutoTest(void);

/**
 * @brief 停止所有测试
 */
void MotorTest_Stop(void);

/**
 * @brief 测试左电机正转 (H4)
 * @param speed: 电机速度 (0-1000)
 */
void MotorTest_LeftForward(int16_t speed);

/**
 * @brief 测试左电机反转 (H4)
 * @param speed: 电机速度 (0-1000)
 */
void MotorTest_LeftBackward(int16_t speed);

/**
 * @brief 测试右电机正转 (H5)
 * @param speed: 电机速度 (0-1000)
 */
void MotorTest_RightForward(int16_t speed);

/**
 * @brief 测试右电机反转 (H5)
 * @param speed: 电机速度 (0-1000)
 */
void MotorTest_RightBackward(int16_t speed);

/**
 * @brief 测试双电机同向正转
 * @param speed: 电机速度 (0-1000)
 */
void MotorTest_BothForward(int16_t speed);

/**
 * @brief 测试双电机同向反转
 * @param speed: 电机速度 (0-1000)
 */
void MotorTest_BothBackward(int16_t speed);

/**
 * @brief 测试速度渐变 (0 -> 最大 -> 0)
 */
void MotorTest_SpeedRamp(void);

/**
 * @brief 测试方向快速切换
 */
void MotorTest_DirectionChange(void);

/**
 * @brief 测试刹车功能
 */
void MotorTest_Brake(void);

/**
 * @brief 设置测试速度
 * @param speed: 测试速度 (0-1000)
 */
void MotorTest_SetSpeed(int16_t speed);

/**
 * @brief 获取当前测试状态
 * @return 测试状态字符串
 */
const char* MotorTest_GetStatus(void);

/**
 * @brief 电机测试更新函数
 * @note 需要在主循环中定期调用，用于自动测试序列的状态机更新
 */
void MotorTest_Update(void);

/**
 * @brief 处理电机测试命令
 * @param cmd: 命令字符串
 * @param param: 参数字符串
 */
void MotorTest_ProcessCommand(const char* cmd, const char* param);

// 测试命令定义
#define MOTOR_TEST_CMD_AUTO     "MTEST"     // 自动测试
#define MOTOR_TEST_CMD_LEFT     "MLEFT"     // 左电机测试
#define MOTOR_TEST_CMD_RIGHT    "MRIGHT"    // 右电机测试
#define MOTOR_TEST_CMD_BOTH     "MBOTH"     // 双电机测试
#define MOTOR_TEST_CMD_STOP     "MSTOP"     // 停止电机
#define MOTOR_TEST_CMD_SPEED    "MSPEED"    // 设置速度

// 测试参数
#define MOTOR_TEST_DEFAULT_SPEED    300     // 默认测试速度 (30%)
#define MOTOR_TEST_MAX_SPEED        1000    // 最大测试速度
#define MOTOR_TEST_STEP_DURATION    2000    // 每个测试步骤持续时间 (ms)

#endif /* __MOTOR_TEST_H */
