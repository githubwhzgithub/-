/**
 * @file motor_test.c
 * @brief 电机测试程序 - 独立测试H4(左电机)和H5(右电机)功能
 * @author Balance Car Project
 * @date 2024
 * 
 * 测试说明：
 * - H4: 左电机 (MotorA) - 连接到TB6612的A组输出
 * - H5: 右电机 (MotorB) - 连接到TB6612的B组输出
 * 
 * 硬件连接：
 * - MotorA: AO1(Pin5), AO2(Pin1) -> H4左电机
 * - MotorB: BO1(Pin6), BO2(Pin2) -> H5右电机
 * - PWMA: TIM3_CH1 -> TB6612 Pin7
 * - PWMB: TIM3_CH2 -> TB6612 Pin10
 */

#include "motor_test.h"
#include "TB6612.h"
#include "bluetooth.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

// 测试状态枚举
typedef enum {
    TEST_IDLE = 0,
    TEST_LEFT_FORWARD,
    TEST_LEFT_BACKWARD,
    TEST_RIGHT_FORWARD,
    TEST_RIGHT_BACKWARD,
    TEST_BOTH_FORWARD,
    TEST_BOTH_BACKWARD,
    TEST_SPEED_RAMP,
    TEST_DIRECTION_CHANGE,
    TEST_BRAKE,
    TEST_COMPLETE
} MotorTestState_t;

// 测试参数
static MotorTestState_t current_test = TEST_IDLE;
static uint32_t test_start_time = 0;
static uint32_t test_duration = 2000; // 每个测试持续2秒
static int16_t test_speed = 300; // 测试速度 (30%)
static uint8_t test_running = 0;
static char test_message[128];

/**
 * @brief 初始化电机测试
 */
void MotorTest_Init(void)
{
    // 初始化TB6612电机驱动
    TB6612_Init();
    
    // 停止所有电机
    TB6612_StopAllMotors();
    
    // 重置测试状态
    current_test = TEST_IDLE;
    test_running = 0;
    
    sprintf(test_message, "Motor Test Initialized. H4=Left Motor, H5=Right Motor\r\n");
    Bluetooth_SendMessage(test_message);
}

/**
 * @brief 开始自动测试序列
 */
void MotorTest_StartAutoTest(void)
{
    if(test_running) {
        sprintf(test_message, "Test already running...\r\n");
        Bluetooth_SendMessage(test_message);
        return;
    }
    
    test_running = 1;
    current_test = TEST_LEFT_FORWARD;
    test_start_time = HAL_GetTick();
    
    sprintf(test_message, "Starting Motor Auto Test Sequence...\r\n");
    Bluetooth_SendMessage(test_message);
    sprintf(test_message, "Test Speed: %d/1000 (%.1f%%)\r\n", test_speed, test_speed/10.0f);
    Bluetooth_SendMessage(test_message);
}

/**
 * @brief 停止测试
 */
void MotorTest_Stop(void)
{
    test_running = 0;
    current_test = TEST_IDLE;
    TB6612_StopAllMotors();
    
    sprintf(test_message, "Motor Test Stopped\r\n");
    Bluetooth_SendMessage(test_message);
}

/**
 * @brief 测试左电机正转 (H4)
 */
void MotorTest_LeftForward(int16_t speed)
{
    TB6612_SetMotorSpeed(&MotorA, speed);
    TB6612_SetMotorSpeed(&MotorB, 0);
    
    sprintf(test_message, "Testing H4 (Left Motor) Forward - Speed: %d\r\n", speed);
    Bluetooth_SendMessage(test_message);
}

/**
 * @brief 测试左电机反转 (H4)
 */
void MotorTest_LeftBackward(int16_t speed)
{
    TB6612_SetMotorSpeed(&MotorA, -speed);
    TB6612_SetMotorSpeed(&MotorB, 0);
    
    sprintf(test_message, "Testing H4 (Left Motor) Backward - Speed: %d\r\n", speed);
    Bluetooth_SendMessage(test_message);
}

/**
 * @brief 测试右电机正转 (H5)
 */
void MotorTest_RightForward(int16_t speed)
{
    TB6612_SetMotorSpeed(&MotorA, 0);
    TB6612_SetMotorSpeed(&MotorB, speed);
    
    sprintf(test_message, "Testing H5 (Right Motor) Forward - Speed: %d\r\n", speed);
    Bluetooth_SendMessage(test_message);
}

/**
 * @brief 测试右电机反转 (H5)
 */
void MotorTest_RightBackward(int16_t speed)
{
    TB6612_SetMotorSpeed(&MotorA, 0);
    TB6612_SetMotorSpeed(&MotorB, -speed);
    
    sprintf(test_message, "Testing H5 (Right Motor) Backward - Speed: %d\r\n", speed);
    Bluetooth_SendMessage(test_message);
}

/**
 * @brief 测试双电机同向
 */
void MotorTest_BothForward(int16_t speed)
{
    TB6612_SetBothMotors(speed, speed);
    
    sprintf(test_message, "Testing Both Motors Forward - Speed: %d\r\n", speed);
    Bluetooth_SendMessage(test_message);
}

/**
 * @brief 测试双电机反向
 */
void MotorTest_BothBackward(int16_t speed)
{
    TB6612_SetBothMotors(-speed, -speed);
    
    sprintf(test_message, "Testing Both Motors Backward - Speed: %d\r\n", speed);
    Bluetooth_SendMessage(test_message);
}

/**
 * @brief 测试速度渐变
 */
void MotorTest_SpeedRamp(void)
{
    static uint32_t ramp_start_time = 0;
    static int16_t current_speed = 0;
    
    if(ramp_start_time == 0) {
        ramp_start_time = HAL_GetTick();
        current_speed = 0;
        sprintf(test_message, "Testing Speed Ramp (0 -> 500 -> 0)\r\n");
        Bluetooth_SendMessage(test_message);
    }
    
    uint32_t elapsed = HAL_GetTick() - ramp_start_time;
    
    if(elapsed < 2000) {
        // 0-2秒：速度从0增加到500
        current_speed = (int16_t)(elapsed * 500 / 2000);
    } else if(elapsed < 4000) {
        // 2-4秒：速度从500减少到0
        current_speed = (int16_t)(500 - (elapsed - 2000) * 500 / 2000);
    } else {
        current_speed = 0;
        ramp_start_time = 0;
    }
    
    TB6612_SetBothMotors(current_speed, current_speed);
}

/**
 * @brief 测试方向切换
 */
void MotorTest_DirectionChange(void)
{
    static uint32_t dir_start_time = 0;
    static uint8_t direction = 0;
    
    if(dir_start_time == 0) {
        dir_start_time = HAL_GetTick();
        sprintf(test_message, "Testing Direction Change (Forward <-> Backward)\r\n");
        Bluetooth_SendMessage(test_message);
    }
    
    uint32_t elapsed = HAL_GetTick() - dir_start_time;
    
    if(elapsed > 1000) {
        direction = !direction;
        dir_start_time = HAL_GetTick();
        
        if(direction) {
            TB6612_SetBothMotors(test_speed, test_speed);
            sprintf(test_message, "Direction: Forward\r\n");
        } else {
            TB6612_SetBothMotors(-test_speed, -test_speed);
            sprintf(test_message, "Direction: Backward\r\n");
        }
        Bluetooth_SendMessage(test_message);
    }
}

/**
 * @brief 测试刹车功能
 */
void MotorTest_Brake(void)
{
    TB6612_BrakeAllMotors();
    sprintf(test_message, "Testing Brake Function\r\n");
    Bluetooth_SendMessage(test_message);
}

/**
 * @brief 设置测试速度
 */
void MotorTest_SetSpeed(int16_t speed)
{
    if(speed < 0) speed = 0;
    if(speed > 1000) speed = 1000;
    
    test_speed = speed;
    sprintf(test_message, "Test speed set to: %d (%.1f%%)\r\n", speed, speed/10.0f);
    Bluetooth_SendMessage(test_message);
}

/**
 * @brief 获取当前测试状态
 */
const char* MotorTest_GetStatus(void)
{
    static char status[64];
    
    if(!test_running) {
        sprintf(status, "IDLE");
    } else {
        switch(current_test) {
            case TEST_LEFT_FORWARD: sprintf(status, "LEFT_FORWARD"); break;
            case TEST_LEFT_BACKWARD: sprintf(status, "LEFT_BACKWARD"); break;
            case TEST_RIGHT_FORWARD: sprintf(status, "RIGHT_FORWARD"); break;
            case TEST_RIGHT_BACKWARD: sprintf(status, "RIGHT_BACKWARD"); break;
            case TEST_BOTH_FORWARD: sprintf(status, "BOTH_FORWARD"); break;
            case TEST_BOTH_BACKWARD: sprintf(status, "BOTH_BACKWARD"); break;
            case TEST_SPEED_RAMP: sprintf(status, "SPEED_RAMP"); break;
            case TEST_DIRECTION_CHANGE: sprintf(status, "DIRECTION_CHANGE"); break;
            case TEST_BRAKE: sprintf(status, "BRAKE"); break;
            case TEST_COMPLETE: sprintf(status, "COMPLETE"); break;
            default: sprintf(status, "UNKNOWN"); break;
        }
    }
    
    return status;
}

/**
 * @brief 电机测试更新函数 (在主循环中调用)
 */
void MotorTest_Update(void)
{
    if(!test_running) return;
    
    uint32_t current_time = HAL_GetTick();
    uint32_t elapsed = current_time - test_start_time;
    
    // 检查当前测试是否完成
    if(elapsed >= test_duration) {
        // 进入下一个测试
        current_test++;
        test_start_time = current_time;
        
        // 停止当前电机
        TB6612_StopAllMotors();
        HAL_Delay(500); // 短暂停顿
        
        switch(current_test) {
            case TEST_LEFT_FORWARD:
                MotorTest_LeftForward(test_speed);
                break;
                
            case TEST_LEFT_BACKWARD:
                MotorTest_LeftBackward(test_speed);
                break;
                
            case TEST_RIGHT_FORWARD:
                MotorTest_RightForward(test_speed);
                break;
                
            case TEST_RIGHT_BACKWARD:
                MotorTest_RightBackward(test_speed);
                break;
                
            case TEST_BOTH_FORWARD:
                MotorTest_BothForward(test_speed);
                break;
                
            case TEST_BOTH_BACKWARD:
                MotorTest_BothBackward(test_speed);
                break;
                
            case TEST_SPEED_RAMP:
                test_duration = 5000; // 速度渐变测试需要5秒
                MotorTest_SpeedRamp();
                break;
                
            case TEST_DIRECTION_CHANGE:
                test_duration = 6000; // 方向切换测试需要6秒
                MotorTest_DirectionChange();
                break;
                
            case TEST_BRAKE:
                test_duration = 1000; // 刹车测试1秒
                MotorTest_Brake();
                break;
                
            case TEST_COMPLETE:
                TB6612_StopAllMotors();
                test_running = 0;
                sprintf(test_message, "\r\n=== Motor Test Complete ===\r\n");
                Bluetooth_SendMessage(test_message);
                sprintf(test_message, "All tests finished. Motors stopped.\r\n");
                Bluetooth_SendMessage(test_message);
                break;
                
            default:
                test_running = 0;
                TB6612_StopAllMotors();
                break;
        }
        
        // 重置测试持续时间为默认值
        if(current_test != TEST_SPEED_RAMP && current_test != TEST_DIRECTION_CHANGE && current_test != TEST_BRAKE) {
            test_duration = 2000;
        }
    }
    
    // 特殊测试的持续更新
    if(current_test == TEST_SPEED_RAMP) {
        MotorTest_SpeedRamp();
    } else if(current_test == TEST_DIRECTION_CHANGE) {
        MotorTest_DirectionChange();
    }
}

/**
 * @brief 手动测试命令处理
 */
void MotorTest_ProcessCommand(const char* cmd, const char* param)
{
    if(strcmp(cmd, "MTEST") == 0) {
        if(param == NULL) {
            MotorTest_StartAutoTest();
        } else if(strcmp(param, "STOP") == 0) {
            MotorTest_Stop();
        } else if(strcmp(param, "STATUS") == 0) {
            sprintf(test_message, "Motor Test Status: %s\r\n", MotorTest_GetStatus());
            Bluetooth_SendMessage(test_message);
        }
    }
    else if(strcmp(cmd, "MLEFT") == 0) {
        int16_t speed = test_speed;
        if(param != NULL) speed = atoi(param);
        MotorTest_LeftForward(speed);
    }
    else if(strcmp(cmd, "MRIGHT") == 0) {
        int16_t speed = test_speed;
        if(param != NULL) speed = atoi(param);
        MotorTest_RightForward(speed);
    }
    else if(strcmp(cmd, "MBOTH") == 0) {
        int16_t speed = test_speed;
        if(param != NULL) speed = atoi(param);
        MotorTest_BothForward(speed);
    }
    else if(strcmp(cmd, "MSTOP") == 0) {
        TB6612_StopAllMotors();
        sprintf(test_message, "All Motors Stopped\r\n");
        Bluetooth_SendMessage(test_message);
    }
    else if(strcmp(cmd, "MSPEED") == 0) {
        if(param != NULL) {
            MotorTest_SetSpeed(atoi(param));
        } else {
            sprintf(test_message, "Current test speed: %d\r\n", test_speed);
            Bluetooth_SendMessage(test_message);
        }
    }
}
