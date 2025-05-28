/**
 * @file    mpu6050.h
 * @brief   MPU6050六轴传感器模块头文件
 * @details 该文件定义了MPU6050六轴传感器(三轴加速度计+三轴陀螺仪)的
 *          接口函数、数据结构和寄存器定义。MPU6050是平衡车姿态检测
 *          的核心传感器，用于测量车体的倾斜角度和角速度，为平衡
 *          控制算法提供关键的反馈信息。模块集成了卡尔曼滤波算法，
 *          融合加速度计和陀螺仪数据，获得精确稳定的姿态角度。
 *
 * @author  STM32平衡车项目组
 * @version 1.0
 * @date    2024
 *
 * @note    使用I2C接口通信，支持卡尔曼滤波和互补滤波算法
 */

#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <math.h>

/*==============================================================================
                                I2C通信地址定义
==============================================================================*/

/**
 * @brief MPU6050 I2C设备地址
 * @note  MPU6050的I2C地址由AD0引脚决定，通常为0x68或0x69
 */
#define MPU6050_I2C_ADDR        0x68                            // MPU6050基础I2C地址
#define MPU6050_I2C_ADDR_WRITE  (MPU6050_I2C_ADDR << 1)        // 写地址：0xD0
#define MPU6050_I2C_ADDR_READ   ((MPU6050_I2C_ADDR << 1) | 1)  // 读地址：0xD1

/*==============================================================================
                                寄存器地址定义
==============================================================================*/

/**
 * @brief MPU6050关键寄存器地址定义
 * @details 定义了配置和数据读取所需的主要寄存器地址
 */
#define MPU6050_REG_PWR_MGMT_1  0x6B    // 电源管理寄存器1：控制设备睡眠和时钟源
#define MPU6050_REG_SMPLRT_DIV  0x19    // 采样率分频寄存器：设置输出数据率
#define MPU6050_REG_CONFIG      0x1A    // 配置寄存器：设置DLPF和外部同步
#define MPU6050_REG_GYRO_CONFIG 0x1B    // 陀螺仪配置寄存器：设置量程和自检
#define MPU6050_REG_ACCEL_CONFIG 0x1C   // 加速度计配置寄存器：设置量程和自检

/**
 * @brief 加速度计数据寄存器地址
 * @note  加速度计数据为16位有符号整数，高字节在前
 */
#define MPU6050_REG_ACCEL_XOUT_H 0x3B   // X轴加速度高字节
#define MPU6050_REG_ACCEL_XOUT_L 0x3C   // X轴加速度低字节
#define MPU6050_REG_ACCEL_YOUT_H 0x3D   // Y轴加速度高字节
#define MPU6050_REG_ACCEL_YOUT_L 0x3E   // Y轴加速度低字节
#define MPU6050_REG_ACCEL_ZOUT_H 0x3F   // Z轴加速度高字节
#define MPU6050_REG_ACCEL_ZOUT_L 0x40   // Z轴加速度低字节

/**
 * @brief 温度传感器数据寄存器地址
 * @note  温度数据为16位有符号整数，可用于温度补偿
 */
#define MPU6050_REG_TEMP_OUT_H  0x41    // 温度传感器高字节
#define MPU6050_REG_TEMP_OUT_L  0x42    // 温度传感器低字节

/**
 * @brief 陀螺仪数据寄存器地址
 * @note  陀螺仪数据为16位有符号整数，高字节在前
 */
#define MPU6050_REG_GYRO_XOUT_H 0x43    // X轴角速度高字节
#define MPU6050_REG_GYRO_XOUT_L 0x44    // X轴角速度低字节
#define MPU6050_REG_GYRO_YOUT_H 0x45    // Y轴角速度高字节
#define MPU6050_REG_GYRO_YOUT_L 0x46    // Y轴角速度低字节
#define MPU6050_REG_GYRO_ZOUT_H 0x47    // Z轴角速度高字节
#define MPU6050_REG_GYRO_ZOUT_L 0x48    // Z轴角速度低字节

/**
 * @brief 设备识别寄存器
 * @note  用于验证设备是否为MPU6050，正常值应为0x68
 */
#define MPU6050_REG_WHO_AM_I    0x75    // 设备ID寄存器：返回设备地址

/*==============================================================================
                                数据类型定义
==============================================================================*/

/**
 * @brief MPU6050原始数据结构
 * @details 存储从MPU6050寄存器直接读取的16位原始数据，
 *          包括三轴加速度、三轴角速度和温度的原始ADC值。
 */
typedef struct {
    int16_t accel_x_raw;    // X轴加速度原始值：沿车体前后方向
    int16_t accel_y_raw;    // Y轴加速度原始值：沿车体左右方向
    int16_t accel_z_raw;    // Z轴加速度原始值：沿车体上下方向
    int16_t temp_raw;       // 温度传感器原始值：用于温度补偿
    int16_t gyro_x_raw;     // X轴角速度原始值：绕前后轴旋转(俯仰)
    int16_t gyro_y_raw;     // Y轴角速度原始值：绕左右轴旋转(横滚)
    int16_t gyro_z_raw;     // Z轴角速度原始值：绕上下轴旋转(偏航)
} MPU6050_RawData_t;

/**
 * @brief MPU6050处理后数据结构
 * @details 存储经过单位转换和滤波处理后的物理量数据，
 *          包括加速度(g)、角速度(°/s)、温度(°C)和姿态角(°)。
 */
typedef struct {
    float accel_x;          // X轴加速度(g)：重力加速度为单位
    float accel_y;          // Y轴加速度(g)：重力加速度为单位
    float accel_z;          // Z轴加速度(g)：重力加速度为单位
    float temp;             // 温度(°C)：摄氏度
    float gyro_x;           // X轴角速度(°/s)：度每秒
    float gyro_y;           // Y轴角速度(°/s)：度每秒
    float gyro_z;           // Z轴角速度(°/s)：度每秒
    float pitch;            // 俯仰角(°)：前后倾斜角度，平衡车主要控制角度
    float roll;             // 横滚角(°)：左右倾斜角度
    float yaw;              // 偏航角(°)：水平旋转角度
} MPU6050_Data_t;

/**
 * @brief 卡尔曼滤波器数据结构
 * @details 卡尔曼滤波器用于融合加速度计和陀螺仪数据，
 *          获得更加稳定准确的姿态角度估计。
 */
typedef struct {
    float Q_angle;          // 过程噪声协方差：角度噪声，影响滤波器对陀螺仪的信任度
    float Q_bias;           // 过程噪声协方差：零偏噪声，陀螺仪零偏的变化率
    float R_measure;        // 测量噪声协方差：加速度计噪声，影响对加速度计的信任度
    float angle;            // 当前最优角度估计：滤波器输出的角度值
    float bias;             // 当前陀螺仪零偏估计：陀螺仪的系统误差
    float P[2][2];          // 误差协方差矩阵：表示估计的不确定性
} Kalman_t;

/*==============================================================================
                                函数声明
==============================================================================*/

/**
 * @brief 初始化MPU6050传感器
 * @return 初始化结果：0-成功，非0-失败
 * @note  配置传感器量程、采样率、滤波器等参数
 */
uint8_t MPU6050_Init(void);

/**
 * @brief 读取MPU6050原始数据
 * @param raw_data 原始数据结构指针
 * @return 读取结果：0-成功，非0-失败
 * @note  从传感器寄存器读取16位原始ADC值
 */
uint8_t MPU6050_ReadRawData(MPU6050_RawData_t* raw_data);

/**
 * @brief 处理原始数据转换为物理量
 * @param raw_data 原始数据指针
 * @param data 处理后数据指针
 * @note  进行单位转换、零偏校正和滤波处理
 */
void MPU6050_ProcessData(MPU6050_RawData_t* raw_data, MPU6050_Data_t* data);

/**
 * @brief 获取俯仰角
 * @return 当前俯仰角度值(°)
 * @note  平衡车的主要控制角度，0°为直立状态
 */
float MPU6050_GetPitch(void);

/**
 * @brief 获取横滚角
 * @return 当前横滚角度值(°)
 * @note  车体左右倾斜角度，用于检测侧翻
 */
float MPU6050_GetRoll(void);

/**
 * @brief 获取偏航角
 * @return 当前偏航角度值(°)
 * @note  车体水平旋转角度，用于方向控制
 */
float MPU6050_GetYaw(void);

/**
 * @brief 更新MPU6050数据
 * @note  读取传感器数据并进行滤波处理，应定期调用
 */
void MPU6050_Update(void);

/**
 * @brief 卡尔曼滤波器角度计算
 * @param kalman 卡尔曼滤波器指针
 * @param newAngle 加速度计计算的新角度
 * @param newRate 陀螺仪测量的角速度
 * @param dt 时间间隔(s)
 * @return 滤波后的最优角度估计
 * @note  融合加速度计和陀螺仪数据的核心算法
 */
float Kalman_GetAngle(Kalman_t* kalman, float newAngle, float newRate, float dt);

/**
 * @brief 初始化卡尔曼滤波器
 * @param kalman 卡尔曼滤波器指针
 * @note  设置滤波器初始参数和协方差矩阵
 */
void Kalman_Init(Kalman_t* kalman);

/*==============================================================================
                                全局变量声明
==============================================================================*/

/**
 * @brief MPU6050处理后数据实例
 * @note  存储当前传感器的所有物理量数据和姿态角
 */
extern MPU6050_Data_t MPU6050_Data;

/**
 * @brief X轴(俯仰)卡尔曼滤波器实例
 * @note  用于平衡车前后倾斜角度的精确估计
 */
extern Kalman_t KalmanX;

/**
 * @brief Y轴(横滚)卡尔曼滤波器实例
 * @note  用于平衡车左右倾斜角度的精确估计
 */
extern Kalman_t KalmanY;

/*
MPU6050内部所有寄存器地址
#define MPU_SELF_TESTX_REG		0X0D	//自检寄存器X
#define MPU_SELF_TESTY_REG		0X0E	//自检寄存器Y
#define MPU_SELF_TESTZ_REG		0X0F	//自检寄存器Z
#define MPU_SELF_TESTA_REG		0X10	//自检寄存器A
#define MPU_SAMPLE_RATE_REG		0X19	//采样频率分频器
#define MPU_CFG_REG				0X1A	//配置寄存器
#define MPU_GYRO_CFG_REG		0X1B	//陀螺仪配置寄存器
#define MPU_ACCEL_CFG_REG		0X1C	//加速度计配置寄存器
#define MPU_MOTION_DET_REG		0X1F	//运动检测阀值设置寄存器
#define MPU_FIFO_EN_REG			0X23	//FIFO使能寄存器
#define MPU_I2CMST_CTRL_REG		0X24	//IIC主机控制寄存器
#define MPU_I2CSLV0_ADDR_REG	0X25	//IIC从机0器件地址寄存器
#define MPU_I2CSLV0_REG			0X26	//IIC从机0数据地址寄存器
#define MPU_I2CSLV0_CTRL_REG	0X27	//IIC从机0控制寄存器
#define MPU_I2CSLV1_ADDR_REG	0X28	//IIC从机1器件地址寄存器
#define MPU_I2CSLV1_REG			0X29	//IIC从机1数据地址寄存器
#define MPU_I2CSLV1_CTRL_REG	0X2A	//IIC从机1控制寄存器
#define MPU_I2CSLV2_ADDR_REG	0X2B	//IIC从机2器件地址寄存器
#define MPU_I2CSLV2_REG			0X2C	//IIC从机2数据地址寄存器
#define MPU_I2CSLV2_CTRL_REG	0X2D	//IIC从机2控制寄存器
#define MPU_I2CSLV3_ADDR_REG	0X2E	//IIC从机3器件地址寄存器
#define MPU_I2CSLV3_REG			0X2F	//IIC从机3数据地址寄存器
#define MPU_I2CSLV3_CTRL_REG	0X30	//IIC从机3控制寄存器
#define MPU_I2CSLV4_ADDR_REG	0X31	//IIC从机4器件地址寄存器
#define MPU_I2CSLV4_REG			0X32	//IIC从机4数据地址寄存器
#define MPU_I2CSLV4_DO_REG		0X33	//IIC从机4写数据寄存器
#define MPU_I2CSLV4_CTRL_REG	0X34	//IIC从机4控制寄存器
#define MPU_I2CSLV4_DI_REG		0X35	//IIC从机4读数据寄存器
#define MPU_I2CMST_STA_REG		0X36	//IIC主机状态寄存器
#define MPU_INTBP_CFG_REG		0X37	//中断/旁路设置寄存器
#define MPU_INT_EN_REG			0X38	//中断使能寄存器
#define MPU_INT_STA_REG			0X3A	//中断状态寄存器
#define MPU_ACCEL_XOUTH_REG		0X3B	//加速度值,X轴高8位寄存器
#define MPU_ACCEL_XOUTL_REG		0X3C	//加速度值,X轴低8位寄存器
#define MPU_ACCEL_YOUTH_REG		0X3D	//加速度值,Y轴高8位寄存器
#define MPU_ACCEL_YOUTL_REG		0X3E	//加速度值,Y轴低8位寄存器
#define MPU_ACCEL_ZOUTH_REG		0X3F	//加速度值,Z轴高8位寄存器
#define MPU_ACCEL_ZOUTL_REG		0X40	//加速度值,Z轴低8位寄存器
#define MPU_TEMP_OUTH_REG		0X41	//温度值高八位寄存器
#define MPU_TEMP_OUTL_REG		0X42	//温度值低8位寄存器
#define MPU_GYRO_XOUTH_REG		0X43	//陀螺仪值,X轴高8位寄存器
#define MPU_GYRO_XOUTL_REG		0X44	//陀螺仪值,X轴低8位寄存器
#define MPU_GYRO_YOUTH_REG		0X45	//陀螺仪值,Y轴高8位寄存器
#define MPU_GYRO_YOUTL_REG		0X46	//陀螺仪值,Y轴低8位寄存器
#define MPU_GYRO_ZOUTH_REG		0X47	//陀螺仪值,Z轴高8位寄存器
#define MPU_GYRO_ZOUTL_REG		0X48	//陀螺仪值,Z轴低8位寄存器
#define MPU_I2CSLV0_DO_REG		0X63	//IIC从机0数据寄存器
#define MPU_I2CSLV1_DO_REG		0X64	//IIC从机1数据寄存器
#define MPU_I2CSLV2_DO_REG		0X65	//IIC从机2数据寄存器
#define MPU_I2CSLV3_DO_REG		0X66	//IIC从机3数据寄存器
#define MPU_I2CMST_DELAY_REG	0X67	//IIC主机延时管理寄存器
#define MPU_SIGPATH_RST_REG		0X68	//信号通道复位寄存器
#define MPU_MDETECT_CTRL_REG	0X69	//运动检测控制寄存器
#define MPU_USER_CTRL_REG		0X6A	//用户控制寄存器
#define MPU_PWR_MGMT1_REG		0X6B	//电源管理寄存器1
#define MPU_PWR_MGMT2_REG		0X6C	//电源管理寄存器2
#define MPU_FIFO_CNTH_REG		0X72	//FIFO计数寄存器高八位
#define MPU_FIFO_CNTL_REG		0X73	//FIFO计数寄存器低八位
#define MPU_FIFO_RW_REG			0X74	//FIFO读写寄存器
#define MPU_DEVICE_ID_REG		0X75	//器件ID寄存器
*/

#endif // __MPU6050_H
