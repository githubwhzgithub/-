
#include "mpu6050.h"
#include <math.h> // For atan2 and sqrt

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Kalman filter instances for pitch and roll
static Kalman_t kalmanX; // For Roll
static Kalman_t kalmanY; // For Pitch

static uint32_t g_timer; // Timer for dt calculation

// Static I2C handle pointer for MPU6050 operations
static I2C_HandleTypeDef *g_hi2c_mpu6050 = NULL;
 
/**
* @brief 		MPU6050初始化函数
* @alter		无
* @param		无
* @retval 		成功返回0，失败返回1
*/
uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx)
{
	g_hi2c_mpu6050 = I2Cx; // Store the I2C handle

	uint8_t check;
	uint8_t Data;
 
	// check device ID WHO_AM_I
	if (HAL_I2C_Mem_Read(g_hi2c_mpu6050, MPU6050_ADDR, MPU_DEVICE_ID_REG, 1, &check, 1, I2C_TimeOut) != HAL_OK)
	{
		return 1; // I2C Read Error
	}

	// 0x68 will be returned by the sensor if everything goes well
	if (check == 104) 
	{
			// power management register 0X6B we should write all 0's to wake the sensor up
			Data = 0;
			if (HAL_I2C_Mem_Write(g_hi2c_mpu6050, MPU6050_ADDR, MPU_PWR_MGMT1_REG, 1, &Data, 1, I2C_TimeOut) != HAL_OK)
			{
				return 1; // I2C Write Error
			}
 
			// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
			Data = 0x07;
			if (HAL_I2C_Mem_Write(g_hi2c_mpu6050, MPU6050_ADDR, MPU_SAMPLE_RATE_REG, 1, &Data, 1, I2C_TimeOut) != HAL_OK)
			{
				return 1; // I2C Write Error
			}
 
			// Set accelerometer configuration in ACCEL_CONFIG Register
			// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 ->   2g
			Data = 0x00;
			if (HAL_I2C_Mem_Write(g_hi2c_mpu6050, MPU6050_ADDR, MPU_ACCEL_CFG_REG, 1, &Data, 1, I2C_TimeOut) != HAL_OK)
			{
				return 1; // I2C Write Error
			}
 
			// Set Gyroscopic configuration in GYRO_CONFIG Register
			// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 ->   250  /s
			Data = 0x00;
			if (HAL_I2C_Mem_Write(g_hi2c_mpu6050, MPU6050_ADDR, MPU_GYRO_CFG_REG, 1, &Data, 1, I2C_TimeOut) != HAL_OK)
			{
				return 1; // I2C Write Error
			}
			return 0; // Success
	}
	return 1; // Device ID check failed
}
 
 
/**
* @brief 		MPU6050温度值获取函数
* @alter		无
* @param		无
* @retval 		温度值
*/
float MPU_Get_Temperature(void)
{
	uint8_t buf[2]; 
	short raw;
	float temp = 0.0f; // Default value in case of error

	if (g_hi2c_mpu6050 == NULL) return temp; // I2C not initialized

	if (HAL_I2C_Mem_Read(g_hi2c_mpu6050, MPU6050_ADDR, MPU_TEMP_OUTH_REG, 1, buf, 2, I2C_TimeOut) == HAL_OK)
	{
		raw=((int16_t)buf[0]<<8)|buf[1];  
		temp=36.53+((double)raw)/340;  
	}
	return temp;
}
 
/**
* @brief 		MPU6050陀螺仪值获取函数(三轴原始值)
* @alter		无
* @param		gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
* @retval 		正常：0，错误：其他
*/
uint8_t MPU_Get_RAW_Gyroscope(int16_t *gx,int16_t *gy,int16_t *gz)
{
	uint8_t buf[6],res;  

	if (g_hi2c_mpu6050 == NULL) return 1; // I2C not initialized

	res = HAL_I2C_Mem_Read(g_hi2c_mpu6050, MPU6050_ADDR, MPU_GYRO_XOUTH_REG, 1, buf, 6, I2C_TimeOut);
	if(res == HAL_OK)
	{
		*gx=((int16_t)buf[0]<<8)|buf[1];  
		*gy=((int16_t)buf[2]<<8)|buf[3];  
		*gz=((int16_t)buf[4]<<8)|buf[5];
		return 0; // Success
	} 	
    return 1; // Error
}

/**
* @brief 		MPU6050加速度值获取函数(转换后)
* @alter		无
* @param		ax,ay,az:加速度计x,y,z轴的读数(单位g)
* @retval 		正常：0，错误：其他
*/
uint8_t MPU_Get_Accelerometer(float *ax,float *ay,float *az)
{
	int16_t raw_ax, raw_ay, raw_az;
	uint8_t res;
	res = MPU_Get_RAW_Accelerometer(&raw_ax, &raw_ay, &raw_az);
	if(res == 0)
	{
		*ax = (float)raw_ax / 16384.0f;
		*ay = (float)raw_ay / 16384.0f;
		*az = (float)raw_az / 16384.0f;
	}
	return res;
}

/**
* @brief 		MPU6050陀螺仪值获取函数(转换后)
* @alter		无
* @param		gx,gy,gz:陀螺仪x,y,z轴的读数(单位°/s)
* @retval 		正常：0，错误：其他
*/
uint8_t MPU_Get_Gyroscope(float *gx,float *gy,float *gz)
{
	int16_t raw_gx, raw_gy, raw_gz;
	uint8_t res;
	res = MPU_Get_RAW_Gyroscope(&raw_gx, &raw_gy, &raw_gz);
	if(res == 0)
	{
		*gx = (float)raw_gx / 131.0f;
		*gy = (float)raw_gy / 131.0f;
		*gz = (float)raw_gz / 131.0f;
	}
	return res;
}


/**
* @brief 		MPU6050加速度值获取函数(三轴原始值)
* @alter		无
* @param		ax,ay,az:加速度计x,y,z轴的原始读数(带符号)
* @retval 		正常：0，错误：其他
*/
uint8_t MPU_Get_RAW_Accelerometer(int16_t *ax,int16_t *ay,int16_t *az)
{
	uint8_t buf[6],res;  

	if (g_hi2c_mpu6050 == NULL) return 1; // I2C not initialized

	res = HAL_I2C_Mem_Read(g_hi2c_mpu6050, MPU6050_ADDR, MPU_ACCEL_XOUTH_REG, 1, buf, 6, I2C_TimeOut);
	if(res == HAL_OK)
	{
		*ax=((int16_t)buf[0]<<8)|buf[1];  
		*ay=((int16_t)buf[2]<<8)|buf[3];  
		*az=((int16_t)buf[4]<<8)|buf[5];
		return 0; // Success
	} 	
    return 1; // Error
}

// 卡尔曼滤波相关实现
void Kalman_Init(void)
{
    kalmanX.Q_angle = 0.001f;
    kalmanX.Q_bias = 0.003f;
    kalmanX.R_measure = 0.03f;
    kalmanX.angle = 0.0f;
    kalmanX.bias = 0.0f;
    kalmanX.P[0][0] = 0.0f;
    kalmanX.P[0][1] = 0.0f;
    kalmanX.P[1][0] = 0.0f;
    kalmanX.P[1][1] = 0.0f;

    kalmanY.Q_angle = 0.001f;
    kalmanY.Q_bias = 0.003f;
    kalmanY.R_measure = 0.03f;
    kalmanY.angle = 0.0f;
    kalmanY.bias = 0.0f;
    kalmanY.P[0][0] = 0.0f;
    kalmanY.P[0][1] = 0.0f;
    kalmanY.P[1][0] = 0.0f;
    kalmanY.P[1][1] = 0.0f;
    g_timer = HAL_GetTick();
}

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
    // 预测
    Kalman->angle += dt * (newRate - Kalman->bias);
    Kalman->P[0][0] += dt * (dt*Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    // 更新
    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;
    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;
    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];
    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;
    return Kalman->angle;
}

void MPU_Get_Filtered_Angles(float *pitch, float *roll)
{
    float ax, ay, az, gx, gy, gz;
    if (MPU_Get_Accelerometer(&ax, &ay, &az) != 0 || MPU_Get_Gyroscope(&gx, &gy, &gz) != 0)
    {
        *pitch = 0.0f;
        *roll = 0.0f;
        return;
    }
    float acc_pitch = atan2f(ax, sqrtf(ay*ay + az*az)) * 180.0f / M_PI;
    float acc_roll = atan2f(ay, sqrtf(ax*ax + az*az)) * 180.0f / M_PI;
    uint32_t now = HAL_GetTick();
    float dt = (now - g_timer) / 1000.0f;
    if (dt <= 0.0f || dt > 1.0f) dt = 0.01f;
    g_timer = now;
    *pitch = (float)Kalman_getAngle(&kalmanY, acc_pitch, gy, dt);
    *roll = (float)Kalman_getAngle(&kalmanX, acc_roll, gx, dt);
}