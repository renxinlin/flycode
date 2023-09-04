#ifndef __SENSORS_H
#define __SENSORS_H

#include "main.h"
#include "data.h"
#include "mpu.h"
#include "positionEstimate.h"

#define M_PI_F (float)3.14159265

#define MPU6500_GYRO_FS_250         0x00
#define MPU6500_GYRO_FS_500         0x01
#define MPU6500_GYRO_FS_1000        0x02
#define MPU6500_GYRO_FS_2000        0x03
#define MPU6500_ACCEL_FS_2        0x00
#define MPU6500_ACCEL_FS_16        0x03
/*

在 MPU6500_DEG_PER_LSB_2000 的定义中，将 2 倍的角速度范围（2000度/秒）除以 65536 是为了获得每个最低有效位（LSB）所代表的角度值。

MPU6500 是一款常用的加速度计和陀螺仪传感器芯片，它通过数字输出提供原始的角速度数据。该芯片使用 16 位的数据格式表示角速度，即最大值为 2^16-1 = 65535，其中包括正负范围。

为了将这个 16 位的数字值转换为实际的角速度值，需要知道每个最低有效位（LSB）所代表的角度值。因此，通过将 2 倍的角速度范围（2000度/秒）除以 65536，即可得到每个最低有效位（LSB）所代表的角度值。

具体计算如下：
每个最低有效位（LSB）所代表的角度值 = (2 * 2000.0) / 65536.0 ≈ 0.060975551

因此，通过这个计算可以得到 MPU6500 中角速度数据的转换比例，以便将原始数值转换为角速度值。


*/

#define MPU6500_G_PER_LSB_2      (float)((2 * 2) / 65536.0)
//#define MPU6500_G_PER_LSB_4      (float)((2 * 4) / 65536.0)
//#define MPU6500_G_PER_LSB_8      (float)((2 * 8) / 65536.0)
#define MPU6500_G_PER_LSB_16     (float)((2 * 16) / 65536.0)
	
//#define MPU6500_DEG_PER_LSB_250  (float)((2 * 250.0) / 65536.0)
//#define MPU6500_DEG_PER_LSB_500  (float)((2 * 500.0) / 65536.0)
//#define MPU6500_DEG_PER_LSB_1000 (float)((2 * 1000.0) / 65536.0)
#define MPU6500_DEG_PER_LSB_2000 (float)((2 * 2000.0) / 65536.0)
	
// 这里我们实际配置取2000与16 来确保精度的准确性
#define SENSORS_GYRO_FS_CFG       MPU6500_GYRO_FS_2000
#define SENSORS_DEG_PER_LSB_CFG   MPU6500_DEG_PER_LSB_2000

#define SENSORS_ACCEL_FS_CFG      MPU6500_ACCEL_FS_2
#define SENSORS_G_PER_LSB_CFG     MPU6500_G_PER_LSB_2

#define SENSORS_NBR_OF_BIAS_SAMPLES		1024	/* 计算方差的采样样本个数 */
#define GYRO_VARIANCE_BASE				8000 *100	/* 陀螺仪零偏方差阈值 */
#define SENSORS_ACC_SCALE_SAMPLES  		200		/* 加速计采样个数 */

// MPU9250主机模式读取数据 缓冲区长度

// 全局的传感器数据[陀螺仪与加速度]
extern sensor_data sensors;



typedef  struct
{
		int16_t x;
		int16_t y;
		int16_t z;
} Axis3i16;

typedef  struct
{
	
		int32_t x;
		int32_t y;
		int32_t z;

} Axis3i32;

typedef  struct
{
	
		int64_t x;
		int64_t y;
		int64_t z;

} Axis3i64;

typedef  struct 
{
	
		float x;
		float y;
		float z;
	
} Axis3f;

/**
  mpu6050 数据获取以及滤波
*/
void sensorsInit(void);
uint8_t sensorsDataGet(void);
static float lpf2pApply(lpf2pData* lpfData, float sample);
void lpf2pSetCutoffFreq(lpf2pData* lpfData, float sample_freq, float cutoff_freq);
void lpf2pInit(lpf2pData* lpfData, float sample_freq, float cutoff_freq);
static void applyAccLpf(lpf2pData *data, acc_data* in);
static void applyGyroLpf(lpf2pData *data, acc_data* in);

static uint8_t processAccScale(int16_t ax, int16_t ay, int16_t az);

static uint8_t processGyroBias(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut);
#endif
