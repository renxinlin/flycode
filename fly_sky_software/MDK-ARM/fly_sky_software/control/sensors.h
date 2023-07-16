#ifndef __SENSORS_H
#define __SENSORS_H

#include "main.h"
#include "data.h"
#include "mpu.h"
#include "positionEstimate.h"

#define M_PI_F (float)3.14159265

// ��������ʵ������ȡ2000��16 ��ȷ�����ȵ�׼ȷ��
#define SENSORS_GYRO_FS_CFG      1 // MPU6500_GYRO_FS_2000
#define SENSORS_DEG_PER_LSB_CFG   1// MPU6500_DEG_PER_LSB_2000

#define SENSORS_ACCEL_FS_CFG     1 // MPU6500_ACCEL_FS_16	
#define SENSORS_G_PER_LSB_CFG     1// MPU6500_G_PER_LSB_16

#define SENSORS_NBR_OF_BIAS_SAMPLES		1024	/* ���㷽��Ĳ����������� */
#define GYRO_VARIANCE_BASE				4000	/* ��������ƫ������ֵ */
#define SENSORS_ACC_SCALE_SAMPLES  		200		/* ���ټƲ������� */

// MPU9250����ģʽ��ȡ���� ����������

// ȫ�ֵĴ���������[����������ٶ�]
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
  mpu6050 ���ݻ�ȡ�Լ��˲�
*/
void sensorsDataGet(void);
static float lpf2pApply(lpf2pData* lpfData, float sample);
void lpf2pSetCutoffFreq(lpf2pData* lpfData, float sample_freq, float cutoff_freq);
void lpf2pInit(lpf2pData* lpfData, float sample_freq, float cutoff_freq);
static void applyAccLpf(lpf2pData *data, acc_data* in);
static void applyGyroLpf(lpf2pData *data, acc_data* in);

static uint8_t processAccScale(int16_t ax, int16_t ay, int16_t az);

static uint8_t processGyroBias(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut);
#endif