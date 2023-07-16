#include "sensors.h"


typedef struct
{
	Axis3f     bias;
	uint8_t       isBiasValueFound;
	uint8_t       isBufferFilled;
	Axis3i16*  bufHead;
	Axis3i16   buffer[SENSORS_NBR_OF_BIAS_SAMPLES];
}BiasObj;


BiasObj	gyroBiasRunning;
// 暂时认为bmp280使用硬件滤波
// 不在进行软件滤波
// static sensorData_t sensors;
static Axis3i16	gyroRaw;
static Axis3i16	accRaw;
static Axis3f  gyroBias;

static float accScaleSum = 0;
static float accScale = 1;

static uint8_t gyroBiasFound = 0;
sensor_data sensors;


/*低通滤波参数*/
#define GYRO_LPF_CUTOFF_FREQ  80
#define ACCEL_LPF_CUTOFF_FREQ 30

static lpf2pData accLpf[3];
static lpf2pData gyroLpf[3];
static void sensorsBiasObjInit(BiasObj* bias);
static void sensorsCalculateVarianceAndMean(BiasObj* bias, Axis3f* varOut, Axis3f* meanOut);
static uint8_t sensorsFindBiasValue(BiasObj* bias);
static void sensorsAddBiasValue(BiasObj* bias, int16_t x, int16_t y, int16_t z);


void sensorsInit(){
		lpf2pInit(&gyroLpf[0], 1000, GYRO_LPF_CUTOFF_FREQ);
		lpf2pInit(&gyroLpf[1], 1000, GYRO_LPF_CUTOFF_FREQ);
		lpf2pInit(&gyroLpf[2], 1000, GYRO_LPF_CUTOFF_FREQ);
		lpf2pInit(&accLpf[0],  1000, ACCEL_LPF_CUTOFF_FREQ);
		lpf2pInit(&accLpf[0],  1000, ACCEL_LPF_CUTOFF_FREQ);
		lpf2pInit(&accLpf[0],  1000, ACCEL_LPF_CUTOFF_FREQ);
}

void sensorsDataGet(void)
{

	
		// 读取mpu 读取bmp280
		
			MPU_Get_Gyroscope(&gyroRaw.x,&gyroRaw.y,&gyroRaw.z);
			MPU_Get_Accelerometer(&accRaw.x,&accRaw.y,&accRaw.z);
			/*处理原始数据，并放入数据队列中*/
			gyroRaw.x -=  gyroBias.x;
			gyroRaw.y -=  gyroBias.y;
			gyroRaw.z -=  gyroBias.z;
		int16_t gx2 = accRaw.x;	
		gyroBiasFound = processGyroBias(gyroRaw.x, gyroRaw.y, gyroRaw.z, &gyroBias);
		
		if (gyroBiasFound)
		{
			processAccScale(accRaw.x, accRaw.y, accRaw.z);	/*计算accScale*/
		}
		
		sensors.gyro.x = -(gyroRaw.x - gyroBias.x) * SENSORS_DEG_PER_LSB_CFG;	/*单位 °/s */
		sensors.gyro.y =  (gyroRaw.y - gyroBias.y) * SENSORS_DEG_PER_LSB_CFG;
		sensors.gyro.z =  (gyroRaw.z - gyroBias.z) * SENSORS_DEG_PER_LSB_CFG;
		applyGyroLpf(gyroLpf, &sensors.gyro);	

		sensors.acc.x = -(accRaw.x) * SENSORS_G_PER_LSB_CFG / accScale;	/*单位 g(9.8m/s^2)*/
		sensors.acc.y =  (accRaw.y) * SENSORS_G_PER_LSB_CFG / accScale;	/*重力加速度缩放因子accScale 根据样本计算得出*/
		sensors.acc.z =  (accRaw.z) * SENSORS_G_PER_LSB_CFG / accScale;
		applyAccLpf(accLpf, &sensors.acc);
	
}

static uint8_t processGyroBias(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut)
{
	sensorsAddBiasValue(&gyroBiasRunning, gx, gy, gz);

	if (!gyroBiasRunning.isBiasValueFound)
	{
		sensorsFindBiasValue(&gyroBiasRunning);
	}

	gyroBiasOut->x = gyroBiasRunning.bias.x;
	gyroBiasOut->y = gyroBiasRunning.bias.y;
	gyroBiasOut->z = gyroBiasRunning.bias.z;

	return gyroBiasRunning.isBiasValueFound;
}

static uint8_t sensorsFindBiasValue(BiasObj* bias)
{
	uint8_t foundbias = 0;

	if (bias->isBufferFilled)
	{
		
		Axis3f mean;
		Axis3f variance;
		sensorsCalculateVarianceAndMean(bias, &variance, &mean);

		if (variance.x < GYRO_VARIANCE_BASE && variance.y < GYRO_VARIANCE_BASE && variance.z < GYRO_VARIANCE_BASE)
		{
			bias->bias.x = mean.x;
			bias->bias.y = mean.y;
			bias->bias.z = mean.z;
			foundbias = 1;
			bias->isBiasValueFound= 1;
		}else
			bias->isBufferFilled=0;
	}
	return foundbias;
}

static void sensorsCalculateVarianceAndMean(BiasObj* bias, Axis3f* varOut, Axis3f* meanOut)
{
	uint32_t i;
	int64_t sum[3] = {0};
	int64_t sumsq[3] = {0};

	for (i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++)
	{
		sum[0] += bias->buffer[i].x;
		sum[1] += bias->buffer[i].y;
		sum[2] += bias->buffer[i].z;
		sumsq[0] += bias->buffer[i].x * bias->buffer[i].x;
		sumsq[1] += bias->buffer[i].y * bias->buffer[i].y;
		sumsq[2] += bias->buffer[i].z * bias->buffer[i].z;
	}

	varOut->x = (sumsq[0] - ((int64_t)sum[0] * sum[0]) / SENSORS_NBR_OF_BIAS_SAMPLES);
	varOut->y = (sumsq[1] - ((int64_t)sum[1] * sum[1]) / SENSORS_NBR_OF_BIAS_SAMPLES);
	varOut->z = (sumsq[2] - ((int64_t)sum[2] * sum[2]) / SENSORS_NBR_OF_BIAS_SAMPLES);

	meanOut->x = (float)sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
	meanOut->y = (float)sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
	meanOut->z = (float)sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
}

/**
 * 根据样本计算重力加速度缩放因子
 */
static uint8_t processAccScale(int16_t ax, int16_t ay, int16_t az)
{
	static uint8_t accBiasFound = 0;
	static uint32_t accScaleSumCount = 0;

	if (!accBiasFound)
	{
		accScaleSum += sqrtf(powf(ax * SENSORS_G_PER_LSB_CFG, 2) + powf(ay * SENSORS_G_PER_LSB_CFG, 2) + powf(az * SENSORS_G_PER_LSB_CFG, 2));
		accScaleSumCount++;

		if (accScaleSumCount == SENSORS_ACC_SCALE_SAMPLES)
		{
			accScale = accScaleSum / SENSORS_ACC_SCALE_SAMPLES;
			accBiasFound = 1;
		}
	}

	return accBiasFound;
}


static void sensorsAddBiasValue(BiasObj* bias, int16_t x, int16_t y, int16_t z)
{
	bias->bufHead->x = x;
	bias->bufHead->y = y;
	bias->bufHead->z = z;
	bias->bufHead++;

	if (bias->bufHead >= &bias->buffer[SENSORS_NBR_OF_BIAS_SAMPLES])
	{
		bias->bufHead = bias->buffer;
		bias->isBufferFilled = 1;
	}
}




	
/*二阶低通滤波*/
static void applyAccLpf(lpf2pData *data, acc_data* in)
{
		in->x = lpf2pApply(&data[0], in->x);
		in->y = lpf2pApply(&data[1], in->y);
		in->z = lpf2pApply(&data[2], in->z);
}


static void applyGyroLpf(lpf2pData *data, acc_data* in)
{
		in->x = lpf2pApply(&data[0], in->x);
		in->y = lpf2pApply(&data[1], in->y);
		in->z = lpf2pApply(&data[2], in->z);
}


float lpf2pApply(lpf2pData* lpfData, float sample)
{
	float delay_element_0 = sample - lpfData->delay_element_1 * lpfData->a1 - lpfData->delay_element_2 * lpfData->a2;
	if (!isfinite(delay_element_0)) 
	{
		// don't allow bad values to propigate via the filter
		delay_element_0 = sample;
	}

	float output = delay_element_0 * lpfData->b0 + lpfData->delay_element_1 * lpfData->b1 + lpfData->delay_element_2 * lpfData->b2;

	lpfData->delay_element_2 = lpfData->delay_element_1;
	lpfData->delay_element_1 = delay_element_0;
	return output;
}



void lpf2pInit(lpf2pData* lpfData, float sample_freq, float cutoff_freq)
{
	if (lpfData == NULL || cutoff_freq <= 0.0f) 
	{
		return;
	}

	lpf2pSetCutoffFreq(lpfData, sample_freq, cutoff_freq);
}

/**
 * 设置二阶低通滤波截至频率
 */
void lpf2pSetCutoffFreq(lpf2pData* lpfData, float sample_freq, float cutoff_freq)
{
	float fr = sample_freq/cutoff_freq;
	float ohm = tanf(M_PI_F/fr);
	float c = 1.0f+2.0f*cosf(M_PI_F/4.0f)*ohm+ohm*ohm;
	lpfData->b0 = ohm*ohm/c;
	lpfData->b1 = 2.0f*lpfData->b0;
	lpfData->b2 = lpfData->b0;
	lpfData->a1 = 2.0f*(ohm*ohm-1.0f)/c;
	lpfData->a2 = (1.0f-2.0f*cosf(M_PI_F/4.0f)*ohm+ohm*ohm)/c;
	lpfData->delay_element_1 = 0.0f;
	lpfData->delay_element_2 = 0.0f;
}
