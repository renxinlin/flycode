#include <math.h>
#include "stdio.h"
#include "bmp280.h"
#include "iic.h"

/*bmp280  采样频率ʽ*/
#define BMP280_PRESSURE_OSR			(BMP280_OVERSAMP_16X)
#define BMP280_TEMPERATURE_OSR		(BMP280_OVERSAMP_16X)
// 10110111
#define BMP280_MODE					(BMP280_PRESSURE_OSR << 2 | BMP280_TEMPERATURE_OSR << 5 | BMP280_NORMAL_MODE)


static uint8_t devAddr;

typedef struct 
{
    uint16_t dig_T1;	/* calibration T1 data */
    uint16_t dig_T2; /* calibration T2 data */
    uint16_t dig_T3; /* calibration T3 data */
    uint16_t dig_P1;	/* calibration P1 data */
    uint16_t dig_P2; /* calibration P2 data */
    uint16_t dig_P3; /* calibration P3 data */
    uint16_t dig_P4; /* calibration P4 data */
    uint16_t dig_P5; /* calibration P5 data */
    uint16_t dig_P6; /* calibration P6 data */
    uint16_t dig_P7; /* calibration P7 data */
    uint16_t dig_P8; /* calibration P8 data */
    uint16_t dig_P9; /* calibration P9 data */
    uint16_t t_fine; /* calibration t_fine data */
} bmp280Calib;

bmp280Calib  bmp280Cal;

static uint8_t bmp280ID = 0;
static uint32_t bmp280RawPressure = 0;
static uint32_t bmp280RawTemperature = 0;

static void bmp280GetPressure(void);





uint8_t bmp280Init()
{	
	devAddr = BMP280_I2C_ADDR;
  HAL_Delay(50);
	IIC_Read_Len(devAddr, BMP280_CHIP_ID, 1,&bmp280ID);	
	if(bmp280ID == BMP280_DEFAULT_CHIP_ID)
		printf("BMP280 ID IS: 0x%X\n",bmp280ID);
  else 
    return 1;
  IIC_Read_Len(devAddr, BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG, 24, (uint8_t *)&bmp280Cal);	 
	printf("dig_T1 = %d\n", bmp280Cal.dig_T1);
    printf("dig_T2 = %d\n", bmp280Cal.dig_T2);
    printf("dig_T3 = %d\n", bmp280Cal.dig_T3);
    printf("dig_P1 = %d\n", bmp280Cal.dig_P1);
    printf("dig_P2 = %d\n", bmp280Cal.dig_P2);
    printf("dig_P3 = %d\n", bmp280Cal.dig_P3);
    printf("dig_P4 = %d\n", bmp280Cal.dig_P4);
    printf("dig_P5 = %d\n", bmp280Cal.dig_P5);
    printf("dig_P6 = %d\n", bmp280Cal.dig_P6);
    printf("dig_P7 = %d\n", bmp280Cal.dig_P7);
    printf("dig_P8 = %d\n", bmp280Cal.dig_P8);
    printf("dig_P9 = %d\n", bmp280Cal.dig_P9);
		uint8_t reset =BMP280_RESET_VALUE ;
    IIC_Write_Len(devAddr,BMP280_RST_REG,1,&reset);    //往复位寄存器写入给定值
	IIC_Read_Len(devAddr,BMP280_RST_REG,1,&reset);
  printf("t_fine = %d\n", bmp280Cal.t_fine);
	HAL_Delay(100);
	//uint8_t mode = BMP280_MODE;
	uint8_t mode = 0xff;
	// 采样率和工作模式
  IIC_Write_Len( devAddr, BMP280_CTRL_MEAS_REG, 1,&mode);
	//uint8_t filterdata = 5<<2;
	uint8_t filterdata = 0x0C;
		// 滤波配置
  IIC_Write_Len( devAddr, BMP280_CONFIG_REG,1,&filterdata);	 

  return 0;
}

static void bmp280GetPressure(void)
{
    uint8_t data[BMP280_DATA_FRAME_SIZE];
    uint8_t data1,data2,data3,data4,data5,data6;

    // read data from sensor
	  IIC_Read_Len(devAddr, BMP280_PRESSURE_MSB_REG, BMP280_DATA_FRAME_SIZE, data);	 
 
    bmp280RawPressure = (uint32_t)((((uint32_t)(data[0])) << 12) | (((uint32_t)(data[1])) << 4) | ((uint32_t)data[2] >> 4));
    bmp280RawTemperature = (uint32_t)((((uint32_t)(data[3])) << 12) | (((uint32_t)(data[4])) << 4) | ((uint32_t)data[5] >> 4));
	 
}





float bmp280_compensate_pressure(int32_t adc_P)
{
    float var1, var2, p;

    var1 = ((float)bmp280Cal.t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * (float)bmp280Cal.dig_P6 / 32768.0;
    var2 = var2 + var1 * (float)bmp280Cal.dig_P5 * 2.0;
    var2 = (var2 / 4.0) + ((float)bmp280Cal.dig_P4 * 65536.0);
    var1 = ((float)bmp280Cal.dig_P3 * var1 * var1 / 524288.0 + (float)bmp280Cal.dig_P2 * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * (float)bmp280Cal.dig_P1;
    if (var1 == 0.0)
    {
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576.0 - (float)adc_P;
    p = (p - (var2 / 4096.0)) * 6250.0 / var1;
    var1 = (float)bmp280Cal.dig_P9 * p * p / 2147483648.0;
    var2 = p * (float)bmp280Cal.dig_P8 / 32768.0;
    p = p + (var1 + var2 + (float)bmp280Cal.dig_P7) / 16.0;
    return p;
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of "5123" equals 51.23 DegC
// t_fine carries fine temperature as global value
uint32_t bmp280CompensateT(uint32_t adcT)
{
    uint32_t var1, var2, T;

    var1 = ((((adcT >> 3) - ((uint32_t)bmp280Cal.dig_T1 << 1))) * ((uint32_t)bmp280Cal.dig_T2)) >> 11;
    var2  = (((((adcT >> 4) - ((uint32_t)bmp280Cal.dig_T1)) * ((adcT >> 4) - ((uint32_t)bmp280Cal.dig_T1))) >> 12) * ((uint32_t)bmp280Cal.dig_T3)) >> 14;
    bmp280Cal.t_fine = var1 + var2;
	
    T = (bmp280Cal.t_fine * 5 + 128) >> 8;

    return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of "24674867" represents 24674867/256 = 96386.2 Pa = 963.862 hPa
uint32_t bmp280CompensateP(uint32_t adcP)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)bmp280Cal.t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)bmp280Cal.dig_P6;
    var2 = var2 + ((var1*(int64_t)bmp280Cal.dig_P5) << 17);
    var2 = var2 + (((int64_t)bmp280Cal.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bmp280Cal.dig_P3) >> 8) + ((var1 * (int64_t)bmp280Cal.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bmp280Cal.dig_P1) >> 33;
    if (var1 == 0)
        return 0;
    p = 1048576 - adcP;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)bmp280Cal.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)bmp280Cal.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)bmp280Cal.dig_P7) << 4);
    return (uint32_t)p;
}

#define FILTER_NUM	5
#define FILTER_A	0.1f

void pressureFilter(float* in, float* out)
{	
	static u8 i=0;
	static float filter_buf[FILTER_NUM]={0.0};
	double filter_sum=0.0;
	u8 cnt=0;	
	float deta;		
	
	if(filter_buf[i] == 0.0f)
	{
		filter_buf[i]=*in;
		*out=*in;
		if(++i>=FILTER_NUM)	i=0;
	} else {
		if(i) deta=*in-filter_buf[i-1];
		else deta=*in-filter_buf[FILTER_NUM-1];
		
		if(fabs(deta)<FILTER_A)
		{
			filter_buf[i]=*in;
			if(++i>=FILTER_NUM)	i=0;
		}
		for(cnt=0;cnt<FILTER_NUM;cnt++)
		{
			filter_sum+=filter_buf[cnt];
		}
		*out=filter_sum /FILTER_NUM;
	}
}


void BMP280_ReadTemperatureAndPressure(float *temperature, float *pressure)
{
    int32_t adcT, adc_P;
    int32_t var1, var2, t_fine;

  
    uint8_t data[BMP280_DATA_FRAME_SIZE];
    uint8_t data1,data2,data3,data4,data5,data6;

    // read data from sensor
	  IIC_Read_Len(devAddr, BMP280_PRESSURE_MSB_REG, BMP280_DATA_FRAME_SIZE, data);	 
 
    adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    adcT = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);

    // 计算温度
	
	  var1 = ((((adcT >> 3) - ((uint32_t)bmp280Cal.dig_T1 << 1))) * ((uint32_t)bmp280Cal.dig_T2)) >> 11;
    var2  = (((((adcT >> 4) - ((uint32_t)bmp280Cal.dig_T1)) * ((adcT >> 4) - ((uint32_t)bmp280Cal.dig_T1))) >> 12) * ((uint32_t)bmp280Cal.dig_T3)) >> 14;
    bmp280Cal.t_fine = var1 + var2;
	
	
	
    t_fine = var1 + var2;
    *temperature = (float)((t_fine * 5 + 128) >> 8) / 100;

    // 计算压力
    var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)bmp280Cal.dig_P6);
    var2 = var2 + ((var1 * ((int32_t)bmp280Cal.dig_P5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)bmp280Cal.dig_P4) << 16);
    var1 = (((bmp280Cal.dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)bmp280Cal.dig_P2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((int32_t)bmp280Cal.dig_P1)) >> 15);
    if (var1 == 0) {
        *pressure = 0;
    } else {
        int32_t p = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2 >> 12))) * 3125;
        if (p < 0x80000000) {
            p = (p << 1) / ((uint32_t)var1);
        } else {
            p = (p / (uint32_t)var1) * 2;
        }
        var1 = (((int32_t)bmp280Cal.dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
        var2 = (((int32_t)(p >> 2)) * ((int32_t)bmp280Cal.dig_P8)) >> 13;
        *pressure = (float)((int32_t)p + ((var1 + var2 + bmp280Cal.dig_P7) >> 4)) / 100;
    }

}


void bmp280GetData(float* pressure, float* temperature, float* asl)
{

	  static float t;
    static float p;
	
	bmp280GetPressure();

	t = bmp280CompensateT(bmp280RawTemperature)/100.0;		
	p = bmp280CompensateP(bmp280RawPressure)/25600.0;		

	 pressureFilter(&p,pressure);
	*temperature = (float)t;/*单位度*/
	*pressure = (float)p ;	/*单位hPa*/	
	
	*asl=bmp280PressureToAltitude(pressure);	/*转换成海拔*/

}

void BMP280_calc_values(double* pressure, double* temperature, double* asl)
{
	
	bmp280GetPressure();

	double var1, var2;
	var1=(((double)bmp280RawTemperature)/16384.0-((double)bmp280Cal.dig_T1)/1024.0)*((double)bmp280Cal.dig_T2);
	var2=((((double)bmp280RawTemperature)/131072.0-((double)bmp280Cal.dig_T1)/8192.0)*(((double)bmp280RawTemperature)/131072.0-((double)bmp280Cal.dig_T1)/8192.0))*((double)bmp280Cal.dig_T3);
	double t_fine = (int32_t)(var1+var2);
volatile	float T = (var1+var2)/5120.0;

	var1=((double)t_fine/2.0)-64000.0;
	var2=var1*var1*((double)bmp280Cal.dig_P6)/32768.0;
	var2=var2+var1*((double)bmp280Cal.dig_P5)*2.0;
	var2=(var2/4.0)+(((double)bmp280Cal.dig_P4)*65536.0);
	var1=(((double)bmp280Cal.dig_P3)*var1*var1/524288.0+((double)bmp280Cal.dig_P2)*var1)/524288.0;
	var1=(1.0+var1/32768.0)*((double)bmp280Cal.dig_P1);
	
	volatile	double p=1048576.0-(double)bmp280RawPressure;
	p=(p-(var2/4096.0))*6250.0/var1;
	var1=((double)bmp280Cal.dig_P9)*p*p/2147483648.0;
	var2=p*((double)bmp280Cal.dig_P8)/32768.0;
	p=p+(var1+var2+((double)bmp280Cal.dig_P7))/16.0;
	*pressure = p;
	*temperature = T;
	*asl=44330.0f*(1-powf(p/101325.0f,1.0f/5.255f));//
	*asl=((powf(101325.0/ *pressure, 1/5.257f)-1)*(*temperature+273.15f))/0.0065f;
	printf(" next IS pressure is %f temperature IS %f asl IS %f \r\n",*pressure,*temperature,*asl);

}

/**
 * Converts pressure to altitude above sea level (ASL) in meters
 */
float bmp280PressureToAltitude(float* pressure)
{	
    if(*pressure > 0)
    {
		    return 44330.f * (powf((1015.7f / *pressure), 0.190295f) - 1.0f);
    }
    else
    {
        return 0;
    }
} 





uint8_t IIC_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{ 
 	MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
	if(MPU_IIC_Wait_Ack())	//等待应答
	{
		MPU_IIC_Stop();	
		return 1;		
	}
    MPU_IIC_Send_Byte(reg);	//写寄存器地址
    MPU_IIC_Wait_Ack();		//等待应答
    MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr<<1)|1);//发送器件地址+读命令	
    MPU_IIC_Wait_Ack();		//等待应答 
	while(len)
	{
		if(len==1)*buf=MPU_IIC_Read_Byte(0);//读数据,发送nACK 
		else *buf=MPU_IIC_Read_Byte(1);		//读数据,发送ACK  
		len--;
		buf++; 
	}    
    MPU_IIC_Stop();	//产生一个停止条件 
	return 0;	
}
 

//IIC连续写
//addr:器件地址 
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
uint8_t IIC_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
	uint8_t i; 
    MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
	if(MPU_IIC_Wait_Ack())	//等待应答
	{
		MPU_IIC_Stop();		 
		return 1;		
	}
    MPU_IIC_Send_Byte(reg);	//写寄存器地址
    MPU_IIC_Wait_Ack();		//等待应答
	for(i=0;i<len;i++)
	{
		MPU_IIC_Send_Byte(buf[i]);	//发送数据
		if(MPU_IIC_Wait_Ack())		//等待ACK
		{
			MPU_IIC_Stop();	 
			return 1;		 
		}		
	}    
    MPU_IIC_Stop();	 
	return 0;	
} 