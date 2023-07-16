/***************************************************************************************
标准大气压是指在海平面上，温度为摄氏15度，相对湿度为0%，重力加速度为9.80665米/秒²时，大气压强的平均值为101325帕斯卡（Pa），
也可以表示为1标准大气压（atm）。
****************************************************************************************/
#include "fbm320.h"
#include "iic.h"
#include "stdio.h"
#include "delay.h"
#include "structconfig.h"
#include "math.h"
#include "filter.h"
#include "iic.h"

FBMTYPE FBM;
uint8_t ALTIUDE_OK = 0,ALT_Updated = 0;
float RPFilter;

/*****************************************************************************
*函  数：uint8_t FBM320_WriteByte(uint8_t addr,uint8_t reg,uint8_t data)
*功  能：写一个字节数据到 FBM320 寄存器
*参  数：reg： 寄存器地址
*        data: 要写入的数据
*返回值：0成功 1失败
*备  注：FBM320代码移植只需把I2C驱动修改成自己的即可
*****************************************************************************/
uint8_t FBM320_WriteByte(uint8_t reg,uint8_t data)
{
	if(FBM_Write_Len(FBMAddr2,reg,1,&data))
	   return 1;
	else
	   return 0;
}

/*****************************************************************************
*函  数：uint8_t FBM320_ReadByte(uint8_t reg,uint8_t *buf)
*功  能：从指定FBM320寄存器读取一个字节数据
*参  数：reg： 寄存器地址
*        buf:  读取数据存放的地址
*返回值：1失败 0成功
*备  注：FBM320代码移植只需把I2C驱动修改成自己的即可
*****************************************************************************/
uint8_t FBM320_ReadByte(uint8_t reg,uint8_t *buf)
{
	if(FBM_Read_Len(FBMAddr2,reg,1,buf))
	   return 1;
	else
	   return 0;
}

/*****************************************************************************
*函  数：uint8_t FBM320_WriteMultBytes(uint8_t reg,uint8_t len,uint8_t *buf)
*功  能：从指定寄存器写入指定长度数据
*参  数：reg：寄存器地址
*        len：写入数据长度 
*        buf: 写入数据存放的地址
*返回值：0成功 1失败
*备  注：FBM320代码移植只需把I2C驱动修改成自己的即可
*****************************************************************************/
uint8_t FBM320_WriteMultBytes(uint8_t reg,uint8_t len,uint8_t *buf)
{
	if(FBM_Write_Len(FBMAddr2,reg,len,buf))
	   return 1;
	else
	   return 0;

}

/*****************************************************************************
*函  数：uint8_t FBM320_ReadMultBytes(uint8_t reg,uint8_t len,uint8_t *buf)
*功  能：从指定寄存器读取指定长度数据
*参  数：reg：寄存器地址
*        len：读取数据长度 
*        buf: 读取数据存放的地址
*返回值：0成功 0失败
*备  注：FBM320代码移植只需把I2C驱动修改成自己的即可
*****************************************************************************/
uint8_t FBM320_ReadMultBytes(uint8_t reg,uint8_t len,uint8_t *buf)
{
	if(FBM_Read_Len(FBMAddr2,reg,len,buf))
	   return 1;
	else
	   return 0;
}

/*============================以上代码移植时需要修改=========================*/

/******************************************************************************
*函  数：uint8_tFBM320_getDeviceID(void)
*功  能：读取  FBM320 WHO_AM_I 标识将返回 0x68
*参  数：无
*返回值：返回读取数据
*备  注：无
*******************************************************************************/
uint8_t FBM320_getDeviceID(void)
{
    uint8_t buf[2];
		uint8_t version = 0;
	  uint8_t res = FBM320_ReadByte(FBM_ID, &buf[0]);
	  FBM320_ReadByte(FBM_VERSION, &buf[1]);
		printf("version bis %d",version);
		version = ((buf[0] & 0xC0) >> 6) | ((buf[1] & 0x70) >> 2);
		printf("version is %d",version);
    return buf[0];
}

/******************************************************************************
*函  数：uint8_tFBM320_testConnection(void)
*功  能：检测FBM320 是否已经连接
*参  数：无
*返回值：1已连接 0未链接
*备  注：无
*******************************************************************************/
uint8_t FBM320_testConnection(void) 
{
   if(FBM320_getDeviceID() == FBMID)  //0x42
   return 1;
   else 
	 return 0;
}

/******************************************************************************
*函  数：void FBM320_Check()
*功  能：检测IIC总线上的FBM320是否存在
*参  数：无
*返回值：无
*备  注：无
*******************************************************************************/
void FBM320_Check(void) 
{ 
	while(!FBM320_testConnection())
	{
		printf("\rFBM320 no connect...\r\n");
	}
} 

/******************************************************************************
*函  数：void FBM320_Init(void)
*功  能：初始化FBM320并获取FBM320校准数据
*参  数：无
*返回值：无
*备  注：无
*******************************************************************************/
void FBM320_Init(void)
{
	uint8_t res = FBM320_WriteByte(FBM_RESET,FBMRESET);//复位FBM320
	HAL_Delay(100);
	FBM320_Check(); //检查FBM320与MCU是否正常通信
	FBM320_GetCoeff(); //获取FBM320的校准数据
}

/******************************************************************************
*函  数：uint8_t Init_Altitude(void)
*功  能：初始化起飞时的高度数据
*参  数：无
*返回值：0 初始化未完成 1 初始化完成 
*备  注：此函数初始化的其实是起飞时的大气压(FBM.InitPress)，为后面求相对高度做准备
*******************************************************************************/
uint8_t Init_Altitude(void)
{
	static uint8_t cnt_p = 0;
	static int64_t PressNUM = 0;
	if(GET_FLAG(BAR_OFFSET)) //INIT_ALTIUDE_OK=1 初始化开始
	{
		if(cnt_p == 0)
		{
			cnt_p = 1;
			PressNUM = 0;
		}
		PressNUM += FBM.RP; //100个气压数据累加
		if(cnt_p == 100)
		{
			FBM.InitPress = (float)PressNUM/cnt_p; //求平均值
			SENSER_FLAG_RESET(BAR_OFFSET);//校准气压计结束
			cnt_p = 0; 
			return 1;
		}
		cnt_p++;
   }
	return 0;
}

/******************************************************************************
*函  数：void FBM320_GetAltitude(void) 
*功  能：FBM320的气压值转换成相对高度
*参  数：无
*返回值：无
*备  注：注意：此函数调用频率最好200Hz,否则获取的高度值误差较大！！！
*******************************************************************************/
void FBM320_GetAltitude(void) //200Hz
{
	uint8_t buf[3];
  static uint8_t timecnt = 0;
	float  AltitudeFilter;
	switch(timecnt)//温度和气压数据转换时间不一样
	{
		/**
		这么写的原因是因为FBM320的温度和气压数据转换时间不一样，需要分别进行处理。
		在进行温度转换时，需要先向FBM320写入TEMP_CONVERSION命令，然后等待280us，
		才能读取温度数据。而在进行气压转换时，
		需要先向FBM320写入PRES_CONVERSION+OSR8192命令，
		然后等待15ms，才能读取气压数据。
		因此，需要根据时间间隔来判断当前需要进行的是温度转换还是气压转换。
		*/
		case 0:  
		  FBM320_WriteByte(FBM_CONFIG,TEMP_CONVERSION); //280us
		  break;
		case 1: //5ms转换一次 (836us)
		  FBM320_ReadMultBytes(DATA_MSB,3,buf); 
		 // FBM.ADTemp = (buf[0]<<16)|(buf[1]<<8)|buf[2];
		  FBM.ADTemp = ((uint32_t)buf[0] << 16) + ((uint32_t)buf[1] << 8) + buf[2];
		  FBM320_WriteByte(FBM_CONFIG,PRES_CONVERSION+OSR8192); //开启气压转化
		  break;
		case 5: //15ms转换一次 (834us)
		  FBM320_ReadMultBytes(DATA_MSB,3,buf);  
		  FBM.ADPress = (buf[0]<<16)|(buf[1]<<8)|buf[2];
		  FBM320_WriteByte(FBM_CONFIG,TEMP_CONVERSION); //开启温度转换
		  FBM320_Calculate( FBM.ADPress , FBM.ADTemp );	//将FBM320的原始数据转换成物理量 气压单位是帕，温度单位摄氏度
		  printf("rp rt is  %f and %f \r\n",FBM.RP,FBM.RT);
		  printf("adrp adrt is  %d and %d \r\n",FBM.ADPress,FBM.ADTemp);
/*
		if(Init_Altitude())
		{
			ALTIUDE_OK = 1;
		}
		   
		SortAver_Filter((float)FBM.RP,&FBM.RPFilter,12); //去极值均值滤波
		if(FBM.RPFilter && ALTIUDE_OK) 
		{
			ALT_Updated = 1;
			FBM.Altitude = 44330.0f * (1 - powf((float)FBM.RPFilter/ FBM.InitPress, 0.190295f));
		}
	//FBM.Altitude =  ((pow((1015.7f / (FBM.RP/100)), 0.1902630958) - 1.0f) * (25 + 273.15f)) / 0.0065f;
			 
		SortAver_Filter1(FBM.Altitude ,&AltitudeFilter,12); //去极值均值滤波
		if(AltitudeFilter)
		Aver_Filter1(AltitudeFilter,&FBM.AltitudeFilter,8); //滑动窗口滤波
		*/
		FBM.Altitude = ((float)Abs_Altitude((int32_t)FBM.RP));		//计算绝对高度(相对于海平面)
		  break;
		default:
		  break;
		
	  }
		timecnt++; //转换时间计数
		if(timecnt>5)
		timecnt = 0;
//		printf("Altitude:%0.2f \r\n",FBM.Altitude);
}
/******************************************************************************
*函  数：void FBM320_GetCoeff(void)
*功  能：获取存储在FBM320片内的校准数据
*参  数：无
*返回值：无
*备  注：无
*******************************************************************************/
void FBM320_GetCoeff(void)
{
	uint8_t data[20],i = 0;
	uint16_t R[10];
	FBM320_ReadMultBytes(FBM_COEFF1,18,data);
	FBM320_ReadByte(FBM_COEFF3,&data[18]);
	FBM320_ReadByte(FBM_COEFF4,&data[19]);
	for(i=0;i<10;i++)
	{
		R[i] = (uint16_t)(data[2*i]<<8)|data[2*i+1];
	}
		/* Use R0~R9 calculate C0~C12 of FBM320-02	*/
	FBM.C0 = R[0] >> 4;
	FBM.C1 = ((R[1] & 0xFF00) >> 5) | (R[2] & 7);
	FBM.C2 = ((R[1] & 0xFF) << 1) | (R[4] & 1);
	FBM.C3 = R[2] >> 3;
	FBM.C4 = ((uint32_t)R[3] << 2) | (R[0] & 3);
	FBM.C5 = R[4] >> 1;
	FBM.C6 = R[5] >> 3;
  FBM.C7 = ((uint32_t)R[6] << 3) | (R[5] & 7);
	FBM.C8 = R[7] >> 3;
	FBM.C9 = R[8] >> 2;
	FBM.C10 = ((R[9] & 0xFF00) >> 6) | (R[8] & 3);
	FBM.C11 = R[9] & 0xFF;
	FBM.C12 = ((R[0] & 0x0C) << 1) | (R[7] & 7);

}

/******************************************************************************
*函  数：void FBM320_Calculate(int32_t UP, int32_t UT)		
*功  能：将FBM320获取到AD值转换成物理量
*参  数：UP 气压的AD值 UT温度的AD值
*返回值：无
*备  注：气压单位是帕，温度单位摄氏度
*******************************************************************************/
void FBM320_Calculate(int32_t UP, int32_t UT)										
{
	/*
    int32_t DT, DT2, X01, X02, X03, X11, X12, X13, X21, X22, X23, X24, X25, X26, X31, X32, CF, PP1, PP2, PP3, PP4;

    DT	=	((UT - 8388608) >> 4) + (FBM.C0 << 4);
    X01	=	(FBM.C1 + 4459) * DT >> 1;
    X02	=	((((FBM.C2 - 256) * DT) >> 14) * DT) >> 4;
    X03	=	(((((FBM.C3 * DT) >> 18) * DT) >> 18) * DT);
    FBM.RT	=	((2500 << 15) - X01 - X02 - X03) >> 15;

    DT2	=	(X01 + X02 + X03) >> 12;

    X11	=	((FBM.C5 - 4443) * DT2);
    X12	=	(((FBM.C6 * DT2) >> 16) * DT2) >> 2;
    X13	=	((X11 + X12) >> 10) + ((FBM.C4 + 120586) << 4);

    X21	=	((FBM.C8 + 7180) * DT2) >> 10;
    X22	=	(((FBM.C9 * DT2) >> 17) * DT2) >> 12;
    X23 = (X22 >= X21) ? (X22 - X21) : (X21 - X22);

    X24	=	(X23 >> 11) * (FBM.C7 + 166426);
    X25	=	((X23 & 0x7FF) * (FBM.C7 + 166426)) >> 11;
    X26 = (X21 >= X22) ? (((0 - X24 - X25) >> 11) + FBM.C7 + 166426) : (((X24 + X25) >> 11) + FBM.C7 + 166426);

    PP1	=	((UP - 8388608) - X13) >> 3;
    PP2	=	(X26 >> 11) * PP1;
    PP3	=	((X26 & 0x7FF) * PP1) >> 11;
    PP4	=	(PP2 + PP3) >> 10;

    CF	=	(2097152 + FBM.C12 * DT2) >> 3;
    X31	=	(((CF * FBM.C10) >> 17) * PP4) >> 2;
    X32	=	(((((CF * FBM.C11) >> 15) * PP4) >> 18) * PP4);
    FBM.RP	=	((X31 + X32) >> 15) + PP4 + 99880; */ //99880气压补偿 
		int32_t DT, DT2, X01, X02, X03, X11, X12, X13, X21, X22, X23, X24, X25, X26, X31, X32, CF, PP1, PP2, PP3, PP4;

    DT	=	((UT - 8388608) >> 4) + (FBM.C0 << 4);
    X01	=	(FBM.C1 + 4459) * DT >> 1;
    X02	=	((((FBM.C2 - 256) * DT) >> 14) * DT) >> 4;
    X03	=	(((((FBM.C3 * DT) >> 18) * DT) >> 18) * DT);
    FBM.RT	=	((2500 << 15) - X01 - X02 - X03) >> 15;

    DT2	=	(X01 + X02 + X03) >> 12;

    X11	=	((FBM.C5 - 4443) * DT2);
    X12	=	(((FBM.C6 * DT2) >> 16) * DT2) >> 2;
    X13	=	((X11 + X12) >> 10) + ((FBM.C4 + 120586) << 4);
		X21	=	((FBM.C8 + 7180) * DT2) >> 10;
    X22	=	(((FBM.C9 * DT2) >> 17) * DT2) >> 12;
    X23 = (X22 >= X21) ? (X22 - X21) : (X21 - X22);

    X24	=	(X23 >> 11) * (FBM.C7 + 166426);
    X25	=	((X23 & 0x7FF) * (FBM.C7 + 166426)) >> 11;
    X26 = (X21 >= X22) ? (((0 - X24 - X25) >> 11) + FBM.C7 + 166426) : (((X24 + X25) >> 11) + FBM.C7 + 166426);

    PP1	=	((UP - 8388608) - X13) >> 3;
    PP2	=	(X26 >> 11) * PP1;
    PP3	=	((X26 & 0x7FF) * PP1) >> 11;
    PP4	=	(PP2 + PP3) >> 10;

    CF	=	(2097152 + FBM.C12 * DT2) >> 3;
    X31	=	(((CF * FBM.C10) >> 17) * PP4) >> 2;
    X32	=	(((((CF * FBM.C11) >> 15) * PP4) >> 18) * PP4);
    FBM.RP	=	((X31 + X32) >> 15) + PP4 + 99880; //99880气压补偿

}

/******************************************************************************
*函  数：int32_t Abs_Altitude(int32_t Press)	
*功  能：获取绝对高度
*参  数：Press 气压值
*返回值：无
*备  注：Calculate absolute altitude, unit: mm
*******************************************************************************/
int32_t Abs_Altitude(int32_t Press)																				
{
	int32_t RP, h0, hs0, HP1, HP2, RH;
	int16_t hs1, dP0;
	int8_t P0;
	
	RP = Press;

	if ( RP >= 824000 ) {
		P0	=	103	;
		h0	=	-138507	;
		hs0	=	-5252	;
		hs1	=	311	;
	} else if ( RP >= 784000 ) {
		P0	=	98	;
		h0	=	280531	;
		hs0	=	-5468	;
		hs1	=	338	;
	} else if ( RP >= 744000 ) {
		P0	=	93	;
		h0	=	717253	;
		hs0	=	-5704	;
		hs1	=	370	;
	} else if ( RP >= 704000 ) {
		P0	=	88	;
		h0	=	1173421	;
		hs0	=	-5964	;
		hs1	=	407	;
	} else if ( RP >= 664000 ) {
		P0	=	83	;
		h0	=	1651084	;
		hs0	=	-6252	;
		hs1	=	450	;
	} else if ( RP >= 624000 ) {
		P0	=	78	;
		h0	=	2152645	;
		hs0	=	-6573	;
		hs1	=	501	;
	} else if ( RP >= 584000 ) {
		P0	=	73	;
		h0	=	2680954	;
		hs0	=	-6934	;
		hs1	=	560	;
	} else if ( RP >= 544000 ) {
		P0	=	68	;
		h0	=	3239426	;
		hs0	=	-7342	;
		hs1	=	632	;
	} else if ( RP >= 504000 ) {
		P0	=	63	;
		h0	=	3832204	;
		hs0	=	-7808	;
		hs1	=	719	;
	} else if ( RP >= 464000 ) {
		P0	=	58	;
		h0	=	4464387	;
		hs0	=	-8345	;
		hs1	=	826	;
	} else if ( RP >= 424000 ) {
		P0	=	53	;
		h0	=	5142359	;
		hs0	=	-8972	;
		hs1	=	960	;
	} else if ( RP >= 384000 ) {
		P0	=	48	;
		h0	=	5874268	;
		hs0	=	-9714	;
		hs1	=	1131	;
	} else if ( RP >= 344000 ) {
		P0	=	43	;
		h0	=	6670762	;
		hs0	=	-10609	;
		hs1	=	1354	;
	} else if ( RP >= 304000 ) {
		P0	=	38	;
		h0	=	7546157	;
		hs0	=	-11711	;
		hs1	=	1654	;
	} else if ( RP >= 264000 ) {
		P0	=	33	;
		h0	=	8520395	;
		hs0	=	-13103	;
		hs1	=	2072	;
	} else {
		P0	=	28	;
		h0	=	9622536	;
		hs0	=	-14926	;
		hs1	=	2682	;
	}

	dP0	=	RP - P0 * 8000;
	HP1	=	( hs0 * dP0 ) >> 1;
	HP2	=	((( hs1 * dP0 ) >> 14 ) * dP0 ) >> 4;
	RH	=	(( HP1 + HP2 ) >> 8 ) + h0;

	return RH;
	/*
	int8_t P0;			
	int16_t hs1, dP0;			
	int32_t h0, hs0, HP1, HP2;			
					
	if(Press >= 103000)
	{	
		P0	=	103;
		h0	=	-138507;
		hs0	=	-21007;
		hs1	=	311;
	}	
	else if(Press >= 98000)
	{	
		P0	=	98;
		h0	=	280531;
		hs0	=	-21869;
		hs1	=	338;
	}	
	else if(Press >= 93000)
	{	
		P0	=	93;
		h0	=	717253;
		hs0	=	-22813;
		hs1	=	370;
	}	
				
	else if(Press >= 88000)
	{	
		P0	=	88;
		h0	=	1173421;
		hs0	=	-23854;
		hs1	=	407;
	}	
	else if(Press >= 83000)
	{	
		P0	=	83;
		h0	=	1651084;
		hs0	=	-25007;
		hs1	=	450;
	}	
	else if(Press >= 78000)
	{	
		P0	=	78;
		h0	=	2152645;
		hs0	=	-26292;
		hs1	=	501;
	}	
	else if(Press >= 73000)
	{	
		P0	=	73;
		h0	=	2680954;
		hs0	=	-27735;
		hs1	=	560;
	}	
	else if(Press >= 68000)
	{	
		P0	=	68;
		h0	=	3239426;
		hs0	=	-29366;
		hs1	=	632;
	}	
	else if(Press >= 63000)
	{	
		P0	=	63;
		h0	=	3832204;
		hs0	=	-31229;
		hs1	=	719;
	}	
	else if(Press >= 58000)
	{	
		P0	=	58;
		h0	=	4464387;
		hs0	=	-33377;
		hs1	=	826;
	}	
	else if(Press >= 53000)
	{	
		P0	=	53;
		h0	=	5142359;
		hs0	=	-35885;
		hs1	=	960;
	}		
	else if(Press >= 48000)
	{	
		P0	=	48;
		h0	=	5874268;
		hs0	=	-38855;
		hs1	=	1131;
	}	
	else if(Press >= 43000)
	{	
		P0	=	43;
		h0	=	6670762;
		hs0	=	-42434;
		hs1	=	1354;
	}	
	else if(Press >= 38000)
	{	
		P0	=	38;
		h0	=	7546157;
		hs0	=	-46841;
		hs1	=	1654;
	}	
	else if(Press >= 33000)
	{	
		P0	=	33;
		h0	=	8520395;
		hs0	=	-52412;
		hs1	=	2072;
	}	
	else
	{	
		P0	=	28;
		h0	=	9622536;
		hs0	=	-59704;
		hs1	=	2682;
	}
					
	dP0	=	Press - P0 * 1000;
	HP1	=	(hs0 * dP0) >> 2;
	HP2	=	(((hs1 * dP0) >> 10)* dP0) >> 4;			

	return	((h0 << 6) + HP1 + HP2) >> 6;	*/									//Return absolute altitude 返回绝度高度
}




uint8_t FBM_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{ 
 	MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
	if(MPU_IIC_Wait_Ack())	//等待应答
	{
		MPU_IIC_Stop();	
		printf("no ack back");		
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
uint8_t FBM_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
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
