/***************************************************************************************

https://blog.csdn.net/ilovezhuanxin/article/details/131797100
https://github.com/taejin-seong/STM32F103XX-Module-Libraries/blob/master/stm32f103_fw_module/src/hw/driver/nrf24l01.c
https://github.com/J20RC/STM32_RC_Transmitter/blob/master/software/User/main.c
                                      声 明
    本项目代码仅供个人学习使用，可以自由移植修改，但必须保留此声明信息。移植过程中出现
其他不可估量的BUG，天际智联不负任何责任。请勿商用！

程序版本：V1.01
程序日期：2018-1-26
程序作者：愤怒的小孩 E-mail：1138550496@qq.com
版权所有：西安天际智联信息技术有限公司
****************************************************************************************/
#include "nrf2401.h"
#include "spi.h"
#include "data.h"
/************************************************************************
*代码移植修改区
*只需要根据原理图修改对应的端口时钟 端口 引脚
************************************************************************/

// spi 片选通信引脚
#define RCC_NRF_SCN   RCC_AHB1Periph_GPIOA  //端口时钟
#define NRF_SCN_PORT  GPIOA                 //端口
#define NRF_SCN       GPIO_PIN_4           //引脚
// nrf使能引脚
#define RCC_NRF_CE    RCC_AHB1Periph_GPIOB  //端口时钟
#define NRF_CE_PORT   GPIOB                 //端口
#define NRF_CE        GPIO_PIN_0            //引脚

// nrf中断
#define RCC_NRF_IRQ   RCC_AHB1Periph_GPIOB  //端口时钟
#define NRF_IRQ_PORT  GPIOB                 //端口
#define NRF_IRQ       GPIO_PIN_1            //引脚
/**************************************************************************/

//设置引脚电平
#define NRF_SCN_LOW   HAL_GPIO_WritePin(NRF_SCN_PORT, NRF_SCN, GPIO_PIN_RESET);

#define NRF_SCN_HIGH   HAL_GPIO_WritePin(NRF_SCN_PORT, NRF_SCN, GPIO_PIN_SET);
#define NRF_CE_LOW      HAL_GPIO_WritePin(NRF_CE_PORT, NRF_CE, GPIO_PIN_RESET);
#define NRF_CE_HIGH    HAL_GPIO_WritePin(NRF_CE_PORT, NRF_CE, GPIO_PIN_SET);
//读取引脚电平
//#define NRF_IRQ_READ  (NRF_IRQ_PORT->IDR & NRF_IRQ)
#define NRFAddrMax 50 //NRF最后一个字节地址最大为50
uint8_t NRFaddr = 0xFF; //初始化NRF最后一字节地址

uint8_t NRF_TX_DATA[TX_PAYLO_WIDTH];//NRF发送缓冲区
uint8_t NRF_RX_DATA[RX_PAYLO_WIDTH];//NRF接收缓冲区

uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0xFF}; //发送地址
uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0xFF}; //接收地址

void NRF24L01_Config(void);
void NRF_GetAddr(void);
void Remote_Data_ReceiveAnalysis(void);

 
 /*****************************************************************************
*函  数：void NRF24l01_Init(void)
*功  能：NRF引脚GPIO初始化
*参  数：无
*返回值：无
*备  注：无
*****************************************************************************/
void NRF24l01_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

		
	
	NRF24L01_Check(); //检查NRF24L01是否与MCU通信                                    
	
  NRF_SCN_HIGH; //失能NRF
  NRF_CE_LOW; //待机模式
	
  NRF24L01_Config();  //配置NRF并初始化为接收模式
}

/*****************************************************************************
*函  数：uint8_t NRF24l01_write_reg(uint8_t reg,uint8_t value)
*功  能：写一字节数据到寄存器
*参  数：reg： 寄存器地址
*        val:  要写入的数据
*返回值：status
*备  注：NRF2401代码移植只需把SPI驱动修改成自己的即可
*****************************************************************************/
uint8_t NRF24l01_write_reg(uint8_t reg,uint8_t value)
{
	uint8_t status;
	NRF_SCN_LOW;
	uint8_t buffer;
	HAL_SPI_Transmit(&hspi1, &reg, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1, &value, 1, HAL_MAX_DELAY);
	NRF_SCN_HIGH;
	return status;
}

/*****************************************************************************
*函  数：uint8_t NRF24l01_read_reg(uint8_t reg)
*功  能：读一字节数据到寄存器
*参  数：reg： 寄存器地址
*返回值：reg_val
*备  注：NRF2401代码移植只需把SPI驱动修改成自己的即可
*****************************************************************************/
uint8_t NRF24l01_read_reg(uint8_t reg)
{
	NRF_SCN_LOW;
	uint8_t command = 0xff;
	uint8_t buffer;
	HAL_SPI_Transmit(&hspi1, &reg, 1, HAL_MAX_DELAY);
	HAL_SPI_TransmitReceive(&hspi1, &command, &buffer, 1, HAL_MAX_DELAY);
  
	NRF_SCN_HIGH;
	return buffer;
}

/*****************************************************************************
*函  数：uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
*功  能：写一组数据到寄存器
*参  数：reg： 寄存器地址
*       pBuf： 要写入数据的地址
*        len:  要写入的数据长度
*返回值：status
*备  注：NRF2401代码移植只需把SPI驱动修改成自己的即可
*****************************************************************************/
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
	uint8_t status;
	int i;
  uint8_t buffer;

	NRF_SCN_LOW;

	status = HAL_SPI_Transmit(&hspi1, &reg, 1, HAL_MAX_DELAY);
	for( i=0;i<len;i++)
	{
    status = HAL_SPI_Transmit(&hspi1, pBuf, 1, HAL_MAX_DELAY);
		if (status != HAL_OK) {
				printf("NRF24L01_Write_Buf not success2 " );
				NRF_SCN_HIGH;
				return status;
		}
		pBuf++;
	}
	NRF_SCN_HIGH;
	return status;
}

/*****************************************************************************
*函  数：uint8_t NRF24L01_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
*功  能：读一组数据到寄存器
*参  数：reg： 寄存器地址
*       pBuf： 要读取数据的地址
*        len:  要读取的数据长度
*返回值：status
*备  注：NRF2401代码移植只需把SPI驱动修改成自己的即可
*****************************************************************************/
uint8_t NRF24L01_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
	uint8_t status;
	int i;
	NRF_SCN_LOW;
  status = HAL_SPI_Transmit(&hspi1, &reg, 1, HAL_MAX_DELAY);
	for(i = 0;i < len ;i++)
	{
    status = HAL_SPI_Receive(&hspi1, pBuf, 1, HAL_MAX_DELAY);
	  if (status != HAL_OK) {
				printf("NRF24L01_Read_Buf not success \r\n			" );
				NRF_SCN_HIGH;
				return status;
		}
		pBuf++;
	}
	NRF_SCN_HIGH;
	return status;
}

/*****************************************************************************
*函  数：void NRFset_Mode(uint8_t mode)
*功  能：切换NRF2401的工作模式模式
*参  数：无
*返回值：无
*备  注：无
*****************************************************************************/
void NRFset_Mode(uint8_t mode)
{
	if(mode == IT_TX)
	{
	  NRF_CE_LOW;
	  NRF24l01_write_reg(W_REGISTER+CONFIG,IT_TX);
    NRF24l01_write_reg(W_REGISTER+STATUS,0X7E); //清除所有中断,防止一进去发送模式就触发中断	
	  NRF_CE_HIGH;
	  delay_us(15);
	}
	else
	{
	  NRF_CE_LOW;
	  NRF24l01_write_reg(W_REGISTER+CONFIG,IT_RX);//配置为接收模式
	  NRF24l01_write_reg(W_REGISTER+STATUS,0X7E); //清除所有中断,防止一进去接收模式就触发中断
	  NRF_CE_HIGH;
	  delay_us(200);
	}
		
}

/*****************************************************************************
*函  数：void NRF24L01_Config(void)
*功  能：NRF基本参数配置，并初始化为接收模式
*参  数：无
*返回值：无
*备  注：无
*****************************************************************************/
void NRF24L01_Config(void)
{
	NRF_CE_LOW;
	NRF24l01_write_reg(W_REGISTER+SETUP_AW, 0x03); //配置通信地址的长度，默认值时0x03,即地址长度为5字节
	NRF24L01_Write_Buf(W_REGISTER+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH); //写TX节点地址 
	NRF24L01_Write_Buf(W_REGISTER+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH); //设置TX节点地址,主要为了使能ACK
	NRF24l01_write_reg(W_REGISTER+SETUP_RETR,0x1A); //设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次 0x1A
	
	NRF24l01_write_reg(W_REGISTER+EN_RXADDR,0x01);//使能通道0的接收地址  
	NRF24l01_write_reg(W_REGISTER+EN_AA,0x01); //使能通道0自动应答
	
	NRF24l01_write_reg(W_REGISTER+RX_PW_P0,RX_PAYLO_WIDTH);//选择通道0的有效数据宽度  
	NRF24L01_Write_Buf(W_REGISTER+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH); //写RX节点地址
	NRF24l01_write_reg(W_REGISTER+RF_CH,40); //设置RF通道为40hz(1-64Hz都可以)
	NRF24l01_write_reg(W_REGISTER+RF_SETUP,0x27); //设置TX发射参数,0db增益,2Mbps,低噪声增益关闭 （注意：低噪声增益关闭/开启直接影响通信,要开启都开启，要关闭都关闭0x0f）0x07
	
	NRFset_Mode(IT_RX); //默认为接收模式
	
	NRF_CE_HIGH;
}	

/*****************************************************************************
*函  数：uint8_t NRF24L01_TxPacket(uint8_t *txbuf)
*功  能：NRF发送一包数据
*参  数：txbuf：要发送数据地址
*返回值：无 
*备  注：无
*****************************************************************************/
void NRF24L01_TxPacket(uint8_t *txbuf)
{
	NRF_CE_LOW;	
	NRF24L01_Write_Buf(W_REGISTER+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);  //写TX节点地址 
	NRF24L01_Write_Buf(W_REGISTER+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH); //设置TX节点地址,主要为了使能ACK
	NRF24L01_Write_Buf(W_RX_PAYLOAD,txbuf,TX_PAYLO_WIDTH); //写数据到TX_BUFF
  NRF24l01_write_reg(W_REGISTER+CONFIG,IT_TX);	//设置为发送模式,开启所有中断
	NRF24l01_write_reg(W_REGISTER+STATUS,0X7E); //清除所有中断,防止一进去发送模式就触发中断
  NRF_CE_HIGH;
	delay_us(10);  //CE持续高电平10us
}

/*****************************************************************************
*函  数：uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
*功  能：NRF接收一包数据
*参  数：rxbuf：接收数据存储地址
*返回值：无
*备  注：无
*****************************************************************************/
void NRF24L01_RxPacket(uint8_t *rxbuf)
{
	NRF_CE_LOW;
	NRF24L01_Read_Buf(R_RX_PAYLOAD,rxbuf,TX_PAYLO_WIDTH);//读取RX的有效数据
	NRF24l01_write_reg(FLUSH_RX,0xff); //清除RX FIFO(注意：这句话很必要)
	NRF_CE_HIGH;
}
/*****************************************************************************
*函  数：uint8_t NRF24L01_testConnection(void)
*功  能：检查NRF2401与MCU的SPI总线是否通信正常
*参  数：无
*返回值：1已连接 0未连接
*备  注：无
*****************************************************************************/
uint8_t NRF24L01_testConnection(void)
{
	uint8_t buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	uint8_t i; 	 
	NRF24L01_Write_Buf(W_REGISTER+TX_ADDR,buf,5); //写入5个字节的地址.	
	NRF24L01_Read_Buf(TX_ADDR,buf,5); //读出写入的地址  
	for(i=0;i<5;i++)
	if(buf[i]!=0XA5)break;	 							   
	if(i!=5){
			printf("i is %d",i);

		return 0; //检测24L01错误	
	}
	printf("2401 success");
	return 1;	//检测到24L01
}	
void NRF24L01_Check(void)
{
	while(!NRF24L01_testConnection())
	{
		HAL_Delay(100);
		printf("\rNRF2401 no connect...\r\n");
 	}
}

/*****************************************************************************
*函  数：void EXTI2_IRQHandler(void)
*功  能：NRF(全双工)的外中断处理函数
*参  数：无
*返回值：无
*备  注: NRF的所有中断事件都在次函数中给予相应的处理
*****************************************************************************/
void EXTI1_IRQHandler(void)
{           
	__HAL_GPIO_EXTI_CLEAR_IT(NRF_IRQ);	
	uint8_t sta; 
	delay_us(1000);
	if(HAL_GPIO_ReadPin(NRF_IRQ_PORT,NRF_IRQ) == RESET )
	{
		 NRF_CE_LOW;//拉低CE，以便读取NRF中STATUS中的数据
		 sta = NRF24l01_read_reg(R_REGISTER+STATUS); //读取STATUS中的数据，以便判断是由什么中断源触发的IRQ中断
		
		/* 发送完成中断 TX_OK */
		if(sta & TX_OK)                                   
		{										
		  NRFset_Mode(IT_RX);	
			NRF24l01_write_reg(W_REGISTER+STATUS,TX_OK); //清除发送完成标志·
		  NRF24l01_write_reg(FLUSH_TX,0xff); //清除TX_FIFO
			//printf("Sent OK!!!!\r\n");
		}
		/* 接收完成中断 RX_OK */
		if(sta & RX_OK) 
		{	
			// 32
      NRF24L01_RxPacket(NRF_RX_DATA);	
			NRF24l01_write_reg(W_REGISTER+STATUS,RX_OK); //清除发送完成标志·
			NRF24l01_write_reg(FLUSH_RX,0xff); //清除TX_FIFO

			// 遥控器数据处理
			Remote_Data_ReceiveAnalysis();
			//printf("Receive OK!!!!\r\n");	
		}
		/* 达到最大重发次数中断  MAX_TX */
		if(sta & MAX_TX)                                  
		{											
			NRFset_Mode(IT_RX);
			NRF24l01_write_reg(W_REGISTER+STATUS,MAX_TX);//清除接达到最大重发标志
			NRF24l01_write_reg(FLUSH_TX,0xff); //清除TX_FIFO
			// printf("Sent Max Data!!!\r\n"); 
				HAL_GPIO_TogglePin(led1_GPIO_Port, led1_Pin);
		}
   // HAL_NVIC_ClearPendingIRQ(EXTI1_IRQn);
	}
}
 



void Remote_Data_Send(void)
{
	NRF_TX_DATA[0] =remoter_buffer.type;//帧头
	NRF_TX_DATA[1] = remoter_buffer.command; //标志位组
  NRF_TX_DATA[2] =remoter_buffer.rcLock;
  NRF_TX_DATA[3] =remoter_buffer.ctrlMode;
  NRF_TX_DATA[4] = remoter_buffer.flightMode;
	NRF_TX_DATA[5] = remoter_buffer.length;
  
	// 分别提取 roll_value 的每个字节，并写入到 NRF_RX_DATA 数组的第6到9位
	  union {
        float f;
        uint8_t bytes[sizeof(float)];
    } convertR,convertP,convertY,convertT,convertTrimPitch,convertTrimRoll;
/*
	 	在某些系统中，将浮点数的内存表示直接转换为整数类型可能导致精度丢失。这可能是由于浮点数内存布局和整数类型内存布局之间的差异。因此，当重新将整数值解释为浮点数时，得到的结果可能不是预期的浮点数值，而是零或其他不准确的值。
    为了避免这种问题，可以考虑使用更可靠的方法将浮点数转换为字节数组，例如使用联合体或库函数。
		*/
  convertR.f = remoter.roll;
	NRF_TX_DATA[6] = convertR.bytes[3];
	NRF_TX_DATA[7] = convertR.bytes[2];
	NRF_TX_DATA[8] = convertR.bytes[1];
	NRF_TX_DATA[9] = convertR.bytes[0];
		
  convertP.f = remoter.pitch;
	NRF_TX_DATA[10] = convertP.bytes[3];
	NRF_TX_DATA[11] = convertP.bytes[2];
	NRF_TX_DATA[12] = convertP.bytes[1];
	NRF_TX_DATA[13] = convertP.bytes[0];
		
   
  convertY.f = remoter.yaw;
	NRF_TX_DATA[14] = convertY.bytes[3];
	NRF_TX_DATA[15] = convertY.bytes[2];
	NRF_TX_DATA[16] = convertY.bytes[1];
	NRF_TX_DATA[17] = convertY.bytes[0];
		
   
	convertT.f = remoter.thrust;
	NRF_TX_DATA[18] = convertT.bytes[3];
	NRF_TX_DATA[19] = convertT.bytes[2];
	NRF_TX_DATA[20] = convertT.bytes[1];
	NRF_TX_DATA[21] = convertT.bytes[0];
		


	convertTrimPitch.f = remoter.trimPitch;
	NRF_TX_DATA[22] = convertTrimPitch.bytes[3];
	NRF_TX_DATA[23] = convertTrimPitch.bytes[2];
	NRF_TX_DATA[24] = convertTrimPitch.bytes[1];
	NRF_TX_DATA[25] = convertTrimPitch.bytes[0];
		
 
 
	convertTrimRoll.f = remoter.trimRoll;
	NRF_TX_DATA[26] = convertTrimRoll.bytes[3];
	NRF_TX_DATA[27] = convertTrimRoll.bytes[2];
	NRF_TX_DATA[28] = convertTrimRoll.bytes[1];
	NRF_TX_DATA[29] = convertTrimRoll.bytes[0];
		
	NRF_TX_DATA[30] = remoter.checksum;
	
	uint8_t dogData = 6;
	NRF_TX_DATA[31] = dogData;
	NRF24L01_TxPacket(NRF_TX_DATA);
}

void Remote_Data_ReceiveAnalysis(void)
{
	remoter.type  = NRF_RX_DATA[0];
	remoter.command  = NRF_RX_DATA[1];
	remoter.rcLock  = NRF_RX_DATA[2];
	remoter.ctrlMode  = NRF_RX_DATA[3];
	remoter.flightMode  = NRF_RX_DATA[4];
	remoter.length  = NRF_RX_DATA[5]; // 有效位 后6位有效 00111111 后6位无效00 000000
	
	union {
		float f;
    uint8_t bytes[sizeof(float)];
  } convertR,convertP,convertY,convertT,convertTrimPitch,convertTrimRoll;
	 
	convertR.bytes[3] = NRF_RX_DATA[6];
  convertR.bytes[2] = NRF_RX_DATA[7];
  convertR.bytes[1] = NRF_RX_DATA[8];
  convertR.bytes[0] = NRF_RX_DATA[9];
	
	convertP.bytes[3] = NRF_RX_DATA[10];
  convertP.bytes[2] = NRF_RX_DATA[11];
  convertP.bytes[1] = NRF_RX_DATA[12];
  convertP.bytes[0] = NRF_RX_DATA[13];
		
	convertY.bytes[3] = NRF_RX_DATA[14];
  convertY.bytes[2] = NRF_RX_DATA[15];
  convertY.bytes[1] = NRF_RX_DATA[16];
  convertY.bytes[0] = NRF_RX_DATA[17];
	
  convertT.bytes[3] = NRF_RX_DATA[18];
  convertT.bytes[2] = NRF_RX_DATA[19];
  convertT.bytes[1] = NRF_RX_DATA[20];
  convertT.bytes[0] = NRF_RX_DATA[21];
	
	convertTrimPitch.bytes[3] = NRF_RX_DATA[22];
  convertTrimPitch.bytes[2] = NRF_RX_DATA[23];
  convertTrimPitch.bytes[1] = NRF_RX_DATA[24];
  convertTrimPitch.bytes[0] = NRF_RX_DATA[25];
  
	convertTrimRoll.bytes[3] = NRF_RX_DATA[26];
  convertTrimRoll.bytes[2] = NRF_RX_DATA[27];
  convertTrimRoll.bytes[1] = NRF_RX_DATA[28];
  convertTrimRoll.bytes[0] = NRF_RX_DATA[29];
	
	remoter.roll  = convertR.f;
	remoter.pitch  =convertP.f;
	remoter.yaw  = convertY.f;
	remoter.thrust  = convertT.f;
	remoter.trimPitch  =convertTrimPitch.f;
	remoter.trimRoll  = convertTrimRoll.f;
	remoter.checksum  = NRF_RX_DATA[30];
	
	commanderBits.ctrlMode = remoter.ctrlMode ;
	// todo 处理命令
	commanderBits.keyFlight = 1; 
	commanderBits.keyLand = 0;
	commanderBits.emerStop = 0;
	commanderBits.flightMode = remoter.flightMode; // 有头 1无头
//	 printf("recive fly r %f p %f y %f \r\n",remoter.roll,remoter.pitch,remoter.yaw);
}

 
  
/*****************************************************************************
*函  数：void NRF_GetAddr(void)
*功  能：给飞机获取上的NRF获取一个地址
*参  数：无
*返回值：无 
*备  注：此函数需要与遥控器的对频函数联合使用否者NRF通信不成功，
         如果自己做的的遥控器可直接用固定地址
****************************************************************************
void NRF_GetAddr(void)
{
	if(NRFaddr > NRFAddrMax)//当 NRFaddr大于NRFAddrMax，就说明次时NRF还未初始化完成
	{
	  srand(SysTick->VAL);//给随机数种子
		//printf("SysTick->VAL:%d\r\n",SysTick->VAL);
		NRFaddr = rand()%NRFAddrMax;//随机获取NRF最后一字节地址（地址:0~50）
    PID_WriteFlash();//保存此地址Flash
	}else if(NRFaddr != TX_ADDRESS[TX_ADR_WIDTH-1])
	{
		TX_ADDRESS[TX_ADR_WIDTH-1] = NRFaddr;
		RX_ADDRESS[TX_ADR_WIDTH-1] = NRFaddr;
	  NRF24L01_Config();
	//printf("NRFAddr:%d\r\n",NRFaddr);
	}
}
*/
/*****************************************************************************
*函  数：void NRF_Test(void)
*功  能：MRF通信测试函数
*参  数：无
*返回值：无 
*备  注：测试时用
*****************************************************************************/
void NRF_Test(void)
{
	uint8_t t=0;
	static uint8_t mode,key;
	mode = ' ';
	key=mode;
	for(t=0;t<32;t++)
	{
		key++;
		if(key>('~'))key=' ';
		NRF_TX_DATA[t]=key;	
	}
	mode++; 
	if(mode>'~')mode=' ';  	  		
	NRF24L01_TxPacket(NRF_TX_DATA);
}

 

