/***************************************************************************************
                                      �� ��
    ����Ŀ�����������ѧϰʹ�ã�����������ֲ�޸ģ������뱣����������Ϣ����ֲ�����г���
�������ɹ�����BUG��������������κ����Ρ��������ã�

����汾��V1.01
�������ڣ�2018-1-26
�������ߣ���ŭ��С�� E-mail��1138550496@qq.com
��Ȩ���У��������������Ϣ�������޹�˾
****************************************************************************************/
#include "nrf24l01.h"
#include "spi.h"
#include "stdio.h"
#include "data.h"
/************************************************************************
*������ֲ�޸���
*ֻ��Ҫ����ԭ��ͼ�޸Ķ�Ӧ�Ķ˿�ʱ�� �˿� ����
************************************************************************/

// spi Ƭѡͨ������
#define RCC_NRF_SCN   RCC_AHB1Periph_GPIOA  //�˿�ʱ��
#define NRF_SCN_PORT  GPIOA                 //�˿�
#define NRF_SCN       GPIO_PIN_4           //����
// nrfʹ������
#define RCC_NRF_CE    RCC_AHB1Periph_GPIOB  //�˿�ʱ��
#define NRF_CE_PORT   GPIOB                 //�˿�
#define NRF_CE        GPIO_PIN_0            //����

// nrf�ж�
#define RCC_NRF_IRQ   RCC_AHB1Periph_GPIOB  //�˿�ʱ��
#define NRF_IRQ_PORT  GPIOB                 //�˿�
#define NRF_IRQ       GPIO_PIN_1            //����
/**************************************************************************/

//�������ŵ�ƽ
#define NRF_SCN_LOW   HAL_GPIO_WritePin(NRF_SCN_PORT, NRF_SCN, GPIO_PIN_RESET);

#define NRF_SCN_HIGH   HAL_GPIO_WritePin(NRF_SCN_PORT, NRF_SCN, GPIO_PIN_SET);
#define NRF_CE_LOW      HAL_GPIO_WritePin(NRF_CE_PORT, NRF_CE, GPIO_PIN_RESET);
#define NRF_CE_HIGH    HAL_GPIO_WritePin(NRF_CE_PORT, NRF_CE, GPIO_PIN_SET);
//��ȡ���ŵ�ƽ
//#define NRF_IRQ_READ  (NRF_IRQ_PORT->IDR & NRF_IRQ)
#define NRFAddrMax 50 //NRF���һ���ֽڵ�ַ���Ϊ50
uint8_t NRFaddr = 0xFF; //��ʼ��NRF���һ�ֽڵ�ַ

uint8_t NRF_TX_DATA[TX_PAYLO_WIDTH];//NRF���ͻ�����
uint8_t NRF_RX_DATA[RX_PAYLO_WIDTH];//NRF���ջ�����

uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0xFF}; //���͵�ַ
uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0xFF}; //���յ�ַ

void NRF24L01_Config(void);
void NRF_GetAddr(void);
void Remote_Data_ReceiveAnalysis(void);


void Remote_Data_Event(void);
/*****************************************************************************
*��  ����void NRF24l01_Init(void)
*��  �ܣ�NRF����GPIO��ʼ��
*��  ������
*����ֵ����
*��  ע����
*****************************************************************************/
void NRF24l01_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

		
	
	NRF24L01_Check(); //���NRF24L01�Ƿ���MCUͨ��                                    
	
  NRF_SCN_HIGH; //ʧ��NRF
  NRF_CE_LOW; //����ģʽ
	
  NRF24L01_Config();  //����NRF����ʼ��Ϊ����ģʽ
}

/*****************************************************************************
*��  ����uint8_t NRF24l01_write_reg(uint8_t reg,uint8_t value)
*��  �ܣ�дһ�ֽ����ݵ��Ĵ���
*��  ����reg�� �Ĵ�����ַ
*        val:  Ҫд�������
*����ֵ��status
*��  ע��NRF2401������ֲֻ���SPI�����޸ĳ��Լ��ļ���
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
*��  ����uint8_t NRF24l01_read_reg(uint8_t reg)
*��  �ܣ���һ�ֽ����ݵ��Ĵ���
*��  ����reg�� �Ĵ�����ַ
*����ֵ��reg_val
*��  ע��NRF2401������ֲֻ���SPI�����޸ĳ��Լ��ļ���
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
*��  ����uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
*��  �ܣ�дһ�����ݵ��Ĵ���
*��  ����reg�� �Ĵ�����ַ
*       pBuf�� Ҫд�����ݵĵ�ַ
*        len:  Ҫд������ݳ���
*����ֵ��status
*��  ע��NRF2401������ֲֻ���SPI�����޸ĳ��Լ��ļ���
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
 				NRF_SCN_HIGH;
				return status;
		}
		pBuf++;
	}
	NRF_SCN_HIGH;
	return status;
}

/*****************************************************************************
*��  ����uint8_t NRF24L01_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
*��  �ܣ���һ�����ݵ��Ĵ���
*��  ����reg�� �Ĵ�����ַ
*       pBuf�� Ҫ��ȡ���ݵĵ�ַ
*        len:  Ҫ��ȡ�����ݳ���
*����ֵ��status
*��  ע��NRF2401������ֲֻ���SPI�����޸ĳ��Լ��ļ���
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
				printf("HAL_OK not success " );// 			�ɻ����ﲻ����	printf("HAL_OK not success " );

				NRF_SCN_HIGH;
				return status;
		}
		pBuf++;
	}
	NRF_SCN_HIGH;
	return status;
}

/*****************************************************************************
*��  ����void NRFset_Mode(uint8_t mode)
*��  �ܣ��л�NRF2401�Ĺ���ģʽģʽ
*��  ������
*����ֵ����
*��  ע����
*****************************************************************************/
void NRFset_Mode(uint8_t mode)
{
	if(mode == IT_TX)
	{
	  NRF_CE_LOW;
	  NRF24l01_write_reg(W_REGISTER+CONFIG,IT_TX);
    NRF24l01_write_reg(W_REGISTER+STATUS,0X7E); //��������ж�,��ֹһ��ȥ����ģʽ�ʹ����ж�	
	  NRF_CE_HIGH;
	  delay_us(15);
	}
	else
	{
	  NRF_CE_LOW;
	  NRF24l01_write_reg(W_REGISTER+CONFIG,IT_RX);//����Ϊ����ģʽ
	  NRF24l01_write_reg(W_REGISTER+STATUS,0X7E); //��������ж�,��ֹһ��ȥ����ģʽ�ʹ����ж�
	  NRF_CE_HIGH;
	  delay_us(200);
	}
		
}

/*****************************************************************************
*��  ����void NRF24L01_Config(void)
*��  �ܣ�NRF�����������ã�����ʼ��Ϊ����ģʽ
*��  ������
*����ֵ����
*��  ע����
*****************************************************************************/
void NRF24L01_Config(void)
{
	NRF_CE_LOW;
	NRF24l01_write_reg(W_REGISTER+SETUP_AW, 0x03); //����ͨ�ŵ�ַ�ĳ��ȣ�Ĭ��ֵʱ0x03,����ַ����Ϊ5�ֽ�
	NRF24L01_Write_Buf(W_REGISTER+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH); //дTX�ڵ��ַ 
	//NRF24L01_Write_Buf(W_REGISTER+RX_ADDR_P0,(uint8_t*)TX_ADDRESS,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK
	NRF24L01_Write_Buf(W_REGISTER+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK
	NRF24l01_write_reg(W_REGISTER+SETUP_RETR,0x1A); //�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10�� 0x1A
	
	NRF24l01_write_reg(W_REGISTER+EN_RXADDR,0x01);//ʹ��ͨ��0�Ľ��յ�ַ  
	NRF24l01_write_reg(W_REGISTER+EN_AA,0x01); //ʹ��ͨ��0�Զ�Ӧ��
	NRF24l01_write_reg(W_REGISTER+RX_PW_P0,RX_PAYLO_WIDTH);//ѡ��ͨ��0����Ч���ݿ��  
	NRF24L01_Write_Buf(W_REGISTER+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH); //дRX�ڵ��ַ
	NRF24l01_write_reg(W_REGISTER+RF_CH,40); //����RFͨ��Ϊ40hz(1-64Hz������)
	NRF24l01_write_reg(W_REGISTER+RF_SETUP,0x27); //����TX�������,0db����,2Mbps,����������ر� ��ע�⣺����������ر�/����ֱ��Ӱ��ͨ��,Ҫ������������Ҫ�رն��ر�0x0f��0x07
	
	NRFset_Mode(IT_RX); //Ĭ��Ϊ����ģʽ
	
	NRF_CE_HIGH;
}	

/*****************************************************************************
*��  ����uint8_t NRF24L01_TxPacket(uint8_t *txbuf)
*��  �ܣ�NRF����һ������
*��  ����txbuf��Ҫ�������ݵ�ַ
*����ֵ���� 
*��  ע����
*****************************************************************************/
void NRF24L01_TxPacket(uint8_t *txbuf)
{
	NRF_CE_LOW;	
	NRF24L01_Write_Buf(W_REGISTER+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);  //дTX�ڵ��ַ 
	//NRF24L01_Write_Buf(W_REGISTER+RX_ADDR_P0,(uint8_t*)TX_ADDRESS,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK
	NRF24L01_Write_Buf(W_REGISTER+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK
	NRF24L01_Write_Buf(W_RX_PAYLOAD,txbuf,TX_PAYLO_WIDTH); //д���ݵ�TX_BUFF
  NRF24l01_write_reg(W_REGISTER+CONFIG,IT_TX);	//����Ϊ����ģʽ,���������ж�
	NRF24l01_write_reg(W_REGISTER+STATUS,0X7E); //��������ж�,��ֹһ��ȥ����ģʽ�ʹ����ж�
  NRF_CE_HIGH;
	delay_us(10);  //CE�����ߵ�ƽ10us
}

/*****************************************************************************
*��  ����uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
*��  �ܣ�NRF����һ������
*��  ����rxbuf���������ݴ洢��ַ
*����ֵ����
*��  ע����
*****************************************************************************/
void NRF24L01_RxPacket(uint8_t *rxbuf)
{
	NRF_CE_LOW;
	NRF24L01_Read_Buf(R_RX_PAYLOAD,rxbuf,TX_PAYLO_WIDTH);//��ȡRX����Ч����
	NRF24l01_write_reg(FLUSH_RX,0xff); //���RX FIFO(ע�⣺��仰�ܱ�Ҫ)
	NRF_CE_HIGH;
}
/*****************************************************************************
*��  ����uint8_t NRF24L01_testConnection(void)
*��  �ܣ����NRF2401��MCU��SPI�����Ƿ�ͨ������
*��  ������
*����ֵ��1������ 0δ����
*��  ע����
*****************************************************************************/
uint8_t NRF24L01_testConnection(void)
{
	uint8_t buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	uint8_t i; 	 
	NRF24L01_Write_Buf(W_REGISTER+TX_ADDR,buf,5); //д��5���ֽڵĵ�ַ.	
	NRF24L01_Read_Buf(TX_ADDR,buf,5); //����д��ĵ�ַ  
	for(i=0;i<5;i++)
	if(buf[i]!=0XA5)break;	 							   
	if(i!=5){
		return 0; //���24L01����	
	}
	printf("2401 success");
	return 1;	//��⵽24L01
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
*��  ����void EXTI2_IRQHandler(void)
*��  �ܣ�NRF(ȫ˫��)�����жϴ�����
*��  ������
*����ֵ����
*��  ע: NRF�������ж��¼����ڴκ����и�����Ӧ�Ĵ���
*****************************************************************************/
void EXTI1_IRQHandler(void)
{        
	
	__HAL_GPIO_EXTI_CLEAR_IT(NRF_IRQ);

	uint8_t read_status = 	HAL_GPIO_ReadPin(NRF_IRQ_PORT,NRF_IRQ);
	uint8_t sta; 
	delay_us(1000);
	if( read_status == RESET )
	{
		 NRF_CE_LOW;//����CE���Ա��ȡNRF��STATUS�е�����
		 sta = NRF24l01_read_reg(R_REGISTER+STATUS); //��ȡSTATUS�е����ݣ��Ա��ж�����ʲô�ж�Դ������IRQ�ж�
		
		/* ��������ж� TX_OK */
		if(sta & TX_OK)                                   
		{										
		  NRFset_Mode(IT_RX);	
			NRF24l01_write_reg(W_REGISTER+STATUS,TX_OK); //���������ɱ�־��
		  NRF24l01_write_reg(FLUSH_TX,0xff); //���TX_FIFO
			// printf("Sent OK!!!!\r\n");
		}
		/* ��������ж� RX_OK */
		if(sta & RX_OK) 
		{	
			// 32
      NRF24L01_RxPacket(NRF_RX_DATA);	
			NRF24l01_write_reg(W_REGISTER+STATUS,RX_OK); //���������ɱ�־��
		// 	printf("rx data ok!!!!\r\n");

			// ң�������ݴ���
			Remote_Data_ReceiveAnalysis();
		}
		/* �ﵽ����ط������ж�  MAX_TX */
		if(sta & MAX_TX)                                  
		{											
			NRFset_Mode(IT_RX);
			NRF24l01_write_reg(W_REGISTER+STATUS,MAX_TX);//����Ӵﵽ����ط���־
			NRF24l01_write_reg(FLUSH_TX,0xff); //���TX_FIFO
		// 	printf("Sent Max Data!!!\r\n"); 
		}
	}
}

void Remote_Data_Event(void){
}

void Remote_Data_Send(void)
{
	NRF_TX_DATA[0] =remoter_buffer.type;//֡ͷ
	NRF_TX_DATA[1] = remoter_buffer.command; //��־λ��
  NRF_TX_DATA[2] =remoter_buffer.rcLock;
  NRF_TX_DATA[3] =remoter_buffer.ctrlMode;
  NRF_TX_DATA[4] = remoter_buffer.flightMode;
	NRF_TX_DATA[5] = remoter_buffer.length;
  
	// �ֱ���ȡ roll_value ��ÿ���ֽڣ���д�뵽 NRF_RX_DATA ����ĵ�6��9λ
	  union {
        float f;
        uint8_t bytes[sizeof(float)];
    } convertR,convertP,convertY,convertT,convertTrimPitch,convertTrimRoll;
/*
	 	��ĳЩϵͳ�У������������ڴ��ʾֱ��ת��Ϊ�������Ϳ��ܵ��¾��ȶ�ʧ������������ڸ������ڴ沼�ֺ����������ڴ沼��֮��Ĳ��졣��ˣ������½�����ֵ����Ϊ������ʱ���õ��Ľ�����ܲ���Ԥ�ڵĸ�����ֵ���������������׼ȷ��ֵ��
    Ϊ�˱����������⣬���Կ���ʹ�ø��ɿ��ķ�����������ת��Ϊ�ֽ����飬����ʹ���������⺯����
		*/
  convertR.f = remoter_buffer.roll;
	NRF_TX_DATA[6] = convertR.bytes[3];
	NRF_TX_DATA[7] = convertR.bytes[2];
	NRF_TX_DATA[8] = convertR.bytes[1];
	NRF_TX_DATA[9] = convertR.bytes[0];
		
  convertP.f = remoter_buffer.pitch;
	NRF_TX_DATA[10] = convertP.bytes[3];
	NRF_TX_DATA[11] = convertP.bytes[2];
	NRF_TX_DATA[12] = convertP.bytes[1];
	NRF_TX_DATA[13] = convertP.bytes[0];
		
   
  convertY.f = remoter_buffer.yaw;
	NRF_TX_DATA[14] = convertY.bytes[3];
	NRF_TX_DATA[15] = convertY.bytes[2];
	NRF_TX_DATA[16] = convertY.bytes[1];
	NRF_TX_DATA[17] = convertY.bytes[0];
		
  // Ĭ�϶���150cm
	convertT.f = remoter_buffer.thrust + 150;
	NRF_TX_DATA[18] = convertT.bytes[3];
	NRF_TX_DATA[19] = convertT.bytes[2];
	NRF_TX_DATA[20] = convertT.bytes[1];
	NRF_TX_DATA[21] = convertT.bytes[0];
		


	convertTrimPitch.f = remoter_buffer.trimPitch;
	NRF_TX_DATA[22] = convertTrimPitch.bytes[3];
	NRF_TX_DATA[23] = convertTrimPitch.bytes[2];
	NRF_TX_DATA[24] = convertTrimPitch.bytes[1];
	NRF_TX_DATA[25] = convertTrimPitch.bytes[0];
		
 
 
	convertTrimRoll.f = remoter_buffer.trimRoll;
	NRF_TX_DATA[26] = convertTrimRoll.bytes[3];
	NRF_TX_DATA[27] = convertTrimRoll.bytes[2];
	NRF_TX_DATA[28] = convertTrimRoll.bytes[1];
	NRF_TX_DATA[29] = convertTrimRoll.bytes[0];
		
	NRF_TX_DATA[30] = remoter_buffer.checksum;
//	printf("send data %f",remoter_buffer.pitch);

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
	remoter.length  = NRF_RX_DATA[5]; // ��Чλ ��6λ��Ч 00111111 ��6λ��Ч00 000000
	
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
 }

/*****************************************************************************
*��  ����void NRF_GetAddr(void)
*��  �ܣ����ɻ���ȡ�ϵ�NRF��ȡһ����ַ
*��  ������
*����ֵ���� 
*��  ע���˺�����Ҫ��ң�����Ķ�Ƶ��������ʹ�÷���NRFͨ�Ų��ɹ���
         ����Լ����ĵ�ң������ֱ���ù̶���ַ
****************************************************************************
void NRF_GetAddr(void)
{
	if(NRFaddr > NRFAddrMax)//�� NRFaddr����NRFAddrMax����˵����ʱNRF��δ��ʼ�����
	{
	  srand(SysTick->VAL);//�����������
		//printf("SysTick->VAL:%d\r\n",SysTick->VAL);
		NRFaddr = rand()%NRFAddrMax;//�����ȡNRF���һ�ֽڵ�ַ����ַ:0~50��
    PID_WriteFlash();//����˵�ַFlash
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
*��  ����void NRF_Test(void)
*��  �ܣ�MRFͨ�Ų��Ժ���
*��  ������
*����ֵ���� 
*��  ע������ʱ��
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


