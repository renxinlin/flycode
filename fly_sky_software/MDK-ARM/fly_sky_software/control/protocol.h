
#ifndef __PROTOCOL_H
#define __PROTOCOL_H
#include "main.h"
#include "data.h"
#include "outterControl.h"
#include "positionEstimate.h"

/*ң���������*/
typedef enum 
{
	REMOTER_CMD,
	REMOTER_DATA,
}remoterType_e;


typedef enum
{
	XMODE     = 0, /*Xģʽ*/
	CAREFREE  = 1, /*��ͷģʽ*/
} YawModeType;



/*��������*/
#define  CMD_GET_MSG		0x01	/*��ȡ������Ϣ���Լ죩*/
#define  CMD_GET_CANFLY		0x02	/*��ȡ�����Ƿ��ܷ�*/
#define  CMD_FLIGHT_LAND	0x03	/*��ɡ�����*/
#define  CMD_EMER_STOP		0x04	/*����ͣ��*/
#define  CMD_FLIP			0x05	/*4D����*/
#define  CMD_POWER_MODULE	0x06	/*�򿪹ر���չģ���Դ*/
#define  CMD_LEDRING_EFFECT	0x07	/*����RGB�ƻ�Ч��*/
#define  CMD_POWER_VL53LXX	0x08	/*�򿪹رռ���*/

/*���б���*/
#define  ACK_MSG			0x01


///// atkp_t msgID dataLen 30�ֽ�����
/////����  dataLen  ������| �������� ���ݿ���null��
/////����  dataLen  ������| �������� ���ݿ���null��
void commanderGetSetpoint(expect_data *setpoint, self_data *state);
void flyerAutoLand(expect_data *setpoint,const self_data *state);
static void ctrlDataUpdate(void);
static void rotateYawCarefree(expect_data *setpoint, const self_data *state);
static void commanderDropToGround(void);

static void commanderLevelRPY(void);
#endif 
