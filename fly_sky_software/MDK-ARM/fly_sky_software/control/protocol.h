
#ifndef __PROTOCOL_H
#define __PROTOCOL_H
#include "main.h"
#include "data.h"
#include "outterControl.h"
#include "positionEstimate.h"

/*遥控数据类别*/
typedef enum 
{
	REMOTER_CMD,
	REMOTER_DATA,
}remoterType_e;


typedef enum
{
	XMODE     = 0, /*X模式*/
	CAREFREE  = 1, /*无头模式*/
} YawModeType;



/*下行命令*/
#define  CMD_GET_MSG		0x01	/*获取四轴信息（自检）*/
#define  CMD_GET_CANFLY		0x02	/*获取四轴是否能飞*/
#define  CMD_FLIGHT_LAND	0x03	/*起飞、降落*/
#define  CMD_EMER_STOP		0x04	/*紧急停机*/
#define  CMD_FLIP			0x05	/*4D翻滚*/
#define  CMD_POWER_MODULE	0x06	/*打开关闭扩展模块电源*/
#define  CMD_LEDRING_EFFECT	0x07	/*设置RGB灯环效果*/
#define  CMD_POWER_VL53LXX	0x08	/*打开关闭激光*/

/*上行报告*/
#define  ACK_MSG			0x01


///// atkp_t msgID dataLen 30字节数据
/////命令  dataLen  【命令| 具体命令 数据可以null】
/////数据  dataLen  【数据| 具体命令 数据可以null】
void commanderGetSetpoint(expect_data *setpoint, self_data *state);
void flyerAutoLand(expect_data *setpoint,const self_data *state);
static void ctrlDataUpdate(void);
static void rotateYawCarefree(expect_data *setpoint, const self_data *state);
static void commanderDropToGround(void);

static void commanderLevelRPY(void);
#endif 
