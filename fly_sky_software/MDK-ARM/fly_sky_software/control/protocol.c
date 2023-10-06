

#include "protocol.h"

#define CLIMB_RATE			100.f
#define MAX_CLIMB_UP		100.f
#define MAX_CLIMB_DOWN		60.f

#define MIN_THRUST  	5000
#define MAX_THRUST  	60000

// nrf2401���������Ǿͷ�������Ļ�����,nrf����ͷŵ�����

static float minAccZ = 0.f; 
static float maxAccZ = 0.f; 

static YawModeType yawMode = XMODE;	/* Ĭ��ΪX����ģʽ */
static commanderBits_data commander;
static uint8_t isRCLocked = 0;				/* ң������״̬ 1�� 0����*/

static uint8_t initHigh ;
static uint8_t isAdjustingPosZ  ; /*����Zλ��*/
static uint8_t isAdjustingPosXY = 1; /*����XYλ��*/
static uint8_t adjustPosXYTime = 0;		/*XYλ�õ���ʱ��*/

remoter_data remoter_lpf;// ң������ͨ������
static float errorPosX = 0.f;		/*Xλ�����*/
static float errorPosY = 0.f;		/*Yλ�����*/
static float errorPosZ = 0.f;		/*Zλ�����*/

void commanderGetSetpoint(expect_data *setpoint, self_data *state)
{	
	// ctrlValLpf
	static float maxAccZ = 0.f;
	ctrlDataUpdate();	/*����ң������������*/
	// mock���� todo ��nrf2401��ȡ
	commanderBits.ctrlMode = ALTHOLD_MODE; 
	commanderBits.keyFlight = 1; 
	commanderBits.keyLand = 0;
	commanderBits.emerStop = 0;
	commanderBits.flightMode = 0; // 0��ͷ 1��ͷ
	state->isRCLocked = remoter.rcLock;	/*����ң��������״̬*/
	
	if(commander.ctrlMode == ALTHOLD_MODE)/*����ģʽ*/
	{
		if(commander.keyFlight)/*һ�����*/ 
		{	
			setpoint->thrust = 0;
			setpoint->mode.z = modeAbs;		
			setpoint->position.z = ctrlValLpf.thrust;				
		}
	}
	
 	
	setpoint->attitude.x = remoter.roll;
	setpoint->attitude.y = remoter.pitch;
	setpoint->attitude.z  = remoter.yaw;	/*ҡ�˷����yaw�����෴*/
	//	printf("  remoter->x %f ,%f,%f \r\n",remoter.roll,remoter.pitch,remoter.yaw);
	//	printf("  setpoint->attitude %f ,%f,%f \r\n",setpoint->attitude.x,setpoint->attitude.y,setpoint->attitude.z);

	// û��x,yά�ȵ�λ�˿���,ֻ�ܼ򵥿�����̬ 
	setpoint->mode.x = modeDisable;
	setpoint->mode.y = modeDisable;		
	
	
	setpoint->mode.roll = modeDisable;	
	setpoint->mode.pitch = modeDisable;	
	/*
	��ͷģʽ
	if(commander.flightMode)
	{
		yawMode = CAREFREE;		
		rotateYawCarefree(setpoint, state);
	}		
	else	
	{
	*/
		/*X����ģʽ*/
		yawMode = XMODE;
	//}		
}

static void ctrlDataUpdate(void)	
{
	// �������ڿ���ʱ���,ң�������ݶ�ʧ�󼴶��ߣ������ǽ���
	isRCLocked  = remoter.rcLock; // 1����
	ctrlValLpf.thrust = remoter.thrust;  
	ctrlValLpf.pitch = remoter.pitch;  
	ctrlValLpf.roll = remoter.roll;
	ctrlValLpf.yaw = remoter.yaw;	
	configParam.trimP = remoter.trimPitch;	/*����΢��ֵ*/
	configParam.trimR = remoter.trimRoll;
}


static void commanderLevelRPY(void)
{
	ctrlValLpf.roll = 0;
	ctrlValLpf.pitch = 0;
	ctrlValLpf.yaw = 0;
}

static void commanderDropToGround(void)
{
	commanderLevelRPY();
	ctrlValLpf.thrust = 0;
	if(commander.keyFlight)	/*���й����У�ң�����źŶϿ���һ������*/
	{
		commander.keyLand = 1;
		commander.keyFlight = 0;
	}	
}

static void rotateYawCarefree(expect_data *setpoint, const self_data *state)
{
	float yawRad = state->attitude.z * DEG2RAD;
	float cosy = cosf(yawRad);
	float siny = sinf(yawRad);
	
	if(setpoint->mode.x ==  modeDisable || setpoint->mode.y ==  modeDisable)	/*�ֶ��Ͷ���ģʽ*/
	{
		float originalRoll = setpoint->attitude.x;
		float originalPitch = setpoint->attitude.y;

		setpoint->attitude.x = originalRoll * cosy + originalPitch * siny;
		setpoint->attitude.y = originalPitch * cosy - originalRoll * siny;
	}
	else if(setpoint->mode.x ==  modeVelocity || setpoint->mode.y ==  modeVelocity)	/*����ģʽ*/
	{
		float originalVy = setpoint->velocity.y;
		float originalVx = setpoint->velocity.x;

		setpoint->velocity.y = originalVy * cosy + originalVx * siny;
		setpoint->velocity.x = originalVx * cosy - originalVy * siny;
	}
}


void flyerAutoLand(expect_data *setpoint,const self_data *state)
{	
	static u8 lowThrustCnt = 0;
	static float stateVelLpf  = -30.f;
	
	setpoint->mode.z = modeVelocity;
	stateVelLpf += (state->velocity.z -  stateVelLpf) * 0.1f;	/*���ʵ�ͨ*/
	setpoint->velocity.z = -70.f - stateVelLpf;	/*�����ٶ� ��λcm/s*/

	if(getAltholdThrust() < 20000.f)	/*��������ֵ�ϵ�*/
	{
		lowThrustCnt++;
		if(lowThrustCnt > 10)
		{
			lowThrustCnt = 0;
			commander.keyLand = 0;
			commander.keyFlight = 0;
			estRstAll();	/*��λ����*/
		}	
	}else
	{
		lowThrustCnt = 0;
	}
	
	//if(isExitFlip == true)	/*�˳��շ����ټ����ٶ�*/
	//{
		float accZ = state->acc.z;
		if(minAccZ > accZ)
			minAccZ = accZ;
		if(maxAccZ < accZ)
			maxAccZ = accZ;
	//}
	
	if (minAccZ < -80.f && maxAccZ > 320.f)
	{
		commander.keyLand = 0;
		commander.keyFlight = 0;
		estRstAll();	/*��λ����*/
	}	
}







