

#include "protocol.h"

#define CLIMB_RATE			100.f
#define MAX_CLIMB_UP		100.f
#define MAX_CLIMB_DOWN		60.f

#define MIN_THRUST  	5000
#define MAX_THRUST  	60000

static float minAccZ = 0.f; 
static float maxAccZ = 0.f; 


static float MAX_HEIGHT = 100.f;		/*����ģʽ���߶�*/

void commanderGetSetpoint(expect_data *setpoint, self_data *state)
{	
	// ctrlValLpf
	static float maxAccZ = 0.f;
	state->isRCLocked = remoter.rcLock;	/*����ң��������״̬*/
	if(remoter.ctrlMode == ALTHOLD_MODE)/*����ģʽ*/
	{
			setpoint->thrust = 0;
			setpoint->mode.z = modeAbs;
		// ң��0~1000 �м���50cm,Ҳ����������50cm ��ʱ����
		setpoint->position.z = (remoter.thrust/655.35) > MAX_HEIGHT ? MAX_HEIGHT:remoter.thrust ;	// �ɻ��ݶ����ܸ���1M
	}else{
		// ����ģʽ
		setpoint->thrust = remoter.thrust;	
		setpoint->mode.z = modeDisable;
	}
	
	setpoint->attitude.x = remoter.roll;
	setpoint->attitude.y = remoter.pitch;
	setpoint->attitude.z  = remoter.yaw;	/*ҡ�˷����yaw�����෴*/
	// û��x,yά�ȵ�λ�˿���,ֻ�ܼ򵥿�����̬ 
	setpoint->mode.x = modeDisable;
	setpoint->mode.y = modeDisable;		
	
	setpoint->mode.roll = modeDisable;	
	setpoint->mode.pitch = modeDisable;	
	setpoint->mode.yaw = modeDisable;	
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
	//}		
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

 






