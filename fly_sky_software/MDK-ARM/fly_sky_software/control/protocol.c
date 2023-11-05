

#include "protocol.h"

#define CLIMB_RATE			100.f
#define MAX_CLIMB_UP		100.f
#define MAX_CLIMB_DOWN		60.f

#define MIN_THRUST  	5000
#define MAX_THRUST  	60000

static float minAccZ = 0.f; 
static float maxAccZ = 0.f; 


static float MAX_HEIGHT = 100.f;		/*定高模式最大高度*/

void commanderGetSetpoint(expect_data *setpoint, self_data *state)
{	
	// ctrlValLpf
	static float maxAccZ = 0.f;
	state->isRCLocked = remoter.rcLock;	/*更新遥控器锁定状态*/
	if(remoter.ctrlMode == ALTHOLD_MODE)/*定高模式*/
	{
			setpoint->thrust = 0;
			setpoint->mode.z = modeAbs;
		// 遥感0~1000 中间是50cm,也就是正常飞50cm 暂时跟随
		setpoint->position.z = (remoter.thrust/655.35) > MAX_HEIGHT ? MAX_HEIGHT:remoter.thrust ;	// 飞机暂定不能高于1M
	}else{
		// 油门模式
		setpoint->thrust = remoter.thrust;	
		setpoint->mode.z = modeDisable;
	}
	
	setpoint->attitude.x = remoter.roll;
	setpoint->attitude.y = remoter.pitch;
	setpoint->attitude.z  = remoter.yaw;	/*摇杆方向和yaw方向相反*/
	// 没有x,y维度的位姿控制,只能简单控制姿态 
	setpoint->mode.x = modeDisable;
	setpoint->mode.y = modeDisable;		
	
	setpoint->mode.roll = modeDisable;	
	setpoint->mode.pitch = modeDisable;	
	setpoint->mode.yaw = modeDisable;	
	/*
	无头模式
	if(commander.flightMode)
	{
		yawMode = CAREFREE;		
		rotateYawCarefree(setpoint, state);
	}		
	else	
	{
	*/
		/*X飞行模式*/
	//}		
}





static void rotateYawCarefree(expect_data *setpoint, const self_data *state)
{
	float yawRad = state->attitude.z * DEG2RAD;
	float cosy = cosf(yawRad);
	float siny = sinf(yawRad);
	
	if(setpoint->mode.x ==  modeDisable || setpoint->mode.y ==  modeDisable)	/*手动和定高模式*/
	{
		float originalRoll = setpoint->attitude.x;
		float originalPitch = setpoint->attitude.y;

		setpoint->attitude.x = originalRoll * cosy + originalPitch * siny;
		setpoint->attitude.y = originalPitch * cosy - originalRoll * siny;
	}
	else if(setpoint->mode.x ==  modeVelocity || setpoint->mode.y ==  modeVelocity)	/*定点模式*/
	{
		float originalVy = setpoint->velocity.y;
		float originalVx = setpoint->velocity.x;

		setpoint->velocity.y = originalVy * cosy + originalVx * siny;
		setpoint->velocity.x = originalVx * cosy - originalVy * siny;
	}
}

 






