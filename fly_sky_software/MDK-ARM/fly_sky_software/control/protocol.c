

#include "protocol.h"

#define CLIMB_RATE			100.f
#define MAX_CLIMB_UP		100.f
#define MAX_CLIMB_DOWN		60.f

#define MIN_THRUST  	5000
#define MAX_THRUST  	60000

// nrf2401的数据我们就放在这里的缓冲区,nrf读完就放到这里

static float minAccZ = 0.f; 
static float maxAccZ = 0.f; 

static YawModeType yawMode = XMODE;	/* 默认为X飞行模式 */
static commanderBits_data commander;
static uint8_t isRCLocked = 0;				/* 遥控锁定状态 1打开 0上锁*/

static uint8_t initHigh ;
static uint8_t isAdjustingPosZ  ; /*调整Z位置*/
static uint8_t isAdjustingPosXY = 1; /*调整XY位置*/
static uint8_t adjustPosXYTime = 0;		/*XY位置调整时间*/

remoter_data remoter_lpf;// 遥控器低通后数据
static float errorPosX = 0.f;		/*X位移误差*/
static float errorPosY = 0.f;		/*Y位移误差*/
static float errorPosZ = 0.f;		/*Z位移误差*/

void commanderGetSetpoint(expect_data *setpoint, self_data *state)
{	
	// ctrlValLpf
	static float maxAccZ = 0.f;
	ctrlDataUpdate();	/*更新遥控器控制数据*/
	// mock数据 todo 从nrf2401获取
	commanderBits.ctrlMode = ALTHOLD_MODE; 
	commanderBits.keyFlight = 1; 
	commanderBits.keyLand = 0;
	commanderBits.emerStop = 0;
	commanderBits.flightMode = 0; // 0有头 1无头
	state->isRCLocked = remoter.rcLock;	/*更新遥控器锁定状态*/
	
	if(commander.ctrlMode == ALTHOLD_MODE)/*定高模式*/
	{
		if(commander.keyFlight)/*一键起飞*/ 
		{	
			setpoint->thrust = 0;
			setpoint->mode.z = modeAbs;		
			setpoint->position.z = ctrlValLpf.thrust;				
		}
	}
	
 	
	setpoint->attitude.x = remoter.roll;
	setpoint->attitude.y = remoter.pitch;
	setpoint->attitude.z  = remoter.yaw;	/*摇杆方向和yaw方向相反*/
	//	printf("  remoter->x %f ,%f,%f \r\n",remoter.roll,remoter.pitch,remoter.yaw);
	//	printf("  setpoint->attitude %f ,%f,%f \r\n",setpoint->attitude.x,setpoint->attitude.y,setpoint->attitude.z);

	// 没有x,y维度的位姿控制,只能简单控制姿态 
	setpoint->mode.x = modeDisable;
	setpoint->mode.y = modeDisable;		
	
	
	setpoint->mode.roll = modeDisable;	
	setpoint->mode.pitch = modeDisable;	
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
		yawMode = XMODE;
	//}		
}

static void ctrlDataUpdate(void)	
{
	// 锁定不在考虑时间戳,遥控器数据丢失后即定高，而不是降落
	isRCLocked  = remoter.rcLock; // 1解锁
	ctrlValLpf.thrust = remoter.thrust;  
	ctrlValLpf.pitch = remoter.pitch;  
	ctrlValLpf.roll = remoter.roll;
	ctrlValLpf.yaw = remoter.yaw;	
	configParam.trimP = remoter.trimPitch;	/*更新微调值*/
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
	if(commander.keyFlight)	/*飞行过程中，遥控器信号断开，一键降落*/
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


void flyerAutoLand(expect_data *setpoint,const self_data *state)
{	
	static u8 lowThrustCnt = 0;
	static float stateVelLpf  = -30.f;
	
	setpoint->mode.z = modeVelocity;
	stateVelLpf += (state->velocity.z -  stateVelLpf) * 0.1f;	/*速率低通*/
	setpoint->velocity.z = -70.f - stateVelLpf;	/*降落速度 单位cm/s*/

	if(getAltholdThrust() < 20000.f)	/*定高油门值较低*/
	{
		lowThrustCnt++;
		if(lowThrustCnt > 10)
		{
			lowThrustCnt = 0;
			commander.keyLand = 0;
			commander.keyFlight = 0;
			estRstAll();	/*复位估测*/
		}	
	}else
	{
		lowThrustCnt = 0;
	}
	
	//if(isExitFlip == true)	/*退出空翻，再检测加速度*/
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
		estRstAll();	/*复位估测*/
	}	
}







