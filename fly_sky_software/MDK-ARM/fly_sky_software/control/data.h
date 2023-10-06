#ifndef __DATA_H
#define __DATA_H
#include "stm32f1xx.h" 

#define DEG2RAD		0.017453293f	/* 度转弧度 π/180 */
#define RAD2DEG		57.29578f		/* 弧度转度 180/π */
typedef struct
{
	float pressure;
	float temperature;
	float asl;
} bmp_data;


typedef struct 
{
	float x;
	float y;
	float z;
} vec3_s;

typedef struct zRange_s 
{
	uint32_t timestamp;	//时间戳
	float distance;		//测量距离
	float quality;		//可信度
} zRange_t;

typedef  vec3_s attitude_data;
typedef  vec3_s velocity_data;
typedef  vec3_s point_data;
typedef  vec3_s acc_data;
typedef  vec3_s gyro_data;

typedef struct
{
	acc_data acc; // 角速度信息
	gyro_data gyro; // 加速度信息
	bmp_data baro; // 气压计信息
	point_data position; // 根据气压计和加速度转算的z轴位置信息
} sensor_data;


typedef struct
{
	acc_data acc;
	uint8_t isRCLocked; // 0 正常 1 上锁
	
	attitude_data attitude; // 角度信息
	gyro_data attitudeRate;	// deg/s

	velocity_data velocity; // 加速度信息
	point_data position; // 根据气压计和加速度转算的z轴位置信息
	
} self_data;

typedef enum
{
	modeDisable,/*关闭模式 */
	modeAbs,		/*绝对值模式*/
	modeVelocity	/*速率模式*/
} mode_e;

typedef struct
{
	mode_e x;
	mode_e y;
	mode_e z;
	mode_e roll;
	mode_e pitch;
	mode_e yaw;
}mode_t;


typedef struct
{
	attitude_data attitude;		// deg	
	gyro_data attitudeRate;	// deg/s
	point_data position;         	// m
	velocity_data velocity;      	// m/s
	mode_t mode;
	float thrust;
} expect_data;




/* Orientation as a quaternion */
typedef struct 
{
	uint32_t timestamp;
	float q0;
	float q1;
	float q2;
	float q3;
	
	
} quaternion_data;

typedef struct
{
	acc_data acc;
	gyro_data gyro;
 	bmp_data baro;
	point_data position;
	zRange_t zrange;
} sensorData_data;

typedef struct
{
	attitude_data attitude;
	quaternion_data quaternion;
	
	point_data position;
	velocity_data velocity;
	acc_data acc;
	uint8_t isRCLocked;
} expire_pid_data;

typedef struct
{
	int roll;
	int pitch;
	int yaw;
	float thrust;
} control_data;


/////////////////////////////pid 相关/////////////////////////////pid 相关/////////////////////////////
										
typedef struct 
{
	float kp;
	float ki;
	float kd;
} pidInit_data; 

typedef struct
{
	pidInit_data roll;
	pidInit_data pitch;	
	pidInit_data yaw;	
} pidParam_data; // 角度和角速度

typedef struct
{
	pidInit_data vx;
	pidInit_data vy;
	pidInit_data vz;
	
	pidInit_data x;
	pidInit_data y;
	pidInit_data z;
} pidParamPos_data;

typedef struct	
{
	u8 version;				/*软件版本号*/
	pidParam_data pidAngle;	/*角度PID*/	
	pidParam_data pidRate;		/*角速度PID*/	
	pidParamPos_data pidPos;	/*位置PID*/
	float trimP;			/*pitch微调*/
	float trimR;			/*roll微调*/
	u16 thrustBase;			/*油门基础值*/
	u8 cksum;				/*校验*/
} configParam_data;





typedef struct
{
	float desired;		  //< set point
	float error;        //< error
	float prevError;    //< previous error
	float integ;        //< integral
	float deriv;        //< derivative
	
	float kp;           //< proportional gain
	float ki;           //< integral gain
	float kd;           //< derivative gain
	
	
	float outP;         //< proportional output (debugging)
	float outI;         //< integral output (debugging)
	float outD;         //< derivative output (debugging)
	
	float iLimit;       //< integral limit
	float outputLimit;  //< total PID output limit, absolute value. '0' means no limit.
	
	float dt;           //< delta-time dt
	float out;		
} pid_data;
/////////////////////////////pid 相关/////////////////////////////pid 相关/////////////////////////////


/////////////////////////////低通滤波 相关/////////////////////////////低通滤波 相关/////////////////////////////

typedef struct 
{
	float a1;
	float a2;
	
	float b0;
	float b1;
	float b2;
	
	float delay_element_1;
	float delay_element_2;
} lpf2pData;


/////////////////////////////遥控数据 相关/////////////////////////////遥控数据 相关/////////////////////////////

enum ctrlMode
{
	ALTHOLD_MODE,// 定高
	MANUAL_MODE,
	THREEHOLD_MODE, // 定点
};

enum flightMode
{
	HEAD_LESS,
	X_MODE,
};

enum flightSpeed
{
	LOW_SPEED,
	MID_SPEED,
	HIGH_SPEED,
};

typedef __packed struct
{
 uint8_t type; /*REMOTER_CMD REMOTER_DATA 0 遥控器命令 遥控器数据1 ....*/ // 31
 uint8_t command;/* REMOTER_CMD 命令值 */
 uint8_t rcLock; /* 0 上锁 1 解锁*/
 uint8_t ctrlMode; /* 1=定高模式 0=手动模式 */
 uint8_t flightMode;  /* 飞行模式 1=无头 0=有头*/
 uint8_t length; // 有效位 后6位有效 00111111 后6位无效00 000000
 float roll;    // 25 
 float pitch;  // 21
 float yaw;      // 17
 float thrust;      // 13
 float trimPitch;             // 9
 float trimRoll;             //5
 uint8_t checksum; // 校验值 1
} remoter_data; 

typedef __packed struct
{
	float roll;       // deg
	float pitch;      // deg
	float yaw;        // deg
	float trimPitch;
	float trimRoll;
	u16 thrust;
} ctrlval_data; // 最后的姿态控制数据

typedef struct
{
	u8 ctrlMode		: 2;	/*bit0  1=定高模式 0=手动模式   bit1  1=定点模式*/
	u8 keyFlight 	: 1;	/*bit2 一键起飞*/
	u8 keyLand 		: 1;	/*bit3 一键降落*/
	u8 emerStop 	: 1;	/*bit4 紧急停机*/
	u8 flightMode 	: 1;	/*bit5 飞行模式 1=无头 0=有头*/
	u8 reserved		: 2;	/*bit6~7 保留*/
}commanderBits_data;

/////////////////////////////电机/////////////////////////////遥控数据 相关/////////////////////////////

typedef struct 
{
	uint32_t m1;
	uint32_t m2;
	uint32_t m3;
	uint32_t m4;
	
}motorPwm_data;


extern ctrlval_data ctrlValLpf;
extern commanderBits_data commanderBits;
extern self_data self;
extern remoter_data remoter; // 遥控器下传飞机数据
extern remoter_data remoter_buffer; // 飞机上传遥控器数据
extern configParam_data configParam; // pid持久化的数据
extern expect_data expect_set_point; // 根据遥控器获得的期望设置目标数据
extern control_data control_info; // 根据遥控器获得的期望设置目标数据

#endif 

