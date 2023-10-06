#ifndef __DATA_H
#define __DATA_H
#include "stm32f1xx.h" 

#define DEG2RAD		0.017453293f	/* ��ת���� ��/180 */
#define RAD2DEG		57.29578f		/* ����ת�� 180/�� */
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
	uint32_t timestamp;	//ʱ���
	float distance;		//��������
	float quality;		//���Ŷ�
} zRange_t;

typedef  vec3_s attitude_data;
typedef  vec3_s velocity_data;
typedef  vec3_s point_data;
typedef  vec3_s acc_data;
typedef  vec3_s gyro_data;

typedef struct
{
	acc_data acc; // ���ٶ���Ϣ
	gyro_data gyro; // ���ٶ���Ϣ
	bmp_data baro; // ��ѹ����Ϣ
	point_data position; // ������ѹ�ƺͼ��ٶ�ת���z��λ����Ϣ
} sensor_data;


typedef struct
{
	acc_data acc;
	uint8_t isRCLocked; // 0 ���� 1 ����
	
	attitude_data attitude; // �Ƕ���Ϣ
	gyro_data attitudeRate;	// deg/s

	velocity_data velocity; // ���ٶ���Ϣ
	point_data position; // ������ѹ�ƺͼ��ٶ�ת���z��λ����Ϣ
	
} self_data;

typedef enum
{
	modeDisable,/*�ر�ģʽ */
	modeAbs,		/*����ֵģʽ*/
	modeVelocity	/*����ģʽ*/
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


/////////////////////////////pid ���/////////////////////////////pid ���/////////////////////////////
										
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
} pidParam_data; // �ǶȺͽ��ٶ�

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
	u8 version;				/*����汾��*/
	pidParam_data pidAngle;	/*�Ƕ�PID*/	
	pidParam_data pidRate;		/*���ٶ�PID*/	
	pidParamPos_data pidPos;	/*λ��PID*/
	float trimP;			/*pitch΢��*/
	float trimR;			/*roll΢��*/
	u16 thrustBase;			/*���Ż���ֵ*/
	u8 cksum;				/*У��*/
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
/////////////////////////////pid ���/////////////////////////////pid ���/////////////////////////////


/////////////////////////////��ͨ�˲� ���/////////////////////////////��ͨ�˲� ���/////////////////////////////

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


/////////////////////////////ң������ ���/////////////////////////////ң������ ���/////////////////////////////

enum ctrlMode
{
	ALTHOLD_MODE,// ����
	MANUAL_MODE,
	THREEHOLD_MODE, // ����
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
 uint8_t type; /*REMOTER_CMD REMOTER_DATA 0 ң�������� ң��������1 ....*/ // 31
 uint8_t command;/* REMOTER_CMD ����ֵ */
 uint8_t rcLock; /* 0 ���� 1 ����*/
 uint8_t ctrlMode; /* 1=����ģʽ 0=�ֶ�ģʽ */
 uint8_t flightMode;  /* ����ģʽ 1=��ͷ 0=��ͷ*/
 uint8_t length; // ��Чλ ��6λ��Ч 00111111 ��6λ��Ч00 000000
 float roll;    // 25 
 float pitch;  // 21
 float yaw;      // 17
 float thrust;      // 13
 float trimPitch;             // 9
 float trimRoll;             //5
 uint8_t checksum; // У��ֵ 1
} remoter_data; 

typedef __packed struct
{
	float roll;       // deg
	float pitch;      // deg
	float yaw;        // deg
	float trimPitch;
	float trimRoll;
	u16 thrust;
} ctrlval_data; // ������̬��������

typedef struct
{
	u8 ctrlMode		: 2;	/*bit0  1=����ģʽ 0=�ֶ�ģʽ   bit1  1=����ģʽ*/
	u8 keyFlight 	: 1;	/*bit2 һ�����*/
	u8 keyLand 		: 1;	/*bit3 һ������*/
	u8 emerStop 	: 1;	/*bit4 ����ͣ��*/
	u8 flightMode 	: 1;	/*bit5 ����ģʽ 1=��ͷ 0=��ͷ*/
	u8 reserved		: 2;	/*bit6~7 ����*/
}commanderBits_data;

/////////////////////////////���/////////////////////////////ң������ ���/////////////////////////////

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
extern remoter_data remoter; // ң�����´��ɻ�����
extern remoter_data remoter_buffer; // �ɻ��ϴ�ң��������
extern configParam_data configParam; // pid�־û�������
extern expect_data expect_set_point; // ����ң������õ���������Ŀ������
extern control_data control_info; // ����ң������õ���������Ŀ������

#endif 

