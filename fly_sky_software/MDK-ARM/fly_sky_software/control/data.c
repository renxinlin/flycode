#ifndef __CONTROL_DATA_H
#define __CONTROL_DATA_H
#include "data.h"
configParam_data configParam; // pid持久化的数据
remoter_data remoter; /* 遥控器数据  */
remoter_data remoter_buffer; // 遥控器发送上位机数据

/**
	u8 ctrlMode	 	bit0  1=定高模式 0=手动模式   bit1  1=定点模式
	u8 keyFlight 	 	bit2 一键起飞
	u8 keyLand 		 	bit3 一键降落
	u8 emerStop 	 	bit4 紧急停机
	u8 flightMode 	 	bit5 飞行模式 1=无头 0=有头
*/
commanderBits_data commanderBits;
self_data self; // 飞机自身位姿信息
ctrlval_data ctrlValLpf  = {0.f}; // 最终控制数据
expect_data expect_set_point;
control_data control_info; // 根据遥控器获得的期望设置目标数据

#endif
