#ifndef __CONTROL_DATA_H
#define __CONTROL_DATA_H
#include "data.h"
configParam_data configParam; // pid�־û�������
remoter_data remoter; /* ң��������  */
remoter_data remoter_buffer; // ң����������λ������

/**
	u8 ctrlMode	 	bit0  1=����ģʽ 0=�ֶ�ģʽ   bit1  1=����ģʽ
	u8 keyFlight 	 	bit2 һ�����
	u8 keyLand 		 	bit3 һ������
	u8 emerStop 	 	bit4 ����ͣ��
	u8 flightMode 	 	bit5 ����ģʽ 1=��ͷ 0=��ͷ
*/
commanderBits_data commanderBits;
self_data self; // �ɻ�����λ����Ϣ
ctrlval_data ctrlValLpf  = {0.f}; // ���տ�������
expect_data expect_set_point;
control_data control_info; // ����ң������õ���������Ŀ������

#endif
