#ifndef __CONTROL_DATA_H
#define __CONTROL_DATA_H
#include "data.h"
configParam_data configParam; // pid�־û�������
remoter_data remoter;
remoter_data remoter_buffer;
commanderBits_data commanderBits;
self_data self; // �ɻ�����λ����Ϣ
ctrlval_data ctrlValLpf  = {0.f}; // ���տ�������
uint8_t RCLock = 1; // 1 ���� 0 ����
uint8_t TrimFlag = 0;  // ΢��ģʽ1 ʱ 0��

#endif 
