#ifndef __CONTROL_DATA_H
#define __CONTROL_DATA_H
#include "data.h"
configParam_data configParam; // pid�־û�������
remoter_data remoter;
commanderBits_data commanderBits;
self_data self; // �ɻ�����λ����Ϣ
ctrlval_data ctrlValLpf  = {0.f}; // ���տ�������

#endif
