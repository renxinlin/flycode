#ifndef __CONTROL_DATA_H
#define __CONTROL_DATA_H
#include "data.h"
configParam_data configParam; // pid持久化的数据
remoter_data remoter;
remoter_data remoter_buffer;
commanderBits_data commanderBits;
self_data self; // 飞机自身位姿信息
ctrlval_data ctrlValLpf  = {0.f}; // 最终控制数据
uint8_t RCLock = 1; // 1 锁定 0 解锁
uint8_t TrimFlag = 0;  // 微调模式1 时 0否

#endif 
