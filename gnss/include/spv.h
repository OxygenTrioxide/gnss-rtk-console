#ifndef SPV_H
#define SPV_H

#include "RTK_Structs.h"

// 单点测速：利用多普勒观测解算接收机速度 + 接收机钟漂
bool SPV(EPOCHOBSDATA* Epoch, RAWDAT* Raw, POSRES* Result);

#endif // SPV_H