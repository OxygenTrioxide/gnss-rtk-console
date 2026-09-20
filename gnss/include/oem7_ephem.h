#ifndef OEM7_EPHEM_H
#define OEM7_EPHEM_H

#include "RTK_Structs.h"

// GPS 广播星历 (OEM7 消息 ID = 7)
int decode_gpsephem(unsigned char* buff, GPSEPHREC* geph);

// BDS 广播星历 (OEM7 消息 ID = 1696)
int decode_bdsephem(unsigned char* buff, GPSEPHREC* beph);

#endif