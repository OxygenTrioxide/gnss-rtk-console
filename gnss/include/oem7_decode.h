#ifndef OEM7_DECODE_H
#define OEM7_DECODE_H

#include "RTK_Structs.h"

// 主解码函数（课件 1-5 第 55 页）
//   Buff/Len 语义：进入时 Len = Buff 中有效字节数；返回时 Len = 剩余有效字节数，
//   剩余字节已移动到 Buff 起始处，实现二进制数据流的拼接。
//   basPos 为非空时，顺带取出数据流中的 PSRPOS(47)/BESTPOS(42) 定位结果
//   （课件 2-1 第 15 页：基站坐标来自基站的 bestpos 数据）。
//   返回 1 = 已解出一个历元观测；0 = 尚未解出；负值不使用（由调用方判断数据是否耗尽）
int DecodeNovOem7Dat(unsigned char Buff[], int& Len, EPOCHOBSDATA* obs,
    GPSEPHREC geph[], GPSEPHREC beph[], POSRES* basPos = nullptr);

// 数据提取子函数（课件 1-5 第 58 页）
int decode_rangeb_oem7(unsigned char* buff, EPOCHOBSDATA* obs);
int decode_gpsephem(unsigned char* buff, GPSEPHREC* eph);
int decode_bdsephem(unsigned char* buff, GPSEPHREC* eph);
int decode_psrpos(unsigned char* buff, POSRES* pos);

#endif // OEM7_DECODE_H
