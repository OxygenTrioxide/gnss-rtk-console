#ifndef OEM7_ASCII_H
#define OEM7_ASCII_H

#include "RTK_Structs.h"

// NovAtel OEM7 ASCII 记录通用解码框架（课件 1-5 第 18 页）
//   支持消息：RANGEA（观测历元）、GPSEPHEMA（GPS 星历）、BDSEPHEMA（BDS 星历）、
//             PSRPOSA / BESTPOSA（定位结果，基站坐标来源见课件 2-1 第 15 页）
// 输入：line 为一条完整 ASCII 记录（以 '#' 开头、以 '\n' 结尾，可含 '\r'）
// 返回：0 = 非上述消息或 CRC 校验失败；1 = 解出一个观测历元；
//       2 = 解出 GPS 星历；3 = 解出 BDS 星历；4 = 解出定位结果
// 说明：obs 仅在返回 1 时被重写；geph/beph/pos 仅在返回对应码时更新。
//       观测赋值、星历字段映射均与二进制解码路径（oem7_decode.cpp /
//       oem7_ephem.cpp）保持一致，仅记录封装格式不同。
int DecodeOem7AsciiLine(const char* line, EPOCHOBSDATA* obs,
    GPSEPHREC geph[], GPSEPHREC beph[], POSRES* pos);

#endif // OEM7_ASCII_H
