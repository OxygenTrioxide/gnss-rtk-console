#ifndef EPH_ORBIT_H
#define EPH_ORBIT_H

#include "RTK_Structs.h"

// 广播星历 → 卫星 ECEF 位置/速度 + 钟差/钟速
// 输入 t_tx：信号发射时刻的周内秒，时系与星历一致
//   （GPS 星历用 GPST，BDS 星历用 BDT，由调用方统一换算）
// 输出 pos[3]（米）、vel[3]（米/秒）、*clk（秒）、*clkrate（秒/秒，含相对论改正导数）
void EphToECEF(const GPSEPHREC* eph, double t_tx, double pos[3], double vel[3],
    double* clk, double* clkrate);

// 星历健康与有效性检查：
//   SVHealth 必须为 0（健康），且 |t - TOE| 不超过有效期（GPS 2h，BDS 1h）
// 输入 t_sow：观测时刻周内秒，时系与星历一致（BDS 用 BDT）
bool EphUsable(const GPSEPHREC* eph, double t_sow);

#endif
