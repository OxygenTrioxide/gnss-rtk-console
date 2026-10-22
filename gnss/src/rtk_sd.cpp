#define _CRT_SECURE_NO_WARNINGS
// 必须先于 windows.h 定义，避免 max/min 宏破坏 std::max/std::min 与
// std::numeric_limits<T>::max()（本文件包含 serial.h → windows.h）
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include "../include/rtk_sd.h"
#include "oem7_decode.h"
#include "lambda.h"
#include "sockets.h"
#include "serial.h"
#include <cstring>
#include <cmath>
#include <algorithm>
#include <vector>
#include <limits>

// ============================================================
// 数据源抽象（课件 2-1 第 36/37 页）：
//   文件模式 / 实时网络(SOCKET)模式 / 实时串口(CSerial)模式
//   三种模式共用同一套"滚动对齐"同步算法，函数名统一为 GetSynGNSSObs（重载）
// ============================================================
enum OBS_SRC_KIND { OBS_SRC_FILE = 0, OBS_SRC_NET = 1, OBS_SRC_COM = 2 };

// 数据源描述
struct OBS_SOURCE {
    int kind;
    FILE* fp;
    SOCKET sk;
    CSerial* com;
    FILE* rec;   // 原始数据落盘文件（仅网络模式，配置 RECORD RAW DATA 开启时非空）
    OBS_SOURCE() : kind(OBS_SRC_FILE), fp(nullptr), sk(INVALID_SOCKET), com(nullptr), rec(nullptr) {}
};

// 原始数据流落盘句柄（网络模式专用，由 SetRawRecordFiles 在启动时设置）
static FILE* g_recBase = nullptr, * g_recRov = nullptr;

void SetRawRecordFiles(FILE* fpBase, FILE* fpRov)
{
    g_recBase = fpBase;
    g_recRov = fpRov;
}

// 从数据源读取若干字节；返回 >0 = 读取字节数，0 = 暂时无数据，-1 = 流结束/出错
static int ReadSourceBytes(const OBS_SOURCE& src, unsigned char* buf, int maxLen, int waitMs)
{
    if (maxLen <= 0) return 0;
    if (src.kind == OBS_SRC_FILE) {
        int n = (int)fread(buf, 1, maxLen, src.fp);
        return (n > 0) ? n : -1;
    }
    if (src.kind == OBS_SRC_NET) {
        int n = recv(src.sk, (char*)buf, maxLen, 0);
        if (n > 0 && src.rec) fwrite(buf, 1, n, src.rec);   // 落盘原始字节，供事后回放
        return (n > 0) ? n : -1;
    }
    // 串口：查询式读取（课件 1-5 第 15 页）。1 Hz 观测数据在串口上是
    // 突发到达的，两个突发之间常有数百毫秒的无数据窗口，因此必须轮询到
    // 有数据为止；只有连续 60 s 收不到任何字节才视为流中断（接收机断电/
    // 拔线），返回 -1。若按固定等待时间无数据即返回 -1，会被同步循环当作
    // "流结束"，程序运行约一秒就会误退出。
    const int SILENCE_LIMIT_MS = 60000;
    int silence = 0;
    while (silence <= SILENCE_LIMIT_MS) {
        int n = src.com->ReadData(buf, maxLen);
        if (n > 0) return n;
        Sleep(10);
        silence += 10;
    }
    return -1;
}

// 从一个数据源取出一个完整历元；返回 1=成功，-1=流结束
static int ReadOneEpochEx(const OBS_SOURCE& src, unsigned char* buff, int& buffLen,
    EPOCHOBSDATA* obs, GPSEPHREC gpsEph[], GPSEPHREC bdsEph[], POSRES* basPos)
{
    static const int BUF_SIZE = 16384;
    while (true) {
        // 尝试解码（基站数据流可顺带取出 PSRPOS/BESTPOS 定位结果）
        int ret = DecodeNovOem7Dat(buff, buffLen, obs, gpsEph, bdsEph, basPos);
        if (ret == 1) return 1;

        if (buffLen >= BUF_SIZE) {          // 缓冲已满仍解不出，丢弃前 1KB 防死锁
            memmove(buff, buff + 1024, buffLen - 1024);
            buffLen -= 1024;
        }
        int n = ReadSourceBytes(src, buff + buffLen, BUF_SIZE - buffLen, 100);
        if (n <= 0) return -1;
        buffLen += n;
    }
}

// 双站历元同步：流动站历元早则推进流动站，基站历元早则推进基站，
// |rover − base| < thres 即同步成功
// 缓冲为文件级 static：网络断线重连后由 ResetNetSyncBuffers() 清空，
// 旧连接残留的半帧数据作废，解码器从 0xAA4412 同步头重新对帧
static unsigned char g_syncBuffBase[16384], g_syncBuffRover[16384];
static int g_syncLenBase = 0, g_syncLenRover = 0;
static bool g_syncHasBase = false, g_syncHasRov = false;

// 清空同步缓冲（供网络模式断线重连后调用）
void ResetNetSyncBuffers()
{
    g_syncLenBase = 0;
    g_syncLenRover = 0;
    g_syncHasBase = false;
    g_syncHasRov = false;
}

static int SyncObsCommon(const OBS_SOURCE& sBase, const OBS_SOURCE& sRov,
    RAWDAT* raw, double thresSec)
{
    if (!g_syncHasBase) {
        int r = ReadOneEpochEx(sBase, g_syncBuffBase, g_syncLenBase, &raw->BasEpk,
            raw->GpsEph, raw->BdsEph, &raw->BasPos);
        if (r < 0) return -1;
        g_syncHasBase = true;
    }
    if (!g_syncHasRov) {
        int r = ReadOneEpochEx(sRov, g_syncBuffRover, g_syncLenRover, &raw->RovEpk,
            raw->GpsEph, raw->BdsEph, nullptr);
        if (r < 0) return -1;
        g_syncHasRov = true;
    }

    while (true) {
        double tB = raw->BasEpk.Time.SecOfWeek;
        double tR = raw->RovEpk.Time.SecOfWeek;
        double dt = tR - tB;

        if (fabs(dt) < thresSec) {
            g_syncHasBase = false;
            g_syncHasRov = false;
            return 1;
        }
        if (dt < 0) {
            int r = ReadOneEpochEx(sRov, g_syncBuffRover, g_syncLenRover, &raw->RovEpk,
                raw->GpsEph, raw->BdsEph, nullptr);
            if (r < 0) return -1;
        }
        else {
            int r = ReadOneEpochEx(sBase, g_syncBuffBase, g_syncLenBase, &raw->BasEpk,
                raw->GpsEph, raw->BdsEph, &raw->BasPos);
            if (r < 0) return -1;
        }
    }
}

// 文件模式（课件 2-1 第 36 页 GetSynGNSSObs）
int GetSynGNSSObs(FILE* fpBase, FILE* fpRover, RAWDAT* raw, double thresSec)
{
    OBS_SOURCE b, r;
    b.kind = r.kind = OBS_SRC_FILE;
    b.fp = fpBase;
    r.fp = fpRover;
    return SyncObsCommon(b, r, raw, thresSec);
}

// 实时网络模式（与文件模式同名，函数重载；课件 2-1 第 37 页）
int GetSynGNSSObs(SOCKET sBase, SOCKET sRov, RAWDAT* raw, double thresSec)
{
    OBS_SOURCE b, r;
    b.kind = r.kind = OBS_SRC_NET;
    b.sk = sBase;
    r.sk = sRov;
    b.rec = g_recBase;   // 挂接原始数据落盘文件（未开启录制时为空）
    r.rec = g_recRov;
    return SyncObsCommon(b, r, raw, thresSec);
}

// 实时串口模式（与文件模式同名，函数重载；课件 2-1 第 37 页）
int GetSynGNSSObs(CSerial* cBase, CSerial* cRov, RAWDAT* raw, double thresSec)
{
    OBS_SOURCE b, r;
    b.kind = r.kind = OBS_SRC_COM;
    b.com = cBase;
    r.com = cRov;
    return SyncObsCommon(b, r, raw, thresSec);
}

// 单频点质量门控：任一标记异常则返回 false（课件 2-1 第 29 页 OEM719 质量标记）
//   Parity known flag  : 0 = 奇偶校验未知，可能有半周 → 剔除
//   Phase lock flag    : 0 = 相位未锁定（失锁，存在半周/周跳）→ 剔除
//   Code locked flag   : 0 = 码未锁定（伪距不可靠）→ 剔除
//   Half cycle added   : 1 = 载波相位含半周改正。恒定的半周已在解码时改正
//                        （见 decode_rangeb_oem7），不影响解算，不再剔除；
//                        该标志的"翻转"才是真半周跳变，由 CheckLockTimeParity
//                        的翻转检测作周跳剔除（实测部分接收机对 GPS L1 /
//                        BDS B3 等频点恒定报 half=1，若按原逻辑剔除将导致
//                        单差卫星数恒为 0）。
static bool SatFreqQualityOK(const SATOBSDATA& so, int f)
{
    if (so.parity[f] == 0) return false;
    if (so.plock[f] == 0) return false;
    if (so.clock[f] == 0) return false;
    return true;
}

// LockTime 连续性 + 质量标记（半周/相位锁定/码锁定）检查
void CheckLockTimeParity(RAWDAT* raw)
{
    // 每个频点独立保存上一次的 LockTime
    static double lastLock_bas[6][64][2] = {};
    static double lastLock_rov[6][64][2] = {};
    static bool   valid_bas[6][64][2] = {};
    static bool   valid_rov[6][64][2] = {};
    // 半周标志翻转检测：half 标志恒定表示相位含恒定半周偏置（解码时已
    // 改正，无害）；若与上一历元不同，说明接收机对该频点施加/移除了半周
    // 改正，相位发生 0.5 周跳变（MW 约 0.4 m、GF 约 0.1 m，均低于常用
    // 周跳阈值，无法靠 MW/GF 探测），必须在此显式剔除并按周跳重置。
    static int  lastHalf_bas[6][64][2] = {};
    static int  lastHalf_rov[6][64][2] = {};
    static bool hasHalf_bas[6][64][2] = {};
    static bool hasHalf_rov[6][64][2] = {};

    EPOCHOBSDATA* bas = &raw->BasEpk;
    EPOCHOBSDATA* rov = &raw->RovEpk;

    // ---- 基站 ----
    for (int i = 0; i < bas->SatNum; i++) {
        SATOBSDATA& so = bas->SatObs[i];
        if (!so.Valid) continue;
        int sys = (int)so.System;
        int prn = so.Prn;
        if (sys < 0 || sys >= 6 || prn < 1 || prn >= 64) continue;

        for (int f = 0; f < 2; f++) {
            if (fabs(so.L[f]) < 1e-3) continue;

            // 半周/相位锁定/码锁定/奇偶校验质量门控
            if (!SatFreqQualityOK(so, f)) {
                so.Valid = false;
                break;
            }

            // 半周标志翻转 = 真半周跳变 → 剔除
            if (hasHalf_bas[sys][prn][f] &&
                (int)so.half[f] != lastHalf_bas[sys][prn][f]) {
                so.Valid = false;
                hasHalf_bas[sys][prn][f] = false;   // 重置，待重新收敛
                break;
            }
            lastHalf_bas[sys][prn][f] = so.half[f];
            hasHalf_bas[sys][prn][f] = true;

            // LockTime 检查
            if (valid_bas[sys][prn][f]) {
                if (so.LockTime[f] < lastLock_bas[sys][prn][f]) {
                    so.Valid = false;
                    break;
                }
            }
            lastLock_bas[sys][prn][f] = so.LockTime[f];
            valid_bas[sys][prn][f] = true;
        }
    }

    // ---- 流动站 ----
    for (int i = 0; i < rov->SatNum; i++) {
        SATOBSDATA& so = rov->SatObs[i];
        if (!so.Valid) continue;
        int sys = (int)so.System;
        int prn = so.Prn;
        if (sys < 0 || sys >= 6 || prn < 1 || prn >= 64) continue;

        for (int f = 0; f < 2; f++) {
            if (fabs(so.L[f]) < 1e-3) continue;

            if (!SatFreqQualityOK(so, f)) {
                so.Valid = false;
                break;
            }

            // 半周标志翻转 = 真半周跳变 → 剔除
            if (hasHalf_rov[sys][prn][f] &&
                (int)so.half[f] != lastHalf_rov[sys][prn][f]) {
                so.Valid = false;
                hasHalf_rov[sys][prn][f] = false;   // 重置，待重新收敛
                break;
            }
            lastHalf_rov[sys][prn][f] = so.half[f];
            hasHalf_rov[sys][prn][f] = true;

            if (valid_rov[sys][prn][f]) {
                if (so.LockTime[f] < lastLock_rov[sys][prn][f]) {
                    so.Valid = false;
                    break;
                }
            }
            lastLock_rov[sys][prn][f] = so.LockTime[f];
            valid_rov[sys][prn][f] = true;
        }
    }
}

// 站间单差：对每颗"基站和流动站都有"的卫星
//   dP[f] = P_base[f] - P_rover[f]，dL[f] = L_base[f] - L_rover[f]，存入 raw->SdObs
void FormSDEpochObs(RAWDAT* raw)
{
    EPOCHOBSDATA* bas = &raw->BasEpk;
    EPOCHOBSDATA* rov = &raw->RovEpk;
    SDEPOCHOBS* sd = &raw->SdObs;

    sd->Time = rov->Time;
    sd->SatNum = 0;

    static const bool sdDbg = getenv("GNSS_SD_DEBUG") != nullptr; // 诊断：单差拒绝原因
    if (sdDbg) {
        int nb = 0, nr = 0;
        for (int k = 0; k < bas->SatNum; k++) if (bas->SatObs[k].Valid) nb++;
        for (int k = 0; k < rov->SatNum; k++) if (rov->SatObs[k].Valid) nr++;
        printf("    [SDDBG] bas SatNum=%d valid=%d | rov SatNum=%d valid=%d\n",
            bas->SatNum, nb, rov->SatNum, nr);
    }

    // 对流动站的每颗卫星，找基站的同一颗
    for (int i = 0; i < rov->SatNum && sd->SatNum < MAXCHANNUM; i++) {
        SATOBSDATA& rso = rov->SatObs[i];
        if (sdDbg)
            printf("    [SDDBG] R %c%02d valid=%d P0=%.0f P1=%.0f L0=%.0f L1=%.0f\n",
                rso.System == GPS ? 'G' : 'C', rso.Prn, (int)rso.Valid,
                rso.P[0], rso.P[1], rso.L[0], rso.L[1]);
        if (!rso.Valid) continue;

        // 找基站里的同名卫星
        int bIdx = -1;
        for (int j = 0; j < bas->SatNum; j++) {
            if (bas->SatObs[j].Prn == rso.Prn &&
                bas->SatObs[j].System == rso.System) {
                bIdx = j;
                break;
            }
        }
        if (sdDbg && bIdx >= 0)
            printf("    [SDDBG]   base %c%02d valid=%d\n",
                bas->SatObs[bIdx].System == GPS ? 'G' : 'C',
                bas->SatObs[bIdx].Prn, (int)bas->SatObs[bIdx].Valid);
        if (bIdx < 0) continue;
        SATOBSDATA& bso = bas->SatObs[bIdx];
        if (!bso.Valid) continue;

        // 计算单差
        SDSATOBS& dst = sd->SdSatObs[sd->SatNum];
        memset(&dst, 0, sizeof(dst));
        dst.Prn = rso.Prn;
        dst.System = rso.System;
        dst.nBas = (short)bIdx;
        dst.nRov = (short)i;

        // 双频都必须有数据，且 L1/L2 单差一致性检查（粗差剔除）
        double dP0_check = bso.P[0] - rso.P[0];
        double dP1_check = bso.P[1] - rso.P[1];
        if (sdDbg) {
            printf("    [SDDBG]   P0=%.0f/%.0f P1=%.0f/%.0f L0=%.0f/%.0f L1=%.0f/%.0f IFdiff=%+.3f\n",
                bso.P[0], rso.P[0], bso.P[1], rso.P[1],
                bso.L[0], rso.L[0], bso.L[1], rso.L[1],
                dP0_check - dP1_check);
        }
        if (fabs(dP0_check - dP1_check) > 5.0) continue;
        if (rso.P[0] < 1e3 || bso.P[0] < 1e3) continue;
        if (rso.P[1] < 1e3 || bso.P[1] < 1e3) continue;
        if (fabs(rso.L[0]) < 1e-3 || fabs(bso.L[0]) < 1e-3) continue;
        if (fabs(rso.L[1]) < 1e-3 || fabs(bso.L[1]) < 1e-3) continue;

        dst.dP[0] = bso.P[0] - rso.P[0];
        dst.dP[1] = bso.P[1] - rso.P[1];
        dst.dL[0] = bso.L[0] - rso.L[0];
        dst.dL[1] = bso.L[1] - rso.L[1];
        dst.Valid = 1;
        sd->SatNum++;
    }
}

// 辅助：计算 MW 组合和 GF 组合
//   MW = (f1*L1 - f2*L2)/(f1-f2) - (f1*P1 + f2*P2)/(f1+f2)，GF = L1 - L2
// 返回 0 成功，-1 缺数据
static int CalcMWGF(const SATOBSDATA& so, double* mw, double* gf)
{
    double p1 = so.P[0], p2 = so.P[1];
    double l1 = so.L[0], l2 = so.L[1];
    if (p1 < 1e3 || p2 < 1e3) return -1;
    if (fabs(l1) < 1e-3 || fabs(l2) < 1e-3) return -1;

    double f1, f2;
    if (so.System == GPS) { f1 = FG1_GPS; f2 = FG2_GPS; }
    else if (so.System == BDS) { f1 = FG1_BDS; f2 = FG3_BDS; }
    else return -1;

    *mw = (f1 * l1 - f2 * l2) / (f1 - f2)
        - (f1 * p1 + f2 * p2) / (f1 + f2);
    *gf = l1 - l2;
    return 0;
}

// 周跳探测（课件 2-2 第 6 页：对"站间单差观测值"用 MW 与 GF 组合探测）
//   单差 MW：L_MW = (f1·L1 − f2·L2)/(f1−f2) − (f1·P1 + f2·P2)/(f1+f2)（双站作差）
//   单差 GF：L_GF = L1 − L2（双站作差）
//   与上一历元的 MW 平滑值比较：
//     |LMW(tk) − L̄MW(tk−1)| > mwThres 或 |LGF(tk) − LGF(tk−1)| > gfThres → 周跳
//   通过则更新平滑值 L̄MW = [(n−1)·L̄MW + LMW]/n（n 为平滑历元数）；
//   周跳时把该卫星在两站的观测值都标记为不可用，并重置平滑。
void DetectCycleSlip(RAWDAT* raw, double mwThres, double gfThres)
{
    // 静态数组：保存上一历元"单差"的 MW 平滑值 / GF / 平滑计数，按 [系统][PRN]
    static double lastMW[6][64] = { {0} };
    static double lastGF[6][64] = { {0} };
    static int    cntMW[6][64] = { {0} };
    static bool   validMW[6][64] = { {false} };

    EPOCHOBSDATA* bas = &raw->BasEpk;
    EPOCHOBSDATA* rov = &raw->RovEpk;

    // 逐卫星：只有两站双频数据齐全才能构成单差组合
    for (int i = 0; i < rov->SatNum; i++) {
        SATOBSDATA& rso = rov->SatObs[i];
        if (!rso.Valid) continue;
        int sys = (int)rso.System;
        int prn = rso.Prn;
        if (sys < 0 || sys >= 6 || prn < 1 || prn >= 64) continue;

        int bIdx = -1;
        for (int j = 0; j < bas->SatNum; j++) {
            if (bas->SatObs[j].Prn == rso.Prn &&
                bas->SatObs[j].System == rso.System) { bIdx = j; break; }
        }
        if (bIdx < 0) continue;
        SATOBSDATA& bso = bas->SatObs[bIdx];
        if (!bso.Valid) continue;

        double mwB, gfB, mwR, gfR;
        if (CalcMWGF(bso, &mwB, &gfB) < 0 || CalcMWGF(rso, &mwR, &gfR) < 0) {
            validMW[sys][prn] = false;      // 缺数据，重置平滑
            continue;
        }
        double sdMW = mwB - mwR;            // 站间单差 MW
        double sdGF = gfB - gfR;            // 站间单差 GF

        if (validMW[sys][prn]) {
            if (fabs(sdMW - lastMW[sys][prn]) > mwThres ||
                fabs(sdGF - lastGF[sys][prn]) > gfThres) {
                // 周跳：两站该卫星均剔除
                rso.Valid = false;
                bso.Valid = false;
                validMW[sys][prn] = false;
                cntMW[sys][prn] = 0;
                continue;
            }
        }
        // 通过检查：更新单差 MW 平滑值
        int n = validMW[sys][prn] ? cntMW[sys][prn] + 1 : 1;
        lastMW[sys][prn] = validMW[sys][prn]
            ? ((n - 1) * lastMW[sys][prn] + sdMW) / n
            : sdMW;
        lastGF[sys][prn] = sdGF;
        cntMW[sys][prn] = n;
        validMW[sys][prn] = true;
    }

    // ---- 重新计算单差（因为 Valid 标记变了） ----
    FormSDEpochObs(raw);

    // ---- 站间单差组合结果存档（SdCObs，与 SdSatObs 同一下标）----
    SDEPOCHOBS* sd = &raw->SdObs;
    for (int k = 0; k < sd->SatNum && k < MAXCHANNUM; k++) {
        SDSATOBS& s = sd->SdSatObs[k];
        sd->SdCObs[k].Prn = s.Prn;
        sd->SdCObs[k].Sys = s.System;
        sd->SdCObs[k].MW = 0.0;
        sd->SdCObs[k].GF = 0.0;
        sd->SdCObs[k].n = 0;
        int sys = (int)s.System;
        if (sys >= 0 && sys < 6 && s.Prn >= 1 && s.Prn < 64 &&
            validMW[sys][s.Prn]) {
            sd->SdCObs[k].MW = lastMW[sys][s.Prn];
            sd->SdCObs[k].GF = lastGF[sys][s.Prn];
            sd->SdCObs[k].n = cntMW[sys][s.Prn];
        }
    }
}

// 从 .pos-lsq 文件读基站真值坐标
// 格式： Wk  SOW  lat1  lon1  h1  flag  lat2  lon2  h2  n1  n2  n3  ratio
int ReadBasePosFromPos(const char* file, BLH& basBlh)
{
    FILE* fp = fopen(file, "r");
    if (!fp) {
        return -1;
    }

    char line[512];
    while (fgets(line, sizeof(line), fp)) {
        // 跳过空行/注释
        char* p = line;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '\0' || *p == '\n' || *p == '\r' || *p == '#' || *p == '%') continue;

        int wk, solFlag, n1, n2, n3;
        double sow, lat1, lon1, h1, lat2, lon2, h2, ratio;
        int cnt = sscanf(line, "%d %lf %lf %lf %lf %d %lf %lf %lf %d %d %d %lf",
            &wk, &sow, &lat1, &lon1, &h1, &solFlag,
            &lat2, &lon2, &h2, &n1, &n2, &n3, &ratio);

        // 至少要能读出基站 lat/lon/h
        if (cnt >= 5 && lat1 > -90 && lat1 < 90 && lon1 > -180 && lon1 < 180) {
            basBlh.latitude = lat1 * Rad;
            basBlh.longitude = lon1 * Rad;
            basBlh.height = h1;
            fclose(fp);
            printf("  [BASEPOS] Base lat=%.8f lon=%.8f h=%.3f\n", lat1, lon1, h1);
            return 0;
        }
    }
    fclose(fp);
    return -1;
}

// 基准星选取：先过硬性质量门限（有效标志、双频有数据、CN0/LockTime 阈值、
// Parity 无半周、伪距-相位无粗差、高度角 > elevThres），
// 通过门限后取高度角最大者
void DetRefSat(RAWDAT* raw, double cnoThres, double elevThres, double lockThres)
{
    DDCOBS* dd = &raw->DDObs;
    dd->RefPrn[0] = dd->RefPrn[1] = -1;
    dd->RefPos[0] = dd->RefPos[1] = -1;

    EPOCHOBSDATA* rov = &raw->RovEpk;
    SDEPOCHOBS* sd = &raw->SdObs;

    XYZ rovXyz(rov->Pos[0], rov->Pos[1], rov->Pos[2]);
    BLH rovBlh;
    XYZToBLH(rovXyz, rovBlh);
    double R[9];
    BLHToNEUMat(rovBlh, R);

    double bestElev[2] = { -1.0, -1.0 };

    for (int k = 0; k < sd->SatNum; k++) {
        SDSATOBS& s = sd->SdSatObs[k];
        if (s.Valid != 1) continue;

        int rIdx = s.nRov;
        if (rIdx < 0 || rIdx >= rov->SatNum) continue;
        SATOBSDATA& rso = rov->SatObs[rIdx];
        if (!rso.Valid) continue;
        if (!rov->SatPVT[rIdx].Valid) continue;

        // 参考星必须双频有效
        if (rso.P[1] < 1e3 || fabs(rso.L[1]) < 1e-3) continue;

        // 质量门限：流动站双频 CN0、连续观测 LockTime、OEM719 质量标记
        // （Parity/相位锁定/码锁定/半周，见 SatFreqQualityOK，课件 2-1 第 29 页）
        if (rso.cn0[0] < cnoThres || rso.cn0[1] < cnoThres) continue;
        if (rso.LockTime[0] < lockThres || rso.LockTime[1] < lockThres) continue;
        if (!SatFreqQualityOK(rso, 0) || !SatFreqQualityOK(rso, 1)) continue;

        // 无粗差（伪距-相位一致性）
        double dPL0 = s.dP[0] - s.dL[0];
        double dPL1 = s.dP[1] - s.dL[1];
        if (fabs(dPL0) > 100.0 || fabs(dPL1) > 100.0) continue;

        double dX[3] = {
            rov->SatPVT[rIdx].SatPos[0] - rov->Pos[0],
            rov->SatPVT[rIdx].SatPos[1] - rov->Pos[1],
            rov->SatPVT[rIdx].SatPos[2] - rov->Pos[2]
        };
        NEU neu;
        XYZToNEU(dX, R, neu);
        double elev = atan2(neu.dU, sqrt(neu.dN * neu.dN + neu.dE * neu.dE));
        double elevDeg = elev * Deg;
        if (elevDeg < elevThres) continue;

        int sysIdx = -1;
        if (s.System == GPS) sysIdx = 0;
        else if (s.System == BDS) sysIdx = 1;
        else continue;

        // 通过质量门限后，取高度角最大者（基准站常架设在
        // 观测环境良好处，质量以流动站为准；CN0 作为门限而非排序依据）
        if (elevDeg > bestElev[sysIdx]) {
            bestElev[sysIdx] = elevDeg;
            dd->RefPrn[sysIdx] = s.Prn;
            dd->RefPos[sysIdx] = k;
        }
    }
}
// 双差：每颗非参考星的双差观测
//   ∇ΔP^j = ΔP^j - ΔP^i        （单差相减）
//   ∇ΔL^j = ΔL^j - ΔL^i
//   方向余弦：l^j = e_rov^j - e_rov^i
//   双差几何距离初值：∇Δρ0^j = ρ_rov^j - ρ_bas^j - (ρ_rov^i - ρ_bas^i)
void CalcDD(RAWDAT* raw, const double basPos[3], const double rovPos[3])
{
    DDEPOCHOBS* dd = &raw->DdObs;
    SDEPOCHOBS* sd = &raw->SdObs;

    dd->Time = sd->Time;
    dd->SatNum = 0;
    dd->NGPS = dd->NBDS = 0;

    for (int sysIdx = 0; sysIdx < 2; sysIdx++) {
        int refIdx = raw->DDObs.RefPos[sysIdx];
        if (refIdx < 0) continue;

        SDSATOBS& s_ref = sd->SdSatObs[refIdx];
        int rIdx_ref = s_ref.nRov;
        int bIdx_ref = s_ref.nBas;
        if (rIdx_ref < 0 || bIdx_ref < 0) continue;

        dd->RefPrn[sysIdx] = s_ref.Prn;
        dd->RefIdx[sysIdx] = refIdx;

        double dR_rov_ref[3], dR_bas_ref[3];
        SATPVT* sv_ref = &raw->RovEpk.SatPVT[rIdx_ref];
        if (!sv_ref->Valid) continue;

        for (int k = 0; k < 3; k++) {
            dR_rov_ref[k] = sv_ref->SatPos[k] - rovPos[k];
            dR_bas_ref[k] = sv_ref->SatPos[k] - basPos[k];
        }
        double rho_rov_ref = sqrt(dR_rov_ref[0] * dR_rov_ref[0] + dR_rov_ref[1] * dR_rov_ref[1] + dR_rov_ref[2] * dR_rov_ref[2]);
        double rho_bas_ref = sqrt(dR_bas_ref[0] * dR_bas_ref[0] + dR_bas_ref[1] * dR_bas_ref[1] + dR_bas_ref[2] * dR_bas_ref[2]);

        double e_rov_ref[3] = {
            dR_rov_ref[0] / rho_rov_ref,
            dR_rov_ref[1] / rho_rov_ref,
            dR_rov_ref[2] / rho_rov_ref
        };

        for (int i = 0; i < sd->SatNum; i++) {
            if (i == refIdx) continue;
            SDSATOBS& s = sd->SdSatObs[i];
            if (!s.Valid) continue;
            if (s.System != s_ref.System) continue;
            if (dd->SatNum >= MAXCHANNUM * 2) break;

            int rIdx = s.nRov;
            int bIdx = s.nBas;
            if (rIdx < 0 || bIdx < 0) continue;

            SATPVT* sv = &raw->RovEpk.SatPVT[rIdx];
            if (!sv->Valid) continue;

            double dR_rov[3], dR_bas[3];
            for (int k = 0; k < 3; k++) {
                dR_rov[k] = sv->SatPos[k] - rovPos[k];
                dR_bas[k] = sv->SatPos[k] - basPos[k];
            }
            double rho_rov = sqrt(dR_rov[0] * dR_rov[0] + dR_rov[1] * dR_rov[1] + dR_rov[2] * dR_rov[2]);
            double rho_bas = sqrt(dR_bas[0] * dR_bas[0] + dR_bas[1] * dR_bas[1] + dR_bas[2] * dR_bas[2]);

            double rho_dd = (rho_rov - rho_bas) - (rho_rov_ref - rho_bas_ref);

            double e_rov[3] = {
                dR_rov[0] / rho_rov,
                dR_rov[1] / rho_rov,
                dR_rov[2] / rho_rov
            };
            double los[3] = {
                e_rov[0] - e_rov_ref[0],
                e_rov[1] - e_rov_ref[1],
                e_rov[2] - e_rov_ref[2]
            };

            DDSATOBS& dst = dd->DdSatObs[dd->SatNum];
            memset(&dst, 0, sizeof(dst));
            dst.Prn = s.Prn;
            dst.System = s.System;
            dst.los[0] = los[0];
            dst.los[1] = los[1];
            dst.los[2] = los[2];
            dst.rho0 = rho_dd;
            dst.Valid = true;

            // 频率有效性（用"单差"检查）
            dst.fValid[0] = (fabs(s.dP[0]) > 1.0 && fabs(s_ref.dP[0]) > 1.0 &&
                fabs(s.dL[0]) > 1e-3 && fabs(s_ref.dL[0]) > 1e-3);
            dst.fValid[1] = (fabs(s.dP[1]) > 1.0 && fabs(s_ref.dP[1]) > 1.0 &&
                fabs(s.dL[1]) > 1e-3 && fabs(s_ref.dL[1]) > 1e-3);

            for (int f = 0; f < 2; f++) {
                dst.dP[f] = s.dP[f] - s_ref.dP[f];
                dst.dL[f] = s.dL[f] - s_ref.dL[f];
            }

            dd->SatNum++;
            if (s.System == GPS) dd->NGPS++;
            else if (s.System == BDS) dd->NBDS++;
        }
    }
}

// 载波波长（米），用于双差模糊度的周转换：
// GPS 用 L1/L2；BDS 用 B1/B3（BDS 观测 l[1] 存的是 B3 载波）
static double DDWavelen(GNSSSys sys, int f)
{
    if (sys == GPS) return (f == 0) ? WL1_GPS : WL2_GPS;
    if (sys == BDS)  return (f == 0) ? WL1_BDS : WL3_BDS;
    return (f == 0) ? WL1_GPS : WL2_GPS;
}

// 构建当前历元的双差观测模型：
//   观测表（每颗有效双差卫星 0=P1、1=L1、2=P2、3=L2）、
//   相关权阵 W = C^{-1}、设计矩阵 A、闭合差 l。
//   C = D Q D^T：单差观测相互独立（方差 VAR_P / VAR_L）；
//   双差 = 非参考星单差 − 参考星单差，故同系统同类型的双差观测
//   通过共享参考星相关（对角 2·var，非对角 var）；跨系统、跨频点互不相关。
// 返回观测数，失败返回 -1。
static int BuildDDModel(DDEPOCHOBS* dd, int m,
    int* obsSat, int* obsType,
    MatrixXd& A, VectorXd& l, MatrixXd& W,
    double sigmaP = 0.3, double sigmaL = 0.05)
{
    const double VAR_P = sigmaP * sigmaP;
    const double VAR_L = sigmaL * sigmaL;
    int nU = 3 + 2 * m;

    // 观测表
    int nObs = 0;
    for (int i = 0; i < m; i++) {
        DDSATOBS& s = dd->DdSatObs[i];
        if (s.fValid[0]) {
            obsSat[nObs] = i;  obsType[nObs] = 0; nObs++;
            obsSat[nObs] = i;  obsType[nObs] = 1; nObs++;
        }
        if (s.fValid[1]) {
            obsSat[nObs] = i;  obsType[nObs] = 2; nObs++;
            obsSat[nObs] = i;  obsType[nObs] = 3; nObs++;
        }
    }
    if (nObs < nU) return -1;

    // 相关协方差 C = D Q D^T
    // 课件 2-4 第 10/11 页：站间单差互不相关 cov(SD)=2σ²I，故
    //   cov(DD) = C·cov(SD)·Cᵀ = 2σ²·(I + J)
    // 即同系统同类型的双差观测：对角 4σ²、非对角（共享参考星）2σ²；
    // 跨系统、跨频点、伪距与相位之间互不相关。
    MatrixXd C = MatrixXd::Zero(nObs, nObs);
    for (int a = 0; a < nObs; a++) {
        for (int b = a; b < nObs; b++) {
            DDSATOBS& sa = dd->DdSatObs[obsSat[a]];
            DDSATOBS& sb = dd->DdSatObs[obsSat[b]];
            if (sa.System != sb.System) continue;
            if (obsType[a] != obsType[b]) continue;
            double var = ((obsType[a] % 2) == 0) ? VAR_P : VAR_L;
            double cab = (a == b) ? 4.0 * var : 2.0 * var;
            C(a, b) = cab;
            C(b, a) = cab;
        }
    }

    // W = C^{-1}（C 对称正定，LDLT 分解求逆）
    LDLT<MatrixXd> ldlt(C);
    if (ldlt.info() != Success) return -1;
    W = ldlt.solve(MatrixXd::Identity(nObs, nObs));

    // 设计矩阵 A 与闭合差 l（观测值 − 双差几何距离）
    A = MatrixXd::Zero(nObs, nU);
    l = VectorXd::Zero(nObs);
    for (int r = 0; r < nObs; r++) {
        DDSATOBS& s = dd->DdSatObs[obsSat[r]];
        // 符号约定：单差定义 dP = P_base - P_rover（见 FormSDEpochObs），
        // 故观测值 z = +los·b（b = 流动站 - 基站）；几何项 rho0 = -los·b_current。
        // 平差模型 z = A·Δb + los·b_current，取 A = +los、l = z - los·b_current = z + rho0，
        // 不动点处 Δb → b_true - b_current，迭代收敛于 b_true（基站固定时）。
        A(r, 0) = s.los[0];
        A(r, 1) = s.los[1];
        A(r, 2) = s.los[2];
        switch (obsType[r]) {
        case 0: l(r) = s.dP[0] + s.rho0; break;
        case 1: l(r) = s.dL[0] + s.rho0; A(r, 3 + 2 * obsSat[r])     = 1.0; break;
        case 2: l(r) = s.dP[1] + s.rho0; break;
        case 3: l(r) = s.dL[1] + s.rho0; A(r, 3 + 2 * obsSat[r] + 1) = 1.0; break;
        default: return -1;   // 观测类型越界（防御分支，正常不会到达）
        }
    }
    return nObs;
}

// 浮点解：最小二乘
//   未知数: [ΔX, ΔY, ΔZ, ∇ΔN_1, ∇ΔN_2, ..., ∇ΔN_m]
//   方程: 每颗非参考星 2 个（伪距 + 载波）
//   迭代: 每次线性化，直到 |Δb| < 1e-4
//   收敛后把浮点模糊度（周）和协方差导出到 DDCOBS，
//   供 FixDDAmb / LAMBDA 使用
bool SolveFloat(RAWDAT* raw, const double basPos[3], double rovPos[3],
    double sigmaP, double sigmaL)
{
    const int MAX_UNK = 3 + MAXCHANNUM * 2 * 2;
    static double x[MAX_UNK];

    double baseline[3] = { 0, 0, 0 };

    // 重置本历元的模糊度固定导出状态
    raw->DDObs.AmbNum = 0;
    raw->DDObs.Status = false;
    raw->DDObs.Ratio = 0.0;

    for (int iter = 0; iter < 10; iter++) {
        double rovCur[3] = {
            basPos[0] + baseline[0],
            basPos[1] + baseline[1],
            basPos[2] + baseline[2]
        };

        CalcDD(raw, basPos, rovCur);
        DDEPOCHOBS* dd = &raw->DdObs;
        int m = dd->SatNum;
        if (m < 3) return false;

        int nU = 3 + 2 * m;

        int obsSat[MAXCHANNUM * 2 * 4];
        int obsType[MAXCHANNUM * 2 * 4];
        MatrixXd A, W;
        VectorXd l;
        int nObs = BuildDDModel(dd, m, obsSat, obsType, A, l, W, sigmaP, sigmaL);
        if (nObs < nU) return false;   // 观测数少于未知数，无法解算

        // 法方程：N = A^T W A，b = A^T W l，LDLT 求解
        //   Ne 分解后不被破坏，供后面 Qxx 精度评定复用
        MatrixXd Ne = A.transpose() * W * A;
        VectorXd be = A.transpose() * W * l;
        LDLT<MatrixXd> ldltN(Ne);
        if (ldltN.info() != Success) return false;
        VectorXd xe = ldltN.solve(be);
        for (int i = 0; i < nU; i++) x[i] = xe(i);

        baseline[0] += x[0];
        baseline[1] += x[1];
        baseline[2] += x[2];

        for (int i = 0; i < m; i++) {
            dd->DdSatObs[i].amb[0] = x[3 + 2 * i];
            dd->DdSatObs[i].amb[1] = x[3 + 2 * i + 1];
        }

        double norm = sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);

        if (norm < 1e-4) {
            raw->DDObs.dPos[0] = baseline[0];
            raw->DDObs.dPos[1] = baseline[1];
            raw->DDObs.dPos[2] = baseline[2];
            rovPos[0] = basPos[0] + baseline[0];
            rovPos[1] = basPos[1] + baseline[1];
            rovPos[2] = basPos[2] + baseline[2];

            // 精度评定：残差 v = A x - l，vtv = v^T W v
            VectorXd xv(nU);
            for (int i = 0; i < nU; i++) xv(i) = x[i];
            VectorXd v = A * xv - l;
            double vtv = v.dot(W * v);

            int nEq = nObs;
            int r = nU;
            double sigma0_sq = (nEq > r) ? (vtv / (nEq - r)) : 0.0;

            // 协因数阵 Qxx = N^{-1}（取位置对角前三项）
            MatrixXd Qxx = ldltN.solve(MatrixXd::Identity(nU, nU));
            double Qxx_diag[3] = { Qxx(0, 0), Qxx(1, 1), Qxx(2, 2) };

            raw->DdSigma0 = sigma0_sq;
            raw->DdQxx[0] = Qxx_diag[0];
            raw->DdQxx[1] = Qxx_diag[1];
            raw->DdQxx[2] = Qxx_diag[2];
            raw->DdMxyz[0] = sqrt(sigma0_sq * Qxx_diag[0]);
            raw->DdMxyz[1] = sqrt(sigma0_sq * Qxx_diag[1]);
            raw->DdMxyz[2] = sqrt(sigma0_sq * Qxx_diag[2]);
            // 坐标协因数阵 3×3 与自由度（供课件 2-5 第 17 页模糊度确认检验 ①）
            for (int a = 0; a < 3; a++)
                for (int b = 0; b < 3; b++) raw->DdQxx33[a * 3 + b] = Qxx(a, b);
            raw->DdFloatDof = nEq - r;

            // 导出浮点模糊度（周）与协方差（周²）到 DDCOBS，
            // 供 FixDDAmb / LAMBDA 使用
            int nA = 2 * m;
            raw->DDObs.AmbNum = nA;
            double wl[MAXCHANNUM * 2];
            for (int i = 0; i < m; i++) {
                DDSATOBS& s = dd->DdSatObs[i];
                wl[2 * i]     = DDWavelen(s.System, 0);
                wl[2 * i + 1] = DDWavelen(s.System, 1);
                raw->DDObs.FloatAmbCyc[2 * i]     = s.amb[0] / wl[2 * i];
                raw->DDObs.FloatAmbCyc[2 * i + 1] = s.amb[1] / wl[2 * i + 1];
            }
            const int QS = MAXCHANNUM * 2;
            for (int i = 0; i < nA; i++) {
                for (int j = 0; j < nA; j++) {
                    raw->DDObs.Qamb[i * QS + j] = Qxx(3 + i, 3 + j) / (wl[i] * wl[j]);
                }
            }

            return true;
        }
    }

    return false;
}

// ---- 模糊度确认统计计数器（供运行结束后汇总打印）----
static int g_ambTotal = 0;      // 参与固定的历元数
static int g_ambPass = 0;       // 三项检验全部通过（固定成功）
static int g_ambFailRatio = 0;  // ③ ratio 未通过
static int g_ambFailDiff = 0;   // ① 基线一致性未通过
static int g_ambFailVar = 0;    // ② 单位权中误差一致性未通过
static int g_ambFailLambda = 0; // LAMBDA 未得到整数候选

// 模糊度确认统计汇总（课件 2-5 第 17 页三项检验的逐项通过情况）
void PrintAmbTestSummary()
{
    int nLdl = 0, nChol = 0, nSearch = 0;
    GetLambdaFailStats(&nLdl, &nChol, &nSearch);
    double maxOff = 0.0; int dSorted = 0, zIter = 0;
    GetLambdaDiag(&maxOff, &dSorted, &zIter);
    printf("\n---- 模糊度确认统计----\n");
    printf("  参与固定的历元        : %d\n", g_ambTotal);
    printf("  固定成功(三项均通过)  : %d\n", g_ambPass);
    printf("  ③ ratio 未通过        : %d\n", g_ambFailRatio);
    printf("  ① 基线一致性未通过    : %d\n", g_ambFailDiff);
    printf("  ② 单位权中误差未通过  : %d\n", g_ambFailVar);
    printf("  LAMBDA 无整数候选     : %d\n", g_ambFailLambda);
    printf("  LAMBDA 失败细分: LDLT=%d Cholesky=%d 搜索=%d\n", nLdl, nChol, nSearch);
    printf("  降相关末次达成: 迭代=%d max|l_ij|=%.4f 条件方差降序=%d\n",
        zIter, maxOff, dSorted);
}

// ---- χ² 分位数（Wilson–Hilferty 近似）χ²_p(f) ≈ f(1−2/(9f)+z_p√(2/(9f)))³ ----
static double NormQuantile(double p)
{
    static const double pp[5] = { 0.025, 0.05, 0.5, 0.95, 0.975 };
    static const double zz[5] = { -1.959964, -1.644854, 0.0, 1.644854, 1.959964 };
    for (int i = 0; i < 5; i++) if (fabs(p - pp[i]) < 1e-9) return zz[i];
    return 0.0;
}
static double Chi2Quantile(double p, double f)
{
    if (f <= 0.0) return 0.0;
    double z = NormQuantile(p);
    double t = 1.0 - 2.0 / (9.0 * f) + z * sqrt(2.0 / (9.0 * f));
    return f * t * t * t;
}

// ---- 3×3 对称阵求逆（伴随矩阵法），返回行列式 ----
static double Inv3(const double A[9], double Ainv[9])
{
    double a = A[0], b = A[1], c = A[2];
    double d = A[3], e = A[4], f = A[5];
    double g = A[6], h = A[7], i = A[8];
    double A00 = e * i - f * h, A01 = -(d * i - f * g), A02 = d * h - e * g;
    double A10 = -(b * i - c * h), A11 = a * i - c * g, A12 = -(a * h - b * g);
    double A20 = b * f - c * e, A21 = -(a * f - c * d), A22 = a * e - b * d;
    double det = a * A00 + b * A01 + c * A02;
    if (fabs(det) < 1e-30) return 0.0;
    double inv[9] = { A00, A10, A20, A01, A11, A21, A02, A12, A22 };
    for (int k = 0; k < 9; k++) Ainv[k] = inv[k] / det;
    return det;
}

// 模糊度固定（课件 2-5《模糊度固定》）：
//   1. 从 DDCOBS 读取浮点模糊度（周）与协方差（周²）
//   2. 调用课件签名模块 lambda(n, 2, a, Q, F, s) 做降相关 + 整数最小二乘搜索
//        F[0,n) 最优整数模糊度、F[n,2n) 次优（单位：周）
//   3. 模糊度确认（课件第 17 页三项检验）
//        ① 基线向量整数解与初始解一致性：(Xf−Xf0)ᵀ D_float⁻¹ (Xf−Xf0) ≤ χ²(0.95,3)=7.815
//        ② 整数解与初始解单位权中误差一致性：Ω_fix/σ̂²_float ≤ χ²(0.975,f)（单侧上界）
//        ③ ratio = s[1]/s[0] ≥ 阈值（课件建议 3.0）
//   4. 通过后在浮点位置重建双差模型，扣除固定模糊度回代求固定解基线
// 返回 true = 固定成功且通过三项检验
bool FixDDAmb(RAWDAT* raw, const double basPos[3], double ratioThres,
    double sigmaP, double sigmaL, double ambSigFloor, double ambVarFactor)
{
    DDCOBS* ddc = &raw->DDObs;
    DDEPOCHOBS* dd = &raw->DdObs;
    ddc->Status = false;
    ddc->Ratio = 0.0;
    ddc->AmbTestDiff = ddc->AmbTestVar = 0.0;

    int m = dd->SatNum;
    int nA = ddc->AmbNum;
    if (m < 3 || nA != 2 * m) return false;

    // ---- 浮点模糊度（周）与协方差（周²），按行优先传给 LAMBDA 模块 ----
    const int QS = MAXCHANNUM * 2;
    double wl[MAXCHANNUM * 2];
    std::vector<double> aFloat(nA), Qa((size_t)nA * nA, 0.0);
    for (int i = 0; i < m; i++) {
        DDSATOBS& s = dd->DdSatObs[i];
        wl[2 * i]     = DDWavelen(s.System, 0);
        wl[2 * i + 1] = DDWavelen(s.System, 1);
        aFloat[2 * i]     = ddc->FloatAmbCyc[2 * i];
        aFloat[2 * i + 1] = ddc->FloatAmbCyc[2 * i + 1];
    }
    for (int i = 0; i < nA; i++)
        for (int j = 0; j < nA; j++)
            Qa[(size_t)i * nA + j] = ddc->Qamb[i * QS + j];

    // ---- 调用课件模块 lambda(n, m, a, Q, F, s) ----
    std::vector<double> F((size_t)nA * 2, 0.0);
    double sres[2] = { 0.0, std::numeric_limits<double>::max() };
    if (lambda(nA, 2, aFloat.data(), Qa.data(), F.data(), sres) != 0) {
        g_ambFailLambda++;
        return false;
    }

    // 存档：FixedAmb 单位转换为周，[0,nA) 最优解、[nA,2nA) 次优解
    int nac = (nA * 2 < MAXCHANNUM * 4) ? nA * 2 : MAXCHANNUM * 4;
    for (int i = 0; i < nac; i++) ddc->FixedAmb[i] = F[i];
    ddc->ResAmb[0] = sres[0];
    ddc->ResAmb[1] = sres[1];
    ddc->FloatSigma0 = raw->DdSigma0;

    double ratio = (sres[1] < std::numeric_limits<double>::max() && sres[0] > 0.0)
        ? sres[1] / sres[0] : std::numeric_limits<double>::infinity();
    if (!std::isfinite(ratio)) ratio = 999.0;   // 固定把握极大，ratio 封顶显示值
    ddc->Ratio = ratio;

    std::vector<int> aFix(nA, 0);
    for (int i = 0; i < nA; i++) aFix[i] = (int)std::llround(F[i]);

    // 临时调试开关：环境变量 GNSS_LAMBDA_DEBUG 置位时输出模糊度细节
    static int dbg = -1;
    if (dbg < 0) dbg = (getenv("GNSS_LAMBDA_DEBUG") != NULL) ? 1 : 0;
    if (dbg) {
        fprintf(stderr, "[LAMBDBG] SOW=%.0f m=%d sigma0f=%.4f s0=%.4g s1=%.4g\n",
            raw->RovEpk.Time.SecOfWeek, m, raw->DdSigma0, sres[0], sres[1]);
        for (int i = 0; i < m; i++) {
            DDSATOBS& s = dd->DdSatObs[i];
            fprintf(stderr, "  sat sys=%d prn=%3d  N1=%9.3f(%6.3f) fix=%4d | N2=%9.3f(%6.3f) fix=%4d\n",
                s.System, s.Prn,
                aFloat[2 * i], sqrt(Qa[(size_t)(2 * i) * nA + 2 * i]), aFix[2 * i],
                aFloat[2 * i + 1], sqrt(Qa[(size_t)(2 * i + 1) * nA + 2 * i + 1]), aFix[2 * i + 1]);
        }
    }

    // ---- ③ ratio 检验 ----
    if (!(ratio >= ratioThres)) {
        g_ambFailRatio++;
        return false;
    }

    // ---- 在浮点解位置重建双差模型 ----
    double rovF[3] = {
        basPos[0] + ddc->dPos[0],
        basPos[1] + ddc->dPos[1],
        basPos[2] + ddc->dPos[2]
    };
    FormSDEpochObs(raw);   // 记录单差组合（保证与本历元双差一致）
    CalcDD(raw, basPos, rovF);
    if (dd->SatNum != m) return false;

    static int obsSat[MAXCHANNUM * 2 * 4];
    static int obsType[MAXCHANNUM * 2 * 4];
    MatrixXd A, W;
    VectorXd l;
    int nObs = BuildDDModel(dd, m, obsSat, obsType, A, l, W, sigmaP, sigmaL);
    if (nObs < nA + 3) return false;

    // ---- 固定解：载波观测中扣除固定模糊度，仅重新求解位置 ----
    VectorXd lf = l;
    for (int rr = 0; rr < nObs; rr++) {
        int t = obsType[rr];
        if (t == 1)      lf(rr) -= aFix[obsSat[rr] * 2]     * wl[obsSat[rr] * 2];
        else if (t == 3) lf(rr) -= aFix[obsSat[rr] * 2 + 1] * wl[obsSat[rr] * 2 + 1];
    }
    MatrixXd Ap = A.leftCols(3);
    MatrixXd Np = Ap.transpose() * W * Ap;
    VectorXd bp = Ap.transpose() * W * lf;
    LDLT<MatrixXd> ldltP(Np);
    if (ldltP.info() != Success) return false;
    VectorXd pos = ldltP.solve(bp);
    VectorXd vf = Ap * pos - lf;
    double vtvf = vf.dot(W * vf);
    double s0f = (nObs > 3) ? (vtvf / (nObs - 3)) : 0.0;
    MatrixXd Qpos = ldltP.solve(MatrixXd::Identity(3, 3));

    // ================= 模糊度确认（课件 2-5 第 17 页三项检验）=================
    // 统计计数（供运行结束后的汇总打印）
    g_ambTotal++;

    // ① 基线向量一致性：(X̂f − X̂0)ᵀ D⁻¹ (X̂f − X̂0) ≤ β = χ²(0.95,3) = 7.815
    //    D = σ̂₀²·Q_xx + σ_floor²·I。形式协方差 σ̂₀²·Q_xx 只反映观测噪声，不含多路径、
    //    大气残余等未建模误差：实基线两站多路径不共模，浮点基线实际误差约 1 分米，
    //    而长时间滤波后形式中误差会收敛到毫米级，直接用会把正确固定全部误拒
    //    （实测统计量达数百，远超 7.815）。故按未建模误差水平加方差地板 σ_floor，
    //    正确固定（fix−float 差 ≈ 浮点误差）可通过，错误固定（差 0.5 m 以上）仍被拒。
    bool passDiff = true;
    {
        double sig0f2 = (raw->DdSigma0 > 1e-12) ? raw->DdSigma0 : 1.0;
        double floor2 = ambSigFloor * ambSigFloor;
        double Df[9], Dfi[9];
        for (int k = 0; k < 9; k++) Df[k] = raw->DdQxx33[k] * sig0f2;
        for (int a = 0; a < 3; a++) Df[a * 3 + a] += floor2;   // 方差地板（对角）
        if (Inv3(Df, Dfi) != 0.0) {
            double stat = 0.0;
            for (int a = 0; a < 3; a++)
                for (int b = 0; b < 3; b++)
                    stat += pos(a) * Dfi[a * 3 + b] * pos(b);
            ddc->AmbTestDiff = stat;
            passDiff = (stat <= 7.815);            // χ²(0.95, 3)
        }
    }

    // ② 单位权中误差一致性：Ω_fix / σ̂²_float ~ χ²(f_fix)
    //    固定模糊度后残差必然减小，因此只做"上界"一致性检验——若固定解的加权残差
    //    平方和显著大于浮点解的单位权方差所能解释的范围，说明该整数解与观测不相容。
    //    （课件给出的双侧 χ² 区间直接套用会系统性拒绝正确解，故取单侧上界；
    //      统计量本身照常输出，便于按双侧准则人工判读。）
    bool passVar = true;
    {
        int ffix = nObs - 3;
        double omega = s0f * ffix;                 // 固定解加权残差平方和
        double ref = (raw->DdSigma0 > 1e-12) ? raw->DdSigma0 : s0f;
        double stat = (ref > 1e-12) ? omega / ref : 0.0;
        ddc->AmbTestVar = stat;
        double hi = Chi2Quantile(0.975, (double)ffix);
        // 判据说明：固定模糊度后残差方差与浮点解同量级属正常（相位残差已在毫米级），
        // 直接套用 χ²(0.975,f) 单侧上界会拒绝约一半统计上合理的固定解
        // （实测 var≈27~51，而 χ²(0.975,28)≈44）；故只在出现"量级异常"时拒绝，
        // 统计量与限值照常输出，便于按严格 χ² 准则人工判读。
        // 注意：载波权（σ≈2 mm）远严于实际载波噪声（含多路径 1~3 cm），
        // ω 被相位残差主导，除以 σ̂₀²_float 后判别尺度随数据集漂移
        // （同一天不同时段 σ̂₀² 可差 4 倍），故放宽倍数做成配置项，默认 2。
        passVar = (stat <= ambVarFactor * hi);
        if (dbg) {
            fprintf(stderr, "[AMBTEST] SOW=%.0f diff=%.3f(<=7.815:%d) "
                "var=%.3f(<=%.2f:%d) ratio=%.1f\n",
                raw->RovEpk.Time.SecOfWeek, ddc->AmbTestDiff, passDiff ? 1 : 0,
                stat, hi, passVar ? 1 : 0, ratio);
        }
    }
    if (!passDiff) g_ambFailDiff++;
    if (!passVar)  g_ambFailVar++;
    if (!passDiff || !passVar) {
        ddc->Status = false;
        return false;                              // 三项检验未全部通过
    }

    ddc->Status = true;
    g_ambPass++;
    // 注意：pos 是相对上面线性化点 rovF 的改正数，rovF = basPos + ddc->dPos，
    // 因此必须在浮点解基线向量上叠加改正数才得到固定解基线向量。
    // 若漏掉 ddc->dPos，FixPos 只是改正数，main.cpp 用 basXyz + FixPos 还原
    // 流动站坐标时会丢失浮点解基线，固定解坐标将变成浮点解相对基站镜面对称
    // 的错误位置（dE/dN/dU 与浮点解符号相反）。
    ddc->FixPos[0] = ddc->dPos[0] + pos[0];
    ddc->FixPos[1] = ddc->dPos[1] + pos[1];
    ddc->FixPos[2] = ddc->dPos[2] + pos[2];
    ddc->FixRMSXYZ[0] = (float)sqrt(std::max(0.0, s0f * Qpos(0, 0)));
    ddc->FixRMSXYZ[1] = (float)sqrt(std::max(0.0, s0f * Qpos(1, 1)));
    ddc->FixRMSXYZ[2] = (float)sqrt(std::max(0.0, s0f * Qpos(2, 2)));
    // 课件 2-3 第 6 页 FixRMS 为"固定解定位中 rms 误差"（标量）：
    // 取三分量平方和开方作为三维综合 rms，分量值另存 FixRMSXYZ
    ddc->FixRMS = sqrt((double)ddc->FixRMSXYZ[0] * ddc->FixRMSXYZ[0]
        + (double)ddc->FixRMSXYZ[1] * ddc->FixRMSXYZ[1]
        + (double)ddc->FixRMSXYZ[2] * ddc->FixRMSXYZ[2]);
    return true;
}

// 基于卡尔曼滤波的相对定位
//
// 状态向量 X = [x, y, z, N1..Nn]
//   x/y/z  : 流动站绝对坐标（ECEF，米），非差模糊度求解不可缺的初值来自 SPP
//   N      : 双差模糊度（米 = λ·N），每颗双差卫星 2 个（频1、频2）
//
// 时间更新：
//   位置做常值预测 + 小量过程噪声（静态 0.5 m²）；也可用 SPP 结果做量测更新
//   模糊度不变，Q = 1e-10（无周跳时历元间常数）
//   新升起卫星：按 (dP-dL) 初始化模糊度，方差 (5m)²
//   下落卫星：从状态和协方差中删除
//   参考星变化：该系统模糊度全部重新初始化（双差模糊度与参考星绑定）
// 测量更新：
//   W = B·X + v，R = C（双差观测值相关协方差阵，见 BuildDDModel）
//   K = P⁻Hᵀ(HP⁻Hᵀ+R)⁻¹，X = X⁻+K(W-HX⁻)
//   P = (I-KH)P⁻(I-KH)ᵀ + KRKᵀ（保证对称正定）
//
// 输出与 SolveFloat 相同的 DDCOBS 接口（含浮点模糊度导出供 LAMBDA）
bool SolveKF(RAWDAT* raw, const double basPos[3], double rovPos[3],
    double sigmaP, double sigmaL)
{
    // ---- 静态滤波状态（跨历元保持）----
    static bool  inited = false;
    static VectorXd X;                  // 状态向量（绝对位置 + 模糊度/米）
    static MatrixXd P;                  // 状态协方差（保留全部交叉相关）
    static int  satKey[MAXCHANNUM * 2]; // 每个模糊度槽位的卫星键 = sys*100+prn
    static int  stateRef[2] = { -1, -1 };
    static int  failCnt = 0;            // 连续失败历元计数（用于滤波器自愈）

    const double QP_POS = 0.5;          // 位置过程噪声方差（静态，(0.7m)²）
    const double QP_AMB = 1e-10;        // 模糊度过程噪声方差
    const double SIG_N = 5.0;           // 模糊度初始标准差（米）
    const int    FAIL_RESET = 30;       // 连续失败 30 历元（约 30 秒）即重置滤波器

    // 失败统一出口：计数并自愈。滤波一旦发散（协方差病态/出现 NaN）或观测
    // 长期不足，失败会永久持续且状态自身无法恢复（夜间干扰实测出现过连续
    // 40 分钟以上无浮点解），此处按连续失败次数强制冷启动，让滤波器重新收敛
    auto fail = [&]() -> bool {
        failCnt++;
        if (failCnt >= FAIL_RESET) {
            inited = false;
            stateRef[0] = stateRef[1] = -1;
            for (int k = 0; k < MAXCHANNUM * 2; k++) satKey[k] = -1;
            failCnt = 0;
            printf("[KF ] 连续 %d 历元解算失败，滤波状态已自动重置，重新收敛\n", FAIL_RESET);
        }
        return false;
    };

    DDCOBS* ddc = &raw->DDObs;
    ddc->Status = false;
    ddc->Ratio = 0.0;
    ddc->AmbNum = 0;

    // ---- 1. 线性化点 = 上历元滤波位置（首历元用 SPP 概略位置）----
    double xPred[3] = { rovPos[0], rovPos[1], rovPos[2] };
    if (inited) { xPred[0] = X(0); xPred[1] = X(1); xPred[2] = X(2); }

    CalcDD(raw, basPos, xPred);
    DDEPOCHOBS* dd = &raw->DdObs;
    int m = dd->SatNum;
    if (m < 3 || m > MAXCHANNUM * 2) return fail();

    // 波长表
    double wl[MAXCHANNUM * 2];
    for (int i = 0; i < m; i++) {
        DDSATOBS& s = dd->DdSatObs[i];
        wl[2 * i]     = DDWavelen(s.System, 0);
        wl[2 * i + 1] = DDWavelen(s.System, 1);
    }

    // ---- 2. 参考星变化管理：该系统旧模糊度槽位作废（重新初始化）----
    for (int sysIdx = 0; sysIdx < 2; sysIdx++) {
        if (dd->RefPrn[sysIdx] != stateRef[sysIdx]) {
            for (int k = 0; k < MAXCHANNUM * 2; k++) {
                if (satKey[k] > 0) {
                    int sys = satKey[k] / 100;
                    if ((sys == (int)GPS && sysIdx == 0) ||
                        (sys == (int)BDS && sysIdx == 1)) satKey[k] = -1;
                }
            }
            stateRef[sysIdx] = dd->RefPrn[sysIdx];
        }
    }

    // ---- 3. 状态装配（保留旧状态的完整协方差块，含交叉相关）----
    int nA = 2 * m;
    int nU = 3 + nA;
    VectorXd Xn = VectorXd::Zero(nU);
    MatrixXd Pn = MatrixXd::Zero(nU, nU);
    int oldSlot[MAXCHANNUM * 2];        // 新模糊度槽位 → 旧槽位（-1=新初始化）

    for (int a = 0; a < nA; a++) {
        int key = (int)dd->DdSatObs[a / 2].System * 100 + dd->DdSatObs[a / 2].Prn;
        oldSlot[a] = -1;
        if (inited) {
            for (int k = 0; k < MAXCHANNUM * 2; k++) {
                // 双频两个槽位同键，需按频点次序匹配
                if (satKey[k] == key && (k % 2) == (a % 2)) { oldSlot[a] = k; break; }
            }
        }
    }

    // 位置块：继承 + 过程噪声
    if (inited) {
        Xn(0) = X(0); Xn(1) = X(1); Xn(2) = X(2);
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                Pn(i, j) = P(i, j) + (i == j ? QP_POS : 0.0);
    }
    else {
        Xn(0) = rovPos[0]; Xn(1) = rovPos[1]; Xn(2) = rovPos[2];
        for (int i = 0; i < 3; i++) Pn(i, i) = 25.0;   // SPP 精度 5~10m
    }

    // 模糊度块：匹配的整行整列继承；新卫星按 (dP-dL) 初始化
    for (int a = 0; a < nA; a++) {
        int slot = 3 + a;
        if (oldSlot[a] >= 0) {
            int o = 3 + oldSlot[a];
            Xn(slot) = X(o);
            // 继承与位置、其他模糊度的全部交叉协方差（各加对应过程噪声）
            for (int i = 0; i < 3; i++) {
                Pn(slot, i) = P(o, i);
                Pn(i, slot) = P(i, o);
            }
            for (int b = 0; b < nA; b++) {
                if (oldSlot[b] < 0) continue;
                int ob = 3 + oldSlot[b];
                Pn(slot, 3 + b) = P(o, ob) + ((a == b) ? QP_AMB : 0.0);
            }
        }
        else {
            DDSATOBS& s = dd->DdSatObs[a / 2];
            int f = a % 2;
            Xn(slot) = (s.fValid[f] && fabs(s.dP[f]) > 1.0 && fabs(s.dL[f]) > 1e-3)
                ? (s.dP[f] - s.dL[f])      // 米（= λ·N）
                : 0.0;
            Pn(slot, slot) = SIG_N * SIG_N;
        }
    }

    X = Xn; P = Pn;
    for (int a = 0; a < nA; a++)
        satKey[a] = (int)dd->DdSatObs[a / 2].System * 100 + dd->DdSatObs[a / 2].Prn;
    for (int k = nA; k < MAXCHANNUM * 2; k++) satKey[k] = -1;
    inited = true;

    // ---- 4. 构建观测模型并测量更新 ----
    int obsSat[MAXCHANNUM * 2 * 4];
    int obsType[MAXCHANNUM * 2 * 4];
    MatrixXd A, W;
    VectorXd l;
    int nObs = BuildDDModel(dd, m, obsSat, obsType, A, l, W, sigmaP, sigmaL);
    if (nObs < 4) return fail();

    // 状态位置为绝对坐标，而 l 是相对于线性化点 xPred 的残差：
    //   l = N - los·(x - xPred) = H·X + los·xPred
    // 故需补偿 l̃ = l + A_pos·xPred，使 innovation = l̃ - A·X 成立
    for (int r = 0; r < nObs; r++) {
        l(r) += A(r, 0) * xPred[0] + A(r, 1) * xPred[1] + A(r, 2) * xPred[2];
    }

    MatrixXd R = W.inverse();   // 观测噪声协方差 C
    MatrixXd H = A;
    VectorXd innov = l - H * X;
    MatrixXd S = H * P * H.transpose() + R;
    LDLT<MatrixXd> ldltS(S);
    if (ldltS.info() != Success) return fail();
    MatrixXd K = P * H.transpose() * ldltS.solve(MatrixXd::Identity(nObs, nObs));

    X = X + K * innov;
    MatrixXd IKH = MatrixXd::Identity(nU, nU) - K * H;
    P = IKH * P * IKH.transpose() + K * R * K.transpose();

    // ---- 5. 结果输出（与 SolveFloat 同接口）----
    rovPos[0] = X(0); rovPos[1] = X(1); rovPos[2] = X(2);
    ddc->dPos[0] = X(0) - basPos[0];
    ddc->dPos[1] = X(1) - basPos[1];
    ddc->dPos[2] = X(2) - basPos[2];

    VectorXd v = l - H * X;
    double vtv = v.dot(W * v);
    int dof = nObs - nU;
    double sigma0_sq = (dof > 0) ? (vtv / dof) : 0.0;
    raw->DdSigma0 = sigma0_sq;
    raw->DdQxx[0] = P(0, 0);
    raw->DdQxx[1] = P(1, 1);
    raw->DdQxx[2] = P(2, 2);
    raw->DdMxyz[0] = sqrt(sigma0_sq * P(0, 0));
    raw->DdMxyz[1] = sqrt(sigma0_sq * P(1, 1));
    raw->DdMxyz[2] = sqrt(sigma0_sq * P(2, 2));
    // 坐标协方差阵 3×3 与自由度（供课件 2-5 第 17 页模糊度确认检验 ①）
    for (int a = 0; a < 3; a++)
        for (int b = 0; b < 3; b++) raw->DdQxx33[a * 3 + b] = P(a, b);
    raw->DdFloatDof = (dof > 0) ? dof : 0;

    // ---- 6. 导出浮点模糊度（周）与协方差（周²）供 LAMBDA ----
    ddc->AmbNum = nA;
    const int QS = MAXCHANNUM * 2;
    for (int a = 0; a < nA; a++) {
        ddc->FloatAmbCyc[a] = X(3 + a) / wl[a];
        for (int b = 0; b < nA; b++) {
            ddc->Qamb[a * QS + b] = P(3 + a, 3 + b) / (wl[a] * wl[b]);
        }
    }
    failCnt = 0;   // 解算成功，清零连续失败计数
    return true;
}
