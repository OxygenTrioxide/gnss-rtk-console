#define _CRT_SECURE_NO_WARNINGS
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <climits>
#include <ctime>
#include <windows.h>
#include <io.h>
#include "RTK_Structs.h"
#include "oem7_decode.h"
#include "sockets.h"
#include "spp.h"
#include "spv.h"
#include "lambda.h"
#include "eph_orbit.h"
#include "gnss_coord.h"
#include "rtk_config.h"
#include "rtk_sd.h"
#include "serial.h"
#include <map>

static const int    MAX_EPOCH = INT_MAX;
// 数据同步阈值（课件 2-1 第 37 页）：取 0.5 倍采样间隔（1Hz → 0.5s）。
// 注意：该阈值是"两站历元配对"的时间容差，不是网络延迟容差——网络延迟由
// 同步循环内的缓冲读取吸收（读到两站 SOW 匹配为止）。若此处放宽到数秒，
// 一旦发生数据缺口，错位配对会被阈值"锁住"而长期保持：卫星在数秒内运动
// 可达数公里，双差观测将产生公里级误差，浮点解随之发散。
static const double SYNC_THRES_FILE = 0.5;
static const double SYNC_THRES_NET = 0.5;

// 出错/结束时的暂停：仅当 stdin 是交互式控制台才等待回车。
// 被 GUI 等父进程以管道方式调用时跳过——否则子进程结束前就挂住，
// 父进程会一直读到超时。
static void PauseIfConsole()
{
    // GetConsoleMode 才严格区分"控制台"与"重定向/NUL"——_isatty 对 NUL 设备
    // 也返回真，会把"按回车退出"带到 GUI/脚本日志里
    DWORD mode = 0;
    if (GetConsoleMode(GetStdHandle(STD_INPUT_HANDLE), &mode)) {
        printf("按回车退出...");
        getchar();
    }
}

// 输出 PVT 结果到屏幕
static void OutputPVT(const char* tag, const POSRES* res, const EPOCHOBSDATA* obs)
{
    XYZ xyz(res->Position[0], res->Position[1], res->Position[2]);
    BLH blh;
    XYZToBLH(xyz, blh);

    printf("[%s] Wk=%d SOW=%.3f  B=%.8f L=%.8f H=%8.3f  "
        "Vx=%7.3f Vy=%7.3f Vz=%7.3f  nSat=%d  PDOP=%5.2f  m=%.2f %.2f %.2f\n",
        tag, obs->Time.Week, obs->Time.SecOfWeek,
        blh.latitude * Deg, blh.longitude * Deg, blh.height,
        res->Velocity[0], res->Velocity[1], res->Velocity[2],
        res->AllSatNum, res->PDOP, res->mX, res->mY, res->mZ);
}

// 参考真值（.pos-lsq 第 3~5 列 = 商业 RTK 流动站坐标，与基站坐标同一来源）
// 及流动站坐标相对真值的站心 E/N/U 差值计算
struct TruthRec {
    int week; double sow; double b, l, h;   // b/l: 弧度, h: 米
};

// 键 = 周 × 1e8 + 周秒(毫秒)，按历元精确匹配
static long long TruthKey(int week, double sow)
{
    return (long long)week * 100000000LL + (long long)llround(sow * 1000.0);
}

static int LoadTruthFile(const char* fname, std::map<long long, TruthRec>& truth)
{
    FILE* fp = fopen(fname, "rt");
    if (!fp) return 0;
    char line[512];
    int n = 0;
    while (fgets(line, sizeof(line), fp)) {
        char* p = line;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '\0' || *p == '\n' || *p == '\r' || *p == '#' || *p == '%') continue;
        int wk; double sow, b, l, h;
        if (sscanf(p, "%d %lf %lf %lf %lf", &wk, &sow, &b, &l, &h) == 5) {
            TruthRec r; r.week = wk; r.sow = sow;
            r.b = b * Rad; r.l = l * Rad; r.h = h;
            truth[TruthKey(wk, sow)] = r;
            n++;
        }
    }
    fclose(fp);
    return n;
}

// BLHToNEUMat 行序：Row0=E, Row1=N, Row2=U；差值 = 解算坐标 − 真值坐标
static bool DiffVsTruth(const std::map<long long, TruthRec>& truth,
                        int week, double sow, const double xyz[3], double enu[3])
{
    auto it = truth.find(TruthKey(week, sow));
    if (it == truth.end()) return false;
    const TruthRec& t = it->second;
    BLH blh; blh.latitude = t.b; blh.longitude = t.l; blh.height = t.h;
    XYZ tx; BLHToXYZ(blh, tx);
    double R[9]; BLHToNEUMat(blh, R);
    double d[3] = { xyz[0] - tx.x, xyz[1] - tx.y, xyz[2] - tx.z };
    enu[0] = R[0] * d[0] + R[1] * d[1] + R[2] * d[2];   // E
    enu[1] = R[3] * d[0] + R[4] * d[1] + R[5] * d[2];   // N
    enu[2] = R[6] * d[0] + R[7] * d[1] + R[8] * d[2];   // U
    return true;
}

int main(int argc, char* argv[])
{
    // 离线自检入口：Nav.exe --lambdatest [文件]（默认为 lambda_input.txt）
    if (argc > 1 && _stricmp(argv[1], "--dumplambda") == 0) SetLambdaDump(1);
    if (argc > 1 && _stricmp(argv[1], "--lambdadebug") == 0) { SetLambdaDump(1); SetLambdaDebug(1); }
    if (argc > 1 && _stricmp(argv[1], "--lambdatest") == 0) {
        return LambdaSelfTestFile(argc > 2 ? argv[2] : "lambda_input.txt");
    }

    SetConsoleOutputCP(936);
    if (getenv("GNSS_SD_DEBUG")) setvbuf(stdout, nullptr, _IONBF, 0); // 诊断模式无缓冲
    printf("========================================\n");
    printf("  RTK Week1: Base + Rover SPP+SPV\n");
    printf("========================================\n");

    // 读配置
    ROVERCFGINFO cfg;
    if (!ReadRTKConfigInfo("rtk.ini", cfg)) {
        printf("Load rtk.ini failed\n");
        PauseIfConsole();
        return -1;
    }
    PrintRTKConfigInfo(cfg);

    // 数据源（课件 1-3 第 4 页 / 2-1 第 15 页：文件 / 串口 / 网络 三种方式）
    //   DATA SOURCE MODE：0=文件 1=网络 2=串口；未设置时按课件
    //   FROM FILE OR COM（1=FILE, 0=COM）推断
    int dataSrc = (cfg.DataSrcMode >= 0) ? cfg.DataSrcMode : (cfg.IsFileData == 1 ? 0 : 2);
    const bool useFile = (dataSrc == 0);
    const bool useNet = (dataSrc == 1);
    const bool useCom = (dataSrc == 2);
    const char* srcName = useFile ? "FILE" : (useNet ? "NETWORK" : "COM");

    FILE* fpBase = nullptr;
    FILE* fpRover = nullptr;
    SOCKET sBase = INVALID_SOCKET;
    SOCKET sRov = INVALID_SOCKET;
    CSerial comBase, comRov;

    printf("Data source mode: %d (%s)\n", dataSrc, srcName);

    if (useFile) {
        fpBase = fopen(cfg.BasObsDatFile, "rb");
        fpRover = fopen(cfg.RovObsDatFile, "rb");
        if (!fpBase) { printf("Cannot open BASE file\n");  PauseIfConsole(); return -1; }
        if (!fpRover) { printf("Cannot open ROVER file\n"); PauseIfConsole(); return -1; }
        printf("Base  file: %s\n", cfg.BasObsDatFile);
        printf("Rover file: %s\n", cfg.RovObsDatFile);
    }
    else if (useNet) {
        if (!OpenSocket(sBase, cfg.BasNetIP, (unsigned short)cfg.BasNetPort)) {
            printf("Open BASE socket %s:%d failed\n", cfg.BasNetIP, cfg.BasNetPort);
            PauseIfConsole();
            return -1;
        }
        if (!OpenSocket(sRov, cfg.RovNetIP, (unsigned short)cfg.RovNetPort)) {
            printf("Open ROVER socket %s:%d failed\n", cfg.RovNetIP, cfg.RovNetPort);
            CloseSocket(sBase);
            PauseIfConsole();
            return -1;
        }
        printf("Base  socket: %s:%d\n", cfg.BasNetIP, cfg.BasNetPort);
        printf("Rover socket: %s:%d\n", cfg.RovNetIP, cfg.RovNetPort);
    }
    else {
        // 串口模式（课件 1-5 第 13/14 页 CSerial；第 4 页"打开串口→设置串口→写入
        // Log 命令→读取数据"）
        if (!comBase.Open(cfg.BasPort, cfg.BasBaud)) {
            printf("Open BASE COM%d failed\n", cfg.BasPort);
            PauseIfConsole();
            return -1;
        }
        if (!comRov.Open(cfg.RovPort, cfg.RovBaud)) {
            printf("Open ROVER COM%d failed\n", cfg.RovPort);
            comBase.Close();
            PauseIfConsole();
            return -1;
        }
        // 写入 Log 配置命令（可选步骤，课件 1-5 第 22 页）
        SendLogCommands(&comBase);
        SendLogCommands(&comRov);
    }

    // 观测噪声与高度角截止角（配置为 0 时用默认值）
    const double sigmaP = (cfg.CodeNoise > 0.0) ? cfg.CodeNoise : 0.3;
    const double sigmaL = (cfg.CPNoise > 0.0) ? cfg.CPNoise : 0.05;
    const double elevMask = (cfg.ElevThreshold > 0.0) ? cfg.ElevThreshold * Rad : 0.0;

    // 粗差探测阈值（课件 1-7 第 11 页，配置项 OUTLIER THRESHOLD）
    SetOutlierThreshold(cfg.OutGfThres, cfg.OutMwThres);

    // 基站坐标优先级（课件 2-1 第 15 页：基站数据含 bestpos）：
    //   ① 配置文件 BASE POSITION（ECEF）
    //   ② 基站数据流中的 PSRPOS/BESTPOS（在主循环中一旦解出即采用）
    //   ③ 参考文件 .pos-lsq 首行（兼容旧配置，仅零基线场景成立）
    BLH basBlh;
    XYZ basXyz;
    bool basPosFromCfg = (cfg.BasPosGiven != 0);
    if (basPosFromCfg) {
        basXyz.x = cfg.BasPosXYZ[0];
        basXyz.y = cfg.BasPosXYZ[1];
        basXyz.z = cfg.BasPosXYZ[2];
        XYZToBLH(basXyz, basBlh);
        printf("[BASEPOS] 使用配置文件基站坐标\n");
    }
    else if (ReadBasePosFromPos(cfg.ResFile, basBlh) != 0) {
        printf("Read base position failed, using fallback\n");
        basBlh.latitude = 30.52844163 * Rad;
        basBlh.longitude = 114.35697689 * Rad;
        basBlh.height = 26.816;
        BLHToXYZ(basBlh, basXyz);
    }
    else {
        BLHToXYZ(basBlh, basXyz);
    }
    printf("Base XYZ: %.3f %.3f %.3f\n", basXyz.x, basXyz.y, basXyz.z);

    // 加载参考真值（同一 .pos-lsq 文件逐历元，用于 E/N/U 差值列）
    std::map<long long, TruthRec> truthMap;
    const int nTruth = LoadTruthFile(cfg.ResFile, truthMap);
    printf("Truth reference: %s (%d epochs)\n", cfg.ResFile, nTruth);

    // 初始化 RAWDAT（static：DDCOBS 含模糊度协方差，体积较大）
    static RAWDAT raw;
    for (int i = 0; i < MAXGPSNUM; i++) { raw.GpsEph[i].PRN = i + 1; raw.GpsEph[i].Sys = GPS; raw.GpsEph[i].Valid = false; }
    for (int i = 0; i < MAXBDSNUM; i++) { raw.BdsEph[i].PRN = i + 1; raw.BdsEph[i].Sys = BDS; raw.BdsEph[i].Valid = false; }

    double basPos[3] = { basXyz.x, basXyz.y, basXyz.z };
    double rovPos[3] = { basXyz.x, basXyz.y, basXyz.z };
    bool basPosFromStream = false;   // 基站坐标是否已取自数据流 PSRPOS/BESTPOS
    bool rtkStarted = false;         // 相对定位是否已开始（用于锁定基站坐标）

    // 拼输出文件路径（OUTPUT DIRECTORY 为空则用当前目录）
    char sppOutFile[512] = { 0 };
    char floatOutFile[512] = { 0 };
    char fixedOutFile[512] = { 0 };

    const char* outDir = (strlen(cfg.OutputDir) > 0) ? cfg.OutputDir : "";
    char dirTmp[256] = { 0 };
    strncpy(dirTmp, outDir, sizeof(dirTmp) - 1);
    int L = (int)strlen(dirTmp);
    if (L > 0 && dirTmp[L - 1] != '/' && dirTmp[L - 1] != '\\') {
        dirTmp[L] = '/';
        dirTmp[L + 1] = '\0';
    }

    // 结果文件格式（配置 RESULT FILE FORMAT：0=txt 默认, 1=csv）
    const bool outCsv = (cfg.OutFormat == 1);
    const char* d = outCsv ? "," : "  ";   // 列分隔符
    const char* ext = outCsv ? "csv" : "txt";

    snprintf(sppOutFile, sizeof(sppOutFile), "%sspp_result.%s", dirTmp, ext);
    snprintf(floatOutFile, sizeof(floatOutFile), "%sfloat_result.%s", dirTmp, ext);
    snprintf(fixedOutFile, sizeof(fixedOutFile), "%sfixed_result.%s", dirTmp, ext);

    printf("SPP   output file: %s\n", sppOutFile);
    printf("Float output file: %s\n", floatOutFile);
    printf("Fixed output file: %s\n", fixedOutFile);

    // 打开输出文件（按配置开关）
    FILE* fpSpp = nullptr;
    FILE* fpFloat = nullptr;
    FILE* fpFixed = nullptr;

    if (cfg.EnableSppOutput) {
        fpSpp = fopen(sppOutFile, "w");
        if (fpSpp) {
            if (outCsv) {
                fprintf(fpSpp, "Week,SOW,Type,X_m,Y_m,Z_m,B_deg,L_deg,H_m,Vx_mps,Vy_mps,Vz_mps,nSat\n");
            }
            else {
                fprintf(fpSpp, "# GNSS SPP Result File\n");
                fprintf(fpSpp, "# Wk  SOW  Type  X  Y  Z  B  L  H  Vx  Vy  Vz  nSat\n");
                fprintf(fpSpp, "# Type: BASE / ROVER\n");
                fprintf(fpSpp, "# ----------------------------------------------------------------\n");
            }
            fflush(fpSpp);
            printf("SPP result output ENABLED\n");
        }
        else {
            printf("WARNING: Cannot open %s\n", sppOutFile);
        }
    }
    else {
        printf("SPP result output DISABLED (config)\n");
    }

    if (cfg.EnableFloatOutput) {
        fpFloat = fopen(floatOutFile, "w");
        if (fpFloat) {
            if (outCsv) {
                fprintf(fpFloat, "Week,SOW,X_m,Y_m,Z_m,dX_m,dY_m,dZ_m,Abs_dPos_m,"
                    "B_deg,L_deg,H_m,RefGps,RefBds,NGPS,NBDS,DD_SatNum,"
                    "Sigma0,mX_m,mY_m,mZ_m,dE_m,dN_m,dU_m\n");
            }
            else {
                fprintf(fpFloat, "# GNSS Float Solution Result File\n");
                fprintf(fpFloat, "# Wk  SOW  X  Y  Z  dX  dY  dZ  |dPos|  B  L  H  RefGps  RefBds  NGPS  NBDS  m  Sigma0  mX  mY  mZ  dE  dN  dU\n");
                fprintf(fpFloat, "# dPos = Rover - Base (ECEF, m); dE/dN/dU = rover - truth (m)\n");
                fprintf(fpFloat, "# ----------------------------------------------------------------\n");
            }
            fflush(fpFloat);
            printf("Float result output ENABLED\n");
        }
        else {
            printf("WARNING: Cannot open %s\n", floatOutFile);
        }
    }
    else {
        printf("Float result output DISABLED (config)\n");
    }

    if (cfg.EnableFixedOutput) {
        fpFixed = fopen(fixedOutFile, "w");
        if (fpFixed) {
            if (outCsv) {
                fprintf(fpFixed, "Week,SOW,X_m,Y_m,Z_m,dX_m,dY_m,dZ_m,Abs_dPos_m,"
                    "B_deg,L_deg,H_m,Ratio,RMSx_m,RMSy_m,RMSz_m,dE_m,dN_m,dU_m\n");
            }
            else {
                fprintf(fpFixed, "# GNSS Fixed Solution Result File\n");
                fprintf(fpFixed, "# Wk  SOW  X  Y  Z  dX  dY  dZ  |dPos|  B  L  H  ratio  RMSx  RMSy  RMSz  dE  dN  dU\n");
                fprintf(fpFixed, "# dPos = Rover - Base (ECEF, m); dE/dN/dU = rover - truth (m)\n");
                fprintf(fpFixed, "# ----------------------------------------------------------------\n");
            }
            fflush(fpFixed);
            printf("Fixed result output ENABLED\n");
        }
        else {
            printf("WARNING: Cannot open %s\n", fixedOutFile);
        }
    }
    else {
        printf("Fixed result output DISABLED (config)\n");
    }

    // ---- B-5 原始数据流落盘（仅网络模式，配置 RECORD RAW DATA: 1 开启）----
    // 基站/流动站收到的原始字节同步写入运行目录，事后可回放分析干扰
    FILE* fpRecBase = nullptr;
    FILE* fpRecRov = nullptr;
    if (cfg.DataSrcMode == 1 && cfg.RecordRaw) {
        char recPath[512];
        snprintf(recPath, sizeof(recPath), "%sbase_record.bin", dirTmp);
        fpRecBase = fopen(recPath, "wb");
        snprintf(recPath, sizeof(recPath), "%srover_record.bin", dirTmp);
        fpRecRov = fopen(recPath, "wb");
        SetRawRecordFiles(fpRecBase, fpRecRov);
        printf("Raw recording: %s / %s (%s)\n", fpRecBase ? "base OK" : "base FAIL",
            fpRecRov ? "rover OK" : "rover FAIL", dirTmp);
    }

    // ---- B-6 解状态流文件：每历元一行，供外部程序直接画曲线 ----
    char statusOutFile[512] = { 0 };
    snprintf(statusOutFile, sizeof(statusOutFile), "%ssolve_status.csv", dirTmp);
    FILE* fpStatus = fopen(statusOutFile, "w");
    if (fpStatus) {
        fprintf(fpStatus, "Week,SOW,State,BaseLen_m,Ratio,Sigma0,NGPS,NBDS\n");
        fflush(fpStatus);
        printf("Status stream file: %s\n", statusOutFile);
    }

    // 主循环
    int nEpoch = 0, nBaseOK = 0, nRoverOK = 0;
    const time_t runStartTime = time(nullptr);   // 解算时长限制（MAX RUNNING MINUTES）计时起点

    // E/N/U 差值统计累计量（float / fixed 分别统计，单位米）
    double flE = 0, flN = 0, flU = 0, flE2 = 0, flN2 = 0, flU2 = 0;
    double flMaxE = 0, flMaxN = 0, flMaxU = 0; long nFlDiff = 0;
    double fxE = 0, fxN = 0, fxU = 0, fxE2 = 0, fxN2 = 0, fxU2 = 0;
    double fxMaxE = 0, fxMaxN = 0, fxMaxU = 0; long nFxDiff = 0;

    while (nEpoch < MAX_EPOCH) {
        // 三种数据源共用同名函数 GetSynGNSSObs（重载，课件 2-1 第 36/37 页）
        int ret;
        if (useFile)     ret = GetSynGNSSObs(fpBase, fpRover, &raw, SYNC_THRES_FILE);
        else if (useNet) ret = GetSynGNSSObs(sBase, sRov, &raw, SYNC_THRES_NET);
        else             ret = GetSynGNSSObs(&comBase, &comRov, &raw, SYNC_THRES_NET);
        if (ret < 0) {
            // 网络模式：断线自动重连。每 30 秒重试一次，累计不超过
            // NET RECONNECT MINUTES 分钟（0 = 不重连，直接结束）。
            // 任一端断开都两端同时重连（同步循环需要两路都存活）；
            // 重连成功后清空同步缓冲，解码器从 0xAA4412 同步头重新对帧。
            bool timeLimitHit = false;
            bool reconnected = false;
            if (useNet && cfg.NetReconnectMinutes > 0.0) {
                printf("\n[NET] 数据流中断，进入自动重连（预算 %.0f 分钟）...\n",
                    cfg.NetReconnectMinutes);
                const time_t retryStart = time(nullptr);
                while (true) {
                    double retryMin = difftime(time(nullptr), retryStart) / 60.0;
                    if (retryMin >= cfg.NetReconnectMinutes) {
                        printf("[NET] 重连预算耗尽，结束解算。\n");
                        break;
                    }
                    // 解算时长到点：与正常历元路径一致地收尾
                    if (cfg.MaxRunMinutes > 0.0) {
                        double elapsedMin = (double)(time(nullptr) - runStartTime) / 60.0;
                        if (elapsedMin >= cfg.MaxRunMinutes) {
                            timeLimitHit = true;
                            break;
                        }
                    }
                    Sleep(30000);
                    if (sBase != INVALID_SOCKET) { CloseSocket(sBase); sBase = INVALID_SOCKET; }
                    if (sRov  != INVALID_SOCKET) { CloseSocket(sRov);  sRov  = INVALID_SOCKET; }
                    bool okB = OpenSocket(sBase, cfg.BasNetIP, (unsigned short)cfg.BasNetPort);
                    bool okR = okB && OpenSocket(sRov, cfg.RovNetIP, (unsigned short)cfg.RovNetPort);
                    if (okR) { reconnected = true; break; }
                    if (okB) { CloseSocket(sBase); sBase = INVALID_SOCKET; }
                    printf("[NET] 重连失败，30 秒后重试（已用 %.0f/%.0f 分钟）...\n",
                        difftime(time(nullptr), retryStart) / 60.0, cfg.NetReconnectMinutes);
                }
            }
            if (reconnected) {
                printf("[NET] 重连成功，继续解算（累计 %d 历元）。\n", nEpoch);
                ResetNetSyncBuffers();
                continue;
            }
            if (timeLimitHit) {
                printf("\n已达设定解算时长 %.1f 分钟，正常收尾（共 %d 历元）。\n",
                    cfg.MaxRunMinutes, nEpoch);
                break;
            }
            printf("End of data\n");
            break;
        }
        if (ret == 0) { printf("Sync failed\n");   break; }
        nEpoch++;

        // 最长解算时长（分钟）到点：正常收尾——走 break 打印统计摘要并以
        // 退出码 0 结束，GUI 据此判定"成功"。（数据接收阻塞期间不计时精度
        // 为秒级，足够分钟级任务使用。）
        if (cfg.MaxRunMinutes > 0.0) {
            double elapsedMin = (double)(time(nullptr) - runStartTime) / 60.0;
            if (elapsedMin >= cfg.MaxRunMinutes) {
                printf("\n已达设定解算时长 %.1f 分钟，正常收尾（共 %d 历元）。\n",
                    cfg.MaxRunMinutes, nEpoch);
                break;
            }
        }

        printf("\n--- Epoch %d ---\n", nEpoch);
        printf("[SYNC] Base Wk=%d SOW=%.3f (nSat=%d)  |  Rover Wk=%d SOW=%.3f (nSat=%d)\n",
            raw.BasEpk.Time.Week, raw.BasEpk.Time.SecOfWeek, raw.BasEpk.SatNum,
            raw.RovEpk.Time.Week, raw.RovEpk.Time.SecOfWeek, raw.RovEpk.SatNum);

        // 基站坐标：配置未指定时，优先采用基站数据流中解出的 PSRPOS/BESTPOS
        // 坐标（课件 2-1 第 15 页）。注意不能用"尚未开始相对定位"作条件——一旦
        // 首历元即解出浮点解，数据流坐标将永远不被采用，残留的 .pos-lsq 首行
        // 旧坐标会一直错下去（基线向量不受影响，但流动站绝对坐标与 E/N/U
        // 真值对比全部错误）。basPosFromStream 锁存保证只采用一次。
        if (!basPosFromCfg && raw.BasPos.IsSuccess && !basPosFromStream) {
            basPosFromStream = true;
            basXyz = XYZ(raw.BasPos.Position[0], raw.BasPos.Position[1],
                raw.BasPos.Position[2]);
            XYZToBLH(basXyz, basBlh);
            basPos[0] = basXyz.x; basPos[1] = basXyz.y; basPos[2] = basXyz.z;
            printf("[BASEPOS] 采用基站数据流 PSRPOS/BESTPOS 坐标: "
                "%.3f %.3f %.3f (nSat=%d)\n",
                basXyz.x, basXyz.y, basXyz.z, raw.BasPos.AllSatNum);
        }

        // 粗差探测（DetectOutlier，SPP 之前的观测值预处理）
        DetectOutlier(&raw.BasEpk);
        DetectOutlier(&raw.RovEpk);

        // 基站 SPP + SPV
        raw.BasEpk.Pos[0] = basPos[0];
        raw.BasEpk.Pos[1] = basPos[1];
        raw.BasEpk.Pos[2] = basPos[2];

        POSRES basRes;
        bool basOK = SPP(&raw.BasEpk, &raw, &basRes, elevMask, cfg.TropModel);
        if (basOK) {
            SPV(&raw.BasEpk, &raw, &basRes);
            OutputPVT("BASE ", &basRes, &raw.BasEpk);
            nBaseOK++;
        }
        else {
            printf("[BASE ] SPP FAILED\n");
        }

        // 流动站 SPP + SPV
        raw.RovEpk.Pos[0] = rovPos[0];
        raw.RovEpk.Pos[1] = rovPos[1];
        raw.RovEpk.Pos[2] = rovPos[2];

        POSRES rovRes;
        bool rovOK = SPP(&raw.RovEpk, &raw, &rovRes, elevMask, cfg.TropModel);
        if (rovOK) {
            SPV(&raw.RovEpk, &raw, &rovRes);
            OutputPVT("ROVER", &rovRes, &raw.RovEpk);
            nRoverOK++;

            rovPos[0] = rovRes.Position[0];
            rovPos[1] = rovRes.Position[1];
            rovPos[2] = rovRes.Position[2];
        }
        else {
            printf("[ROVER] SPP FAILED\n");
        }

        // 写 SPP 结果文件（BASE + ROVER 各一行）
        if (fpSpp) {
            if (basOK) {
                XYZ xyz(basRes.Position[0], basRes.Position[1], basRes.Position[2]);
                BLH blh; XYZToBLH(xyz, blh);
                fprintf(fpSpp, "%d%s%.3f%sBASE%s%.3f%s%.3f%s%.3f%s%.8f%s%.8f%s%.3f%s%.3f%s%.3f%s%.3f%s%d\n",
                    raw.BasEpk.Time.Week, d, raw.BasEpk.Time.SecOfWeek, d,
                    d,
                    basRes.Position[0], d, basRes.Position[1], d, basRes.Position[2], d,
                    blh.latitude * Deg, d, blh.longitude * Deg, d, blh.height, d,
                    basRes.Velocity[0], d, basRes.Velocity[1], d, basRes.Velocity[2], d,
                    basRes.AllSatNum);
            }
            if (rovOK) {
                XYZ xyz(rovRes.Position[0], rovRes.Position[1], rovRes.Position[2]);
                BLH blh; XYZToBLH(xyz, blh);
                fprintf(fpSpp, "%d%s%.3f%sROVER%s%.3f%s%.3f%s%.3f%s%.8f%s%.8f%s%.3f%s%.3f%s%.3f%s%.3f%s%d\n",
                    raw.RovEpk.Time.Week, d, raw.RovEpk.Time.SecOfWeek, d,
                    d,
                    rovRes.Position[0], d, rovRes.Position[1], d, rovRes.Position[2], d,
                    blh.latitude * Deg, d, blh.longitude * Deg, d, blh.height, d,
                    rovRes.Velocity[0], d, rovRes.Velocity[1], d, rovRes.Velocity[2], d,
                    rovRes.AllSatNum);
            }
            fflush(fpSpp);
        }

        // 站间单差、周跳探测
        if (getenv("GNSS_SD_DEBUG")) {
            auto cntValid = [](EPOCHOBSDATA* e) {
                int n = 0; for (int k = 0; k < e->SatNum; k++) if (e->SatObs[k].Valid) n++;
                return n; };
            printf("    [SDDBG] SPP后 bas valid=%d/%d rov valid=%d/%d\n",
                cntValid(&raw.BasEpk), raw.BasEpk.SatNum,
                cntValid(&raw.RovEpk), raw.RovEpk.SatNum);
        }
        CheckLockTimeParity(&raw);
        if (getenv("GNSS_SD_DEBUG")) {
            int nb = 0, nr = 0;
            for (int k = 0; k < raw.BasEpk.SatNum; k++) if (raw.BasEpk.SatObs[k].Valid) nb++;
            for (int k = 0; k < raw.RovEpk.SatNum; k++) if (raw.RovEpk.SatObs[k].Valid) nr++;
            printf("    [SDDBG] 质量门控后 bas valid=%d rov valid=%d\n", nb, nr);
        }
        DetectCycleSlip(&raw, cfg.CsMwThres, cfg.CsGfThres);
        if (getenv("GNSS_SD_DEBUG")) {
            int nb = 0, nr = 0;
            for (int k = 0; k < raw.BasEpk.SatNum; k++) if (raw.BasEpk.SatObs[k].Valid) nb++;
            for (int k = 0; k < raw.RovEpk.SatNum; k++) if (raw.RovEpk.SatObs[k].Valid) nr++;
            printf("    [SDDBG] 周跳探测后 bas valid=%d rov valid=%d\n", nb, nr);
        }
        FormSDEpochObs(&raw);
        printf("[SD] SD SatNum = %d\n", raw.SdObs.SatNum);

        // 基准星选取
        DetRefSat(&raw, 35.0, 30.0, 6.0);
        printf("[REF] GPS RefSat = %s%02d (idx=%d)   BDS RefSat = %s%02d (idx=%d)\n",
            (raw.DDObs.RefPrn[0] >= 0 ? "G" : "-"),
            (raw.DDObs.RefPrn[0] >= 0 ? raw.DDObs.RefPrn[0] : 0),
            raw.DDObs.RefPos[0],
            (raw.DDObs.RefPrn[1] >= 0 ? "C" : "-"),
            (raw.DDObs.RefPrn[1] >= 0 ? raw.DDObs.RefPrn[1] : 0),
            raw.DDObs.RefPos[1]);

        // 双差 + 浮点解
        // 容错（课件 2-4 第 17 页）：流动站 SPP 失败则不做相对定位
        // epState/epBaseLen/epRatio：本历元解状态（写入 solve_status.csv）
        int epState = 0;
        double epBaseLen = 0.0, epRatio = 0.0;
        if (!rovOK) {
            printf("[RTK ] Rover SPP failed, skip relative positioning\n");
        }
        else if (raw.DDObs.RefPrn[0] > 0 || raw.DDObs.RefPrn[1] > 0) {
            double basPos2[3] = { basXyz.x, basXyz.y, basXyz.z };
            double rovPos2[3] = { rovRes.Position[0], rovRes.Position[1], rovRes.Position[2] };

            // 课件 2-1 第 24 页 ROVERCFGINFO 注释：RTKProcMode 1 = EKF，2 = LSQ
            const bool useKF = (cfg.RTKProcMode == 1);
            bool floatOK = useKF
                ? SolveKF(&raw, basPos2, rovPos2, sigmaP, sigmaL)
                : SolveFloat(&raw, basPos2, rovPos2, sigmaP, sigmaL);

            if (floatOK) {
                epState = 1;   // 浮点解
                rtkStarted = true;   // 相对定位已开始，此后不再更换基站坐标
                XYZ rovFix(rovPos2[0], rovPos2[1], rovPos2[2]);
                BLH rovBlhFix;
                XYZToBLH(rovFix, rovBlhFix);

                double dX = raw.DDObs.dPos[0];
                double dY = raw.DDObs.dPos[1];
                double dZ = raw.DDObs.dPos[2];
                double lenB = sqrt(dX * dX + dY * dY + dZ * dZ);
                epBaseLen = lenB;

                printf("[FLOAT] B=%.9f L=%.9f H=%.3f  dPos=(%.3f, %.3f, %.3f)  |b|=%.3f m\n",
                    rovBlhFix.latitude * Deg, rovBlhFix.longitude * Deg, rovBlhFix.height,
                    dX, dY, dZ, lenB);
                printf("        Sigma0=%.4f  mX=%.4f mY=%.4f mZ=%.4f\n",
                    raw.DdSigma0,
                    raw.DdMxyz[0], raw.DdMxyz[1], raw.DdMxyz[2]);

                // 写浮点解结果（含 rover XYZ 及相对真值的 E/N/U 差值）
                if (fpFloat) {
                    const char* refG = (raw.DDObs.RefPrn[0] > 0) ? "G" : "-";
                    const char* refC = (raw.DDObs.RefPrn[1] > 0) ? "C" : "-";
                    double enu[3];
                    bool hasDiff = DiffVsTruth(truthMap,
                        raw.BasEpk.Time.Week, raw.BasEpk.Time.SecOfWeek, rovPos2, enu);
                    fprintf(fpFloat, "%d%s%.3f%s%.3f%s%.3f%s%.3f%s%.3f%s%.3f%s%.3f%s%.4f%s"
                        "%.9f%s%.9f%s%.3f%s%s%02d%s%s%02d%s%d%s%d%s%d%s"
                        "%.4f%s%.4f%s%.4f%s%.4f",
                        raw.BasEpk.Time.Week, d, raw.BasEpk.Time.SecOfWeek, d,
                        rovPos2[0], d, rovPos2[1], d, rovPos2[2], d,
                        dX, d, dY, d, dZ, d, lenB, d,
                        rovBlhFix.latitude* Deg, d, rovBlhFix.longitude* Deg, d, rovBlhFix.height, d,
                        refG, (raw.DDObs.RefPrn[0] > 0 ? raw.DDObs.RefPrn[0] : 0), d,
                        refC, (raw.DDObs.RefPrn[1] > 0 ? raw.DDObs.RefPrn[1] : 0), d,
                        raw.DdObs.NGPS, d, raw.DdObs.NBDS, d, raw.DdObs.SatNum, d,
                        raw.DdSigma0, d, raw.DdMxyz[0], d, raw.DdMxyz[1], d, raw.DdMxyz[2]);
                    if (hasDiff) {
                        fprintf(fpFloat, ",%.4f,%.4f,%.4f\n", enu[0], enu[1], enu[2]);
                        flE += enu[0]; flN += enu[1]; flU += enu[2];
                        flE2 += enu[0] * enu[0]; flN2 += enu[1] * enu[1]; flU2 += enu[2] * enu[2];
                        if (fabs(enu[0]) > flMaxE) flMaxE = fabs(enu[0]);
                        if (fabs(enu[1]) > flMaxN) flMaxN = fabs(enu[1]);
                        if (fabs(enu[2]) > flMaxU) flMaxU = fabs(enu[2]);
                        nFlDiff++;
                    }
                    else {
                        fprintf(fpFloat, "%s%s\n", d, d);
                    }
                    fflush(fpFloat);
                }

                // LAMBDA 模糊度固定（失败不影响浮点解输出）
                if (cfg.EnableFixedOutput && FixDDAmb(&raw, basPos2, cfg.RatioThres, sigmaP, sigmaL, cfg.AmbTestSigmaFloor, cfg.AmbTestVarFactor)) {
                    epState = 2;   // 本历元固定解
                    epRatio = raw.DDObs.Ratio;
                    XYZ rovFixXyz(
                        basXyz.x + raw.DDObs.FixPos[0],
                        basXyz.y + raw.DDObs.FixPos[1],
                        basXyz.z + raw.DDObs.FixPos[2]);
                    BLH fixBlh;
                    XYZToBLH(rovFixXyz, fixBlh);

                    double fx = raw.DDObs.FixPos[0];
                    double fy = raw.DDObs.FixPos[1];
                    double fz = raw.DDObs.FixPos[2];
                    double flen = sqrt(fx * fx + fy * fy + fz * fz);

                    printf("[FIXED] B=%.9f L=%.9f H=%.3f  dPos=(%.3f, %.3f, %.3f)  "
                        "|b|=%.3f m  ratio=%.2f  RMS=%.4f %.4f %.4f  检验: diff=%.2f var=%.2f\n",
                        fixBlh.latitude * Deg, fixBlh.longitude * Deg, fixBlh.height,
                        fx, fy, fz, flen,
                        raw.DDObs.Ratio,
                        raw.DDObs.FixRMSXYZ[0], raw.DDObs.FixRMSXYZ[1], raw.DDObs.FixRMSXYZ[2],
                        raw.DDObs.AmbTestDiff, raw.DDObs.AmbTestVar);

                    if (fpFixed) {
                        double fxyz[3] = { rovFixXyz.x, rovFixXyz.y, rovFixXyz.z };
                        double enu[3];
                        bool hasDiff = DiffVsTruth(truthMap,
                            raw.BasEpk.Time.Week, raw.BasEpk.Time.SecOfWeek, fxyz, enu);
                        fprintf(fpFixed, "%d%s%.3f%s%.3f%s%.3f%s%.3f%s%.3f%s%.3f%s%.3f%s%.4f%s"
                            "%.9f%s%.9f%s%.3f%s%.2f%s%.4f%s%.4f%s%.4f%s%.4f%s%.4f",
                            raw.BasEpk.Time.Week, d, raw.BasEpk.Time.SecOfWeek, d,
                            fxyz[0], d, fxyz[1], d, fxyz[2], d,
                            fx, d, fy, d, fz, d, flen, d,
                            fixBlh.latitude * Deg, d, fixBlh.longitude * Deg, d, fixBlh.height, d,
                            raw.DDObs.Ratio, d,
                            raw.DDObs.FixRMSXYZ[0], d, raw.DDObs.FixRMSXYZ[1], d, raw.DDObs.FixRMSXYZ[2],
                            d, raw.DDObs.AmbTestDiff, d, raw.DDObs.AmbTestVar);
                        if (hasDiff) {
                            fprintf(fpFixed, ",%.4f,%.4f,%.4f\n", enu[0], enu[1], enu[2]);
                            fxE += enu[0]; fxN += enu[1]; fxU += enu[2];
                            fxE2 += enu[0] * enu[0]; fxN2 += enu[1] * enu[1]; fxU2 += enu[2] * enu[2];
                            if (fabs(enu[0]) > fxMaxE) fxMaxE = fabs(enu[0]);
                            if (fabs(enu[1]) > fxMaxN) fxMaxN = fabs(enu[1]);
                            if (fabs(enu[2]) > fxMaxU) fxMaxU = fabs(enu[2]);
                            nFxDiff++;
                        }
                        else {
                            fprintf(fpFixed, "%s%s\n", d, d);
                        }
                        fflush(fpFixed);
                    }
                }
            }
            else {
                printf("[FLOAT] %s failed\n", useKF ? "SolveKF" : "SolveFloat");
            }

        }

        // 写本历元解状态（solve_status.csv，固定解优先于浮点解）
        if (fpStatus) {
            const char* st = (epState == 2) ? "FIXED" : (epState == 1) ? "FLOAT" : "NONE";
            fprintf(fpStatus, "%d,%.3f,%s,%.4f,%.2f,%.4f,%d,%d\n",
                raw.BasEpk.Time.Week, raw.BasEpk.Time.SecOfWeek, st,
                epBaseLen, epRatio, raw.DdSigma0,
                raw.DdObs.NGPS, raw.DdObs.NBDS);
        }
    }

    // 解算摘要
    printf("\n========================================\n");
    printf("Summary:\n");
    printf("  Total epochs      : %d\n", nEpoch);
    printf("  Base SPP OK       : %d\n", nBaseOK);
    printf("  Rover SPP OK      : %d\n", nRoverOK);
    printf("========================================\n");

    // 模糊度确认三项检验的逐项通过情况（课件 2-5 第 17 页）
    PrintAmbTestSummary();

    // 与参考真值的 E/N/U 差值统计摘要（均值/RMS/最大绝对值，单位米）
    printf("\n---- Truth comparison (E/N/U, m) ----\n");
    auto PrintStat = [](const char* name, long n,
        double sE, double sN, double sU, double qE, double qN, double qU,
        double mE, double mN, double mU)
    {
        printf("  %-6s n=%-5ld E: mean=%+.4f rms=%.4f max=%.4f | "
            "N: mean=%+.4f rms=%.4f max=%.4f | U: mean=%+.4f rms=%.4f max=%.4f\n",
            name, n,
            n ? sE / n : 0.0, n ? sqrt(qE / n) : 0.0, mE,
            n ? sN / n : 0.0, n ? sqrt(qN / n) : 0.0, mN,
            n ? sU / n : 0.0, n ? sqrt(qU / n) : 0.0, mU);
    };
    PrintStat("Float", nFlDiff, flE, flN, flU, flE2, flN2, flU2, flMaxE, flMaxN, flMaxU);
    PrintStat("Fixed", nFxDiff, fxE, fxN, fxU, fxE2, fxN2, fxU2, fxMaxE, fxMaxN, fxMaxU);

    // 统计摘要同步写出 CSV
    FILE* fpSum = fopen("compare_summary.csv", "w");
    if (fpSum) {
        fprintf(fpSum, "Solution,Count,MeanE_m,RMSE_m,MaxAbsE_m,"
            "MeanN_m,RMSN_m,MaxAbsN_m,MeanU_m,RMSU_m,MaxAbsU_m\n");
        fprintf(fpSum, "Float,%ld,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
            nFlDiff,
            nFlDiff ? flE / nFlDiff : 0.0, nFlDiff ? sqrt(flE2 / nFlDiff) : 0.0, flMaxE,
            nFlDiff ? flN / nFlDiff : 0.0, nFlDiff ? sqrt(flN2 / nFlDiff) : 0.0, flMaxN,
            nFlDiff ? flU / nFlDiff : 0.0, nFlDiff ? sqrt(flU2 / nFlDiff) : 0.0, flMaxU);
        fprintf(fpSum, "Fixed,%ld,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
            nFxDiff,
            nFxDiff ? fxE / nFxDiff : 0.0, nFxDiff ? sqrt(fxE2 / nFxDiff) : 0.0, fxMaxE,
            nFxDiff ? fxN / nFxDiff : 0.0, nFxDiff ? sqrt(fxN2 / nFxDiff) : 0.0, fxMaxN,
            nFxDiff ? fxU / nFxDiff : 0.0, nFxDiff ? sqrt(fxU2 / nFxDiff) : 0.0, fxMaxU);
        fclose(fpSum);
        printf("  compare_summary.csv written\n");
    }

    if (useFile) {
        if (fpBase)  fclose(fpBase);
        if (fpRover) fclose(fpRover);
    }
    else if (useNet) {
        if (sBase != INVALID_SOCKET) CloseSocket(sBase);
        if (sRov  != INVALID_SOCKET) CloseSocket(sRov);
    }
    else {
        comBase.Close();
        comRov.Close();
    }
    if (fpSpp)   fclose(fpSpp);
    if (fpFloat) fclose(fpFloat);
    if (fpFixed) fclose(fpFixed);
    if (fpStatus) fclose(fpStatus);
    if (fpRecBase) fclose(fpRecBase);
    if (fpRecRov)  fclose(fpRecRov);

    printf("\nDone.\n");
    PauseIfConsole();
    return 0;
}
