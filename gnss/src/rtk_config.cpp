#define _CRT_SECURE_NO_WARNINGS
#include "rtk_config.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>

// 关键字列表（按关键字匹配，配置文件中行顺序不限）
// 前 12 项与课件 2-1 第 26 页 CFGITEMS 列表一致，其后为本工程扩展项
static const char* CFG_KEYS[] = {
    "FROM FILE OR COM",
    "RTK PROCESSING MODE",
    "ROVER IP ADDRESS AND PORT",
    "BASE IP ADDRESS AND PORT",
    "ROVER COM SETUP",
    "BASE OBSDATA SOURCE FILE",
    "ROVER OBSDATA SOURCE FILE",
    "POSITION RESULT FILE",
    "CODE AND CARRIER PHASE NOISE",
    "THRESHOLD FOR ELEVATION MASK",
    "RATIO FOR DD FIXED SOLUTION",
    "ENABLE SPP OUTPUT",
    "ENABLE FLOAT OUTPUT",
    "ENABLE FIXED OUTPUT",
    "OUTPUT DIRECTORY",
    "TROP MODEL",
    "RESULT FILE FORMAT",
    "DATA SOURCE MODE",
    "BASE COM SETUP",
    "BASE POSITION",
    "CYCLE SLIP THRESHOLD",
    "OUTLIER THRESHOLD",
    "ENABLE AR TESTS",
    "MAX RUNNING MINUTES",
    "AMB TEST SIGMA FLOOR",
    "AMB TEST VAR FACTOR",
    "NET RECONNECT MINUTES",
    "RECORD RAW DATA"
};
static const int CFG_NUM = (int)(sizeof(CFG_KEYS) / sizeof(CFG_KEYS[0]));

static void TrimEOL(char* s)
{
    int L = (int)strlen(s);
    while (L > 0 && (s[L - 1] == '\r' || s[L - 1] == '\n' ||
        s[L - 1] == ' ' || s[L - 1] == '\t'))
        s[--L] = '\0';
}

// 跳过 UTF-8 BOM（配置文件可能被编辑器保存为带 BOM 的 UTF-8）
static char* SkipBom(char* p)
{
    unsigned char* q = (unsigned char*)p;
    if (q[0] == 0xEF && q[1] == 0xBB && q[2] == 0xBF) return p + 3;
    return p;
}

bool ReadRTKConfigInfo(const char* FName, ROVERCFGINFO& cfg)
{
    FILE* fp = fopen(FName, "rt");
    if (!fp) {
        printf("ERROR: Cannot open config file: %s\n", FName);
        return false;
    }

    char line[512];
    while (fgets(line, sizeof(line), fp)) {
        // 跳过空行和注释
        char* p = SkipBom(line);
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '\0' || *p == '\n' || *p == '\r' || *p == '#' || *p == '[') continue;

        // 找关键字
        for (int i = 0; i < CFG_NUM; i++) {
            int klen = (int)strlen(CFG_KEYS[i]);
            if (strncmp(p, CFG_KEYS[i], klen) != 0) continue;
            // 关键字后必须是 ':'
            char* colon = strchr(p + klen, ':');
            if (!colon) break;
            char* val = colon + 1;
            while (*val == ' ' || *val == '\t') val++;

            // 按 key 赋值
            switch (i) {
            case 0:  cfg.IsFileData = (short)atoi(val); break;
            case 1:  cfg.RTKProcMode = (short)atoi(val); break;
            case 2: { // ROVER IP + PORT
                char ip[32] = { 0 };
                int port = 0;
                sscanf(val, "%31s %d", ip, &port);
                strncpy(cfg.RovNetIP, ip, sizeof(cfg.RovNetIP) - 1);
                cfg.RovNetPort = (short)port;
                break;
            }
            case 3: { // BASE IP + PORT
                char ip[32] = { 0 };
                int port = 0;
                sscanf(val, "%31s %d", ip, &port);
                strncpy(cfg.BasNetIP, ip, sizeof(cfg.BasNetIP) - 1);
                cfg.BasNetPort = (short)port;
                break;
            }
            case 4: { // 流动站串口：COM 口号 + 波特率（只写一个数时按波特率解释）
                int a = 0, b = 0;
                int nn = sscanf(val, "%d %d", &a, &b);
                if (nn >= 2) { cfg.RovPort = a; cfg.RovBaud = b; }
                else if (nn == 1) { cfg.RovPort = 2; cfg.RovBaud = a; }
                break;
            }
            case 5: strncpy(cfg.BasObsDatFile, val, sizeof(cfg.BasObsDatFile) - 1);
                cfg.BasObsDatFile[sizeof(cfg.BasObsDatFile) - 1] = '\0';
                TrimEOL(cfg.BasObsDatFile); break;
            case 6: strncpy(cfg.RovObsDatFile, val, sizeof(cfg.RovObsDatFile) - 1);
                cfg.RovObsDatFile[sizeof(cfg.RovObsDatFile) - 1] = '\0';
                TrimEOL(cfg.RovObsDatFile); break;
            case 7: strncpy(cfg.ResFile, val, sizeof(cfg.ResFile) - 1);
                cfg.ResFile[sizeof(cfg.ResFile) - 1] = '\0';
                TrimEOL(cfg.ResFile); break;
            case 8:  sscanf(val, "%lf %lf", &cfg.CodeNoise, &cfg.CPNoise); break;
            case 9:  cfg.ElevThreshold = atof(val); break;
            case 10: cfg.RatioThres = atof(val); break;
            case 11: cfg.EnableSppOutput = (short)atoi(val); break;
            case 12: cfg.EnableFloatOutput = (short)atoi(val); break;
            case 13: cfg.EnableFixedOutput = (short)atoi(val); break;
            case 14: strncpy(cfg.OutputDir, val, sizeof(cfg.OutputDir) - 1);
                cfg.OutputDir[sizeof(cfg.OutputDir) - 1] = '\0';
                TrimEOL(cfg.OutputDir); break;
            case 15: cfg.TropModel = (short)atoi(val); break;
            case 16: cfg.OutFormat = (short)atoi(val); break;
            case 17: cfg.DataSrcMode = atoi(val); break;              // 0=文件 1=网络 2=串口
            case 18: { // 基站串口
                int a = 0, b = 0;
                int nn = sscanf(val, "%d %d", &a, &b);
                if (nn >= 2) { cfg.BasPort = a; cfg.BasBaud = b; }
                else if (nn == 1) { cfg.BasPort = 2; cfg.BasBaud = a; }
                break;
            }
            case 19: // 基站坐标（ECEF X Y Z，单位 m）
                if (sscanf(val, "%lf %lf %lf", &cfg.BasPosXYZ[0], &cfg.BasPosXYZ[1],
                    &cfg.BasPosXYZ[2]) == 3) cfg.BasPosGiven = 1;
                break;
            case 20: sscanf(val, "%lf %lf", &cfg.CsMwThres, &cfg.CsGfThres); break;
            case 21: sscanf(val, "%lf %lf", &cfg.OutGfThres, &cfg.OutMwThres); break;
            case 22: cfg.EnableArTests = (short)atoi(val); break;
            case 23: cfg.MaxRunMinutes = atof(val); break;
            case 24: cfg.AmbTestSigmaFloor = atof(val); break;
            case 25: cfg.AmbTestVarFactor = atof(val); break;
            case 26: cfg.NetReconnectMinutes = atof(val); break;
            case 27: cfg.RecordRaw = (short)atoi(val); break;
            default: break;
            }
            break;
        }
    }
    fclose(fp);
    return true;
}

void PrintRTKConfigInfo(const ROVERCFGINFO& cfg)
{
    printf("=== RTK Config ===\n");
    printf("  From File/COM : %d\n", cfg.IsFileData);
    printf("  Data Source   : %d (%s)\n", cfg.DataSrcMode,
        cfg.DataSrcMode == 0 ? "File" : (cfg.DataSrcMode == 1 ? "Network" : "COM/Serial"));
    printf("  RTK Mode      : %d (%s)\n", cfg.RTKProcMode,
        cfg.RTKProcMode == 1 ? "EKF/卡尔曼滤波" : "LSQ/最小二乘");
    printf("  Base IP:Port  : %s:%d\n", cfg.BasNetIP, cfg.BasNetPort);
    printf("  Rover IP:Port : %s:%d\n", cfg.RovNetIP, cfg.RovNetPort);
    printf("  Base COM      : COM%d %d bps\n", cfg.BasPort, cfg.BasBaud);
    printf("  Rover COM     : COM%d %d bps\n", cfg.RovPort, cfg.RovBaud);
    printf("  Base File     : %s\n", cfg.BasObsDatFile);
    printf("  Rover File    : %s\n", cfg.RovObsDatFile);
    printf("  Result File   : %s\n", cfg.ResFile);
    printf("  Code/CP Noise : %.3f %.4f\n", cfg.CodeNoise, cfg.CPNoise);
    printf("  Elev Thres    : %.1f deg\n", cfg.ElevThreshold);
    printf("  Ratio Thres   : %.2f\n", cfg.RatioThres);
    printf("  Trop Model    : %d (%s)\n", cfg.TropModel,
        cfg.TropModel == 0 ? "Hopfield" : "Saastamoinen");
    printf("  Output Format : %d (%s)\n", cfg.OutFormat,
        cfg.OutFormat == 1 ? "csv" : "txt");
    printf("  CycleSlip MW/GF: %.2f %.3f\n", cfg.CsMwThres, cfg.CsGfThres);
    printf("  Outlier GF/MW : %.3f %.2f\n", cfg.OutGfThres, cfg.OutMwThres);
    printf("  AR extra tests: %d\n", cfg.EnableArTests);
    printf("  AmbTest floor : %.3f m (检验①未建模误差方差地板)\n", cfg.AmbTestSigmaFloor);
    printf("  AmbTest vfact : %.1f (检验② chi2 上界放宽倍数)\n", cfg.AmbTestVarFactor);
    printf("  Net reconnect : %.1f min (断线自动重连预算, 0=不重连)\n", cfg.NetReconnectMinutes);
    if (cfg.MaxRunMinutes > 0.0) {
        int totalSec = (int)(cfg.MaxRunMinutes * 60.0 + 0.5);
        printf("  Max run time  : %d 小时 %d 分 %d 秒（到点自动收尾退出）\n",
            totalSec / 3600, (totalSec % 3600) / 60, totalSec % 60);
    }
    if (cfg.BasPosGiven)
        printf("  Base Position : %.3f %.3f %.3f (ECEF)\n",
            cfg.BasPosXYZ[0], cfg.BasPosXYZ[1], cfg.BasPosXYZ[2]);
    else
        printf("  Base Position : (未指定，取数据流 PSRPOS/BESTPOS 或真值文件首行)\n");
    printf("  EnableSppOut  : %d\n", cfg.EnableSppOutput);
    printf("  EnableFltOut  : %d\n", cfg.EnableFloatOutput);
    printf("  EnableFixOut  : %d\n", cfg.EnableFixedOutput);
    printf("  OutputDir     : %s\n", cfg.OutputDir);
}
