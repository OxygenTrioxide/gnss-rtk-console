#ifndef RTK_STRUCTS_H
#define RTK_STRUCTS_H

#include <Eigen/Dense>
#include <cmath>
#include <cstdio>
#include <cstring>

using namespace Eigen;

// ---------------- 常量定义 ----------------

#define PAI 3.1415926535898
#define PAI2 (2.0 * PAI)
#define Rad (PAI / 180.0)      // 度 → 弧度
#define Deg (180.0 / PAI)      // 弧度 → 度
#define C_Light 299792458.0    // 真空光速 (m/s)

// 地球椭球与引力常数：WGS84 用于 GPS，CGCS2000 用于 BDS
#define R_WGS84 6378137.0              // 长半轴 (m)
#define F_WGS84 (1.0 / 298.257223563)  // 扁率
#define Omega_WGS 7.2921151467e-5      // 地球自转角速度 (rad/s)
#define GM_Earth 398600.5e+9           // 地球引力常数 (m³/s²)
#define R_CGS2K 6378137.0
#define F_CGS2K (1.0 / 298.257222101)
#define Omega_BDS 7.2921150e-5
#define GM_BDS 398600.4418e+9

// GPS 载波频率 (Hz) 与双频组合系数：IF 组合 PIF = (f1²·P1 − f2²·P2)/(f1² − f2²)
#define FG1_GPS 1575.42E6      // L1
#define FG2_GPS 1227.60E6      // L2
#define FG12R (77 / 60.0)      // f1/f2
#define FG12R2 (5929 / 3600.0) // (f1/f2)²
#define WL1_GPS (C_Light / FG1_GPS)   // L1 波长 (m)
#define WL2_GPS (C_Light / FG2_GPS)   // L2 波长 (m)

// BDS 载波频率 (Hz)：B1/B2/B3
#define FG1_BDS 1561.098E6
#define FG2_BDS 1207.140E6
#define FG3_BDS 1268.520E6
#define FC12R (FG1_BDS / FG2_BDS)
#define FC12R2 (FC12R * FC12R)
#define FC13R (FG1_BDS / FG3_BDS)
#define FC13R2 (FC13R * FC13R)
#define WL1_BDS (C_Light / FG1_BDS)
#define WL2_BDS (C_Light / FG2_BDS)
#define WL3_BDS (C_Light / FG3_BDS)

#define GPST_BDT 14        // GPST 与 BDT 的系统时差 (s)：GPST = BDT + 14
#define MAXCHANNUM 36      // 单历元最大通道数
#define MAXGPSNUM 32
#define MAXBDSNUM 63
#define MAXRAWLEN 40960    // OEM7 原始数据缓冲区长度

// ---------------- 枚举定义 ----------------

enum GNSSSys { UNKS = 0, GPS, BDS, GLONASS, GALILEO, QZSS };

// ---------------- 时间结构体 ----------------

struct COMMONTIME {
    short Year;
    unsigned short Month;
    unsigned short Day;
    unsigned short Hour;
    unsigned short Minute;
    double Second;

    COMMONTIME() {
        Year = 0;
        Month = 0;
        Day = 0;
        Hour = 0;
        Minute = 0;
        Second = 0.0;
    }
};

struct GPSTIME {
    short Week;
    double SecOfWeek;

    GPSTIME() {
        Week = 0;
        SecOfWeek = 0.0;
    }
};

struct MJDTIME {
    int Days;
    double FracDay;

    MJDTIME() {
        Days = 0;
        FracDay = 0.0;
    }
};

// ---------------- 星历结构体（广播星历开普勒根数） ----------------

struct GPSEPHREC {
    unsigned short PRN;
    GNSSSys Sys;
    GPSTIME TOC, TOE;
    short SVHealth;
    double ClkBias, ClkDrift, ClkDriftRate;   // 卫星钟差参数 a0/a1/a2
    double IODE, IODC;
    double TGD1, TGD2;                        // 频间偏差改正
    // 开普勒轨道根数与调和改正项
    double SqrtA;     // 轨道长半轴平方根 (√m)
    double e;         // 偏心率
    double M0;        // 参考时刻平近点角 (rad)
    double OMEGA;     // 参考时刻升交点赤经 (rad)
    double i0;        // 参考时刻轨道倾角 (rad)，BDS 中 i0<30° 判定 GEO
    double omega;     // 近地点幅角 (rad)
    double OMEGADot;  // 升交点赤经变率 (rad/s)
    double iDot;      // 轨道倾角变率 (rad/s)
    double DeltaN;    // 平均运动改正量 (rad/s)
    double Crs, Cuc, Cus, Cic, Cis, Crc;   // 二阶调和改正项振幅
    double SVAccuracy;
    bool Valid;       // 星历是否有效（解码成功即置位，健康检查见 EphUsable）

    GPSEPHREC() {
        PRN = SVHealth = 0;
        Sys = UNKS;
        Valid = false;
        ClkBias = ClkDrift = ClkDriftRate = IODE = IODC = TGD1 = TGD2 = 0.0;
        SqrtA = e = M0 = OMEGA = i0 = omega = OMEGADot = iDot = DeltaN = 0.0;
        Crs = Cuc = Cus = Cic = Cis = Crc = SVAccuracy = 0.0;
    }
};

// ---------------- 观测数据结构体 ----------------

// 每颗卫星的观测数据（字段名与课件 1-5 第 48 页 SATOBSDATA 一致）
struct SATOBSDATA {
    short Prn;
    GNSSSys System;
    double P[2];       // 伪距 (L1/B1, L2/B3, m)
    double L[2];       // 载波相位 (m)
    double D[2];       // 多普勒 (Hz)
    double cn0[2];     // 载噪比 (dB-Hz)
    double LockTime[2];// 连续跟踪时长 (s)
    unsigned char half[2];    // 半周模糊度标志（跟踪状态 bit28）
    unsigned char parity[2];  // 奇偶校验标志（跟踪状态 bit11，=0 可能有半周）
    unsigned char plock[2];   // 相位锁定标志（跟踪状态 bit10）
    unsigned char clock[2];   // 码锁定标志（跟踪状态 bit12）
    bool Valid;

    SATOBSDATA() {
        Prn = 0;
        System = UNKS;
        for (int i = 0; i < 2; i++) {
            P[i] = L[i] = D[i] = cn0[i] = LockTime[i] = 0.0;
            half[i] = 0;
            parity[i] = 0;
            plock[i] = 0;
            clock[i] = 0;
        }
        Valid = false;
    }
};

struct MWGF {
    short Prn;
    GNSSSys Sys;
    double MW, GF, PIF;   // Melbourne-Wübbena / 几何无关（载波减伪距）/ 消电离层组合
    int n;                // MW 平滑计数（L̄ = [(n−1)·L̄ + L]/n）

    MWGF() {
        Prn = n = 0;
        Sys = UNKS;
        MW = GF = PIF = 0.0;
    }
};

struct SATPVT {
    double SatPos[3], SatVel[3];      // 卫星位置（自转改正后）/速度 (ECEF)
    double SatClkOft, SatClkSft;      // 卫星钟差 (s) / 钟速 (s/s)
    double Elevation, Azimuth;        // 高度角/方位角 (rad)
    double TropCorr;                  // 对流层改正 (m)
    double Tgd1, Tgd2;                // 频间偏差改正 (m)
    bool Valid;

    SATPVT() {
        SatPos[0] = SatPos[1] = SatPos[2] = 0.0;
        SatVel[0] = SatVel[1] = SatVel[2] = 0.0;
        Elevation = PAI / 2.0;
        SatClkOft = SatClkSft = 0.0;
        Tgd1 = Tgd2 = TropCorr = 0.0;
        Valid = false;
    }
};

// ---------------- 历元观测数据结构体 ----------------

struct EPOCHOBSDATA {
    GPSTIME Time;
    short SatNum;
    SATOBSDATA SatObs[MAXCHANNUM];
    SATPVT SatPVT[MAXCHANNUM];
    MWGF ComObs[MAXCHANNUM];   // 粗差探测组合观测值（跨历元保留，解码只重置 SatObs）
    double Pos[3];

    EPOCHOBSDATA() {
        SatNum = 0;
        Pos[0] = Pos[1] = Pos[2] = 0.0;
    }

    void Reset() {
        Time.Week = 0;
        Time.SecOfWeek = 0.0;
        SatNum = 0;
        for (int i = 0; i < MAXCHANNUM; i++) {
            SatObs[i] = SATOBSDATA();
            SatPVT[i] = SATPVT();
            ComObs[i] = MWGF();
        }
        Pos[0] = Pos[1] = Pos[2] = 0.0;
    }
};

// ---------------- 单差/双差结构体 ----------------

struct SDSATOBS {
    short Prn;
    GNSSSys System;
    short Valid;
    double dP[2], dL[2];   // 站间单差伪距/载波
    short nBas, nRov;      // 基/流动站在观测数组中的索引

    SDSATOBS() {
        Prn = nBas = nRov = 0;
        System = UNKS;
        dP[0] = dL[0] = dP[1] = dL[1] = 0.0;
        Valid = -1;
    }
};

struct SDEPOCHOBS {
    GPSTIME Time;
    short SatNum;
    SDSATOBS SdSatObs[MAXCHANNUM];
    MWGF SdCObs[MAXCHANNUM];   // 单差 MW/GF 周跳探测组合（含平滑计数）
};

struct DDCOBS {
    int RefPrn[2], RefPos[2];
    int Sats, DDSatNum[2];
    // 双频最优解 [0,AmbNum) 与次优解 [AmbNum,2*AmbNum)（单位：周）
    double FixedAmb[MAXCHANNUM * 4];
    double ResAmb[2], Ratio;   // LAMBDA 最优解与次优解的残差二次型 s1/s2 及 ratio=s2/s1
    double FixRMS;             // 固定解定位的 rms 误差（三维综合，m）
    float  FixRMSXYZ[3];       // 固定解 X/Y/Z 三分量中误差 (m)
    double dPos[3];
    double FixPos[3];      // 固定解基线向量 (m)，Status 时有效
    // 供 LAMBDA 使用的浮点模糊度导出量（单位：周）
    int    AmbNum;                                // 模糊度个数 = 2 × 双差星数，0=无效
    double FloatAmbCyc[MAXCHANNUM * 2];           // 浮点模糊度 (周)
    double Qamb[MAXCHANNUM * 2 * MAXCHANNUM * 2]; // 协方差阵 (周²)，行宽 MAXCHANNUM*2
    bool Status;                                  // true=固定，false=未固定
    // 模糊度确认统计量（课件 2-5 第 17 页三项检验）
    double AmbTestDiff;    // ① 基线一致性：(Xf-Xf0)ᵀQ⁻¹(Xf-Xf0)，限值 7.815（χ²0.95,3）
    double AmbTestVar;     // ② 单位权中误差一致性：σ̂²_fix/σ̂²_float
    double FloatSigma0;    // 浮点解单位权中误差（用于 ②）

    DDCOBS() {
        for (int i = 0; i < 2; i++) {
            DDSatNum[i] = 0;
            RefPos[i] = RefPrn[i] = -1;
        }
        Sats = 0;
        dPos[0] = dPos[1] = dPos[2] = 0.0;
        FixPos[0] = FixPos[1] = FixPos[2] = 0.0;
        ResAmb[0] = ResAmb[1] = Ratio = 0.0;
        AmbNum = 0;
        for (int i = 0; i < MAXCHANNUM * 2; i++) FloatAmbCyc[i] = 0.0;
        for (int i = 0; i < MAXCHANNUM * 2 * MAXCHANNUM * 2; i++) Qamb[i] = 0.0;
        FixRMS = 0.0;
        FixRMSXYZ[0] = FixRMSXYZ[1] = FixRMSXYZ[2] = 0.0f;
        AmbTestDiff = AmbTestVar = FloatSigma0 = 0.0;
        Status = false;
        for (int i = 0; i < MAXCHANNUM * 2; i++) {
            FixedAmb[2 * i + 0] = FixedAmb[2 * i + 1] = 0.0;
        }
    }
};

// 每颗卫星的双差观测
struct DDSATOBS {
    short Prn;
    GNSSSys System;
    double dP[2];         // 双差伪距 (L1/B1, L2/B3, m)
    double dL[2];         // 双差载波 (m)
    double los[3];        // 双差方向余弦 = e_rovʲ − e_rovⁱ
    double rho0;          // 双差几何距离（线性化初值，m）
    double amb[2];        // 双频模糊度 (周)
    bool fValid[2];       // 两频点观测是否有效
    bool Valid;
};

// 每个历元的双差观测
struct DDEPOCHOBS {
    GPSTIME Time;
    short SatNum;                       // 双差卫星总数
    short RefPrn[2], RefIdx[2];         // 参考星 PRN 和索引
    DDSATOBS DdSatObs[MAXCHANNUM * 2];  // 双差观测数组
    int NGPS, NBDS;                     // 各系统双差卫星数
};

// ---------------- 定位结果结构体 ----------------

struct POSRES {
    GPSTIME Time;
    double Position[3];
    double Velocity[3];
    double RcvClkOft[2];   // 接收机钟差 (GPS/BDS, m)
    double RcvClkSft;      // 接收机钟漂 (m/s)
    double PDOP, SigmaPos, SigmaVel;
    short GPSSatNum, BDSSatNum;
    short AllSatNum;
    bool IsSuccess;
    double Sigma0;         // 单位权中误差
    double Qxx, Qyy, Qzz;  // 协因数对角元
    double mX, mY, mZ;     // 坐标分量中误差 (m)

    POSRES() {
        for (int i = 0; i < 3; i++) Position[i] = Velocity[i] = 0.0;
        RcvClkOft[0] = RcvClkOft[1] = RcvClkSft = 0.0;
        PDOP = SigmaPos = SigmaVel = 999.9;
        GPSSatNum = BDSSatNum = AllSatNum = 0;
        IsSuccess = false;
    }
};

// ---------------- RTK 相关结构体 ----------------

struct RAWDAT {
    EPOCHOBSDATA BasEpk;
    EPOCHOBSDATA RovEpk;
    SDEPOCHOBS SdObs;
    DDCOBS DDObs;
    DDEPOCHOBS DdObs;
    GPSEPHREC GpsEph[MAXGPSNUM], BdsEph[MAXBDSNUM];
    double DdSigma0;       // 单位权中误差
    double DdQxx[3];       // 坐标协因数对角元
    double DdMxyz[3];      // 坐标分量中误差 (m)
    double DdQxx33[9];     // 浮点解坐标协因数阵 3×3（行优先，模糊度确认用）
    int    DdFloatDof;     // 浮点解自由度 n−u
    POSRES BasPos;         // 基站数据流解出的定位结果（PSRPOS/BESTPOS）
    bool   BasPosValid;    // 基站坐标是否已从数据流获得

    RAWDAT() {
        DdSigma0 = 0.0;
        DdQxx[0] = DdQxx[1] = DdQxx[2] = 0.0;
        DdMxyz[0] = DdMxyz[1] = DdMxyz[2] = 0.0;
        for (int i = 0; i < 9; i++) DdQxx33[i] = 0.0;
        DdFloatDof = 0;
        BasPosValid = false;
    }
};

struct ROVERCFGINFO {
    short IsFileData, RTKProcMode;   // 课件 2-1 第 24 页：IsFileData 1=FILE 0=COM；RTKProcMode 1=EKF 2=LSQ
    int DataSrcMode;                 // 数据源：0=文件 1=网络 2=串口（-1=未设置，用 IsFileData 推断）
    int RovPort, RovBaud;
    int BasPort, BasBaud;
    char BasNetIP[20], RovNetIP[20];
    short BasNetPort, RovNetPort;
    double CodeNoise, CPNoise;
    double ElevThreshold;
    double RatioThres;
    short TropModel;            // 对流层模型：0=Hopfield, 1=Saastamoinen (默认1)
    short OutFormat;            // 结果文件格式：0=txt (默认), 1=csv
    double BasPosXYZ[3];        // 基站坐标（ECEF, m）；BasPosGiven=1 时优先使用
    short BasPosGiven;
    double CsMwThres, CsGfThres;    // 周跳探测 MW/GF 阈值
    double OutGfThres, OutMwThres;  // 粗差探测 GF/MW 阈值
    short EnableArTests;            // 模糊度确认是否启用附加两项 χ² 检验
    double AmbTestSigmaFloor;       // 检验①未建模误差方差地板（m），0 = 不加地板
    double AmbTestVarFactor;        // 检验② χ² 上界放宽倍数，默认 2
    double NetReconnectMinutes;     // 网络断线自动重连预算（分钟），0 = 不重连
    short RecordRaw;                // 1=网络模式把原始数据流落盘到运行目录，0=关闭
    char BasObsDatFile[256], RovObsDatFile[256];
    char ResFile[256];
    short EnableSppOutput;      // 1=输出 SPP 结果, 0=不输出
    short EnableFloatOutput;    // 1=输出浮点解, 0=不输出
    short EnableFixedOutput;    // 1=输出固定解, 0=不输出
    char OutputDir[256];        // 输出文件目录（末尾必须带 / 或 \）

    double MaxRunMinutes;       // 最长解算时长（分钟），0 = 不限；到点后正常收尾退出，
                                // 打印统计摘要并以退出码 0 结束（供 GUI 计时任务使用）

    ROVERCFGINFO() {
        MaxRunMinutes = 0.0;
        IsFileData = RTKProcMode = 1;
        DataSrcMode = -1;
        RovPort = RovBaud = BasNetPort = RovNetPort = 0;
        BasPort = 2; BasBaud = 9600;        // 课件 1-5 第 13 页默认值
        CodeNoise = CPNoise = ElevThreshold = 0.0;
        RatioThres = 3.0;
        TropModel = 1;
        OutFormat = 0;
        BasPosXYZ[0] = BasPosXYZ[1] = BasPosXYZ[2] = 0.0;
        BasPosGiven = 0;
        CsMwThres = 3.0; CsGfThres = 0.15;
        OutGfThres = 0.05; OutMwThres = 3.0;
        EnableArTests = 1;
        AmbTestSigmaFloor = 0.1;
        AmbTestVarFactor = 2.0;
        NetReconnectMinutes = 30.0;
        RecordRaw = 0;
        BasNetIP[0] = RovNetIP[0] = '\0';
        BasObsDatFile[0] = RovObsDatFile[0] = '\0';
        ResFile[0] = '\0';
        EnableSppOutput = 1;
        EnableFloatOutput = 1;
        EnableFixedOutput = 1;
        OutputDir[0] = '\0';
    }
};

// ---------------- 函数声明 ----------------

// 解码（完整声明与默认参数见 oem7_decode.h）
int DecodeNovOem7Dat(unsigned char Buff[], int& Len, EPOCHOBSDATA* obs,
    GPSEPHREC geph[], GPSEPHREC beph[], POSRES* basPos);

// 粗差探测
void DetectOutlier(EPOCHOBSDATA* Obs);

// 数据同步（三种数据源重载；文件版声明在此，SOCKET/串口版见 sockets.h / serial.h）
int GetSynGNSSObs(FILE* fpBase, FILE* fpRover, RAWDAT* raw, double thresSec);

// SPP/SPV 的声明在 spp.h / spv.h（与各自定义一致）

#endif // RTK_STRUCTS_H
