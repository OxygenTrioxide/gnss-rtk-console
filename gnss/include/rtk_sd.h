#ifndef RTK_SD_H
#define RTK_SD_H

#include "RTK_Structs.h"
#include "gnss_coord.h"
#include <cstdio>

// 数据同步：文件模式（网络/串口模式为同名重载，见 sockets.h / serial.h）
int  GetSynGNSSObs(FILE* fpBase, FILE* fpRover, RAWDAT* raw, double thresSec);

void FormSDEpochObs(RAWDAT* raw);
void DetectCycleSlip(RAWDAT* raw, double mwThres, double gfThres);
void CheckLockTimeParity(RAWDAT* raw);
int  ReadBasePosFromPos(const char* file, BLH& basBlh);
void DetRefSat(RAWDAT* raw, double cnoThres, double elevThres, double lockThres);
void CalcDD(RAWDAT* raw, const double basPos[3], const double rovPos[3]);

// 相对定位浮点解（最小二乘）
//   sigmaP/sigmaL : 伪距/载波相位观测噪声（米），对应配置 CODE AND CP NOISE
bool SolveFloat(RAWDAT* raw, const double basPos[3], double rovPos[3],
    double sigmaP = 0.3, double sigmaL = 0.05);

// 相对定位浮点解（卡尔曼滤波），输出接口与 SolveFloat 相同
bool SolveKF(RAWDAT* raw, const double basPos[3], double rovPos[3],
    double sigmaP = 0.3, double sigmaL = 0.05);

// LAMBDA 模糊度固定（课件 2-5：整数变换降相关 + 搜索 + 三项模糊度确认 + 回代）
// ambSigFloor：检验①的未建模误差方差地板（m），见 .cpp 中注释
// ambVarFactor：检验②的放宽倍数（乘在 χ² 上界），见 .cpp 中注释
bool FixDDAmb(RAWDAT* raw, const double basPos[3], double ratioThres,
    double sigmaP = 0.3, double sigmaL = 0.05, double ambSigFloor = 0.1,
    double ambVarFactor = 2.0);

// 模糊度确认统计汇总（三项检验逐项通过情况），在程序结束时打印
void PrintAmbTestSummary();

// 清空双站同步缓冲（网络模式断线重连后调用，防止跨连接拼帧）
void ResetNetSyncBuffers();

// 原始数据流落盘（可选，配置 RECORD RAW DATA 开启）：
// 网络模式下把基站/流动站收到的原始字节同步写入对应文件，供事后回放分析
void SetRawRecordFiles(FILE* fpBase, FILE* fpRov);

#endif
