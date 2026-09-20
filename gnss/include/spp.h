#ifndef SPP_H
#define SPP_H

#include "RTK_Structs.h"

// 粗差探测：GF/MW 组合 + 限差检查 + MW 平滑 + PIF 计算
//   函数名与签名同课件 1-7 第 12 页 void DetectOutlier(EPOCHOBSDATA* Obs);
void DetectOutlier(EPOCHOBSDATA* Obs);

// 设置粗差探测限差（课件 1-7 第 11 页建议 GF 5cm / MW 3m）
// 由 main 依据配置项 OUTLIER THRESHOLD 调用；参数 ≤0 时保持原值
void SetOutlierThreshold(double gfThres, double mwThres);

// 单点定位：输入观测历元 + 星历，输出接收机位置/钟差/精度指标
//   elevMask  : 高度角截止角（弧度）
//   tropModel : 0=Hopfield, 1=Saastamoinen
bool SPP(EPOCHOBSDATA* Epoch, RAWDAT* Raw, POSRES* Result,
    double elevMask = 10.0 * Rad, int tropModel = 1);

#endif
