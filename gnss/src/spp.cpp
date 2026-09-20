#define _CRT_SECURE_NO_WARNINGS
#include "spp.h"
#include "eph_orbit.h"
#include "gnss_coord.h"
#include "gnss_time.h"
#include <cmath>
#include <algorithm>

// 对流层改正模型，model：0=Hopfield，1=Saastamoinen
static double Hopfield(double H, double elev)
{
    // 标准气象元素（海平面 H0=0：T0=15℃+273.16，p0=1013.25mbar，RH0=50%）
    const double T0 = 15.0 + 273.16;    // K
    const double p0 = 1013.25;          // mbar
    const double RH0 = 0.5;             // 50%

    // 容错：测站高度不在对流层范围，直接输出 0
    if (H < 0.0 || H > 50000.0) return 0.0;

    double T  = T0 - 0.0065 * H;                        // 测站干温
    double p  = p0 * pow(1.0 - 0.0000226 * H, 5.225);   // 测站气压
    double RH = RH0 * exp(-0.0006396 * H);              // 测站相对湿度
    double e  = RH * exp(-37.2465 + 0.213166 * T - 0.000256908 * T * T);

    double hd = 40136.0 + 148.72 * (T - 273.16);        // 干分量高度
    double hw = 11000.0;                                // 湿分量高度

    // 课件 1-7 第 3 页 Hopfield 映射函数：
    //   ΔTrop = Kd / sin√(E²+6.25) + Kw / sin√(E²+2.25)
    // 注记：E 以"度"为单位代入（6.25/2.25 的量纲为度²），而 sin 的
    //       输入参数必须为弧度，故需乘 π/180。
    double elev_deg = elev * Deg;
    if (elev_deg < 0.01) elev_deg = 0.01;
    double sinKd = sin(sqrt(elev_deg * elev_deg + 6.25) * Rad);
    double sinKw = sin(sqrt(elev_deg * elev_deg + 2.25) * Rad);
    if (sinKd < 1e-6) sinKd = 1e-6;
    if (sinKw < 1e-6) sinKw = 1e-6;

    double dDry = 155.2e-7 * (p / T) * (hd - H) / sinKd;
    double dWet = 155.2e-7 * (4810.0 * e / (T * T)) * (hw - H) / sinKw;
    return dDry + dWet;
}

static double Saastamoinen(double H, double elev, double lat)
{
    const double P0 = 1013.25;
    const double T0 = 288.15;
    const double rh = 0.5;

    double T = T0 - 0.0065 * H;
    double P = P0 * pow(1.0 - 0.0065 * H / T0, 5.2561);
    double e = rh * 6.11 * exp(17.15 * (T - 273.15) / (T - 38.35));

    double f_phi_h = 1.0 - 0.00266 * cos(2.0 * lat) - 0.00028 * (H / 1000.0);
    double ztd = 0.002277 * (P + (1255.0 / T + 0.05) * e) / f_phi_h;

    double z = PAI / 2.0 - elev;
    double cosz = cos(z);
    if (cosz < 1e-6) cosz = 1e-6;

    return ztd / cosz;
}

static double TropDelay(double H, double elev, double lat, int model)
{
    return (model == 0) ? Hopfield(H, elev) : Saastamoinen(H, elev, lat);
}

// 无电离层组合（双频伪距 IF 组合）
static bool GetPIF(const SATOBSDATA& so, double* P)
{
    double p1 = so.P[0], p2 = so.P[1];
    if (p1 < 1e3 || p1 > 5e7) return false;
    if (p2 < 1e3 || p2 > 5e7) return false;

    if (so.System == GPS) {
        double f1 = FG1_GPS, f2 = FG2_GPS;
        *P = f1 * f1 * p1 / (f1 * f1 - f2 * f2) - f2 * f2 * p2 / (f1 * f1 - f2 * f2);
        return true;
    }
    if (so.System == BDS) {
        double f1 = FG1_BDS, f3 = FG3_BDS;
        *P = f1 * f1 * p1 / (f1 * f1 - f3 * f3) - f3 * f3 * p2 / (f1 * f1 - f3 * f3);
        return true;
    }
    return false;
}

// 通用高斯消元（列主元）
static int SolveN(double* A, double* b, int n)
{
    for (int col = 0; col < n; col++) {
        int piv = col; double mx = fabs(A[col * n + col]);
        for (int r = col + 1; r < n; r++) {
            double v = fabs(A[r * n + col]);
            if (v > mx) { mx = v; piv = r; }
        }
        if (mx < 1e-14) return -1;
        if (piv != col) {
            for (int j = 0; j < n; j++) std::swap(A[col * n + j], A[piv * n + j]);
            std::swap(b[col], b[piv]);
        }
        double p = A[col * n + col];
        for (int j = 0; j < n; j++) A[col * n + j] /= p;
        b[col] /= p;
        for (int r = 0; r < n; r++) {
            if (r == col) continue;
            double f = A[r * n + col];
            for (int j = 0; j < n; j++) A[r * n + j] -= f * A[col * n + j];
            b[r] -= f * b[col];
        }
    }
    return 0;
}

// 粗差探测（DetectOutlier 完整流程）：
//   1. 检查双频伪距和相位数据有效完整，否则 Valid=false
//   2. 计算 GF、MW 组合
//   3. 与上一历元（ComObs，MW 为平滑值）比较
//   4. dGF > 5cm 或 dMW > 3m → 粗差/周跳，Valid=false 并重置平滑
//   5. 通过则更新 MW 平滑值 L̄MW = [(n−1)·L̄MW + LMW]/n
//   6. 计算 IF 组合伪距 PIF 供 SPP 使用
//   7. 全部循环后 CurComObs 拷贝到 ComObs
// 粗差探测限差（课件 1-7 第 11 页建议 5cm / 3m），可由配置项 OUTLIER THRESHOLD 覆盖
static double g_GfThres = 0.05;   // GF 组合限差 (m)
static double g_MwThres = 3.0;    // MW 组合限差 (m)

void SetOutlierThreshold(double gfThres, double mwThres)
{
    if (gfThres > 0.0) g_GfThres = gfThres;
    if (mwThres > 0.0) g_MwThres = mwThres;
}

void DetectOutlier(EPOCHOBSDATA* Obs)
{
    if (!Obs) return;

    const double GF_THRES = g_GfThres;
    const double MW_THRES = g_MwThres;

    MWGF CurComObs[MAXCHANNUM];     // 当前历元组合结果（与 SatObs 同一下标）

    for (int i = 0; i < Obs->SatNum && i < MAXCHANNUM; i++)
    {
        SATOBSDATA& so = Obs->SatObs[i];
        CurComObs[i].Prn = so.Prn;
        CurComObs[i].Sys = so.System;

        // 步骤1：双频伪距和相位有效性与完整性
        if (!so.Valid ||
            so.P[0] < 1e3 || so.P[1] < 1e3 ||
            fabs(so.L[0]) < 1e-3 || fabs(so.L[1]) < 1e-3)
        {
            so.Valid = false;
            CurComObs[i].n = 0;     // 重置平滑
            continue;
        }

        // 步骤2：GF / MW 组合（载波相位单位为米）
        double f1, f2;
        if (so.System == GPS)      { f1 = FG1_GPS; f2 = FG2_GPS; }
        else if (so.System == BDS) { f1 = FG1_BDS; f2 = FG3_BDS; }
        else { so.Valid = false; CurComObs[i].n = 0; continue; }

        double mw = (f1 * so.L[0] - f2 * so.L[1]) / (f1 - f2)
            - (f1 * so.P[0] + f2 * so.P[1]) / (f1 + f2);
        double gf = so.L[0] - so.L[1];

        // 步骤3：查找上一历元该卫星的组合结果
        int prev = -1;
        for (int k = 0; k < MAXCHANNUM; k++) {
            if (Obs->ComObs[k].n > 0 &&
                Obs->ComObs[k].Prn == so.Prn &&
                Obs->ComObs[k].Sys == so.System) { prev = k; break; }
        }

        // 步骤4：限差检查
        if (prev >= 0) {
            double dGF = gf - Obs->ComObs[prev].GF;
            double dMW = mw - Obs->ComObs[prev].MW;   // 与平滑值比较
            if (fabs(dGF) > GF_THRES || fabs(dMW) > MW_THRES) {
                so.Valid = false;                     // 粗差/周跳
                CurComObs[i].MW = 0.0;
                CurComObs[i].GF = 0.0;
                CurComObs[i].PIF = 0.0;
                CurComObs[i].n = 0;                   // 重置平滑
                continue;
            }
        }

        // 步骤5：通过检查，更新 MW 平滑值
        int n = (prev >= 0) ? Obs->ComObs[prev].n + 1 : 1;
        double mwBar = (prev >= 0)
            ? ((n - 1) * Obs->ComObs[prev].MW + mw) / n
            : mw;

        // 步骤6：IF 组合伪距（供 SPP 使用）
        double pif;
        if (!GetPIF(so, &pif)) { so.Valid = false; CurComObs[i].n = 0; continue; }

        so.Valid = true;
        CurComObs[i].MW = mwBar;
        CurComObs[i].GF = gf;
        CurComObs[i].PIF = pif;
        CurComObs[i].n = n;
    }

    // 步骤7：当前历元结果保存到 ComObs
    memcpy(Obs->ComObs, CurComObs, sizeof(CurComObs));
}

// SPP 单点定位，elevMask 为高度角截止角（弧度）
bool SPP(EPOCHOBSDATA* Epoch, RAWDAT* Raw, POSRES* Result,
    double elevMask, int tropModel)
{
    if (!Epoch || !Raw || !Result) return false;

    double st[5] = { Epoch->Pos[0], Epoch->Pos[1], Epoch->Pos[2], 0.0, 0.0 };

    // 星历健康/过期检查：不健康或过期的卫星直接剔除，不参与定位
    for (int i = 0; i < Epoch->SatNum && i < MAXCHANNUM; i++) {
        if (!Epoch->SatObs[i].Valid) continue;
        GPSEPHREC* eph = nullptr;
        double t_chk = Epoch->Time.SecOfWeek;
        if (Epoch->SatObs[i].System == GPS && Epoch->SatObs[i].Prn >= 1
            && Epoch->SatObs[i].Prn <= MAXGPSNUM)
            eph = &Raw->GpsEph[Epoch->SatObs[i].Prn - 1];
        else if (Epoch->SatObs[i].System == BDS && Epoch->SatObs[i].Prn >= 1
            && Epoch->SatObs[i].Prn <= MAXBDSNUM) {
            eph = &Raw->BdsEph[Epoch->SatObs[i].Prn - 1];
            t_chk -= GPST_BDT;   // BDS：观测时刻转 BDT（TOE 为 BDT）
            if (t_chk < 0.0) t_chk += 604800.0;
        }
        if (!EphUsable(eph, t_chk)) { Epoch->SatObs[i].Valid = false; continue; }
    }

    // 未知数个数在每轮迭代内按"实际参与解算"的卫星重新确定（矩阵重构）
    int nU = 4;
    int n = 0;
    bool ok = false;

    double Q[25] = { 0 };   // 最后一次迭代的协因数阵

    for (int iter = 0; iter < 10; iter++)
    {
        const int MAXEQ = MAXCHANNUM;
        double H[MAXEQ][5];
        double z[MAXEQ];
        double w[MAXEQ];
        // ---- 第一遍：逐星计算几何量，只统计"真正能参与解算"的卫星 ----
        int    recIdx[MAXCHANNUM];
        double recD[MAXCHANNUM][3];
        double recRho[MAXCHANNUM], recElev[MAXCHANNUM], recP[MAXCHANNUM];
        double recTrop[MAXCHANNUM], recTgd[MAXCHANNUM];
        int    recSys[MAXCHANNUM];
        int    usedGps = 0, usedBds = 0;

        // 每轮迭代前重置卫星中间结果（上一轮的 Valid 不可带入本轮）
        for (int i = 0; i < Epoch->SatNum && i < MAXCHANNUM; i++)
            Epoch->SatPVT[i].Valid = false;

        n = 0;
        for (int i = 0; i < Epoch->SatNum && i < MAXCHANNUM; i++)
        {
            SATOBSDATA& so = Epoch->SatObs[i];
            if (!so.Valid) continue;

            double P;
            if (!GetPIF(so, &P)) continue;

            GPSEPHREC* eph = nullptr;
            if (so.System == GPS && so.Prn >= 1 && so.Prn <= MAXGPSNUM)
                eph = &Raw->GpsEph[so.Prn - 1];
            else if (so.System == BDS && so.Prn >= 1 && so.Prn <= MAXBDSNUM)
                eph = &Raw->BdsEph[so.Prn - 1];
            if (!eph || !eph->Valid) continue;

            // 信号发射时刻（观测时刻统一到星历时系：BDS→BDT，不改 TOE）
            double t_recv = Epoch->Time.SecOfWeek;
            if (so.System == BDS) {
                t_recv -= GPST_BDT;
                while (t_recv < 0.0)       t_recv += 604800.0;
                while (t_recv >= 604800.0) t_recv -= 604800.0;
            }
            double t_tx = t_recv - P / C_Light;
            double sv[3], vv[3], dts = 0.0, dtsd = 0.0;
            for (int k = 0; k < 3; k++) {   // 迭代计算两次，提高精度
                EphToECEF(eph, t_tx, sv, vv, &dts, &dtsd);
                t_tx = t_recv - P / C_Light - dts;
            }
            EphToECEF(eph, t_tx, sv, vv, &dts, &dtsd);

            // 地球自转改正（用几何距离计算传播时间，不能用伪距）
            double dx0 = sv[0] - st[0], dy0 = sv[1] - st[1], dz0 = sv[2] - st[2];
            double rho0 = sqrt(dx0 * dx0 + dy0 * dy0 + dz0 * dz0);
            if (rho0 < 1e3) continue;

            double omge = (eph->Sys == GPS) ? Omega_WGS : Omega_BDS;
            double th = omge * (rho0 / C_Light);
            double x1 = sv[0] * cos(th) + sv[1] * sin(th);
            double y1 = -sv[0] * sin(th) + sv[1] * cos(th);
            sv[0] = x1; sv[1] = y1;
            // 速度同步做自转改正
            double vx1 = vv[0] * cos(th) + vv[1] * sin(th);
            double vy1 = -vv[0] * sin(th) + vv[1] * cos(th);
            vv[0] = vx1; vv[1] = vy1;

            double dx = sv[0] - st[0], dy = sv[1] - st[1], dz = sv[2] - st[2];
            double rho = sqrt(dx * dx + dy * dy + dz * dz);
            if (rho < 1e3) continue;

            // 高度角 / 方位角（固定截止高度角，位置合理时才施加）
            BLH blh; XYZ xyz(st[0], st[1], st[2]);
            XYZToBLH(xyz, blh);
            double R[9]; BLHToNEUMat(blh, R);
            double ds[3] = { dx, dy, dz };
            NEU neu; XYZToNEU(ds, R, neu);
            double elev = atan2(neu.dU, sqrt(neu.dN * neu.dN + neu.dE * neu.dE));
            double azim = atan2(neu.dE, neu.dN);
            if (azim < 0.0) azim += PAI2;

            double rSize = sqrt(st[0] * st[0] + st[1] * st[1] + st[2] * st[2]);
            bool posOK = (rSize > 6.0e6 && rSize < 6.8e6);
            if (posOK && elev < elevMask) continue;   // 截止高度角掩码

            double elev_safe = (elev > 0.01) ? elev : 0.01;
            double trop = TropDelay(blh.height, elev_safe, blh.latitude, tropModel);

            // BDS TGD 改正（GPS 双频不加）
            double tgd = 0.0;
            if (so.System == BDS) {
                tgd = eph->TGD1 * FG1_BDS * FG1_BDS / (FG1_BDS * FG1_BDS - FG3_BDS * FG3_BDS);
            }

            // 保存卫星中间结果（自转改正后的位置，供 RTK 使用）
            SATPVT& mid = Epoch->SatPVT[i];
            mid.SatPos[0] = sv[0]; mid.SatPos[1] = sv[1]; mid.SatPos[2] = sv[2];
            mid.SatVel[0] = vv[0]; mid.SatVel[1] = vv[1]; mid.SatVel[2] = vv[2];
            mid.SatClkOft = dts;
            mid.SatClkSft = dtsd;
            mid.Elevation = elev;
            mid.Azimuth = azim;
            mid.TropCorr = trop;
            mid.Tgd1 = tgd;
            mid.Valid = true;

            // 暂存本星几何量，待未知数个数确定后再装配设计矩阵
            recIdx[n] = i;
            recD[n][0] = dx; recD[n][1] = dy; recD[n][2] = dz;
            recRho[n] = rho; recElev[n] = elev_safe;
            recP[n] = P; recTrop[n] = trop; recTgd[n] = tgd;
            recSys[n] = (int)so.System;
            n++;
            if (so.System == GPS) usedGps++; else usedBds++;
        }

        // 矩阵重构（课件 1-8 第 10 页、第 13 页⑤）：以"实际参与解算"的卫星
        // 判断系统构成——GPS 与 BDS 都有 → 5 个未知数（双钟差）；
        // 只有一种系统 → 删除另一系统的钟差行/列，未知数降为 4
        nU = (usedGps > 0 && usedBds > 0) ? 5 : 4;

        if (n < nU) {
            Result->IsSuccess = false;
            return false;
        }

        // ---- 第二遍：装配设计矩阵 H、残差 z 与权 w ----
        for (int r = 0; r < n; r++) {
            H[r][0] = -recD[r][0] / recRho[r];
            H[r][1] = -recD[r][1] / recRho[r];
            H[r][2] = -recD[r][2] / recRho[r];
            if (nU == 5) {
                H[r][3] = (recSys[r] == GPS) ? 1.0 : 0.0;
                H[r][4] = (recSys[r] == BDS) ? 1.0 : 0.0;
            }
            else {
                H[r][3] = 1.0;
                H[r][4] = 0.0;
            }

            double clkTerm = (nU == 5)
                ? H[r][3] * st[3] + H[r][4] * st[4]
                : H[r][3] * st[3];

            double dts_r = Epoch->SatPVT[recIdx[r]].SatClkOft;
            z[r] = recP[r] - recRho[r] + dts_r * C_Light
                - recTrop[r] - recTgd[r] - clkTerm;

            // sin²(E) 定权
            double s = sin(recElev[r]);
            w[r] = s * s;
        }

        // 加权最小二乘：N = BᵀPB（保留原矩阵，用于协因数阵求逆）
        double N[25] = { 0 };
        double b[5] = { 0 };
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < nU; j++) {
                b[j] += w[i] * H[i][j] * z[i];
                for (int k = 0; k < nU; k++)
                    N[j * nU + k] += w[i] * H[i][j] * H[i][k];
            }
        }

        // 用 N 的副本解方程：SolveN 是高斯-约当消元，会把传入矩阵就地
        // 变成单位阵，绝不能用它计算协因数阵
        double dxU[5];
        {
            double Nw[25];
            memcpy(Nw, N, sizeof(double) * nU * nU);
            for (int i = 0; i < nU; i++) dxU[i] = b[i];
            if (SolveN(Nw, dxU, nU) != 0) {
                Result->IsSuccess = false;
                return false;
            }
        }

        for (int i = 0; i < nU; i++) st[i] += dxU[i];

        double norm3 = sqrt(dxU[0] * dxU[0] + dxU[1] * dxU[1] + dxU[2] * dxU[2]);

        if (norm3 < 1e-4) {
            ok = true;

            // 协因数阵 Q = (BᵀPB)⁻¹：对 N 的副本逐列求单位向量解
            memset(Q, 0, sizeof(Q));
            for (int c = 0; c < nU; c++) {
                double Nw[25], e[5] = { 0.0, 0.0, 0.0, 0.0, 0.0 };
                memcpy(Nw, N, sizeof(double) * nU * nU);
                e[c] = 1.0;
                if (SolveN(Nw, e, nU) == 0) {
                    for (int r = 0; r < nU; r++) Q[r * nU + c] = e[r];
                }
            }

            // 精度评定：
            //   σ̂₀² = vᵀPv/(n−t)，D = σ̂₀²·Q，PDOP = √(qxx+qyy+qzz)
            double vtv = 0.0;
            for (int i = 0; i < n; i++) {
                // 残差 v = z − B·x̂（x̂ 为本轮改正数）
                double pred = H[i][0] * dxU[0] + H[i][1] * dxU[1] + H[i][2] * dxU[2]
                    + H[i][3] * dxU[3] + H[i][4] * dxU[4];
                double v = z[i] - pred;
                vtv += w[i] * v * v;
            }
            int dof = n - nU;
            double sigma0 = (dof > 0) ? sqrt(vtv / dof) : 0.0;

            Result->Sigma0 = sigma0;
            Result->Qxx = Q[0];
            Result->Qyy = Q[nU + 1];
            Result->Qzz = Q[2 * nU + 2];
            Result->mX = sigma0 * sqrt(fabs(Q[0]));
            Result->mY = sigma0 * sqrt(fabs(Q[nU + 1]));
            Result->mZ = sigma0 * sqrt(fabs(Q[2 * nU + 2]));
            Result->PDOP = sqrt(fabs(Q[0]) + fabs(Q[nU + 1]) + fabs(Q[2 * nU + 2]));
            Result->GPSSatNum = (short)usedGps;
            Result->BDSSatNum = (short)usedBds;
            break;
        }
    }

    for (int i = 0; i < 3; i++) Result->Position[i] = st[i];
    Result->RcvClkOft[0] = st[3];
    Result->RcvClkOft[1] = (nU == 5) ? st[4] : 0.0;
    Result->AllSatNum = (short)n;
    Result->IsSuccess = ok;
    return ok;
}
