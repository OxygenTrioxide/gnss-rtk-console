#define _CRT_SECURE_NO_WARNINGS
#include "spv.h"
#include "gnss_coord.h"
#include <cmath>
#include <algorithm>

// --------- 高斯消元 ---------
static int SolveVelocity(double* A, double* b, int n)
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

// ============================================================
// SPV 单点测速
//   卫星位置/速度/钟差/钟速直接复用 SPP 计算的 SatPVT 中间结果
//   （自转改正后的位置与速度，时系一致）
//   未知数: Vrx, Vry, Vrz, c·dtr （GPS/BDS 共用一个钟漂）
// ============================================================
bool SPV(EPOCHOBSDATA* Epoch, RAWDAT* Raw, POSRES* Result)
{
    if (!Epoch || !Raw || !Result) return false;

    // 接收机位置：优先用 SPP 解算结果，否则用 Epoch->Pos
    double Xr[3];
    if (Result->Position[0] != 0.0 || Result->Position[1] != 0.0 || Result->Position[2] != 0.0) {
        Xr[0] = Result->Position[0];
        Xr[1] = Result->Position[1];
        Xr[2] = Result->Position[2];
    }
    else {
        Xr[0] = Epoch->Pos[0];
        Xr[1] = Epoch->Pos[1];
        Xr[2] = Epoch->Pos[2];
    }

    const int MAXEQ = MAXCHANNUM;
    double H[MAXEQ][4], z[MAXEQ], w[MAXEQ];
    int n = 0;

    for (int i = 0; i < Epoch->SatNum && i < MAXCHANNUM; i++)
    {
        SATOBSDATA& so = Epoch->SatObs[i];
        if (!so.Valid) continue;

        double dop = so.D[0];                 // 多普勒观测（Hz）
        if (dop == 0.0) continue;

        // 卫星 PVT 中间结果（SPP 计算，自转改正后）
        SATPVT& mid = Epoch->SatPVT[i];
        if (!mid.Valid) continue;

        double sv[3] = { mid.SatPos[0], mid.SatPos[1], mid.SatPos[2] };
        double Vs[3] = { mid.SatVel[0], mid.SatVel[1], mid.SatVel[2] };
        double dtsd = mid.SatClkSft;          // 卫星钟漂（s/s，含相对论导数）

        double dx = sv[0] - Xr[0], dy = sv[1] - Xr[1], dz = sv[2] - Xr[2];
        double rho = sqrt(dx * dx + dy * dy + dz * dz);
        if (rho < 1e3) continue;
        double ex = dx / rho, ey = dy / rho, ez = dz / rho;

        // ---- 多普勒 → 距离变化率 (m/s) ----
        // 解码时多普勒已按课件 1-5 第 35 页乘以 -1（与相位一致），
        // 故此处 ρ̇ = λ·D 即可得到 RINEX 约定（接近卫星为正）的距离变化率。
        double f = (so.System == GPS) ? FG1_GPS : FG1_BDS;
        double lam = C_Light / f;
        double rr = lam * dop;

        // ---- 观测方程： ρ̇ = e·(Vs - Vr) + c·(dtsd - dtr)
        //      z = ρ̇ - e·Vs + c·dtsd = -e·Vr + c·dtr
        z[n] = rr - (ex * Vs[0] + ey * Vs[1] + ez * Vs[2]) + dtsd * C_Light;

        H[n][0] = -ex;
        H[n][1] = -ey;
        H[n][2] = -ez;
        H[n][3] = 1.0;                        // c·dtr (m/s)

        // sin²(E) 定权（与 SPP 一致）
        double s = sin(mid.Elevation > 0.01 ? mid.Elevation : 0.01);
        w[n] = s * s;
        n++;
    }

    if (n < 4) {
        Result->IsSuccess = false;
        return false;
    }

    // ---- 法方程 ----
    double N[16] = { 0 }, b[4] = { 0 };
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < 4; j++) {
            b[j] += w[i] * H[i][j] * z[i];
            for (int k = 0; k < 4; k++)
                N[j * 4 + k] += w[i] * H[i][j] * H[i][k];
        }
    }

    double sol[4] = { b[0], b[1], b[2], b[3] };
    if (SolveVelocity(N, sol, 4) != 0) {
        Result->IsSuccess = false;
        return false;
    }

    // ---- 速度精度评定：σ̂ = √(vᵀPv/(n-4)) ----
    double vtv = 0.0;
    for (int i = 0; i < n; i++) {
        double v = z[i] - (H[i][0] * sol[0] + H[i][1] * sol[1] +
            H[i][2] * sol[2] + H[i][3] * sol[3]);
        vtv += w[i] * v * v;
    }
    int dof = n - 4;
    Result->SigmaVel = (dof > 0) ? sqrt(vtv / dof) : 0.0;

    // ---- 存结果（ECEF 速度分量）----
    Result->Velocity[0] = sol[0];
    Result->Velocity[1] = sol[1];
    Result->Velocity[2] = sol[2];
    Result->RcvClkSft = sol[3] / C_Light;   // 接收机钟漂 (s/s)
    Result->AllSatNum = (short)n;
    Result->IsSuccess = true;
    return true;
}
