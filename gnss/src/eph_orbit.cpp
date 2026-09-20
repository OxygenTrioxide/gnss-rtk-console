#include "eph_orbit.h"
#include <cmath>

// 给定发射时刻计算卫星位置、速度、钟差、钟速
// 位置与钟差公式：GPS ICD-200 / BDS ICD
// 速度与钟速公式：对位置/钟差公式解析求导
static void EphAtTx(const GPSEPHREC* eph, double t_tx,
    double pos[3], double vel[3], double* clk, double* clkrate)
{
    const double GM = (eph->Sys == GPS) ? GM_Earth : GM_BDS;
    const double omge = (eph->Sys == GPS) ? Omega_WGS : Omega_BDS;

    double a = eph->SqrtA * eph->SqrtA;
    double n0 = sqrt(GM / (a * a * a));
    double n = n0 + eph->DeltaN;

    // ---- 相对参考历元的时间 tk（±302400s 截断）----
    double tk = t_tx - eph->TOE.SecOfWeek;
    if (tk > 302400.0)  tk -= 604800.0;
    if (tk < -302400.0) tk += 604800.0;

    double dt_toc = t_tx - eph->TOC.SecOfWeek;
    if (dt_toc > 302400.0)  dt_toc -= 604800.0;
    if (dt_toc < -302400.0) dt_toc += 604800.0;

    // ---- 平近点角 / 开普勒方程 ----
    double Mk = eph->M0 + n * tk;
    double Ek = Mk;
    for (int i = 0; i < 15; i++) {
        double Ek_new = Mk + eph->e * sin(Ek);
        if (fabs(Ek_new - Ek) < 1e-13) { Ek = Ek_new; break; }
        Ek = Ek_new;
    }
    double sinE = sin(Ek), cosE = cos(Ek), e = eph->e;

    // ---- 真近点角 / 升交角距（atan2，避免象限错误）----
    double vk = atan2(sqrt(1.0 - e * e) * sinE, cosE - e);
    double phik = vk + eph->omega;
    double s2 = sin(2.0 * phik), c2 = cos(2.0 * phik);

    // ---- 二阶调和改正 ----
    double duk = eph->Cus * s2 + eph->Cuc * c2;
    double drk = eph->Crs * s2 + eph->Crc * c2;
    double dik = eph->Cis * s2 + eph->Cic * c2;

    double uk = phik + duk;
    double rk = a * (1.0 - e * cosE) + drk;
    double ik = eph->i0 + dik + eph->iDot * tk;

    // ---- 轨道平面位置 ----
    double xkp = rk * cos(uk);
    double ykp = rk * sin(uk);

    // ---- 升交点经度 ----
    // BDS GEO 卫星（i0 < 30°）：Ωk 中不含 -Ωe·tk 项（BDS ICD GEO 公式）
    bool isGEO = (eph->Sys == BDS && eph->i0 < 30.0 * Rad);
    double omge_eff = isGEO ? 0.0 : omge;
    double omgk_dot = eph->OMEGADot - omge_eff;
    double omk = eph->OMEGA + omgk_dot * tk - omge * eph->TOE.SecOfWeek;

    double co = cos(omk), so = sin(omk);
    double ci = cos(ik), si = sin(ik);

    pos[0] = xkp * co - ykp * ci * so;
    pos[1] = xkp * so + ykp * ci * co;
    pos[2] = ykp * si;

    // ---- 卫星速度（解析求导）----
    // 偏近点角导数 / 真近点角导数 / 升交角距导数
    double Ek_dot = n / (1.0 - e * cosE);
    double vk_dot = Ek_dot * sqrt(1.0 - e * e) / (1.0 - e * cosE);
    double phik_dot = vk_dot;

    // 改正数的导数
    double duk_dot = 2.0 * phik_dot * (eph->Cus * c2 - eph->Cuc * s2);
    double drk_dot = 2.0 * phik_dot * (eph->Crs * c2 - eph->Crc * s2);
    double dik_dot = 2.0 * phik_dot * (eph->Cis * c2 - eph->Cic * s2);

    double uk_dot = phik_dot + duk_dot;
    double rk_dot = a * e * sinE * Ek_dot + drk_dot;
    double ik_dot = eph->iDot + dik_dot;

    // 轨道平面速度
    double xkp_dot = rk_dot * cos(uk) - rk * uk_dot * sin(uk);
    double ykp_dot = rk_dot * sin(uk) + rk * uk_dot * cos(uk);

    // 地固系速度（旋转矩阵求导，含 i、Ω 的变化）
    vel[0] = xkp_dot * co - ykp_dot * ci * so
        + ykp * si * ik_dot * so
        - omgk_dot * (xkp * so + ykp * ci * co);
    vel[1] = xkp_dot * so + ykp_dot * ci * co
        - ykp * si * ik_dot * co
        + omgk_dot * (xkp * co - ykp * ci * so);
    vel[2] = ykp_dot * si + ykp * ci * ik_dot;

    // ---- BDS GEO 专用旋转（课件 1-6 第 23–24 页）----
    // 上面 GEO 已按"惯性系"公式 Ωk = Ω0 + Ω̇·tk − Ω̇e·toe 算出的是自定义坐标系
    // (GK) 中的坐标，必须再左乘 R_Z(Ω̇e·tk)·R_X(−5°) 才是 BDCS 地固坐标：
    //     [Xk Yk Zk]ᵀ = R_Z(Ω̇e·tk)·R_X(−5°)·[X_GK Y_GK Z_GK]ᵀ
    // 速度对应为：
    //     d[Xk Yk Zk]ᵀ/dt = R_Z(Ω̇e·tk)·R_X(−5°)·d[X_GK Y_GK Z_GK]ᵀ/dt
    //                     + Ṙ_Z(Ω̇e·tk)·R_X(−5°)·[X_GK Y_GK Z_GK]ᵀ
    if (isGEO) {
        const double c5 = cos(5.0 * Rad), s5 = sin(5.0 * Rad);   // R_X(−5°) 的 cos/sin
        const double ang = omge * tk;
        const double cz = cos(ang), sz = sin(ang);

        double px = pos[0], py = pos[1], pz = pos[2];
        double vx = vel[0], vy = vel[1], vz = vel[2];

        // R_X(−5°)：按课件 R_X(φ)=[[1,0,0],[0,+cosφ,+sinφ],[0,−sinφ,+cosφ]]
        double ux = px, uy = c5 * py - s5 * pz, uz = s5 * py + c5 * pz;
        double wx = vx, wy = c5 * vy - s5 * vz, wz = s5 * vy + c5 * vz;

        // R_Z(Ω̇e·tk)：按课件 R_Z(φ)=[[+cosφ,+sinφ,0],[−sinφ,+cosφ,0],[0,0,1]]
        pos[0] = ux * cz + uy * sz;
        pos[1] = -ux * sz + uy * cz;
        pos[2] = uz;

        // Ṙ_Z(Ω̇e·tk)·R_X(−5°)·X_GK，其中 Ṙ_Z = Ω̇e·[[−sin,cos,0],[−cos,−sin,0],[0,0,0]]
        vel[0] = wx * cz + wy * sz - omge * (sz * ux - cz * uy);
        vel[1] = -wx * sz + wy * cz - omge * (cz * ux + sz * uy);
        vel[2] = wz;
    }

    // ---- 卫星钟差（含相对论改正）与钟速 ----
    double F = -2.0 * sqrt(GM) / (C_Light * C_Light);
    double rel = F * e * eph->SqrtA * sinE;
    *clk = eph->ClkBias
        + eph->ClkDrift * dt_toc
        + eph->ClkDriftRate * dt_toc * dt_toc
        + rel;
    *clkrate = eph->ClkDrift
        + 2.0 * eph->ClkDriftRate * dt_toc
        + F * e * eph->SqrtA * cosE * Ek_dot;
}

void EphToECEF(const GPSEPHREC* eph, double t_tx, double pos[3], double vel[3],
    double* clk, double* clkrate)
{
    EphAtTx(eph, t_tx, pos, vel, clk, clkrate);
}

bool EphUsable(const GPSEPHREC* eph, double t_sow)
{
    if (!eph || !eph->Valid) return false;
    if (eph->SqrtA <= 0.0) return false;
    if (eph->SVHealth != 0) return false;   // 星历不健康

    // 星历过期判断：GPS 更新周期 2h，BDS 1h
    double dt = t_sow - eph->TOE.SecOfWeek;
    if (dt > 302400.0)  dt -= 604800.0;
    if (dt < -302400.0) dt += 604800.0;
    double limit = (eph->Sys == GPS) ? 7200.0 : 3600.0;
    return fabs(dt) <= limit;
}
