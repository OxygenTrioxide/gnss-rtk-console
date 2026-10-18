#define _CRT_SECURE_NO_WARNINGS
#include "lambda.h"
#include <cmath>
#include <vector>
#include <limits>
#include <cstdio>
#include <cstdlib>

using namespace Eigen;

// ============================================================
// LAMBDA 模糊度固定（课件 2-5《模糊度固定》）
//
// 课件流程（第 14 页）：
//   模糊度浮点解 fa、协方差阵 Qa
//   → Cholesky 分解得单位下三角 L 与对角 D
//   → 高斯消元降相关（整数变换）fz = Zᵀfa, Qz = ZᵀQaZ  ← 循环，含"条件方差排序"
//   → 整数最小二乘搜索，得 fz 的固定解与残差二次型 sqnorm
//   → 逆变换 a = z/Z
//   → 输出 ratio
//
// 降相关两个条件（课件第 12 页）：
//   ① 尽可能对角化：单位下三角阵元素 |l_ij| ≤ 0.5 (i>j)
//   ② 条件方差尽可能降序：d1 ≥ d2 ≥ … ≥ dn
//
// 模块接口（课件第 16 页）：
//   int lambda(int n, int m, const double *a, const double *Q, double *F, double *s)
//   F：长度 2n，[0,n) 最优整数模糊度，[n,2n) 次优；
//   s：长度 2，最优/次优解的残差二次型（ratio = s[1]/s[0]）
// 本实现按行优先读取 Q（n×n 对称阵，行列优先无差别）。
// ============================================================

// 上次降相关的诊断量（供 GNSS_LAMBDA_DEBUG 打印）
static double g_maxOffDiag = 0.0;   // 变换后 max|l_ij|
static int    g_dSorted = 0;        // 变换后条件方差是否降序
static int    g_zIter = 0;          // 降相关迭代次数
// 失败原因计数
static int    g_failLdl = 0;        // LDLᵀ 分解失败（协方差非正定）
static int    g_failChol = 0;       // 权重阵 Cholesky 分解失败
static int    g_failSearch = 0;     // 未搜索到整数候选
static int    g_dumpOn = 0;         // 是否转储首次调用输入（由命令行 --dumplambda 置位）
static int    g_debugOn = 0;        // 是否输出调试信息（由命令行 --lambdadebug 置位）
static long long g_nodes = 0;       // 搜索已访问的叶节点数
static long long g_maxNodes = 20000; // 叶节点上限（防止病态椭球下的搜索爆炸）
static int    g_capped = 0;         // 是否因达到上限而提前结束

void SetLambdaDump(int on) { g_dumpOn = on; }
void SetLambdaDebug(int on) { g_debugOn = on; }

void GetLambdaDiag(double* maxOffDiag, int* dSorted, int* zIter)
{
    if (maxOffDiag) *maxOffDiag = g_maxOffDiag;
    if (dSorted)    *dSorted = g_dSorted;
    if (zIter)      *zIter = g_zIter;
}

void GetLambdaFailStats(int* nLdl, int* nChol, int* nSearch)
{
    if (nLdl)    *nLdl = g_failLdl;
    if (nChol)   *nChol = g_failChol;
    if (nSearch) *nSearch = g_failSearch;
}

// ---- Cholesky 上三角：P = RᵀR ----
static bool CholeskyUpper(const MatrixXd& P, MatrixXd& R)
{
    LLT<MatrixXd> llt(P);
    if (llt.info() != Success) return false;
    R = llt.matrixL().transpose();
    double err = (R.transpose() * R - P).norm();
    if (!(err <= 1e-10 * std::max(1.0, P.norm()))) return false;
    return true;
}

// ---- LDLᵀ 分解：Q = L·D·Lᵀ（L 单位下三角，D 对角）----
// 模糊度协方差阵可能严重病态，主元经多次整数变换后会出现数值上的微小负值；
// 此处仅在"相对整体量级可忽略"时钳位，真正非正定仍判为失败。
static bool LdlDecompose(const MatrixXd& Q, MatrixXd& L, VectorXd& D)
{
    int n = (int)Q.rows();
    L = MatrixXd::Identity(n, n);
    D = VectorXd::Zero(n);
    double scale = Q.diagonal().cwiseAbs().maxCoeff();
    if (!(scale > 0.0)) return false;
    double floorVal = 1e-12 * scale;             // 主元下限（相对量级）
    for (int j = 0; j < n; j++) {
        double dj = Q(j, j);
        for (int k = 0; k < j; k++) dj -= L(j, k) * L(j, k) * D(k);
        if (dj <= 0.0) {
            if (dj > -1e-6 * scale) dj = floorVal;   // 舍入导致的微小负值 → 钳位
            else return false;                        // 真正非正定
        }
        D(j) = dj;
        for (int i = j + 1; i < n; i++) {
            double s = Q(i, j);
            for (int k = 0; k < j; k++) s -= L(i, k) * L(j, k) * D(k);
            L(i, j) = s / dj;
        }
    }
    return true;
}

// 对称化（整数变换后只存在舍入级的不对称）
static void Symmetrize(MatrixXd& M)
{
    M = 0.5 * (M + M.transpose());
}

// ---- 对"模糊度协方差阵"做 LAMBDA 整数变换（降相关，课件 2-5 第 12 页）----
//   输入 Q（原模糊度空间协方差），输出整数变换基 E 与变换后协方差
//   Qz = E⁻¹·Q·E⁻ᵀ（对应课件 z = Zᵀa、Qz = ZᵀQaZ 的等价形式，a = E·z）。
//   本实现直接作用在 Q 的 LDLᵀ 因子（Q = LᵀDL）上：
//     ① 整数高斯变换消除非对角相关：使 |l_ij| ≤ 0.5 (i>j)；
//     ② 条件方差排序：使 d1 ≥ d2 ≥ … ≥ dn。
//   每次改进后用 E 重算协方差，保证数值稳定。
static bool DecorrelateCov(const MatrixXd& Q, MatrixXd& E, MatrixXd& Qz, int* iterOut)
{
    int n = (int)Q.rows();
    E = MatrixXd::Identity(n, n);
    MatrixXd Cz = Q;
    Symmetrize(Cz);

    int iter = 0;
    for (iter = 0; iter < 200; iter++) {
        MatrixXd L; VectorXd D;
        if (!LdlDecompose(Cz, L, D)) return false;
        double scale = D.cwiseAbs().maxCoeff();
        bool improved = false;

        // ① 整数高斯变换：z_i ← z_i − round(l_ki)·z_k（k>i），使 |l_ki| ≤ 0.5
        for (int k = n - 1; k >= 1; k--) {
            for (int i = k - 1; i >= 0; i--) {
                double r = std::floor(L(k, i) + 0.5);   // 就近取整
                if (r == 0.0) continue;
                E.col(i) += r * E.col(k);               // a = E·z 下的整数列变换
                improved = true;
            }
        }
        // 变换后按定义重算协方差（E 已含全部历史整数变换）
        Cz = E.inverse() * Q * E.inverse().transpose();
        Symmetrize(Cz);

        // ② 条件方差排序：d1 ≥ d2 ≥ … ≥ dn，逐对相邻交换
        if (!LdlDecompose(Cz, L, D)) return false;
        for (int k = 0; k < n - 1; k++) {
            if (D(k) < D(k + 1) - 1e-12 * scale) {
                E.col(k).swap(E.col(k + 1));
                improved = true;
                Cz = E.inverse() * Q * E.inverse().transpose();
                Symmetrize(Cz);
                LdlDecompose(Cz, L, D);
            }
        }

        if (!improved) { Qz = Cz; if (iterOut) *iterOut = iter + 1; return true; }

        Cz = E.inverse() * Q * E.inverse().transpose();
        Symmetrize(Cz);
    }
    Qz = Cz;
    if (iterOut) *iterOut = iter;
    return true;
}

// ---- 整数最小二乘搜索（深度优先、椭球收缩）----
//   min ‖R(w − a)‖²，w ∈ Zⁿ，R 为上三角（RᵀR = Q⁻¹）
static void IlsSearch(const MatrixXd& R, const VectorXd& a, int lvl,
    double partial, double& bound, VectorXi& w,
    VectorXi& best, VectorXi& second, double& s1, double& s2)
{
    int n = (int)R.rows();
    if (g_capped) return;
    g_nodes++;
    if (g_nodes > g_maxNodes) { g_capped = 1; return; }
    if (lvl < 0) {
        if (partial < s1) {
            s2 = s1; second = best;
            s1 = partial; best = w;
        }
        else if (partial < s2 && w != best) {
            s2 = partial; second = w;
        }
        if (partial < bound) bound = partial;
        return;
    }

    double d_coef = 0.0;
    for (int j = lvl + 1; j < n; j++) {
        d_coef += R(lvl, j) * (w[j] - a[j]);
    }
    double center = a[lvl] - d_coef / R(lvl, lvl);
    double radius = std::sqrt(std::max(0.0, bound - partial)) / std::fabs(R(lvl, lvl)) + 1e-6;
    int lo = (int)std::ceil(center - radius);
    int hi = (int)std::floor(center + radius);
    if (lo > hi) return;

    int c0 = (int)std::llround(center);
    for (int d = 0; ; d++) {
        if (g_capped) break;                      // 达到搜索上限，立即停止枚举
        bool any = false;
        for (int sgn = 0; sgn < 2; sgn++) {
            if (d == 0 && sgn == 1) continue;
            int cand = (sgn == 0) ? c0 + d : c0 - d;
            if (cand < lo || cand > hi) continue;
            any = true;
            w[lvl] = cand;
            double t = R(lvl, lvl) * (cand - a[lvl]) + d_coef;
            IlsSearch(R, a, lvl - 1, partial + t * t, bound, w, best, second, s1, s2);
        }
        if (!any && d > 0) break;
    }
    w[lvl] = 0;
}

// ============================================================
// LAMBDA 核心：输入 ahat/Q（原模糊度空间），输出 m 个候选整数解及二次型
// ============================================================
static bool LambdaCore(const VectorXd& ahat, const MatrixXd& Q, int m,
    std::vector<double>& F, double s[2])
{
    int n = (int)ahat.size();
    F.assign((size_t)n * 2, 0.0);
    s[0] = s[1] = std::numeric_limits<double>::max();
    if (n <= 0) return false;

    // ---- 1. 整数变换（降相关，课件 2-5 第 10/12 页）----
    // 对模糊度协方差 Q 的 LDLᵀ 因子（Q = LᵀDL）做整数高斯变换与条件方差排序，
    // 得到整数变换基 E（a = E·z，z 为变换后的整数参数），使变换后协方差
    // Qz = E⁻¹·Q·E⁻ᵀ 满足：① |l_ij| ≤ 0.5 (i>j)；② 条件方差 d1 ≥ d2 ≥ … ≥ dn。
    MatrixXd Qinv = Q.inverse();
    MatrixXd E, Qz;
    int iter = 0;
    if (!DecorrelateCov(Q, E, Qz, &iter)) { g_failChol++; return false; }

    // 诊断量：变换后单位下三角阵 |l_ij| 最大值与条件方差是否降序
    {
        MatrixXd Ld; VectorXd Dd;
        if (LdlDecompose(Qz, Ld, Dd)) {
            g_maxOffDiag = 0.0;
            for (int ii = 0; ii < n; ii++)
                for (int jj = 0; jj < ii; jj++)
                    g_maxOffDiag = std::max(g_maxOffDiag, std::fabs(Ld(ii, jj)));
            g_dSorted = 1;
            for (int kk = 1; kk < n; kk++)
                if (Dd(kk) > Dd(kk - 1) + 1e-12 * Dd(0)) g_dSorted = 0;
        }
    }
    g_zIter = iter;

    // ---- 2. 搜索空间权重阵 R（RᵀR = EᵀQ⁻¹E）----
    MatrixXd P = E.transpose() * Qinv * E;
    MatrixXd R;
    if (!CholeskyUpper(P, R)) { g_failChol++; return false; }
    MatrixXd Ei = E.inverse();
    VectorXd zhat = Ei * ahat;                 // ẑ = E⁻¹â
    // ---- 3. 整数最小二乘搜索（椭球初值取 Bootstrap 解）----
    VectorXi wb = zhat.array().round().cast<int>();
    double s0 = (R * (wb.cast<double>() - zhat)).squaredNorm();
    // 椭球半径取 Bootstrap 解代价并留相对裕量：降相关后 z̃ 往往已非常接近整数，
    // s0 会小到接近 0，此时过紧的界会因浮点舍入把真正的最优整数解剪掉，
    // 表现为"搜索不到任何整数候选"。留 1e-6 相对裕量与 1e-9 绝对裕量即可。
    double bound = s0 * (1.0 + 1e-6) + 1e-9;

    VectorXi w = VectorXi::Zero(n), best = VectorXi::Zero(n), second = VectorXi::Zero(n);
    double s1 = std::numeric_limits<double>::max();
    double s2 = std::numeric_limits<double>::max();
    g_nodes = 0; g_capped = 0;
    IlsSearch(R, zhat, n - 1, 0.0, bound, w, best, second, s1, s2);

    if (s1 == std::numeric_limits<double>::max()) {
        if (g_capped) {                 // 搜索被截断：退回 Bootstrap 解（代价 s0，上界）
            best = wb; s1 = s0;
        }
        else { g_failSearch++; return false; }
    }

    // ---- 4. 逆变换 a = Z⁻ᵀw（回到原模糊度空间）----
    VectorXd a1 = E * best.cast<double>();
    for (int i = 0; i < n; i++) F[i] = std::round(a1(i));
    s[0] = s1;
    if (m >= 2 && s2 < std::numeric_limits<double>::max()) {
        VectorXd a2 = E * second.cast<double>();
        for (int i = 0; i < n; i++) F[n + i] = std::round(a2(i));
        s[1] = s2;
    }
    return true;
}

// ============================================================
// 课件 2-5 第 16 页要求的模块接口
//   int lambda(int n, int m, const double *a, const double *Q, double *F, double *s)
//   返回 0 = 成功；-1 = 输入非法/分解失败；-2 = 未搜索到整数候选
// ============================================================
int lambda(int n, int m, const double* a, const double* Q, double* F, double* s)
{
    if (!a || !Q || !F || !s || n <= 0) return -1;
    VectorXd ahat(n);
    MatrixXd Qm(n, n);
    for (int i = 0; i < n; i++) {
        ahat(i) = a[i];
        for (int j = 0; j < n; j++) Qm(i, j) = Q[(size_t)i * n + j];
    }

    // 首次调用时转储真实输入（环境变量 GNSS_LAMBDA_DUMP 开启），
    // 供 LambdaSelfTestFile 离线复现与定位
    static int dumped = 0;
    if (!dumped && g_dumpOn) {
        dumped = 1;
        FILE* fp = fopen("lambda_input.txt", "w");
        if (fp) {
            fprintf(fp, "n %d\na", n);
            for (int i = 0; i < n; i++) fprintf(fp, " %.10f", a[i]);
            fprintf(fp, "\n");
            for (int i = 0; i < n; i++) {
                fprintf(fp, "Q");
                for (int j = 0; j < n; j++) fprintf(fp, " %.12e", Q[(size_t)i * n + j]);
                fprintf(fp, "\n");
            }
            fclose(fp);
            printf("  [LAMBDA] 已转储首次调用输入到 lambda_input.txt (n=%d)\n", n);
        }
    }

    std::vector<double> Fv;
    double sv[2];
    if (!LambdaCore(ahat, Qm, m, Fv, sv)) {
        if (g_debugOn)
            fprintf(stderr, "[LAMBDA-FAIL] n=%d LDLT=%d Chol=%d 搜索=%d\n",
                n, g_failLdl, g_failChol, g_failSearch);
        return -2;
    }
    for (int i = 0; i < 2 * n; i++) F[i] = Fv[i];
    s[0] = sv[0];
    s[1] = sv[1];
    if (g_debugOn) {
        fprintf(stderr, "[LAMBDA] n=%d m=%d zIter=%d max|l_ij|=%.4f dSorted=%d s0=%.4g s1=%.4g\n",
            n, m, g_zIter, g_maxOffDiag, g_dSorted, sv[0], sv[1]);
    }
    return 0;
}

// ------------------------------------------------------------
// 离线自检：读取 lambda_input.txt（由 GNSS_LAMBDA_DUMP 转储的真实输入），
// 调用课件模块 lambda() 并输出返回值、降相关达成情况与失败原因，
// 便于不跑全量数据即可定位模糊度固定问题。
// 返回 0 = 成功
// ------------------------------------------------------------
int LambdaSelfTestFile(const char* fname)
{
    FILE* fp = fopen(fname, "rt");
    if (!fp) { printf("无法打开 %s\n", fname); return -1; }

    int n = 0;
    char tag[8];
    if (fscanf(fp, "%7s %d", tag, &n) != 2 || n <= 0 || n > 256) {
        fclose(fp); printf("文件格式错误\n"); return -1;
    }
    std::vector<double> a(n, 0.0), Q((size_t)n * n, 0.0);
    if (fscanf(fp, "%7s", tag) != 1) { fclose(fp); return -1; }
    for (int i = 0; i < n; i++) if (fscanf(fp, "%lf", &a[i]) != 1) { fclose(fp); return -1; }
    for (int i = 0; i < n; i++) {
        if (fscanf(fp, "%7s", tag) != 1) { fclose(fp); return -1; }
        for (int j = 0; j < n; j++)
            if (fscanf(fp, "%lf", &Q[(size_t)i * n + j]) != 1) { fclose(fp); return -1; }
    }
    fclose(fp);

    printf("=== LAMBDA 自检===\n");
    printf("  模糊度维数 n = %d\n", n);
    std::vector<double> F((size_t)n * 2, 0.0);
    double s[2] = { 0.0, 0.0 };
    int rc = lambda(n, 2, a.data(), Q.data(), F.data(), s);
    printf("  lambda() 返回码 = %d （0=成功, -2=未搜索到候选）\n", rc);
    printf("  失败计数: LDLT=%d Cholesky=%d 搜索=%d\n", g_failLdl, g_failChol, g_failSearch);
    printf("  搜索: 叶节点=%lld 上限=%lld 是否截断=%d\n", g_nodes, g_maxNodes, g_capped);
    printf("  降相关: 迭代=%d max|l_ij|=%.4f(条件①≤0.5) 条件方差降序=%d(条件②=1)\n",
        g_zIter, g_maxOffDiag, g_dSorted);
    if (rc == 0) {
        printf("  最优整数模糊度:");
        for (int i = 0; i < n; i++) printf(" %.0f", F[i]);
        printf("\n  浮点模糊度   :");
        for (int i = 0; i < n; i++) printf(" %.3f", a[i]);
        printf("\n  s0=%.6g s1=%.6g ratio=s1/s0=%.3f\n", s[0], s[1],
            (s[0] > 0.0 && s[1] < 1e300) ? s[1] / s[0] : 0.0);
    }
    return rc;
}

// ============================================================
// C++ 接口：返回最优整数解与 ratio = s1/s0（课件 2-5 第 17 页）
// ============================================================
bool LambdaFix(const VectorXd& ahat, const MatrixXd& Q,
    VectorXi& afix, double& ratio)
{
    int n = (int)ahat.size();
    ratio = 0.0;
    if (n <= 0) return false;
    std::vector<double> Fv;
    double sv[2];
    if (!LambdaCore(ahat, Q, 2, Fv, sv)) return false;

    afix = VectorXi(n);
    for (int i = 0; i < n; i++) afix(i) = (int)std::llround(Fv[i]);

    if (sv[1] >= std::numeric_limits<double>::max() || sv[0] <= 0.0)
        ratio = std::numeric_limits<double>::infinity();
    else
        ratio = sv[1] / sv[0];      // 课件 2-5 第 17 页：σ²_次最小 / σ²_最小

    if (g_debugOn) {
        fprintf(stderr, "[LAMBDA] n=%d zIter=%d max|l_ij|=%.4f dSorted=%d s0=%.4g s1=%.4g ratio=%.3f\n",
            n, g_zIter, g_maxOffDiag, g_dSorted, sv[0], sv[1], ratio);
    }
    return true;
}
