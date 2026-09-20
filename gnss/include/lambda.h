#ifndef LAMBDA_H
#define LAMBDA_H

#include <Eigen/Dense>

// ============================================================
// LAMBDA 模糊度固定（课件 2-5《模糊度固定》）
// ============================================================

// 课件 2-5 第 16 页要求的模块接口：
//   int lambda(int n, int m, const double *a, const double *Q, double *F, double *s)
//   n : 模糊度个数
//   m : 需要输出的候选解个数（常用 2 = 最优 + 次优）
//   a : 浮点模糊度（长度 n，单位：周）
//   Q : 浮点模糊度协方差阵（n×n 行优先，单位：周²）
//   F : 输出，长度 2n；F[0,n) 最优整数模糊度，F[n,2n) 次优（单位：周）
//   s : 输出，长度 2；s[0]/s[1] 为最优/次优解的残差二次型，ratio = s[1]/s[0]
//   返回 0 = 成功；-1 = 输入非法或分解失败；-2 = 未搜索到整数候选
int lambda(int n, int m, const double* a, const double* Q, double* F, double* s);

// C++ 便捷接口（内部同样走课件流程：整数变换降相关 + 条件方差排序 + 搜索）
//   ahat : 浮点模糊度向量（周）
//   Q    : 浮点模糊度协方差阵（周²，对称正定）
//   afix : 固定后的整数模糊度（周）
//   ratio: ≥1，次优解/最优解的残差二次型之比 s2/s1（课件 2-5 第 17 页
//          ratio = σ²_次最小/σ²_最小，两者自由度相同）
//   返回 true 表示搜索完成；是否采用由调用方按 ratio 阈值判定
bool LambdaFix(const Eigen::VectorXd& ahat, const Eigen::MatrixXd& Q,
    Eigen::VectorXi& afix, double& ratio);

// 诊断接口：最近一次调用的降相关达成情况
//   maxOffDiag：变换后单位下三角阵元素绝对值最大值（条件①要求 ≤ 0.5）
//   dSorted  ：条件方差是否降序（条件②，1=是）
//   zIter    ：整数变换迭代次数
void GetLambdaDiag(double* maxOffDiag, int* dSorted, int* zIter);

// 诊断接口：失败原因计数
//   nLdl：LDLᵀ 分解失败次数；nChol：Cholesky 分解失败次数；nSearch：未搜索到整数候选次数
void GetLambdaFailStats(int* nLdl, int* nChol, int* nSearch);

// 离线自检：读取 lambda_input.txt 并调用 lambda()，输出降相关达成情况与失败原因
int LambdaSelfTestFile(const char* fname);

// 调试开关（由命令行参数置位；本环境命令行参数比环境变量可靠）
void SetLambdaDump(int on);
void SetLambdaDebug(int on);

#endif // LAMBDA_H
