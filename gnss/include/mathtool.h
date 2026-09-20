#ifndef MATHTOOL_H
#define MATHTOOL_H

#include <cmath>
#include <cstring>
#include <cstdio>

/**
 * 三维向量加法：result = a + b
 */
void VecAdd(const double a[3], const double b[3], double result[3]);

/**
 * 三维向量减法：result = a - b
 */
void VecSub(const double a[3], const double b[3], double result[3]);

/**
 * 三维向量点积
 */
double VecDot(const double a[3], const double b[3]);

/**
 * 三维向量叉积：result = a × b
 */
void VecCross(const double a[3], const double b[3], double result[3]);

/**
 * 向量数乘：result = scalar · a
 */
void VecScale(const double a[3], double scalar, double result[3]);

/**
 * 向量模长
 */
double VecNorm(const double a[3]);

/**
 * 矩阵加法：C = A + B（rows×cols）
 */
void MatAdd(const double* A, const double* B, int rows, int cols, double* C);

/**
 * 矩阵减法：C = A - B（rows×cols）
 */
void MatSub(const double* A, const double* B, int rows, int cols, double* C);

/**
 * 矩阵乘法：C = A · B（A 为 m×n，B 为 n×p）
 */
void MatMul(const double* A, const double* B, int m, int n, int p, double* C);

/**
 * 矩阵转置：B = Aᵀ（A 为 rows×cols）
 */
void MatTranspose(const double* A, int rows, int cols, double* B);

/**
 * 矩阵求逆（高斯-约旦消元法），n 阶方阵
 * @return 0=成功，-1=矩阵奇异
 */
int MatInverse(const double* A, int n, double* invA);

/**
 * 打印矩阵（调试用）
 */
void MatPrint(const double* A, int rows, int cols, const char* name);

/**
 * 矩阵拷贝
 */
void MatCopy(const double* src, int rows, int cols, double* dst);

/**
 * 创建 n 阶单位矩阵
 */
void MatIdentity(double* I, int n);

#endif // MATHTOOL_H
