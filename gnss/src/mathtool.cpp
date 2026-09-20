#include "mathtool.h"

void VecAdd(const double a[3], const double b[3], double result[3]) {
    for (int i = 0; i < 3; i++) {
        result[i] = a[i] + b[i];
    }
}

void VecSub(const double a[3], const double b[3], double result[3]) {
    for (int i = 0; i < 3; i++) {
        result[i] = a[i] - b[i];
    }
}

double VecDot(const double a[3], const double b[3]) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void VecCross(const double a[3], const double b[3], double result[3]) {
    result[0] = a[1] * b[2] - a[2] * b[1];
    result[1] = a[2] * b[0] - a[0] * b[2];
    result[2] = a[0] * b[1] - a[1] * b[0];
}

void VecScale(const double a[3], double scalar, double result[3]) {
    for (int i = 0; i < 3; i++) {
        result[i] = scalar * a[i];
    }
}

double VecNorm(const double a[3]) {
    return sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
}

void MatAdd(const double* A, const double* B, int rows, int cols, double* C) {
    int size = rows * cols;
    for (int i = 0; i < size; i++) {
        C[i] = A[i] + B[i];
    }
}

void MatSub(const double* A, const double* B, int rows, int cols, double* C) {
    int size = rows * cols;
    for (int i = 0; i < size; i++) {
        C[i] = A[i] - B[i];
    }
}

void MatMul(const double* A, const double* B, int m, int n, int p, double* C) {
    // 初始化C为0
    for (int i = 0; i < m * p; i++) {
        C[i] = 0.0;
    }
    
    // 矩阵乘法: C[i][j] = sum_k A[i][k] * B[k][j]
    for (int i = 0; i < m; i++) {
        for (int k = 0; k < n; k++) {
            double aik = A[i * n + k];
            if (fabs(aik) < 1e-30) continue;  // 跳过零值提高效率
            for (int j = 0; j < p; j++) {
                C[i * p + j] += aik * B[k * p + j];
            }
        }
    }
}

void MatTranspose(const double* A, int rows, int cols, double* B) {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            B[j * rows + i] = A[i * cols + j];
        }
    }
}

int MatInverse(const double* A, int n, double* invA) {
    // 创建增广矩阵 [A | I]
    int size = n * n;
    double* aug = new double[n * 2 * n];
    if (!aug) return -1;
    
    // 初始化增广矩阵
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            aug[i * 2 * n + j] = A[i * n + j];
            aug[i * 2 * n + j + n] = (i == j) ? 1.0 : 0.0;
        }
    }
    
    // 高斯-约旦消元
    for (int col = 0; col < n; col++) {
        // 寻找主元（列主元）
        int maxRow = col;
        double maxVal = fabs(aug[col * 2 * n + col]);
        for (int row = col + 1; row < n; row++) {
            double val = fabs(aug[row * 2 * n + col]);
            if (val > maxVal) {
                maxVal = val;
                maxRow = row;
            }
        }
        
        // 检查矩阵是否奇异
        if (maxVal < 1e-30) {
            delete[] aug;
            return -1;
        }
        
        // 交换行
        if (maxRow != col) {
            for (int j = 0; j < 2 * n; j++) {
                double temp = aug[col * 2 * n + j];
                aug[col * 2 * n + j] = aug[maxRow * 2 * n + j];
                aug[maxRow * 2 * n + j] = temp;
            }
        }
        
        // 主元归一化
        double pivot = aug[col * 2 * n + col];
        for (int j = 0; j < 2 * n; j++) {
            aug[col * 2 * n + j] /= pivot;
        }
        
        // 消去其他行
        for (int row = 0; row < n; row++) {
            if (row == col) continue;
            double factor = aug[row * 2 * n + col];
            for (int j = 0; j < 2 * n; j++) {
                aug[row * 2 * n + j] -= factor * aug[col * 2 * n + j];
            }
        }
    }
    
    // 提取逆矩阵
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            invA[i * n + j] = aug[i * 2 * n + j + n];
        }
    }
    
    delete[] aug;
    return 0;
}

void MatPrint(const double* A, int rows, int cols, const char* name) {
    printf("\n%s [%d x %d]:\n", name, rows, cols);
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            printf("%12.6f ", A[i * cols + j]);
        }
        printf("\n");
    }
}

void MatCopy(const double* src, int rows, int cols, double* dst) {
    memcpy(dst, src, rows * cols * sizeof(double));
}

void MatIdentity(double* I, int n) {
    for (int i = 0; i < n * n; i++) {
        I[i] = 0.0;
    }
    for (int i = 0; i < n; i++) {
        I[i * n + i] = 1.0;
    }
}