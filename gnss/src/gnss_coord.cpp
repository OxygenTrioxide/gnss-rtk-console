#include "gnss_coord.h"

#define PI 3.141592653589793
#define DEG2RAD (PI / 180.0)
#define RAD2DEG (180.0 / PI)
#define MAX_ITER 20
#define CONV_THRESH 1e-12

void BLHToXYZ(const BLH& blh, XYZ& xyz, double a, double e2) {
    double cosLat = cos(blh.latitude);
    double sinLat = sin(blh.latitude);
    double cosLon = cos(blh.longitude);
    double sinLon = sin(blh.longitude);
    
    // 卯酉圈曲率半径 N = a / sqrt(1 - e^2 * sin^2(B))
    double N = a / sqrt(1.0 - e2 * sinLat * sinLat);
    
    xyz.x = (N + blh.height) * cosLat * cosLon;
    xyz.y = (N + blh.height) * cosLat * sinLon;
    xyz.z = (N * (1.0 - e2) + blh.height) * sinLat;
}

int XYZToBLH(const XYZ& xyz, BLH& blh, double a, double e2) {
    // 检查坐标是否有效 (非零)
    double r = sqrt(xyz.x * xyz.x + xyz.y * xyz.y);
    if (r < 1e-10 && fabs(xyz.z) < 1e-10) {
        blh.latitude = 0;
        blh.longitude = 0;
        blh.height = 0;
        return -1;  // 坐标为零
    }
    
    // 经度直接计算
    blh.longitude = atan2(xyz.y, xyz.x);
    
    // 纬度迭代计算
    double lat_prev = atan2(xyz.z, r);  // 初始值
    double N = a;
    double h = 0;
    
    for (int i = 0; i < MAX_ITER; i++) {
        double sinLat = sin(lat_prev);
        N = a / sqrt(1.0 - e2 * sinLat * sinLat);
        double lat_new = atan2(xyz.z + N * e2 * sinLat, r);
        
        // 检查收敛
        if (fabs(lat_new - lat_prev) < CONV_THRESH) {
            blh.latitude = lat_new;
            blh.height = r / cos(lat_new) - N;
            return 0;
        }
        lat_prev = lat_new;
    }
    
    // 未收敛，返回最后一次迭代结果
    blh.latitude = lat_prev;
    blh.height = r / cos(lat_prev) - N;
    return -1;
}

void BLHToNEUMat(const BLH& blh, double R[9]) {
    double sinLat = sin(blh.latitude);
    double cosLat = cos(blh.latitude);
    double sinLon = sin(blh.longitude);
    double cosLon = cos(blh.longitude);
    
    // 旋转矩阵 (行优先存储)
    // Row0: E方向 (东)
    R[0] = -sinLon;
    R[1] = cosLon;
    R[2] = 0;
    
    // Row1: N方向 (北)
    R[3] = -sinLat * cosLon;
    R[4] = -sinLat * sinLon;
    R[5] = cosLat;
    
    // Row2: U方向 (天)
    R[6] = cosLat * cosLon;
    R[7] = cosLat * sinLon;
    R[8] = sinLat;
}

void XYZToNEU(const double dX[3], const double R[9], NEU& neu) {
    // dX 是空间直角坐标差 (X_sat - X_ref)
    // 旋转到站心系: [E, N, U]^T = R * dX
    neu.dE = R[0] * dX[0] + R[1] * dX[1] + R[2] * dX[2];
    neu.dN = R[3] * dX[0] + R[4] * dX[1] + R[5] * dX[2];
    neu.dU = R[6] * dX[0] + R[7] * dX[1] + R[8] * dX[2];
}

void CompSatElAz(const double Xr[3], const double Xs[3], 
                 const double R[9], double* elev, double* azim) {
    // 计算站星空间直角坐标差
    double dX[3] = {
        Xs[0] - Xr[0],
        Xs[1] - Xr[1],
        Xs[2] - Xr[2]
    };
    
    // 转换到站心系
    NEU neu;
    XYZToNEU(dX, R, neu);
    
    // 计算高度角: elev = atan2(U, sqrt(E^2 + N^2))
    double horiz = sqrt(neu.dN * neu.dN + neu.dE * neu.dE);
    *elev = atan2(neu.dU, horiz);
    
    // 计算方位角: azim = atan2(E, N) (从北顺时针)
    *azim = atan2(neu.dE, neu.dN);
    // 保证方位角在 [0, 2*PI) 范围内
    if (*azim < 0) *azim += 2.0 * PI;
}

void CompEnudPos(const double X0[3], const double Xr[3], 
                const double R[9], double dNeu[3]) {
    double dX[3] = {
        Xr[0] - X0[0],
        Xr[1] - X0[1],
        Xr[2] - X0[2]
    };
    
    // 旋转到站心系
    dNeu[0] = R[0] * dX[0] + R[1] * dX[1] + R[2] * dX[2];  // E
    dNeu[1] = R[3] * dX[0] + R[4] * dX[1] + R[5] * dX[2];  // N
    dNeu[2] = R[6] * dX[0] + R[7] * dX[1] + R[8] * dX[2];  // U
}

void PrintXYZ(const XYZ& xyz, const char* name) {
    printf("%s: X=%.3f, Y=%.3f, Z=%.3f\n", name, xyz.x, xyz.y, xyz.z);
}

void PrintBLH(const BLH& blh, const char* name) {
    printf("%s: lon=%.9f° (%.6f rad), lat=%.9f° (%.6f rad), h=%.3f m\n", 
           name, blh.longitude * RAD2DEG, blh.longitude, 
           blh.latitude * RAD2DEG, blh.latitude, blh.height);
}