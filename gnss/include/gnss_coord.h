#ifndef GNSS_COORD_H
#define GNSS_COORD_H

#include <cmath>
#include <cstdio>

// WGS84 椭球参数
#define WGS84_A 6378137.0           // 长半轴 (m)
#define WGS84_F 1.0 / 298.257223563 // 扁率
#define WGS84_E2 (2.0 * WGS84_F - WGS84_F * WGS84_F)  // 第一偏心率平方 e²

// BDS 椭球参数（CGCS2000）
#define CGCS2000_A 6378137.0
#define CGCS2000_F 1.0 / 298.257222101
#define CGCS2000_E2 (2.0 * CGCS2000_F - CGCS2000_F * CGCS2000_F)

/**
 * 空间直角坐标（地心地固系 ECEF）
 */
struct XYZ {
    double x, y, z;
    XYZ() : x(0), y(0), z(0) {}
    XYZ(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
};

/**
 * 大地坐标（经纬度单位为弧度，高程为椭球高/米）
 * 字段名与课件 1-4 第 19 页 struct BLh 一致
 */
struct BLH {
    double longitude;  // 经度（弧度）
    double latitude;   // 纬度（弧度）
    double height;     // 椭球高（m）
    BLH() : longitude(0), latitude(0), height(0) {}
    BLH(double lon_, double lat_, double h_) : longitude(lon_), latitude(lat_), height(h_) {}
};

// 课件 1-4 第 19 页写作 struct BLh，此处给出别名，两种写法均可使用
typedef BLH BLh;

/**
 * 站心坐标（北 dN、东 dE、天 dU）
 * 字段名与课件 1-4 第 20 页 struct NEU 一致
 */
struct NEU {
    double dN, dE, dU;
    NEU() : dN(0), dE(0), dU(0) {}
    NEU(double dN_, double dE_, double dU_) : dN(dN_), dE(dE_), dU(dU_) {}
};

/**
 * 大地坐标 → 空间直角坐标
 * @param a  椭球长半轴，默认 WGS84
 * @param e2 椭球第一偏心率平方，默认 WGS84
 */
void BLHToXYZ(const BLH& blh, XYZ& xyz, double a = WGS84_A, double e2 = WGS84_E2);

/**
 * 空间直角坐标 → 大地坐标（迭代法）
 * @return 0=成功，-1=坐标无效（x/y/z 均为 0）
 */
int XYZToBLH(const XYZ& xyz, BLH& blh, double a = WGS84_A, double e2 = WGS84_E2);

/**
 * 站心坐标系旋转矩阵（行优先存储：Row0=E，Row1=N，Row2=U）
 */
void BLHToNEUMat(const BLH& blh, double R[9]);

/**
 * 空间直角坐标差 → 站心坐标差（NEU）
 * @param dX 坐标差 [3]
 * @param R  旋转矩阵 [9]（由 BLHToNEUMat 计算）
 */
void XYZToNEU(const double dX[3], const double R[9], NEU& neu);

/**
 * 卫星高度角与方位角
 * @param elev 高度角（弧度）
 * @param azim 方位角（弧度，自北顺时针）
 */
void CompSatElAz(const double Xr[3], const double Xs[3],
                 const double R[9], double* elev, double* azim);

/**
 * 定位误差：解算坐标相对参考坐标的站心误差（N, E, U）
 * @param X0 参考坐标（真值）[3]
 * @param Xr 解算坐标 [3]
 */
void CompEnudPos(const double X0[3], const double Xr[3],
                const double R[9], double dNeu[3]);

/**
 * 打印 XYZ 坐标
 */
void PrintXYZ(const XYZ& xyz, const char* name);

/**
 * 打印 BLH 坐标（经纬度以度显示）
 */
void PrintBLH(const BLH& blh, const char* name);

#endif // GNSS_COORD_H
