#define _CRT_SECURE_NO_WARNINGS
#include "oem7_ephem.h"
#include <cstring>
#include <cstdio>
#include <cmath>

// OEM7 二进制消息字段读取（小端，与 memcpy 对应）
static double R8(unsigned char* p) { double r; memcpy(&r, p, 8); return r; }
static float  R4(unsigned char* p) { float  r; memcpy(&r, p, 4); return r; }
static int    I4(unsigned char* p) { int    r; memcpy(&r, p, 4); return r; }
static unsigned int   UI4(unsigned char* p) { unsigned int   r; memcpy(&r, p, 4); return r; }
static short          I2(unsigned char* p) { short          r; memcpy(&r, p, 2); return r; }
static unsigned short UI2(unsigned char* p) { unsigned short r; memcpy(&r, p, 2); return r; }

// GPS 广播星历解码（OEM7 消息 ID=7）
int decode_gpsephem(unsigned char* buff, GPSEPHREC* geph)
{
    int pos = 28;
    int prn = I4(buff + pos + 0);
    if (prn < 1 || prn > MAXGPSNUM) return 0;
    GPSEPHREC* eph = geph + prn - 1;

    unsigned short wk = UI2(buff + 14);

    eph->PRN = (unsigned short)prn;
    eph->Sys = GPS;
    eph->SVHealth = (short)UI4(buff + pos + 12);       //卫星健康状态
    eph->IODE = (double)UI4(buff + pos + 16);          //星历数据龄期
    eph->IODC = (double)UI4(buff + pos + 160);         //时钟数据龄期
    eph->TOC.Week = wk;  eph->TOE.Week = wk;           //设置周数
    eph->TOC.SecOfWeek = R8(buff + pos + 164);         //时钟参考时刻
    eph->TOE.SecOfWeek = R8(buff + pos + 32);          //星历参考时刻
    eph->TGD1 = R8(buff + pos + 172);                  //群波延迟改正参数
    eph->TGD2 = 0.0;                                   //L2频点TGD，此处设为0
    eph->ClkBias = R8(buff + pos + 180);               //卫星钟差
    eph->ClkDrift = R8(buff + pos + 188);              //卫星钟漂
    eph->ClkDriftRate = R8(buff + pos + 196);          //频率漂移率
    eph->Crs = R8(buff + pos + 104);                   //轨道半径正弦调和改正参数
    eph->DeltaN = R8(buff + pos + 48);                 //平均角速度修正值
    eph->M0 = R8(buff + pos + 56);                     //参考时刻的平近点角
    eph->Cuc = R8(buff + pos + 80);                    //升交角距余弦调和改正幅值
    eph->e = R8(buff + pos + 64);                      //轨道偏心率
    eph->Cus = R8(buff + pos + 88);                    //升交角距正弦调和改正幅值
    double A = R8(buff + pos + 40);                    //长半轴
    eph->SqrtA = (A > 0) ? sqrt(A) : 0.0;
    eph->Cic = R8(buff + pos + 112);                   //轨道倾角余弦调和改正幅值
    eph->OMEGA = R8(buff + pos + 144);                 //参考时刻的升交点赤经(Ω0)
    eph->Cis = R8(buff + pos + 120);                   //轨道倾角正弦调和改正幅值
    eph->i0 = R8(buff + pos + 128);                    //参考时刻的轨道倾角
    eph->Crc = R8(buff + pos + 96);                    //轨道半径余弦调和改正振幅
    eph->omega = R8(buff + pos + 72);                  //近地点角距(ω)
    eph->OMEGADot = R8(buff + pos + 152);              //升交点赤经变化率
    eph->iDot = R8(buff + pos + 136);                  //轨道倾角变化率
    eph->SVAccuracy = 0.0;                             //卫星精度(暂设为0)
    eph->Valid = true;

    return 2;
}

// BDS 广播星历解码（OEM7 消息 ID=1696）
// 注意：TOE/TOC 保持 BDT（北斗时）周内秒，BDS 星历时系为 BDT，不做 +14s 换算
int decode_bdsephem(unsigned char* buff, GPSEPHREC* beph)
{
    int pos = 28;
    int prn = I4(buff + pos + 0);
    if (prn < 1 || prn > MAXBDSNUM) return 0;
    GPSEPHREC* eph = beph + prn - 1;

    unsigned short wk = UI2(buff + 14);   // 消息头为 GPS 周，需转成 BDT 周

    eph->PRN = (unsigned short)prn;
    eph->Sys = BDS;
    eph->SVHealth = (short)UI4(buff + pos + 16);
    eph->IODE = (double)UI4(buff + pos + 68);
    eph->IODC = (double)UI4(buff + pos + 36);

    // BDT 周 = GPS 周 − 1356
    short bdt_week = (short)wk - 1356;
    eph->TOC.Week = bdt_week;
    eph->TOE.Week = bdt_week;
    eph->TOC.SecOfWeek = (double)UI4(buff + pos + 40);   // BDT 周内秒
    eph->TOE.SecOfWeek = (double)UI4(buff + pos + 72);   // BDT 周内秒

    eph->TGD1 = R8(buff + pos + 20);
    eph->TGD2 = R8(buff + pos + 28);
    eph->ClkBias = R8(buff + pos + 44);
    eph->ClkDrift = R8(buff + pos + 52);
    eph->ClkDriftRate = R8(buff + pos + 60);
    eph->Crs = R8(buff + pos + 172);
    eph->DeltaN = R8(buff + pos + 100);
    eph->M0 = R8(buff + pos + 108);
    eph->Cuc = R8(buff + pos + 148);
    eph->e = R8(buff + pos + 84);
    eph->Cus = R8(buff + pos + 156);
    eph->SqrtA = R8(buff + pos + 76);
    eph->Cic = R8(buff + pos + 180);
    eph->OMEGA = R8(buff + pos + 116);
    eph->Cis = R8(buff + pos + 188);
    eph->i0 = R8(buff + pos + 132);
    eph->Crc = R8(buff + pos + 164);
    eph->omega = R8(buff + pos + 92);
    eph->OMEGADot = R8(buff + pos + 124);
    eph->iDot = R8(buff + pos + 140);
    eph->SVAccuracy = 0.0;
    eph->Valid = true;

    return 3;
}
