#define _CRT_SECURE_NO_WARNINGS
#include "RTK_Structs.h"
#include <iostream>
#include "oem7_ephem.h"
#include "oem7_ascii.h"      // OEM7 ASCII 记录通用解码框架（课件 1-5 第 18 页）
#include "gnss_coord.h"      // BLH/XYZ/BLHToXYZ（定位结果消息解码用）

#define POLYCRC32   0xEDB88320u

// CRC32 校验（OEM7 帧尾 4 字节）
unsigned int crc32(const unsigned char* buff, int len)
{
    unsigned int crc = 0;
    for (int i = 0; i < len; i++) {
        crc ^= buff[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ POLYCRC32;
            else crc >>= 1;
        }
    }
    return crc;
}

// OEM7 消息字段读取（小端，与 memcpy 对应）
double R8(unsigned char* p) {
    double r;
    memcpy(&r, p, 8);
    return r;
}

float R4(unsigned char* p) {
    float r;
    memcpy(&r, p, 4);
    return r;
}

int I4(unsigned char* p) {
    int r;
    memcpy(&r, p, 4);
    return r;
}

unsigned int UI4(unsigned char* p) {
    unsigned int r;
    memcpy(&r, p, 4);
    return r;
}

short I2(unsigned char* p) {
    short r;
    memcpy(&r, p, 2);
    return r;
}

unsigned short UI2(unsigned char* p) {
    unsigned short r;
    memcpy(&r, p, 2);
    return r;
}

// RANGE 观测值消息解码（OEM7 消息 ID=43，每条 44 字节/通道）
int decode_rangeb_oem7(unsigned char* buff, EPOCHOBSDATA* obs)
{
    int i, j, pos = 28;

    obs->Time.Week = UI2(buff + 14);
    obs->Time.SecOfWeek = UI4(buff + 16) / 1000.0;
    obs->SatNum = 0;

    double wavelength = 0.0;
    int obsnum = I4(buff + pos);
    int start = pos + 4;

    for (i = 0, j = 0; i < obsnum; i++, j += 44)
    {
        int track_status = UI4(buff + start + j + 40);
        int sys_flag = (track_status >> 16) & 0x07;      // 系统标识：0=GPS, 4=BDS
        int sig_flag = (track_status >> 21) & 0x1F;      // 信号标识
        int phase_lock = (track_status >> 10) & 0x01;    // 相位锁定
        int parity_known = (track_status >> 11) & 0x01;  // 奇偶校验已知
        int code_lock = (track_status >> 12) & 0x01;     // 码锁定
        int half_cycle = (track_status >> 13) & 0x01;    // 半周已改正

        if (sys_flag != 0 && sys_flag != 4) continue;

        // 频点匹配：GPS L1/L2，BDS B1/B3
        // 课件 1-5 第 56 页：BDS 只保留 B1I 与 B3I。BDS 的 B1I/B3I 有两套
        // 导航电文（D1：MEO/IGSO，信号标识 0/2；D2：GEO，信号标识 4/6），
        // 二者频率相同，必须都接收，否则 GEO 卫星观测值会被整体丢弃。
        int s = -1;
        if (sys_flag == 0) {
            if (sig_flag == 0) { s = 0; wavelength = C_Light / FG1_GPS; }
            else if (sig_flag == 9) { s = 1; wavelength = C_Light / FG2_GPS; }
        }
        if (sys_flag == 4) {
            if (sig_flag == 0 || sig_flag == 4) { s = 0; wavelength = C_Light / FG1_BDS; }
            else if (sig_flag == 2 || sig_flag == 6) { s = 1; wavelength = C_Light / FG3_BDS; }
        }
        if (s == -1) continue;

        short prn = UI2(buff + start + j + 0);
        GNSSSys sys = (sys_flag == 0) ? GPS : BDS;

        // 同星多频点合并到同一 SatObs 槽位
        int sta_idx = -1;
        for (int k = 0; k < obs->SatNum; k++) {
            if (obs->SatObs[k].Prn == prn && obs->SatObs[k].System == sys) {
                sta_idx = k; break;
            }
        }
        if (sta_idx == -1) {
            if (obs->SatNum >= MAXCHANNUM) continue;
            sta_idx = obs->SatNum;
            obs->SatObs[sta_idx].Prn = prn;
            obs->SatObs[sta_idx].System = sys;
            obs->SatNum++;
        }

        obs->SatObs[sta_idx].P[s] = R8(buff + start + j + 4);
        // 课件 1-5 第 35 页：相位使用前乘以 -1
        // 半周改正：half=1 表示接收机已对该载波相位"加过半周"，观测值中含
        // 一个 ±0.5λ 的恒定偏置。若不去除，双差模糊度将呈半整数导致 LAMBDA
        // 无法固定；减去已添加的半周使相位恢复整周一致（改正方向反了也只会
        // 差 1 整周，可被模糊度整数吸收）。该标志的"翻转"才是真半周跳变，
        // 由 CheckLockTimeParity 按周跳剔除。
        obs->SatObs[sta_idx].L[s] = -R8(buff + start + j + 16) * wavelength
                                    - 0.5 * wavelength * half_cycle;
        // 课件 1-5 第 35 页：相位与多普勒观测值使用前均需乘以 -1
        obs->SatObs[sta_idx].D[s] = -R4(buff + start + j + 28);
        obs->SatObs[sta_idx].cn0[s] = R4(buff + start + j + 32);
        obs->SatObs[sta_idx].LockTime[s] = R4(buff + start + j + 36);
        // 质量数据：奇偶校验、半周、相位锁定、码锁定（课件 2-1 第 29 页）
        obs->SatObs[sta_idx].parity[s] = (unsigned char)parity_known;
        obs->SatObs[sta_idx].half[s] = (unsigned char)half_cycle;
        obs->SatObs[sta_idx].plock[s] = (unsigned char)phase_lock;
        obs->SatObs[sta_idx].clock[s] = (unsigned char)code_lock;
        obs->SatObs[sta_idx].Valid = true;
    }
    return (obs->SatNum > 0) ? 1 : 0;
}

// 定位结果消息解码（课件 1-5 第 43/44 页 PSRPOS；BESTPOS 前 36 字节布局相同）
//   H+8  lat (Double, 度)      H+16 lon (Double, 度)
//   H+24 hgt (Double, m, 大地水准面高)
//   H+32 undulation (Float, m) → 椭球高 = hgt + undulation
//   H+65 #solnSVs (Uchar)
// 输出：POSRES.Position 为天线 ECEF 坐标，AllSatNum 为参与解算卫星数
int decode_psrpos(unsigned char* buff, POSRES* pos)
{
    if (!buff || !pos) return 0;

    const int H = 28;                       // 消息头长度
    double lat = R8(buff + H + 8);
    double lon = R8(buff + H + 16);
    double hgt = R8(buff + H + 24);
    double und = (double)R4(buff + H + 32);

    if (lat < -90.0 || lat > 90.0) return 0;
    if (lon < -180.0 || lon > 180.0) return 0;
    if (fabs(lat) < 1e-9 && fabs(lon) < 1e-9) return 0;   // 未定位，解无效

    BLH blh;
    blh.latitude = lat * Rad;
    blh.longitude = lon * Rad;
    blh.height = hgt + und;                  // 椭球高 = 水准面高 + 高程异常
    XYZ xyz;
    BLHToXYZ(blh, xyz);

    pos->Time.Week = (short)UI2(buff + 14);
    pos->Time.SecOfWeek = UI4(buff + 16) / 1000.0;
    pos->Position[0] = xyz.x;
    pos->Position[1] = xyz.y;
    pos->Position[2] = xyz.z;
    pos->AllSatNum = (short)buff[H + 65];
    pos->IsSuccess = true;
    return 1;
}

// 主解码函数：扫描 OEM7 帧头(0xAA4412)，CRC 校验通过后按消息 ID 分发
int DecodeNovOem7Dat(unsigned char Buff[], int& Len, EPOCHOBSDATA* obs,
    GPSEPHREC geph[], GPSEPHREC beph[], POSRES* basPos)
{
    int haveobs = 0;
    int i = 0;
    int MsgId, MsgLen;
    static unsigned char Msg[10240];
    // ASCII 记录缓冲区：RANGEA 全通道记录可长达数万字节
    static char AsciiLine[131072];
    memset(Msg, 0, sizeof(Msg));

    for (i = 0; i < Len; ) {
        // 找下一条记录起点：二进制同步头(0xAA4412) 或 ASCII 起始符 '#'
        for (; i < Len; i++) {
            if (Buff[i] == '#') break;
            if (i + 2 < Len && Buff[i + 0] == 0xAA && Buff[i + 1] == 0x44
                && Buff[i + 2] == 0x12) break;
        }
        if (i >= Len) {
            // 未找到记录起点：末尾保留 2 字节，防止同步头(3字节)跨包拆分丢失
            i = (Len >= 2) ? Len - 2 : 0;
            break;
        }

        if (Buff[i] == '#') {
            // ASCII 记录：提取至换行符为止，交给通用 ASCII 解码器
            int j = i;
            while (j < Len && Buff[j] != '\n') j++;
            if (j >= Len) break;              // 行不完整，留待下次数据拼接
            int linelen = j - i;
            if (linelen > (int)sizeof(AsciiLine) - 1) {
                i = j + 1;                    // 超长行无法解析，直接跳过
                continue;
            }
            memcpy(AsciiLine, Buff + i, linelen);
            if (linelen > 0 && AsciiLine[linelen - 1] == '\r') linelen--;
            AsciiLine[linelen] = 0;
            int rc = DecodeOem7AsciiLine(AsciiLine, obs, geph, beph, basPos);
            i = j + 1;
            if (rc == 1) { haveobs = 1; break; }   // 得到一个历元观测即返回
            continue;                              // 星历/定位结果继续扫描
        }

        if (i + 28 > Len) break;

        memcpy(Msg, Buff + i, 28);
        MsgId = UI2(Msg + 4);
        MsgLen = UI2(Msg + 8);

        if (i + 28 + MsgLen + 4 > Len) break;

        memcpy(Msg + 28, Buff + i + 28, MsgLen + 4);

        unsigned int crcCalc = crc32(Msg, 28 + MsgLen);
        unsigned int crcMsg = UI4(Msg + 28 + MsgLen);

        if (crcCalc != crcMsg) {
            i += 3;
            continue;
        }

        if (MsgId == 43) {
            // 注意：解码时只能重置 SatObs 数组，
            // ComObs（粗差探测的上历元平滑结果）必须保留，不能 memset 整个结构体
            obs->SatNum = 0;
            for (int k = 0; k < MAXCHANNUM; k++) obs->SatObs[k] = SATOBSDATA();
            haveobs = decode_rangeb_oem7(Msg, obs);
            i += 28 + MsgLen + 4;
            if (haveobs) break;   // 得到一个历元观测即返回
        }
        else if (MsgId == 7) {
            decode_gpsephem(Msg, geph);
            i += 28 + MsgLen + 4;
        }
        else if (MsgId == 1696) {
            decode_bdsephem(Msg, beph);
            i += 28 + MsgLen + 4;
        }
        else if ((MsgId == 47 || MsgId == 42) && basPos) {
            // PSRPOS(47) / BESTPOS(42)：基站坐标来源（课件 2-1 第 15 页）
            decode_psrpos(Msg, basPos);
            i += 28 + MsgLen + 4;
        }
        else {
            i += 28 + MsgLen + 4;
        }
    }

    if (i > Len) i = Len;
    if (i < 0)    i = 0;

    if (Len - i > 0) {
        memcpy(Buff, Buff + i, Len - i);
    }
    Len = Len - i;
    return haveobs;
}
