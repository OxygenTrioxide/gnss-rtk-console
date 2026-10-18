#define _CRT_SECURE_NO_WARNINGS
#include "oem7_ascii.h"
#include "gnss_coord.h"      // BLH/XYZ/BLHToXYZ（定位结果消息解码用）
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <string>
#include <vector>

// CRC32 校验函数复用二进制解码路径的实现（oem7_decode.cpp）。
// 课件 1-5 第 18 页：ASCII 记录的 CRC 计算范围为 '#' 之后到 '*' 之前的
// 全部字节，多项式与二进制帧相同。
unsigned int crc32(const unsigned char* buff, int len);

// 按逗号切分记录字段（课件 1-5 第 18 页）
// 规则：双引号字符串内的逗号不算分隔符（如定位结果消息中的 stn_id 字段 ""）
static void SplitAsciiFields(const std::string& text, std::vector<std::string>& fields)
{
    fields.clear();
    std::string cur;
    bool in_quote = false;               // 当前是否处于双引号字符串内
    for (size_t k = 0; k < text.size(); k++) {
        char c = text[k];
        if (c == '"') { in_quote = !in_quote; cur += c; }
        else if (c == ',' && !in_quote) { fields.push_back(cur); cur.clear(); }
        else cur += c;
    }
    fields.push_back(cur);               // 最后一个域之后没有逗号
}

// 十六进制域解析（如接收机状态、ch_tr_status）
static unsigned int HexField(const std::string& s)
{
    return (unsigned int)strtoul(s.c_str(), NULL, 16);
}

// ---------------- RANGEA 观测消息解码（OEM7 消息 ID=43 的 ASCII 长格式） ----------------
// 体域布局（官方文档，首个域为观测数 n，其后每组 10 个域）：
//   prn, glofreq, psr(m), psr_std, adr(周), adr_std, dopp(Hz),
//   cn0(dB-Hz), locktime(s), ch_tr_status(十六进制)
// 观测赋值规则与二进制 decode_rangeb_oem7 完全一致：
//   P=psr，L=-adr×波长，D=-dopp（课件 1-5 第 35 页：相位与多普勒使用前乘 -1）
// ch_tr_status 位定义与二进制相同：sys=bit16-18（0=GPS，4=BDS）、
//   sig=bit21-25、plock=bit10、parity=bit11、clock=bit12；
//   半周标志按 OEM7 官方文档取 bit28（二进制路径取 bit13 且已通过
//   OEM719 实测数据验证，两者差异留待真实 ASCII 数据确认）
static int DecodeRangeA(const std::vector<std::string>& bf, unsigned short week,
    double sec, EPOCHOBSDATA* obs)
{
    if (bf.empty()) return 0;
    int obsnum = atoi(bf[0].c_str());
    if (obsnum <= 0 || bf.size() < (size_t)(1 + (long long)obsnum * 10)) return 0;

    // 注意：只能重置 SatObs 数组，ComObs（粗差探测的上历元平滑结果）
    // 必须保留，不能 memset 整个结构体（与二进制路径相同）
    obs->Time.Week = week;
    obs->Time.SecOfWeek = sec;
    obs->SatNum = 0;
    for (int k = 0; k < MAXCHANNUM; k++) obs->SatObs[k] = SATOBSDATA();

    double wavelength = 0.0;
    for (int i = 0; i < obsnum; i++)
    {
        size_t b = 1 + (size_t)i * 10;
        unsigned int tr_status = HexField(bf[b + 9]);

        int sys_flag     = (tr_status >> 16) & 0x07;   // 系统标识：0=GPS，4=BDS
        int sig_flag     = (tr_status >> 21) & 0x1F;   // 信号标识
        int phase_lock   = (tr_status >> 10) & 0x01;   // 相位锁定
        int parity_known = (tr_status >> 11) & 0x01;   // 奇偶校验已知
        int code_lock    = (tr_status >> 12) & 0x01;   // 码锁定
        int half_cycle   = (tr_status >> 28) & 0x01;   // 半周已改正（官方文档 bit28）

        if (sys_flag != 0 && sys_flag != 4) continue;

        // 频点匹配：GPS L1/L2，BDS B1/B3（课件 1-5 第 56 页：BDS 只保留
        // B1I 与 B3I；B1I/B3I 的 D1/D2 两套导航电文信号标识都要接收）
        int s = -1;
        if (sys_flag == 0) {
            if (sig_flag == 0)      { s = 0; wavelength = C_Light / FG1_GPS; }
            else if (sig_flag == 9) { s = 1; wavelength = C_Light / FG2_GPS; }
        }
        if (sys_flag == 4) {
            if (sig_flag == 0 || sig_flag == 4)     { s = 0; wavelength = C_Light / FG1_BDS; }
            else if (sig_flag == 2 || sig_flag == 6){ s = 1; wavelength = C_Light / FG3_BDS; }
        }
        if (s == -1) continue;

        int prn = atoi(bf[b + 0].c_str());
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

        obs->SatObs[sta_idx].P[s] = atof(bf[b + 2].c_str());
        // 半周改正：half=1 表示相位含接收机施加的 ±0.5λ 恒定偏置，解码时
        // 减去以恢复整周一致（与二进制路径 decode_rangeb_oem7 相同）；
        // 标志翻转才是真半周跳变，由 CheckLockTimeParity 剔除。
        obs->SatObs[sta_idx].L[s] = -atof(bf[b + 4].c_str()) * wavelength
                                    - 0.5 * wavelength * half_cycle;
        obs->SatObs[sta_idx].D[s] = -atof(bf[b + 6].c_str());
        obs->SatObs[sta_idx].cn0[s] = atof(bf[b + 7].c_str());
        obs->SatObs[sta_idx].LockTime[s] = atof(bf[b + 8].c_str());
        // 质量数据：奇偶校验、半周、相位锁定、码锁定（课件 2-1 第 29 页）
        obs->SatObs[sta_idx].parity[s] = (unsigned char)parity_known;
        obs->SatObs[sta_idx].half[s] = (unsigned char)half_cycle;
        obs->SatObs[sta_idx].plock[s] = (unsigned char)phase_lock;
        obs->SatObs[sta_idx].clock[s] = (unsigned char)code_lock;
        obs->SatObs[sta_idx].Valid = true;
    }
    return (obs->SatNum > 0) ? 1 : 0;
}

// ---------------- GPSEPHEMA GPS 星历解码（OEM7 消息 ID=7 的 ASCII 长格式） ----------------
// 体域共 32 个（官方文档），字段顺序与二进制 GPSEPHEM 一致，映射照
// 二进制 decode_gpsephem（oem7_ephem.cpp）：
//   0 prn, 1 tow, 2 health, 3 IODE1, 4 IODE2, 5 week, 6 zweek, 7 toe,
//   8 A(长半轴), 9 ΔN, 10 M0, 11 ecc(偏心率), 12 ω(近地点角距),
//   13 cuc, 14 cus, 15 crc, 16 crs, 17 cic, 18 cis, 19 i0(轨道倾角),
//   20 iDot, 21 Ω0(升交点赤经), 22 Ωdot, 23 iodc, 24 toc, 25 tgd,
//   26 af0(钟差), 27 af1(钟漂), 28 af2(频漂率), 29 AS, 30 N, 31 URA
static int DecodeGpsephemA(const std::vector<std::string>& bf, unsigned short week,
    GPSEPHREC* geph)
{
    if (bf.size() < 32) return 0;
    int prn = atoi(bf[0].c_str());
    if (prn < 1 || prn > MAXGPSNUM) return 0;
    GPSEPHREC* eph = geph + prn - 1;

    eph->PRN = (unsigned short)prn;
    eph->Sys = GPS;
    eph->SVHealth = (short)atoi(bf[2].c_str());        // 卫星健康状态
    eph->IODE = (double)atoi(bf[3].c_str());           // 星历数据龄期（IODE1）
    eph->IODC = (double)strtoul(bf[23].c_str(), NULL, 16); // 时钟数据龄期（十六进制域）
    eph->TOC.Week = week;  eph->TOE.Week = week;       // 周数取消息头
    eph->TOC.SecOfWeek = atof(bf[24].c_str());         // 时钟参考时刻
    eph->TOE.SecOfWeek = atof(bf[7].c_str());          // 星历参考时刻
    eph->TGD1 = atof(bf[25].c_str());                  // 群波延迟改正参数
    eph->TGD2 = 0.0;                                   // L2 频点 TGD，设为 0
    eph->ClkBias = atof(bf[26].c_str());               // 卫星钟差 af0
    eph->ClkDrift = atof(bf[27].c_str());              // 卫星钟漂 af1
    eph->ClkDriftRate = atof(bf[28].c_str());          // 频率漂移率 af2
    eph->Crs = atof(bf[16].c_str());                   // 轨道半径正弦调和改正参数
    eph->DeltaN = atof(bf[9].c_str());                 // 平均角速度修正值
    eph->M0 = atof(bf[10].c_str());                    // 参考时刻的平近点角
    eph->Cuc = atof(bf[13].c_str());                   // 升交角距余弦调和改正幅值
    eph->e = atof(bf[11].c_str());                     // 轨道偏心率
    eph->Cus = atof(bf[14].c_str());                   // 升交角距正弦调和改正幅值
    double A = atof(bf[8].c_str());                    // 长半轴
    eph->SqrtA = (A > 0) ? sqrt(A) : 0.0;
    eph->Cic = atof(bf[17].c_str());                   // 轨道倾角余弦调和改正幅值
    eph->OMEGA = atof(bf[21].c_str());                 // 参考时刻的升交点赤经(Ω0)
    eph->Cis = atof(bf[18].c_str());                   // 轨道倾角正弦调和改正幅值
    eph->i0 = atof(bf[19].c_str());                    // 参考时刻的轨道倾角
    eph->Crc = atof(bf[15].c_str());                   // 轨道半径余弦调和改正振幅
    eph->omega = atof(bf[12].c_str());                 // 近地点角距(ω)
    eph->OMEGADot = atof(bf[22].c_str());              // 升交点赤经变化率
    eph->iDot = atof(bf[20].c_str());                  // 轨道倾角变化率
    eph->SVAccuracy = 0.0;                             // 卫星精度（暂设为 0）
    eph->Valid = true;

    return 2;
}

// ---------------- BDSEPHEMA BDS 星历解码（OEM7 消息 ID=1696 的 ASCII 长格式） ----------------
// OEM7 官方文档未给出 BDSEPHEMA ASCII 体域表，字段顺序由本工程已验证的
// 二进制解码器 decode_bdsephem（oem7_ephem.cpp）推导，共 29 个域：
//   0 prn, 1 周(历元参考), 2 保留(+8), 3 保留(+12), 4 health, 5 TGD1, 6 TGD2,
//   7 AODC(钟数据龄期), 8 TOC秒(BDT周内秒), 9 a0, 10 a1, 11 a2, 12 IODE,
//   13 TOE秒(BDT周内秒), 14 SqrtA, 15 e, 16 ω, 17 ΔN, 18 M0, 19 Ω0,
//   20 Ωdot, 21 i0, 22 iDot, 23 Cuc, 24 Cus, 25 Crc, 26 Crs, 27 Cic, 28 Cis
// 其中 2、3 两个域含义未知，仅按位置跳过；字段计数若与实际不符会整体
// 错位，需真实接收机 ASCII 数据验证（二进制路径已对真实数据验证通过）
static int DecodeBdsephemA(const std::vector<std::string>& bf, unsigned short week,
    GPSEPHREC* beph)
{
    if (bf.size() < 29) return 0;
    int prn = atoi(bf[0].c_str());
    if (prn < 1 || prn > MAXBDSNUM) return 0;
    GPSEPHREC* eph = beph + prn - 1;

    // 消息头为 GPS 周，需转成 BDT 周：BDT 周 = GPS 周 − 1356
    short bdt_week = (short)week - 1356;

    eph->PRN = (unsigned short)prn;
    eph->Sys = BDS;
    eph->SVHealth = (short)atoi(bf[4].c_str());
    eph->IODE = (double)atoi(bf[12].c_str());
    eph->IODC = (double)atoi(bf[7].c_str());
    eph->TOC.Week = bdt_week;
    eph->TOE.Week = bdt_week;
    eph->TOC.SecOfWeek = atof(bf[8].c_str());          // BDT 周内秒
    eph->TOE.SecOfWeek = atof(bf[13].c_str());         // BDT 周内秒
    eph->TGD1 = atof(bf[5].c_str());
    eph->TGD2 = atof(bf[6].c_str());
    eph->ClkBias = atof(bf[9].c_str());
    eph->ClkDrift = atof(bf[10].c_str());
    eph->ClkDriftRate = atof(bf[11].c_str());
    eph->Crs = atof(bf[26].c_str());
    eph->DeltaN = atof(bf[17].c_str());
    eph->M0 = atof(bf[18].c_str());
    eph->Cuc = atof(bf[23].c_str());
    eph->e = atof(bf[15].c_str());
    eph->Cus = atof(bf[24].c_str());
    eph->SqrtA = atof(bf[14].c_str());
    eph->Cic = atof(bf[27].c_str());
    eph->OMEGA = atof(bf[19].c_str());
    eph->Cis = atof(bf[28].c_str());
    eph->i0 = atof(bf[21].c_str());
    eph->Crc = atof(bf[25].c_str());
    eph->omega = atof(bf[16].c_str());
    eph->OMEGADot = atof(bf[20].c_str());
    eph->iDot = atof(bf[22].c_str());
    eph->SVAccuracy = 0.0;
    eph->Valid = true;

    return 3;
}

// ---------------- PSRPOSA/BESTPOSA 定位结果解码（OEM7 消息 ID=47/42 的 ASCII 长格式） ----------------
// 体域共 21 个（官方文档）：
//   0 sol_status, 1 pos_type, 2 lat(度), 3 lon(度), 4 hgt(水准面高),
//   5 undulation(高程异常), 6 datum, 7 latσ, 8 lonσ, 9 hgtσ,
//   10 stn_id(引号串), 11 diff_age, 12 sol_age, 13 #SVs, 14 #solnSVs,
//   15 #L1, 16 #multi, 17 reserved, 18 ext, 19 mask1, 20 mask2
// 椭球高 = hgt + undulation；BLH 转 XYZ 后存入 POSRES（与二进制 decode_psrpos 相同）
static int DecodePosA(const std::vector<std::string>& bf, unsigned short week,
    double sec, POSRES* pos)
{
    if (!pos || bf.size() < 21) return 0;
    if (bf[0] != "SOL_COMPUTED") return 0;             // 只认有效解算结果

    double lat = atof(bf[2].c_str());
    double lon = atof(bf[3].c_str());
    double hgt = atof(bf[4].c_str());
    double und = atof(bf[5].c_str());

    if (lat < -90.0 || lat > 90.0) return 0;
    if (lon < -180.0 || lon > 180.0) return 0;
    if (fabs(lat) < 1e-9 && fabs(lon) < 1e-9) return 0;   // 未定位，解无效

    BLH blh;
    blh.latitude = lat * Rad;
    blh.longitude = lon * Rad;
    blh.height = hgt + und;                  // 椭球高 = 水准面高 + 高程异常
    XYZ xyz;
    BLHToXYZ(blh, xyz);

    pos->Time.Week = (short)week;
    pos->Time.SecOfWeek = sec;
    pos->Position[0] = xyz.x;
    pos->Position[1] = xyz.y;
    pos->Position[2] = xyz.z;
    pos->AllSatNum = (short)atoi(bf[14].c_str());      // 参与解算卫星数 #solnSVs
    pos->IsSuccess = true;
    return 4;
}

// ---------------- ASCII 记录通用入口（课件 1-5 第 18 页） ----------------
// 记录格式：#消息名,端口,序列号,空闲率,时间状态,周,秒,接收机状态,保留,
//           软件版本;体域...*8 位十六进制 CRC\r\n
// 处理流程：CRC 校验 → 切分头域/体域 → 按消息名分发到对应解码器
int DecodeOem7AsciiLine(const char* line, EPOCHOBSDATA* obs,
    GPSEPHREC geph[], GPSEPHREC beph[], POSRES* pos)
{
    if (!line || line[0] != '#') return 0;

    // CRC 校验：'#' 之后至 '*' 之前的全部字节（不含 '#'，不含 '*'；
    // 与二进制帧从同步头起算不同，范围经 OEM7 官方文档 PSRPOSA
    // 例句 *bc4f3b04 验证）
    const char* star = strchr(line, '*');
    if (!star) return 0;
    unsigned int crc_calc = crc32((const unsigned char*)(line + 1),
        (int)(star - line - 1));
    unsigned int crc_msg = (unsigned int)strtoul(star + 1, NULL, 16);
    if (crc_calc != crc_msg) return 0;                 // 校验失败，丢弃

    // 分出头域（'#' 与 ';' 之间）与体域（';' 与 '*' 之间）
    std::string rec(line, star);                       // 去掉 *CRC 部分
    size_t semi = rec.find(';');
    if (semi == std::string::npos) return 0;
    std::string head = rec.substr(1, semi - 1);        // 去掉开头的 '#'
    std::string body = rec.substr(semi + 1);

    std::vector<std::string> hf, bf;
    SplitAsciiFields(head, hf);
    if (hf.size() < 10) return 0;                      // 头域固定 10 个
    SplitAsciiFields(body, bf);

    // 头域：0 消息名，5 周，6 秒（周/秒供观测与定位消息填时间）
    unsigned short week = (unsigned short)atoi(hf[5].c_str());
    double sec = atof(hf[6].c_str());
    const std::string& name = hf[0];

    // 按消息名前缀分发（消息名后可带应答标志等后缀，故用前缀匹配）
    if (name.compare(0, 6, "RANGEA") == 0 && obs)
        return DecodeRangeA(bf, week, sec, obs);
    if (name.compare(0, 9, "GPSEPHEMA") == 0 && geph)
        return DecodeGpsephemA(bf, week, geph);
    if (name.compare(0, 9, "BDSEPHEMA") == 0 && beph)
        return DecodeBdsephemA(bf, week, beph);
    if ((name.compare(0, 7, "PSRPOSA") == 0 || name.compare(0, 8, "BESTPOSA") == 0) && pos)
        return DecodePosA(bf, week, sec, pos);

    return 0;                                          // 其他消息不处理
}
