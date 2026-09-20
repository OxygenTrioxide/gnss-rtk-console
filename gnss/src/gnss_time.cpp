#include "gnss_time.h"

// 常量定义
#define GPS_MJD_START 44244.0  // GPS起点: 1980-01-06 00:00:00 UTC
#define SEC_PER_DAY 86400.0
#define SEC_PER_WEEK 604800.0

/**
 * 判断是否为闰年
 */
static int IsLeapYear(int year) {
    return ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0));
}

/**
 * 获取某年某月的天数
 */
static int DaysInMonth(int year, int month) {
    static int days[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if (month == 2 && IsLeapYear(year)) return 29;
    return days[month - 1];
}

void CommonTimeToMjdTime(const COMMONTIME* ct, MJDTIME* mjd) {
    int y = ct->Year;
    int m = ct->Month;
    int d = ct->Day;
    
    // 如果月份<=2，按前一年处理
    if (m <= 2) {
        y--;
        m += 12;
    }
    
    // 计算儒略日 (JD)
    int A = (int)(y / 100);
    int B = 2 - A + (int)(A / 4);
    double JD = (int)(365.25 * (y + 4716)) + (int)(30.6001 * (m + 1)) + d + B - 1524.5;

    mjd->Days = (int)(JD - 2400000.5);
    // 小数天直接用通用时的时分秒计算，防止大数 JD 拆小数损失精度
    mjd->FracDay = ct->Hour / 24.0 + ct->Minute / 1440.0 + ct->Second / 86400.0;

    // 确保小数天在[0, 1)范围内
    if (mjd->FracDay >= 1.0) {
        mjd->FracDay -= 1.0;
        mjd->Days++;
    }
    if (mjd->FracDay < 0.0) {
        mjd->FracDay += 1.0;
        mjd->Days--;
    }
}

void MjdTimeToCommonTime(const MJDTIME* mjd, COMMONTIME* ct) {
    // 恢复儒略日
    double JD = mjd->Days + mjd->FracDay + 2400000.5;
    
    // 儒略日转通用时 (Meeus算法)
    double Z = floor(JD + 0.5);
    double F = (JD + 0.5) - Z;

    int A, B;
    if (Z < 2299161.0) {
        // 儒略历：无格里历修正
        A = (int)(Z);
        B = (int)(Z);
    }
    else {
        A = (int)((Z - 1867216.25) / 36524.25);
        B = (int)(Z + 1 + A - (int)(A / 4.0));
    }

    int C = B + 1524;
    int D = (int)((C - 122.1) / 365.25);
    int E = (int)(365.25 * D);
    int G = (int)((C - E) / 30.6001);
    
    int day = C - E - (int)(30.6001 * G);
    int month = G - 1;
    if (month > 12) month -= 12;
    int year = D - 4715;
    if (month > 2) year--;
    if (year <= 0) year--;
    
    // 计算时分秒
    double dayFrac = F;
    int hour = (int)(dayFrac * 24.0);
    dayFrac -= hour / 24.0;
    int minute = (int)(dayFrac * 1440.0);
    dayFrac -= minute / 1440.0;
    double second = dayFrac * 86400.0;
    
    // 处理舍入误差
    if (second >= 60.0) {
        second -= 60.0;
        minute++;
    }
    if (minute >= 60) {
        minute -= 60;
        hour++;
    }
    if (hour >= 24) {
        hour -= 24;
        day++;
    }
    
    ct->Year = (short)year;
    ct->Month = (unsigned short)month;
    ct->Day = (unsigned short)day;
    ct->Hour = (unsigned short)hour;
    ct->Minute = (unsigned short)minute;
    ct->Second = second;
}

void MjdTimeToGpsTime(const MJDTIME* mjd, GPSTIME* gt) {
    // 计算从GPS起点(1980-01-06)开始的天数
    double mjdVal = mjd->Days + mjd->FracDay;
    double deltaDays = mjdVal - GPS_MJD_START;
    
    // 计算周数和周内秒
    gt->Week = (short)floor(deltaDays / 7.0);
    double weekDays = deltaDays - gt->Week * 7.0;
    gt->SecOfWeek = weekDays * SEC_PER_DAY;
    
    // 处理舍入误差
    if (gt->SecOfWeek >= SEC_PER_WEEK) {
        gt->SecOfWeek -= SEC_PER_WEEK;
        gt->Week++;
    }
    if (gt->SecOfWeek < 0) {
        gt->SecOfWeek += SEC_PER_WEEK;
        gt->Week--;
    }
}

void GpsTimeToMjdTime(const GPSTIME* gt, MJDTIME* mjd) {
    double mjdVal = GPS_MJD_START + gt->Week * 7.0 + gt->SecOfWeek / SEC_PER_DAY;
    mjd->Days = (int)floor(mjdVal);
    mjd->FracDay = mjdVal - mjd->Days;
}

void CommonTimeToGpsTime(const COMMONTIME* ct, GPSTIME* gt) {
    MJDTIME mjd;
    CommonTimeToMjdTime(ct, &mjd);
    MjdTimeToGpsTime(&mjd, gt);
}

void GpsTimeToCommonTime(const GPSTIME* gt, COMMONTIME* ct) {
    MJDTIME mjd;
    GpsTimeToMjdTime(gt, &mjd);
    MjdTimeToCommonTime(&mjd, ct);
}

void PrintGpsTime(const GPSTIME* gt, const char* name) {
    printf("%s: Week=%d, SecOfWeek=%.6f\n", name, gt->Week, gt->SecOfWeek);
}

void PrintCommonTime(const COMMONTIME* ct, const char* name) {
    printf("%s: %04d-%02d-%02d %02d:%02d:%09.6f\n", 
           name, ct->Year, ct->Month, ct->Day, 
           ct->Hour, ct->Minute, ct->Second);
}