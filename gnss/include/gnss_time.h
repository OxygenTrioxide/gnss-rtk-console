#ifndef GNSS_TIME_H
#define GNSS_TIME_H

#include "RTK_Structs.h"   // COMMONTIME / GPSTIME / MJDTIME 结构定义
#include <cmath>
#include <cstdio>

// 时间系统转换：通用时(年月日时分秒) ↔ 儒略日(MJD) ↔ GPS 时(周 + 周内秒)
void CommonTimeToMjdTime(const COMMONTIME* ct, MJDTIME* mjd);
void MjdTimeToCommonTime(const MJDTIME* mjd, COMMONTIME* ct);
void MjdTimeToGpsTime(const MJDTIME* mjd, GPSTIME* gt);
void GpsTimeToMjdTime(const GPSTIME* gt, MJDTIME* mjd);
void CommonTimeToGpsTime(const COMMONTIME* ct, GPSTIME* gt);
void GpsTimeToCommonTime(const GPSTIME* gt, COMMONTIME* ct);

void PrintGpsTime(const GPSTIME* gt, const char* name);
void PrintCommonTime(const COMMONTIME* ct, const char* name);

#endif // GNSS_TIME_H
