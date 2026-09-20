#ifndef RTK_CONFIG_H
#define RTK_CONFIG_H

#include "RTK_Structs.h"   // ROVERCFGINFO 配置结构

// 读取 rtk.ini 配置文件到 cfg
bool ReadRTKConfigInfo(const char* FName, ROVERCFGINFO& cfg);

// 打印配置内容到控制台
void PrintRTKConfigInfo(const ROVERCFGINFO& cfg);

#endif
