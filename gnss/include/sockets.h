#ifndef SOCKETS_H
#define SOCKETS_H

#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")

#include <cstdio>

#include "rtk_sd.h"   // RAWDAT 等结构

// 建立 TCP 连接（课件 1-8 第 26/27 页：socket.h 中的 OpenSocket/CloseSocket）
// 内含 WSAStartup 初始化
bool OpenSocket(SOCKET& sock, const char* ip, unsigned short port);

// 关闭连接
void CloseSocket(SOCKET& sock);

// 数据同步：实时网络模式（TCP 数据流）
// 与文件模式同名（函数重载，课件 2-1 第 37 页），算法与文件版一致
// 返回 1=同步成功，-1=连接中断/结束
int GetSynGNSSObs(SOCKET sBase, SOCKET sRov, RAWDAT* raw, double thresSec);

#endif
