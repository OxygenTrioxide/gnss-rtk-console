#define _CRT_SECURE_NO_WARNINGS
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include "sockets.h"

bool OpenSocket(SOCKET& sock, const char* ip, unsigned short port)
{
    WSADATA wsaData;
    SOCKADDR_IN addrSrv;

    // 初始化 Winsock 2.2
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        printf("WSAStartup failed\n");
        return false;
    }

    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock == INVALID_SOCKET) {
        printf("socket() failed\n");
        WSACleanup();
        return false;
    }

    addrSrv.sin_addr.S_un.S_addr = inet_addr(ip);
    addrSrv.sin_family = AF_INET;
    addrSrv.sin_port = htons(port);

    if (connect(sock, (SOCKADDR*)&addrSrv, sizeof(SOCKADDR)) == SOCKET_ERROR) {
        printf("connect() failed: %d\n", WSAGetLastError());
        closesocket(sock);
        WSACleanup();
        return false;
    }

    return true;
}

void CloseSocket(SOCKET& sock)
{
    if (sock != INVALID_SOCKET) {
        closesocket(sock);
    }
    WSACleanup();
}

// 网络模式的数据同步已统一到 rtk_sd.cpp 的 GetSynGNSSObs(SOCKET, SOCKET, ...)
// 重载（与文件/串口模式共用同一套滚动对齐算法，课件 2-1 第 37 页）。
