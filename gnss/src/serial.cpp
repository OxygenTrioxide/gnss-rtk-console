#define _CRT_SECURE_NO_WARNINGS
#include "serial.h"
#include <cstring>

// 串口通信类实现（查询式读取，课件 1-5 第 15 页）
//
// 说明：课件 1-5 第 15 页给出三种读取控制方式——定时器查询式、事件驱动
//（串口监测线程）、控制台 Sleep 定时读取。本实现采用"查询式"：
//   ReadData 立即返回当前缓冲区中的数据（无数据返回 0），
// 调用方（GetSynGNSSObs 串口重载）用 Sleep 轮询，符合课件第 15 页做法。
// ReadIntervalTimeout = MAXDWORD, ReadTotalTimeout* = 0

CSerial::CSerial()
{
    m_hIDComDev = INVALID_HANDLE_VALUE;
    m_bOpened = FALSE;
    m_nPort = 0;
    m_nBaud = 0;
    memset(&m_OverlappedRead, 0, sizeof(m_OverlappedRead));
    memset(&m_OverlappedWrite, 0, sizeof(m_OverlappedWrite));
}

CSerial::~CSerial()
{
    Close();
}

BOOL CSerial::Open(int nPort, int nBaud)
{
    if (m_bOpened) return TRUE;

    // COM10 及以上必须使用 \\.\COMxx 形式
    char szPort[32];
    if (nPort < 10) snprintf(szPort, sizeof(szPort), "COM%d", nPort);
    else            snprintf(szPort, sizeof(szPort), "\\\\.\\COM%d", nPort);

    m_hIDComDev = CreateFileA(szPort, GENERIC_READ | GENERIC_WRITE, 0, NULL,
        OPEN_EXISTING, 0, NULL);
    if (m_hIDComDev == INVALID_HANDLE_VALUE) {
        printf("  [COM] Cannot open %s (error %lu)\n", szPort, GetLastError());
        return FALSE;
    }

    DCB dcb;
    memset(&dcb, 0, sizeof(dcb));
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(m_hIDComDev, &dcb)) {
        printf("  [COM] GetCommState failed\n");
        CloseHandle(m_hIDComDev);
        m_hIDComDev = INVALID_HANDLE_VALUE;
        return FALSE;
    }

    // 8 数据位 / 无校验 / 1 停止位（课件 1-5 第 11/12 页起止式异步协议）
    dcb.BaudRate = (DWORD)nBaud;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fParity = FALSE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;
    dcb.fDsrSensitivity = FALSE;
    dcb.fRtsControl = RTS_CONTROL_ENABLE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    dcb.fAbortOnError = FALSE;

    if (!SetCommState(m_hIDComDev, &dcb)) {
        printf("  [COM] SetCommState failed（波特率 %d）\n", nBaud);
        CloseHandle(m_hIDComDev);
        m_hIDComDev = INVALID_HANDLE_VALUE;
        return FALSE;
    }

    // 同步读 + 立即返回（无数据时返回 0）
    COMMTIMEOUTS to;
    memset(&to, 0, sizeof(to));
    to.ReadIntervalTimeout = MAXDWORD;
    to.ReadTotalTimeoutMultiplier = 0;
    to.ReadTotalTimeoutConstant = 0;
    to.WriteTotalTimeoutMultiplier = 0;
    to.WriteTotalTimeoutConstant = 1000;
    if (!SetCommTimeouts(m_hIDComDev, &to)) {
        printf("  [COM] SetCommTimeouts failed\n");
        CloseHandle(m_hIDComDev);
        m_hIDComDev = INVALID_HANDLE_VALUE;
        return FALSE;
    }

    SetupComm(m_hIDComDev, 16384, 16384);
    PurgeComm(m_hIDComDev, PURGE_TXCLEAR | PURGE_RXCLEAR);

    m_bOpened = TRUE;
    m_nPort = nPort;
    m_nBaud = nBaud;
    printf("  [COM] COM%d opened, %d bps, 8-N-1\n", nPort, nBaud);
    return TRUE;
}

BOOL CSerial::Close(void)
{
    if (m_hIDComDev != INVALID_HANDLE_VALUE) {
        CloseHandle(m_hIDComDev);
        m_hIDComDev = INVALID_HANDLE_VALUE;
    }
    m_bOpened = FALSE;
    return TRUE;
}

BOOL CSerial::WriteCommByte(unsigned char ucByte)
{
    if (!m_bOpened) return FALSE;
    DWORD dwBytesWritten = 0;
    BOOL bWriteStat = WriteFile(m_hIDComDev, (LPCVOID)&ucByte, 1, &dwBytesWritten, NULL);
    return (bWriteStat && dwBytesWritten == 1);
}

int CSerial::SendData(const char* buffer, int size)
{
    if (!m_bOpened || size <= 0) return 0;
    DWORD dwBytesWritten = 0;
    if (!WriteFile(m_hIDComDev, (LPCVOID)buffer, (DWORD)size, &dwBytesWritten, NULL))
        return 0;
    return (int)dwBytesWritten;
}

int CSerial::ReadDataWaiting(void)
{
    if (!m_bOpened) return 0;
    COMSTAT cs;
    DWORD dwError = 0;
    memset(&cs, 0, sizeof(cs));
    if (!ClearCommError(m_hIDComDev, &dwError, &cs)) return 0;
    return (int)cs.cbInQue;
}

int CSerial::ReadData(void* buffer, int limit)
{
    if (!m_bOpened || limit <= 0) return 0;
    DWORD dwBytesRead = 0;
    if (!ReadFile(m_hIDComDev, buffer, (DWORD)limit, &dwBytesRead, NULL))
        return 0;
    return (int)dwBytesRead;
}

// ------------------------------------------------------------
// 向接收机写入 Log 配置命令（课件 1-5 第 22 页"输出数据控制命令"）
//   log rangeb     ontime 1     观测数据（二进制）
//   log gpsephemb  onchanged    GPS 广播星历（二进制）
//   log bdsephemb  onchanged    BDS 广播星历（二进制）
//   log bestposb   ontime 1     定位结果（二进制，基站坐标来源）
//   log psrposb    ontime 1     伪距定位结果（二进制）
// 每条命令以 [CR] 结尾；课件第 4 页把"写入 Log 命令"列为可选步骤。
// ------------------------------------------------------------
int SendLogCommands(CSerial* com)
{
    if (!com || !com->IsOpened()) return 0;

    static const char* cmds[] = {
        "log rangeb ontime 1\r",
        "log gpsephemb onchanged\r",
        "log bdsephemb onchanged\r",
        "log bestposb ontime 1\r",
        "log psrposb ontime 1\r",
    };
    int total = 0;
    for (int i = 0; i < (int)(sizeof(cmds) / sizeof(cmds[0])); i++) {
        int n = com->SendData(cmds[i], (int)strlen(cmds[i]));
        printf("  [COM] send: %-28s (%d bytes)\n", cmds[i], n);
        total += n;
        Sleep(50);          // 命令之间留间隔，避免接收机命令缓冲溢出
    }
    return total;
}
