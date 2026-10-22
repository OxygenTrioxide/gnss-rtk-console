#ifndef SERIAL_H
#define SERIAL_H

// 串口通信类（课件 1-5 第 13/14 页 CSerial）
// 课件给出类接口：
//   CSerial();
//   ~CSerial();
//   BOOL Open(int nPort = 2, int nBaud = 9600);
//   BOOL Close(void);
//   int  ReadData(void*, int);
//   int  SendData(const char*, int);
//   int  ReadDataWaiting(void);
//   BOOL IsOpened(void);
//   protected: BOOL WriteCommByte(unsigned char);
// 课件第 14 页注：CSerial 使用 Win32 函数，控制台工程需链接 Win 函数库
// （kernel32 由工程默认链接，无需额外设置）。

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX          // 禁止 windows.h 定义 max/min 宏，避免破坏 std::max 等
#endif
#include <windows.h>

#include <cstdio>

#include "rtk_sd.h"   // RAWDAT 等结构（串口数据同步重载使用）

class CSerial
{
public:
    CSerial();
    ~CSerial();

    // 打开串口：nPort = COM 口号（如 2 表示 COM2），nBaud = 波特率
    BOOL Open(int nPort = 2, int nBaud = 9600);
    BOOL Close(void);

    // 读取数据：返回实际读到的字节数（0 = 当前无数据）
    int  ReadData(void* buffer, int limit);

    // 发送数据：返回实际发送的字节数
    int  SendData(const char* buffer, int size);

    // 输入缓冲区中待读字节数（用于"查询式"定时读取）
    int  ReadDataWaiting(void);

    BOOL IsOpened(void) { return m_bOpened; }

protected:
    BOOL WriteCommByte(unsigned char ucByte);

    HANDLE     m_hIDComDev;      // 串口句柄
    OVERLAPPED m_OverlappedRead;  // 保留课件接口（本实现用同步超时读，不实际使用）
    OVERLAPPED m_OverlappedWrite;
    BOOL       m_bOpened;        // 是否已打开
    int        m_nPort;          // 端口号
    int        m_nBaud;          // 波特率
};

// 数据同步：实时串口模式，与文件模式同名（函数重载，课件 2-1 第 37 页）
// 返回 1=同步成功，-1=串口无数据/中断
int GetSynGNSSObs(CSerial* cBase, CSerial* cRov, RAWDAT* raw, double thresSec);

// 向串口写入 NovAtel Log 配置命令（课件 1-5 第 22 页，可选步骤）
// 返回实际发送的字节数
int SendLogCommands(CSerial* com);

#endif // SERIAL_H
