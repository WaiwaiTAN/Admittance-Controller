// pch.cpp: 与预编译标头对应的源文件

#include "comm.h"

// 当使用预编译的头时，需要使用此源文件，编译才能成功。


double data_convert(unsigned char* buf)
{
    int data_int = 0;

    if (0x00 == (buf[0] & 0x80))
    {
        data_int = (unsigned char)buf[0];
        data_int <<= 8;
        data_int += (unsigned char)buf[1];
        data_int <<= 8;
        data_int += (unsigned char)buf[2];
    }
    else
    {
        data_int = ((unsigned char)buf[0] & 0x7F);
        data_int <<= 8;
        data_int += (unsigned char)buf[1];
        data_int <<= 8;
        data_int += (unsigned char)buf[2];
        data_int *= -1;
    }
    return (double)data_int / 1000;
}

HANDLE com_config()
{
    HANDLE m_hCom = CreateFile(L"COM3", GENERIC_WRITE | GENERIC_READ, 0, NULL, OPEN_EXISTING, FILE_FLAG_OVERLAPPED | FILE_ATTRIBUTE_NORMAL, NULL);
    if (m_hCom == INVALID_HANDLE_VALUE)
    {
        fprintf(stderr, "serial port open failed!\n");
        return INVALID_HANDLE_VALUE;
    }

    DCB dcb;
    memset(&dcb, 0, sizeof(DCB));
    if (!GetCommState(m_hCom, &dcb))
    {
        fprintf(stderr, "dcb get failed!\n");
        return INVALID_HANDLE_VALUE;
    }
    dcb.BaudRate = CBR_115200;

    if (!SetCommState(m_hCom, &dcb))
    {
        fprintf(stderr, "baurate set failed!\n");
        return INVALID_HANDLE_VALUE;
    }

    COMMTIMEOUTS TimeOuts;
    TimeOuts.ReadIntervalTimeout = 1000;
    TimeOuts.ReadTotalTimeoutMultiplier = 500;
    TimeOuts.ReadTotalTimeoutConstant = 5000;
    TimeOuts.WriteTotalTimeoutMultiplier = 500;
    TimeOuts.WriteTotalTimeoutConstant = 2000;
    SetCommTimeouts(m_hCom, &TimeOuts);

    PurgeComm(m_hCom, PURGE_TXCLEAR | PURGE_RXCLEAR);
    return m_hCom;
}


BOOL com_rw(double data[], HANDLE m_hCom)
{
    unsigned char request_data_cmd[] = { 0x01, 0x03, 0x02, 0x00, 0x12, 0x38, 0x49 };
    DWORD dwBytesWrite = 7;
    DWORD dwErrorFlags;
    COMSTAT comStat;
    OVERLAPPED m_osWrite;
    memset(&m_osWrite, 0, sizeof(OVERLAPPED));
    m_osWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

    ClearCommError(m_hCom, &dwErrorFlags, &comStat);
    BOOL bWriteStat = WriteFile(m_hCom, request_data_cmd, dwBytesWrite, &dwBytesWrite, &m_osWrite);
    if (!bWriteStat)
    {
        if (GetLastError() == ERROR_IO_PENDING)
        {
            WaitForSingleObject(m_osWrite.hEvent, 1000);
        }
        else
        {
            ClearCommError(m_hCom, &dwErrorFlags, &comStat);
            CloseHandle(m_osWrite.hEvent);
        }
    }

    DWORD wCount = 23;
    OVERLAPPED m_osRead;
    memset(&m_osRead, 0, sizeof(OVERLAPPED));
    m_osRead.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

    ClearCommError(m_hCom, &dwErrorFlags, &comStat);

    unsigned char buf[23];
    BOOL bReadStat = ReadFile(m_hCom, buf, wCount, &wCount, &m_osRead);
    if (!bReadStat)
    {
        if (GetLastError() == ERROR_IO_PENDING)
        {
            GetOverlappedResult(m_hCom, &m_osRead, &wCount, TRUE);
        }
        else
        {
            ClearCommError(m_hCom, &dwErrorFlags, &comStat);
            CloseHandle(m_osRead.hEvent);
        }
    }

    unsigned char buf_desired[3] = { 0x01, 0x03, 0x12 };
    for (int i = 0; i < 3; i++)
    {
        if (buf_desired[i] != buf[i])
        {
            fprintf(stderr, "com request data failed!\n");
            return FALSE;
        }
    }

    for (int i = 0; i < 6; i++)
    {
        data[i] = data_convert(&buf[3 * i + 3]);
    }
    return TRUE;
}

BOOL com_reset(HANDLE m_hCom)
{
    unsigned char request_data_cmd[] = { 0x01, 0x12, 0x00, 0x00, 0x01, 0xdd, 0x78 };
    DWORD dwBytesWrite = 7;
    DWORD dwErrorFlags;
    COMSTAT comStat;
    OVERLAPPED m_osWrite;
    memset(&m_osWrite, 0, sizeof(OVERLAPPED));
    m_osWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

    ClearCommError(m_hCom, &dwErrorFlags, &comStat);
    BOOL bWriteStat = WriteFile(m_hCom, request_data_cmd, dwBytesWrite, &dwBytesWrite, &m_osWrite);
    if (!bWriteStat)
    {
        if (GetLastError() == ERROR_IO_PENDING)
        {
            WaitForSingleObject(m_osWrite.hEvent, 1000);
        }
        else
        {
            ClearCommError(m_hCom, &dwErrorFlags, &comStat);
            CloseHandle(m_osWrite.hEvent);
        }
    }

    DWORD wCount = 23;
    OVERLAPPED m_osRead;
    memset(&m_osRead, 0, sizeof(OVERLAPPED));
    m_osRead.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

    ClearCommError(m_hCom, &dwErrorFlags, &comStat);

    unsigned char buf[5];
    BOOL bReadStat = ReadFile(m_hCom, buf, wCount, &wCount, &m_osRead);
    if (!bReadStat)
    {
        if (GetLastError() == ERROR_IO_PENDING)
        {
            GetOverlappedResult(m_hCom, &m_osRead, &wCount, TRUE);
        }
        else
        {
            ClearCommError(m_hCom, &dwErrorFlags, &comStat);
            CloseHandle(m_osRead.hEvent);
        }
    }

    unsigned char buf_desired[5] = { 0x01, 0x12, 0x01, 0xed, 0x60 };
    for (int i = 0; i < 5; i++)
    {
        if (buf_desired[i] != buf[i])
        {
            fprintf(stderr, "com set zero failed!\n");
            return FALSE;
        }
    }
    return TRUE;
}