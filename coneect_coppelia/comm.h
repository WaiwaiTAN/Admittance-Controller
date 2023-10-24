/*! \file comm.h
    \brief 本头文件定义提取传感器数据相关的几个函数

    版本：1.0

    \author 谭正
    \date 2021.05.27
*/
#ifndef _COMM_H_
#define _COMM_H_

#include <stdio.h>
#include <Windows.h>

#ifdef __cplusplus
extern "C" {
#endif

/*! \fn data_convert
* \brief 解码数据
*/
double data_convert(unsigned char* buf);

/*! \fn com_config
* \brief 打开串口，获取句柄
* 
* 如打开失败将会在标准错误流中打印错误信息
*/
HANDLE com_config();

/*! \fn com_reset
* \brief 向串口发送传感器置零命令
* 
* 该函数会等待置零完成
* 如置零失败将会在标准错误流中打印错误信息并返回FALSE
* 
* \param m_hCom 传感器串口句柄
* \return 置零成功与否
*/
BOOL com_reset(HANDLE m_hCom);

/*! \fn com_rw
* \brief 获取传感器数据
* 
* \param data 获取到的传感器数据
* \param m_hCom 传感器串口句柄
* \return 获取成功与否
*/
BOOL com_rw(double data[], HANDLE m_hCom);

#ifdef __cplusplus
}
#endif

#endif //_COMM_H_
