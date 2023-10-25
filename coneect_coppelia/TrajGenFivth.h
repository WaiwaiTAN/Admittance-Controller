/*! \file TrajGenFiv.h
    \brief 本头文件定义类TrajGenFivth

    版本：1.0

    \author 谭正
    \date 2021.05.27
*/
#pragma once
#include "trajgen.h"
#include "comm.h"
#include "kalmanfilter.h"

/*!\brief 五次多项式轨迹生成类
* 
* 该类负责计算期望运动轨迹和力
* 也可以读取传感器末端的力
*/
class TrajGenFivth :
    public TrajGen_base
{
private:
    /*! \brief 五次多项式轨迹规划的系数矩阵
    */
    Eigen::MatrixXd coefs;

    /*! \brief 传感器串口句柄*/
    HANDLE h;

    /*! \brief 错误信息文件句柄*/
    FILE* fs;

    /*! \brief 传感器数据*/
    double fe[6];

    /*! \brief 传感器数据（矩阵类型）*/
    Eigen::MatrixXd fe_mat;

    kalman kal;

public:
    /*! \brief 五次多项式轨迹生成类构造函数
    *
    * 对传感器的偏置值和阈值做默认设定
    */
    TrajGenFivth();

    /*! \brief 五次多项式轨迹生成类析构函数
    */
    ~TrajGenFivth();

    /*! \brief 设置五次多项式轨迹生成的系数矩阵
    *
    * 根据给定的初始/终止位姿和初始/终止时刻计算五次多项式轨迹生成时需要用到的系数矩阵
    *
    * \param from 初始时刻的位置姿态坐标值，使用6-1的向量表示，Eigen::MatixXd类型
    * \param to 终止时刻的位置姿态坐标值，表示方式同from
    * \param t0 初始时刻的时间
    * \param tf 终止时刻的时间
    */
    void SetCoefs(Eigen::MatrixXd from, Eigen::MatrixXd to, double t0, double tf);

    /*! \brief 计算当前的期望位置姿态坐标值
    *
    * \param time 当前时间
    * \return 当前时刻的期望位姿坐标及其速度和加速度，使用18-1的向量表示，Eigen::MatrixXd类型，其中前六个是坐标，中间六个是速度，后面六个是加速度
    */
    Eigen::MatrixXd GetXd(double time);

    /*! \brief 计算当前的期望力\力矩坐标值
    *
    * \param time 当前时间
    * \return 当前时刻的期望力\力矩坐标值，使用6-1的向量表示，Eigen::MatrixXd类型，其中前三个是力，后三个是力矩，顺序皆为xyz
    */
    Eigen::MatrixXd GetFd(double time);

    /*! \brief 计算当前的末端受力
    *
    * \param time 当前时间
    * \return 当前时刻的末端传感器力\力矩坐标值，使用6-1的向量表示，Eigen::MatrixXd类型，其中前三个是力，后三个是力矩，顺序皆为xyz
    */
    Eigen::MatrixXd GetFr(double time);
};

