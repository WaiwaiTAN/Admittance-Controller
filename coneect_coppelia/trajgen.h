/*! \file trajgen.h
    \brief 本头文件定义基类TrajGen_base

    版本：1.0

    \author 谭正
    \date 2021.05.20
*/
#pragma once
#include <Eigen/Dense>

/*! \brief 轨迹生成基类
   *
   * 本基类中以纯虚函数的形式定义了轨迹生成类和控制仿真类的接口
   */
class TrajGen_base
{
public:
    /*! \brief TrajGen_base构造函数
    *
    * TrajGen_base默认构造函数，初始化轨迹规划的初始时刻和终止时刻为0和1
    */
    TrajGen_base() { t0 = 0; tf = 1; }

    /*! \brief TrajectoryGen析构函数
    *
    * TrajectoryGen默认析构函数
    */
    ~TrajGen_base() {}

    /*! \brief 计算当前的期望位置姿态坐标值
    *
    * \param time 当前时间
    * \return 当前时刻的期望位姿坐标及其速度和加速度，使用18-1的向量表示，Eigen::MatrixXd类型，其中前六个是坐标，中间六个是速度，后面六个是加速度
    */
    virtual Eigen::MatrixXd GetXd(double time) = 0;

    /*! \brief 计算当前的期望力\力矩坐标值
    *
    * \param time 当前时间
    * \return 当前时刻的期望力\力矩坐标值，使用6-1的向量表示，Eigen::MatrixXd类型，其中前三个是力，后三个是力矩，顺序皆为xyz
    */
    virtual Eigen::MatrixXd GetFd(double time) = 0;

    /*! \brief 计算当前的末端受力
    *
    * \param time 当前时间
    * \return 当前时刻的末端传感器力\力矩坐标值，使用6-1的向量表示，Eigen::MatrixXd类型，其中前三个是力，后三个是力矩，顺序皆为xyz
    */
    virtual Eigen::MatrixXd GetFr(double time) = 0;

    /*! \brief 轨迹规划的初始时刻
    */
    double t0;

    /*! \brief 轨迹规划的终止时刻*/
    double tf;
};