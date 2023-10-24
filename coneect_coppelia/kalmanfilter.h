/*! \file kalmanfilter.h
    \brief 本头文件定义卡尔曼滤波器类kalman

    版本：1.0

    \author 谭正
    \date 2023.10.24
*/
#ifndef KALMANFILTER_H
#define KALMANFILTER_H
#pragma once

#include <Eigen/Dense>

class kalman
{
private:
    Eigen::MatrixXd last_estimated, last_estimated_cov;
    /*! \brief the dynamic model defined in the Kalman filter
    *
    *   x(k+1) = Ax(k) + Bu(k)
    *   y(k) = Hx(k)
    */
    Eigen::MatrixXd A, B, H;
    Eigen::MatrixXd measure_cov;
public:
    kalman(/* args */);
    ~kalman();
};

#endif