//#include "trajgen.h"
//
///*! \brief 设置五次多项式轨迹生成的系数矩阵
//*
//* 根据给定的初始/终止位姿和初始/终止时刻计算五次多项式轨迹生成时需要用到的系数矩阵
//*
//* \param from 初始时刻的位置姿态坐标值，使用6-1的向量表示，Eigen::MatixXd类型
//* \param to 终止时刻的位置姿态坐标值，表示方式同from
//* \param t0 初始时刻的时间
//* \param tf 终止时刻的时间
//*/
//void SetCoefs(Eigen::MatrixXd from, Eigen::MatrixXd to, double t0, double tf);
//void TrajGen_base::SetCoefs(Eigen::MatrixXd from, Eigen::MatrixXd to, double t0i, double tfi)
//{
//    t0 = t0i;
//    tf = tfi;
//    Eigen::Array<double, 6, 6> b, A;
//    b.col(0) = from.block(0, 0, 6, 1);
//    b.col(1) = from.block(6, 0, 6, 1);
//    b.col(2) = Eigen::Array<double, 6, 1>::Zero(6, 1);
//    b.col(3) = to.block(0, 0, 6, 1);
//    b.col(4) = to.block(6, 0, 6, 1);
//    b.col(5) = Eigen::Array<double, 6, 1>::Zero(6, 1);
//
//    A(0, 0) = 1;
//    A(0, 1) = 0;
//    A(0, 2) = 0;
//    A(1, 2) = 0;
//    A(0, 3) = 1;
//    A(0, 4) = 0;
//    A(0, 5) = 0;
//    A(1, 5) = 0;
//    for (int i = 1; i < 6; i++)
//    {
//        A(i, 0) = A((int)i - 1, 0) * t0i;
//        A(i, 3) = A((int)i - 1, 3) * tfi;
//    }
//    for (int i = 1; i < 6; i++)
//    {
//        A(i, 1) = A((int)i - 1, 0) * i;
//        A(i, 4) = A((int)i - 1, 3) * i;
//    }
//    for (int i = 2; i < 6; i++)
//    {
//        A(i, 2) = A((int)i - 1, 1) * i;
//        A(i, 5) = A((int)i - 1, 4) * i;
//    }
//    Eigen::MatrixXd coefs_transpose = ((Eigen::MatrixXd)A.transpose()).fullPivLu().solve((Eigen::MatrixXd)b.transpose());
//    coefs = coefs_transpose.transpose();
//}
//
//Eigen::MatrixXd TrajGen_base::GetXd(double time)
//{
//    if (time < t0)
//    {
//        return GetXd(t0);
//    }
//    
//    if (time > tf)
//    {
//        return GetXd(tf);
//    }
//
//    Eigen::Array<double, 18, 1> ans(18, 1);
//    Eigen::MatrixXd t_vec(6, 1);
//    Eigen::MatrixXd t_vec_d(6, 1);
//    Eigen::MatrixXd t_vec_dd(6, 1);
//    t_vec(0) = 1;
//    t_vec_d(0) = 0;
//    t_vec_dd(0) = 0;
//    t_vec_dd(1) = 0;
//    for (int i = 1; i < 6; i++)
//    {
//        t_vec(i) = t_vec((int)i - 1) * time;
//    }
//    for (int i = 1; i < 6; i++)
//    {
//        t_vec_d(i) = t_vec((int)i - 1) * i;
//    }
//    for (int i = 2; i < 6; i++)
//    {
//        t_vec_dd(i) = t_vec_d((int)i - 1) * i;
//    }
//    ans.block(0, 0, 6, 1) = coefs * t_vec;
//    ans.block(6, 0, 6, 1) = coefs * t_vec_d;
//    ans.block(12, 0, 6, 1) = coefs * t_vec_dd;
//    return ans;
//}
//
//
//Eigen::MatrixXd  TrajGen_base::GetFd(double time)
//{
//    Eigen::MatrixXd ans(6, 1);
//    ans << 0, 0, 0, 0, 0, 0;
//    return ans;
//}
//
//Eigen::MatrixXd TrajGen_base::GetFr(double t)
//{
//    auto fs = freopen("freopen.out", "w", stderr);
//    double fe[6];
//    Eigen::MatrixXd fe_mat(6, 1);
//    auto h = com_config();
//    com_reset(h);
//
//    if (com_rw(fe, h))
//    {
//        for (int i = 0; i < 6; i++)
//        {
//            fe_mat(i) = fe[i];
//        }
//    }
//
//    fclose(fs);
//    return fe_mat;
//}
//
