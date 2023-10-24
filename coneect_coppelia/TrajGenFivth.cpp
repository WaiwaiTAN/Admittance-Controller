#include "TrajGenFivth.h"
#include <iostream>

TrajGenFivth::TrajGenFivth() : TrajGen_base()
{
    fs = freopen("freopen.out", "w", stderr);
    h = com_config();
    fe_mat = Eigen::MatrixXd::Zero(6, 1);
    for (int i = 0; i < 6; i++)
    {
        fe[i] = -1;
    }
    com_reset(h);
    fe_offset[0] = -0.3208;
    fe_offset[1] = -0.3082;
    fe_offset[2] = -0.3913;
    fe_offset[3] = -0.0135;
    fe_offset[4] = -0.0099;
    fe_offset[5] = -0.0091;

    threholds[0] = 0.1433;
    threholds[1] = 0.1590;
    threholds[2] = 0.1529;
    threholds[3] = 0.3858e-2;
    threholds[4] = 0.4158e-2;
    threholds[5] = 0.3538e-2;
}

TrajGenFivth::~TrajGenFivth()
{
    fclose(fs);
}

void TrajGenFivth::SetCoefs(Eigen::MatrixXd from, Eigen::MatrixXd to, double t0i, double tfi)
{
    t0 = t0i;
    tf = tfi;
    Eigen::Array<double, 6, 6> b, A;
    b.col(0) = from.block(0, 0, 6, 1);
    b.col(1) = from.block(6, 0, 6, 1);
    b.col(2) = Eigen::Array<double, 6, 1>::Zero(6, 1);
    b.col(3) = to.block(0, 0, 6, 1);
    b.col(4) = to.block(6, 0, 6, 1);
    b.col(5) = Eigen::Array<double, 6, 1>::Zero(6, 1);

    A(0, 0) = 1;
    A(0, 1) = 0;
    A(0, 2) = 0;
    A(1, 2) = 0;
    A(0, 3) = 1;
    A(0, 4) = 0;
    A(0, 5) = 0;
    A(1, 5) = 0;
    for (int i = 1; i < 6; i++)
    {
        A(i, 0) = A((int)i - 1, 0) * t0i;
        A(i, 3) = A((int)i - 1, 3) * tfi;
    }
    for (int i = 1; i < 6; i++)
    {
        A(i, 1) = A((int)i - 1, 0) * i;
        A(i, 4) = A((int)i - 1, 3) * i;
    }
    for (int i = 2; i < 6; i++)
    {
        A(i, 2) = A((int)i - 1, 1) * i;
        A(i, 5) = A((int)i - 1, 4) * i;
    }
    Eigen::MatrixXd coefs_transpose = ((Eigen::MatrixXd)A.transpose()).fullPivLu().solve((Eigen::MatrixXd)b.transpose());
    coefs = coefs_transpose.transpose();
}


Eigen::MatrixXd TrajGenFivth::GetXd(double time)
{
    if (time < t0)
    {
        return GetXd(t0);
    }
    
    if (time > tf)
    {
        return GetXd(tf);
    }

    Eigen::Array<double, 18, 1> ans(18, 1);
    Eigen::MatrixXd t_vec(6, 1);
    Eigen::MatrixXd t_vec_d(6, 1);
    Eigen::MatrixXd t_vec_dd(6, 1);
    t_vec(0) = 1;
    t_vec_d(0) = 0;
    t_vec_dd(0) = 0;
    t_vec_dd(1) = 0;
    for (int i = 1; i < 6; i++)
    {
        t_vec(i) = t_vec((int)i - 1) * time;
    }
    for (int i = 1; i < 6; i++)
    {
        t_vec_d(i) = t_vec((int)i - 1) * i;
    }
    for (int i = 2; i < 6; i++)
    {
        t_vec_dd(i) = t_vec_d((int)i - 1) * i;
    }
    ans.block(0, 0, 6, 1) = coefs * t_vec;
    ans.block(6, 0, 6, 1) = coefs * t_vec_d;
    ans.block(12, 0, 6, 1) = coefs * t_vec_dd;
    return ans;
}


Eigen::MatrixXd  TrajGenFivth::GetFd(double time)
{
    Eigen::MatrixXd ans(6, 1);
    ans << 0, 0, 0, 0, 0, 0;
    return ans;
}

#define sensor_not_enabled
//#define debug_enabled
Eigen::MatrixXd TrajGenFivth::GetFr(double t)
{
#ifdef sensor_not_enabled
    return Eigen::MatrixXd::Zero(6, 1);
#endif

    if (com_rw(fe, h))
    {
        for (int i = 0; i < 6; i++)
        {
            fe_mat(i) = fe[i] - fe_offset[i];
            if (abs(fe_mat(i)) < threholds[i])
            {
                fe_mat(i) = 0;
            }
        }
    }
#ifdef debug_enabled
    std::cerr << fe_mat.transpose() << std::endl;
#endif //  debug_enabled

    return fe_mat;
}

void TrajGenFivth::SetThrehold(double thre[])
{
    for (int i = 0; i < 6; i++)
    {
        threholds[i] = thre[i];
    }
}

void TrajGenFivth::SetOffset(double off[])
{
    for (int i = 0; i < 6; i++)
    {
        fe_offset[i] = off[i];
    }
}