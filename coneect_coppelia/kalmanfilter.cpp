#include "kalmanfilter.h"

kalman::kalman(int dimx, int dimu, int dimz)
{
    last_estimated.resize(dimx, 1);
    last_estimated_cov.resize(dimx, dimx);
    
    A.resize(dimx, dimx);
    B.resize(dimx, dimu);
    H.resize(dimz, dimx);

    estimated_x.resize(dimx, 1);
    estimated_cov.resize(dimx, dimx);

    Q.resize(dimx, dimx);
    R.resize(dimz, dimz);

    x_dim = dimx;
    u_dim = dimu;
    z_dim = dimz;
}

kalman::~kalman()
{
}

bool kalman::setModel(const Eigen::MatrixXd &a, const Eigen::MatrixXd &b, const Eigen::MatrixXd &h)
{
    if (a.cols() != x_dim || a.rows() != x_dim)
    {
        return false;
    }

    if (b.cols() != u_dim || b.rows() != x_dim)
    {
        return false;
    }

    if (h.cols() != x_dim || h.rows() != z_dim)
    {
        return false;
    }

    A = a;
    B = b;
    H = h;
    return true;
}

bool kalman::setCov(const Eigen::MatrixXd &q, const Eigen::MatrixXd &r)
{
    if (q.cols() != x_dim || q.rows() != x_dim)
    {
        return false;
    }

    if (r.cols() != z_dim || r.rows() != z_dim)
    {
        return false;
    }

    Q = q;
    R = r;
    return true;
}

bool kalman::setInitEstimate(const Eigen::MatrixXd &x, const Eigen::MatrixXd &cov)
{
    if (x.cols() != 1 || x.rows() != x_dim)
    {
        return false;
    }

    if (cov.cols() != x_dim || cov.rows() != x_dim)
    {
        return false;
    }

    last_estimated = x;
    last_estimated_cov = cov;
    return true;
}

Eigen::MatrixXd kalman::get_opt_state(const Eigen::MatrixXd &uk, const Eigen::MatrixXd &zk)
{
    estimated_x = A * last_estimated + B * uk;
    estimated_cov = A * last_estimated_cov * A.transpose() + Q;

    K = (H * estimated_cov * H.transpose() + R).transpose().fullPivHouseholderQr().solve((estimated_cov * H.transpose()).transpose()).transpose();
    last_estimated = estimated_x + K * (zk - H * estimated_x);
    last_estimated_cov = (Eigen::MatrixXd::Identity(x_dim, x_dim) - K * H) * estimated_cov;
    return last_estimated;
}

#ifdef _UNIT_TEST_KAL_
#include <iostream>

void unit_test()
{
    kalman k(2, 2, 2);
    Eigen::MatrixXd a(2, 2);
    a << 1, 1, 0, 1;

    k.setModel(a, Eigen::MatrixXd::Identity(2, 2), Eigen::MatrixXd::Identity(2, 2));
    k.setCov(Eigen::MatrixXd::Identity(2, 2) * 0.1, Eigen::MatrixXd::Identity(2, 2));
    Eigen::MatrixXd x0(2, 1);
    x0 << 0, 1;
    k.setInitEstimate(x0, Eigen::MatrixXd::Identity(2, 2));
    Eigen::MatrixXd zk[5];

    zk[0].resize(2, 1);
    zk[0] << 2.775, 0.366;

    zk[1].resize(2, 1);
    zk[1] << 1.716, -0.536;

    zk[2].resize(2, 1);
    zk[2] << 1.528, 0.530;

    zk[3].resize(2, 1);
    zk[3] << 2.639, 0.680;

    zk[4].resize(2, 1);
    zk[4] << 1.568, 1.493;

    for (int i = 0; i < 5; i++)
    {
        std::cout << "\ni = " << i << std::endl;
        Eigen::MatrixXd tmp = k.get_opt_state(Eigen::MatrixXd::Zero(2, 1), zk[i]);
        std::cout << tmp;
    }
}

int main()
{
    std::cout << "test begins:" << std::endl;
    unit_test();
    return 0;
}

#endif