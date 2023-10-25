#include "simulation.h"


 void Simulation::Admittance::intial_sim()
{
    if (disp_flag)
    {
        client_id = simxStart("127.0.0.1", 19999, true, true, 5000, 5);

        if (client_id != -1)
        {
            std::cout << "success\n";
        }
        else
        {
            std::cout << "failed\n";
        }
        simxGetObjectHandle(client_id, "joint_1", &handle[0], simx_opmode_blocking);
        simxGetObjectHandle(client_id, "joint_2", &handle[1], simx_opmode_blocking);
        simxGetObjectHandle(client_id, "joint_3", &handle[2], simx_opmode_blocking);
        simxGetObjectHandle(client_id, "joint_4", &handle[3], simx_opmode_blocking);
        simxGetObjectHandle(client_id, "joint_5", &handle[4], simx_opmode_blocking);
        simxGetObjectHandle(client_id, "joint_6", &handle[5], simx_opmode_blocking);
        for (int i = 0; i < 6; i++)
        {
            simxSetJointPosition(client_id, handle[i], 0, simx_opmode_streaming);
            std::cout << "Joint" << i << "'s position set: " << 0 << std::endl;
        }
        Sleep(8);
    }
}

 Simulation::Admittance::Admittance()
 {

     start_time = 0;
     time_gap = 0.008;
     b = 0.1 * Eigen::MatrixXd::Identity(6,6);
     m = b / 60;
     k = 0.5 * Eigen::MatrixXd::Identity(6, 6);
     last_move_time = 0;
     sim_tf = 10;
     dof_enabled = 0;
 }

 Eigen::MatrixXd  Simulation::Admittance::differential_eq(double t, Eigen::MatrixXd x)
 {
     Eigen::Array<double, 12, 1> dot_x_arr;
     dot_x_arr.block(0, 0, 6, 1) = x.block(6, 0, 6, 1);
     Eigen::MatrixXd f_error = traj->GetFd(t) - traj->GetFr(t);
     if (debug_flag)
     {
         std::cout << "k = " << k << std::endl;
         std::cout << "f_error = " << f_error.transpose() << std::endl;
     }
     dot_x_arr.block(6, 0, 6, 1) = m.fullPivLu().solve((f_error - b * x.block(6, 0, 6, 1) - k * x.block(0, 0, 6, 1)));
     return Eigen::MatrixXd(dot_x_arr);
 }


void  Simulation::Admittance::sim_rk4()
{
    double t = start_time;

    Eigen::MatrixXd xds(18, 1), xd_(6, 1), xd_dot(6, 1), xd_ddot(6, 1);
    Eigen::MatrixXd k1(12, 1), k2(12, 1), k3(12, 1), k4(12, 1);

    xds = traj->GetXd(t);
    xd_ = xds.block(0, 0, 6, 1);
    xd_dot = xds.block(6, 0, 6, 1);
    xd_ddot = xds.block(12, 0, 6, 1);
    Eigen::MatrixXd xr_(6, 1), xr_dot = Eigen::MatrixXd::Zero(6, 1);

    Eigen::MatrixXd last_q(6, 1), q(6, 1);

    Eigen::MatrixXd last_tmat = Kinematics::tw2tmat(xd_), tmat;
    last_q = rk.InvKinematics_1(last_tmat);
    
    if (disp_flag)
    {
        for (int i = 0; i < 6; i++)
        {
            simxSetJointPosition(client_id, handle[i], last_q(i), simx_opmode_blocking);
            std::cout << "Joint" << i << "'s position set: " << last_q(i) << std::endl;
        }
    }

    xr_ = Kinematics::tmat2tw(last_tmat);
    Eigen::Array<double, 12, 1> x_err_arr;
    Eigen::MatrixXd x_err(12, 1), x_err_next(12, 1);

    x_err_arr.block(0, 0, 6, 1) = Kinematics::add_tw(xd_, -xr_);
    x_err_arr.block(6, 0, 6, 1) = xd_dot - xr_dot;
    
    x_err = Eigen::MatrixXd(x_err_arr);

    if (debug_flag)
    {
        std::cout << "xds: " << xds.transpose() << " at time = " << t << std::endl;
        std::cout << "pos: " << xr_.transpose() << xr_dot.transpose() << " at time = " << t << std::endl;
    }
    while (t < sim_tf)
    {
        k1 = differential_eq(t, x_err);
        k2 = differential_eq(t + time_gap / 2, x_err + time_gap * k1 / 2);
        k3 = differential_eq(t + time_gap / 2, x_err + time_gap * k2 / 2);
        k4 = differential_eq(t + time_gap, x_err + time_gap * k3);
        x_err = x_err + time_gap * (k1 + 2 * k2 + 2 * k3 + k4) / 6;

        xds = traj->GetXd(t + time_gap);
        xd_ = xds.block(0, 0, 6, 1);
        xd_dot = xds.block(6, 0, 6, 1);
        xd_ddot = xds.block(12, 0, 6, 1);

        if (debug_flag)
        {
            std::cout << "x_err: " << x_err.transpose() << " at time = " << t << std::endl;
            std::cout << "xds: " << xds.transpose() << " at time = " << t << std::endl;
        }

        x_err_arr.block(0, 0, 6, 1) = Kinematics::add_tw(xd_, -x_err.block(0, 0, 6, 1));
        x_err_arr.block(6, 0, 6, 1) = xd_dot - x_err.block(6, 0, 6, 1);
        
        for (int i = 0; i < 6; i++)
        {
            if ((dof_enabled >> i) & 1)
            {
                x_err_arr(i, 0) = xd_(i);
            }
        }

        t += time_gap;

        tmat = Kinematics::tw2tmat(x_err_arr.block(0, 0, 6, 1));

        q = rk.InvKinematicsChoose(last_q, last_tmat, tmat);
        if (debug_flag)
        {
            std::cout << "x_err_arr: " << x_err_arr.transpose() << " at time = " << t << std::endl;
            std::cout << "pos: " << x_err_arr.transpose() << " at time = " << t << std::endl;
            std::cout << "all solution: \n" << rk.InverseKinematics(tmat) << std::endl;
            std::cout << "joint position: " << q.transpose() << std::endl;
            std::cout << "end pos: " << rk.ForwardKinematics(q) << "at time = " << t << std::endl;
        }

        last_q = q;
        last_tmat = tmat;

        if (disp_flag)
        {
            while (GetTickCount64() < last_move_time + 8)
                ;
            for (int i = 0; i < 6; i++)
            {
                if (dynamics_enabled)
                {
                    simxSetJointTargetPosition(client_id, handle[i], last_q(i), simx_opmode_streaming);
                }
                else
                {
                    simxSetJointPosition(client_id, handle[i], last_q(i), simx_opmode_streaming);
                }
                std::cout << "Joint" << i << "'s position set: " << last_q(i) << std::endl;
            }
            last_move_time = GetTickCount64();
        }
    }

    if (disp_flag)
        simxFinish(client_id);
}

void Simulation::Admittance::set_traj_gen(TrajGen_base* tr)
{
    traj = tr;
}

void Simulation::Admittance::set_portions(double mp, double bp, double kp, int tag)
{
    m(tag, tag) = mp;
    b(tag, tag) = bp;
    k(tag, tag) = kp;
}

void Simulation::Admittance::set_disp_mode(bool disp)
{
    disp_flag = disp;
    if (client_id != -1)
        simxFinish(client_id);
    intial_sim();
}

void Simulation::Admittance::set_dof_enble_flag(unsigned short a)
{
    dof_enabled = a;
}

void Simulation::Admittance::set_dynamic(bool flag)
{
    dynamics_enabled = flag;
}
