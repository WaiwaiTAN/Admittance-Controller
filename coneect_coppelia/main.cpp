#include "stdhaed.h"
void compute_offset()
{
    TrajGenFivth trg;
    auto now = GetTickCount();
    for (size_t i = 0; i < 2000; i++)
    {
        trg.GetFr(1);
    }
    auto time_consumed = GetTickCount() - now;
    std::cout << time_consumed;
}

void adcontrol_test()
{
    freopen("log.txt", "w", stdout);
    Eigen::MatrixXd from_pos(12, 1), to_pos(12, 1);

    to_pos << 500, 116.25, 385, 0, 0, M_PI_2, 0, 0, 0, 0, 0, 0;
    from_pos << 250, 100, 350, M_PI_2, 0, 0, 0, 0, 0, 0, 0, 0;
    TrajGenFivth trg;
    trg.SetCoefs(from_pos, to_pos, 0, 1);
    Simulation::Admittance as;
    as.set_traj_gen(&trg);
    as.set_debug_mode(true);
    as.set_disp_mode(true);
    auto a = 0b111110;
    as.set_dof_enble_flag(a);
    as.sim_rk4();
}


void invkine()
{
    freopen("log.txt", "w", stdout);
    Kinematics::Robot rbt;
    Eigen::MatrixXd to_pos(6, 1);
    to_pos << 500, 116.25, 385, 0, 0, M_PI_2;
    Eigen::MatrixXd qs = rbt.InverseKinematics(Kinematics::tw2tmat(to_pos)), tmat = rbt.ForwardKinematics(qs.block(0, 0, 6, 1));
    std::cout << "desired transformation matrix:\n" << tmat << std::endl;
    std::cout << "analysis solution:\n" << rbt.InverseKinematics(tmat) << std::endl;
    rbt.InvKineNumberic(Eigen::MatrixXd::Random(6, 1) * 0.2 + qs.block(0, 0, 6, 1), tmat, 0.1);
}

int main(int argc, char** argv)
{
    invkine();
    return 0;
}

