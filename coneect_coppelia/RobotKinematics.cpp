#include "RobotKinematics.h"
#include <iostream>

Eigen::MatrixXd Kinematics::dhLinkTransMat(double a, double alpha, double d, double theta)
{
	double trans[] = { 0, 0, d };
	Eigen::MatrixXd m = translateTransMat(trans);
	m = m * rotzTransMat(theta);
	trans[0] = a;
	trans[2] = 0;
	m = m * translateTransMat(trans);
	m = m * rotxTransMat(alpha);
	return m;
}

Eigen::MatrixXd Kinematics::rotxTransMat(double theta)
{
	Eigen::MatrixXd trans_mat(4, 4);
	double ct = cos(theta), st = sin(theta);
	trans_mat(2, 2) = ct;
	trans_mat(2, 1) = st;
	trans_mat(1, 2) = -st;
	trans_mat(1, 1) = ct;
	for (int i = 0; i < 3; i++)
	{
		trans_mat(3, i) = 0;
		trans_mat(i, 3) = 0;
	}
	trans_mat(0, 1) = 0;
	trans_mat(0, 2) = 0;
	trans_mat(2, 0) = 0;
	trans_mat(1, 0) = 0;
	trans_mat(0, 0) = 1;
	trans_mat(3, 3) = 1;
	return trans_mat;
}

Eigen::MatrixXd Kinematics::rotyTransMat(double theta)
{
	Eigen::MatrixXd trans_mat(4, 4);
	double ct = cos(theta), st = sin(theta);
	trans_mat(2, 2) = ct;
	trans_mat(2, 0) = -st;
	trans_mat(0, 2) = st;
	trans_mat(0, 0) = ct;
	for (int i = 0; i < 3; i++)
	{
		trans_mat(3, i) = 0;
		trans_mat(i, 3) = 0;
	}
	trans_mat(0, 1) = 0;
	trans_mat(2, 1) = 0;
	trans_mat(1, 2) = 0;
	trans_mat(1, 0) = 0;
	trans_mat(1, 1) = 1;
	trans_mat(3, 3) = 1;
	return trans_mat;
}

Eigen::MatrixXd Kinematics::rotzTransMat(double theta)
{
	Eigen::MatrixXd trans_mat(4, 4);
	double ct = cos(theta), st = sin(theta);
	trans_mat(0, 0) = ct;
	trans_mat(0, 1) = -st;
	trans_mat(1, 0) = st;
	trans_mat(1, 1) = ct;
	for (int i = 0; i < 3; i++)
	{
		trans_mat(3, i) = 0;
		trans_mat(i, 3) = 0;
	}
	trans_mat(2, 0) = 0;
	trans_mat(2, 1) = 0;
	trans_mat(0, 2) = 0;
	trans_mat(1, 2) = 0;
	trans_mat(2, 2) = 1;
	trans_mat(3, 3) = 1;
	return trans_mat;
}

Eigen::MatrixXd Kinematics::translateTransMat(double trans[3])
{
	Eigen::MatrixXd m = Eigen::MatrixXd::Identity(4, 4);
	for (int i = 0; i < 3; i++)
	{
		m(i, 3) = trans[i];
	}
	return m;
}

Eigen::MatrixXd Kinematics::translateTransMat(Eigen::MatrixXd trans)
{
	Eigen::MatrixXd m = Eigen::MatrixXd::Identity(4, 4);
	for (int i = 0; i < 3; i++)
	{
		m(i, 3) = trans(i);
	}
	return m;
}

Eigen::MatrixXd Kinematics::Robot::Jacobe(double q[])
{
	Eigen::MatrixXd tmats[6], tmat_end = Eigen::MatrixXd::Identity(4, 4);
	Eigen::Vector3d z[6], p[6];
	Eigen::Array<double, 6, 6> jaco;
	for (int i = 0; i < 6; i++)
	{
		z[i] = tmat_end.block(0, 2, 3, 1);
		p[i] = tmat_end.block(0, 3, 3, 1);
		tmats[i] = dhLinkTransMat(dh_a[i], dh_alpha[i], dh_d[i], q[i]);
		tmat_end = tmat_end * tmats[i];
	}
	auto p_end = (Eigen::Vector3d)tmat_end.block(0, 3, 3, 1);
	auto r_end = tmat_end.topLeftCorner(3, 3);

	for (int i = 0; i < 6; i++)
	{
		jaco.block(0, i, 3, 1) = z[i].cross(p_end - p[i]);
		jaco.block(3, i, 3, 1) = z[i];
	}
	Eigen::Array<double, 6, 6> big_rmat = Eigen::Array<double, 6, 6>::Zero();
	big_rmat.block(0, 0, 3, 3) = r_end;
	big_rmat.block(3, 3, 3, 3) = r_end;
	return Eigen::MatrixXd(big_rmat).transpose() * Eigen::MatrixXd(jaco);
}

Eigen::MatrixXd Kinematics::Robot::Jacobe(Eigen::MatrixXd q)
{
	Eigen::MatrixXd tmats[6], tmat_end = Eigen::MatrixXd::Identity(4, 4);
	Eigen::Vector3d z[6], p[6];
	Eigen::Array<double, 6, 6> jaco;
	for (int i = 0; i < 6; i++)
	{
		z[i] = tmat_end.block(0, 2, 3, 1);
		p[i] = tmat_end.block(0, 3, 3, 1);
		tmats[i] = dhLinkTransMat(dh_a[i], dh_alpha[i], dh_d[i], q(i));
		tmat_end = tmat_end * tmats[i];
	}
	auto p_end = (Eigen::Vector3d)tmat_end.block(0, 3, 3, 1);
	auto r_end = tmat_end.topLeftCorner(3, 3);

	for (int i = 0; i < 6; i++)
	{
		jaco.block(0, i, 3, 1) = z[i].cross(p_end - p[i]);
		jaco.block(3, i, 3, 1) = z[i];
	}
	Eigen::Array<double, 6, 6> big_rmat = Eigen::Array<double, 6, 6>::Zero();
	big_rmat.block(0, 0, 3, 3) = r_end;
	big_rmat.block(3, 3, 3, 3) = r_end;
	return Eigen::MatrixXd(big_rmat).transpose() * Eigen::MatrixXd(jaco);
}

Eigen::MatrixXd Kinematics::Robot::ForwardKinematics(double q[])
{
	Eigen::MatrixXd m = Eigen::MatrixXd::Identity(4, 4);
	for (int i = 0; i < dof; i++)
	{
		m = m * dhLinkTransMat(dh_a[i], dh_alpha[i], dh_d[i], q[i]);
	}
	return m;
}

Eigen::MatrixXd Kinematics::Robot::ForwardKinematics(Eigen::MatrixXd q)
{
	Eigen::MatrixXd m = Eigen::MatrixXd::Identity(4, 4);
	for (int i = 0; i < dof; i++)
	{
		m = m * dhLinkTransMat(dh_a[i], dh_alpha[i], dh_d[i], q(i));
	}
	return m;
}

Eigen::MatrixXd Kinematics::Robot::InverseKinematics(Eigen::MatrixXd target_tmatrix)
{
	// This function gives an analytical solution and is therefore not applicable to robots in general.
	Eigen::MatrixXd ans(6, 8);
	double theta1_parax = target_tmatrix(0, 3) - dh_d[5] * target_tmatrix(0, 2);
	double theta1_paray = target_tmatrix(1, 3) - dh_d[5] * target_tmatrix(1, 2);
	double theta1_para[2];
	theta1_para[0] = atan2(theta1_paray, theta1_parax);
	theta1_para[1] = sqrt(theta1_parax * theta1_parax + theta1_paray * theta1_paray - dh_d[3] * dh_d[3]);
	for (int i = 0; i < 8; i++)
	{
		if (i < 4)
			ans(0,i) = theta1_para[0] - atan2(-dh_d[3], theta1_para[1]);
		else
			ans(0,i) = theta1_para[0] - atan2(-dh_d[3], -theta1_para[1]);
	}
	double theta5, theta3, s234, c234, N, M, A, B;
	Eigen::MatrixXd a_mat(2, 2), b_mat(2, 1), x_mat(2, 1);
	for (int i = 0; i < 8; i++)
	{
		theta5 = acos(target_tmatrix(0, 2) * sin(ans(0, i)) - target_tmatrix(1, 2) * cos(ans(0, i)));

		if ((i >> 1) & 1)
			ans(4, i) = theta5;
		else
			ans(4, i) = -theta5;

		if (ans(4, i) == 0.0)
		{
			// error, theta5 got a value of 0.0
			continue;
		}
		ans(5, i) = atan2((-target_tmatrix(0, 1) * sin(ans(0, i)) + target_tmatrix(1, 1) * cos(ans(0, i))) / sin(ans(4, i)), -(-target_tmatrix(0, 0) * sin(ans(0, i)) + target_tmatrix(1, 0) * cos(ans(0, i))) / sin(ans(4, i)));
		c234 = -(target_tmatrix(0, 2) * cos(ans(0, i)) + target_tmatrix(1, 2) * sin(ans(0, i))) / sin(ans(4, i));
		s234 = -target_tmatrix(2, 2) / sin(ans(4, i));
		double tmp = atan2(s234, c234);
		N = target_tmatrix(2, 3) - dh_d[0] - target_tmatrix(2, 2) * dh_d[5] + c234 * dh_d[4];
		M = cos(ans(0, i)) * target_tmatrix(0, 3) + sin(ans(0, i)) * target_tmatrix(1, 3) - dh_d[5] * (target_tmatrix(0, 2) * cos(ans(0, i)) + target_tmatrix(1, 2) * sin(ans(0, i))) - dh_d[4] * s234;
		theta3 = acos((M * M + N * N - dh_a[1] * dh_a[1] - dh_a[2] * dh_a[2]) / (2 * dh_a[1] * dh_a[2]));
		if (i & 1)
		{
			ans(2, i) = theta3;
		}
		else
		{
			ans(2, i) = -theta3;
		}
		A = dh_a[1] + dh_a[2] * cos(ans(2, i));
		B = dh_a[2] * sin(ans(2, i));
		
		a_mat(0, 0) = A;
		a_mat(0, 1) = -B;
		a_mat(1, 0) = B;
		a_mat(1, 1) = A;

		b_mat(0) = M;
		b_mat(1) = N;

		x_mat = a_mat.fullPivLu().solve(b_mat);
		ans(1, i) = atan2(x_mat(1), x_mat(0));
		ans(3, i) = atan2(s234, c234) - ans(1, i) - ans(2, i);

		for (int j = 0; j < 6; j++)
		{
			ans(j, i) = normaliz_to_2pi(ans(j, i));
		}
	}
	return ans;
}

Eigen::MatrixXd Kinematics::Robot::InvKinematics_1(Eigen::MatrixXd target_tmatrix)
{
	Eigen::MatrixXd ans(6, 1);
	double theta1_parax = target_tmatrix(0, 3) - dh_d[5] * target_tmatrix(0, 2);
	double theta1_paray = target_tmatrix(1, 3) - dh_d[5] * target_tmatrix(1, 2);
	double theta1_para[2];
	theta1_para[0] = atan2(theta1_paray, theta1_parax);
	theta1_para[1] = sqrt(theta1_parax * theta1_parax + theta1_paray * theta1_paray - dh_d[3] * dh_d[3]);
	ans(0) = theta1_para[0] - atan2(-dh_d[3], theta1_para[1]);
	double theta5, theta3, s234, c234, N, M, A, B;
	Eigen::MatrixXd a_mat(2, 2), b_mat(2, 1), x_mat(2, 1);
	theta5 = acos(target_tmatrix(0, 2) * sin(ans(0)) - target_tmatrix(1, 2) * cos(ans(0)));
	ans(4) = -theta5;
	if (ans(4) == 0.0)
	{
		// error, theta5 got a value of 0.0
		std::cerr << "error, theta5 got a value of 0.0";
		return ans;
	}
	ans(5) = atan2((-target_tmatrix(0, 1) * sin(ans(0)) + target_tmatrix(1, 1) * cos(ans(0))) / sin(ans(4)), -(-target_tmatrix(0, 0) * sin(ans(0)) + target_tmatrix(1, 0) * cos(ans(0))) / sin(ans(4)));
	c234 = -(target_tmatrix(0, 2) * cos(ans(0)) + target_tmatrix(1, 2) * sin(ans(0))) / sin(ans(4));
	s234 = -target_tmatrix(2, 2) / sin(ans(4));
	double tmp = atan2(s234, c234);
	N = target_tmatrix(2, 3) - dh_d[0] - target_tmatrix(2, 2) * dh_d[5] + c234 * dh_d[4];
	M = cos(ans(0)) * target_tmatrix(0, 3) + sin(ans(0)) * target_tmatrix(1, 3) - dh_d[5] * (target_tmatrix(0, 2) * cos(ans(0)) + target_tmatrix(1, 2) * sin(ans(0))) - dh_d[4] * s234;
	theta3 = acos((M * M + N * N - dh_a[1] * dh_a[1] - dh_a[2] * dh_a[2]) / (2 * dh_a[1] * dh_a[2]));
	ans(2) = -theta3;
	A = dh_a[1] + dh_a[2] * cos(ans(2));
	B = dh_a[2] * sin(ans(2));
	a_mat(0, 0) = A;
	a_mat(0, 1) = -B;
	a_mat(1, 0) = B;
	a_mat(1, 1) = A;
	b_mat(0) = M;
	b_mat(1) = N;
	x_mat = a_mat.fullPivLu().solve(b_mat);
	ans(1) = atan2(x_mat(1), x_mat(0));
	ans(3) = atan2(s234, c234) - ans(1) - ans(2);
	for (int j = 0; j < 6; j++)
	{
		ans(j) = normaliz_to_2pi(ans(j));
	}
	return ans;
}

Eigen::MatrixXd Kinematics::Robot::InvKinematics_2(Eigen::MatrixXd target_tmatrix)
{
	Eigen::MatrixXd ans(6, 1);
	double theta1_parax = target_tmatrix(0, 3) - dh_d[5] * target_tmatrix(0, 2);
	double theta1_paray = target_tmatrix(1, 3) - dh_d[5] * target_tmatrix(1, 2);
	double theta1_para[2];
	theta1_para[0] = atan2(theta1_paray, theta1_parax);
	theta1_para[1] = sqrt(theta1_parax * theta1_parax + theta1_paray * theta1_paray - dh_d[3] * dh_d[3]);
	ans(0) = theta1_para[0] - atan2(-dh_d[3], theta1_para[1]);
	double theta5, theta3, s234, c234, N, M, A, B;
	Eigen::MatrixXd a_mat(2, 2), b_mat(2, 1), x_mat(2, 1);
	theta5 = acos(target_tmatrix(0, 2) * sin(ans(0)) - target_tmatrix(1, 2) * cos(ans(0)));
	ans(4) = -theta5;
	if (ans(4) == 0.0)
	{
		// error, theta5 got a value of 0.0
		std::cerr << "error, theta5 got a value of 0.0";
		return ans;
	}
	ans(5) = atan2((-target_tmatrix(0, 1) * sin(ans(0)) + target_tmatrix(1, 1) * cos(ans(0))) / sin(ans(4)), -(-target_tmatrix(0, 0) * sin(ans(0)) + target_tmatrix(1, 0) * cos(ans(0))) / sin(ans(4)));
	c234 = -(target_tmatrix(0, 2) * cos(ans(0)) + target_tmatrix(1, 2) * sin(ans(0))) / sin(ans(4));
	s234 = -target_tmatrix(2, 2) / sin(ans(4));
	double tmp = atan2(s234, c234);
	N = target_tmatrix(2, 3) - dh_d[0] - target_tmatrix(2, 2) * dh_d[5] + c234 * dh_d[4];
	M = cos(ans(0)) * target_tmatrix(0, 3) + sin(ans(0)) * target_tmatrix(1, 3) - dh_d[5] * (target_tmatrix(0, 2) * cos(ans(0)) + target_tmatrix(1, 2) * sin(ans(0))) - dh_d[4] * s234;
	theta3 = acos((M * M + N * N - dh_a[1] * dh_a[1] - dh_a[2] * dh_a[2]) / (2 * dh_a[1] * dh_a[2]));
	ans(2) = theta3;
	A = dh_a[1] + dh_a[2] * cos(ans(2));
	B = dh_a[2] * sin(ans(2));
	a_mat(0, 0) = A;
	a_mat(0, 1) = -B;
	a_mat(1, 0) = B;
	a_mat(1, 1) = A;
	b_mat(0) = M;
	b_mat(1) = N;
	x_mat = a_mat.fullPivLu().solve(b_mat);
	ans(1) = atan2(x_mat(1), x_mat(0));
	ans(3) = atan2(s234, c234) - ans(1) - ans(2);
	for (int j = 0; j < 6; j++)
	{
		ans(j) = normaliz_to_2pi(ans(j));
	}
	return ans;
}

Eigen::MatrixXd Kinematics::Robot::InvKinematics_3(Eigen::MatrixXd target_tmatrix)
{
	Eigen::MatrixXd ans(6, 1);
	double theta1_parax = target_tmatrix(0, 3) - dh_d[5] * target_tmatrix(0, 2);
	double theta1_paray = target_tmatrix(1, 3) - dh_d[5] * target_tmatrix(1, 2);
	double theta1_para[2];
	theta1_para[0] = atan2(theta1_paray, theta1_parax);
	theta1_para[1] = sqrt(theta1_parax * theta1_parax + theta1_paray * theta1_paray - dh_d[3] * dh_d[3]);
	ans(0) = theta1_para[0] - atan2(-dh_d[3], theta1_para[1]);
	double theta5, theta3, s234, c234, N, M, A, B;
	Eigen::MatrixXd a_mat(2, 2), b_mat(2, 1), x_mat(2, 1);
	theta5 = acos(target_tmatrix(0, 2) * sin(ans(0)) - target_tmatrix(1, 2) * cos(ans(0)));
	ans(4) = theta5;
	if (ans(4) == 0.0)
	{
		// error, theta5 got a value of 0.0
		std::cerr << "error, theta5 got a value of 0.0";
		return ans;
	}
	ans(5) = atan2((-target_tmatrix(0, 1) * sin(ans(0)) + target_tmatrix(1, 1) * cos(ans(0))) / sin(ans(4)), -(-target_tmatrix(0, 0) * sin(ans(0)) + target_tmatrix(1, 0) * cos(ans(0))) / sin(ans(4)));
	c234 = -(target_tmatrix(0, 2) * cos(ans(0)) + target_tmatrix(1, 2) * sin(ans(0))) / sin(ans(4));
	s234 = -target_tmatrix(2, 2) / sin(ans(4));
	double tmp = atan2(s234, c234);
	N = target_tmatrix(2, 3) - dh_d[0] - target_tmatrix(2, 2) * dh_d[5] + c234 * dh_d[4];
	M = cos(ans(0)) * target_tmatrix(0, 3) + sin(ans(0)) * target_tmatrix(1, 3) - dh_d[5] * (target_tmatrix(0, 2) * cos(ans(0)) + target_tmatrix(1, 2) * sin(ans(0))) - dh_d[4] * s234;
	theta3 = acos((M * M + N * N - dh_a[1] * dh_a[1] - dh_a[2] * dh_a[2]) / (2 * dh_a[1] * dh_a[2]));
	ans(2) = -theta3;
	A = dh_a[1] + dh_a[2] * cos(ans(2));
	B = dh_a[2] * sin(ans(2));
	a_mat(0, 0) = A;
	a_mat(0, 1) = -B;
	a_mat(1, 0) = B;
	a_mat(1, 1) = A;
	b_mat(0) = M;
	b_mat(1) = N;
	x_mat = a_mat.fullPivLu().solve(b_mat);
	ans(1) = atan2(x_mat(1), x_mat(0));
	ans(3) = atan2(s234, c234) - ans(1) - ans(2);
	for (int j = 0; j < 6; j++)
	{
		ans(j) = normaliz_to_2pi(ans(j));
	}
	return ans;
}

Eigen::MatrixXd Kinematics::Robot::InvKinematics_4(Eigen::MatrixXd target_tmatrix)
{
	Eigen::MatrixXd ans(6, 1);
	double theta1_parax = target_tmatrix(0, 3) - dh_d[5] * target_tmatrix(0, 2);
	double theta1_paray = target_tmatrix(1, 3) - dh_d[5] * target_tmatrix(1, 2);
	double theta1_para[2];
	theta1_para[0] = atan2(theta1_paray, theta1_parax);
	theta1_para[1] = sqrt(theta1_parax * theta1_parax + theta1_paray * theta1_paray - dh_d[3] * dh_d[3]);
	ans(0) = theta1_para[0] - atan2(-dh_d[3], theta1_para[1]);
	double theta5, theta3, s234, c234, N, M, A, B;
	Eigen::MatrixXd a_mat(2, 2), b_mat(2, 1), x_mat(2, 1);
	theta5 = acos(target_tmatrix(0, 2) * sin(ans(0)) - target_tmatrix(1, 2) * cos(ans(0)));
	ans(4) = theta5;
	if (ans(4) == 0.0)
	{
		// error, theta5 got a value of 0.0
		std::cerr << "error, theta5 got a value of 0.0";
		return ans;
	}
	ans(5) = atan2((-target_tmatrix(0, 1) * sin(ans(0)) + target_tmatrix(1, 1) * cos(ans(0))) / sin(ans(4)), -(-target_tmatrix(0, 0) * sin(ans(0)) + target_tmatrix(1, 0) * cos(ans(0))) / sin(ans(4)));
	c234 = -(target_tmatrix(0, 2) * cos(ans(0)) + target_tmatrix(1, 2) * sin(ans(0))) / sin(ans(4));
	s234 = -target_tmatrix(2, 2) / sin(ans(4));
	double tmp = atan2(s234, c234);
	N = target_tmatrix(2, 3) - dh_d[0] - target_tmatrix(2, 2) * dh_d[5] + c234 * dh_d[4];
	M = cos(ans(0)) * target_tmatrix(0, 3) + sin(ans(0)) * target_tmatrix(1, 3) - dh_d[5] * (target_tmatrix(0, 2) * cos(ans(0)) + target_tmatrix(1, 2) * sin(ans(0))) - dh_d[4] * s234;
	theta3 = acos((M * M + N * N - dh_a[1] * dh_a[1] - dh_a[2] * dh_a[2]) / (2 * dh_a[1] * dh_a[2]));
	ans(2) = theta3;
	A = dh_a[1] + dh_a[2] * cos(ans(2));
	B = dh_a[2] * sin(ans(2));
	a_mat(0, 0) = A;
	a_mat(0, 1) = -B;
	a_mat(1, 0) = B;
	a_mat(1, 1) = A;
	b_mat(0) = M;
	b_mat(1) = N;
	x_mat = a_mat.fullPivLu().solve(b_mat);
	ans(1) = atan2(x_mat(1), x_mat(0));
	ans(3) = atan2(s234, c234) - ans(1) - ans(2);
	for (int j = 0; j < 6; j++)
	{
		ans(j) = normaliz_to_2pi(ans(j));
	}
	return ans;
}

Eigen::MatrixXd Kinematics::Robot::InvKinematics_5(Eigen::MatrixXd target_tmatrix)
{
	Eigen::MatrixXd ans(6, 1);
	double theta1_parax = target_tmatrix(0, 3) - dh_d[5] * target_tmatrix(0, 2);
	double theta1_paray = target_tmatrix(1, 3) - dh_d[5] * target_tmatrix(1, 2);
	double theta1_para[2];
	theta1_para[0] = atan2(theta1_paray, theta1_parax);
	theta1_para[1] = sqrt(theta1_parax * theta1_parax + theta1_paray * theta1_paray - dh_d[3] * dh_d[3]);
	ans(0) = theta1_para[0] - atan2(-dh_d[3], -theta1_para[1]);
	double theta5, theta3, s234, c234, N, M, A, B;
	Eigen::MatrixXd a_mat(2, 2), b_mat(2, 1), x_mat(2, 1);
	theta5 = acos(target_tmatrix(0, 2) * sin(ans(0)) - target_tmatrix(1, 2) * cos(ans(0)));
	ans(4) = -theta5;
	if (ans(4) == 0.0)
	{
		// error, theta5 got a value of 0.0
		std::cerr << "error, theta5 got a value of 0.0";
		return ans;
	}
	ans(5) = atan2((-target_tmatrix(0, 1) * sin(ans(0)) + target_tmatrix(1, 1) * cos(ans(0))) / sin(ans(4)), -(-target_tmatrix(0, 0) * sin(ans(0)) + target_tmatrix(1, 0) * cos(ans(0))) / sin(ans(4)));
	c234 = -(target_tmatrix(0, 2) * cos(ans(0)) + target_tmatrix(1, 2) * sin(ans(0))) / sin(ans(4));
	s234 = -target_tmatrix(2, 2) / sin(ans(4));
	double tmp = atan2(s234, c234);
	N = target_tmatrix(2, 3) - dh_d[0] - target_tmatrix(2, 2) * dh_d[5] + c234 * dh_d[4];
	M = cos(ans(0)) * target_tmatrix(0, 3) + sin(ans(0)) * target_tmatrix(1, 3) - dh_d[5] * (target_tmatrix(0, 2) * cos(ans(0)) + target_tmatrix(1, 2) * sin(ans(0))) - dh_d[4] * s234;
	theta3 = acos((M * M + N * N - dh_a[1] * dh_a[1] - dh_a[2] * dh_a[2]) / (2 * dh_a[1] * dh_a[2]));
	ans(2) = -theta3;
	A = dh_a[1] + dh_a[2] * cos(ans(2));
	B = dh_a[2] * sin(ans(2));
	a_mat(0, 0) = A;
	a_mat(0, 1) = -B;
	a_mat(1, 0) = B;
	a_mat(1, 1) = A;
	b_mat(0) = M;
	b_mat(1) = N;
	x_mat = a_mat.fullPivLu().solve(b_mat);
	ans(1) = atan2(x_mat(1), x_mat(0));
	ans(3) = atan2(s234, c234) - ans(1) - ans(2);
	for (int j = 0; j < 6; j++)
	{
		ans(j) = normaliz_to_2pi(ans(j));
	}
	return ans;
}

Eigen::MatrixXd Kinematics::Robot::InvKinematics_6(Eigen::MatrixXd target_tmatrix)
{
	Eigen::MatrixXd ans(6, 1);
	double theta1_parax = target_tmatrix(0, 3) - dh_d[5] * target_tmatrix(0, 2);
	double theta1_paray = target_tmatrix(1, 3) - dh_d[5] * target_tmatrix(1, 2);
	double theta1_para[2];
	theta1_para[0] = atan2(theta1_paray, theta1_parax);
	theta1_para[1] = sqrt(theta1_parax * theta1_parax + theta1_paray * theta1_paray - dh_d[3] * dh_d[3]);
	ans(0) = theta1_para[0] - atan2(-dh_d[3], -theta1_para[1]);
	double theta5, theta3, s234, c234, N, M, A, B;
	Eigen::MatrixXd a_mat(2, 2), b_mat(2, 1), x_mat(2, 1);
	theta5 = acos(target_tmatrix(0, 2) * sin(ans(0)) - target_tmatrix(1, 2) * cos(ans(0)));
	ans(4) = -theta5;
	if (ans(4) == 0.0)
	{
		// error, theta5 got a value of 0.0
		std::cerr << "error, theta5 got a value of 0.0";
		return ans;
	}
	ans(5) = atan2((-target_tmatrix(0, 1) * sin(ans(0)) + target_tmatrix(1, 1) * cos(ans(0))) / sin(ans(4)), -(-target_tmatrix(0, 0) * sin(ans(0)) + target_tmatrix(1, 0) * cos(ans(0))) / sin(ans(4)));
	c234 = -(target_tmatrix(0, 2) * cos(ans(0)) + target_tmatrix(1, 2) * sin(ans(0))) / sin(ans(4));
	s234 = -target_tmatrix(2, 2) / sin(ans(4));
	double tmp = atan2(s234, c234);
	N = target_tmatrix(2, 3) - dh_d[0] - target_tmatrix(2, 2) * dh_d[5] + c234 * dh_d[4];
	M = cos(ans(0)) * target_tmatrix(0, 3) + sin(ans(0)) * target_tmatrix(1, 3) - dh_d[5] * (target_tmatrix(0, 2) * cos(ans(0)) + target_tmatrix(1, 2) * sin(ans(0))) - dh_d[4] * s234;
	theta3 = acos((M * M + N * N - dh_a[1] * dh_a[1] - dh_a[2] * dh_a[2]) / (2 * dh_a[1] * dh_a[2]));
	ans(2) = theta3;
	A = dh_a[1] + dh_a[2] * cos(ans(2));
	B = dh_a[2] * sin(ans(2));
	a_mat(0, 0) = A;
	a_mat(0, 1) = -B;
	a_mat(1, 0) = B;
	a_mat(1, 1) = A;
	b_mat(0) = M;
	b_mat(1) = N;
	x_mat = a_mat.fullPivLu().solve(b_mat);
	ans(1) = atan2(x_mat(1), x_mat(0));
	ans(3) = atan2(s234, c234) - ans(1) - ans(2);
	for (int j = 0; j < 6; j++)
	{
		ans(j) = normaliz_to_2pi(ans(j));
	}
	return ans;
}

Eigen::MatrixXd Kinematics::Robot::InvKinematics_7(Eigen::MatrixXd target_tmatrix)
{
	Eigen::MatrixXd ans(6, 1);
	double theta1_parax = target_tmatrix(0, 3) - dh_d[5] * target_tmatrix(0, 2);
	double theta1_paray = target_tmatrix(1, 3) - dh_d[5] * target_tmatrix(1, 2);
	double theta1_para[2];
	theta1_para[0] = atan2(theta1_paray, theta1_parax);
	theta1_para[1] = sqrt(theta1_parax * theta1_parax + theta1_paray * theta1_paray - dh_d[3] * dh_d[3]);
	ans(0) = theta1_para[0] - atan2(-dh_d[3], -theta1_para[1]);
	double theta5, theta3, s234, c234, N, M, A, B;
	Eigen::MatrixXd a_mat(2, 2), b_mat(2, 1), x_mat(2, 1);
	theta5 = acos(target_tmatrix(0, 2) * sin(ans(0)) - target_tmatrix(1, 2) * cos(ans(0)));
	ans(4) = theta5;
	if (ans(4) == 0.0)
	{
		// error, theta5 got a value of 0.0
		std::cerr << "error, theta5 got a value of 0.0";
		return ans;
	}
	ans(5) = atan2((-target_tmatrix(0, 1) * sin(ans(0)) + target_tmatrix(1, 1) * cos(ans(0))) / sin(ans(4)), -(-target_tmatrix(0, 0) * sin(ans(0)) + target_tmatrix(1, 0) * cos(ans(0))) / sin(ans(4)));
	c234 = -(target_tmatrix(0, 2) * cos(ans(0)) + target_tmatrix(1, 2) * sin(ans(0))) / sin(ans(4));
	s234 = -target_tmatrix(2, 2) / sin(ans(4));
	double tmp = atan2(s234, c234);
	N = target_tmatrix(2, 3) - dh_d[0] - target_tmatrix(2, 2) * dh_d[5] + c234 * dh_d[4];
	M = cos(ans(0)) * target_tmatrix(0, 3) + sin(ans(0)) * target_tmatrix(1, 3) - dh_d[5] * (target_tmatrix(0, 2) * cos(ans(0)) + target_tmatrix(1, 2) * sin(ans(0))) - dh_d[4] * s234;
	theta3 = acos((M * M + N * N - dh_a[1] * dh_a[1] - dh_a[2] * dh_a[2]) / (2 * dh_a[1] * dh_a[2]));
	ans(2) = -theta3;
	A = dh_a[1] + dh_a[2] * cos(ans(2));
	B = dh_a[2] * sin(ans(2));
	a_mat(0, 0) = A;
	a_mat(0, 1) = -B;
	a_mat(1, 0) = B;
	a_mat(1, 1) = A;
	b_mat(0) = M;
	b_mat(1) = N;
	x_mat = a_mat.fullPivLu().solve(b_mat);
	ans(1) = atan2(x_mat(1), x_mat(0));
	ans(3) = atan2(s234, c234) - ans(1) - ans(2);
	for (int j = 0; j < 6; j++)
	{
		ans(j) = normaliz_to_2pi(ans(j));
	}
	return ans;
}

Eigen::MatrixXd Kinematics::Robot::InvKinematics_8(Eigen::MatrixXd target_tmatrix)
{
	Eigen::MatrixXd ans(6, 1);
	double theta1_parax = target_tmatrix(0, 3) - dh_d[5] * target_tmatrix(0, 2);
	double theta1_paray = target_tmatrix(1, 3) - dh_d[5] * target_tmatrix(1, 2);
	double theta1_para[2];
	theta1_para[0] = atan2(theta1_paray, theta1_parax);
	theta1_para[1] = sqrt(theta1_parax * theta1_parax + theta1_paray * theta1_paray - dh_d[3] * dh_d[3]);
	ans(0) = theta1_para[0] - atan2(-dh_d[3], -theta1_para[1]);
	double theta5, theta3, s234, c234, N, M, A, B;
	Eigen::MatrixXd a_mat(2, 2), b_mat(2, 1), x_mat(2, 1);
	theta5 = acos(target_tmatrix(0, 2) * sin(ans(0)) - target_tmatrix(1, 2) * cos(ans(0)));
	ans(4) = theta5;
	if (ans(4) == 0.0)
	{
		// error, theta5 got a value of 0.0
		std::cerr << "error, theta5 got a value of 0.0";
		return ans;
	}
	ans(5) = atan2((-target_tmatrix(0, 1) * sin(ans(0)) + target_tmatrix(1, 1) * cos(ans(0))) / sin(ans(4)), -(-target_tmatrix(0, 0) * sin(ans(0)) + target_tmatrix(1, 0) * cos(ans(0))) / sin(ans(4)));
	c234 = -(target_tmatrix(0, 2) * cos(ans(0)) + target_tmatrix(1, 2) * sin(ans(0))) / sin(ans(4));
	s234 = -target_tmatrix(2, 2) / sin(ans(4));
	double tmp = atan2(s234, c234);
	N = target_tmatrix(2, 3) - dh_d[0] - target_tmatrix(2, 2) * dh_d[5] + c234 * dh_d[4];
	M = cos(ans(0)) * target_tmatrix(0, 3) + sin(ans(0)) * target_tmatrix(1, 3) - dh_d[5] * (target_tmatrix(0, 2) * cos(ans(0)) + target_tmatrix(1, 2) * sin(ans(0))) - dh_d[4] * s234;
	theta3 = acos((M * M + N * N - dh_a[1] * dh_a[1] - dh_a[2] * dh_a[2]) / (2 * dh_a[1] * dh_a[2]));
	ans(2) = theta3;
	A = dh_a[1] + dh_a[2] * cos(ans(2));
	B = dh_a[2] * sin(ans(2));
	a_mat(0, 0) = A;
	a_mat(0, 1) = -B;
	a_mat(1, 0) = B;
	a_mat(1, 1) = A;
	b_mat(0) = M;
	b_mat(1) = N;
	x_mat = a_mat.fullPivLu().solve(b_mat);
	ans(1) = atan2(x_mat(1), x_mat(0));
	ans(3) = atan2(s234, c234) - ans(1) - ans(2);
	for (int j = 0; j < 6; j++)
	{
		ans(j) = normaliz_to_2pi(ans(j));
	}
	return ans;
}

Eigen::MatrixXd Kinematics::Robot::InvKinematicsChoose(Eigen::MatrixXd last_postion, Eigen::MatrixXd last_target_tranformation_matrix, Eigen::MatrixXd target_transformation_matrix, double tol)
{
	auto all_last_position = InverseKinematics(last_target_tranformation_matrix);
	
	double err_norm = 1, min_norm = 1;
	int index = 0, min_norm_index = 1;
	while (err_norm > tol && index < 6)
	{
		err_norm = (last_postion - all_last_position.col(index++)).norm();
		if (min_norm > err_norm)
		{
			min_norm = err_norm;
			min_norm_index = index;
		}
	}
	index = (err_norm > tol) ? min_norm_index : index;
	switch (index)
	{
	case 1:
		return InvKinematics_1(target_transformation_matrix);
	case 2:
		return InvKinematics_2(target_transformation_matrix);
	case 3:
		return InvKinematics_3(target_transformation_matrix);
	case 4:
		return InvKinematics_4(target_transformation_matrix);
	case 5:
		return InvKinematics_5(target_transformation_matrix);
	case 6:
		return InvKinematics_6(target_transformation_matrix);
	case 7:
		return InvKinematics_7(target_transformation_matrix);
	case 8:
		return InvKinematics_8(target_transformation_matrix);
	default:
		break;
	}
	return Eigen::MatrixXd();
}

#define debug_invKine
Eigen::MatrixXd Kinematics::Robot::InvKineNumberic(Eigen::MatrixXd last_postion, Eigen::MatrixXd target_transformation_matrix, double tol)
{
	Eigen::MatrixXd pos = last_postion;
	Eigen::MatrixXd last_tmat = ForwardKinematics(last_postion);
	Eigen::MatrixXd last_tw = tmat2tw(last_tmat);
	Eigen::MatrixXd err = add_tw(tmat2tw(target_transformation_matrix), -last_tw);
	Eigen::MatrixXd jacob = Jacobe(last_postion);
	while (err.norm() > tol)
	{
#ifdef debug_invKine
		//std::cout << "last postion:\n" << pos.transpose() << std::endl;
		//std::cout << "err:\n";
		//std::cout << err.transpose() << std::endl;
		std::cout << last_tw.transpose() << std::endl;
		//std::cout << "SVD of jacobe:\n";
		//std::cout << jacob.bdcSvd().singularValues().transpose() << std::endl;
#endif // debug_invKine
		pos = jacob.fullPivLu().solve(err) + pos;
		for (int i = 0; i < 6; i++)
		{
			pos(i) = normaliz_to_2pi(pos(i));
		}
		last_tmat = ForwardKinematics(pos);
		last_tw = tmat2tw(last_tmat);
		err = tmat2tw(last_tmat.fullPivLu().solve(target_transformation_matrix));
		jacob = Jacobe(pos);
	}
	return Eigen::MatrixXd();
}

double Kinematics::normaliz_to_2pi(double arg)
{
	double pi_two = 2 * M_PI;
	while (arg >= pi_two)
	{
		arg -= pi_two;
	}

	while (arg < 0)
	{
		arg += pi_two;
	}
	return arg;
}

Eigen::MatrixXd Kinematics::tw2tmat(double twist[])
{
	double theta = 0;
	for (int i = 0; i < 3; i++)
	{
		theta += twist[i + 3] * twist[i + 3];
	}
	theta = sqrt(theta);
	Eigen::MatrixXd ans = Eigen::MatrixXd::Identity(4, 4);
	if (theta == 0.0)
	{
		for (int i = 0; i < 3; i++)
		{
			ans(i, 3) = twist[i];
		}
		return ans;
	}

	double w[3];
	for (int i = 0; i < 3; i++)
	{
		w[i] = twist[i + 3] / theta;
	}
	double cth = cos(theta);
	double sth = sin(theta);
	double vth = 1 - cth;
	ans << w[0] * w[0] * vth + cth, w[0] * w[1] * vth - w[2] * sth, w[0] * w[2] * vth + w[1] * sth, twist[0],
		w[0] * w[1] * vth + w[2] * sth, w[1] * w[1] * vth + cth, w[1] * w[2] * vth - w[0] * sth, twist[1],
		w[0] * w[2] * vth - w[1] * sth, w[1] * w[2] * vth + w[0] * sth, w[2] * w[2] * vth + cth, twist[2],
		0, 0, 0, 1;
	return ans;
}

Eigen::MatrixXd Kinematics::tw2tmat(Eigen::MatrixXd twist)
{
	double theta = 0;
	for (int i = 0; i < 3; i++)
	{
		theta += twist(i + 3) * twist(i + 3);
	}
	theta = sqrt(theta);
	Eigen::MatrixXd ans = Eigen::MatrixXd::Identity(4, 4);
	if (theta == 0.0)
	{
		for (int i = 0; i < 3; i++)
		{
			ans(i, 3) = twist(i);
		}
		return ans;
	}
	double w[3];
	for (int i = 0; i < 3; i++)
	{
		w[i] = twist(i + 3) / theta;
	}
	double cth = cos(theta);
	double sth = sin(theta);
	double vth = 1 - cth;
	ans << w[0] * w[0] * vth + cth, w[0] * w[1] * vth - w[2] * sth, w[0] * w[2] * vth + w[1] * sth, twist(0),
		w[0] * w[1] * vth + w[2] * sth, w[1] * w[1] * vth + cth, w[1] * w[2] * vth - w[0] * sth, twist(1),
		w[0] * w[2] * vth - w[1] * sth, w[1] * w[2] * vth + w[0] * sth, w[2] * w[2] * vth + cth, twist(2),
		0, 0, 0, 1;
	return ans;
}

Eigen::MatrixXd Kinematics::tmat2tw(Eigen::MatrixXd tmat)
{
	Eigen::Array<double, 6, 1> tw;
	tw.block(0, 0, 3, 1) = tmat.block(0, 3, 3, 1);
	double trace = 0;
	for (int i = 0; i < 3; i++)
	{
		trace += tmat(i, i);
	}
	if (trace == 3)
	{
		for (int i = 0; i < 3; i++)
		{
			tw(i + 3) = 0;
		}
	}
	else
	{
		if (trace>-1 && trace <3)
		{
			double theta = acos((trace - 1) / 2);
			tw(3) = tmat(2, 1) - tmat(1, 2);
			tw(4) = tmat(0, 2) - tmat(2, 0);
			tw(5) = tmat(1, 0) - tmat(0, 1);
			for (int i = 0; i < 3; i++)
			{
				tw(i + 3) /= 2 * sin(theta);
				tw(i + 3) *= theta;
			}
		}
		else
		{
			double theta = M_PI;
			tw(3) = sqrt((1 + tmat(0, 0)) / 2) * theta;
			if (tw(3) == 0.0)
			{
				tw(4) = sqrt((1 + tmat(1, 1)) / 2);
				if (tw(4) == 0.0)
				{
					tw(5) = sqrt((1 + tmat(2, 2)) / 2) * theta;
				}
				else
				{
					tw(5) = tmat(2, 0) / (tw(4) * 2) * theta;
				}
			}
			else
			{
				tw(4) = tmat(0, 1) / (tw(3) * 2) * theta;
				tw(5) = tmat(0, 2) / (tw(3) * 2) * theta;
			}
		}
	}
	return tw;
}

Eigen::MatrixXd Kinematics::subtract_tw(Eigen::MatrixXd ltw, Eigen::MatrixXd rtw)
{
	auto ltmat = tw2tmat(ltw);
	auto rtmat = tw2tmat(rtw);
	auto lrmat = ltmat.block(0, 0, 3, 3);
	auto rrmat = rtmat.block(0, 0, 3, 3);
	auto delta_tmat = (Eigen::Array<double, 4, 4>)translateTransMat((ltmat.block(0, 3, 3, 1) - rtmat.block(0, 3, 3, 1)));
	delta_tmat.block(0, 0, 3, 3) = ltmat.block(0, 0, 3, 3).transpose() * rtmat.block(0, 0, 3, 3);
	return tmat2tw(Eigen::MatrixXd(delta_tmat));
}

Eigen::MatrixXd Kinematics::add_tw(Eigen::MatrixXd ltw, Eigen::MatrixXd rtw)
{
	auto ltmat = tw2tmat(ltw);
	auto rtmat = tw2tmat(rtw);
	auto lrmat = ltmat.block(0, 0, 3, 3);
	auto rrmat = rtmat.block(0, 0, 3, 3);
	auto delta_tmat = (Eigen::Array<double, 4, 4>)translateTransMat((ltmat.block(0, 3, 3, 1) + rtmat.block(0, 3, 3, 1)));
	delta_tmat.block(0, 0, 3, 3) = rtmat.block(0, 0, 3, 3) * ltmat.block(0, 0, 3, 3);
	return tmat2tw(Eigen::MatrixXd(delta_tmat));
}

Eigen::MatrixXd Kinematics::Quaternion2Tmat(double quaterion[])
{
	return Eigen::MatrixXd();
}

Eigen::MatrixXd Kinematics::Euler2Tmat(double xyz[])
{
	return rotxTransMat(xyz[0]) * rotyTransMat(xyz[1]) * rotzTransMat(xyz[2]);
}

Kinematics::Robot::Robot(int d)
{
	dof = d;

	method = DH;
	dh_a = new double[d];
	dh_alpha = new double[d];
	dh_d = new double[d];


	dh_a[0] = 0;
	dh_a[1] = 353;
	dh_a[2] = 303;
	dh_a[3] = 0;
	dh_a[4] = 0;
	dh_a[5] = 0;

	dh_alpha[0] = M_PI_2; 
	dh_alpha[1] = 0;
	dh_alpha[2] = 0;
	dh_alpha[3] = M_PI_2;
	dh_alpha[4] = -M_PI_2;
	dh_alpha[5] = 0;

	dh_d[0] = 114.75;
	dh_d[1] = 0;
	dh_d[2] = 0;
	dh_d[3] = -116.25;
	dh_d[4] = 103.25;
	dh_d[5] = 93.75;
}


