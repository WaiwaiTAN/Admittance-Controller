/*! \file RobotKinematics.h
	\brief 本头文件定义类Robot
	
	类Robot用DH法描述机器人运动学，提供了正向动力学求解、逆向动力学求解（分析解，仅适用于特定结构的机器人），和微分运动学求解
    版本：1.0

	\author 谭正
	\date 2021.05.27
*/
#pragma once

/*! \def _USE_MATH_DEFINES
* \brief 使用cmath库中的宏定义（M_PI）等
*/
#define _USE_MATH_DEFINES
#include <cmath>
#include <Eigen/Dense>

namespace Kinematics
{
	/*! \brief 工具函数dhLinkTransMat
	*
	* dhLinkTransMat根据DH参数计算连杆间的transformation matrix
	*
	* \param a DH参数a，double类型
	* \param alpha DH参数alpha（单位rad），double类型
	* \param d DH参数d，double类型
	* \param theta DH参数theta（单位rad），double类型
	* \return Eigen::MatrixXd类型的transformation matrix
	*/
	Eigen::MatrixXd dhLinkTransMat(double a, double alpha, double d, double theta);

	/*! \brief 工具函数rotxTranMat
	*
	* rotxTranMat计算绕x轴旋转theta后的transformation matrix
	*
	* \param theta 旋转角度（单位rad），double类型
	* \return Eigen::MatrixXd类型的transformation matrix
	*/
	Eigen::MatrixXd rotxTransMat(double theta);

	/*! \brief 工具函数rotyTranMat
	*
	* rotyTranMat计算绕y轴旋转theta后的transformation matrix
	*
	* \param theta 旋转角度（单位rad），double类型
	* \return Eigen::MatrixXd类型的transformation matrix
	*/
	Eigen::MatrixXd rotyTransMat(double theta);

	/*! \brief 工具函数rotzTranMat
	*
	* rotzTranMat计算绕z轴旋转theta后的transformation matrix
	*
	* \param theta 旋转角度（单位rad），double类型
	* \return Eigen::MatrixXd类型的transformation matrix
	*/
	Eigen::MatrixXd rotzTransMat(double theta);

	/*! \brief 工具函数translateTransMat
	*
	* translateTransMat计算向[x, y, z]方向移动后的transformation matrix
	*
	* \param trans 移动的向量[x, y, z]，以double类型，长度为3的数组保存
	* \return Eigen::MatrixXd类型的transformation matrix
	*/
	Eigen::MatrixXd translateTransMat(double trans[3]);

	/*! \brief 工具函数translateTransMat
	*
	* translateTransMat计算向[x, y, z]方向移动后的transformation matrix
	*
	* \param trans 移动的向量[x, y, z]，Eigen::MatrixXd保存，且要求是列向量或者行向量
	* \return Eigen::MatrixXd类型的transformation matrix
	*/
	Eigen::MatrixXd translateTransMat(Eigen::MatrixXd trans);

	/*! \brief 归一化关节角
	*
	* normaliz_to_2pi函数归一化关节角到0-2pi
	*
	* \param arg 需要归一化处理的关节角（单位rad），double类型
	* \return 归一化完成后的关节角
	*/
	double normaliz_to_2pi(double arg);

	/*! \brief 将Twist转化到对应的Transformation Matrix
	*
	* tw2tmat函数，使用Rodrigue公式将Twist转化成对应的Transformation Matrix
	*
	* \param twist 需要转化的Twist，以double类型，长度为6的数组保存
	* \return 对应的transformation matrix，Eigen::MatrixXd类型
	*/
	Eigen::MatrixXd tw2tmat(double twist[]);

	/*! \brief 将Twist转化到对应的Transformation Matrix
	*
	* tw2tmat函数，使用Rodrigue公式将Twist转化成对应的Transformation Matrix
	*
	* \param twist 需要转化的Twist，6-1的Eigen::MatrixXd类型
	* \return 对应的transformation matrix，Eigen::MatrixXd类型
	*/
	Eigen::MatrixXd tw2tmat(Eigen::MatrixXd twist);

	/*! \brief 将Transformation Matrix转化到对应的Twist
	*
	* tmat2tw函数，使用Rodrigue公式根据Transformation Matrix求解对应的Twist
	*
	* \param tmat 需要转化的Transformation Matrix，Eigen::MatrixXd类型
	* \return 对应的Twist，Eigen::MatrixXd类型
	*/
	Eigen::MatrixXd tmat2tw(Eigen::MatrixXd tmat);

	/*! \brief Twist的相减运算
	*
	* subtract_tw函数计算两个Twist之间的差值
	* 两个Twist之间的差值这样表述：
	* 差值的平动部分就是平动部分之差；
	* 差值的转动部分定义为对应旋转矩阵的商对应的Twist
	*
	* \param ltw 被减数，Eigen::MatrixXd类型
	* \param rtw 减数，Eigen::MatrixXd类型
	* \return 差值，Eigen::MatrixXd类型
	*/
	Eigen::MatrixXd subtract_tw(Eigen::MatrixXd ltw, Eigen::MatrixXd rtw);

	/*! \brief Twist的相加运算
	*
	* add_tw函数计算两个Twist之间的差值
	* 两个Twist之和这样表述：
	* 和的平动部分就是平动部分之和；
	* 和的转动部分定义为ltw对应旋转矩阵左乘rtw对应的旋转矩阵
	*
	* \param ltw 左加数，Eigen::MatrixXd类型
	* \param rtw 右加数，Eigen::MatrixXd类型
	* \return 和，Eigen::MatrixXd类型
	*/
	Eigen::MatrixXd add_tw(Eigen::MatrixXd ltw, Eigen::MatrixXd rtw);

	Eigen::MatrixXd Quaternion2Tmat(double quaterion[]);
	Eigen::MatrixXd Euler2Tmat(double xyz[]);

	/*! \brief 机器人运动学求解类
	*
	* 使用DH参数方法对机器人进行建模，对特定机器人提供正向和逆向运动学求解函数和微分运动学求解函数
	*/
	class Robot
	{
		/*! \brief 机器人运动学描述方法类型
		*
		* 枚举类型KineMethod列出描述机器人运动学的方法类型，现仅提供DH类型可以使用
		*/
		enum KineMethod
		{
			DH, Twist
		};

	private:
		/*! \brief 机器人运动学描述方法
		*/
		KineMethod method;

		/*! \brief 机器人自由度
		*/
		int dof;

		/*! \brief 机器人DH参数表中的a参数表
		*/
		double* dh_a;

		/*! \brief 机器人DH参数表中的alpha参数表
		*/
		double* dh_alpha;

		/*! \brief 机器人DH参数表中的d参数表
		*/
		double* dh_d;

	public:
		/*! \brief Robot构造函数
		*
		* Robot构造函数，默认情况下会构造一个六自由度机器人并初始化其DH参数
		*
		* \param d 机器人的自由度
		*/
		Robot(int d = 6);		

		/*! \brief 计算机器人的雅克比矩阵
		*
		* Jacobe计算机器人在当前位置下的雅克比矩阵
		* 该雅克比矩阵中，末端的速度采用末端坐标系下的坐标，因此该雅克比矩阵又称为末端雅克比
		*
		* \param q 当前机器人的关节角（单位rad），以double类型，长度为dof的数组保存
		* \return 雅克比矩阵，大小为6-dof的矩阵
		*/
		Eigen::MatrixXd Jacobe(double q[]);

		/*! \brief 计算机器人的雅克比矩阵
		*
		* Jacobe计算机器人在当前位置下的雅克比矩阵
		* 该雅克比矩阵中，末端的速度采用末端坐标系下的坐标，因此该雅克比矩阵又称为末端雅克比
		*
		* \param q 当前机器人的关节角（单位rad）
		* \return 雅克比矩阵，大小为6-dof的矩阵
		*/
		Eigen::MatrixXd Jacobe(Eigen::MatrixXd q);

		/*! \brief 计算机器人的末端位姿
		*
		* ForwardKinamatcs计算机器人在当前位置下的末端位姿
		*
		* \param q 当前机器人的关节角（单位rad），以double类型，长度为dof的数组保存
		* \return Eigen::MatrixXd类型的transformation matrix
		*/
		Eigen::MatrixXd ForwardKinematics(double q[]);
		
		/*! \brief 计算机器人的末端位姿
		*
		* ForwardKinamatcs计算机器人在当前位置下的末端位姿
		*
		* \param q 当前机器人的关节角（单位rad），Eigen::MatrixXd类型
		* \return Eigen::MatrixXd类型的transformation matrix
		*/
		Eigen::MatrixXd ForwardKinematics(Eigen::MatrixXd q);

		/*! \brief 计算机器人的关节角
		*
		* InverseKinematics函数根据给定末端位姿计算机器人的关节角
		* 采用解析方法计算，此方法只适用于特定类型的机器人（第2、3、4轴平行）
		*
		* \param target_transformation_matrix Eigen::MatrixXd类型的末端坐标系transformation matrix
		* \return 6-8的矩阵，Eigen::MatrixXd类型，每列都是一组可能的解
		*/
		Eigen::MatrixXd InverseKinematics(Eigen::MatrixXd target_transformation_matrix);

		/*! \brief 计算机器人的关节角
		*
		* InvKinematics_1函数根据给定末端位姿计算机器人的关节角
		* 采用解析方法计算，此方法只适用于特定类型的机器人（第2、3、4轴平行）
		*
		* \param target_transformation_matrix Eigen::MatrixXd类型的末端坐标系transformation matrix
		* \return 6-1的矩阵，Eigen::MatrixXd类型，该结果和InverseKinematics返回的第一列的结果一致
		*/
		Eigen::MatrixXd InvKinematics_1(Eigen::MatrixXd target_transformation_matrix);

		/*! \brief 计算机器人的关节角
		*
		* InvKinematics_2函数根据给定末端位姿计算机器人的关节角
		* 采用解析方法计算，此方法只适用于特定类型的机器人（第2、3、4轴平行）
		*
		* \param target_transformation_matrix Eigen::MatrixXd类型的末端坐标系transformation matrix
		* \return 6-1的矩阵，Eigen::MatrixXd类型，该结果和InverseKinematics返回的第二列的结果一致
		*/
		Eigen::MatrixXd InvKinematics_2(Eigen::MatrixXd target_transformation_matrix);

		/*! \brief 计算机器人的关节角
		*
		* InvKinematics_3函数根据给定末端位姿计算机器人的关节角
		* 采用解析方法计算，此方法只适用于特定类型的机器人（第2、3、4轴平行）
		*
		* \param target_transformation_matrix Eigen::MatrixXd类型的末端坐标系transformation matrix
		* \return 6-1的矩阵，Eigen::MatrixXd类型，该结果和InverseKinematics返回的第三列的结果一致
		*/
		Eigen::MatrixXd InvKinematics_3(Eigen::MatrixXd target_transformation_matrix);

		/*! \brief 计算机器人的关节角
		*
		* InvKinematics_4函数根据给定末端位姿计算机器人的关节角
		* 采用解析方法计算，此方法只适用于特定类型的机器人（第2、3、4轴平行）
		*
		* \param target_transformation_matrix Eigen::MatrixXd类型的末端坐标系transformation matrix
		* \return 6-1的矩阵，Eigen::MatrixXd类型，该结果和InverseKinematics返回的第四列的结果一致
		*/
		Eigen::MatrixXd InvKinematics_4(Eigen::MatrixXd target_transformation_matrix);

		/*! \brief 计算机器人的关节角
		*
		* InvKinematics_5函数根据给定末端位姿计算机器人的关节角
		* 采用解析方法计算，此方法只适用于特定类型的机器人（第2、3、4轴平行）
		*
		* \param target_transformation_matrix Eigen::MatrixXd类型的末端坐标系transformation matrix
		* \return 6-1的矩阵，Eigen::MatrixXd类型，该结果和InverseKinematics返回的第五列的结果一致
		*/
		Eigen::MatrixXd InvKinematics_5(Eigen::MatrixXd target_transformation_matrix);

		/*! \brief 计算机器人的关节角
		*
		* InvKinematics_6函数根据给定末端位姿计算机器人的关节角
		* 采用解析方法计算，此方法只适用于特定类型的机器人（第2、3、4轴平行）
		*
		* \param target_transformation_matrix Eigen::MatrixXd类型的末端坐标系transformation matrix
		* \return 6-1的矩阵，Eigen::MatrixXd类型，该结果和InverseKinematics返回的第六列的结果一致
		*/
		Eigen::MatrixXd InvKinematics_6(Eigen::MatrixXd target_transformation_matrix);

		/*! \brief 计算机器人的关节角
		*
		* InvKinematics_7函数根据给定末端位姿计算机器人的关节角
		* 采用解析方法计算，此方法只适用于特定类型的机器人（第2、3、4轴平行）
		*
		* \param target_transformation_matrix Eigen::MatrixXd类型的末端坐标系transformation matrix
		* \return 6-1的矩阵，Eigen::MatrixXd类型，该结果和InverseKinematics返回的第七列的结果一致
		*/
		Eigen::MatrixXd InvKinematics_7(Eigen::MatrixXd target_transformation_matrix);

		/*! \brief 计算机器人的关节角
		*
		* InvKinematics_8函数根据给定末端位姿计算机器人的关节角
		* 采用解析方法计算，此方法只适用于特定类型的机器人（第2、3、4轴平行）
		*
		* \param target_transformation_matrix Eigen::MatrixXd类型的末端坐标系transformation matrix
		* \return 6-1的矩阵，Eigen::MatrixXd类型，该结果和InverseKinematics返回的第八列的结果一致
		*/
		Eigen::MatrixXd InvKinematics_8(Eigen::MatrixXd target_transformation_matrix);

		/*! \brief 计算机器人的关节角
		*
		* InvKinematicsChoose函数根据给定已知的末端位姿和关节角对应信息计算计算机器人的关节角
		* 采用解析方法计算，此方法只适用于特定类型的机器人（第2、3、4轴平行）
		* 该函数首先会确定last_postion是last_target_tranformation_matrix的哪一组解，然后再调用相应的求解函数
		*
		* \param last_target_transformation_matrix Eigen::MatrixXd类型的参考末端坐标系transformation matrix
		* \param last_postion Eigen::MatrixXd类型的关节角，参考末端坐标系对应
		* \param target_transformation_matrix Eigen::MatrixXd类型的末端坐标系transformation matrix
		* \return 6-1的矩阵，Eigen::MatrixXd类型，是target_transformation_matrix对应的逆运动学求解结果
		*/
		Eigen::MatrixXd InvKinematicsChoose(Eigen::MatrixXd last_postion, Eigen::MatrixXd last_target_tranformation_matrix, Eigen::MatrixXd target_transformation_matrix, double tol = 1e-5);

		/*! \brief 计算机器人的关节角
		*
		* InvKineNumberic函数根据给定已知的末端位姿和关节角对应信息计算计算机器人的关节角
		* 采用数值方法计算，此方法适用于所有类型的机器人
		*
		* \param last_postion Eigen::MatrixXd类型的关节角，参考末端坐标系对应
		* \param target_transformation_matrix Eigen::MatrixXd类型的末端坐标系transformation matrix
		* \return 6-1的矩阵，Eigen::MatrixXd类型，是target_transformation_matrix对应的逆运动学求解结果
		*/
		Eigen::MatrixXd InvKineNumberic(Eigen::MatrixXd last_postion, Eigen::MatrixXd target_transformation_matrix, double tol = 1e-5);
	};

}