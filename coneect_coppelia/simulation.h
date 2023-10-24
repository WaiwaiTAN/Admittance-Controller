/*! \file simulation.h
    \brief 本头文件定义类Admittance

    版本：1.0

    \author 谭正
    \date 2021.05.27
*/
#pragma once
#define NON_MATLAB_PARSING
#define MAX_EXT_API_CONNECTIONS = 255
#define _CRT_SECURE_NO_WARNINGS

extern "C"
{
#include "extApi.h"
#include "extApiPlatform.h"
}

#include "RobotKinematics.h"
#include "trajgen.h"
#include "comm.h"
#include <iostream>

namespace Simulation
{
    /*! /var X_DISABLED
    * 
    * \brief 限制X方向一定跟随期望轨迹（关闭X平动方向上的控制）
    */
    const unsigned short X_DISABLED = 0b1;
    
    /*! /var Y_DISABLED
    *
    * \brief 限制Y方向一定跟随期望轨迹（关闭Y平动方向上的控制）
    */
    const unsigned short Y_DISABLED = 0b10;

    /*! /var Z_DISABLED
    *
    * \brief 限制Z方向一定跟随期望轨迹（关闭Z平动方向上的控制）
    */
    const unsigned short Z_DISABLED = 0b100;

    /*! /var RX_DISABLED
    *
    * \brief 限制X方向的旋转位姿一定跟随期望轨迹（关闭X旋转方向上的控制）
    */
    const unsigned short RX_DISABLED = 0b1000;
    
    /*! /var RY_DISABLED
    *
    * \brief 限制Y方向的旋转位姿一定跟随期望轨迹（关闭Y旋转方向上的控制）
    */
    const unsigned short RY_DISABLED = 0b10000;

    /*! /var RZ_DISABLED
    *
    * \brief 限制Z方向的旋转位姿一定跟随期望轨迹（关闭Z旋转方向上的控制）
    */
    const unsigned short RZ_DISABLED = 0b100000;

    /*! \brief 导纳控制仿真类
    *
    *
    */
    class Admittance
    {
    private:
        /*! \brief 初始化Coppelia接口*/
        void intial_sim();

        /*! \brief 被控机器人对象*/
        Kinematics::Robot rk;

        /*！\brief Coppelia显示开关*/
        bool disp_flag = false;

        /*！\brief debug开关，开启后会将仿真过程中的位置信息等输出到标准输出流*/
        bool debug_flag = false;

        /*! \brief 仿真开始时间，默认为0*/
        double start_time;

        /*! \brief 仿真时间步长，默认为0.008*/
        double time_gap;

        /*! \brief 仿真结束时间*/
        double sim_tf;

        /*! \brief 导纳控制环境质量参数*/
        Eigen::MatrixXd m;

        /*! \brief 导纳控制环境阻尼参数*/
        Eigen::MatrixXd b;

        /*! \brief 导纳控制环境刚度参数*/
        Eigen::MatrixXd k;

        /*! \brief 连接到Coppelia服务器的客户端*/
        simxInt client_id;

        /*! \brief Coppelia服务器中六个关节的handle*/
        simxInt handle[6];

        /*! \brief 使用轨迹生成类的指针*/
        TrajGen_base* traj;

        /*! \brief 上次移动指令发出的时间*/
        ULONGLONG last_move_time;

        /*! \brief 自由度限制标识
        *
        * 限制自由度一定跟随期望位置，限制与否用dof_enabled的某一位表示
        * 例如，0b111110限制除了x方向以外的五个自由度
        * 要求其一定跟随期望位置
        */
        unsigned short dof_enabled;

        /*! \var dynamics_enabled
        * \brief Coppelia动力学开启
        * 
        * Coppelia动力学开启标识符。
        * 若开启，进行仿真会调用函数simxSetJointTargetPosition
        */
        bool dynamics_enabled = false;

    public:
        /*! \brief Admittance的默认构造函数
        *
        * 初始化仿真开始时间，仿真时间步长，导纳控制的质量参数、阻尼参数和刚度参数等
        */
        Admittance();

        /*! \brief Admittance的默认析构函数*/
        ~Admittance() {}

        /*! \brief 导纳控制微分方程
        *
        * 形如 e' = f(t, e) 的形式的导纳控制微分方程
        */
        Eigen::MatrixXd differential_eq(double t, Eigen::MatrixXd x);

        /*! \brief 运行一个导纳控制仿真
        *
        * 使用四阶龙格库塔积分，运行一个导纳控制仿真
        * 运行前必须调用set_traj_gen()设置期望轨迹
        */
        void sim_rk4();

        /*! \fn set_traj_gen
        * \brief 设定对应的轨迹生成类
        * 
        * \param tr 对应轨迹生成类对象的地址
        */
        void set_traj_gen(TrajGen_base* tr);

        /*! \fn set_portions
        * \brief 设定导纳控制的参数
        * 
        * \param mp 导纳控制的质量参数
        * \param bp 导纳控制的阻尼参数
        * \param kp 导纳控制的刚度参数
        * \param tag 所设定的自由度
        */
        void set_portions(double mp, double bp, double kp, int tag);

        /*! \fn set_debug_mode
        * \brief 设置是否以调试模式运行
        * 
        */
        void set_debug_mode(bool debug) { debug_flag = debug; }

        /*! \fn set_disp_mode
        * \brief 设置是否连接Coppelia服务器运行
        * 
        */
        void set_disp_mode(bool disp);
        
        /*! \fn set_dof_enble_flag
        * \brief 设置自由度限制标识
        * 
        * 更改当前的自由度限制标识
        * 
        * 例：限制X方向的位移和Y方向的位移
        * 可以使用如下的代码：
        * \code
        * obj.set_dof_enble_flag(X_DISABLED | Y_DISABLED);
        * \endcode
        */
        void set_dof_enble_flag(unsigned short a);

        /*! \fn set_dynamic 
        * \brief 设置动力学是否启用
        * 
        */
        void set_dynamic(bool flag);
    };
};

   



