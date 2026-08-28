#pragma once

#include <Eigen/Dense>

enum CONTROLLER_SELECTOR
{
  JOINT_PD,
  JOINT_PD_FRIC,
  JOINT_NRIC,
  TASK_PD,
  TASK_PD_FRIC,
  TASK_NRIC,
  GRAVITY_COMPENSATION,
};

enum TORQUE_INTERFACE
{
  FIRST_ORDER_FRIC,
  SECOND_ORDER_FRIC,
  THIRD_ORDER_FRIC,
  INERTIA_RESHAPING,
  L1_FRIC,
  IMPLICIT_L1_FRIC,
  COULOMB_OBSERVER,
  ASYM_FRICTION,
  STATIC_FRICTION,
};

// u = Kp * e + Kd * de
struct JOINT_PD_Gains
{
  Eigen::MatrixXd Kp; // Proportional gain
  Eigen::MatrixXd Kd; // Derivative gain
};

struct JOINT_NRIC_Gains
{
  Eigen::MatrixXd k1;
  Eigen::MatrixXd k2;
  Eigen::MatrixXd K;
  Eigen::MatrixXd gamma;
  Eigen::MatrixXd reflected_inertia;
  Eigen::MatrixXd Kp; // Proportional gain
  Eigen::MatrixXd Ki; // Integral gain
};

struct TASK_PD_Gains
{
  Eigen::MatrixXd Kp; // Proportional gain
  Eigen::MatrixXd Kd; // Derivative gain
};

struct TASK_NIRC_Gains
{
  Eigen::MatrixXd k1;
  Eigen::MatrixXd k2;
  Eigen::MatrixXd K;
  Eigen::MatrixXd gamma;
  Eigen::MatrixXd reflected_inertia;
  Eigen::MatrixXd Kp; // Proportional gain
  Eigen::MatrixXd Ki; // Integral gain
};

struct JOINT_FRIC_Gains
{
  Eigen::MatrixXd Kp;
  Eigen::MatrixXd Kd;
  Eigen::MatrixXd joint_stiffness_matrix;
  TORQUE_INTERFACE torque_interface;
};

struct TASK_FRIC_Gains
{
  Eigen::MatrixXd Kp;
  Eigen::MatrixXd Kd;
  Eigen::MatrixXd joint_stiffness_matrix;
  TORQUE_INTERFACE torque_interface;
};

//for torque control interface
struct FIRST_FRIC_Gains
{
  Eigen::MatrixXd motor_inertia_matrix;
  Eigen::MatrixXd L; 
};

struct SECOND_FRIC_Gains
{
  Eigen::MatrixXd motor_inertia_matrix;
  Eigen::MatrixXd L; 
  Eigen::MatrixXd Lp;
};

struct THIRD_FRIC_Gains
{
  Eigen::MatrixXd motor_inertia_matrix;
  Eigen::MatrixXd L; 
  Eigen::MatrixXd Lp;
  Eigen::MatrixXd Li;
};

struct INERTIA_RESHAPING_Gains
{
  Eigen::MatrixXd motor_inertia_matrix;
  Eigen::MatrixXd desired_motor_inertia_matrix;
};

struct L1_FRIC_Gains
{
  Eigen::MatrixXd motor_inertia_matrix;
  Eigen::MatrixXd As; 
  Eigen::MatrixXd Gamma;
  Eigen::MatrixXd W;
};

struct IMPLICIT_L1_FRIC_Gains
{
  Eigen::MatrixXd motor_inertia_matrix;
  Eigen::MatrixXd Gamma;
  Eigen::MatrixXd Gamma_p;
  Eigen::MatrixXd W;
};

struct COULOMB_OBSERVER_Gains
{
  Eigen::MatrixXd motor_inertia_matrix;
  Eigen::MatrixXd K;
  Eigen::MatrixXd L;
};

struct ASYM_FRICTION_Gains
{
  Eigen::VectorXd f_v1_p, f_v1_n, f_v2_p, f_v2_n, f_v3_p, f_v3_n;
  Eigen::VectorXd f_s_p, f_s_n, f_c_p, f_c_n;
  Eigen::VectorXd qdot_s_p, qdot_s_n, qdot_c_p, qdot_c_n;
  Eigen::VectorXd delta_s_p, delta_s_n, delta_c_p, delta_c_n;
  Eigen::VectorXd f_l1_p, f_l1_n, f_l2_p, f_l2_n;
  Eigen::VectorXd f_l3_p, f_l3_n, f_l4_p, f_l4_n;
  Eigen::VectorXd f_T1, f_T2, f_T3, f_T4, mu;
};

struct STATIC_FRICTION_Gains
{
  Eigen::MatrixXd Fc;
  Eigen::MatrixXd Fv1;
  Eigen::MatrixXd Fv2;
  Eigen::MatrixXd friction_compensation_weights;
};
