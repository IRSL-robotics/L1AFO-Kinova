#pragma once

#include <Eigen/Dense>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include "pinocchio/algorithm/joint-configuration.hpp"

#include <yaml-cpp/yaml.h>
#include "kinova_controller/data/gen3_state.hpp"
#include "kinova_controller/data/controller_logging_data.hpp"

class RobotState
{   
    public:
    RobotState(const YAML::Node &config_arg, const pinocchio::Model &pinocchio_model_arg, const std::string & config_path_arg);

    void update_robot_state(const Gen3State & gen3_state);

    Eigen::VectorXd integrate(const Eigen::VectorXd &q, const Eigen::VectorXd &dq, const double &dt);
    Eigen::VectorXd difference(const Eigen::VectorXd &q1, const Eigen::VectorXd &q2); // dqout = q2 - q1

    Eigen::MatrixXd get_mass(const Eigen::VectorXd &q);
    Eigen::MatrixXd get_coriolis(const Eigen::VectorXd &q, const Eigen::VectorXd &dq);
    Eigen::MatrixXd get_gravity(const Eigen::VectorXd &q);
    
    Eigen::MatrixXd get_ee_jacobian_w(const Eigen::VectorXd &q, const std::string & ee_name);
    Eigen::MatrixXd get_ee_jacobian_w(const std::string & ee_name);
    
    Eigen::MatrixXd get_ee_jacobian_b(const Eigen::VectorXd &q, const std::string & ee_name);
    Eigen::MatrixXd get_ee_jacobian_b(const std::string & ee_name);

    Eigen::Affine3d get_ee_pose(const Eigen::VectorXd &q, const std::string & ee_name);
    Eigen::Affine3d get_ee_pose(const std::string & ee_name);

    Eigen::VectorXd get_joint_nominal_torque(const Eigen::VectorXd &gacc);


    YAML::Node get_config() { return config; }
    void update_config();
    void convert_q(Gen3State &gen3_state);
    void convert_q(Eigen::VectorXd &q_arg);

    pinocchio::Model pinocchio_model;
    pinocchio::Data pinocchio_data;
    YAML::Node config;
    const std::string config_path; 

    Eigen::VectorXd q; // link position
    Eigen::VectorXd dq; // qdot

    Eigen::VectorXd q_d; // desired link positions
    Eigen::VectorXd dq_d; // desired link velocity

    Eigen::VectorXd ddq_d; // desired link acceleration

    Eigen::VectorXd theta; // motor position
    Eigen::VectorXd dtheta;

    Eigen::VectorXd theta_next;
    Eigen::VectorXd x_des_prev;

    Eigen::VectorXd tau_J; // JTS measurement
    Eigen::VectorXd joint_temperatures; // joint motor temperatures

    std::vector<Eigen::Affine3d> ee_htm; // End-effector position

    pinocchio::SE3 htm_d; // desried end-effector pose

    double time = 0.0;
    
    Eigen::VectorXd theta_init;

    bool is_rigid;
    bool is_simulation;

    const int nq;
    const int nv;
    const int nu;
    const int nx;
    const int ndx;
    double dt;

    Eigen::VectorXd bias;
    bool init_tau_j = true;

    unsigned int count = 1;
    int avg_filter_count = 1000;

    //for data logging
    bool is_controller_logging;
    ControllerLoggingData controller_logging_data;

    private:
};
