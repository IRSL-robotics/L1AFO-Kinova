#pragma once

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/fwd.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <chrono>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include "kinova_controller/data/robot_state.hpp"
#include "kinova_controller/data/nominal_plant.hpp"
#include "kinova_controller/data/coulomb_observer_state.hpp"
#include <unsupported/Eigen/MatrixFunctions>

#include <yaml-cpp/yaml.h>
#include "kinova_controller/data/controller_state.hpp"
#include "kinova_controller/controller/controller_gain.hpp"

#include "controller_interface_msgs/msg/controller_config.hpp"

#include "kinova_controller/math_type_define.h"

class Controller
{
    public:
    Controller();

    void assign_robot_state(std::shared_ptr<RobotState> robot_state_arg);
    void init();
    CONTROLLER_SELECTOR select(const std::string & controller_type);
    Eigen::VectorXd get_control_input(const int& selector);
    void update_controller_gain(const YAML::Node& node);


    private:
    std::shared_ptr<RobotState> robot_state;
    ControllerGain controller_gain;
    int nq, nv, nu;
    double dt;
    CONTROLLER_SELECTOR selector_prev{CONTROLLER_SELECTOR::GRAVITY_COMPENSATION};

    //for lugre friction
    Eigen::VectorXd z_;

    //for NRIC
    NominalPlant nominal_plant;

    //for coulomb observer
    CoulombObserverState coulomb_observer_state;

    //for flexible joint robot parameter
    Eigen::MatrixXd joint_stiffness_matrix_;
    Eigen::MatrixXd rotor_inertia_matrix_;

    
    //*****************************Controller**********************************//

    bool init_PD_joint_controller();
    Eigen::VectorXd PD_joint_controller();

    bool init_PD_task_controller();
    Eigen::VectorXd PD_task_controller();

    bool init_NRIC_joint_controller();
    Eigen::VectorXd NRIC_joint_controller();

    bool init_NRIC_task_controller();
    Eigen::VectorXd NRIC_task_controller();

    bool init_FRIC_joint_controller();
    Eigen::VectorXd FRIC_joint_controller();

    bool init_FRIC_task_controller();
    Eigen::VectorXd FRIC_task_controller();

    bool init_gravity_compensation();
    Eigen::VectorXd gravity_compensation();

    //torque control interface
    bool init_first_order_friction_compensation();
    Eigen::VectorXd first_order_friction_compensation(const Eigen::VectorXd &desired_u);
    bool init_second_order_friction_compensation();
    Eigen::VectorXd second_order_friction_compensation(const Eigen::VectorXd &desired_u);
    bool init_third_order_friction_compensation();
    Eigen::VectorXd third_order_friction_compensation(const Eigen::VectorXd &desired_u);
    bool init_intertia_reshaping();
    Eigen::VectorXd inertia_reshaping(const Eigen::VectorXd &desired_u);
    bool init_l1_friction_compensation();
    Eigen::VectorXd l1_friction_compensation(const Eigen::VectorXd &desired_u);
    bool init_implicit_l1_friction_compensation();
    Eigen::VectorXd implicit_l1_friction_compensation(const Eigen::VectorXd &desired_u);
    bool init_coulomb_observer_friction_compensation();
    Eigen::VectorXd coulomb_observer_friction_compensation(const Eigen::VectorXd &desired_u);
    bool init_asym_friction_compensation();
    Eigen::VectorXd asym_friction_compensation(const Eigen::VectorXd &desired_u);
    bool init_static_friction_compensation();
    Eigen::VectorXd static_friction_compensation(const Eigen::VectorXd &desired_u);

    //*************************************************************************//
};
