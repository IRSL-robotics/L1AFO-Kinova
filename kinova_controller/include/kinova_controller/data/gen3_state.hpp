#pragma once

#include <Eigen/Core>
#include <iostream>

// This struct is used to store the current state of the real gen3 robot
struct Gen3State
{
    Eigen::Matrix<double, 7, 1> joint_positions;
    Eigen::Matrix<double, 7, 1> joint_velocities;
    Eigen::Matrix<double, 7, 1> joint_torques;
    Eigen::Matrix<double, 7, 1> joint_temperatures;
    Eigen::VectorXd converted_q;

    void print();

};