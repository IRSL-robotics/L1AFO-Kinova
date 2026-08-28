#pragma once

#include <Eigen/Dense>

struct NominalPlant
{   
    Eigen::VectorXd ddtheta;
    Eigen::VectorXd dtheta;
    Eigen::VectorXd theta;

    Eigen::VectorXd theta_next;

    Eigen::VectorXd q;
    Eigen::VectorXd dq;
    Eigen::VectorXd ddq;

    Eigen::VectorXd integral_e_NR;

    Eigen::VectorXd est_tau_f_prev; 
    Eigen::VectorXd sigma_hat_prev; 

    Eigen::VectorXd friction;
};