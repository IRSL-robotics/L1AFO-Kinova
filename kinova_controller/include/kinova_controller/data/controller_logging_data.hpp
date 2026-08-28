#pragma once

#include <Eigen/Dense>

struct ControllerLoggingData
{
    Eigen::VectorXd est_tau_f;
    Eigen::VectorXd tau_f;
    Eigen::VectorXd nominal_theta;
    Eigen::VectorXd nominal_dtheta;
    Eigen::VectorXd e_dn;
    Eigen::VectorXd e_nr;
    Eigen::VectorXd theta_des;

    void resize(const int & nq, const int & nv);
    void setZero();
};
