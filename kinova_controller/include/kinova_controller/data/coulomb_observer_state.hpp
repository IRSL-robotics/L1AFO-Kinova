#pragma once

#include <Eigen/Dense>

struct CoulombObserverState
{
    Eigen::VectorXd dtheta_hat;
    Eigen::VectorXd a_c_hat;
    Eigen::VectorXd f_hat;
    Eigen::VectorXd u_prev;
};
