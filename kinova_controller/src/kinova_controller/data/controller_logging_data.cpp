#include "kinova_controller/data/controller_logging_data.hpp"


void ControllerLoggingData::resize(const int & nq, const int & nv)
{
    est_tau_f.resize(nv);
    tau_f.resize(nv);
    nominal_theta.resize(nq);
    nominal_dtheta.resize(nv);
    e_dn.resize(nv);
    e_nr.resize(nv);
    theta_des.resize(nq);
}

void ControllerLoggingData::setZero()
{
    est_tau_f.setZero();
    tau_f.setZero();
    nominal_theta.setZero();
    nominal_dtheta.setZero();
    e_dn.setZero();
    e_nr.setZero();
    theta_des.setZero();
}
