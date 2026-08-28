#include "kinova_controller/filter/lpf.hpp"

LPF::LPF()
{}

void LPF::init(double sampling_rate, double cutoff_frequency, const Eigen::VectorXd &init_value)
{   
    double tau = 1/(2*M_PI*cutoff_frequency);
    alpha_ = (tau)/(tau+sampling_rate);

    vlaue_prev = init_value;
}

Eigen::VectorXd LPF::get_filtered_value(const Eigen::VectorXd &raw)
{
    Eigen::VectorXd filtered_value(7);
    filtered_value = vlaue_prev * alpha_ + raw * (1-alpha_);
    vlaue_prev = filtered_value;

    return filtered_value;
}