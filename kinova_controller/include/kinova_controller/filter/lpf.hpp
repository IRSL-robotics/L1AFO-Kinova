#ifndef LPF_HPP
#define LPF_HPP

#include <Eigen/Core>
using namespace std;
using namespace Eigen;

//low-pass filter
class LPF
{
    private:
    Eigen::VectorXd  vlaue_prev;
    double alpha_;
    
    public:
    LPF();
    void init(double sampling_rate, double cutoff_frequency, const Eigen::VectorXd &init_value);
    Eigen::VectorXd get_filtered_value(const Eigen::VectorXd &raw);
};

#endif