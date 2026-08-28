#include "kinova_controller/data/gen3_state.hpp"


void Gen3State::print()
{
    std::cout<<"------------Gen3State------------"<<std::endl;
    std::cout<<"joint_positions : " << joint_positions.transpose()<<std::endl;
    std::cout<<"joint_velocities : " << joint_velocities.transpose()<<std::endl;
    std::cout<<"joint_torques : " << joint_torques.transpose()<<std::endl;
    std::cout<<"converted_q : " << converted_q.transpose()<<std::endl;
    std::cout<<"---------------------------------"<<std::endl;
}