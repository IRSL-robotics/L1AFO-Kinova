#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <vector>

#include "kinova_controller/data/robot_state.hpp"
#include "kinova_controller/data/gen3_state.hpp"
#include <yaml-cpp/yaml.h>

#include "kinova_controller/filter/lpf.hpp"

class KinovaGen3Base
{
    public:
    KinovaGen3Base(){}
    virtual ~KinovaGen3Base() = default;

    virtual bool init(std::shared_ptr<RobotState> robot_state_arg) =0 ;
    //destroy
    virtual void destroy()=0;
    virtual void update_state()=0;
    virtual bool set_joint_torque(const Eigen::VectorXd &u) =0;

    protected:
};
