#pragma once

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include "kinova_controller/data/controller_state.hpp"
#include <iostream>

class ControllerGain
{   
    public:
        ControllerGain();
        ~ControllerGain();
        void init(const YAML::Node& node);

        JOINT_PD_Gains get_JOINT_PD_Gains(){return joint_pd_gains_;}
        JOINT_NRIC_Gains get_JOINT_NRIC_Gains(){return joint_nric_gains_;}
        TASK_PD_Gains get_TASK_PD_Gains(){return task_pd_gains_;}
        TASK_NIRC_Gains get_TASK_NIRC_Gains(){return task_nric_gains_;}
        JOINT_FRIC_Gains get_JOINT_FRIC_Gains(){return joint_fric_gains_;}
        TASK_FRIC_Gains get_TASK_FRIC_Gains(){return task_fric_gains_;}
        FIRST_FRIC_Gains get_FIRST_FRIC_Gains(){return first_fric_gains_;}
        SECOND_FRIC_Gains get_SECOND_FRIC_Gains(){return second_fric_gains_;}
        THIRD_FRIC_Gains get_THIRD_FRIC_Gains(){return thrid_fric_gains_;}
        INERTIA_RESHAPING_Gains get_INERTIA_RESHAPING_Gains(){return inertia_reshaping_gains_;}
        L1_FRIC_Gains get_L1_FRIC_Gains(){return l1_fric_gains_;}
        IMPLICIT_L1_FRIC_Gains get_IMPLICIT_L1_FRIC_Gains(){return implicit_l1_fric_gains_;}
        COULOMB_OBSERVER_Gains get_COULOMB_OBSERVER_Gains(){return coulomb_observer_gains_;}
        const ASYM_FRICTION_Gains& get_ASYM_FRICTION_Gains() const{return asym_friction_gains_;}
        const STATIC_FRICTION_Gains& get_STATIC_FRICTION_Gains() const{return static_friction_gains_;}

    private:
        void print_matrix(const Eigen::MatrixXd& matrix, const std::string& name);
        TORQUE_INTERFACE select_torque_interface(const std::string& torque_interface_str);
        
        JOINT_PD_Gains joint_pd_gains_;
        void read_JOINT_PD_Gains(const YAML::Node& node);
        
        JOINT_NRIC_Gains joint_nric_gains_;
        void read_JOINT_NRIC_Gains(const YAML::Node& node);

        TASK_PD_Gains task_pd_gains_;
        void read_TASK_PD_Gains(const YAML::Node& node);

        TASK_NIRC_Gains task_nric_gains_;
        void read_TASK_NRIC_Gains(const YAML::Node& node);

        JOINT_FRIC_Gains joint_fric_gains_;
        void read_JOINT_FRIC_Gains(const YAML::Node& node);

        TASK_FRIC_Gains task_fric_gains_;
        void read_TASK_FRIC_Gains(const YAML::Node& node);

        FIRST_FRIC_Gains first_fric_gains_;
        void read_FIRST_FRIC_Gains(const YAML::Node& node);

        SECOND_FRIC_Gains second_fric_gains_;
        void read_SECOND_FRIC_Gains(const YAML::Node& node);

        THIRD_FRIC_Gains thrid_fric_gains_;
        void read_THIRD_FRIC_Gains(const YAML::Node& node);

        INERTIA_RESHAPING_Gains inertia_reshaping_gains_;
        void read_INERTIA_RESHAPING_Gains(const YAML::Node& node);

        L1_FRIC_Gains l1_fric_gains_;
        void read_L1_FRIC_Gains(const YAML::Node& node);

        IMPLICIT_L1_FRIC_Gains implicit_l1_fric_gains_;
        void read_IMPLICIT_L1_FRIC_Gains(const YAML::Node& node);

        COULOMB_OBSERVER_Gains coulomb_observer_gains_;
        void read_COULOMB_OBSERVER_Gains(const YAML::Node& node);

        ASYM_FRICTION_Gains asym_friction_gains_;
        void read_ASYM_FRICTION_Gains(const YAML::Node& node);

        STATIC_FRICTION_Gains static_friction_gains_;
        void read_STATIC_FRICTION_Gains(const YAML::Node& node);
};
