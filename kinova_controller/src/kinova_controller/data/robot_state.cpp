#include "kinova_controller/data/robot_state.hpp"

#include <iostream>

RobotState::RobotState(const YAML::Node &config_arg, const pinocchio::Model &pinocchio_model_arg, const std::string & config_path_arg):
pinocchio_model(pinocchio_model_arg), pinocchio_data(pinocchio_model_arg), 
nq(pinocchio_model_arg.nq),
nv(pinocchio_model_arg.nv),
nu(pinocchio_model_arg.nv),
nx(pinocchio_model_arg.nv + pinocchio_model_arg.nq),
ndx(pinocchio_model_arg.nv *2),
config(config_arg),
config_path(config_path_arg)
{
    q = Eigen::VectorXd::Zero(nq);
    dq = Eigen::VectorXd::Zero(nv);
    q_d = Eigen::VectorXd::Zero(nq);
    dq_d = Eigen::VectorXd::Zero(nv);
    ddq_d = Eigen::VectorXd::Zero(nv);
    theta = Eigen::VectorXd::Zero(nv);
    dtheta = Eigen::VectorXd::Zero(nv);
    tau_J = Eigen::VectorXd::Zero(nv);
    joint_temperatures = Eigen::VectorXd::Zero(nv);
    ee_htm.resize(2);
    time = 0.0;
    
    try
    {
        is_controller_logging = config["is_controller_logging"].as<bool>();
    }
    catch(YAML::Exception &e)
    {   
        std::cout<<"ERROR: is_controller_logging is poorly defined in the yaml file"<<std::endl;
        std::cout<<"YAML Exception : "<<e.what()<<std::endl;
    }

    try
    {   
        is_rigid = config["is_rigid"].as<bool>();
        dt = config["dt"].as<double>();
        is_simulation = config["is_simulation"].as<bool>();
    }
    catch(YAML::Exception &e)
    {   
        std::cout<<"ERROR: is_rigid, dt, or is_simulation is poorly defined in the yaml file"<<std::endl;
        std::cout<<"YAML Exception : "<<e.what()<<std::endl;
    }

    if(is_controller_logging)
    {
        controller_logging_data.resize(nq, nv);
        controller_logging_data.setZero();
    }
}

void RobotState::update_config()
{
    try
    {
        config = YAML::LoadFile(config_path);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    
}

Eigen::VectorXd RobotState::integrate(const Eigen::VectorXd &q, const Eigen::VectorXd &dq, const double &dt)
{   
    Eigen::VectorXd qout(nq);
    qout.resize(nq); qout.setZero();
    pinocchio::integrate(pinocchio_model, q, dq*dt, qout);

    return qout;
}
// dxout = q2 - q1
Eigen::VectorXd RobotState::difference(const Eigen::VectorXd &q1, const Eigen::VectorXd &q2)
{   
    Eigen::VectorXd dqout(nv);
    dqout.setZero();

    pinocchio::difference(pinocchio_model, q1, q2, dqout);

    return dqout;
}

void RobotState::update_robot_state(const Gen3State & gen3_state)
{   
    q = gen3_state.converted_q;
    theta = gen3_state.converted_q;

    dq = gen3_state.joint_velocities;
    dtheta = gen3_state.joint_velocities;

    tau_J = gen3_state.joint_torques;
    joint_temperatures = gen3_state.joint_temperatures;
    pinocchio::forwardKinematics(pinocchio_model, pinocchio_data, q, dq);
    pinocchio::updateFramePlacements(pinocchio_model, pinocchio_data);
}

Eigen::MatrixXd RobotState::get_mass(const Eigen::VectorXd &q)
{
    pinocchio::crba(pinocchio_model,pinocchio_data,q); // compute mass matrix

    Eigen::MatrixXd temp = pinocchio_data.M;
    temp += temp.transpose().eval();
    temp.diagonal() = 0.5*temp.diagonal();

    return temp;
}

Eigen::MatrixXd RobotState::get_coriolis(const Eigen::VectorXd &q, const Eigen::VectorXd &dq)
{
    pinocchio::computeCoriolisMatrix(pinocchio_model,pinocchio_data, q, dq); // compute Coriolis matrix
    return pinocchio_data.C;
}

Eigen::MatrixXd RobotState::get_gravity(const Eigen::VectorXd &q)
{
    pinocchio::computeGeneralizedGravity(pinocchio_model, pinocchio_data, q);
    return pinocchio_data.g;
}

Eigen::MatrixXd RobotState::get_ee_jacobian_w(const Eigen::VectorXd &q, const std::string & ee_name)
{   
    pinocchio::framesForwardKinematics(pinocchio_model, pinocchio_data, q); // compute FK

    return this->get_ee_jacobian_w(ee_name);
}

Eigen::MatrixXd RobotState::get_ee_jacobian_w(const std::string & ee_name)
{
    auto ee_id = pinocchio_model.getFrameId(ee_name,pinocchio::FrameType::FIXED_JOINT);
    auto SE3 = pinocchio_data.oMf[ee_id]; // 1~7 joint frame index, 8 : end-effector index

    Eigen::MatrixXd Jacobian_b(6,pinocchio_model.nv); Jacobian_b.setZero();
    pinocchio::computeFrameJacobian(pinocchio_model, pinocchio_data, q, ee_id, Jacobian_b);

    Eigen::Matrix<double,6,6> gen_rot; gen_rot.setZero();
    Eigen::MatrixXd Jacobian_w(6,pinocchio_model.nv); Jacobian_w.setZero();
    gen_rot.topLeftCorner(3,3) = SE3.rotation();
    gen_rot.bottomRightCorner(3,3) = SE3.rotation(); 

    Jacobian_w = gen_rot * Jacobian_b;

    return Jacobian_w;
}

Eigen::MatrixXd RobotState::get_ee_jacobian_b(const Eigen::VectorXd &q, const std::string & ee_name)
{   
    pinocchio::framesForwardKinematics(pinocchio_model, pinocchio_data, q); // compute FK
    
    return this->get_ee_jacobian_b(ee_name);
}

Eigen::MatrixXd RobotState::get_ee_jacobian_b(const std::string & ee_name)
{
    auto ee_id = pinocchio_model.getFrameId(ee_name,pinocchio::FrameType::FIXED_JOINT);
    auto SE3 = pinocchio_data.oMf[ee_id]; // 1~7 joint frame index, 8 : end-effector index

    Eigen::MatrixXd Jacobian_b(6,pinocchio_model.nv); Jacobian_b.setZero();
    pinocchio::computeFrameJacobian(pinocchio_model, pinocchio_data, q, ee_id, Jacobian_b);
    Jacobian_b;

    return Jacobian_b;   
}

Eigen::Affine3d RobotState::get_ee_pose(const Eigen::VectorXd &q, const std::string & ee_name)
{   
    pinocchio::framesForwardKinematics(pinocchio_model, pinocchio_data, q); // compute FK

    return this->get_ee_pose(ee_name);
}

Eigen::Affine3d RobotState::get_ee_pose(const std::string & ee_name)
{
    Eigen::Affine3d ee_htm;
    auto ee_id = pinocchio_model.getFrameId(ee_name,pinocchio::FrameType::FIXED_JOINT);

    // pinocchio::updateFramePlacement(pinocchio_model, pinocchio_data, ee_id); // compute FK
    // bool is_exist = pinocchio_model.existFrame(end_effector_name,pinocchio::FrameType::FIXED_JOINT);
    
    auto SE3 = pinocchio_data.oMf[ee_id]; // 1~7 joint frame index, 8 : end-effector index
    ee_htm = SE3.toHomogeneousMatrix();
    
    return ee_htm;
}

Eigen::VectorXd RobotState::get_joint_nominal_torque(const Eigen::VectorXd &gacc)
{
    pinocchio::rnea(pinocchio_model, pinocchio_data, q, dq, gacc);

    Eigen::VectorXd joint_torque(7); joint_torque.setZero();
    joint_torque = pinocchio_data.tau.head(7);

    return joint_torque;
}


void RobotState::convert_q(Gen3State &gen3_state)
{   
    std::vector<double> q_vec;

    for(int i=0; i<nv; i++)
    {   
        //except for universe joint
        auto joint = pinocchio_model.joints[i+1];
        if(joint.nq() != joint.nv())
        {   
            q_vec.push_back(std::cos(gen3_state.joint_positions(i)));
            q_vec.push_back(std::sin(gen3_state.joint_positions(i)));
        }
        else
        {   
            q_vec.push_back(gen3_state.joint_positions(i));
        }
    }
    Eigen::VectorXd q_vec_eigen = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(q_vec.data(), q_vec.size());

    if(q_vec_eigen.size() != nq)
    {
        std::cout<<"q_vec_eigen size is not equal to pinocchio_model_.nq"<<std::endl;
        std::cout<<"q_vec_eigen size : "<<q_vec_eigen.size()<<std::endl;
        std::cout<<"pinocchio_model_.nq : "<<nq<<std::endl;
    }

    gen3_state.converted_q = q_vec_eigen;
}

void RobotState::convert_q(Eigen::VectorXd &q_arg)
{
    std::vector<double> q_vec;

    for(int i=0; i<nv; i++)
    {   
        //except for universe joint
        auto joint = pinocchio_model.joints[i+1];
        if(joint.nq() != joint.nv())
        {   
            q_vec.push_back(std::cos(q_arg(i)));
            q_vec.push_back(std::sin(q_arg(i)));
        }
        else
        {   
            q_vec.push_back(q_arg(i));
        }
    }
    q_arg= Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(q_vec.data(), q_vec.size());
}
