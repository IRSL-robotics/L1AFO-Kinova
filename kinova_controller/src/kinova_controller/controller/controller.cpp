#include <kinova_controller/controller/controller.hpp>

#include <algorithm>
#include <array>
#include <cmath>

Controller::Controller()
{   
}   

void Controller::assign_robot_state(std::shared_ptr<RobotState> robot_state_arg)
{
    robot_state = robot_state_arg;
}

void Controller::init()
{   
    nq = robot_state->nq;
    nv = robot_state->nv;
    nu = robot_state->nu;
    dt = robot_state->dt;
    //for lugre friction
    z_.resize(nv); z_.setZero(); 
    joint_stiffness_matrix_.resize(nv,nv); joint_stiffness_matrix_.setZero();
    rotor_inertia_matrix_.resize(nv,nv); rotor_inertia_matrix_.setZero();

    YAML::Node config = robot_state->get_config();
    controller_gain.init(config);
    
    try
    {
        std::vector<double> joint_stiffness_vec = config["joint_stiffness"].as<std::vector<double>>();
        std::vector<double> rotor_inertia_vec = config["rotor_inertia"].as<std::vector<double>>();
        joint_stiffness_matrix_.diagonal() = Eigen::Map<Eigen::VectorXd>(joint_stiffness_vec.data(), joint_stiffness_vec.size());
        rotor_inertia_matrix_.diagonal() = Eigen::Map<Eigen::VectorXd>(rotor_inertia_vec.data(), rotor_inertia_vec.size());
    }
    catch(YAML::Exception &e)
    {   
        std::cout<<"ERROR: joint stiffness or rotor intertia are poorly defined in the yaml file"<<std::endl;
        std::cout<<"YAML Exception : "<<e.what()<<std::endl;
    }
    
    // initialize desired value
    robot_state->q_d = robot_state->q;
    robot_state->dq_d = Eigen::VectorXd::Zero(robot_state->nv);
    robot_state->ddq_d = Eigen::VectorXd::Zero(robot_state->nv);
    // robot_state->dtheta = Eigen::VectorXd::Zero(robot_state->nv);

    //Initialize nominal plant
    nominal_plant.q.resize(nq); nominal_plant.q.setZero();
    nominal_plant.q = robot_state->q;
    nominal_plant.dq.resize(nv); nominal_plant.dq.setZero();
    nominal_plant.ddq.resize(nv); nominal_plant.ddq.setZero();
    nominal_plant.theta.resize(nq); nominal_plant.theta.setZero();
    nominal_plant.theta = robot_state->theta;
    nominal_plant.dtheta.resize(nv); nominal_plant.dtheta.setZero();
    nominal_plant.ddtheta.resize(nv); nominal_plant.ddtheta.setZero();
    nominal_plant.friction.resize(nv); nominal_plant.friction.setZero();
    nominal_plant.integral_e_NR.resize(nv); nominal_plant.integral_e_NR.setZero();
}

void Controller::update_controller_gain(const YAML::Node& node)
{
    controller_gain.init(node);
}

CONTROLLER_SELECTOR Controller::select(const std::string & controller_type)
{   
    CONTROLLER_SELECTOR controller_selector;
    if(controller_type == "JOINT_NRIC")
    {   
        std::cout<<"JOINT_NRIC controller is selected"<<std::endl;
        if(this->init_NRIC_joint_controller())
        {
            controller_selector = CONTROLLER_SELECTOR::JOINT_NRIC;
            selector_prev = controller_selector;
        }
        else
        {
            std::cout<<"JOINT_NRIC controller initialization failed"<<std::endl;
            controller_selector = selector_prev;
        }
    }
    else if(controller_type == "JOINT_PD")
    {   
        std::cout<<"JOINT_PD controller is selected"<<std::endl;
        if(this->init_PD_joint_controller())
        {
            controller_selector = CONTROLLER_SELECTOR::JOINT_PD;
            selector_prev = controller_selector;
        }
        else
        {
            std::cout<<"JOINT_PD controller initialization failed"<<std::endl;
            controller_selector = selector_prev;
        }
    }
    else if(controller_type == "TASK_NRIC")
    {   
        std::cout<<"TASK_NRIC controller is selected"<<std::endl;
        if(this->init_NRIC_task_controller())
        {
            controller_selector = CONTROLLER_SELECTOR::TASK_NRIC;
            selector_prev = controller_selector;
        }
        else
        {
            std::cout<<"TASK_NRIC controller initialization failed"<<std::endl;
            controller_selector = selector_prev;
        }
    }
    else if(controller_type == "TASK_PD")
    {   
        std::cout<<"TASK_PD controller is selected"<<std::endl;
        if(this->init_PD_task_controller())
        {
            controller_selector = CONTROLLER_SELECTOR::TASK_PD;
            selector_prev = controller_selector;
        }
        else
        {
            std::cout<<"TASK_PD controller initialization failed"<<std::endl;
            controller_selector = selector_prev;
        }
    }
    else if(controller_type == "GRAVITY_COMPENSATION")
    {   
        std::cout<<"GRAVITY_COMPENSATION controller is selected"<<std::endl;
        if(this->init_gravity_compensation())
        {
            controller_selector = CONTROLLER_SELECTOR::GRAVITY_COMPENSATION;
            selector_prev = controller_selector;
        }
        else
        {
            std::cout<<"GRAVITY_COMPENSATION controller initialization failed"<<std::endl;
            controller_selector = selector_prev;
        }

    }
    else if(controller_type == "JOINT_PD_FRIC")
    {   
        std::cout<<"JOINT_PD_FRIC controller is selected"<<std::endl;
        if(this->init_FRIC_joint_controller())
        {
            controller_selector = CONTROLLER_SELECTOR::JOINT_PD_FRIC;
            selector_prev = controller_selector;
        }
        else
        {
            std::cout<<"JOINT_PD_FRIC controller initialization failed"<<std::endl;
            controller_selector = selector_prev;
        }
    }
    else if(controller_type == "TASK_PD_FRIC")
    {   
        std::cout<<"TASK_PD_FRIC controller is selected"<<std::endl;
        if(this->init_FRIC_task_controller())
        {
            controller_selector = CONTROLLER_SELECTOR::TASK_PD_FRIC;
            selector_prev = controller_selector;
        }
        else
        {
            std::cout<<"TASK_PD_FRIC controller initialization failed"<<std::endl;
            controller_selector = selector_prev;
        }
    }    
    else
    {   
        std::cout<<"controller type is not valid"<<std::endl;
        controller_selector = selector_prev;
    }
    
    return controller_selector;
}

Eigen::VectorXd Controller::get_control_input(const int& selector)
{   
    Eigen::VectorXd u = Eigen::VectorXd::Zero(nu);
    switch (selector)
    {
    case CONTROLLER_SELECTOR::JOINT_PD:
        u=this->PD_joint_controller();
        break;
    case CONTROLLER_SELECTOR::JOINT_PD_FRIC:
        u=this->FRIC_joint_controller();
        break;

    case CONTROLLER_SELECTOR::TASK_PD:
        u=this->PD_task_controller();
        break;

    case CONTROLLER_SELECTOR::TASK_PD_FRIC:
        u=this->FRIC_task_controller();
        break;

    case CONTROLLER_SELECTOR::JOINT_NRIC:
        u = this->NRIC_joint_controller();
        break;

    case CONTROLLER_SELECTOR::TASK_NRIC:
        u = this->NRIC_task_controller();
        break;

    case CONTROLLER_SELECTOR::GRAVITY_COMPENSATION:
        u = this->gravity_compensation();
        break;

    default:
        break;
    }

    return u;
}

bool Controller::init_PD_joint_controller()
{
    // initialize desired value
    robot_state->q_d = robot_state->q;
    robot_state->dq_d = Eigen::VectorXd::Zero(robot_state->nv);
    robot_state->ddq_d = Eigen::VectorXd::Zero(robot_state->nv);

    return true;
}

Eigen::VectorXd Controller::PD_joint_controller()
{
    //Gain
    const JOINT_PD_Gains &gain = controller_gain.get_JOINT_PD_Gains();

    Eigen::VectorXd control_motor_torque;

    Eigen::VectorXd gravity_compensation_torque(nv);
    gravity_compensation_torque = robot_state->get_gravity(robot_state->q_d);
    control_motor_torque = -gain.Kp*(robot_state->difference(robot_state->q_d, robot_state->q))-gain.Kd*(robot_state->dq - robot_state->dq_d)+gravity_compensation_torque;

    
    return control_motor_torque;
}

bool Controller::init_NRIC_joint_controller()
{
    // initialize desired value
    robot_state->q_d = robot_state->q;
    robot_state->dq_d = Eigen::VectorXd::Zero(robot_state->nv);
    robot_state->ddq_d = Eigen::VectorXd::Zero(robot_state->nv);

    // initialize nominal plant
    nominal_plant.q.resize(nq); nominal_plant.q.setZero();
    nominal_plant.q = robot_state->q;
    nominal_plant.dq.resize(nv); nominal_plant.dq.setZero();
    nominal_plant.ddq.resize(nv); nominal_plant.ddq.setZero();

    return true;
}

Eigen::VectorXd Controller::NRIC_joint_controller()
{   
    const JOINT_NRIC_Gains &gain = controller_gain.get_JOINT_NRIC_Gains();

    Eigen::MatrixXd M = robot_state->get_mass(robot_state->q) + gain.reflected_inertia;
    Eigen::MatrixXd C = robot_state->get_coriolis(robot_state->q,robot_state->dq);
    Eigen::VectorXd g = robot_state->get_gravity(robot_state->q);

    Eigen::MatrixXd nominal_M = robot_state->get_mass(nominal_plant.q) + gain.reflected_inertia;
    Eigen::MatrixXd nominal_C = robot_state->get_coriolis(nominal_plant.q,nominal_plant.dq);
    Eigen::VectorXd nominal_g = robot_state->get_gravity(nominal_plant.q);

    Eigen::VectorXd tauC(nv); tauC.setZero();
    Eigen::VectorXd tauA(nv); tauA.setZero();
    Eigen::VectorXd tauGrav(nv); tauGrav.setZero();
    Eigen::VectorXd tauPD(nv); tauPD.setZero();

    /// ****** NRIC
    
    // Eigen::VectorXd e_DN = robot_state.q_d - nominal_plant.q;
    Eigen::VectorXd e_DN = robot_state->difference(nominal_plant.q, robot_state->q_d);
    Eigen::VectorXd edot_DN = robot_state->dq_d - nominal_plant.dq;
    Eigen::VectorXd e_RN = robot_state->difference(nominal_plant.q, robot_state->q);
    Eigen::VectorXd edot_RN = robot_state->dq - nominal_plant.dq;

    tauC = nominal_M*(robot_state->ddq_d + gain.k1 * edot_DN + gain.k2 * e_DN) + nominal_C*nominal_plant.dq + nominal_g;

    nominal_plant.ddq = nominal_M.inverse() * (tauC - nominal_C*nominal_plant.dq - nominal_g);
    nominal_plant.dq += nominal_plant.ddq * dt;
    nominal_plant.q = robot_state->integrate(nominal_plant.q, nominal_plant.dq, dt);

    Eigen::VectorXd s_vector = (edot_RN + gain.Kp * e_RN + gain.Ki * nominal_plant.integral_e_NR);
    nominal_plant.integral_e_NR += e_RN * dt;

    
    tauA = -(gain.K + gain.gamma) * s_vector;

    // Eigen::VectorXd e_RD = robot_state.q - robot_state.q_d;
    Eigen::VectorXd e_RD = robot_state->difference(robot_state->q_d, robot_state->q);

    return tauA + tauC;
}

bool Controller::init_PD_task_controller()
{
    // initialize desired value
    robot_state->q_d = robot_state->q;
    robot_state->dq_d = Eigen::VectorXd::Zero(robot_state->nv);
    robot_state->ddq_d = Eigen::VectorXd::Zero(robot_state->nv);
    
    return true;
}

Eigen::VectorXd Controller::PD_task_controller()
{   
    //gain
    const TASK_PD_Gains & gain = controller_gain.get_TASK_PD_Gains();

    // task space error
    pinocchio::SE3 dhtm(robot_state->get_ee_pose(robot_state->q_d, "end_effector").matrix());
    pinocchio::SE3 htm(robot_state->get_ee_pose(robot_state->q, "end_effector").matrix());
    pinocchio::SE3 rhtm = dhtm.inverse() * htm;
    Eigen::Matrix<double, 6, 1> error_task = -pinocchio::log6(rhtm).toVector();

    // projection onto joint space
    auto nominalJ = robot_state->get_ee_jacobian_b(robot_state->q, "end_effector");
    Eigen::VectorXd e_task = gain.Kp * error_task;
    Eigen::VectorXd edot_task = gain.Kd * nominalJ * robot_state->dq;

    Eigen::VectorXd g = robot_state->get_gravity(robot_state->q);

    Eigen::VectorXd u = nominalJ.transpose()*(-edot_task + e_task) + g;

    return u;
}

bool Controller::init_NRIC_task_controller()
{
    // initialize desired value
    robot_state->q_d = robot_state->q;
    robot_state->dq_d = Eigen::VectorXd::Zero(robot_state->nv);
    robot_state->ddq_d = Eigen::VectorXd::Zero(robot_state->nv);

    // initialize nominal plant
    nominal_plant.q.resize(nq); nominal_plant.q.setZero();
    nominal_plant.q = robot_state->q;
    nominal_plant.dq.resize(nv); nominal_plant.dq.setZero();
    nominal_plant.ddq.resize(nv); nominal_plant.ddq.setZero();

    return true;
}

Eigen::VectorXd Controller::NRIC_task_controller()
{   
    //gain
    const TASK_NIRC_Gains & gain = controller_gain.get_TASK_NIRC_Gains();

    Eigen::MatrixXd M = robot_state->get_mass(robot_state->q) + gain.reflected_inertia;
    Eigen::MatrixXd C = robot_state->get_coriolis(robot_state->q,robot_state->dq);
    Eigen::VectorXd g = robot_state->get_gravity(robot_state->q);

    Eigen::MatrixXd nominal_M = robot_state->get_mass(nominal_plant.q) + gain.reflected_inertia;
    Eigen::MatrixXd nominal_C = robot_state->get_coriolis(nominal_plant.q,nominal_plant.dq);
    Eigen::VectorXd nominal_g = robot_state->get_gravity(nominal_plant.q);

    Eigen::VectorXd tauC(nv); tauC.setZero();
    Eigen::VectorXd tauA(nv); tauA.setZero();
    Eigen::VectorXd tauGrav(nv); tauGrav.setZero();
    Eigen::VectorXd tauPD(nv); tauPD.setZero();

    /// ****** NRIC
    pinocchio::SE3 dhtm(robot_state->get_ee_pose(robot_state->q_d, "end_effector").matrix());
    pinocchio::SE3 nominal_htm(robot_state->get_ee_pose(nominal_plant.q, "end_effector").matrix());
    pinocchio::SE3 rhtm = dhtm.inverse() * nominal_htm;
    Eigen::Matrix<double, 6, 1> error_task = -pinocchio::log6(rhtm).toVector();

    auto nominalJ = robot_state->get_ee_jacobian_b(nominal_plant.q, "end_effector");
    Eigen::VectorXd e_task_DN = gain.k2 * error_task;
    Eigen::VectorXd edot_task_DN = gain.k1 * nominalJ * nominal_plant.dq;

    Eigen::VectorXd temp = robot_state->ddq_d + nominalJ.transpose()*(-edot_task_DN + e_task_DN);
    tauC = nominal_M*temp + nominal_C*nominal_plant.dq + nominal_g;

    nominal_plant.ddq = nominal_M.inverse() * (tauC - nominal_C*nominal_plant.dq - nominal_g);
    nominal_plant.dq += nominal_plant.ddq * dt;
    nominal_plant.q = robot_state->integrate(nominal_plant.q, nominal_plant.dq, dt);

    Eigen::VectorXd e_RN = robot_state->difference(nominal_plant.q, robot_state->q);
    Eigen::VectorXd edot_RN = robot_state->dq - nominal_plant.dq;

    Eigen::VectorXd s_vector = (edot_RN + gain.Kp * e_RN + gain.Ki * nominal_plant.integral_e_NR);
    nominal_plant.integral_e_NR += e_RN * dt;

    tauA = -(gain.K + gain.gamma) * s_vector;


    pinocchio::SE3 real_htm(robot_state->get_ee_pose(robot_state->q, "end_effector").matrix());
    pinocchio::SE3 rdhtm = dhtm.inverse() * real_htm;
    Eigen::Matrix<double, 6, 1> e_RD = -pinocchio::log6(rdhtm).toVector();
    
    return tauA + tauC;
}
bool Controller::init_FRIC_joint_controller()
{
    // initialize desired value
    robot_state->q_d = robot_state->q;
    robot_state->dq_d = Eigen::VectorXd::Zero(robot_state->nv);

    robot_state->time = 0.0;

    // initialize torque control interface

    switch (controller_gain.get_JOINT_FRIC_Gains().torque_interface)
    {
    case TORQUE_INTERFACE::FIRST_ORDER_FRIC:
        this->init_first_order_friction_compensation();
        break;
    case TORQUE_INTERFACE::SECOND_ORDER_FRIC:
        this->init_second_order_friction_compensation();
        break;
    case TORQUE_INTERFACE::THIRD_ORDER_FRIC:
        this->init_third_order_friction_compensation();
        break;
    case TORQUE_INTERFACE::INERTIA_RESHAPING:
        this->init_intertia_reshaping();
        break;
    case TORQUE_INTERFACE::L1_FRIC:
        this->init_l1_friction_compensation();
        break;  
    case TORQUE_INTERFACE::IMPLICIT_L1_FRIC:
        this->init_implicit_l1_friction_compensation();
        break;  
    case TORQUE_INTERFACE::COULOMB_OBSERVER:
        this->init_coulomb_observer_friction_compensation();
        break;
    case TORQUE_INTERFACE::ASYM_FRICTION:
        if (!this->init_asym_friction_compensation()) return false;
        break;
    case TORQUE_INTERFACE::STATIC_FRICTION:
        if (!this->init_static_friction_compensation()) return false;
        break;
    default:
        return false;
        break;
    }

    for(int i=0; i<10; i++)
    {
        this->FRIC_joint_controller();
    }

    std::cout<<"FRIC_joint_controller is initialized"<<std::endl;

    return true;
}

Eigen::VectorXd Controller::FRIC_joint_controller()
{
    const JOINT_FRIC_Gains &gain = controller_gain.get_JOINT_FRIC_Gains();

    // Eigen::VectorXd g = robot_state->get_gravity(robot_state->q_d); // gravity compensation torque w.r.t desired q
    // Eigen::VectorXd theta_d = robot_state->q_d + (gain.joint_stiffness_matrix).inverse() * g; // desired theta

    Eigen::VectorXd traj_d(nv), theta_d(nv), traj_d_final(nv);
    Eigen::VectorXd init_theta(nq), d_traj_d(nq), g(nv);
    double period = 5.0;
    double final_time = 7.5;
    init_theta << 0.0, 0.386, 0.0, 2.29, 0.0, -1.07, 0.0;
    
    for (int i=0; i<7; i++){
        traj_d(i) = init_theta(i) + 0.1*sin(3.141592/5*(robot_state->time));
        d_traj_d(i) = 0.1*3.141592/5*cos(3.141592/5*(robot_state->time));
        traj_d_final(i) = init_theta(i) + 0.1*sin(3.141592/period*final_time);
    }
    
    if ((robot_state->time < final_time) && (robot_state->time >= 0))
    {
        robot_state->convert_q(traj_d);
        robot_state->dq_d = d_traj_d;
        g = robot_state->get_gravity(traj_d);
        theta_d = traj_d + (gain.joint_stiffness_matrix).inverse() * g; // desired theta
    }
    else
    {
        robot_state->convert_q(traj_d_final);
        robot_state->dq_d = Eigen::VectorXd::Zero(nv);
        g = robot_state->get_gravity(traj_d_final);
        theta_d = traj_d_final + (gain.joint_stiffness_matrix).inverse() * g; // desired theta

    }
    robot_state->time += dt;

    // Hard-coded desired trajectory retained from the task-space PD trajectory-tracking experiment.
    // TODO: Add desired-trajectory publisher/subscriber logic instead of defining the trajectory in the controller.
    // Eigen::VectorXd traj_d(nv), theta_d(nv), traj_d_final(nv), g(nv);
    // Eigen::VectorXd init_theta(nq), amplitude(nq), d_traj_d(nq), dd_traj_d(nq);
    // double period = 7; // real period : X2 
    // double final_time = 24.5;
    // amplitude << -0.4, -0.4, -0.4, -0.2, -0.2, 0.2, -0.2;
    // // amplitude << 0.4, 0.4, 0.4, 0.4, 0.4, -0.4, 0.4;
    // init_theta << 0.0, -0.2618, 0.0, 1.85, 0.0, 1.571, 0.0;
    
    // for (int i=0; i<7; i++){
    //     traj_d(i) = init_theta(i) + amplitude(i)*sin(3.141592/period*(robot_state->time));
    //     d_traj_d(i) = amplitude(i)*3.141592/period*cos(3.141592/period*(robot_state->time));
    //     dd_traj_d(i) = -amplitude(i)*(3.141592/period)*(3.141592/period)*sin(3.141592/period*(robot_state->time));
    //     traj_d_final(i) = init_theta(i) + amplitude(i)*sin(3.141592/period*final_time);
    // }
    
    // if ((robot_state->time < final_time)&(robot_state->time >= 0))
    // {
    //     robot_state->convert_q(traj_d);
    //     robot_state->dq_d = d_traj_d;
    //     robot_state->ddq_d = dd_traj_d;
    //     g = robot_state->get_gravity(traj_d);
    //     theta_d = traj_d + (gain.joint_stiffness_matrix).inverse() * g; // desired theta
    // }
    // else
    // {
    //     robot_state->convert_q(traj_d_final);
    //     robot_state->dq_d = Eigen::VectorXd::Zero(nv);
    //     robot_state->ddq_d = Eigen::VectorXd::Zero(nv);
    //     g = robot_state->get_gravity(traj_d_final);
    //     theta_d = traj_d_final + (gain.joint_stiffness_matrix).inverse() * g; // desired theta
    // }
    // robot_state->time += 0.001;

    Eigen::VectorXd tauC(nu); tauC.setZero();
    Eigen::VectorXd u(nu); u.setZero();
    Eigen::VectorXd e_DN(nv); e_DN.setZero();
    Eigen::VectorXd edot_DN(nv); edot_DN.setZero();

    
    TORQUE_INTERFACE torque_interface = controller_gain.get_JOINT_FRIC_Gains().torque_interface;

    if(torque_interface == TORQUE_INTERFACE::INERTIA_RESHAPING ||
       torque_interface == TORQUE_INTERFACE::FIRST_ORDER_FRIC ||
       torque_interface == TORQUE_INTERFACE::COULOMB_OBSERVER ||
       torque_interface == TORQUE_INTERFACE::ASYM_FRICTION ||
       torque_interface == TORQUE_INTERFACE::STATIC_FRICTION)
    {
        // Real state feedback
        e_DN = robot_state->difference(robot_state->theta, theta_d);
        edot_DN = robot_state->dq_d - robot_state->dtheta;
    }
    else
    {
        // Nominal state feedback
        e_DN = robot_state->difference(nominal_plant.theta, theta_d);
        edot_DN = robot_state->dq_d - nominal_plant.dtheta;
    }
    tauC = gain.Kp*e_DN + gain.Kd*edot_DN + g; // motor PD controller
    // tauC = gain.Kp*e_DN + gain.Kd*edot_DN + g + rotor_inertia_matrix_*robot_state->ddq_d; // motor PD tracking controller with nonlinearity compensation term

    switch (controller_gain.get_JOINT_FRIC_Gains().torque_interface)
    {
    case TORQUE_INTERFACE::FIRST_ORDER_FRIC:
        u = this->first_order_friction_compensation(tauC);
        break;
    case TORQUE_INTERFACE::SECOND_ORDER_FRIC:
        u = this->second_order_friction_compensation(tauC);
        break;
    case TORQUE_INTERFACE::THIRD_ORDER_FRIC:
        u = this->third_order_friction_compensation(tauC);
        break;
    case TORQUE_INTERFACE::INERTIA_RESHAPING:
        u = this->inertia_reshaping(tauC);
        break;
    case TORQUE_INTERFACE::L1_FRIC:
        u = this->l1_friction_compensation(tauC);
        break;   
    case TORQUE_INTERFACE::IMPLICIT_L1_FRIC:
        u = this->implicit_l1_friction_compensation(tauC);
        break;    
    case TORQUE_INTERFACE::COULOMB_OBSERVER:
        u = this->coulomb_observer_friction_compensation(tauC);
        break;
    case TORQUE_INTERFACE::ASYM_FRICTION:
        u = this->asym_friction_compensation(tauC);
        break;
    case TORQUE_INTERFACE::STATIC_FRICTION:
        u = this->static_friction_compensation(tauC);
        break;
    default:
        return Eigen::VectorXd::Zero(1); // ERROR signal
        break;
    }

    Eigen::VectorXd tauf(nu); tauf.setZero();
    if(robot_state->is_simulation)
    {   
        // lugre friction modeling 
        double sigma_0, sigma_1, sigma_2, Fc, Fs, vs;
        double sigma_02, sigma_12, sigma_22, Fc2, Fs2, vs2;
        sigma_0 = 2750; sigma_1 = 45.2; sigma_2 = 1.819; Fc = 8.875; Fs = 6.975275; vs = 0.6109;
        sigma_02 = 2750; sigma_12 = 45.2; sigma_22 = 1.819; Fc2 = 8.875; Fs2 = 6.975275; vs2 = 0.6109; // for joint 5~7
        
        Eigen::VectorXd r(nv), g_v(nv), z_dot(nv); 
        r.setZero(); g_v.setZero(); z_dot.setZero(); 
        r = -(robot_state->dtheta/vs) * (robot_state->dtheta/vs);

        for(int i=0; i<4; i++)
            {
                g_v(i) = Fc + (Fs - Fc) * exp(r(i));   
                z_dot(i) = robot_state->dtheta(i) - sigma_0 * fabs(robot_state->dtheta(i)) * z_(i) / g_v(i);
                tauf(i) = -(sigma_0 * z_(i) + sigma_1 * z_dot(i) + sigma_2 * robot_state->dtheta(i));
            } 
        for(int i=4; i<nv; i++)
            {
                g_v(i) = Fc2 + (Fs2 - Fc2) * exp(r(i));   
                z_dot(i) = robot_state->dtheta(i) - sigma_02 * fabs(robot_state->dtheta(i)) * z_(i) / g_v(i);
                tauf(i) = -(sigma_02 * z_(i) + sigma_12 * z_dot(i) + sigma_22 * robot_state->dtheta(i));
            }         
        z_ = z_ + z_dot * dt;
        u += tauf;
    }

    //for data logging
    if(robot_state->is_controller_logging)
    {
        ControllerLoggingData & data = robot_state->controller_logging_data;

        data.tau_f = tauf;
        data.nominal_theta = nominal_plant.theta;
        data.nominal_dtheta = nominal_plant.dtheta;
        data.e_dn = e_DN;
        data.theta_des = theta_d;
    }
    return u;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool Controller::init_FRIC_task_controller()
{
    // initialize desired value
    robot_state->q_d = robot_state->q;
    robot_state->dq_d = Eigen::VectorXd::Zero(robot_state->nv);
    pinocchio::SE3 dhtm(robot_state->get_ee_pose(robot_state->q, "end_effector").matrix());
    robot_state->htm_d = dhtm;
    robot_state->theta_init = robot_state->q;
    robot_state->time = 0.0;

    robot_state->x_des_prev = Eigen::VectorXd::Zero(3);
    robot_state->x_des_prev << 0.466941, -0.024872,  0.387849;  
    robot_state->theta_init = robot_state->q;

    // initialize torque control interface
    switch (controller_gain.get_TASK_FRIC_Gains().torque_interface)
    {
    case TORQUE_INTERFACE::FIRST_ORDER_FRIC:
        this->init_first_order_friction_compensation();
        break;
    case TORQUE_INTERFACE::SECOND_ORDER_FRIC:
        this->init_second_order_friction_compensation();
        break;
    case TORQUE_INTERFACE::THIRD_ORDER_FRIC:
        this->init_third_order_friction_compensation();
        break;
    case TORQUE_INTERFACE::INERTIA_RESHAPING:
        this->init_intertia_reshaping();
        break;
    case TORQUE_INTERFACE::L1_FRIC:
        this->init_l1_friction_compensation();
        break;  
    case TORQUE_INTERFACE::IMPLICIT_L1_FRIC:
        this->init_implicit_l1_friction_compensation();
        break; 
    case TORQUE_INTERFACE::COULOMB_OBSERVER:
        this->init_coulomb_observer_friction_compensation();
        break;
    case TORQUE_INTERFACE::ASYM_FRICTION:
        if (!this->init_asym_friction_compensation()) return false;
        break;
    case TORQUE_INTERFACE::STATIC_FRICTION:
        if (!this->init_static_friction_compensation()) return false;
        break;
    default:
        return false;
        break;
    }
    for(int i=0; i<10; i++)
    {
        this->FRIC_task_controller();
    }

    std::cout<<"FRIC_task_controller is initialized"<<std::endl;

    return true;
}

Eigen::VectorXd Controller::FRIC_task_controller()
{
    const TASK_FRIC_Gains &gain = controller_gain.get_TASK_FRIC_Gains();
    TORQUE_INTERFACE torque_interface = controller_gain.get_TASK_FRIC_Gains().torque_interface;

    Eigen::VectorXd tauC(nu); tauC.setZero();
    Eigen::VectorXd u(nu); u.setZero();
    Eigen::VectorXd e_task(6); e_task.setZero();

    // gravity compensation torque w.r.t quasi static q
    int iter = 2;
    Eigen::VectorXd mapping_T(nq), l(nv), q_bar(nq), k(nq);
    Eigen::VectorXd mapping_T_nom(nq), l_nom(nv), q_bar_nom(nq), k_nom(nq);
    Eigen::VectorXd h(nq);
    h << 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0;

    q_bar = robot_state->theta; // nq
    q_bar_nom = nominal_plant.theta;

    for (int i=0; i<iter; i++)
    {
        l = robot_state->get_gravity(robot_state->theta); // nv
        k = (gain.joint_stiffness_matrix).inverse() * l;
        robot_state->convert_q(k);
        mapping_T = robot_state->theta - (k - h); 
        q_bar = mapping_T;

        l_nom = robot_state->get_gravity(nominal_plant.theta); // nv
        k_nom = (gain.joint_stiffness_matrix).inverse() * l_nom;
        robot_state->convert_q(k_nom);
        mapping_T_nom = nominal_plant.theta - (k_nom - h); 
        q_bar_nom = mapping_T_nom;
    }

    Eigen::VectorXd init_x(3), final_x(3), x_des(3), x_dot_des(3), x_ddot_des(6);
    double total_time = 20;
    double segment_time = total_time / 4;
    double length = 0.20;
    init_x << 0.466941, -0.024872,  0.387849;
    final_x << 0.533, -0.2279,  0.3713;  

    // for collision tracking experiment
    for (int i = 0; i < 3; i++){
        x_des(i) = DyrosMath::cubic(robot_state->time, 0.0, segment_time, init_x(i), final_x(i), 0, 0);}


    // Hard-coded desired trajectory retained from the task-space PD trajectory-tracking experiment.
    // TODO: Add desired-trajectory publisher/subscriber logic instead of defining the trajectory in the controller.
    // Eigen::MatrixXd waypoints(4, 3);
    // waypoints << init_x(0), init_x(1), init_x(2) - length,
    //              init_x(0), init_x(1) + length, init_x(2) - length, 
    //              init_x(0), init_x(1) + length, init_x(2),
    //              init_x(0), init_x(1), init_x(2); 

    // x_des(0) = init_x(0);
    
    // if (robot_state->time < segment_time)
    //     for (int i = 1; i < 3; i++){
    //         x_des(i) = DyrosMath::cubic(robot_state->time, 0.0, segment_time, init_x(i), waypoints(0,i), 0, 0);}
    // else if ((robot_state->time > segment_time)&&(robot_state->time < 2*segment_time))
    //     for (int i = 1; i < 3; i++){
    //         x_des(i) = DyrosMath::cubic(robot_state->time, segment_time, 2*segment_time, waypoints(0,i), waypoints(1,i), 0, 0);}
    // else if ((robot_state->time > 2*segment_time)&&(robot_state->time < 3*segment_time))
    //     for (int i = 1; i < 3; i++){
    //         x_des(i) = DyrosMath::cubic(robot_state->time, 2*segment_time, 3*segment_time, waypoints(1,i), waypoints(2,i), 0, 0);}     
    // else if (robot_state->time > 3*segment_time)
    //     for (int i = 1; i < 3; i++){
    //         x_des(i) = DyrosMath::cubic(robot_state->time, 3*segment_time, 4*segment_time, waypoints(2,i), waypoints(3,i), 0, 0);}  

    robot_state->time += dt;

    x_dot_des = (x_des - robot_state->x_des_prev)/dt;
    Eigen::VectorXd x_dot_des_final(3);
    x_dot_des_final << -x_dot_des(2), x_dot_des(1), x_dot_des(0); // convention : position = [z, y, x]
    robot_state->x_des_prev = x_des;

    // pinocchio::SE3 dhtm(robot_state->get_ee_pose(robot_state->q_d, "end_effector").matrix());
    (robot_state->htm_d).translation() = x_des;
    pinocchio::SE3 dhtm = robot_state->htm_d;
    pinocchio::SE3 htm(robot_state->get_ee_pose(q_bar, "end_effector").matrix());
    pinocchio::SE3 nhtm(robot_state->get_ee_pose(q_bar_nom, "end_effector").matrix());
    pinocchio::SE3 rhtm = dhtm.inverse() * htm;
    pinocchio::SE3 nrhtm = dhtm.inverse() * nhtm;

    auto J = robot_state->get_ee_jacobian_b(q_bar, "end_effector");
    auto nJ = robot_state->get_ee_jacobian_b(q_bar_nom, "end_effector");

    // null space projection input 
    Eigen::MatrixXd N_2(7,7), I(7,7), Kp(7,7), Kd(7,7), Jacobian_general_inv(7,6);
    Eigen::VectorXd error_theta(7), tau_2(nv);
    Kp.setIdentity(); Kd.setIdentity();
    Kp = Kp*0; Kd = Kd*5;

    Eigen::VectorXd theta_d(nq);
    theta_d = robot_state->theta_init;
    I.setIdentity();

    if(torque_interface == TORQUE_INTERFACE::INERTIA_RESHAPING ||
       torque_interface == TORQUE_INTERFACE::FIRST_ORDER_FRIC ||
       torque_interface == TORQUE_INTERFACE::IMPLICIT_L1_FRIC ||
       torque_interface == TORQUE_INTERFACE::COULOMB_OBSERVER ||
       torque_interface == TORQUE_INTERFACE::ASYM_FRICTION ||
       torque_interface == TORQUE_INTERFACE::STATIC_FRICTION)
    {   
        // real state feedback
        Eigen::Matrix<double, 6, 1> error_task = -pinocchio::log6(rhtm).toVector();
        Eigen::Matrix<double, 6, 1> error_dot_task;
        error_dot_task.head(3) = x_dot_des_final - (J*robot_state->dtheta).head(3);  
        error_dot_task.tail(3) =  - (J*robot_state->dtheta).tail(3);

        e_task = error_task;
        Eigen::VectorXd g = robot_state->get_gravity(q_bar); 
        tauC = J.transpose()*(gain.Kp*e_task + gain.Kd*error_dot_task) + g;

        Jacobian_general_inv = J.transpose() * (J * J.transpose()).inverse();
        N_2 =  I - J.transpose() * Jacobian_general_inv.transpose();
        error_theta = robot_state->difference(theta_d, robot_state->theta);
        tau_2 = N_2 * (-Kp*error_theta-Kd*(robot_state->dtheta));
    }
    else
    {   
        // nominal state feedback
        Eigen::Matrix<double, 6, 1> error_task = -pinocchio::log6(nrhtm).toVector();
        Eigen::Matrix<double, 6, 1> error_dot_task;
        error_dot_task.head(3) = x_dot_des_final - (nJ*nominal_plant.dtheta).head(3);  
        error_dot_task.tail(3) =  - (nJ*nominal_plant.dtheta).tail(3); 

        e_task = error_task;
        Eigen::VectorXd g = robot_state->get_gravity(q_bar_nom); 
        tauC = nJ.transpose()*(gain.Kp*e_task + gain.Kd*error_dot_task) + g;

        Jacobian_general_inv = nJ.transpose() * (nJ * nJ.transpose()).inverse();
        N_2 =  I - nJ.transpose() * Jacobian_general_inv.transpose();
        error_theta = robot_state->difference(theta_d, nominal_plant.theta);
        tau_2 = N_2 * (-Kp*error_theta-Kd*(nominal_plant.dtheta));
    }
    tauC += tau_2;

    // switch (controller_gain.get_TASK_FRIC_Gains().torque_interface)
    switch (torque_interface)
    {
    case TORQUE_INTERFACE::FIRST_ORDER_FRIC:
        u = this->first_order_friction_compensation(tauC);
        break;
    case TORQUE_INTERFACE::SECOND_ORDER_FRIC:
        u = this->second_order_friction_compensation(tauC);
        break;
    case TORQUE_INTERFACE::THIRD_ORDER_FRIC:
        u = this->third_order_friction_compensation(tauC);
        break;
    case TORQUE_INTERFACE::INERTIA_RESHAPING:
        u = this->inertia_reshaping(tauC);
        break;
    case TORQUE_INTERFACE::L1_FRIC:
        u = this->l1_friction_compensation(tauC);
        break;   
    case TORQUE_INTERFACE::IMPLICIT_L1_FRIC:
        u = this->implicit_l1_friction_compensation(tauC);
        break;    
    case TORQUE_INTERFACE::COULOMB_OBSERVER:
        u = this->coulomb_observer_friction_compensation(tauC);
        break;
    case TORQUE_INTERFACE::ASYM_FRICTION:
        u = this->asym_friction_compensation(tauC);
        break;
    case TORQUE_INTERFACE::STATIC_FRICTION:
        u = this->static_friction_compensation(tauC);
        break;
    default:
        return Eigen::VectorXd::Zero(1); // ERROR signal
        break;
    }

    Eigen::VectorXd tauf(nu); tauf.setZero();
    if(robot_state->is_simulation)
    {   
        // lugre friction modeling 
        double sigma_0, sigma_1, sigma_2, Fc, Fs, vs;
        double sigma_02, sigma_12, sigma_22, Fc2, Fs2, vs2;
        sigma_0 = 2750; sigma_1 = 45.2; sigma_2 = 1.819; Fc = 8.875; Fs = 6.975275; vs = 0.6109;
        
        Eigen::VectorXd r(nv), g_v(nv), z_dot(nv); 
        r.setZero(); g_v.setZero(); z_dot.setZero(); 
        r = -(robot_state->dtheta/vs) * (robot_state->dtheta/vs);

        tauf = -(sigma_0 * z_ + sigma_1 * z_dot + sigma_2 * robot_state->dtheta);
    
        z_ = z_ + z_dot * dt;
        // u += tauf;
    }

    //for data logging
    if(robot_state->is_controller_logging)
    {
        ControllerLoggingData & data = robot_state->controller_logging_data;

        data.tau_f = tauf;
        data.nominal_theta = nominal_plant.theta;
        data.nominal_dtheta = nominal_plant.dtheta;
        data.e_dn = e_task;
        // data.theta_des = theta_d;
    }

    return u;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Controller::init_gravity_compensation()
{
    // do nothing..
    return true;
}

Eigen::VectorXd Controller::gravity_compensation()
{   
    Eigen::VectorXd control_motor_torque = robot_state->get_gravity(robot_state->q);

    // Eigen::VectorXd tau_j(7);
    // tau_j = robot_state->get_joint_nominal_torque(Eigen::VectorXd::Zero(robot_state->nv));
    // std::cout << "initial joint torque: " << tau_j.transpose() << std::endl;
    // std::cout << "current joint torque: " << robot_state->tau_J.transpose() << std::endl;

    return control_motor_torque;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Controller::init_first_order_friction_compensation()
{
    // initialize nominal plant
    nominal_plant.theta.resize(nq); nominal_plant.theta.setZero();
    nominal_plant.theta = robot_state->theta;
    nominal_plant.dtheta.resize(nv); nominal_plant.dtheta.setZero();
    nominal_plant.ddtheta.resize(nv); nominal_plant.ddtheta.setZero();

    return true;
}

Eigen::VectorXd Controller::first_order_friction_compensation(const Eigen::VectorXd &desired_u)
{   
    const FIRST_FRIC_Gains &gain = controller_gain.get_FIRST_FRIC_Gains();

    // nominal plant update
    nominal_plant.ddtheta = (gain.motor_inertia_matrix).inverse() * (desired_u - robot_state->tau_J);
    nominal_plant.dtheta = nominal_plant.dtheta + nominal_plant.ddtheta * dt;
    nominal_plant.theta = robot_state->integrate(nominal_plant.theta, nominal_plant.dtheta, dt);


    // error in nominal & real plant
    Eigen::VectorXd e_NR = robot_state->difference(nominal_plant.theta, robot_state->theta);
    Eigen::VectorXd edot_NR = robot_state->dtheta - nominal_plant.dtheta;

    Eigen::VectorXd est_tauf =  (gain.L) * (gain.motor_inertia_matrix) * (edot_NR); // friction compensation torque

    //for data logging
    if(robot_state->is_controller_logging)
    {
        ControllerLoggingData & data = robot_state->controller_logging_data;

        data.est_tau_f = est_tauf;
        data.e_nr = e_NR;
    }

    return desired_u - est_tauf;
}

bool Controller::init_second_order_friction_compensation()
{
    // initialize nominal plant
    nominal_plant.theta.resize(nq); nominal_plant.theta.setZero();
    nominal_plant.theta = robot_state->theta;
    nominal_plant.dtheta.resize(nv); nominal_plant.dtheta.setZero();
    nominal_plant.ddtheta.resize(nv); nominal_plant.ddtheta.setZero();

    return true;
}

Eigen::VectorXd Controller::second_order_friction_compensation(const Eigen::VectorXd &desired_u)
{
    const SECOND_FRIC_Gains &gain = controller_gain.get_SECOND_FRIC_Gains();
    // nominal plant update
    nominal_plant.ddtheta = (rotor_inertia_matrix_).inverse() * (desired_u - robot_state->tau_J);
    nominal_plant.dtheta = nominal_plant.dtheta + nominal_plant.ddtheta * dt;
    nominal_plant.theta = robot_state->integrate(nominal_plant.theta, nominal_plant.dtheta, dt);

    // error in nominal & real plant
    Eigen::VectorXd e_NR = robot_state->difference(nominal_plant.theta, robot_state->theta);
    Eigen::VectorXd edot_NR = robot_state->dtheta - nominal_plant.dtheta;

    Eigen::VectorXd est_tauf =  (gain.L) * (rotor_inertia_matrix_) * (edot_NR + gain.Lp*e_NR); // friction compensation torque

    //for data logging
    if(robot_state->is_controller_logging)
    {
        ControllerLoggingData & data = robot_state->controller_logging_data;

        data.est_tau_f = est_tauf;
        data.e_nr = e_NR;
    }

    return desired_u - est_tauf;
}

bool Controller::init_third_order_friction_compensation()
{
    // initialize nominal plant
    nominal_plant.theta.resize(nq); nominal_plant.theta.setZero();
    nominal_plant.theta = robot_state->theta;
    nominal_plant.dtheta.resize(nv); nominal_plant.dtheta.setZero();
    nominal_plant.ddtheta.resize(nv); nominal_plant.ddtheta.setZero();
    nominal_plant.integral_e_NR.resize(nv); nominal_plant.integral_e_NR.setZero();

    return true;
} 

Eigen::VectorXd Controller::third_order_friction_compensation(const Eigen::VectorXd &desired_u)
{
    const THIRD_FRIC_Gains &gain = controller_gain.get_THIRD_FRIC_Gains();

    // nominal plant update
    nominal_plant.ddtheta = (gain.motor_inertia_matrix).inverse() * (desired_u - robot_state->tau_J);
    nominal_plant.dtheta = nominal_plant.dtheta + nominal_plant.ddtheta * dt;
    nominal_plant.theta = robot_state->integrate(nominal_plant.theta, nominal_plant.dtheta, dt);

    // error in nominal & real plant
    Eigen::VectorXd e_NR = robot_state->difference(nominal_plant.theta, robot_state->theta);
    Eigen::VectorXd edot_NR = robot_state->dtheta - nominal_plant.dtheta;
    nominal_plant.integral_e_NR += e_NR * dt;

    Eigen::VectorXd est_tauf =  (gain.L) * (gain.motor_inertia_matrix) * (edot_NR + gain.Lp*e_NR + gain.Li*nominal_plant.integral_e_NR); // friction compensation torque

    //for data logging
    if(robot_state->is_controller_logging)
    {
        ControllerLoggingData & data = robot_state->controller_logging_data;

        data.est_tau_f = est_tauf;
        data.e_nr = e_NR;
    }

    return desired_u - est_tauf;
}

bool Controller::init_intertia_reshaping()
{
    // donothing
    return true;
}

Eigen::VectorXd Controller::inertia_reshaping(const Eigen::VectorXd &desired_u)
{
    const INERTIA_RESHAPING_Gains &gain = controller_gain.get_INERTIA_RESHAPING_Gains();

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(robot_state->nv, robot_state->nv);

    Eigen::VectorXd u(nu); u.setZero();

    u = (I - gain.motor_inertia_matrix*gain.desired_motor_inertia_matrix.inverse())
            *robot_state->tau_J + gain.motor_inertia_matrix*gain.desired_motor_inertia_matrix.inverse()*desired_u;

    return u;
}

bool Controller::init_l1_friction_compensation()
{
    // initialize nominal plant
    nominal_plant.theta.resize(nq); nominal_plant.theta.setZero();
    nominal_plant.theta = robot_state->theta;
    nominal_plant.dtheta.resize(nv); nominal_plant.dtheta.setZero();
    nominal_plant.sigma_hat_prev.resize(nv); nominal_plant.sigma_hat_prev.setZero();
    nominal_plant.est_tau_f_prev.resize(nv); nominal_plant.est_tau_f_prev.setZero();

    return true;
}

Eigen::VectorXd Controller::l1_friction_compensation(const Eigen::VectorXd &desired_u)
{
    const L1_FRIC_Gains &gain = controller_gain.get_L1_FRIC_Gains();

    Eigen::VectorXd est_tauf(nv); est_tauf.setZero();
    Eigen::VectorXd sigma_hat(nv); sigma_hat.setZero();

    // nominal plant update => state predictor
    nominal_plant.dtheta = nominal_plant.dtheta + dt*(gain.motor_inertia_matrix.inverse()*(desired_u - robot_state->tau_J - nominal_plant.est_tau_f_prev + nominal_plant.sigma_hat_prev) + gain.As*(nominal_plant.dtheta - robot_state->dtheta));
    nominal_plant.theta = robot_state->integrate(nominal_plant.theta, nominal_plant.dtheta + gain.As*robot_state->difference(robot_state->theta, nominal_plant.theta), dt);

    // error between estimated motor position and measured motor position
    Eigen::VectorXd e_NR = robot_state->difference(nominal_plant.theta, robot_state->theta);
    Eigen::VectorXd edot_NR = robot_state->dtheta - nominal_plant.dtheta;

    // adaptation law
    sigma_hat = (gain.Gamma) * (gain.motor_inertia_matrix) * edot_NR;
    nominal_plant.sigma_hat_prev = sigma_hat;

    for (int i=0; i<nv; i++)
    {
        est_tauf(i) = exp(-gain.W(i,i)*dt)*nominal_plant.est_tau_f_prev(i) + (1-exp(-gain.W(i,i)*dt))*sigma_hat(i);
        nominal_plant.est_tau_f_prev(i) = est_tauf(i);
    }

    //for data logging
    if(robot_state->is_controller_logging)
    {
        ControllerLoggingData & data = robot_state->controller_logging_data;

        data.est_tau_f = est_tauf;
        data.e_nr = e_NR;
    }
    
    return desired_u - est_tauf;
}

bool Controller::init_implicit_l1_friction_compensation()
{
    // initialize nominal plant
    nominal_plant.theta.resize(nq); nominal_plant.theta.setZero();
    nominal_plant.theta = robot_state->theta;
    nominal_plant.dtheta.resize(nv); nominal_plant.dtheta.setZero();
    nominal_plant.sigma_hat_prev.resize(nv); nominal_plant.sigma_hat_prev.setZero();
    nominal_plant.est_tau_f_prev.resize(nv); nominal_plant.est_tau_f_prev.setZero();

    nominal_plant.theta_next.resize(nq); nominal_plant.theta_next.setZero();
    robot_state->theta_next.resize(nq); robot_state->theta_next.setZero();

    return true;
}

Eigen::VectorXd Controller::implicit_l1_friction_compensation(const Eigen::VectorXd &desired_u)
{
    const IMPLICIT_L1_FRIC_Gains &gain = controller_gain.get_IMPLICIT_L1_FRIC_Gains();

    Eigen::VectorXd est_tauf(nv); est_tauf.setZero();
    Eigen::VectorXd sigma_hat(nv); sigma_hat.setZero();

    // nominal plant update => state predictor
    Eigen::VectorXd n_dtheta(nv), n_theta(nq);
    n_dtheta.setZero(); n_theta.setZero();
    n_dtheta = nominal_plant.dtheta + dt*(gain.motor_inertia_matrix.inverse()*(desired_u - robot_state->tau_J - nominal_plant.est_tau_f_prev + nominal_plant.sigma_hat_prev));
    n_theta = robot_state->integrate(nominal_plant.theta, nominal_plant.dtheta, dt);

    nominal_plant.dtheta = n_dtheta;
    nominal_plant.theta = n_theta;

    // implicit Euler method for PD adaptation law
    const Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(nv, nv);
    const Eigen::MatrixXd filter_decay = (-gain.W * dt).exp();
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2 * nv, 2 * nv);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(2 * nv);
    A.block(0, 0, nv, nv) = gain.motor_inertia_matrix * (identity + filter_decay * gain.Gamma * dt);
    A.block(nv, 0, nv, nv) = -gain.motor_inertia_matrix * (identity - filter_decay) * gain.Gamma * dt;
    A.block(0, nv, nv, nv) = -gain.motor_inertia_matrix * filter_decay * gain.Gamma * dt;
    A.block(nv, nv, nv, nv) = gain.motor_inertia_matrix * (identity + (identity - filter_decay) * gain.Gamma * dt);
    b.segment(0, nv) = desired_u - robot_state->tau_J - filter_decay * nominal_plant.est_tau_f_prev - filter_decay * gain.motor_inertia_matrix * gain.Gamma
                     * (n_dtheta - robot_state->dtheta + gain.Gamma_p*(robot_state->difference(robot_state->theta, n_theta) + dt*n_dtheta - dt*robot_state->dtheta));
    b.segment(nv, nv) = desired_u - robot_state->tau_J - filter_decay * nominal_plant.est_tau_f_prev + (identity - filter_decay) * gain.motor_inertia_matrix * gain.Gamma
                     * (n_dtheta - robot_state->dtheta + gain.Gamma_p*(robot_state->difference(robot_state->theta, n_theta) + dt*n_dtheta - dt*robot_state->dtheta));
    
    Eigen::VectorXd n_ddtheta(nv), ddtheta(nv); n_ddtheta.setZero(); ddtheta.setZero();
    const Eigen::VectorXd accelerations = A.partialPivLu().solve(b);
    n_ddtheta = accelerations.head(nv);
    ddtheta = accelerations.tail(nv);

    Eigen::VectorXd n_dtheta_next(nv), dtheta_next(nv); n_dtheta_next.setZero(); dtheta_next.setZero();
    n_dtheta_next = n_dtheta + dt*n_ddtheta;
    dtheta_next = robot_state->dtheta + dt*ddtheta;
    nominal_plant.theta_next = robot_state->integrate(n_theta, n_dtheta, dt);
    robot_state->theta_next = robot_state->integrate(robot_state->theta, robot_state->dtheta, dt);

    Eigen::VectorXd e_NR(nv), edot_NR(nv);
    e_NR.setZero(); edot_NR.setZero();
    e_NR = robot_state->difference(nominal_plant.theta_next, robot_state->theta_next);
    edot_NR = dtheta_next - n_dtheta_next;

    // adaptation law
    sigma_hat = (gain.Gamma) * (gain.motor_inertia_matrix) * (edot_NR + (gain.Gamma_p)*e_NR);
    nominal_plant.sigma_hat_prev = sigma_hat;

    for (int i=0; i<nv; i++)
    {
        est_tauf(i) = exp(-gain.W(i,i)*dt)*nominal_plant.est_tau_f_prev(i) + (1-exp(-gain.W(i,i)*dt))*sigma_hat(i);
        nominal_plant.est_tau_f_prev(i) = est_tauf(i);
    }
    //for data logging
    if(robot_state->is_controller_logging)
    {
        ControllerLoggingData & data = robot_state->controller_logging_data;

        data.est_tau_f = est_tauf;
        data.e_nr = e_NR;
    }
    
    return desired_u - est_tauf;
}

bool Controller::init_coulomb_observer_friction_compensation()
{
    coulomb_observer_state.dtheta_hat = Eigen::VectorXd::Zero(nv);
    coulomb_observer_state.a_c_hat = Eigen::VectorXd::Zero(nv);
    coulomb_observer_state.f_hat = Eigen::VectorXd::Zero(nv);
    coulomb_observer_state.u_prev = Eigen::VectorXd::Zero(nv);

    return true;
}

Eigen::VectorXd Controller::coulomb_observer_friction_compensation(const Eigen::VectorXd &desired_u)
{
    const COULOMB_OBSERVER_Gains &gain = controller_gain.get_COULOMB_OBSERVER_Gains();

    Eigen::VectorXd dtheta_hat = coulomb_observer_state.dtheta_hat;
    Eigen::VectorXd a_c_hat = coulomb_observer_state.a_c_hat;
    const Eigen::VectorXd &u_prev = coulomb_observer_state.u_prev;

    const Eigen::VectorXd e_w = robot_state->dtheta - dtheta_hat;
    const Eigen::VectorXd da_c_hat =
        -gain.L * e_w.cwiseProduct(robot_state->dtheta.cwiseSign());
    const Eigen::VectorXd ddtheta_hat =
        -a_c_hat.cwiseProduct(robot_state->dtheta.cwiseSign())
        + gain.K * e_w + u_prev - robot_state->tau_J;

    a_c_hat += da_c_hat * dt;
    dtheta_hat += ddtheta_hat * dt;

    const Eigen::VectorXd tau_f_hat = gain.motor_inertia_matrix
        * a_c_hat.cwiseProduct(robot_state->dtheta.cwiseSign());

    coulomb_observer_state.u_prev = desired_u + tau_f_hat;
    coulomb_observer_state.dtheta_hat = dtheta_hat;
    coulomb_observer_state.a_c_hat = a_c_hat;
    coulomb_observer_state.f_hat = tau_f_hat;

    if(robot_state->is_controller_logging)
    {
        robot_state->controller_logging_data.est_tau_f = tau_f_hat;
    }

    return desired_u + tau_f_hat;
}

bool Controller::init_asym_friction_compensation()
{
    const ASYM_FRICTION_Gains &gain = controller_gain.get_ASYM_FRICTION_Gains();
    const std::array<const Eigen::VectorXd*, 31> parameters = {{
        &gain.f_v1_p, &gain.f_v1_n, &gain.f_v2_p, &gain.f_v2_n,
        &gain.f_v3_p, &gain.f_v3_n, &gain.f_s_p, &gain.f_s_n,
        &gain.f_c_p, &gain.f_c_n, &gain.qdot_s_p, &gain.qdot_s_n,
        &gain.qdot_c_p, &gain.qdot_c_n, &gain.delta_s_p, &gain.delta_s_n,
        &gain.delta_c_p, &gain.delta_c_n, &gain.f_l1_p, &gain.f_l1_n,
        &gain.f_l2_p, &gain.f_l2_n, &gain.f_l3_p, &gain.f_l3_n,
        &gain.f_l4_p, &gain.f_l4_n, &gain.f_T1, &gain.f_T2,
        &gain.f_T3, &gain.f_T4, &gain.mu
    }};

    for (const Eigen::VectorXd* parameter : parameters)
    {
        if (parameter->size() != nv || !parameter->allFinite())
        {
            std::cerr << "ASYM_FRICTION requires finite, offline-identified "
                      << nv << "-element vectors for every parameter." << std::endl;
            return false;
        }
    }

    return true;
}

Eigen::VectorXd Controller::asym_friction_compensation(const Eigen::VectorXd &desired_u)
{
    const ASYM_FRICTION_Gains &gain = controller_gain.get_ASYM_FRICTION_Gains();
    Eigen::VectorXd tau_f_hat = Eigen::VectorXd::Zero(nv);

    const Eigen::VectorXd &dtheta = robot_state->dtheta;
    const Eigen::VectorXd &temperatures = robot_state->joint_temperatures;
    const Eigen::VectorXd &tau_j = robot_state->tau_J;
    constexpr double minimum_scale = 1e-6;

    for (int i = 0; i < nv; ++i)
    {
        const double velocity = dtheta(i);
        const double absolute_velocity = std::abs(velocity);
        const double direction = velocity > 0.0 ? 1.0 : (velocity < 0.0 ? -1.0 : 0.0);

        double velocity_friction = 0.0;
        if (velocity > 0.0)
        {
            const double qdot_s = std::max(gain.qdot_s_p(i), minimum_scale);
            const double qdot_c = std::max(gain.qdot_c_p(i), minimum_scale);
            const double delta_s = std::clamp(gain.delta_s_p(i), 1.0, 2.0);
            const double delta_c = std::clamp(gain.delta_c_p(i), 1.0, 2.0);

            velocity_friction = gain.f_v1_p(i) * velocity
                + gain.f_v2_p(i) * direction * absolute_velocity * absolute_velocity
                + gain.f_v3_p(i) * velocity * velocity * velocity
                + gain.f_s_p(i) * direction
                    * std::exp(-std::pow(absolute_velocity / qdot_s, delta_s))
                + gain.f_c_p(i) * direction
                    * (1.0 - std::exp(-std::pow(absolute_velocity / qdot_c, delta_c)));
        }
        else if (velocity < 0.0)
        {
            const double qdot_s = std::max(gain.qdot_s_n(i), minimum_scale);
            const double qdot_c = std::max(gain.qdot_c_n(i), minimum_scale);
            const double delta_s = std::clamp(gain.delta_s_n(i), 1.0, 2.0);
            const double delta_c = std::clamp(gain.delta_c_n(i), 1.0, 2.0);

            velocity_friction = gain.f_v1_n(i) * velocity
                + gain.f_v2_n(i) * direction * absolute_velocity * absolute_velocity
                + gain.f_v3_n(i) * velocity * velocity * velocity
                + gain.f_s_n(i) * direction
                    * std::exp(-std::pow(absolute_velocity / qdot_s, delta_s))
                + gain.f_c_n(i) * direction
                    * (1.0 - std::exp(-std::pow(absolute_velocity / qdot_c, delta_c)));
        }

        double load_friction = 0.0;
        const double mechanical_power = tau_j(i) * velocity;
        if (mechanical_power > 0.0)
        {
            const double load_scale = std::max(gain.f_l3_p(i), minimum_scale);
            const double load_gain = gain.f_l1_p(i)
                + gain.f_l2_p(i) * std::exp(-std::abs(velocity / load_scale));
            load_friction = load_gain
                * (tau_j(i) * tau_j(i) + gain.f_l4_p(i) * std::abs(tau_j(i)))
                * direction;
        }
        else if (mechanical_power < 0.0)
        {
            const double load_scale = std::max(gain.f_l3_n(i), minimum_scale);
            const double load_gain = gain.f_l1_n(i)
                + gain.f_l2_n(i) * std::exp(-std::abs(velocity / load_scale));
            load_friction = load_gain
                * (tau_j(i) * tau_j(i) + gain.f_l4_n(i) * std::abs(tau_j(i)))
                * direction;
        }

        const double temperature = temperatures(i);
        const double temperature_polynomial = gain.f_T1(i) * temperature
            + gain.f_T2(i) * temperature * temperature
            + gain.f_T3(i) * temperature * temperature * temperature
            + gain.f_T4(i);
        const double velocity_exponent = std::max(0.0, gain.mu(i));
        const double velocity_power = absolute_velocity > 0.0
            ? std::pow(absolute_velocity, velocity_exponent)
            : (velocity_exponent == 0.0 ? 1.0 : 0.0);
        const double temperature_friction =
            direction * velocity_power * temperature_polynomial;

        tau_f_hat(i) = velocity_friction + load_friction + temperature_friction;
    }

    if (robot_state->is_controller_logging)
    {
        robot_state->controller_logging_data.est_tau_f = tau_f_hat;
    }

    return desired_u - tau_f_hat;
}

bool Controller::init_static_friction_compensation()
{
    const STATIC_FRICTION_Gains &gain = controller_gain.get_STATIC_FRICTION_Gains();
    const std::array<const Eigen::MatrixXd*, 4> parameters = {{
        &gain.Fc, &gain.Fv1, &gain.Fv2, &gain.friction_compensation_weights
    }};

    for (const Eigen::MatrixXd* parameter : parameters)
    {
        if (parameter->rows() != nv || parameter->cols() != nv || !parameter->allFinite())
        {
            std::cerr << "STATIC_FRICTION requires finite, offline-identified or tuned "
                      << nv << "-element vectors for every parameter." << std::endl;
            return false;
        }
    }

    if ((gain.Fv2.diagonal().array() <= 0.0).any())
    {
        std::cerr << "STATIC_FRICTION requires strictly positive Fv2 velocity scales."
                  << std::endl;
        return false;
    }

    return true;
}

Eigen::VectorXd Controller::static_friction_compensation(const Eigen::VectorXd &desired_u)
{
    const STATIC_FRICTION_Gains &gain = controller_gain.get_STATIC_FRICTION_Gains();
    constexpr double minimum_scale = 1e-6;

    const Eigen::ArrayXd velocity = robot_state->dtheta.array();
    const Eigen::ArrayXd direction = robot_state->dtheta.cwiseSign().array();
    const Eigen::ArrayXd velocity_scale =
        gain.Fv2.diagonal().array().abs().max(minimum_scale);
    const Eigen::ArrayXd saturation =
        1.0 - (-velocity.abs() / velocity_scale).exp();
    const Eigen::VectorXd tau_f_hat =
        (gain.Fc.diagonal().array() * direction
        + gain.Fv1.diagonal().array() * direction * saturation).matrix();

    if (robot_state->is_controller_logging)
    {
        robot_state->controller_logging_data.est_tau_f = tau_f_hat;
    }

    return desired_u
        + gain.friction_compensation_weights.diagonal().cwiseProduct(tau_f_hat);
}
