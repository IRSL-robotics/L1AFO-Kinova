#include "kinova_controller/robot/kinova_gen3.hpp"

KinovaGen3::KinovaGen3():
KinovaGen3Base()
{

}

KinovaGen3::~KinovaGen3()
{
    this->destroy();
}

bool KinovaGen3::init(std::shared_ptr<RobotState> robot_state_arg)
{
    robot_state = robot_state_arg;
    sampling_time = robot_state->dt;

    // configuration
    try
    {
        gen3_ip = robot_state->get_config()["gen3_ip"].as<std::string>();
        lpf_dq_cutoff_freq = robot_state->get_config()["lpf_dq_cutoff_freq"].as<double>();
        lpf_tau_j_cutoff_freq = robot_state->get_config()["lpf_tau_j_cutoff_freq"].as<double>();
    }
    catch(YAML::Exception &e)
    {   
        std::cout<<"ERROR: gen3_ip, lpf_dq_cutoff_freq, lpf_tau_j_cutoff_freq are poorly defined in the yaml file"<<std::endl;
        std::cout<<"YAML Exception : "<<e.what()<<std::endl;
    }

    // Kortex API
    auto error_callback = [](Kinova::Api::KError err){ cout << "_________ callback error _________" << err.toString(); };

    // std::cout << "Creating transport objects" << std::endl;
    transport = new Kinova::Api::TransportClientTcp();
    router = new Kinova::Api::RouterClient(transport, error_callback);
    transport->connect(gen3_ip, PORT);

    // std::cout << "Creating transport real time objects" << std::endl;
    transport_real_time = new Kinova::Api::TransportClientUdp();
    router_real_time = new Kinova::Api::RouterClient(transport_real_time, error_callback);
    transport_real_time->connect(gen3_ip, PORT_REAL_TIME);

    // Set session data connection information
    auto create_session_info = Kinova::Api::Session::CreateSessionInfo();
    create_session_info.set_username("admin");
    create_session_info.set_password("admin");
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // Session manager service wrapper
    // std::cout << "Creating sessions for communication" << std::endl;
    session_manager = new Kinova::Api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    session_manager_real_time = new Kinova::Api::SessionManager(router_real_time);
    session_manager_real_time->CreateSession(create_session_info);
    // std::cout << "Sessions created" << std::endl;

    //Create Services
    base = new Kinova::Api::Base::BaseClient(router);
    base_cyclic = new Kinova::Api::BaseCyclic::BaseCyclicClient(router_real_time);
    actuator_config = new Kinova::Api::ActuatorConfig::ActuatorConfigClient(router);
    is_initialized_ = true;

    //Initialize Kinova Gen3
    bool success = true;
    success &= move2home_position();
    success &= set_current_mode();

    for(unsigned int i=0; i<7; i++)
    {
        gen3_state.joint_positions(i) = (double)base_feedback.actuators(i).position()* M_PI/180; // motor position
        gen3_state.joint_velocities(i) = (double)base_feedback.actuators(i).velocity() * M_PI/180; // motor velocity
        gen3_state.joint_torques(i) = -(double)base_feedback.actuators(i).torque(); // joint torque sensor
        gen3_state.joint_temperatures(i) = (double)base_feedback.actuators(i).temperature_motor(); // motor temperature
    }

    lpf_dq.init(0.001, lpf_dq_cutoff_freq, Eigen::VectorXd::Zero(7));
    lpf_tau_j.init(0.001, lpf_tau_j_cutoff_freq, gen3_state.joint_torques);

    return success;
}

void KinovaGen3::destroy()
{
    if (!is_initialized_)
    {
        return;
    }

    //Set position control mode
    auto control_mode_message = Kinova::Api::ActuatorConfig::ControlModeInformation();
    control_mode_message.set_control_mode(Kinova::Api::ActuatorConfig::ControlMode::POSITION);
    for(int i=0; i<7; i++)
    {
        actuator_config->SetControlMode(control_mode_message, i+1);    
    }

    std::cout<<"Delete Kinova Controller"<<std::endl;

    // Close API session
    session_manager->CloseSession();
    session_manager_real_time->CloseSession();

    // Deactivate the router and cleanly disconnect from the transport object
    router->SetActivationStatus(false);
    transport->disconnect();
    router_real_time->SetActivationStatus(false);
    transport_real_time->disconnect();

    // Destroy the API
    delete base;
    delete base_cyclic;
    delete actuator_config;
    delete session_manager;
    delete session_manager_real_time;
    delete router;
    delete router_real_time;
    delete transport;
    delete transport_real_time;

    is_initialized_ = false;
}

bool KinovaGen3::move2home_position()
{   
    constexpr auto TIMEOUT_PROMISE_DURATION = std::chrono::seconds{20};

    // Make sure the arm is in Single Level Servoing before executing an Action
    auto servoingMode = Kinova::Api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(Kinova::Api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Move arm to ready position
    std::cout<<"Moving the arm to a safe position"<<std::endl;
    auto action_type = Kinova::Api::Base::RequestedActionType();
    action_type.set_action_type(Kinova::Api::Base::REACH_JOINT_ANGLES);
    auto action_list = base->ReadAllActions(action_type);
    auto action_handle = Kinova::Api::Base::ActionHandle();
    action_handle.set_identifier(0);
    for (auto action : action_list.action_list())
    {
        if (action.name() == "New initial pose")
        {
            action_handle = action.handle();
        }
    }

    if (action_handle.identifier() == 0)
    {   
        std::cout<<"Can't reach safe position, exiting"<<std::endl;
        return false;
    }
    else
    {
        // Connect to notification action topic
        std::promise<Kinova::Api::Base::ActionEvent> finish_promise;
        auto finish_future = finish_promise.get_future();
        auto promise_notification_handle = base->OnNotificationActionTopic(
                create_event_listener_by_promise(finish_promise),
                Kinova::Api::Common::NotificationOptions()
        );

        // Execute action
        base->ExecuteActionFromReference(action_handle);

        // Wait for future value from promise
        const auto status = finish_future.wait_for(TIMEOUT_PROMISE_DURATION);
        base->Unsubscribe(promise_notification_handle);

        if(status != std::future_status::ready)
        {
            std::cout<<"Timeout on action notification wait"<<std::endl;
            return false;
        }
        const auto promise_event = finish_future.get();

        std::cout<<"Move to Home completed"<<std::endl;

        // Wait for a bit
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        return true;
    }
}

std::function<void(Kinova::Api::Base::ActionNotification)>
KinovaGen3::create_event_listener_by_promise(std::promise<Kinova::Api::Base::ActionEvent>& finish_promise)
{
    return [&finish_promise] (Kinova::Api::Base::ActionNotification notification)
    {
        const auto action_event = notification.action_event();
        switch(action_event)
        {
            case Kinova::Api::Base::ActionEvent::ACTION_END:
            case Kinova::Api::Base::ActionEvent::ACTION_ABORT:
                finish_promise.set_value(action_event);
                break;
            default:
                break;
        }
    };
}

bool KinovaGen3::set_current_mode()
{   
    bool return_status = true;
    // Clearing faults
    try
    {
        base->ClearFaults();
    }
    catch(...)
    {   
        std::cout<<"Unable to clear robot faults"<<std::endl;
        return false;
    }


    std::vector<float> commands;
    Eigen::VectorXd tau_J(7);

    auto servoing_mode = Kinova::Api::Base::ServoingModeInformation();

    // std::cout << "Setting robot control mode as current control" << std::endl;
    try
    {
        // Set the base in low-level servoing mode
        servoing_mode.set_servoing_mode(Kinova::Api::Base::ServoingMode::LOW_LEVEL_SERVOING);
        base->SetServoingMode(servoing_mode);
        base_feedback = base_cyclic->RefreshFeedback();

        // Initialize each actuator to their current position
        for (int i = 0; i < 7; i++)
        {
            commands.push_back(base_feedback.actuators(i).position());
            // Save the current actuator position, to avoid a following error
            base_command.add_actuators()->set_position(base_feedback.actuators(i).position());
            
        }

        // Send a first frame
        base_feedback = base_cyclic->Refresh(base_command);

        // Set all actuators in current control mode.
        auto control_mode_message = Kinova::Api::ActuatorConfig::ControlModeInformation();
        control_mode_message.set_control_mode(Kinova::Api::ActuatorConfig::ControlMode::CURRENT);
        for (int i = 0; i < 7; i++)
        {
            actuator_config->SetControlMode(control_mode_message, i+1);
        }

    }
    catch (Kinova::Api::KDetailedException& ex)
    {   
        std::cout << "API error: " << ex.what() << std::endl;
        return_status = false;
    }
    catch (std::runtime_error& ex2)
    {   
        std::cout << "Error: " << ex2.what() << std::endl;
        return_status = false;
    }

    return return_status;
}

void KinovaGen3::update_state()
{   
    for(unsigned int i=0; i<7; i++)
    {
        gen3_state.joint_positions(i) = (double)base_feedback.actuators(i).position()* M_PI/180; // motor position
        gen3_state.joint_velocities(i) = (double)base_feedback.actuators(i).velocity() * M_PI/180; // motor velocity
        gen3_state.joint_torques(i) = -(double)base_feedback.actuators(i).torque(); // joint torque sensor
    }

    // for initialize joint torque sensor
    if(robot_state->init_tau_j)
    {   
        const double alpha = (robot_state->count -1) / robot_state->count;
        Eigen::VectorXd nominal_tau_j(robot_state->nv);
        nominal_tau_j = robot_state->get_joint_nominal_torque(Eigen::VectorXd::Zero(robot_state->nv));
        robot_state->bias.resize(robot_state->nv);
        robot_state->bias = alpha * robot_state->bias + (1-alpha) * (gen3_state.joint_torques - nominal_tau_j);
        robot_state->count ++;
        if(robot_state->count > robot_state->avg_filter_count)
        {
            robot_state->init_tau_j = false;
            robot_state->count = 1;
            std::cout<<"Joint torque sensor bias is initialized"<<std::endl;
            std::cout<<"bias : " << robot_state->bias.transpose()<<std::endl;
        }
    }

    // low pass filter
    // gen3_state.joint_velocities = lpf_dq.get_filtered_value(gen3_state.joint_velocities);
    gen3_state.joint_velocities = gen3_state.joint_velocities;
    gen3_state.joint_torques = lpf_tau_j.get_filtered_value(gen3_state.joint_torques-robot_state->bias);

    // solve joint space jump problem
    convert_joint_position(gen3_state.joint_positions);

    // continuous joint position
    robot_state->convert_q(gen3_state);

    // update robot state
    robot_state->update_robot_state(gen3_state);
}


bool KinovaGen3::set_joint_torque(const Eigen::VectorXd &u)
{
    //Convert Eigen::Vector to std::vector<float>
    std::array<float, 7> u_vec;
    for(int i=0; i<7; i++)
    {
        u_vec[i] = (float)u(i);
    }

    //Convert Torque (N/m) to current (A)
    // motor coefficient 1~4 : 11 Nm/A, 5~7 : 7.6 Nm/A (Consider gear ratio...!)
    for(int i=0; i<4; i++)
    {
        u_vec[i] = u_vec[i]/11;
    }
    for(int i=4; i<7; i++)
    {
        u_vec[i] = u_vec[i]/7.6;
    }
    
    //Apply control input to the kinova gen3
    bool return_status = true;
    
    //Input torque.
    for(int i=0; i<7; i++)
    {   
        //To prevent following error.. set position as current position..!
        base_command.mutable_actuators(i)->set_position(base_feedback.actuators(i).position());
        //Set actuator current.
        base_command.mutable_actuators(i)->set_current_motor(u_vec[i]);
    }


    // Incrementing identifier ensures actuators can reject out of time frames
    base_command.set_frame_id(base_command.frame_id() + 1);
    if (base_command.frame_id() > 65535)
        base_command.set_frame_id(0);

    for (int i = 0; i < 7; i++)
    {
        base_command.mutable_actuators(i)->set_command_id(base_command.frame_id());
    }

    try
    {
        base_feedback = base_cyclic->Refresh(base_command, 0);
    }
    catch (Kinova::Api::KDetailedException& ex)
    {
        std::cout << "Kortex exception: " << ex.what() << std::endl;

        std::cout << "Error sub-code: " << Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))) << std::endl;
        return_status = false;
    }
    catch (std::runtime_error& ex2)
    {
        std::cout << "runtime error: " << ex2.what() << std::endl;
        return_status = false;
    }
    catch(...)
    {
        std::cout << "Unknown error." << std::endl;
        return_status = false;
    }

    return return_status;
}

// Joiint information //
// Joint 1: continuous       -limit in rad,   -limit in deg,
// Joint 2: revolute, limits: [-2.41, 2.41]    [-138, 138]
// Joint 3: continuous
// Joint 4: revolute, limits: [-2.66, 2.66]    [-152, 152]
// Joint 5: continuous
// Joint 6: revolute, limits: [-2.33, 2.33]    [-134, 134]
// Joint 7: continuous

//Convert law (feedback in degree) (output in degree) (feedback in degree) (output in degree)
//Joint 2:        [222~359.99]   ->    [-138 ~ -0.01]    [0.01 ~ 138]    ->   [0.01 ~ 138]
//Joint 4:        [208~359.99]   ->    [-152 ~ -0.01]    [0.01 ~ 152]    ->   [0.01 ~ 152]
//Joint 6:        [226~359.99]   ->    [-134 ~ -0.01]    [0.01 ~ 134]    ->   [0.01 ~ 134]
//                             [-360]                                  [nothing]

void KinovaGen3::convert_joint_position(Eigen::Matrix<double, 7, 1>& joint_positions)
{
    for(unsigned int i=0; i<7; i++)
    {
        if(is_limit[i]>0 && joint_positions(i) > M_PI)
        {
            joint_positions(i) -= 2*M_PI;
        }
    }
}
