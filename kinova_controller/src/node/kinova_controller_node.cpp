#include "node/kinova_controller_node.hpp"

KinovaControllerNode::KinovaControllerNode(const rclcpp::NodeOptions & options)
: KinovaControllerNode("kinova_controller",  options)
{}

KinovaControllerNode::KinovaControllerNode(const std::string & node_name, const rclcpp::NodeOptions & options):
LifecycleNode(node_name, options),
state_topic_name(declare_parameter<std::string>("state_topic_name", "extended_robot_state")),
main_control_loop_period(std::chrono::microseconds{declare_parameter<std::uint16_t>("state_publish_period_us", 1000U)}),
controller_config_topic_name(declare_parameter<std::string>("controller_config_topic_name", "controller_config")),
desired_joint_value_topic_name(declare_parameter<std::string>("desired_joint_value_topic_name", "desired_joint_value")),
desired_task_value_topic_name(declare_parameter<std::string>("desired_task_value_topic_name", "desired_task_value")),
update_gain_topic_name(declare_parameter<std::string>("update_gain_topic_name", "update_gain")),
controller_logging_topic_name(declare_parameter<std::string>("controller_logging_topic_name", "controller_logging")),
num_missed_deadlines_pub{0U},
deadline_duration{std::chrono::milliseconds{declare_parameter<std::uint16_t>("deadline_duration_ms", 0U)}}
{
    RCLCPP_INFO(get_logger(), "KinovaControllerNode constructor called");
    create_main_control_loop_callback();
    create_state_publisher();
    create_controller_config_subscriber();
    create_desired_joint_value_subscriber();
    create_desired_task_value_subscriber();
    create_update_gain_subscriber();
}

KinovaControllerNode::~KinovaControllerNode()
{
    RCLCPP_INFO(get_logger(), "KinovaControllerNode destructor called");
}

void KinovaControllerNode::main_control_loop()
{    
    // update robot_state from kinova gen3
    kinova_gen3->update_state();

    // Get control input
    Eigen::VectorXd u = controller.get_control_input(controller_selector);

    // Error handling
    if(u.size() != robot_state->nu)
    {
        RCLCPP_WARN(get_logger(), "Control input size is not matched with robot state");
        RCLCPP_WARN(get_logger(), "Changed to gravity compensation mode");
        controller_selector = controller.select("GRAVITY_COMPENSATION");
        u = controller.get_control_input(controller_selector);
    }

    if(u.hasNaN())
    {
        RCLCPP_WARN(get_logger(), "Control input has NaN");
        RCLCPP_WARN(get_logger(), "Changed to gravity compensation mode");
        controller_selector = controller.select("GRAVITY_COMPENSATION");
        u = controller.get_control_input(controller_selector);
    }

    

    // Set control input
    kinova_gen3->set_joint_torque(u);
    // std::cout<<"main loop u: "<<u.transpose()<<std::endl;

    // fill state_message
    robot_state_msgs::msg::ExtendedState robot_state_msg;

    for (int i=0;i<robot_state->nq;i++)
    {
        robot_state_msg.q.push_back(robot_state->q(i));        
    }

    for(int i=0; i<robot_state->nv; i++)
    {   
        robot_state_msg.dq.push_back(robot_state->dq(i));
        robot_state_msg.tau_j.push_back(robot_state->tau_J(i));
        // robot_state_msg.effort.push_back(u(i));
    }

    // end-effector pose
    auto ee_htm = robot_state->get_ee_pose("end_effector");
    for(int i=0; i<3; i++)
    {
        robot_state_msg.ee_pos[i] = (ee_htm.translation()(i));
    }
    // for(int i=0; i<9; i++)
    // {
    //     robot_state_msg.ee_rot[i] = (ee_htm.rotation()(i));
    // }

    // Publish state
    state_pub->publish(robot_state_msg);

    // publish controller logging
    if(robot_state->is_controller_logging)
    {
        publish_controller_logging();
    
    }
}

void KinovaControllerNode::create_real_robot()
{   
    kinova_gen3 = std::make_shared<KinovaGen3>();
}

void KinovaControllerNode::create_simulation_robot(mj::Simulate* sim)
{
    kinova_gen3 = std::make_shared<KinovaGen3Simulation>(sim);
}

void KinovaControllerNode::create_state_publisher()
{
    rclcpp::PublisherOptions sensor_publisher_options;
    sensor_publisher_options.event_callbacks.deadline_callback =
        [this](rclcpp::QOSDeadlineOfferedInfo &) -> void
        {
            num_missed_deadlines_pub++;
        };
    state_pub = this->create_publisher<robot_state_msgs::msg::ExtendedState>(
        state_topic_name,
        rclcpp::QoS(10).deadline(deadline_duration),
        sensor_publisher_options
    );
}

void KinovaControllerNode::create_main_control_loop_callback()
{   
    // Callback function for state timer
    auto main_loop_callback = std::bind(&KinovaControllerNode::main_control_loop, this);
    main_control_loop_timer = this->create_wall_timer(main_control_loop_period, main_loop_callback);
    
    // cancel immediately to prevent triggering it in this state
    main_control_loop_timer->cancel();
}

void KinovaControllerNode::create_controller_config_subscriber()
{   
    auto callback = [this](const controller_interface_msgs::msg::ControllerConfig::SharedPtr msg) -> void
    {
        RCLCPP_INFO(get_logger(), "Received controller config message");
        std::string controller_type = msg->controller_selector;
        controller_selector = controller.select(controller_type);
    };

    controller_config_sub = this->create_subscription<controller_interface_msgs::msg::ControllerConfig>(
        controller_config_topic_name,
        rclcpp::QoS(10),
        callback
    );
}

void KinovaControllerNode::create_desired_joint_value_subscriber()
{
    auto callback = [this](const controller_interface_msgs::msg::SetDesiredJointValue::SharedPtr msg) -> void
    {
        if (msg->q_d.size() != static_cast<std::size_t>(robot_state->nq) ||
            msg->dq_d.size() != static_cast<std::size_t>(robot_state->nv) ||
            msg->ddq_d.size() != static_cast<std::size_t>(robot_state->nv))
        {
            RCLCPP_WARN(
                get_logger(),
                "Ignoring joint reference: expected q/dq/ddq sizes %d/%d/%d, got %zu/%zu/%zu",
                robot_state->nq, robot_state->nv, robot_state->nv,
                msg->q_d.size(), msg->dq_d.size(), msg->ddq_d.size());
            return;
        }

        Eigen::VectorXd q_d_temp = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(msg->q_d.data(), msg->q_d.size());        
        robot_state->convert_q(q_d_temp);
        robot_state->q_d = q_d_temp;
        robot_state->dq_d = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(msg->dq_d.data(), msg->dq_d.size());
        robot_state->ddq_d = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(msg->ddq_d.data(), msg->ddq_d.size());
    };

    desired_joint_value_sub = this->create_subscription<controller_interface_msgs::msg::SetDesiredJointValue>(
        desired_joint_value_topic_name,
        rclcpp::QoS(10),
        callback
    );

}

void KinovaControllerNode::create_desired_task_value_subscriber()
{
    auto callback = [this](const controller_interface_msgs::msg::SetDesiredTaskValue::SharedPtr msg) -> void
    {
        if (!msg->quat_d.empty() && msg->quat_d.size() != 4U)
        {
            RCLCPP_WARN(
                get_logger(),
                "Ignoring task reference: quat_d must be empty or contain four values (w, x, y, z)");
            return;
        }

        Eigen::VectorXd quat = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(msg->quat_d.data(), msg->quat_d.size());
        Eigen::Matrix3d rot;

        if(quat.size()!= 0)
        {
            rot = Eigen::Quaterniond(quat(0), quat(1), quat(2), quat(3)).toRotationMatrix(); // w, x, y, z
        }
        else
        {
            rot = Eigen::Map<Eigen::Matrix3d, Eigen::Unaligned>(msg->rot_d.data(), 3, 3);
        }
        
        Eigen::Vector3d pos = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(msg->pos_d.data(), msg->pos_d.size());

        Eigen::Matrix4d htm_d_mat; htm_d_mat.setIdentity();
        htm_d_mat.block(0,0,3,3) = rot;
        htm_d_mat.block(0,3,3,1) = pos;

        pinocchio::SE3 htm_d(htm_d_mat);

        robot_state->htm_d = htm_d;
    };

    desired_task_value_sub = this->create_subscription<controller_interface_msgs::msg::SetDesiredTaskValue>(
        desired_task_value_topic_name,
        rclcpp::QoS(10),
        callback
    );
}

void KinovaControllerNode::create_controller_logging_publisher()
{   
    controller_logging_pub = this->create_publisher<robot_state_msgs::msg::ControllerLogging>(
        controller_logging_topic_name,
        rclcpp::QoS(10)
    );

    controller_logging_pub->on_activate();
}
void KinovaControllerNode::publish_controller_logging()
{
    const ControllerLoggingData & controller_logging_data = robot_state->controller_logging_data;

    robot_state_msgs::msg::ControllerLogging msg;

    msg.time = this->now();
    const auto copy_vector = [](const Eigen::VectorXd & source, auto & destination) {
        destination.assign(source.data(), source.data() + source.size());
    };
    copy_vector(controller_logging_data.nominal_theta, msg.nominal_theta);
    copy_vector(controller_logging_data.nominal_dtheta, msg.nominal_dtheta);
    copy_vector(controller_logging_data.theta_des, msg.theta_des);
    copy_vector(controller_logging_data.e_dn, msg.e_dn);
    copy_vector(controller_logging_data.e_nr, msg.e_nr);
    copy_vector(controller_logging_data.tau_f, msg.tau_f);
    copy_vector(controller_logging_data.est_tau_f, msg.est_tau_f);

    controller_logging_pub->publish(msg);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
KinovaControllerNode::on_configure(const rclcpp_lifecycle::State &)
{   
    RCLCPP_INFO(get_logger(), "Configuring");

    // create controller logging publisher
    if(robot_state->is_controller_logging)
    {
        create_controller_logging_publisher();
    }

    // Initialize kinova gen3 & set current control mode
    if(!kinova_gen3->init(robot_state))
    {   
        RCLCPP_ERROR(get_logger(), "Failed to initialize kinova gen3");
        return LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    else
    {
        RCLCPP_INFO(get_logger(), "Robot is initialized");
    }
    
    controller.assign_robot_state(robot_state);
    if(!robot_state->is_simulation)
    {
        // additional initialize process (3s)
        RCLCPP_INFO(get_logger(), "Get initial state...");
        rclcpp::Rate loop_rate(1000); unsigned int loop_counter = 0;
        while(rclcpp::ok() && loop_counter < 1500)
        {   
            // Get state
            kinova_gen3->update_state();

            // Get control input
            const Eigen::VectorXd u = robot_state->get_gravity(robot_state->q);

            // Set control input
            kinova_gen3->set_joint_torque(u);
            loop_counter++;

            loop_rate.sleep();
        }

        RCLCPP_INFO(get_logger(), "Get initial state done!");
    }
    // get initial state
    kinova_gen3->update_state();

    // Initialize robot state & controller
    controller.init();
    std::string controller_type = robot_state->config["init_controller"].as<std::string>();
    controller_selector = controller.select(controller_type);

    //start main control loop
    state_pub->on_activate();
    main_control_loop_timer->reset();

    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void KinovaControllerNode::create_update_gain_subscriber()
{
    auto callback = [this](const controller_interface_msgs::msg::UpdateGain::SharedPtr msg) -> void
    {
        RCLCPP_INFO(get_logger(), "Received update gain message");
        if(msg->update)
        {   
            RCLCPP_INFO(get_logger(), "Update gain");
            robot_state->update_config();
            controller.update_controller_gain(robot_state->config);
        }
        
    };

    update_gain_sub = this->create_subscription<controller_interface_msgs::msg::UpdateGain>(
        update_gain_topic_name,
        rclcpp::QoS(10),
        callback
    );
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
KinovaControllerNode::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Activating");
    
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
KinovaControllerNode::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Deactivating");
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
KinovaControllerNode::on_cleanup(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Cleaning up");

    // Destroy kinova gen3: set position control mode
    kinova_gen3->destroy();
    state_pub->on_deactivate();
    main_control_loop_timer->cancel();

    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
KinovaControllerNode::on_shutdown(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Shutting down");
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point,
// allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(KinovaControllerNode)
