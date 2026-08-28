#pragma once

#include <string>
#include <climits>
#include <memory>
#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"

#include "kinova_controller/robot/kinova_gen3.hpp"
#include "kinova_controller/robot/kinova_gen3_simulation.hpp"
#include "kinova_controller/controller/controller.hpp"

#include "robot_state_msgs/msg/extended_state.hpp"
#include "robot_state_msgs/msg/controller_logging.hpp"

#include "controller_interface_msgs/msg/controller_config.hpp"
#include "controller_interface_msgs/msg/set_desired_joint_value.hpp"
#include "controller_interface_msgs/msg/set_desired_task_value.hpp"
#include "controller_interface_msgs/msg/update_gain.hpp"

using namespace std::chrono_literals;

class KinovaControllerNode : public rclcpp_lifecycle::LifecycleNode
{
    public:
    /// \brief Default constructor, needed for node composition
    // / \param[in] options Node options for rclcpp internals
    explicit KinovaControllerNode(const rclcpp::NodeOptions & options);

    /// \brief Parameter file constructor
    /// \param[in] node_name Name of this node
    /// \param[in] options Node options for rclcpp internals
    explicit KinovaControllerNode(const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    ~KinovaControllerNode();

    void assign_robot_state(std::shared_ptr<RobotState> robot_state_arg){robot_state = robot_state_arg;}
    void create_real_robot();
    void create_simulation_robot(mj::Simulate* sim);

    private:
    // Gen3 Robot
    std::shared_ptr<KinovaGen3Base> kinova_gen3;

    // Controller
    Controller controller;
    CONTROLLER_SELECTOR controller_selector;
    
    // Config
    std::shared_ptr<RobotState> robot_state;

    //for state publish
    const std::string state_topic_name;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<robot_state_msgs::msg::ExtendedState>> state_pub;
    rclcpp::TimerBase::SharedPtr main_control_loop_timer;
    std::chrono::microseconds main_control_loop_period;

    //for controller config subscribe
    const std::string controller_config_topic_name;
    rclcpp::Subscription<controller_interface_msgs::msg::ControllerConfig>::SharedPtr controller_config_sub;

    //for desired joint value subscribe
    const std::string desired_joint_value_topic_name;
    rclcpp::Subscription<controller_interface_msgs::msg::SetDesiredJointValue>::SharedPtr desired_joint_value_sub;

    //for desired task value subscribe
    const std::string desired_task_value_topic_name;
    rclcpp::Subscription<controller_interface_msgs::msg::SetDesiredTaskValue>::SharedPtr desired_task_value_sub;

    //for update gain subscribe
    const std::string update_gain_topic_name;
    rclcpp::Subscription<controller_interface_msgs::msg::UpdateGain>::SharedPtr update_gain_sub;

    //for controller logging
    const std::string controller_logging_topic_name;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<robot_state_msgs::msg::ControllerLogging>> controller_logging_pub;


    /// \brief main control loop
    void main_control_loop();

    /// \brief Create robot_state_msgs/msg/extended_state publisher
    void create_state_publisher();

    /// \brief Create main control loop callback
    void create_main_control_loop_callback();

    /// \brief Create controller config subscriber
    void create_controller_config_subscriber();

    /// \brief Create desired joint value subscriber
    void create_desired_joint_value_subscriber();

    /// \brief Create desired task value subscriber
    void create_desired_task_value_subscriber();

    /// \biref Create update_gain msg subscriber
    void create_update_gain_subscriber();

    /// \brief Create controller logging publisher
    void create_controller_logging_publisher();
    void publish_controller_logging();

    

    /// \brief Transition callback for state configuring
    /// \param[in] lifecycle node state
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &) override;

    /// \brief Transition callback for state activating
    /// \param[in] lifecycle node state
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &) override;

    /// \brief Transition callback for state deactivating
    /// \param[in] lifecycle node state
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &) override;

    /// \brief Transition callback for state cleaningup
    /// \param[in] lifecycle node state
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &) override;

    /// \brief Transition callback for state shutting down
    /// \param[in] lifecycle node state
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State & state) override;

    uint32_t num_missed_deadlines_pub;
    std::chrono::milliseconds deadline_duration;
};
