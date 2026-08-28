#pragma once

#include <string>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"

namespace utils
{
/// \brief Transit a LifecycleNode from inactive to active state
void autostart(rclcpp_lifecycle::LifecycleNode & node)
{ 
  if (lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE != node.configure().id()) {
    throw std::runtime_error("Could not configure " + std::string(node.get_name()));
  }
  if (lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE != node.activate().id()) {
    throw std::runtime_error("Could not activate " + std::string(node.get_name()));
  }
}
}  // namespace utils


