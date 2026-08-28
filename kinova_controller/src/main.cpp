#include "node/kinova_controller_node.hpp"

#include <memory>
#include <filesystem>
#include <utility>
#include <string>
#include <thread>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "utils/process_settings.hpp"
#include "utils/lifecycle_autostart.hpp"

int main(int argc, char * argv[])
{
  utils::ProcessSettings settings;
  if (!settings.init(argc, argv)) {
    return EXIT_FAILURE;
  }

  int32_t ret = 0;
  std::unique_ptr<mj::Simulate> sim;
  std::thread render_thread;
  try {
    // configure process real-time settings
    if (settings.configure_child_threads) {
      // process child threads created by ROS nodes will inherit the settings
      settings.configure_process();
    }
    rclcpp::init(argc, argv);

    // Create a static executor
    rclcpp::executors::StaticSingleThreadedExecutor exec;

    const std::filesystem::path controller_share =
      ament_index_cpp::get_package_share_directory("kinova_controller");
    const std::filesystem::path robot_model_share =
      ament_index_cpp::get_package_share_directory("robot_model");
    const std::filesystem::path config_path =
      controller_share / "config" / "kinova_controller_config.yaml";

    std::cout << "config yaml file path: " << config_path << std::endl;
    YAML::Node config = YAML::LoadFile(config_path.string());

    const auto resolve_robot_model_path = [&robot_model_share](const std::string & path) {
      const std::filesystem::path model_path(path);
      return model_path.is_absolute() ? model_path : robot_model_share / model_path;
    };

    const auto mujoco_xml_path = resolve_robot_model_path(
      config["mujoco_xml_path"].as<std::string>());
    const auto pinocchio_urdf_path = resolve_robot_model_path(
      config["pinocchio_urdf_path"].as<std::string>());
    config["mujoco_xml_path"] = mujoco_xml_path.string();
    config["pinocchio_urdf_path"] = pinocchio_urdf_path.string();


    // Create pinocchio model
    pinocchio::Model pinocchio_model;
    pinocchio::urdf::buildModel(pinocchio_urdf_path.string(), pinocchio_model);
    pinocchio_model.gravity.linear(pinocchio::Model::gravity981);

    // Create RobotState
    std::shared_ptr<RobotState> robot_state =
      std::make_shared<RobotState>(config, pinocchio_model, config_path.string());

    // For simulation
    if(robot_state->is_simulation)
    {
        // init GLFW
        if (!Glfw().glfwInit()) {
            mju_error("could not initialize GLFW");
        }

        sim = std::make_unique<mj::Simulate>();
        
        sim->ui0_enable = 0; // fold left ui
        sim->ui1_enable = 0; // fold right ui
        sim->info = 1; // show info

        render_thread = std::thread([sim_ptr = sim.get()]() {sim_ptr->renderloop();});
    }

    // Create KinovaControllerNode
    const auto kinova_controller_node_ptr = std::make_shared<KinovaControllerNode>("kinova_controller");
    kinova_controller_node_ptr->assign_robot_state(robot_state);

    if(robot_state->is_simulation)
    {
      kinova_controller_node_ptr->create_simulation_robot(sim.get());
    }
    else
    {
      kinova_controller_node_ptr->create_real_robot();
    }

  
    exec.add_node(kinova_controller_node_ptr->get_node_base_interface());

    // configure process real-time settings
    if (!settings.configure_child_threads) {
      // process child threads created by ROS nodes will NOT inherit the settings
      settings.configure_process();
    }

    if (settings.auto_start_nodes) {
      utils::autostart(*kinova_controller_node_ptr);
    }
    exec.spin();

  } catch (const std::exception & e) {
    RCLCPP_INFO(rclcpp::get_logger("kinova_controller"), e.what());
    ret = 2;
  } catch (...) {
    RCLCPP_INFO(
      rclcpp::get_logger("kinova_controller"), "Unknown exception caught. "
      "Exiting...");
    ret = -1;
  }

  if (sim) {
    sim->exitrequest.store(true);
  }
  if (render_thread.joinable()) {
    render_thread.join();
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return ret;
}
