// roscpp
#include <ros/ros.h>
#include <ros/console.h>
// controller manager
#include <controller_manager/controller_manager.h>
// aubo_hardware_interface
#include "aubo_hardware_interface/aubo_hardware_interface.h"


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "aubo_hardware_interface");

  // Node
  ros::NodeHandle node("~");

  // Parameters
  auto host = node.param<std::string>("tcp/host", "localhost");
  auto port = node.param<int>("tcp/port", 8899);

  double loop_hz;
  if (!node.getParam("hardware_interface/loop_hz", loop_hz))
  {
    std::string param_name = node.resolveName("hardware_interface/loop_hz");
    ROS_ERROR("Failed to retrieve '%s' parameter.", param_name.c_str());
    return 1;
  }

  std::vector<std::string> joints;
  if (!node.getParam("hardware_interface/joints", joints))
  {
    std::string param_name = node.resolveName("hardware_interface/joints");
    ROS_ERROR("Failed to retrieve '%s' parameter.", param_name.c_str());
    return 1;
  }

  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Hardware Interface
  aubo_hardware_interface::AuboHW aubo_hw(node);
  if (!aubo_hw.init(loop_hz, joints))
  {
    ROS_FATAL("Failed to initialize Hardware Interface.");
    return 1;
  }

  // Services
  auto login_srv = node.advertiseService("login", &aubo_hardware_interface::AuboHW::login, &aubo_hw);
  auto logout_srv = node.advertiseService("logout", &aubo_hardware_interface::AuboHW::logout, &aubo_hw);
  auto robot_startup_srv = node.advertiseService("robot_startup", &aubo_hardware_interface::AuboHW::robot_startup, &aubo_hw);
  auto robot_shutdown_srv = node.advertiseService("robot_shutdown", &aubo_hardware_interface::AuboHW::robot_shutdown, &aubo_hw);
  auto reset_controllers_srv = node.advertiseService("reset_controllers", &aubo_hardware_interface::AuboHW::reset, &aubo_hw);

  ros::waitForShutdown();
  return 0;
}
