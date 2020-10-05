//STL
#include <string>
#include <vector>

// roscpp
#include <ros/ros.h>
#include <ros/console.h>
// std_msgs
#include <std_msgs/Bool.h>
// industrial_msgs
#include <industrial_msgs/RobotStatus.h>
#include <industrial_msgs/RobotMode.h>
#include <industrial_msgs/TriState.h>
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
  auto freq = node.param<double>("publish_frequency", 10);

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

  // Published Topics
  auto connected_pub = node.advertise<std_msgs::Bool>("connected", 1);
  auto arm_powered_pub = node.advertise<std_msgs::Bool>("arm_powered", 1);
  auto robot_collision_pub = node.advertise<std_msgs::Bool>("robot_collision", 1);
  auto singularity_overspeed_pub = node.advertise<std_msgs::Bool>("singularity_overspeed", 1);
  auto robot_overcurrent_pub = node.advertise<std_msgs::Bool>("robot_overcurrent", 1);
  auto robot_status_pub = node.advertise<industrial_msgs::RobotStatus>("status", 10);


  node.setParam("connected", false);

  // Loop
  ros::Rate rate(freq);
  while (ros::ok())
  {
    ros::spinOnce();

    const ros::Time time = ros::Time::now();

    std_msgs::Bool msg;

    bool connected;
    node.getParamCached("connected", connected);

    msg.data = connected;
    connected_pub.publish(msg);

    msg.data = aubo_hw.robot.arm_powered;
    arm_powered_pub.publish(msg);

    msg.data = aubo_hw.robot.collision;
    robot_collision_pub.publish(msg);

    msg.data = aubo_hw.robot.singularity_overspeed;
    singularity_overspeed_pub.publish(msg);

    msg.data = aubo_hw.robot.overcurrent;
    robot_overcurrent_pub.publish(msg);

    if (!connected)
    {
      industrial_msgs::RobotStatus msg;
      msg.header.stamp = time;
      msg.mode.val = industrial_msgs::RobotMode::UNKNOWN;
      msg.e_stopped.val = industrial_msgs::TriState::UNKNOWN;
      msg.drives_powered.val = industrial_msgs::TriState::UNKNOWN;
      msg.motion_possible.val = industrial_msgs::TriState::UNKNOWN;
      msg.in_motion.val = industrial_msgs::TriState::UNKNOWN;
      msg.in_error.val = industrial_msgs::TriState::UNKNOWN;
      msg.error_code = 0;
      robot_status_pub.publish(msg);
    }
    else
    {
      industrial_msgs::RobotStatus msg;
      msg.header.stamp = time;
      msg.mode.val = industrial_msgs::RobotMode::AUTO;
      msg.e_stopped.val = (aubo_hw.robot.soft_emergency || aubo_hw.robot.remote_emergency) ? industrial_msgs::TriState::TRUE : industrial_msgs::TriState::FALSE;
      msg.drives_powered.val = (aubo_hw.robot.arm_powered) ? industrial_msgs::TriState::TRUE : industrial_msgs::TriState::FALSE;
      msg.motion_possible.val = (aubo_hw.robot.arm_powered) ? industrial_msgs::TriState::TRUE : industrial_msgs::TriState::FALSE;
      msg.in_motion.val = industrial_msgs::TriState::UNKNOWN;
      msg.in_error.val = (aubo_hw.robot.collision || aubo_hw.robot.singularity_overspeed || aubo_hw.robot.overcurrent) ? industrial_msgs::TriState::TRUE : industrial_msgs::TriState::FALSE;
      msg.error_code = 0;
      robot_status_pub.publish(msg);
    }

    rate.sleep();
  }

  return 0;
}
