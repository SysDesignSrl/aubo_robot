#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
// roscpp
#include <ros/ros.h>
#include <ros/console.h>
// std_msgs
#include <std_msgs/UInt16.h>
// std_srvs
#include <std_srvs/Trigger.h>
// geometry_msgs
#include <geometry_msgs/PoseStamped.h>
// sensor_msgs
#include <sensor_msgs/JointState.h>
// diagnostic_msgs
#include <diagnostic_msgs/DiagnosticArray.h>
// aubo_driver
#include "aubo_driver/aubo_robot.h"
// AUBO SDK
#include "lib/AuboRobotMetaType.h"
#include "lib/serviceinterface.h"
// Boost
#include <boost/bind.hpp>
#include <boost/function.hpp>


void realtime_waypoint_cb(const aubo_robot_namespace::wayPoint_S *wayPoint, void *arg)
{
  aubo::AuboRobot* robot = (aubo::AuboRobot*)arg;

  ros::Time stamp = ros::Time::now();

  // double x, y, z;
  // x = wayPoint->cartPos.position.x;
  // y = wayPoint->cartPos.position.y;
  // z = wayPoint->cartPos.position.z;
  //
  // double qx, qy, qz, qw;
  // qx = wayPoint->orientation.x;
  // qy = wayPoint->orientation.y;
  // qz = wayPoint->orientation.z;
  // qw = wayPoint->orientation.w;
  //
  // geometry_msgs::PoseStamped tool_pose;
  // tool_pose.header.stamp = stamp;
  // tool_pose.header.frame_id = "base_link";
  // tool_pose.pose.position.x = x;
  // tool_pose.pose.position.y = y;
  // tool_pose.pose.position.z = z;
  // tool_pose.pose.orientation.x = qx;
  // tool_pose.pose.orientation.y = qy;
  // tool_pose.pose.orientation.z = qz;
  // tool_pose.pose.orientation.w = qw;
  // tool_pose_pub.publish(tool_pose);

  std::vector<double> joint_pos(aubo_robot_namespace::ARM_DOF);
  std::copy(wayPoint->jointpos, wayPoint->jointpos + aubo_robot_namespace::ARM_DOF, joint_pos.begin());

  sensor_msgs::JointState joint_state;
  joint_state.header.stamp = stamp;
  joint_state.name = robot->joint_names;
  joint_state.position = joint_pos;
  robot->joint_state_pub.publish(joint_state);
}


void event_info_cb(const aubo_robot_namespace::RobotEventInfo *eventInfo, void *arg)
{
  aubo::AuboRobot* robot = (aubo::AuboRobot*)arg;

  int code =  eventInfo->eventCode;
  std::string message = eventInfo->eventContent;

  ROS_INFO("Event Code: %d, %s", code, message.c_str());
}


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "aubo_driver");

  // Node
  ros::NodeHandle node("~");

  // Parameters
  auto host = node.param<std::string>("tcp/host", "localhost");
  auto port = node.param<int>("tcp/port", 8899);

  auto collision_class = node.param<int>("aubo/collision_class", 6);
  auto blend_radius = node.param<double>("aubo/blend_radius", 0.02);

  std::vector<std::string> joint_names;
  if (!node.getParam("joints", joint_names))
  {
    std::string param_name = node.resolveName("joints");
    ROS_ERROR("Failed to retrieve '%s' parameter.", param_name.c_str());
    return 1;
  }

  // AUBO Robot
  aubo::AuboRobot robot(node);

  if (robot.init(joint_names))
  {
    ROS_INFO("Robot initialized correctly.");
  }
  else
  {
    ROS_ERROR("Failed to initialize the robot.");
  }

  if (robot.login(host, port))
  {
    ROS_INFO("Logged in to %s:%d", host.c_str(), port);
  }
  else
  {
    ROS_ERROR("Failed to login to %s:%d", host.c_str(), port);
    return 1;
  }

  if (robot.init_robot())
  {
    ROS_INFO("Initialized robot parameters");
  }
  else
  {
    ROS_ERROR("Failed to initialize robot parameters");
    return 1;
  }

  if (robot.robot_startup(collision_class))
  {
    ROS_INFO("Robot started up with collision class: %d", collision_class);
  }
  else
  {
    ROS_ERROR("Failed to startup the robot.");
    return 1;
  }

  if (robot.start())
  {
    ROS_INFO("Action server started correctly");
  }
  else
  {
    ROS_ERROR("Failed to start action server.");
  }

  // Advertised Services
  auto login_srv = node.advertiseService("login", &aubo::AuboRobot::login, &robot);
  auto logout_srv = node.advertiseService("logout", &aubo::AuboRobot::logout, &robot);
  auto init_profile_srv = node.advertiseService("init_profile", &aubo::AuboRobot::init_profile, &robot);
  auto robot_startup_srv = node.advertiseService("robot_startup", &aubo::AuboRobot::robot_startup, &robot);
  auto robot_shutdown_srv = node.advertiseService("robot_shutdown", &aubo::AuboRobot::robot_shutdown, &robot);
  auto print_diagnostic_srv = node.advertiseService("print_diagnostic_info", &aubo::AuboRobot::print_diagnostic_info, &robot);

  // Subscribed Topics
  auto velocity_scaling_sub = node.subscribe<std_msgs::Float64>("velocity_scaling", 10, &aubo::AuboRobot::velocity_scaling_cb, &robot);
  auto acceleration_scaling_sub = node.subscribe<std_msgs::Float64>("acceleration_scaling", 10, &aubo::AuboRobot::acceleration_scaling_cb, &robot);
  auto digital_output_sub = node.subscribe<std_msgs::UInt16>("digital_output", 10, &aubo::AuboRobot::digital_output_cb, &robot);

  // Published Topics
  robot.joint_state_pub = node.advertise<sensor_msgs::JointState>("joint_states", 10);
  robot.tool_pose_pub = node.advertise<geometry_msgs::PoseStamped>("tool_pose", 10);
  robot.digital_input_pub = node.advertise<std_msgs::UInt16>("digital_input", 10);
  robot.diagnostic_pub = node.advertise<diagnostic_msgs::DiagnosticArray>("diagnostic", 10);

  robot.register_realtime_waypoint(realtime_waypoint_cb, &robot);
  robot.register_event_info(event_info_cb, &robot);

  ros::spin();

  if (robot.robot_shutdown())
  {
    ROS_INFO("Robot shutted down correctly.");
  }
  else
  {
    ROS_ERROR("Failed to shutdown the robot.");
    return 1;
  }

  if (robot.logout())
  {
    ROS_INFO("Logged out.");
  }
  else
  {
    ROS_ERROR("Failed to log out.");
    return 1;
  }

  return 0;
}
