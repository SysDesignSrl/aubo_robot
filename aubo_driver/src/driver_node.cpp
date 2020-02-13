// STL
#include <sstream>
#include <string>
#include <vector>
//
#include <ros/ros.h>
#include <ros/console.h>
// std_srvs
#include <std_srvs/Trigger.h>
// sensor_msgs
#include <sensor_msgs/JointState.h>
//
#include "aubo_driver/aubo_robot.h"
// AUBO SDK
#include "lib/AuboRobotMetaType.h"
#include "lib/serviceinterface.h"


// Parameters
std::vector<std::string> joint_names;

// Published Topics
ros::Publisher joint_state_pub;


void realtime_waypoint_cb(const aubo_robot_namespace::wayPoint_S *wayPoint, void *arg)
{
  std::vector<double> joint_pos(aubo_robot_namespace::ARM_DOF);
  std::copy(wayPoint->jointpos, wayPoint->jointpos + aubo_robot_namespace::ARM_DOF, joint_pos.begin());

  sensor_msgs::JointState joint_state;
  joint_state.header.stamp = ros::Time::now();
  joint_state.name = joint_names;
  joint_state.position = joint_pos;
  joint_state_pub.publish(joint_state);
}


void event_info_cb(const aubo_robot_namespace::RobotEventInfo *eventInfo, void *arg)
{
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
  if (!node.getParam("joint_names", joint_names))
  {
    ROS_FATAL("Failed to retrieve '%s' parameter.", "joint_names");
    return 1;
  }

  // Published Topics
  joint_state_pub = node.advertise<sensor_msgs::JointState>("joint_states", 100);


  // AUBO Robot
  aubo::AuboRobot robot(node);

  if (!robot.init())
  {
    ROS_FATAL("Failed to initialize the robot!");
    return 1;
  }

  // Login
  if (!robot.login())
  {
    ROS_FATAL("Failed to log in.");
    return 1;
  }

  // Robot startup
  if (!robot.robot_startup())
  {
    ROS_FATAL("Failed to startup the robot.");
    return 1;
  }

  // register robot callbacks
  robot.register_realtime_waypoint(realtime_waypoint_cb, nullptr);
  robot.register_event_info(event_info_cb, nullptr);


  // init motion profile
  robot.init_profile();

  // set max joint acceleration
  std::vector<double> max_joint_acc;
  if (node.getParam("aubo/max_joint_acceleration", max_joint_acc))
  {
    robot.set_max_joint_acceleration(max_joint_acc);
  }

  // set max joint velocity
  std::vector<double> max_joint_vel;
  if (node.getParam("aubo/max_joint_velocity", max_joint_vel))
  {
    robot.set_max_joint_velocity(max_joint_vel);
  }


  robot.get_max_joint_acceleration(max_joint_acc);

  std::stringstream ass;
  ass << "[ ";
  for (double val : max_joint_acc)
  {
    ass << val << " ";
  }
  ass << "] ";
  ROS_DEBUG_STREAM("max joint acceleration: " << ass.str() << "[rad/s^2]");


  robot.get_max_joint_velocity(max_joint_vel);

  std::stringstream vss;
  vss << "[ ";
  for (double val : max_joint_vel)
  {
    vss << val << " ";
  }
  vss << "] ";
  ROS_DEBUG_STREAM("max joint velocity: " << vss.str() << "[rad/s]");


  double max_linear_acc;
  if (node.getParam("aubo/max_linear_acceleration", max_linear_acc))
  {
    robot.set_max_linear_acceleration(max_linear_acc);
  }
  double max_linear_vel;
  if (node.getParam("aubo/max_linear_velocity", max_linear_vel))
  {
    robot.set_max_linear_velocity(max_linear_vel);
  }
  double max_angular_acc;
  if (node.getParam("aubo/max_angular_acceleration", max_angular_acc))
  {
    robot.set_max_angular_acceleration(max_angular_acc);
  }
  double max_angular_vel;
  if (node.getParam("aubo/max_angular_velocity", max_angular_vel))
  {
    robot.set_max_angular_velocity(max_angular_vel);
  }

  // double max_linear_acc;
  robot.get_max_linear_acceleration(max_linear_acc);
  ROS_DEBUG("max linear acceleration: %.2f [m/s^2]", max_linear_acc);

  // double max_linear_vel;
  robot.get_max_linear_velocity(max_linear_vel);
  ROS_DEBUG("max linear velocity: %.2f [m/s]", max_linear_vel);

  // double max_angular_acc;
  robot.get_max_angular_acceleration(max_angular_acc);
  ROS_DEBUG("max angular acceleration: %.2f [rad/s^2]", max_angular_acc);

  // double max_angular_vel;
  robot.get_max_angular_velocity(max_angular_vel);
  ROS_DEBUG("max angular velocity: %.2f [rad/s]", max_angular_vel);


  //
  ros::spin();


  // Robot Shutdown
  if (!robot.robot_shutdown())
  {
    ROS_FATAL("Failed to shutdown the robot.");
    return 1;
  }

  // Logout
  if (!robot.logout())
  {
    ROS_FATAL("Failed to log out.");
    return 1;
  }

  return 0;
}
