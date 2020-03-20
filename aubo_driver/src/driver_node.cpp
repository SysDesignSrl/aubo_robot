// STL
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
//
#include <ros/ros.h>
#include <ros/console.h>
// std_srvs
#include <std_srvs/Trigger.h>
// geometry_msgs
#include <geometry_msgs/PoseStamped.h>
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
ros::Publisher tool_pose_pub;


void realtime_waypoint_cb(const aubo_robot_namespace::wayPoint_S *wayPoint, void *arg)
{
  ros::Time stamp = ros::Time::now();

  double x, y, z;
  x = wayPoint->cartPos.position.x;
  y = wayPoint->cartPos.position.y;
  z = wayPoint->cartPos.position.z;

  double qx, qy, qz, qw;
  qx = wayPoint->orientation.x;
  qy = wayPoint->orientation.y;
  qz = wayPoint->orientation.z;
  qw = wayPoint->orientation.w;

  geometry_msgs::PoseStamped tool_pose;
  tool_pose.header.stamp = stamp;
  tool_pose.header.frame_id = "base_link";
  tool_pose.pose.position.x = x;
  tool_pose.pose.position.y = y;
  tool_pose.pose.position.z = z;
  tool_pose.pose.orientation.x = qx;
  tool_pose.pose.orientation.y = qy;
  tool_pose.pose.orientation.z = qz;
  tool_pose.pose.orientation.w = qw;
  tool_pose_pub.publish(tool_pose);

  std::vector<double> joint_pos(aubo_robot_namespace::ARM_DOF);
  std::copy(wayPoint->jointpos, wayPoint->jointpos + aubo_robot_namespace::ARM_DOF, joint_pos.begin());

  sensor_msgs::JointState joint_state;
  joint_state.header.stamp = stamp;
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
  tool_pose_pub = node.advertise<geometry_msgs::PoseStamped>("tool_pose", 100);


  // AUBO Robot
  aubo::AuboRobot robot(node);

  if (!robot.init())
  {
    ROS_FATAL("Failed to initialize the robot!");
    return 1;
  }

  if (!robot.start())
  {
    ROS_FATAL("Failed to start up the robot!");
    return 1;
  }


  //
  robot.register_realtime_waypoint(realtime_waypoint_cb, nullptr);
  //
  robot.register_event_info(event_info_cb, nullptr);


  //
  ros::spin();


  if (!robot.stop())
  {
    ROS_FATAL("Failed to shutdown the robot!");
    return 1;
  }

  return 0;
}
