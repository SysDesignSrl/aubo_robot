#ifndef AUBO_HARDWARE_INTERFACE_H
#define AUBO_HARDWARE_INTERFACE_H
// STL
#include <string>
#include <vector>
// roscpp
#include <ros/ros.h>
#include <ros/console.h>
// hardware_interface
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
// aubo_hardware_interface
#include "aubo_hardware_interface/aubo_driver.h"


namespace aubo_hardware_interface {

class AuboHW : public hardware_interface::RobotHW {
private:
  aubo::AuboDriver aubo_driver;

  ros::NodeHandle node;

  double loop_hz;
  std::vector<std::string> joint_names;

  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;

  std::vector<double> j_pos, j_pos_cmd;
  std::vector<double> j_vel, j_vel_cmd;
  std::vector<double> j_eff, j_eff_cmd;

public:

  AuboHW(const ros::NodeHandle &node = ros::NodeHandle()) : node(node)
  {
    init();
  }


  bool init()
  {
    if (!node.getParam("/aubo/hardware_interface/loop_hz", loop_hz))
    {
      ROS_ERROR("Failed to retrieve '/aubo/hardware_interface/loop_hz' parameter.");
      return false;
    }

    if (!node.getParam("/aubo/hardware_interface/joints", joint_names))
    {
      ROS_ERROR("Failed to retrieve '/aubo/hardware_interface/joints' parameter.");
      return false;
    }

    int n_joints = joint_names.size();

    j_pos.resize(n_joints, 0.0); j_pos_cmd.resize(n_joints, 0.0);
    j_vel.resize(n_joints, 0.0); j_vel_cmd.resize(n_joints, 0.0);
    j_eff.resize(n_joints, 0.0); j_eff_cmd.resize(n_joints, 0.0);

    for (int i=0; i < n_joints; i++)
    {
      hardware_interface::JointStateHandle jnt_state_handle(joint_names[i], &j_pos[i], &j_vel[i], &j_eff[i]);
      jnt_state_interface.registerHandle(jnt_state_handle);

      hardware_interface::JointHandle jnt_pos_handle(jnt_state_handle, &j_pos_cmd[i]);
      jnt_pos_interface.registerHandle(jnt_pos_handle);

      hardware_interface::JointHandle jnt_vel_handle(jnt_state_handle, &j_vel_cmd[i]);
      jnt_vel_interface.registerHandle(jnt_vel_handle);
    }

    registerInterface(&jnt_state_interface);
    registerInterface(&jnt_pos_interface);
    registerInterface(&jnt_vel_interface);

    return true;
  }


  bool start()
  {
    auto hostname = node.param<std::string>("tcp/hostname", "localhost");
    auto port = node.param<int>("tcp/port", 8899);

    // Login
    if (!aubo_driver.login(hostname, port))
    {
      node.setParam("robot_connected", false);
      ROS_ERROR("Failed to connect to %s:%d", hostname.c_str(), port);
      return false;
    }

    node.setParam("robot_connected", true);
    ROS_INFO("Connected to %s:%d", hostname.c_str(), port);

    // Startup
    if (!aubo_driver.robot_startup())
    {
      ROS_ERROR("Failed to start up the Robot.");
      return false;
    }

    ROS_INFO("Robot started up correctly.");

    // TCP 2 CANbus
    if (!aubo_driver.enable_tcp_canbus_mode())
    {
      ROS_ERROR("Failed to enable TCP 2 CANbus Mode.");
      return false;
    }

    ROS_INFO("Enabled TCP 2 CANbus Mode.");

    return true;
  }


  bool stop()
  {
    // TCP 2 CANbus
    if (!aubo_driver.disable_tcp_canbus_mode())
    {
      ROS_ERROR("Failed to disable TCP 2 CANbus Mode.");
      return false;
    }

    ROS_INFO("Disabled TCP 2 CANbus Mode.");

    // Shutdown
    if (!aubo_driver.robot_shutdown())
    {
      ROS_ERROR("Failed to shutdown the Robot.");
      return false;
    }

    ROS_INFO("Robot shutted down correctly.");

    // Logout
    if (!aubo_driver.logout())
    {
      ROS_ERROR("Failed to log out.");
      return false;
    }

    node.setParam("robot_connected", false);
    ROS_INFO("Logged out.");

    return true;
  }


  void read()
  {
    if (!aubo_driver.read(j_pos))
    {
      ROS_ERROR_THROTTLE(1.0, "Failed to read joint positions from robot!");
    }
  }


  void write()
  {
    if (!aubo_driver.write(j_pos_cmd))
    {
      ROS_ERROR_THROTTLE(1.0, "Failed to write joint positions command to robot!");
    }
  }

};

}  // namespace
#endif
