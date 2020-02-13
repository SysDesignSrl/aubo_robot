#ifndef AUBO_HARDWARE_INTERFACE_H
#define AUBO_HARDWARE_INTERFACE_H
// STL
#include <string>
#include <vector>
#include <algorithm>
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


  bool init_robot()
  {
    std::vector<double> max_joint_acc;
    if (!node.getParam("aubo/max_joint_acceleration", max_joint_acc))
    {
      ROS_ERROR("Failed to retrieve 'max_joint_acceleration' parameter.");
      return false;
    }
    std::vector<double> max_joint_vel;
    if (!node.getParam("aubo/max_joint_velocity", max_joint_vel))
    {
      ROS_ERROR("Failed to retrieve 'max_joint_velocity' parameter.");
      return false;
    }

    if (!aubo_driver.set_max_joint_acceleration(max_joint_acc))
    {
      ROS_ERROR("Failed to set max joint acceleration!");
      return false;
    }

    aubo_driver.get_max_joint_acceleration(max_joint_acc);
    ROS_DEBUG("max joint acc: [ %f %f %f %f %f %f ]", max_joint_acc[0], max_joint_acc[1], max_joint_acc[2], max_joint_acc[3], max_joint_acc[4], max_joint_acc[5]);

    if (!aubo_driver.set_max_joint_velocity(max_joint_vel))
    {
      ROS_ERROR("Failed to set max joint velocity!");
      return false;
    }

    aubo_driver.get_max_joint_velocity(max_joint_vel);
    ROS_DEBUG("max joint vel: [ %f %f %f %f %f %f ]", max_joint_vel[0], max_joint_vel[1], max_joint_vel[2], max_joint_vel[3], max_joint_vel[4], max_joint_vel[5]);

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


    if (!init_robot())
    {
      ROS_FATAL("Failed to initialize the Robot.");
      return false;
    }


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
    // ROS_DEBUG("j_pos: [ %f %f %f %f %f %f ]", j_pos[0], j_pos[1], j_pos[2], j_pos[3], j_pos[4], j_pos[5]);
  }


  void write()
  {
    if (std::all_of(j_pos_cmd.cbegin(), j_pos_cmd.cend(), [](double value) { return value == 0.0; }))
    {
      return;
    }

    if (aubo_driver.get_robot_diagnostic_info())
    {
      ROS_ERROR_THROTTLE(1.0, "Failed to get robot diagnostic info!");
    }

    uint16 buffer_max_size = aubo_driver.robotDiagnosis.macTargetPosBufferSize;
    uint16 buffer_size = aubo_driver.robotDiagnosis.macTargetPosDataSize;
    uint8 buffer_warning = aubo_driver.robotDiagnosis.macDataInterruptWarning;

    ROS_DEBUG_THROTTLE(1.0, "CAN buffer max size: %d", buffer_max_size);
    ROS_DEBUG_THROTTLE(1.0, "CAN buffer size: %d", buffer_size);
    ROS_DEBUG_THROTTLE(1.0, "CAN buffer Warining: %d", buffer_warning);


    if (!aubo_driver.write(j_pos_cmd))
    {
      ROS_ERROR_THROTTLE(1.0, "Failed to write joint positions command to robot!");
    }
  }

};

}  // namespace
#endif
