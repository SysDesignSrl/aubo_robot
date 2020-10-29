#ifndef AUBO_HARDWARE_INTERFACE_H
#define AUBO_HARDWARE_INTERFACE_H
// STL
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>

// roscpp
#include <ros/ros.h>
#include <ros/console.h>
// std_srvs
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

// hardware_interface
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
// controller manager
#include <controller_manager/controller_manager.h>
// aubo_hardware_interface
#include "aubo_hardware_interface/aubo_robot.h"


namespace aubo_hardware_interface {

class AuboHW : public hardware_interface::RobotHW {
private:

  // Controller Manager
  controller_manager::ControllerManager controller_manager;
  bool reset_controllers = true;

  ros::NodeHandle node;

  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;

  std::vector<double> j_pos, j_pos_cmd;
  std::vector<double> j_vel, j_vel_cmd;
  std::vector<double> j_eff, j_eff_cmd;


  void control_loop_cb(const ros::TimerEvent &ev)
  {
    if (robot.soft_emergency)
    {
      // control_loop.stop();
      robot_shutdown();
      print_diagnostic_info();
      return;
    }

    if (robot.remote_emergency)
    {
      // control_loop.stop();
      robot_shutdown();
      print_diagnostic_info();
      return;
    }

    if (robot.robot_collision)
    {
      // control_loop.stop();
      robot_shutdown();
      print_diagnostic_info();
      return;
    }

    if (robot.singularity_overspeed)
    {
      // control_loop.stop();
      robot_shutdown();
      print_diagnostic_info();
      return;
    }

    if (robot.robot_overcurrent)
    {
      // control_loop.stop();
      robot_shutdown();
      print_diagnostic_info();
      return;
    }

    if (robot.safe_io)
    {
      // control_loop.stop();
      robot_shutdown();
      print_diagnostic_info();
      return;
    }

    const ros::Time time = ev.current_real;
    const ros::Duration period = ev.current_real - ev.last_real;

    read(time, period);
    controller_manager.update(time, period, reset_controllers);
    write(time, period);

    reset_controllers = false;
  }

public:
  aubo::AuboRobot robot;

  ros::Timer control_loop;

  // Diagnostic Info
  struct
  {
    bool arm_power_status;              // The switch status (on, off) of robot 48V power
    double arm_power_current;           // The current of robot 48V power
    double arm_power_voltage;           // The voltage of robot 48V power
    uint8 arm_canbus_status;            // 0x00: No error 0xff: CAN bus error

    bool remote_halt;                   // Remote halt signal
    bool soft_emergency;                // Robot soft emergency
    bool remote_emergency;              // Remote emergency sugnal

    bool robot_collision;               // Collision detection bit
    bool static_collision;              // Static collision detection switch
    uint8 joint_collision;              // Joint collision detection, each joint occupies 1 bit, 0-collision inexistence 1-collision existence

    bool force_control_mode;            // The flag bit of robot starting force control mode
    bool brake_status;                  // Brake status
    bool orpe_status;                   // The status bit of the software (ORPE)

    bool encoder_error;                 // Magnetic encoder error status
    bool encoder_lines_error;           // Optical-electricity encoders are not same, 0-no error, 1-error
    bool joint_error;                   // Joint error status
    uint8 tool_io_error;                // Tool error

    bool singularity_overspeed;         // The overspeed alarm of robot singularity
    bool robot_overcurrent;             // The alarm of robot current flow

    bool robot_mounting_pose_warning;   // The mounting position of the robot is wrong (Working on the force control only)

    uint16 can_buffer_size;             // The size of the mac buffer
    uint16 can_data_size;               // The valid data size of the mac buffer
    uint8 can_data_warning;             // The mac data interruption
  }
  robot_diagnostic;


  AuboHW(const ros::NodeHandle &node = ros::NodeHandle()) :
    node(node),
    controller_manager(this, node) { }


  bool init(double loop_hz, const std::vector<std::string> &joints)
  {
    ros::Duration period(1.0/loop_hz);
    control_loop = node.createTimer(period, &aubo_hardware_interface::AuboHW::control_loop_cb, this, false, false);

    const int n_joints = joints.size();

    j_pos.resize(n_joints, 0.0); j_pos_cmd.resize(n_joints, 0.0);
    j_vel.resize(n_joints, 0.0); j_vel_cmd.resize(n_joints, 0.0);
    j_eff.resize(n_joints, 0.0); j_eff_cmd.resize(n_joints, 0.0);

    for (int i = 0; i < n_joints; i++)
    {
      hardware_interface::JointStateHandle jnt_state_handle(joints[i], &j_pos[i], &j_vel[i], &j_eff[i]);
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


  bool init_robot();

  bool login(std::string host,  unsigned int port);
  bool login(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  bool logout();
  bool logout(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  bool robot_startup();
  bool robot_startup(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  bool robot_shutdown();
  bool robot_shutdown(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  void print_diagnostic_info();
  bool print_diagnostic_info(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res);


  void read(const ros::Time &time, const ros::Duration &period)
  {
    if (!robot.read(j_pos))
    {
      reset_controllers = true;
      ROS_ERROR_THROTTLE(1.0, "Failed to read joint positions state from robot!");
    }
  }


  void write(const ros::Time &time, const ros::Duration &period)
  {
    if (!robot.write(j_pos_cmd))
    {
      reset_controllers = true;
      ROS_ERROR_THROTTLE(1.0, "Failed to write joint positions command to robot!");
    }
  }

};

}  // namespace
#endif
