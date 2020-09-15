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

  /*** Diagnostinc Info ***/

  // Arm
  bool arm_power_status;
  double arm_power_current;
  double arm_power_voltage;
  uint8 arm_canbus_status;
  // Emergency
  bool soft_emergency;
  bool remote_emergency;

  bool robot_collision;
  bool force_control_mode;
  bool brake_status;
  // Alarm
  bool singularity_overspeed_alarm;
  bool robot_current_alarm;
  // CAN bus
  uint16 can_buffer_size;
  uint16 can_data_size;
  uint8 can_data_warning;


  ros::NodeHandle node;
  ros::Timer refresh_cycle;
  ros::Timer control_loop;

  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;

  std::vector<double> j_pos, j_pos_cmd;
  std::vector<double> j_vel, j_vel_cmd;
  std::vector<double> j_eff, j_eff_cmd;

public:
  aubo::AuboRobot aubo_robot;

  AuboHW(const ros::NodeHandle &node = ros::NodeHandle()) :
    node(node),
    controller_manager(this, node)
  {
    // ros::Duration period(1.0);
    // refresh_cycle = node.createTimer(period, &aubo_hardware_interface::AuboHW::refresh_cycle_cb, this, false, false);
  }


  // void refresh_cycle_cb(const ros::TimerEvent &ev)
  // {
  //   if (!aubo_robot.get_robot_diagnostic_info())
  //   {
  //     ROS_WARN("Failed to retrieve diagnostic info from the robot!");
  //   }
  //
  //   arm_power_status = aubo_robot.robotDiagnosis.armPowerStatus;                // The switch status (on, off)of robot 48V power
  //   arm_power_current = aubo_robot.robotDiagnosis.armPowerCurrent;              // The current of robot 48V power
  //   arm_power_voltage = aubo_robot.robotDiagnosis.armPowerVoltage;              // The voltage of robot 48V power
  //   arm_canbus_status = aubo_robot.robotDiagnosis.armCanbusStatus;              // 0x00: No error 0xff:CAN bus error
  //
  //   soft_emergency = aubo_robot.robotDiagnosis.softEmergency;                   // Robot soft emergency
  //   remote_emergency = aubo_robot.robotDiagnosis.remoteEmergency;               // Remote emergency signal
  //
  //   robot_collision = aubo_robot.robotDiagnosis.robotCollision;                 // Collision detection flag
  //
  //   force_control_mode = aubo_robot.robotDiagnosis.forceControlMode;            // Force Control Mode flag
  //   brake_status = aubo_robot.robotDiagnosis.brakeStuats;                       // Brake status
  //
  //   singularity_overspeed_alarm = aubo_robot.robotDiagnosis.singularityOverSpeedAlarm;  // The overspeed alarm of robot singularity
  //   robot_current_alarm = aubo_robot.robotDiagnosis.robotCurrentAlarm;                  // The alarm of robot current flow
  //
  //   can_buffer_size = aubo_robot.robotDiagnosis.macTargetPosBufferSize;         // The maximum size of the CANbus buffer
  //   can_data_size = aubo_robot.robotDiagnosis.macTargetPosDataSize;             // The current data size of the CANbus buffer
  //   can_data_warning = aubo_robot.robotDiagnosis.macDataInterruptWarning;       // The CANbus buffer data interruption
  //
  //   ROS_ERROR_COND(arm_canbus_status != 0x00, "Arm CAN bus Error: 0x%.2X", arm_canbus_status);
  //
  //   ROS_ERROR_COND(soft_emergency, "Soft Emergency.");
  //   ROS_FATAL_COND(remote_emergency, "Remote Emergency!");
  //
  //   ROS_FATAL_COND(robot_collision, "Robot collision!");
  //
  //   ROS_INFO_COND(force_control_mode, "Force Control mode enabled.");
  //   ROS_WARN_COND(brake_status, "Brake active.");
  //
  //   ROS_FATAL_COND(singularity_overspeed_alarm, "Singularity Overspeed!");
  //   ROS_FATAL_COND(robot_current_alarm, "Robot Current Overflow!");
  //
  //   ROS_DEBUG_THROTTLE(0.0, "CAN buffer size: %d", can_buffer_size);
  //   ROS_DEBUG_THROTTLE(0.0, "CAN data size: %d", can_data_size);
  //   ROS_WARN_COND(can_data_warning != 0x00, "CAN data Warining: %d", can_data_warning);
  // }


  void control_loop_cb(const ros::TimerEvent &ev)
  {
    if (aubo_robot.soft_emergency)
    {
      control_loop.stop();
      return;
    }

    if (aubo_robot.collision)
    {
      control_loop.stop();
      return;
    }

    if (aubo_robot.singularity_overspeed)
    {
      control_loop.stop();
      return;
    }

    if (aubo_robot.overcurrent)
    {
      control_loop.stop();
      return;
    }

    const ros::Time time = ev.current_real;
    const ros::Duration period = ev.current_real - ev.last_real;

    read(time, period);
    controller_manager.update(time, period, reset_controllers);
    write(time, period);

    reset_controllers = false;
  }


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

  bool reset(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);


  void read(const ros::Time &time, const ros::Duration &period)
  {
    if (!aubo_robot.read(j_pos))
    {
      reset_controllers = true;
      ROS_ERROR_THROTTLE(1.0, "Failed to read joint positions state from robot!");
    }
  }


  void write(const ros::Time &time, const ros::Duration &period)
  {
    if (!aubo_robot.write(j_pos_cmd))
    {
      reset_controllers = true;
      ROS_ERROR_THROTTLE(1.0, "Failed to write joint positions command to robot!");
    }
  }

};

}  // namespace
#endif
