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
// aubo_hardware_interface
#include "aubo_hardware_interface/aubo_robot.h"


namespace aubo_hardware_interface {

class AuboHW : public hardware_interface::RobotHW {
private:
  aubo::AuboRobot aubo_robot;

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

  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;

  std::vector<double> j_pos, j_pos_cmd;
  std::vector<double> j_vel, j_vel_cmd;
  std::vector<double> j_eff, j_eff_cmd;

public:

  AuboHW(const ros::NodeHandle &node = ros::NodeHandle()) : node(node)
  {
    ros::Duration period(1.0);
    refresh_cycle = node.createTimer(period, &aubo_hardware_interface::AuboHW::refresh_cycle_cb, this, false, false);
  }


  void refresh_cycle_cb(const ros::TimerEvent &ev)
  {
    if (!aubo_robot.get_robot_diagnostic_info())
    {
      ROS_WARN("Failed to retrieve diagnostic info from the robot!");
    }

    arm_power_status = aubo_robot.robotDiagnosis.armPowerStatus;                // The switch status (on, off)of robot 48V power
    arm_power_current = aubo_robot.robotDiagnosis.armPowerCurrent;              // The current of robot 48V power
    arm_power_voltage = aubo_robot.robotDiagnosis.armPowerVoltage;              // The voltage of robot 48V power
    arm_canbus_status = aubo_robot.robotDiagnosis.armCanbusStatus;              // 0x00: No error 0xff:CAN bus error

    soft_emergency = aubo_robot.robotDiagnosis.softEmergency;                   // Robot soft emergency
    remote_emergency = aubo_robot.robotDiagnosis.remoteEmergency;               // Remote emergency signal

    robot_collision = aubo_robot.robotDiagnosis.robotCollision;                 // Collision detection flag

    force_control_mode = aubo_robot.robotDiagnosis.forceControlMode;            // Force Control Mode flag
    brake_status = aubo_robot.robotDiagnosis.brakeStuats;                       // Brake status

    singularity_overspeed_alarm = aubo_robot.robotDiagnosis.singularityOverSpeedAlarm;  // The overspeed alarm of robot singularity
    robot_current_alarm = aubo_robot.robotDiagnosis.robotCurrentAlarm;                  // The alarm of robot current flow

    can_buffer_size = aubo_robot.robotDiagnosis.macTargetPosBufferSize;         // The maximum size of the CANbus buffer
    can_data_size = aubo_robot.robotDiagnosis.macTargetPosDataSize;             // The current data size of the CANbus buffer
    can_data_warning = aubo_robot.robotDiagnosis.macDataInterruptWarning;       // The CANbus buffer data interruption

    ROS_ERROR_COND(arm_canbus_status != 0x00, "Arm CAN bus Error: 0x%.2X", arm_canbus_status);

    ROS_ERROR_COND(soft_emergency, "Soft Emergency.");
    ROS_FATAL_COND(remote_emergency, "Remote Emergency!");

    ROS_FATAL_COND(robot_collision, "Robot collision!");

    ROS_INFO_COND(force_control_mode, "Force Control mode enabled.");
    ROS_WARN_COND(brake_status, "Brake active.");

    ROS_FATAL_COND(singularity_overspeed_alarm, "Singularity Overspeed!");
    ROS_FATAL_COND(robot_current_alarm, "Robot Current Overflow!");

    ROS_DEBUG_THROTTLE(0.0, "CAN buffer size: %d", can_buffer_size);
    ROS_DEBUG_THROTTLE(0.0, "CAN data size: %d", can_data_size);
    ROS_WARN_COND(can_data_warning != 0x00, "CAN data Warining: %d", can_data_warning);
  }


  bool init(const std::vector<std::string> &joints)
  {
    const int n_joints = joints.size();

    j_pos.resize(n_joints, 0.0); j_pos_cmd.resize(n_joints, 0.0);
    j_vel.resize(n_joints, 0.0); j_vel_cmd.resize(n_joints, 0.0);
    j_eff.resize(n_joints, 0.0); j_eff_cmd.resize(n_joints, 0.0);

    for (int i=0; i < n_joints; i++)
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


  bool init_robot()
  {
    std::vector<double> max_joint_acc;
    if (node.getParam("aubo/max_joint_acceleration", max_joint_acc))
    {
      aubo_robot.set_max_joint_acceleration(max_joint_acc);
    }

    std::vector<double> max_joint_vel;
    if (node.getParam("aubo/max_joint_velocity", max_joint_vel))
    {
      aubo_robot.set_max_joint_velocity(max_joint_vel);
    }

    aubo_robot.get_max_joint_acceleration(max_joint_acc);
    {
      std::stringstream ss;
      ss << "[ ";
      for (double val : max_joint_acc)
      {
        ss << val << " ";
      }
      ss << "] ";
      ROS_DEBUG_STREAM("max joint acceleration: " << ss.str() << "[rad/s^2]");
    }

    aubo_robot.get_max_joint_velocity(max_joint_vel);
    {
      std::stringstream ss;
      ss << "[ ";
      for (double val : max_joint_vel)
      {
        ss << val << " ";
      }
      ss << "] ";
      ROS_DEBUG_STREAM("max joint velocity: " << ss.str() << "[rad/s]");
    }

    return true;
  }


  bool start(std::string host,  unsigned int port)
  {
    // Login
    if (aubo_robot.login(host, port))
    {
      node.setParam("robot_connected", true);
      ROS_INFO("Connected to %s:%d", host.c_str(), port);
    }
    else
    {
      node.setParam("robot_connected", false);
      ROS_ERROR("Failed to connect to %s:%d", host.c_str(), port);
      return false;
    }

    if (!aubo_robot.register_event_info())
    {
      ROS_ERROR("Failed to register to robot events");
      return false;
    }

    if (!init_robot())
    {
      return false;
    }

    if (!robot_startup())
    {
      return false;
    }

    if (aubo_robot.enable_tcp_canbus_mode())
    {
      ROS_INFO("Enabled TCP 2 CANbus Mode.");
    }
    else
    {
      ROS_ERROR("Failed to enable TCP 2 CANbus Mode.");
      return false;
    }

    return true;
  }


  bool robot_startup()
  {
    XmlRpc::XmlRpcValue tool_dynamics;
    if (!node.getParam("aubo/tool_dynamics", tool_dynamics))
    {
      std::string param_name = node.resolveName("aubo/tool_dynamics");
      ROS_WARN("Failed to retrieve '%s' parameter.", param_name.c_str());
      return false;
    }

    int collision_class;
    if (!node.getParam("aubo/collision_class", collision_class))
    {
      std::string param_name = node.resolveName("aubo/collision_class");
      ROS_WARN("Failed to retrieve '%s' parameter.", param_name.c_str());
    }

    if (aubo_robot.robot_startup(tool_dynamics, collision_class))
    {
      ROS_INFO("Robot started up with collision class: %d", collision_class);
    }
    else
    {
      ROS_ERROR("Failed to startup the Robot.");
      return false;
    }

    return true;
  }


  bool robot_startup(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
  {
    if (robot_startup())
    {
      res.success = true;
      res.message = "Robot started up successfully.";
    }
    else
    {
      res.success = false;
      res.message = "Failed to startup the Robot.";
    }

    return true;
  }


  bool stop()
  {
    refresh_cycle.stop();

    // TCP 2 CANbus
    if (!aubo_robot.disable_tcp_canbus_mode())
    {
      ROS_ERROR("Failed to disable TCP 2 CANbus Mode.");
      return false;
    }

    ROS_INFO("Disabled TCP 2 CANbus Mode.");

    // Shutdown
    if (!aubo_robot.robot_shutdown())
    {
      ROS_ERROR("Failed to shutdown the Robot.");
      return false;
    }

    ROS_INFO("Robot shutted down correctly.");

    // Logout
    if (!aubo_robot.logout())
    {
      ROS_ERROR("Failed to log out.");
      return false;
    }

    ROS_INFO("Logged out.");

    node.setParam("robot_connected", false);
    return true;
  }


  void read(const ros::Time &time, const ros::Duration &period)
  {
    if (!aubo_robot.read(j_pos))
    {
      ROS_ERROR_THROTTLE(1.0, "Failed to read joint positions from robot!");
    }
  }


  void write(const ros::Time &time, const ros::Duration &period)
  {
    //
    if (std::all_of(j_pos_cmd.cbegin(), j_pos_cmd.cend(), [](double value) { return value == 0.0; }))
    {
      return;
    }

    // Don't send trajectoy command if it's equal to the current joint state to
    // not overload CANbus buffer.
    // if (std::equal(j_pos_cmd.cbegin(), j_pos_cmd.cend(), j_pos_cmd_1.cbegin()))
    // {
    //   return;
    // }
    //
    // std::copy(j_pos_cmd.cbegin(), j_pos_cmd.cend(), j_pos_cmd_1.begin());


    if (!aubo_robot.write(j_pos_cmd))
    {
      ROS_ERROR_THROTTLE(1.0, "Failed to write joint positions command to robot!");
    }
  }

};

}  // namespace
#endif
