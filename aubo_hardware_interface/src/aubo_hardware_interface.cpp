#include "aubo_hardware_interface/aubo_hardware_interface.h"


bool aubo_hardware_interface::AuboHW::init_robot()
{
  std::vector<double> max_joint_acc;
  if (node.getParam("aubo/max_joint_acceleration", max_joint_acc))
  {
    robot.set_max_joint_acceleration(max_joint_acc);
  }

  std::vector<double> max_joint_vel;
  if (node.getParam("aubo/max_joint_velocity", max_joint_vel))
  {
    robot.set_max_joint_velocity(max_joint_vel);
  }

  robot.get_max_joint_acceleration(max_joint_acc);
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

  robot.get_max_joint_velocity(max_joint_vel);
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


bool aubo_hardware_interface::AuboHW::login(std::string host,  unsigned int port = 8899)
{
  // Login
  if (robot.login(host, port))
  {
    node.setParam("connected", true);
    ROS_INFO("Connected to %s:%d", host.c_str(), port);
  }
  else
  {
    ROS_ERROR("Failed to connect to %s:%d", host.c_str(), port);
    return false;
  }

  if (!robot.register_event_info())
  {
    ROS_ERROR("Failed to register to robot events");
    return false;
  }

  if (!init_robot())
  {
    return false;
  }

  print_diagnostic_info();
  return true;
}


bool aubo_hardware_interface::AuboHW::login(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
  auto host = node.param<std::string>("tcp/host", "localhost");
  auto port = node.param<int>("tcp/port", 8899);

  if (login(host, port))
  {
    res.success = true;
    res.message = "Logged in successfully.";
  }
  else
  {
    res.success = false;
    res.message = "Failed to log in.";
  }

  return true;
}


bool aubo_hardware_interface::AuboHW::logout()
{
  if (robot.logout())
  {
    ROS_INFO("Logged out.");
    node.setParam("connected", false);
    return true;
  }
  else
  {
    ROS_ERROR("Failed to log out.");
    return false;
  }
}


bool aubo_hardware_interface::AuboHW::logout(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
  if (logout())
  {
    res.success = true;
    res.message = "Logged out.";
  }
  else
  {
    res.success = false;
    res.message = "Failed to log out.";
  }

  return true;
}


bool aubo_hardware_interface::AuboHW::robot_startup()
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

  if (robot.robot_startup(tool_dynamics, collision_class))
  {
    ROS_INFO("Robot started up with collision class: %d", collision_class);
    print_diagnostic_info();
  }
  else
  {
    ROS_ERROR("Failed to startup the Robot.");
    print_diagnostic_info();
    return false;
  }

  if (robot.enable_tcp_canbus_mode())
  {
    ROS_INFO("Enabled TCP 2 CANbus Mode.");
  }
  else
  {
    ROS_ERROR("Failed to enable TCP 2 CANbus Mode.");
  }

  reset_controllers = true;
  start();

  return true;
}


bool aubo_hardware_interface::AuboHW::robot_startup(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
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


bool aubo_hardware_interface::AuboHW::robot_shutdown()
{
  stop();
  reset_controllers = true;

  if (robot.disable_tcp_canbus_mode())
  {
    ROS_INFO("Disabled TCP 2 CANbus Mode.");
  }
  else
  {
    ROS_ERROR("Failed to disable TCP 2 CANbus Mode.");
  }

  if (robot.robot_shutdown())
  {
    ROS_INFO("Robot shutted down correctly.");
    print_diagnostic_info();
  }
  else
  {
    ROS_ERROR("Failed to shutdown the Robot.");
    print_diagnostic_info();
    return false;
  }

  return true;
}


bool aubo_hardware_interface::AuboHW::robot_shutdown(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
  if (robot_shutdown())
  {
    res.success = true;
    res.message = "Robot shutted down successfully.";
  }
  else
  {
    res.success = false;
    res.message = "Failed to shutdown the Robot.";
  }

  return true;
}


void aubo_hardware_interface::AuboHW::print_diagnostic_info()
{
  if (!robot.get_robot_diagnostic_info())
  {
    ROS_ERROR("Failed to retrieve diagnostic info from the robot!");
  }

  robot_diagnostic.arm_power_status = robot.robotDiagnosis.armPowerStatus;
  robot_diagnostic.arm_power_current = robot.robotDiagnosis.armPowerCurrent;
  robot_diagnostic.arm_power_voltage = robot.robotDiagnosis.armPowerVoltage;
  robot_diagnostic.arm_canbus_status = robot.robotDiagnosis.armCanbusStatus;

  robot_diagnostic.remote_halt = robot.robotDiagnosis.remoteHalt;
  robot_diagnostic.soft_emergency = robot.robotDiagnosis.softEmergency;
  robot_diagnostic.remote_emergency = robot.robotDiagnosis.remoteEmergency;

  robot_diagnostic.robot_collision = robot.robotDiagnosis.robotCollision;
  robot_diagnostic.static_collision = robot.robotDiagnosis.staticCollisionDetect;
  robot_diagnostic.joint_collision = robot.robotDiagnosis.jointCollisionDetect;

  robot_diagnostic.force_control_mode = robot.robotDiagnosis.forceControlMode;
  robot_diagnostic.brake_status = robot.robotDiagnosis.brakeStuats;
  robot_diagnostic.orpe_status = robot.robotDiagnosis.orpeStatus;

  robot_diagnostic.encoder_error = robot.robotDiagnosis.encoderErrorStatus;
  robot_diagnostic.encoder_lines_error = robot.robotDiagnosis.encoderLinesError;
  robot_diagnostic.joint_error = robot.robotDiagnosis.jointErrorStatus;
  robot_diagnostic.tool_io_error = robot.robotDiagnosis.toolIoError;

  robot_diagnostic.singularity_overspeed = robot.robotDiagnosis.singularityOverSpeedAlarm;
  robot_diagnostic.robot_overcurrent = robot.robotDiagnosis.robotCurrentAlarm;

  robot_diagnostic.robot_mounting_pose_warning = robot.robotDiagnosis.robotMountingPoseWarning;

  robot_diagnostic.can_buffer_size = robot.robotDiagnosis.macTargetPosBufferSize;
  robot_diagnostic.can_data_size = robot.robotDiagnosis.macTargetPosDataSize;
  robot_diagnostic.can_data_warning = robot.robotDiagnosis.macDataInterruptWarning;

  ROS_INFO("Arm Power Status: %s", (robot_diagnostic.arm_power_status) ? "ON" : "OFF");
  ROS_INFO("Arm Power Current: %.1f", robot_diagnostic.arm_power_current);
  ROS_INFO("Arm Power Voltage: %.1f", robot_diagnostic.arm_power_voltage);
  ROS_ERROR_COND(robot_diagnostic.arm_canbus_status != 0x00, "Arm CAN bus Error: 0x%.2X", robot_diagnostic.arm_canbus_status);

  ROS_WARN_COND(robot_diagnostic.remote_halt, "Remote Halt.");
  ROS_ERROR_COND(robot_diagnostic.soft_emergency, "Soft Emergency!");
  ROS_ERROR_COND(robot_diagnostic.remote_emergency, "Remote Emergency!");

  ROS_ERROR_COND(robot_diagnostic.robot_collision, "Robot collision detected!");
  ROS_ERROR_COND(robot_diagnostic.static_collision, "Static collision detected!");
  ROS_ERROR_COND(robot_diagnostic.joint_collision != 0x00, "Joint collision: 0x%.2X", robot_diagnostic.joint_collision);

  ROS_INFO_COND(robot_diagnostic.force_control_mode, "Force Control mode enabled.");
  ROS_WARN_COND(robot_diagnostic.brake_status, "Brake active.");
  ROS_INFO_COND(robot_diagnostic.orpe_status, "ORPE active.");

  ROS_FATAL_COND(robot_diagnostic.encoder_error, "Encoder Error!");
  ROS_FATAL_COND(robot_diagnostic.encoder_lines_error, "Encoder Lines Error!");
  ROS_FATAL_COND(robot_diagnostic.joint_error, "Joint Error!");
  ROS_FATAL_COND(robot_diagnostic.tool_io_error, "Tool IO Error!");

  ROS_ERROR_COND(robot_diagnostic.singularity_overspeed, "Singularity Overspeed!");
  ROS_ERROR_COND(robot_diagnostic.robot_overcurrent, "Robot OverCurrent!");

  ROS_WARN_COND(robot_diagnostic.robot_mounting_pose_warning, "Mounting Pose Warning.");

  ROS_DEBUG_THROTTLE(0.0, "CAN buffer size: %d", robot_diagnostic.can_buffer_size);
  ROS_DEBUG_THROTTLE(0.0, "CAN data size: %d", robot_diagnostic.can_data_size);
  ROS_WARN_COND(robot_diagnostic.can_data_warning != 0x00, "CAN data Warining: %d", robot_diagnostic.can_data_warning);
}


bool aubo_hardware_interface::AuboHW::print_diagnostic_info(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
  print_diagnostic_info();
  return true;
}
