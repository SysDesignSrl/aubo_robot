// aubo_driver
#include "aubo_driver/aubo_robot.h"
#include "aubo_driver/error_codes.h"


bool aubo::AuboRobot::login(std::string host, int port, std::string username, std::string password)
{
  int error_code;

  error_code = service_interface.robotServiceLogin(host.c_str(), port, username.c_str(), password.c_str());
  if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
    return false;
  }

  return true;
}


bool aubo::AuboRobot::login(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
  auto host = node.param<std::string>("tcp/host", "localhost");
  auto port = node.param<int>("tcp/port", 8899);

  auto username = node.param<std::string>("login/username", "AUBO");
  auto password = node.param<std::string>("login/password", "123456");

  if (login(host, port, username, password))
  {
    res.success = true;
    res.message = "Logged in.";
  }
  else
  {
    res.success = false;
    res.message = "Failed to log in.";
  }

  return true;
}


bool aubo::AuboRobot::logout()
{
  int error_code;

  error_code = service_interface.robotServiceLogout();
  if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
    return false;
  }

  return true;
}


bool aubo::AuboRobot::logout(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
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


bool aubo::AuboRobot::robot_startup(int collision_class)
{
  int error_code;

  XmlRpc::XmlRpcValue tool_dynamics;
  if (!node.getParam("aubo/tool_dynamics", tool_dynamics))
  {
    std::string param_name = node.resolveName("aubo/tool_dynamics");
    ROS_WARN("Failed to retrieve '%s' parameter.", param_name.c_str());
    return false;
  }

  aubo_robot_namespace::ToolDynamicsParam tool_dynamics_param;
  try
  {
    tool_dynamics_param.positionX = tool_dynamics["position"]["x"];
    tool_dynamics_param.positionY = tool_dynamics["position"]["y"];
    tool_dynamics_param.positionZ = tool_dynamics["position"]["z"];
    tool_dynamics_param.payload = tool_dynamics["payload"];
    tool_dynamics_param.toolInertia.xx = tool_dynamics["inertia"]["xx"];
    tool_dynamics_param.toolInertia.xy = tool_dynamics["inertia"]["xy"];
    tool_dynamics_param.toolInertia.xz = tool_dynamics["inertia"]["xz"];
    tool_dynamics_param.toolInertia.yy = tool_dynamics["inertia"]["yy"];
    tool_dynamics_param.toolInertia.yz = tool_dynamics["inertia"]["yz"];
    tool_dynamics_param.toolInertia.zz = tool_dynamics["inertia"]["zz"];
  }
  catch (const XmlRpc::XmlRpcException &ex)
  {
    auto code = ex.getCode();
    auto message = ex.getMessage();
    ROS_ERROR("Error %d, %s", code, message.c_str());
    return false;
  }

  aubo_robot_namespace::ROBOT_SERVICE_STATE robot_state;

  error_code = service_interface.rootServiceRobotStartup(tool_dynamics_param, collision_class, true, true, 1000, robot_state);
  if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
    return false;
  }

  switch (robot_state)
  {
    case aubo_robot_namespace::ROBOT_SERVICE_STATE::ROBOT_SERVICE_READY:
      ROS_INFO("ROBOT SERVICE STATE: %s", "READY");
      break;
    case aubo_robot_namespace::ROBOT_SERVICE_STATE::ROBOT_SERVICE_STARTING:
      ROS_INFO("ROBOT SERVICE STATE: %s", "STARTING");
      break;
    case aubo_robot_namespace::ROBOT_SERVICE_STATE::ROBOT_SERVICE_WORKING:
      ROS_INFO("ROBOT SERVICE STATE: %s", "WORKING");
      break;
    case aubo_robot_namespace::ROBOT_SERVICE_STATE::ROBOT_SERVICE_CLOSING:
      ROS_INFO("ROBOT SERVICE STATE: %s", "CLOSING");
      break;
    case aubo_robot_namespace::ROBOT_SERVICE_STATE::ROBOT_SERVICE_CLOSED:
      ROS_INFO("ROBOT SERVICE STATE: %s", "CLOSED");
      break;
    case aubo_robot_namespace::ROBOT_SERVICE_STATE::ROBOT_SETVICE_FAULT_POWER:
      ROS_ERROR("ROBOT SERVICE STATE: %s", "FAULT POWER");
      break;
    case aubo_robot_namespace::ROBOT_SERVICE_STATE::ROBOT_SETVICE_FAULT_BRAKE:
      ROS_ERROR("ROBOT SERVICE STATE: %s", "FAULT BRAKE");
      break;
    case aubo_robot_namespace::ROBOT_SERVICE_STATE::ROBOT_SETVICE_FAULT_NO_ROBOT:
      ROS_ERROR("ROBOT SERVICE STATE: %s", "FAULT NO ROBOT");
      break;
  }

  return true;
}


bool aubo::AuboRobot::robot_startup(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
  if (robot_startup())
  {
    res.success = true;
    res.message = "Robot startup correctly.";
  }
  else
  {
    res.success = false;
    res.message = "Failed to startup the Robot.";
  }

  return true;
}


bool aubo::AuboRobot::robot_shutdown()
{
  int error_code;

  error_code = service_interface.robotServiceRobotShutdown();
  if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
    return false;
  }

  return true;
}


bool aubo::AuboRobot::robot_shutdown(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
  if (robot_shutdown())
  {
    res.success = true;
    res.message = "Robot shutdown correctly.";
  }
  else
  {
    res.success = false;
    res.message = "Failed to shutdown the Robot.";
  }

  return true;
}


bool aubo::AuboRobot::init_profile()
{
  int error_code;

  error_code = service_interface.robotServiceInitGlobalMoveProfile();
  if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
    return false;
  }

  return true;
}


bool aubo::AuboRobot::init_profile(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
  if (init_profile())
  {
    res.success = true;
    res.message = "Movement profile initialized correctly.";
  }
  else
  {
    res.success = false;
    res.message = "Failed to initialize movement profile.";
  }

  return true;
}


bool aubo::AuboRobot::set_max_joint_acceleration(const std::vector<double> &value)
{
  int error_code;

  aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
  std::copy(value.begin(), value.end(), jointMaxAcc.jointPara);

  error_code = service_interface.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);
  if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
    return false;
  }

  return true;
}


void aubo::AuboRobot::get_max_joint_acceleration(std::vector<double> &result)
{
  aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
  service_interface.robotServiceGetGlobalMoveJointMaxAcc(jointMaxAcc);

  result.resize(aubo_robot_namespace::ARM_DOF);
  std::copy(jointMaxAcc.jointPara, jointMaxAcc.jointPara + aubo_robot_namespace::ARM_DOF, result.begin());
}


bool aubo::AuboRobot::set_max_joint_velocity(const std::vector<double> &value)
{
  int error_code;

  aubo_robot_namespace::JointVelcAccParam jointMaxVel;
  std::copy(value.begin(), value.end(), jointMaxVel.jointPara);

  error_code = service_interface.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVel);
  if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
    return false;
  }

  return true;
}


void aubo::AuboRobot::get_max_joint_velocity(std::vector<double> &result)
{
  aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
  service_interface.robotServiceGetGlobalMoveJointMaxVelc(jointMaxAcc);

  result.resize(aubo_robot_namespace::ARM_DOF);
  std::copy(jointMaxAcc.jointPara, jointMaxAcc.jointPara + aubo_robot_namespace::ARM_DOF, result.begin());
}


bool aubo::AuboRobot::set_max_linear_acceleration(double value)
{
  int error_code;

  error_code = service_interface.robotServiceSetGlobalMoveEndMaxLineAcc(value);
  if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
    return false;
  }

  return true;
}


void aubo::AuboRobot::get_max_linear_acceleration(double &result)
{
  service_interface.robotServiceGetGlobalMoveEndMaxLineAcc(result);
}


bool aubo::AuboRobot::set_max_linear_velocity(double value)
{
  int error_code;

  error_code = service_interface.robotServiceSetGlobalMoveEndMaxLineVelc(value);
  if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
    return false;
  }

  return true;
}


void aubo::AuboRobot::get_max_linear_velocity(double &result)
{
  service_interface.robotServiceGetGlobalMoveEndMaxLineVelc(result);
}


bool aubo::AuboRobot::set_max_angular_acceleration(double value)
{
  int error_code;

  error_code = service_interface.robotServiceSetGlobalMoveEndMaxAngleAcc(value);
  if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
    return false;
  }

  return true;
}


void aubo::AuboRobot::get_max_angular_acceleration(double &result)
{
  service_interface.robotServiceGetGlobalMoveEndMaxAngleAcc(result);
}


bool aubo::AuboRobot::set_max_angular_velocity(double value)
{
  int error_code;

  error_code = service_interface.robotServiceSetGlobalMoveEndMaxAngleVelc(value);
  if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
    return false;
  }

  return true;
}


void aubo::AuboRobot::get_max_angular_velocity(double &result)
{
  service_interface.robotServiceGetGlobalMoveEndMaxAngleVelc(result);
}


bool aubo::AuboRobot::set_blend_radius(double value)
{
  int error_code;

  error_code = service_interface.robotServiceSetGlobalBlendRadius(value);
  if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
    return false;
  }

  return true;
}


double aubo::AuboRobot::get_blend_radius()
{
  return service_interface.robotServiceGetGlobalBlendRadius();
}


bool aubo::AuboRobot::move_joint(const std::vector<double> &joint_pos)
{
  int error_code;

  double jointAngle[aubo_robot_namespace::ARM_DOF];
  std::copy(joint_pos.cbegin(), joint_pos.cend(), jointAngle);

  error_code = service_interface.robotServiceJointMove(jointAngle, false);
  if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
    return false;
  }

  return true;
}


bool aubo::AuboRobot::move_line(const std::vector<double> &joint_pos)
{
  int error_code;

  double jointAngle[aubo_robot_namespace::ARM_DOF];
  std::copy(joint_pos.cbegin(), joint_pos.cend(), jointAngle);

  error_code = service_interface.robotServiceLineMove(jointAngle, false);
  if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
    return false;
  }

  return true;
}


bool aubo::AuboRobot::move_track(const trajectory_msgs::JointTrajectory &trajectory)
{
  int error_code;

  // clear waypoints
  service_interface.robotServiceClearGlobalWayPointVector();

  int n_joints = joint_names.size();

  std::vector<double> j_pos_cmd;
  std::vector<double> j_vel_cmd;
  std::vector<double> j_acc_cmd;

  j_pos_cmd.resize(n_joints);
  j_vel_cmd.resize(n_joints);
  j_acc_cmd.resize(n_joints);

  // add trajectory waypoints
  for (const trajectory_msgs::JointTrajectoryPoint &trajectory_pt: trajectory.points)
  {
    for (int i = 0; i < n_joints; i++)
    {
      for (int j = 0; j < trajectory.joint_names.size(); j++)
      {
        if (joint_names[i] == trajectory.joint_names[j])
        {
          j_pos_cmd[i] = trajectory_pt.positions[j];
          j_vel_cmd[i] = trajectory_pt.velocities[j];
          j_acc_cmd[i] = trajectory_pt.accelerations[j];
        }
      }
    }

    error_code = service_interface.robotServiceAddGlobalWayPoint(j_pos_cmd.data());
    if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
    {
      ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
      ROS_ERROR("Failed to add trajectory waypoint to robot controller.");
      return false;
    }
  }

  // start trajectory execution
  error_code = service_interface.robotServiceTrackMove(aubo_robot_namespace::move_track::JIONT_CUBICSPLINE, true);
  // error_code = service_interface.robotServiceTrackMove(aubo_robot_namespace::move_track::JOINT_UBSPLINEINTP, false);
  if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
    return false;
  }

  return true;
}


bool aubo::AuboRobot::move_stop()
{
  int error_code;

  error_code = service_interface.rootServiceRobotMoveControl(aubo_robot_namespace::RobotMoveControlCommand::RobotMoveStop);
  if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
    return false;
  }

  return true;
}


bool aubo::AuboRobot::move_stop(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
  if (move_stop())
  {
    res.success = true;
    res.message = "Movement stopped.";
  }
  else
  {
    res.success = false;
    res.message = "Failed to stop movement.";
  }

  return true;
}


bool aubo::AuboRobot::move_pause()
{
  int error_code;

  error_code = service_interface.rootServiceRobotMoveControl(aubo_robot_namespace::RobotMoveControlCommand::RobotMovePause);
  if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
    return false;
  }

  return true;
}


bool aubo::AuboRobot::move_pause(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
  if (move_pause())
  {
    res.success = true;
    res.message = "Movement paused.";
  }
  else
  {
    res.success = false;
    res.message = "Failed to pause movement.";
  }

  return true;
}


bool aubo::AuboRobot::move_resume()
{
  int error_code;

  error_code = service_interface.rootServiceRobotMoveControl(aubo_robot_namespace::RobotMoveControlCommand::RobotMoveContinue);
  if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
    return false;
  }

  return true;
}


bool aubo::AuboRobot::move_resume(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
  if (move_resume())
  {
    res.success = true;
    res.message = "Movement resumed.";
  }
  else
  {
    res.success = false;
    res.message = "Failed to resume movement.";
  }

  return true;
}


bool aubo::AuboRobot::get_joint_angle(std::vector<double> &joint_pos)
{
  int error_code;

  aubo_robot_namespace::JointParam jointParam;
  error_code = service_interface.robotServiceGetJointAngleInfo(jointParam);
  if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
    return false;
  }

  joint_pos.resize(aubo_robot_namespace::ARM_DOF);
  std::copy(jointParam.jointPos, jointParam.jointPos + aubo_robot_namespace::ARM_DOF, joint_pos.begin());
  return true;
}


void aubo::AuboRobot::get_current_waypoint()
{
  int error_code;

  aubo_robot_namespace::wayPoint_S wayPoint;
  error_code = service_interface.robotServiceGetCurrentWaypointInfo(wayPoint);
  if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
    ROS_ERROR("Failed to get current waypoint.");
    return;
  }

  std::vector<double> joint_pos(aubo_robot_namespace::ARM_DOF);
  std::copy(wayPoint.jointpos, wayPoint.jointpos + aubo_robot_namespace::ARM_DOF, joint_pos.begin());

  sensor_msgs::JointState joint_state;
  joint_state.header.stamp = ros::Time::now();
  joint_state.name = joint_names;
  joint_state.position = joint_pos;
  joint_state_pub.publish(joint_state);
}


void aubo::AuboRobot::print_diagnostic_info()
{
  int error_code;
  aubo_robot_namespace::RobotDiagnosis robotDiagnosis;

  error_code = service_interface.robotServiceGetRobotDiagnosisInfo(robotDiagnosis);
  if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
    return;
  }

  robot_diagnostic.arm_power_status = robotDiagnosis.armPowerStatus;
  robot_diagnostic.arm_power_current = robotDiagnosis.armPowerCurrent;
  robot_diagnostic.arm_power_voltage = robotDiagnosis.armPowerVoltage;
  robot_diagnostic.arm_canbus_status = robotDiagnosis.armCanbusStatus;

  robot_diagnostic.remote_halt = robotDiagnosis.remoteHalt;
  robot_diagnostic.soft_emergency = robotDiagnosis.softEmergency;
  robot_diagnostic.remote_emergency = robotDiagnosis.remoteEmergency;

  robot_diagnostic.robot_collision = robotDiagnosis.robotCollision;
  robot_diagnostic.static_collision = robotDiagnosis.staticCollisionDetect;
  robot_diagnostic.joint_collision = robotDiagnosis.jointCollisionDetect;

  robot_diagnostic.force_control_mode = robotDiagnosis.forceControlMode;
  robot_diagnostic.brake_status = robotDiagnosis.brakeStuats;
  robot_diagnostic.orpe_status = robotDiagnosis.orpeStatus;

  robot_diagnostic.encoder_error = robotDiagnosis.encoderErrorStatus;
  robot_diagnostic.encoder_lines_error = robotDiagnosis.encoderLinesError;
  robot_diagnostic.joint_error = robotDiagnosis.jointErrorStatus;
  robot_diagnostic.tool_io_error = robotDiagnosis.toolIoError;

  robot_diagnostic.singularity_overspeed = robotDiagnosis.singularityOverSpeedAlarm;
  robot_diagnostic.robot_overcurrent = robotDiagnosis.robotCurrentAlarm;

  robot_diagnostic.robot_mounting_pose_warning = robotDiagnosis.robotMountingPoseWarning;

  robot_diagnostic.can_buffer_size = robotDiagnosis.macTargetPosBufferSize;
  robot_diagnostic.can_data_size = robotDiagnosis.macTargetPosDataSize;
  robot_diagnostic.can_data_warning = robotDiagnosis.macDataInterruptWarning;

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

  ROS_DEBUG("CAN buffer size: %d", robot_diagnostic.can_buffer_size);
  ROS_DEBUG("CAN data size: %d", robot_diagnostic.can_data_size);
  ROS_WARN_COND(robot_diagnostic.can_data_warning != 0x00, "CAN data Warining: %d", robot_diagnostic.can_data_warning);
}


bool aubo::AuboRobot::print_diagnostic_info(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
  print_diagnostic_info();
  return true;
}


bool aubo::AuboRobot::register_realtime_waypoint(RealTimeRoadPointCallback waypoint_cb, void *arg)
{
  int error_code;

  error_code = service_interface.robotServiceRegisterRealTimeRoadPointCallback(waypoint_cb, arg);
  if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
    return false;
  }

  return true;
}


bool aubo::AuboRobot::register_event_info(RobotEventCallback event_cb, void *arg)
{
  int error_code;

  error_code = service_interface.robotServiceRegisterRobotEventInfoCallback(event_cb, arg);
  if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
    return false;
  }

  return true;
}


bool aubo::AuboRobot::get_digital_input(int addr, bool &value)
{
  int error_code;

  auto ioType = aubo_robot_namespace::RobotIoType::RobotBoardUserDI;

  double ioValue;
  error_code = service_interface.robotServiceGetBoardIOStatus(ioType, addr, ioValue);
  if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
    return false;
  }

  value = (ioValue > 0.0) ? true : false;

  return true;
}


bool aubo::AuboRobot::get_digital_input(std::string name, bool &value)
{
  int error_code;

  auto ioType = aubo_robot_namespace::RobotIoType::RobotBoardUserDI;

  double ioValue;
  error_code = service_interface.robotServiceGetBoardIOStatus(ioType, name, ioValue);
  if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
    return false;
  }

  value = (ioValue > 0.0) ? true : false;

  return true;
}


bool aubo::AuboRobot::set_digital_output(int addr, bool value)
{
  int error_code;

  auto ioType = aubo_robot_namespace::RobotIoType::RobotBoardUserDO;

  error_code = service_interface.robotServiceSetBoardIOStatus(ioType, addr, (value) ? 1.0 : 0.0);
  if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
    return false;
  }

  return true;
}


bool aubo::AuboRobot::set_digital_output(std::string name, bool value)
{
  int error_code;

  auto ioType = aubo_robot_namespace::RobotIoType::RobotBoardUserDO;

  error_code = service_interface.robotServiceSetBoardIOStatus(ioType, name, (value) ? 1.0 : 0.0);
  if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
    return false;
  }

  return true;
}


bool aubo::AuboRobot::get_analog_input(int addr, double &value)
{
  int error_code;

  auto ioType = aubo_robot_namespace::RobotIoType::RobotBoardUserAI;

  error_code = service_interface.robotServiceGetBoardIOStatus(ioType, addr, value);
  if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
    return false;
  }

  return true;
}


bool aubo::AuboRobot::get_analog_input(std::string name, double &value)
{
  int error_code;

  auto ioType = aubo_robot_namespace::RobotIoType::RobotBoardUserAI;

  error_code = service_interface.robotServiceGetBoardIOStatus(ioType, name, value);
  if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
    return false;
  }

  return true;
}


bool aubo::AuboRobot::set_analog_output(int addr, double value)
{
  int error_code;

  auto ioType = aubo_robot_namespace::RobotIoType::RobotBoardUserAO;

  error_code = service_interface.robotServiceSetBoardIOStatus(ioType, addr, value);
  if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
    return false;
  }

  return true;
}


bool aubo::AuboRobot::set_analog_output(std::string name, double value)
{
  int error_code;

  auto ioType = aubo_robot_namespace::RobotIoType::RobotBoardUserAO;

  error_code = service_interface.robotServiceSetBoardIOStatus(ioType, name, value);
  if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
    return false;
  }

  return true;
}


bool aubo::AuboRobot::get_digital_inputs(std::vector<bool> &digital_inputs)
{
  int error_code;

  std::vector<aubo_robot_namespace::RobotIoType> ioType;
  std::vector<aubo_robot_namespace::RobotIoDesc> statusVector;

  ioType.push_back(aubo_robot_namespace::RobotIoType::RobotBoardUserDI);

  error_code = service_interface.robotServiceGetBoardIOStatus(ioType, statusVector);
  if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
    return false;
  }

  //
  for (int i = 0; i < statusVector.size(); i++)
  {
    char* name = statusVector[i].ioName;
    int addr = statusVector[i].ioAddr;
    double val = statusVector[i].ioValue;

    ROS_DEBUG("%s, %d: %f", name, addr, val);
    digital_inputs.push_back((val > 0.0) ? true : false);
  }

  return true;
}


bool aubo::AuboRobot::get_analog_inputs(std::vector<double> &analog_inputs)
{
  int error_code;

  std::vector<aubo_robot_namespace::RobotIoType> ioType;
  std::vector<aubo_robot_namespace::RobotIoDesc> statusVector;

  ioType.push_back(aubo_robot_namespace::RobotIoType::RobotBoardUserAI);

  error_code = service_interface.robotServiceGetBoardIOStatus(ioType, statusVector);
  if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
    return false;
  }

  //
  for (int i = 0; i < statusVector.size(); i++)
  {
    char* name = statusVector[i].ioName;
    int addr = statusVector[i].ioAddr;
    double val = statusVector[i].ioValue;

    ROS_DEBUG("%s, %d: %f", name, addr, val);
    analog_inputs.push_back(val);
  }

  return true;
}
