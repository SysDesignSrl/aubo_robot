// aubo_driver
#include "aubo_driver/aubo_robot.h"
#include "aubo_driver/error_codes.h"


bool aubo::AuboRobot::init()
{
  // Parameters
  if (!node.getParam("joint_names", joint_names))
  {
    ROS_FATAL("Failed to get parameter: '%s'", "joint_names");
    return false;
  }

  collision_class = node.param<int>("aubo/collision_class", 6);
  blend_radius = node.param<double>("aubo/blend_radius", 0.02);

  // Services
  login_srv = node.advertiseService("login", &aubo::AuboRobot::login, this);
  logout_srv = node.advertiseService("logout", &aubo::AuboRobot::logout, this);

  robot_startup_srv = node.advertiseService("robot_startup", &aubo::AuboRobot::robot_startup, this);
  robot_shutdown_srv = node.advertiseService("robot_shutdown", &aubo::AuboRobot::robot_shutdown, this);

  init_profile_srv = node.advertiseService("init_profile", &aubo::AuboRobot::init_profile, this);

  print_diagnostic_srv = node.advertiseService("print_diagnostic_info", &aubo::AuboRobot::print_diagnostic_info, this);

  // Topics
  joint_state_pub = node.advertise<sensor_msgs::JointState>("joint_states", 100);

  // Actions
  joint_trajectory_act.start();
  return true;
}


bool aubo::AuboRobot::login(std::string username, std::string password)
{
  int error_code;

  auto hostname = node.param<std::string>("tcp/hostname", "localhost");
  auto port = node.param<int>("tcp/port", 8899);

  error_code = service_interface.robotServiceLogin(hostname.c_str(), port, username.c_str(), password.c_str());
  if (error_code != 0)
  {
    ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
    ROS_ERROR("Failed to login to %s:%d", hostname.c_str(), port);
    return false;
  }

  ROS_INFO("Logged in to %s:%d", hostname.c_str(), port);
  return true;
}


bool aubo::AuboRobot::login(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
  auto username = node.param<std::string>("login/username", "AUBO");
  auto password = node.param<std::string>("login/password", "123456");

  if (login(username, password))
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
  if (error_code != 0)
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


bool aubo::AuboRobot::robot_startup()
{
  int error_code;

  aubo_robot_namespace::ToolDynamicsParam tool_dynamics_param;
  tool_dynamics_param.positionX = 0.0;
  tool_dynamics_param.positionY = 0.0;
  tool_dynamics_param.positionZ = 0.0;
  tool_dynamics_param.payload = 0.0;
  tool_dynamics_param.toolInertia.xx = 0.0;
  tool_dynamics_param.toolInertia.xy = 0.0;
  tool_dynamics_param.toolInertia.xz = 0.0;
  tool_dynamics_param.toolInertia.yy = 0.0;
  tool_dynamics_param.toolInertia.yz = 0.0;
  tool_dynamics_param.toolInertia.zz = 0.0;

  aubo_robot_namespace::ROBOT_SERVICE_STATE result;

  error_code = service_interface.rootServiceRobotStartup(tool_dynamics_param, collision_class, true, true, 1000, result);
  if (error_code != 0)
  {
    ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
    return false;
  }

  switch (result)
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
  if (error_code != 0)
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
  if (error_code != 0)
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
  if (error_code != 0)
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
  if (error_code != 0)
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
  if (error_code != 0)
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
  if (error_code != 0)
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
  if (error_code != 0)
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
  if (error_code != 0)
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


void aubo::AuboRobot::move_track(const control_msgs::JointTrajectoryGoal::ConstPtr &goal)
{
  int error_code;

  // clear waypoints
  service_interface.robotServiceClearGlobalWayPointVector();

  //
  int n_joints = joint_names.size();

  std::vector<double> j_pos_cmd;
  std::vector<double> j_vel_cmd;
  std::vector<double> j_acc_cmd;

  j_pos_cmd.resize(n_joints);
  j_vel_cmd.resize(n_joints);
  j_acc_cmd.resize(n_joints);

  // add trajectory waypoints
  for (const trajectory_msgs::JointTrajectoryPoint &trajectory_pt: goal->trajectory.points)
  {
    for (int i = 0; i < n_joints; i++)
    {
      for (int j = 0; j < goal->trajectory.joint_names.size(); j++)
      {
        if (joint_names[i] == goal->trajectory.joint_names[j])
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
      joint_trajectory_act.setAborted();
      return;
    }
  }

  // start trajectory execution
  error_code = service_interface.robotServiceTrackMove(aubo_robot_namespace::move_track::JIONT_CUBICSPLINE, true);
  // error_code = service_interface.robotServiceTrackMove(aubo_robot_namespace::move_track::JOINT_UBSPLINEINTP, false);
  if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
    ROS_ERROR("Failed to execute trajectory.");
    joint_trajectory_act.setAborted();
    return;
  }

  ROS_INFO("Trajectory executed succesfully.");
  joint_trajectory_act.setSucceeded();
}


bool aubo::AuboRobot::move_stop()
{
  int error_code;

  error_code = service_interface.rootServiceRobotMoveControl(aubo_robot_namespace::RobotMoveControlCommand::RobotMoveStop);
  if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
    ROS_ERROR("Failed to stop trajectory excution.");
    return false;
  }

  // joint_trajectory_act.setAborted();
  ROS_INFO("Trajectory execution stopped.");
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
    ROS_ERROR("Failed to pause trajectory excution.");
    return false;
  }

  // joint_trajectory_act.setAborted();
  ROS_INFO("Trajectory execution paused.");
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
    ROS_ERROR("Failed to continue trajectory excution.");
    return false;
  }

  // joint_trajectory_act.setAborted();
  ROS_INFO("Trajectory execution continue.");
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
  if (error_code != 0)
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
  if (error_code != 0)
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
    ROS_ERROR("Failed to get Robot Diagnostic Info.");
    return;
  }

  ROS_INFO("AUBO Robot Diagnostic Info:");

  ROS_INFO_COND(robotDiagnosis.armCanbusStatus == 0x00, "\tArm CAN Bus Status: 0x%.2X ", robotDiagnosis.armCanbusStatus);
  ROS_WARN_COND(robotDiagnosis.armCanbusStatus != 0x00, "\tArm CAN Bus Status: 0x%.2X ", robotDiagnosis.armCanbusStatus);

  ROS_INFO("\tArm Power Current: %.1fA", robotDiagnosis.armPowerCurrent);
  ROS_INFO("\tArm Power Voltage: %.1fV", robotDiagnosis.armPowerVoltage);
  ROS_INFO("\tArm Power Status: %s", (robotDiagnosis.armPowerStatus) ? "On" : "Off");

  ROS_INFO("\tController Temperature: %d°", robotDiagnosis.contorllerTemp);
  ROS_INFO("\tController Humidity: %d%%", robotDiagnosis.contorllerHumidity);

  ROS_INFO_COND(!robotDiagnosis.remoteHalt, "\tRemote Halt: %s", "Off");
  ROS_WARN_COND(robotDiagnosis.remoteHalt, "\tRemote Halt: %s", "On");

  ROS_INFO_COND(!robotDiagnosis.softEmergency, "\tSoft Emergency: %s", "Off");
  ROS_WARN_COND(robotDiagnosis.softEmergency, "\tSoft Emergency: %s", "On");

  ROS_INFO_COND(!robotDiagnosis.remoteEmergency, "\tRemote Emergency: %s", "Off");
  ROS_WARN_COND(robotDiagnosis.remoteEmergency, "\tRemote Emergency: %s", "On");

  ROS_INFO_COND(!robotDiagnosis.robotCollision, "\tRobot Collision: %s", "Off");
  ROS_WARN_COND(robotDiagnosis.robotCollision, "\tRobot Collision: %s", "On");

  ROS_INFO_COND(!robotDiagnosis.forceControlMode, "\tForce Control Mode: %s", "Off");
  ROS_WARN_COND(robotDiagnosis.forceControlMode, "\tForce Control Mode: %s", "On");

  ROS_INFO_COND(!robotDiagnosis.brakeStuats, "\tBrake Status: %s", "Off");
  ROS_WARN_COND(robotDiagnosis.brakeStuats, "\tBrake Status: %s", "On");

  ROS_INFO("\tRobot End Speed: %.1f [m/s]", robotDiagnosis.robotEndSpeed);
  ROS_INFO("\tRobot Max Acceleration: %d [m/s^2]", robotDiagnosis.robotMaxAcc);
  ROS_INFO("\tORPE (Software) Status: %s", (robotDiagnosis.orpeStatus) ? "On" : "Off");
  ROS_INFO("\tEnable Read Pose: %s", (robotDiagnosis.enableReadPose) ? "On" : "Off");
  ROS_INFO("\tRobot Mounting Pose Changed: %s", (robotDiagnosis.robotMountingPoseChanged) ? "On" : "Off");

  ROS_INFO_COND(!robotDiagnosis.encoderErrorStatus, "\tEncoder Error Status: %s", "Off");
  ROS_ERROR_COND(robotDiagnosis.encoderErrorStatus, "\tEncoder Error Status: %s", "On");

  ROS_INFO_COND(!robotDiagnosis.staticCollisionDetect, "\tStatic Collision Detect: %s", "Off");
  ROS_WARN_COND(robotDiagnosis.staticCollisionDetect, "\tStatic Collision Detect: %s", "On");

  ROS_INFO_COND(robotDiagnosis.jointCollisionDetect == 0x00, "\tJoint Collision Detect: 0x%.2x", robotDiagnosis.jointCollisionDetect);
  ROS_WARN_COND(robotDiagnosis.jointCollisionDetect != 0x00, "\tJoint Collision Detect: 0x%.2x", robotDiagnosis.jointCollisionDetect);

  ROS_INFO_COND(!robotDiagnosis.encoderLinesError, "\tEncoder Lines Error: %s", "Off");
  ROS_ERROR_COND(robotDiagnosis.encoderLinesError, "\tEncoder Lines Error: %s", "On");

  ROS_INFO_COND(!robotDiagnosis.jointErrorStatus, "\tJoint Error Status: %s", "Off");
  ROS_ERROR_COND(robotDiagnosis.jointErrorStatus, "\tJoint Error Status: %s", "On");

  ROS_INFO_COND(!robotDiagnosis.singularityOverSpeedAlarm, "\tSingularity Overspeed Alarm: %s", "Off");
  ROS_WARN_COND(robotDiagnosis.singularityOverSpeedAlarm, "\tSingularity Overspeed Alarm: %s", "On");

  ROS_INFO_COND(!robotDiagnosis.robotCurrentAlarm, "\tRobot Current Alarm: %s", "Off");
  ROS_WARN_COND(robotDiagnosis.robotCurrentAlarm, "\tRobot Current Alarm: %s", "On");

  ROS_INFO_COND(!robotDiagnosis.toolIoError, "\tTool IO Error: %s", "Off");
  ROS_ERROR_COND(robotDiagnosis.toolIoError, "\tTool IO Error: %s", "On");

  ROS_INFO_COND(!robotDiagnosis.robotMountingPoseWarning, "\tRobot Mounting Pose Warning: %s", "Off");
  ROS_WARN_COND(robotDiagnosis.robotMountingPoseWarning, "\tRobot Mounting Pose Warning: %s", "On");

  ROS_INFO("\tMAC Target Pos Buffer Size: %d", robotDiagnosis.macTargetPosBufferSize);
  ROS_INFO("\tMAC Target Pos Data Size: %d", robotDiagnosis.macTargetPosDataSize);

  ROS_INFO_COND(robotDiagnosis.macDataInterruptWarning == 0x00, "\tMAC Data Interrupt Warning: 0x%.2x", robotDiagnosis.macDataInterruptWarning);
  ROS_WARN_COND(robotDiagnosis.macDataInterruptWarning != 0x00, "\tMAC Data Interrupt Warning: 0x%.2x", robotDiagnosis.macDataInterruptWarning);
}


bool aubo::AuboRobot::print_diagnostic_info(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
  print_diagnostic_info();
  return true;
}


bool aubo::AuboRobot::register_waipoint_callback(RealTimeRoadPointCallback ptr, void *arg)
{
  int error_code;

  error_code = service_interface.robotServiceRegisterRealTimeRoadPointCallback(ptr, arg);
  if (error_code != 0)
  {
    ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
    return false;
  }

  return true;
}


bool aubo::AuboRobot::register_event_info(RobotEventCallback ptr, void *arg)
{
  int error_code;

  error_code = service_interface.robotServiceRegisterRobotEventInfoCallback(ptr, arg);
  if (error_code != 0)
  {
    ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
    return false;
  }

  return true;
}


void aubo::AuboRobot::real_time_waypoint_callback(const aubo_robot_namespace::wayPoint_S *wayPoint, void *arg)
{
  std::vector<double> joint_pos(aubo_robot_namespace::ARM_DOF);
  std::copy(wayPoint->jointpos, wayPoint->jointpos + aubo_robot_namespace::ARM_DOF, joint_pos.begin());

  sensor_msgs::JointState joint_state;
  joint_state.header.stamp = ros::Time::now();
  joint_state.name = joint_names;
  joint_state.position = joint_pos;
  joint_state_pub.publish(joint_state);
}
