#include "aubo_hardware_interface/aubo_robot.h"


void event_info_cb(const aubo_robot_namespace::RobotEventInfo *eventInfo, void *arg)
{
  aubo::AuboRobot* aubo_robot = (aubo::AuboRobot*)arg;

  auto type = eventInfo->eventType;
  auto code =  eventInfo->eventCode;
  auto message = eventInfo->eventContent;

  switch (type)
  {
    case aubo_robot_namespace::RobotEventType::RobotEvent_armCanbusError:
      ROS_ERROR("Arm CANbus Error: %d,  %s", code, message.c_str());
      break;
    case aubo_robot_namespace::RobotEventType::RobotEvent_remoteHalt:
      ROS_WARN("Remote Halt: %d, %s", code, message.c_str());
      break;
    case aubo_robot_namespace::RobotEventType::RobotEvent_remoteEmergencyStop:
      ROS_FATAL("Remote Emergency Stop: %d, %s", code, message.c_str());
      break;
    case aubo_robot_namespace::RobotEventType::RobotEvent_jointError:
      ROS_ERROR("Joint Error: %d,  %s", code, message.c_str());
      break;

    case aubo_robot_namespace::RobotEventType::RobotEvent_forceControl:
      ROS_INFO("Force Control: %d, %s", code, message.c_str());
      break;
    case aubo_robot_namespace::RobotEventType::RobotEvent_exitForceControl:
      ROS_INFO("Exit Force Control: %d, %s", code, message.c_str());
      break;

    case aubo_robot_namespace::RobotEventType::RobotEvent_softEmergency:
      aubo_robot->soft_emergency = true;
      ROS_ERROR("Soft Emergency: %d, %s", code, message.c_str());
      break;
    case aubo_robot_namespace::RobotEventType::RobotEvent_exitSoftEmergency:
      aubo_robot->soft_emergency = false;
      ROS_INFO("Exit Soft Emergency: %d, %s", code, message.c_str());
      break;

    case aubo_robot_namespace::RobotEventType::RobotEvent_collision:
      aubo_robot->robot_collision = true;
      ROS_FATAL("Collision: %d, %s", code, message.c_str());
      break;
    case aubo_robot_namespace::RobotEventType::RobotEvent_collisionStatusChanged:
      ROS_INFO("Collision Status Changed: %d, %s", code, message.c_str());
      break;
    case aubo_robot_namespace::RobotEventType::RobotEvent_tcpParametersSucc:
      ROS_INFO("Tool dynamic parameters Success: %d, %s", code, message.c_str());
      break;
    case aubo_robot_namespace::RobotEventType::RobotEvent_ArmPowerOff:
      ROS_WARN("Arm Power Off: %d, %s", code, message.c_str());
      break;

    case aubo_robot_namespace::RobotEventType::RobotEvent_singularityOverspeed:
      aubo_robot->singularity_overspeed = true;
      ROS_ERROR("Event Code: %d, Singularity Overspeed!: %s", code, message.c_str());
      break;
    case aubo_robot_namespace::RobotEventType::RobotEvent_currentAlarm:
      aubo_robot->robot_overcurrent = true;
      ROS_ERROR("Event Code: %d, Current Alarm!: %s", code, message.c_str());
      break;
    case aubo_robot_namespace::RobotEventType::RobotEvent_robotStartupPhase:
      ROS_INFO("Robot Startup Phase: %d, %s", code, message.c_str());
      break;
    case aubo_robot_namespace::RobotEventType::RobotEvent_robotStartupDoneResult:
      aubo_robot->arm_powered = true;
      ROS_INFO("Robot Startup Done Result: %d, %s", code, message.c_str());
      break;
    case aubo_robot_namespace::RobotEventType::RobotEvent_robotShutdownDone:
      aubo_robot->arm_powered = false;
      aubo_robot->robot_collision = false;
      aubo_robot->singularity_overspeed = false;
      aubo_robot->robot_overcurrent = false;
      ROS_INFO("Robot Shutdown Done: %d, %s", code, message.c_str());
      break;

    case aubo_robot_namespace::RobotEventType::RobotSetPowerOnDone:
      ROS_INFO("Event Code: %d, Robot Set Power Done: %s", code, message.c_str());
      break;
    case aubo_robot_namespace::RobotEventType::RobotReleaseBrakeDone:
      ROS_INFO("Event Code: %d, Release Brake Done: %s", code, message.c_str());
      break;
    case aubo_robot_namespace::RobotEventType::RobotEvent_robotControllerStateChaned:
      ROS_INFO("Event Code: %d, Robot Controller State Changed: %s", code, message.c_str());
      break;
    case aubo_robot_namespace::RobotEventType::RobotEvent_socketDisconnected:
      ROS_INFO("Event Code: %d, Socked Disconnected: %s", code, message.c_str());
      break;
  }
}


bool aubo::AuboRobot::login(std::string hostname, unsigned int port, std::string username, std::string password)
{
  int error_code;

  error_code = service_interface.robotServiceLogin(hostname.c_str(), port, username.c_str(), password.c_str());
  if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    return false;
  }

  return true;
}


bool aubo::AuboRobot::logout()
{
  int error_code;

  error_code = service_interface.robotServiceLogout();
  if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    return false;
  }

  return true;
}


bool aubo::AuboRobot::robot_startup(XmlRpc::XmlRpcValue &tool_dynamics, unsigned int collision_class)
{
  int error_code;

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
    return false;
  }

  switch (robot_state)
  {
    case aubo_robot_namespace::ROBOT_SERVICE_STATE::ROBOT_SERVICE_READY:
      break;
    case aubo_robot_namespace::ROBOT_SERVICE_STATE::ROBOT_SERVICE_STARTING:
      break;
    case aubo_robot_namespace::ROBOT_SERVICE_STATE::ROBOT_SERVICE_WORKING:
      break;
    case aubo_robot_namespace::ROBOT_SERVICE_STATE::ROBOT_SERVICE_CLOSING:
      break;
    case aubo_robot_namespace::ROBOT_SERVICE_STATE::ROBOT_SERVICE_CLOSED:
      break;
    case aubo_robot_namespace::ROBOT_SERVICE_STATE::ROBOT_SETVICE_FAULT_POWER:
      break;
    case aubo_robot_namespace::ROBOT_SERVICE_STATE::ROBOT_SETVICE_FAULT_BRAKE:
      break;
    case aubo_robot_namespace::ROBOT_SERVICE_STATE::ROBOT_SETVICE_FAULT_NO_ROBOT:
      break;
  }

  return true;
}


bool aubo::AuboRobot::robot_shutdown()
{
  int error_code;

  error_code = service_interface.robotServiceRobotShutdown();
  if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    return false;
  }

  return true;
}


bool aubo::AuboRobot::enable_tcp_canbus_mode()
{
  int error_code;

  error_code = service_interface.robotServiceEnterTcp2CanbusMode();
  if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    return false;
  }

  return true;
}


bool aubo::AuboRobot::disable_tcp_canbus_mode()
{
  int error_code;

  error_code = service_interface.robotServiceLeaveTcp2CanbusMode();
  if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    return false;
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


bool aubo::AuboRobot::get_robot_diagnostic_info()
{
  int error_code;

  error_code = service_interface.robotServiceGetRobotDiagnosisInfo(robotDiagnosis);
  if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
  {
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
    return false;
  }

  //
  for (int i = 0; i < statusVector.size(); i++)
  {
    char* name = statusVector[i].ioName;
    int addr = statusVector[i].ioAddr;
    double val = statusVector[i].ioValue;

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
    return false;
  }

  //
  for (int i = 0; i < statusVector.size(); i++)
  {
    char* name = statusVector[i].ioName;
    int addr = statusVector[i].ioAddr;
    double val = statusVector[i].ioValue;

    analog_inputs.push_back(val);
  }

  return true;
}


bool aubo::AuboRobot::register_event_info()
{
  int error_code;

  error_code = service_interface.robotServiceRegisterRobotEventInfoCallback(event_info_cb, this);
  if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    return false;
  }

  return true;
}
