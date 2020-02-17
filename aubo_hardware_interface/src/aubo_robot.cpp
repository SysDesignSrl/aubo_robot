#include "aubo_hardware_interface/aubo_robot.h"


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

  int collision_class = 6;

  aubo_robot_namespace::ROBOT_SERVICE_STATE result;

  error_code = service_interface.rootServiceRobotStartup(tool_dynamics_param, collision_class, true, true, 1000, result);
  if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    return false;
  }

  switch (result)
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
