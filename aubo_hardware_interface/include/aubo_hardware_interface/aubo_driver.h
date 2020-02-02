#ifndef AUBO_DRIVER_H
#define AUBO_DRIVER_H
// STL
#include <iostream>
#include <string>
#include <algorithm>
// AUBO SDK
#include "lib/AuboRobotMetaType.h"
#include "lib/robot_state.h"
#include "lib/serviceinterface.h"


namespace aubo {

class AuboDriver {
private:

  ServiceInterface service_interface;

public:

  bool login(std::string hostname, unsigned int port, std::string username = "AUBO", std::string password = "123456")
  {
    int error_code;

    error_code = service_interface.robotServiceLogin(hostname.c_str(), port, username.c_str(), password.c_str());
    if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
    {
      return false;
    }

    return true;
  }


  bool logout()
  {
    int error_code;

    error_code = service_interface.robotServiceLogout();
    if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
    {
      return false;
    }

    return true;
  }


  bool robot_startup()
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


  bool robot_shutdown()
  {
    int error_code;

    error_code = service_interface.robotServiceRobotShutdown();
    if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
    {
      return false;
    }

    return true;
  }


  bool enable_tcp_canbus_mode()
  {
    int error_code;

    error_code = service_interface.robotServiceEnterTcp2CanbusMode();
    if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
    {
      return false;
    }

    return true;
  }


  bool disable_tcp_canbus_mode()
  {
    int error_code;

    error_code = service_interface.robotServiceLeaveTcp2CanbusMode();
    if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
    {
      return false;
    }

    return true;
  }


  bool read(std::vector<double> &joint_pos)
  {
    int error_code;
    aubo_robot_namespace::JointParam jointParam;

    error_code = service_interface.robotServiceGetJointAngleInfo(jointParam);
    if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
    {
      return false;
    }

    joint_pos.resize(aubo_robot_namespace::ARM_DOF);
    std::copy(jointParam.jointPos, jointParam.jointPos + aubo_robot_namespace::ARM_DOF, joint_pos.begin());

    return true;
  }


  bool write(const std::vector<double> &joint_pos)
  {
    int error_code;
    double jointAngle[aubo_robot_namespace::ARM_DOF];

    std::copy(joint_pos.cbegin(), joint_pos.cend(), jointAngle);

    error_code = service_interface.robotServiceSetRobotPosData2Canbus(jointAngle);
    if (error_code != aubo_robot_namespace::InterfaceCallSuccCode)
    {
      return false;
    }

    return true;
  }

};

} // namespace
#endif
