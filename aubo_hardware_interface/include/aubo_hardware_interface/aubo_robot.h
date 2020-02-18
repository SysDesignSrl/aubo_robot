#ifndef AUBO_ROBOT_H
#define AUBO_ROBOT_H
// STL
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
// roscpp
#include <ros/ros.h>
#include <ros/console.h>
//
#include <XmlRpcValue.h>
// AUBO SDK
#include "lib/AuboRobotMetaType.h"
#include "lib/robot_state.h"
#include "lib/serviceinterface.h"


namespace aubo {

class AuboRobot {
private:

  ServiceInterface service_interface;

public:
  aubo_robot_namespace::RobotDiagnosis robotDiagnosis;    // Robot Diagnostic


  bool login(std::string hostname, unsigned int port, std::string username = "AUBO", std::string password = "123456");

  bool logout();


  bool robot_startup();

  bool robot_shutdown();


  bool enable_tcp_canbus_mode();

  bool disable_tcp_canbus_mode();


  bool set_max_joint_acceleration(const std::vector<double> &value);

  void get_max_joint_acceleration(std::vector<double> &result);

  bool set_max_joint_velocity(const std::vector<double> &value);

  void get_max_joint_velocity(std::vector<double> &result);


  bool get_digital_input(int addr, bool &value);
  bool get_digital_input(std::string name, bool &value);

  bool set_digital_output(int addr, bool value);
  bool set_digital_output(std::string name, bool value);

  bool get_analog_input(int addr, double &value);
  bool get_analog_input(std::string name, double &value);

  bool set_analog_output(int addr, double value);
  bool set_analog_output(std::string name, double value);

  bool get_digital_inputs(std::vector<bool> &digital_inputs);

  bool get_analog_inputs(std::vector<double> &analog_inputs);


  bool get_robot_diagnostic_info();


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
