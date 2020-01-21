#ifndef AUBO_DRIVER_H
#define AUBO_DRIVER_H
// STL
#include <iostream>
#include <string>
// aubo_driver
#include <aubo_driver/aubo_driver.h>
#include <aubo_driver/serviceinterface.h>


namespace aubo {

class AuboDriver {
private:


public:
  bool controller_connected_flag;
  bool real_robot_exist;

  aubo_driver::RobotState robot_state;

  ServiceInterface robot_send_service;
  ServiceInterface robot_receive_service;


  bool login(std::string hostname, unsigned int port, std::string username = "aubo", std::string password = "123456") {
    int ret = -1;
    std::cout << "Logging to: " << hostname << ":" << port << "..." << std::endl;
    std::cout << "Username: " << username << std::endl;
    std::cout << "Password: " << password << std::endl;

    // Login
    ret = robot_send_service.robotServiceLogin(hostname.c_str(), port, username.c_str(), password.c_str());

    if (ret != aubo_robot_namespace::InterfaceCallSuccCode)
    {
      controller_connected_flag  = false;
      std::cout << "Login failed." << std::endl;
      return false;
    }

    controller_connected_flag  = true;
    std::cout << "Login success." << std::endl;


    // Real Robot
    ret = robot_receive_service.robotServiceGetIsRealRobotExist(real_robot_exist);

    if (ret != aubo_robot_namespace::InterfaceCallSuccCode)
    {
      std::cout << "Failed to check if real robot exists." << std::endl;
      return false;
    }

    if (real_robot_exist) {
      std::cout << "real robot exist." << std::endl;
    }
    else {
      std::cout << "real robot doesn't exist." << std::endl;
    }

    //power on the robot.

    return true;
  }

  bool logout() {}


  bool read() {}

  bool write() {}

};

} // namespace
#endif
