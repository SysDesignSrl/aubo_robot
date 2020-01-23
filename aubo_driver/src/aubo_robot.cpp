#include "aubo_robot/aubo_robot.h"


bool AuboRobot::login(std::string username, std::string password) {
  int error_code;

  error_code = service_interface.robotServiceLogin(hostname.c_str(), port, username.c_str(), password.c_str());
  if (error_code != 0) {
    ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
    ROS_ERROR("Failed to login to %s:%d", hostname.c_str(), port);
    ROS_ERROR("Username %s", username.c_str());
    ROS_ERROR("Password %s", password.c_str());
    return false;
  }

  ROS_INFO("Logged in to %s:%d", hostname.c_str(), port);
  ROS_INFO("Username: %s", username.c_str());
  ROS_INFO("Password: %s", password.c_str());
  return true;
}


bool AuboRobot::logout() {
  int error_code;

  error_code = service_interface.robotServiceLogout();
  if (error_code != 0) {
    ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
    ROS_ERROR("Failed to logout.");
    return false;
  }

  ROS_INFO("Logged out.");
  return true;
}
