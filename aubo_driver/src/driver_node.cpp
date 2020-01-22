// STL
#include <string>
//
#include <ros/ros.h>
#include <ros/console.h>
// std_srvs
#include <std_srvs/Trigger.h>
//
#include "aubo_driver/aubo_robot.h"


int main(int argc, char* argv[]) {

    ros::init(argc, argv, "aubo_driver");

    // Node
    ros::NodeHandle node("~");

    // Parameters
    auto freq = node.param<double>("publish_frequency", 10);

    // Client Services
    auto login_cli = node.serviceClient<std_srvs::Trigger>("login");
    auto logout_cli = node.serviceClient<std_srvs::Trigger>("logout");

    auto robot_startup_cli = node.serviceClient<std_srvs::Trigger>("robot_startup");
    auto robot_shutdown_cli = node.serviceClient<std_srvs::Trigger>("robot_shutdown");


    ros::AsyncSpinner spinner(0);
    spinner.start();


    // AUBO Robot
    aubo::AuboRobot robot(node);
    if (!robot.init()) {
      ROS_FATAL("Failed to initialize the robot!");
      return 1;
    }


    // Login
    while (!login_cli.waitForExistence(ros::Duration(1.0))) {
      ROS_INFO_THROTTLE(10, "Waiting for service: '%s'...", login_cli.getService().c_str());
    }

    std_srvs::Trigger login_trigger;
    if (!login_cli.call(login_trigger)) {
      ROS_FATAL("Login error!");
      return 1;
    }

    if (!login_trigger.response.success) {
      ROS_FATAL_STREAM(login_trigger.response.message);
      return 1;
    }


    // Loop
    ros::Rate rate(freq);
    while (ros::ok()) {
      rate.sleep();

      robot.get_joint_angle();
    }


    // Logout
    while (!logout_cli.waitForExistence(ros::Duration(1.0))) {
      ROS_INFO_THROTTLE(10, "Waiting for service: '%s'...", logout_cli.getService().c_str());
    }

    std_srvs::Trigger logout_trigger;
    if (!logout_cli.call(login_trigger)) {
      ROS_FATAL("Logout error!");
      return 1;
    }

    if (!logout_trigger.response.success) {
      ROS_FATAL_STREAM(logout_trigger.response.message);
      return 1;
    }


    return 0;
}
