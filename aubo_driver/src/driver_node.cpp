// STL
#include <string>
//
#include <ros/ros.h>
#include <ros/console.h>
// std_srvs
#include <std_srvs/Trigger.h>
//
#include "aubo_driver/aubo_robot.h"
// AUBO SDK
#include "lib/AuboRobotMetaType.h"
#include "lib/serviceinterface.h"



// void robot_waypoint_callback(const aubo_robot_namespace::wayPoint_S *wayPoint, void *arg) {
//
//   std::vector<double> joint_pos(aubo_robot_namespace::ARM_DOF);
//   std::copy(wayPoint.jointpos, wayPoint.jointpos + aubo_robot_namespace::ARM_DOF, joint_pos.begin());
//
//   sensor_msgs::JointState joint_state;
//   joint_state.header.stamp = ros::Time::now();
//   joint_state.name = joint_names;
//   joint_state.position = joint_pos;
//   joint_state_pub.publish(joint_state);
// }

void robot_event_info_callback(const aubo_robot_namespace::RobotEventInfo *eventInfo, void *arg) {

  std::stringstream ss;
  ROS_INFO("Event Code: %d, %s", eventInfo->eventCode, eventInfo->eventContent.c_str());

}


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
    // auto init_profile_cli = node.serviceClient<std_srvs::Trigger>("init_profile");

    // Published Topics
    auto joint_state_pub = node.advertise<sensor_msgs::JointState>("joint_states", 100);


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


    // Robot Startup
    while (!robot_startup_cli.waitForExistence(ros::Duration(1.0))) {
      ROS_INFO_THROTTLE(10, "Waiting for service: '%s'...", robot_startup_cli.getService().c_str());
    }

    std_srvs::Trigger startup_trigger;
    if (!robot_startup_cli.call(startup_trigger)) {
      ROS_FATAL("Robot startup error!");
      return 1;
    }

    if (!startup_trigger.response.success) {
      ROS_FATAL_STREAM(startup_trigger.response.message);
      return 1;
    }


    // register to event info
    robot.register_event_info(robot_event_info_callback, nullptr);


    // init motion profile
    robot.init_profile();

    // set max joint acceleration
    std::vector<double> joint_max_acc
    {
      50.0 / 180*M_PI,
      50.0 / 180*M_PI,
      50.0 / 180*M_PI,
      50.0 / 180*M_PI,
      50.0 / 180*M_PI,
      50.0 / 180*M_PI
    };
    robot.set_max_joint_acceleration(joint_max_acc);

    // set max joint velocity
    std::vector<double> joint_max_vel
    {
      50.0 / 180*M_PI,
      50.0 / 180*M_PI,
      50.0 / 180*M_PI,
      50.0 / 180*M_PI,
      50.0 / 180*M_PI,
      50.0 / 180*M_PI
    };
    robot.set_max_joint_velocity(joint_max_vel);

    // init motion profile
    robot.init_profile();

    robot.set_max_linear_acceleration(0.2);
    robot.set_max_linear_velocity(0.2);
    robot.set_max_angular_acceleration(0.2);
    robot.set_max_angular_velocity(0.2);


    // Loop
    ros::Rate rate(freq);
    while (ros::ok()) {
      rate.sleep();

      robot.get_joint_angle();
    }


    // Robot Shutdown
    while (!robot_shutdown_cli.waitForExistence(ros::Duration(1.0))) {
      ROS_INFO_THROTTLE(10, "Waiting for service: '%s'...", robot_shutdown_cli.getService().c_str());
    }

    std_srvs::Trigger shutdown_trigger;
    if (!robot_shutdown_cli.call(shutdown_trigger)) {
      ROS_FATAL("Robot shutdown error!");
      return 1;
    }

    if (!shutdown_trigger.response.success) {
      ROS_FATAL_STREAM(shutdown_trigger.response.message);
      return 1;
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
