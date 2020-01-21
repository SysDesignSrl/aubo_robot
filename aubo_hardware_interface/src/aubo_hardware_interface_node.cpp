// roscpp
#include <ros/ros.h>
#include <ros/console.h>
// controller manager
#include <controller_manager/controller_manager.h>
// aubo_hardware_interface
#include "aubo_hardware_interface/aubo_hardware_interface.h"


int main(int argc, char* argv[]) {

  ros::init(argc, argv, "aubo_hardware_interface");

  // Node
  ros::NodeHandle node("~");

  // Hardware Interface
  aubo_hardware_interface::AuboHardwareInterface aubo_hw(node);

  // Controller Manager
  controller_manager::ControllerManager controller_manager(&aubo_hw, node);


  ros::Rate rate(aubo_hw.loop_hz);
  ros::Time prev_time = ros::Time::now();

  while (ros::ok()) {
    rate.sleep();

    ros::Time curr_time = ros::Time::now();
    ros::Duration period = curr_time - prev_time;

    aubo_hw.read();
    controller_manager.update(curr_time, period);
    aubo_hw.write();

    prev_time = curr_time;
  }

  return 0;
}
