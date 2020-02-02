// roscpp
#include <ros/ros.h>
#include <ros/console.h>
// controller manager
#include <controller_manager/controller_manager.h>
// aubo_hardware_interface
#include "aubo_hardware_interface/aubo_hardware_interface.h"


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "aubo_hardware_interface");

  // Node
  ros::NodeHandle node("~");

  // Parameters

  double loop_hz;
  if (!node.getParam("/aubo/hardware_interface/loop_hz", loop_hz))
  {
    ROS_ERROR("Failed to retrieve '/aubo/hardware_interface/loop_hz' parameter.");
    return 1;
  }


  ros::AsyncSpinner spinner(0);
  spinner.start();


  // Hardware Interface
  aubo_hardware_interface::AuboHW aubo_hw(node);

  if (!aubo_hw.start())
  {
    ROS_FATAL("Failed to start Hardware Interface.");
    return 1;
  }

  // Controller Manager
  controller_manager::ControllerManager controller_manager(&aubo_hw, node);


  ros::Rate rate(loop_hz);
  ros::Time prev_time = ros::Time::now();

  while (ros::ok())
  {
    ros::Time curr_time = ros::Time::now();
    ros::Duration period = curr_time - prev_time;

    aubo_hw.read();
    controller_manager.update(curr_time, period);
    aubo_hw.write();

    prev_time = curr_time;
    rate.sleep();
  }


  if (!aubo_hw.stop())
  {
    ROS_FATAL("Failed to stop Hardware Interface.");
    return 1;
  }


  return 0;
}
