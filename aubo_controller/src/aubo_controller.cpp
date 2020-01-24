// roscpp
#include <ros/ros.h>
#include <ros/console.h>
// aubo_controller
#include "aubo_controller/aubo_controller.h"


int main(int argc, char* argv[]) {

  ros::init(argc, argv, "aubo_controller");

  // Node
  ros::NodeHandle node("~");


  // AUBO Controller
  aubo::AuboController controller(node);
  // controller.start();


  // Loop
  ros::MultiThreadedSpinner spinner(0);
  spinner.spin();

  return 0;
}
