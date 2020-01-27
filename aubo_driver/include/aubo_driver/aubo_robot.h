#ifndef AUBO_AUBO_ROBOT_H
#define AUBO_AUBO_ROBOT_H
// STL
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
// roscpp
#include <ros/ros.h>
#include <ros/console.h>
// actionlib
#include <actionlib/server/simple_action_server.h>
// std_srvs
#include <std_srvs/Trigger.h>
// sensor_msgs
#include <sensor_msgs/JointState.h>
// trajectory_msgs
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
// control_msgs
#include <control_msgs/JointTrajectoryAction.h>
// aubo_driver
// #include "aubo_driver/error_codes.h"
// AUBO SDK
#include "lib/AuboRobotMetaType.h"
#include "lib/serviceinterface.h"


namespace aubo {


class AuboRobot {
private:
  ros::NodeHandle node;
  std::vector<std::string> joint_names;

  actionlib::SimpleActionServer<control_msgs::JointTrajectoryAction> joint_trajectory_act;

  ros::ServiceServer login_srv;
  ros::ServiceServer logout_srv;

  ros::ServiceServer robot_startup_srv;
  ros::ServiceServer robot_shutdown_srv;

  ros::ServiceServer init_profile_srv;

  ros::ServiceServer stop_movement_srv;
  ros::ServiceServer pause_movement_srv;
  ros::ServiceServer resume_movement_srv;

  ros::Publisher joint_state_pub;


  ServiceInterface service_interface;
  int collision_class;
  double blend_radius;


public:

  AuboRobot(const ros::NodeHandle &node = ros::NodeHandle()) :
    node(node),
    joint_trajectory_act(node, "joint_trajectory", boost::bind(&aubo::AuboRobot::move_track, this, _1), false) { }


  /* */
  bool init();

  /*
   * Establishing network connection with the manipulator sever */
  bool login(std::string username = "AUBO", std::string password = "123456");
  bool login(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  /*
   * Disconnecting from the manipulator server */
  bool logout();
  bool logout(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);


  /* Initializing the manipulator, including power on, release the brake, set up
   * the collision class, set up the kinematics parameters.
   * This function needs a longtime to complete, so user can set its block mode
   * to adjust the return time of the function. When it is set to unblocked mode,
   * it will return immediately after calling function, the result will be notified by event.
   * When it is set to block mode, return value represents whether the interface
   * has been called successfully. */

  bool robot_startup();
  bool robot_startup(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  /*
   * Shutdown the manipulator, including power off, hold the brake */

  bool robot_shutdown();
  bool robot_shutdown(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  /*
  * Initialize the movement property, set properties to default. */

  bool init_profile();
  bool init_profile(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);


  /*  */
  void get_max_joint_acceleration(std::vector<double> &result);       // Get the maximum acceleration of each joint.
  bool set_max_joint_acceleration(const std::vector<double> &value);  // Set the maximum acceleration of each joint.

  void get_max_joint_velocity(std::vector<double> &result);           // Get the maximum velocity of each joint.
  bool set_max_joint_velocity(const std::vector<double> &value);      // Set the maximum velocity of each joint.

  /* */
  void get_max_linear_acceleration(double &result);   // Get the maximum linear acceleration of end-effector movement.
  bool set_max_linear_acceleration(double value);     // Set the maximum linear acceleration of end-effector movement.

  void get_max_linear_velocity(double &result);       // Get the maximum linear velocity of end-effector movement.
  bool set_max_linear_velocity(double value);         // Set the maximum linear velocity of end-effector movement.

  void get_max_angular_acceleration(double &result);  // Get the maximum angular acceleration of end-effector movement.
  bool set_max_angular_acceleration(double value);    // Set the maximum angular acceleration of end-effector movement.

  void get_max_angular_velocity(double &result);      // Get the maximum angular velocity of end-effector movement.
  bool set_max_angular_velocity(double value);        // Set the maximum angular velocity of end-effector movement.


/* The manipulator moves to the target position through the joint movement
 * (Move Joint), the target position is described by the angle of each joint.
 * The maximum velocity and the maximum acceleration of the joint movement
 * should be set before calling it, and the offset attribute should be set if
 * the offset is used. The input has two option, one is the waypoint, or directly
 * gives the angle of each joint. */

  bool move_joint(const std::vector<double> &joint_pos);


/* The manipulator moves to the target position through the linear movement
 * (Move Line), the target position is described by the angle of each joint.
 * The maximum linear velocity and the maximum linear acceleration should be set
 * before calling it, and the offset attribute should be set if the offset is used.
 * The input has two option, one is the waypoint, or directly gives the angle of
 * each joint. */

  bool move_line(const std::vector<double> &joint_pos);


/* The track movement of the manipulator. This movement is belonged to different
 * movement type according to the different type of subMoveMode. */

  void move_track(const control_msgs::JointTrajectoryGoal::ConstPtr &goal);


  /*
   * Get the joint angles of the manipulator */
  bool get_joint_angle(std::vector<double> &joint_pos);

  /*
   * Get the waypoint information of the manipulator */
  void get_current_waypoint();


  /*
   * Get the diagnostic information of the manipulator */
  void print_diagnostic_info();


/** Real-time manipulator information push module **/

/* The interfaces are mainly about the push of the manipulator information.
 * The information push of the manipulator in the interface is implemented by the callback function.
 * The developer needs to define the callback function and register the defined callback
 * function into the system using the following interface to implement the information push.
 * This part of the interface includes the push of real-time joint information,
 * the push of real-time waypoint information, the push of real-time end velocity,
 * and the push of the event information of the manipulator.
 * The following interfaces are used to register the user-defined callback function into our system. */

/* Registers the callback function for obtaining a real-time waypoint.
 * After registering the callback function, the server pushes the current waypoint
 * information in real time. */

 bool register_waipoint_callback(RealTimeRoadPointCallback ptr, void *arg);

/* Registers the callback function for obtaining the event information of the manipulator.
 * After registering the callback function, the server pushes the event information in real time.
 * Regarding the event information pushing, it does not provide the interface
 * for changing whether it is allowed the information to be pushed because many
 * important notifications of the manipulator are implemented by pushing event information.
 * So, the event information is the system default push, not allowed to cancel. */

  bool register_event_info(RobotEventCallback ptr, void *arg);

  /*
   * Define the function pointer for the waypoint information push */
  void real_time_waypoint_callback(const aubo_robot_namespace::wayPoint_S *wayPoint, void *arg);

};

} // namespace
#endif
