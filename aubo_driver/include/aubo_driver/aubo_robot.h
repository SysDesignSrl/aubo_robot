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
// std_msgs
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
// std_srvs
#include <std_srvs/Empty.h>
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

  ros::Timer refresh_cycle;

  std::vector<std::string> joint_names;

  actionlib::SimpleActionServer<control_msgs::JointTrajectoryAction> joint_trajectory_act;

  // Services
  ros::ServiceServer login_srv;
  ros::ServiceServer logout_srv;

  ros::ServiceServer robot_startup_srv;
  ros::ServiceServer robot_shutdown_srv;

  ros::ServiceServer init_profile_srv;

  ros::ServiceServer stop_movement_srv;
  ros::ServiceServer pause_movement_srv;
  ros::ServiceServer resume_movement_srv;

  ros::ServiceServer print_diagnostic_srv;

  // Topics
  ros::Publisher joint_state_pub;

  ros::Publisher di_00_pub;
  ros::Publisher di_01_pub;
  ros::Publisher di_02_pub;
  ros::Publisher di_03_pub;
  ros::Publisher di_04_pub;
  ros::Publisher di_05_pub;
  ros::Publisher di_06_pub;
  ros::Publisher di_07_pub;
  ros::Publisher di_10_pub;
  ros::Publisher di_11_pub;
  ros::Publisher di_12_pub;
  ros::Publisher di_13_pub;
  ros::Publisher di_14_pub;
  ros::Publisher di_15_pub;
  ros::Publisher di_16_pub;
  ros::Publisher di_17_pub;

  ros::Subscriber do_00_sub;
  ros::Subscriber do_01_sub;
  ros::Subscriber do_02_sub;
  ros::Subscriber do_03_sub;
  ros::Subscriber do_04_sub;
  ros::Subscriber do_05_sub;
  ros::Subscriber do_06_sub;
  ros::Subscriber do_07_sub;
  ros::Subscriber do_10_sub;
  ros::Subscriber do_11_sub;
  ros::Subscriber do_12_sub;
  ros::Subscriber do_13_sub;
  ros::Subscriber do_14_sub;
  ros::Subscriber do_15_sub;
  ros::Subscriber do_16_sub;
  ros::Subscriber do_17_sub;

  ros::Publisher ai_00_pub;
  ros::Publisher ai_01_pub;
  ros::Publisher ai_02_pub;
  ros::Publisher ai_03_pub;

  ros::Subscriber ao_00_sub;
  ros::Subscriber ao_01_sub;
  ros::Subscriber ao_02_sub;
  ros::Subscriber ao_03_sub;


  ServiceInterface service_interface;
  int collision_class;
  double blend_radius;


  void refresh_cycle_cb(const ros::TimerEvent &ev)
  {
    //
    publish_digital_inputs();
    //
    publish_analog_inputs();
  }


  void publish_digital_inputs()
  {
    //
    bool val;
    std_msgs::Bool msg;

    get_digital_input("U_DI_00", val);
    msg.data = val;
    di_00_pub.publish(msg);
    get_digital_input("U_DI_01", val);
    msg.data = val;
    di_01_pub.publish(msg);
    get_digital_input("U_DI_02", val);
    msg.data = val;
    di_02_pub.publish(msg);
    get_digital_input("U_DI_03", val);
    msg.data = val;
    di_03_pub.publish(msg);
    get_digital_input("U_DI_04", val);
    msg.data = val;
    di_04_pub.publish(msg);
    get_digital_input("U_DI_05", val);
    msg.data = val;
    di_05_pub.publish(msg);
    get_digital_input("U_DI_06", val);
    msg.data = val;
    di_06_pub.publish(msg);
    get_digital_input("U_DI_07", val);
    msg.data = val;
    di_07_pub.publish(msg);
    get_digital_input("U_DI_10", val);
    msg.data = val;
    di_10_pub.publish(msg);
    get_digital_input("U_DI_11", val);
    msg.data = val;
    di_11_pub.publish(msg);
    get_digital_input("U_DI_12", val);
    msg.data = val;
    di_12_pub.publish(msg);
    get_digital_input("U_DI_13", val);
    msg.data = val;
    di_13_pub.publish(msg);
    get_digital_input("U_DI_14", val);
    msg.data = val;
    di_14_pub.publish(msg);
    get_digital_input("U_DI_15", val);
    msg.data = val;
    di_15_pub.publish(msg);
    get_digital_input("U_DI_16", val);
    msg.data = val;
    di_16_pub.publish(msg);
    get_digital_input("U_DI_17", val);
    msg.data = val;
    di_17_pub.publish(msg);
  }


  void publish_analog_inputs()
  {
    //
    double val;
    std_msgs::Float64 msg;

    get_analog_input("VI0", val);
    msg.data = val;
    ai_00_pub.publish(msg);
    get_analog_input("VI1", val);
    msg.data = val;
    ai_01_pub.publish(msg);
    get_analog_input("VI2", val);
    msg.data = val;
    ai_02_pub.publish(msg);
    get_analog_input("VI3", val);
    msg.data = val;
    ai_03_pub.publish(msg);
  }


  void do_00_cb(const std_msgs::Bool::ConstPtr &msg) { set_digital_output(00, msg->data); }
  void do_01_cb(const std_msgs::Bool::ConstPtr &msg) { set_digital_output(01, msg->data); }
  void do_02_cb(const std_msgs::Bool::ConstPtr &msg) { set_digital_output(02, msg->data); }
  void do_03_cb(const std_msgs::Bool::ConstPtr &msg) { set_digital_output(03, msg->data); }
  void do_04_cb(const std_msgs::Bool::ConstPtr &msg) { set_digital_output(04, msg->data); }
  void do_05_cb(const std_msgs::Bool::ConstPtr &msg) { set_digital_output(05, msg->data); }
  void do_06_cb(const std_msgs::Bool::ConstPtr &msg) { set_digital_output(06, msg->data); }
  void do_07_cb(const std_msgs::Bool::ConstPtr &msg) { set_digital_output(07, msg->data); }
  void do_10_cb(const std_msgs::Bool::ConstPtr &msg) { set_digital_output(010, msg->data); }
  void do_11_cb(const std_msgs::Bool::ConstPtr &msg) { set_digital_output(011, msg->data); }
  void do_12_cb(const std_msgs::Bool::ConstPtr &msg) { set_digital_output(012, msg->data); }
  void do_13_cb(const std_msgs::Bool::ConstPtr &msg) { set_digital_output(013, msg->data); }
  void do_14_cb(const std_msgs::Bool::ConstPtr &msg) { set_digital_output(014, msg->data); }
  void do_15_cb(const std_msgs::Bool::ConstPtr &msg) { set_digital_output(015, msg->data); }
  void do_16_cb(const std_msgs::Bool::ConstPtr &msg) { set_digital_output(016, msg->data); }
  void do_17_cb(const std_msgs::Bool::ConstPtr &msg) { set_digital_output(017, msg->data); }


  void ao_00_cb(const std_msgs::Float64::ConstPtr &msg) { set_analog_output(00, msg->data); }
  void ao_01_cb(const std_msgs::Float64::ConstPtr &msg) { set_analog_output(01, msg->data); }
  void ao_02_cb(const std_msgs::Float64::ConstPtr &msg) { set_analog_output(02, msg->data); }
  void ao_03_cb(const std_msgs::Float64::ConstPtr &msg) { set_analog_output(03, msg->data); }


public:

  AuboRobot(const ros::NodeHandle &node = ros::NodeHandle()) :
    node(node),
    joint_trajectory_act(node, "joint_trajectory", boost::bind(&aubo::AuboRobot::move_track, this, _1), false) { }


  bool init()
  {
    // Parameters
    if (!node.getParam("joint_names", joint_names))
    {
      ROS_FATAL("Failed to get parameter: '%s'", "joint_names");
      return false;
    }

    collision_class = node.param<int>("aubo/collision_class", 6);
    blend_radius = node.param<double>("aubo/blend_radius", 0.02);

    // Services
    login_srv = node.advertiseService("login", &aubo::AuboRobot::login, this);
    logout_srv = node.advertiseService("logout", &aubo::AuboRobot::logout, this);

    robot_startup_srv = node.advertiseService("robot_startup", &aubo::AuboRobot::robot_startup, this);
    robot_shutdown_srv = node.advertiseService("robot_shutdown", &aubo::AuboRobot::robot_shutdown, this);

    init_profile_srv = node.advertiseService("init_profile", &aubo::AuboRobot::init_profile, this);

    print_diagnostic_srv = node.advertiseService("print_diagnostic_info", &aubo::AuboRobot::print_diagnostic_info, this);

    // Topics
    joint_state_pub = node.advertise<sensor_msgs::JointState>("joint_states", 100);

    di_00_pub = node.advertise<std_msgs::Bool>("DI/00", 1);
    di_01_pub = node.advertise<std_msgs::Bool>("DI/01", 1);
    di_02_pub = node.advertise<std_msgs::Bool>("DI/02", 1);
    di_03_pub = node.advertise<std_msgs::Bool>("DI/03", 1);
    di_04_pub = node.advertise<std_msgs::Bool>("DI/04", 1);
    di_05_pub = node.advertise<std_msgs::Bool>("DI/05", 1);
    di_06_pub = node.advertise<std_msgs::Bool>("DI/06", 1);
    di_07_pub = node.advertise<std_msgs::Bool>("DI/07", 1);
    di_10_pub = node.advertise<std_msgs::Bool>("DI/10", 1);
    di_11_pub = node.advertise<std_msgs::Bool>("DI/11", 1);
    di_12_pub = node.advertise<std_msgs::Bool>("DI/12", 1);
    di_13_pub = node.advertise<std_msgs::Bool>("DI/13", 1);
    di_14_pub = node.advertise<std_msgs::Bool>("DI/14", 1);
    di_15_pub = node.advertise<std_msgs::Bool>("DI/15", 1);
    di_16_pub = node.advertise<std_msgs::Bool>("DI/16", 1);
    di_17_pub = node.advertise<std_msgs::Bool>("DI/17", 1);

    do_00_sub = node.subscribe<std_msgs::Bool>("DO/00", 1, &aubo::AuboRobot::do_00_cb, this);
    do_01_sub = node.subscribe<std_msgs::Bool>("DO/01", 1, &aubo::AuboRobot::do_01_cb, this);
    do_02_sub = node.subscribe<std_msgs::Bool>("DO/02", 1, &aubo::AuboRobot::do_02_cb, this);
    do_03_sub = node.subscribe<std_msgs::Bool>("DO/03", 1, &aubo::AuboRobot::do_03_cb, this);
    do_04_sub = node.subscribe<std_msgs::Bool>("DO/04", 1, &aubo::AuboRobot::do_04_cb, this);
    do_05_sub = node.subscribe<std_msgs::Bool>("DO/05", 1, &aubo::AuboRobot::do_05_cb, this);
    do_06_sub = node.subscribe<std_msgs::Bool>("DO/06", 1, &aubo::AuboRobot::do_06_cb, this);
    do_07_sub = node.subscribe<std_msgs::Bool>("DO/07", 1, &aubo::AuboRobot::do_07_cb, this);
    do_10_sub = node.subscribe<std_msgs::Bool>("DO/10", 1, &aubo::AuboRobot::do_10_cb, this);
    do_11_sub = node.subscribe<std_msgs::Bool>("DO/11", 1, &aubo::AuboRobot::do_11_cb, this);
    do_12_sub = node.subscribe<std_msgs::Bool>("DO/12", 1, &aubo::AuboRobot::do_12_cb, this);
    do_13_sub = node.subscribe<std_msgs::Bool>("DO/13", 1, &aubo::AuboRobot::do_13_cb, this);
    do_14_sub = node.subscribe<std_msgs::Bool>("DO/14", 1, &aubo::AuboRobot::do_14_cb, this);
    do_15_sub = node.subscribe<std_msgs::Bool>("DO/15", 1, &aubo::AuboRobot::do_15_cb, this);
    do_16_sub = node.subscribe<std_msgs::Bool>("DO/16", 1, &aubo::AuboRobot::do_16_cb, this);
    do_17_sub = node.subscribe<std_msgs::Bool>("DO/17", 1, &aubo::AuboRobot::do_17_cb, this);

    ai_00_pub = node.advertise<std_msgs::Float64>("AI/00", 1);
    ai_01_pub = node.advertise<std_msgs::Float64>("AI/01", 1);
    ai_02_pub = node.advertise<std_msgs::Float64>("AI/02", 1);
    ai_03_pub = node.advertise<std_msgs::Float64>("AI/03", 1);

    ao_00_sub = node.subscribe<std_msgs::Float64>("AO/00", 1, &aubo::AuboRobot::ao_00_cb, this);
    ao_01_sub = node.subscribe<std_msgs::Float64>("AO/01", 1, &aubo::AuboRobot::ao_01_cb, this);
    ao_02_sub = node.subscribe<std_msgs::Float64>("AO/02", 1, &aubo::AuboRobot::ao_02_cb, this);
    ao_03_sub = node.subscribe<std_msgs::Float64>("AO/03", 1, &aubo::AuboRobot::ao_03_cb, this);


    // Refresh Cycle
    ros::Duration period(1.0);
    refresh_cycle = node.createTimer(period, &aubo::AuboRobot::refresh_cycle_cb, this, false, false);

    return true;
  }


  bool start()
  {
    // Login
    if (!login())
    {
      ROS_FATAL("Failed to log in.");
      return false;
    }

    // Robot startup
    if (!robot_startup())
    {
      ROS_FATAL("Failed to startup the robot.");
      return false;
    }


    init_profile();


    std::vector<double> max_joint_acc;
    if (node.getParam("aubo/max_joint_acceleration", max_joint_acc))
    {
      set_max_joint_acceleration(max_joint_acc);
    }
    std::vector<double> max_joint_vel;
    if (node.getParam("aubo/max_joint_velocity", max_joint_vel))
    {
      set_max_joint_velocity(max_joint_vel);
    }

    get_max_joint_acceleration(max_joint_acc);
    std::stringstream ass;
    ass << "[ ";
    for (double val : max_joint_acc)
    {
      ass << val << " ";
    }
    ass << "] ";
    ROS_DEBUG_STREAM("max joint acceleration: " << ass.str() << "[rad/s^2]");

    get_max_joint_velocity(max_joint_vel);
    std::stringstream vss;
    vss << "[ ";
    for (double val : max_joint_vel)
    {
      vss << val << " ";
    }
    vss << "] ";
    ROS_DEBUG_STREAM("max joint velocity: " << vss.str() << "[rad/s]");


    double max_linear_acc;
    if (node.getParam("aubo/max_linear_acceleration", max_linear_acc))
    {
      set_max_linear_acceleration(max_linear_acc);
    }
    double max_linear_vel;
    if (node.getParam("aubo/max_linear_velocity", max_linear_vel))
    {
      set_max_linear_velocity(max_linear_vel);
    }
    double max_angular_acc;
    if (node.getParam("aubo/max_angular_acceleration", max_angular_acc))
    {
      set_max_angular_acceleration(max_angular_acc);
    }
    double max_angular_vel;
    if (node.getParam("aubo/max_angular_velocity", max_angular_vel))
    {
      set_max_angular_velocity(max_angular_vel);
    }

    get_max_linear_acceleration(max_linear_acc);
    ROS_DEBUG("max linear acceleration: %.2f [m/s^2]", max_linear_acc);

    get_max_linear_velocity(max_linear_vel);
    ROS_DEBUG("max linear velocity: %.2f [m/s]", max_linear_vel);

    get_max_angular_acceleration(max_angular_acc);
    ROS_DEBUG("max angular acceleration: %.2f [rad/s^2]", max_angular_acc);

    get_max_angular_velocity(max_angular_vel);
    ROS_DEBUG("max angular velocity: %.2f [rad/s]", max_angular_vel);


    refresh_cycle.start();
    joint_trajectory_act.start();
  }


  bool stop()
  {
    // Robot Shutdown
    if (!robot_shutdown())
    {
      ROS_FATAL("Failed to shutdown the robot.");
      return false;
    }

    // Logout
    if (!logout())
    {
      ROS_FATAL("Failed to log out.");
      return false;
    }
  }


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
   * Control the movement by using control command */
  bool move_stop();
  bool move_stop(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  bool move_pause();
  bool move_pause(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  bool move_resume();
  bool move_resume(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  /*
   * Get the joint angles of the manipulator */
  bool get_joint_angle(std::vector<double> &joint_pos);

  /*
   * Get the waypoint information of the manipulator */
  void get_current_waypoint();


  /*
   * Get the diagnostic information of the manipulator */
  void print_diagnostic_info();
  bool print_diagnostic_info(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res);


/// Real-time manipulator information push module ///

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

 bool register_realtime_waypoint(RealTimeRoadPointCallback waypoint_cb, void *arg);

/* Registers the callback function for obtaining the event information of the manipulator.
 * After registering the callback function, the server pushes the event information in real time.
 * Regarding the event information pushing, it does not provide the interface
 * for changing whether it is allowed the information to be pushed because many
 * important notifications of the manipulator are implemented by pushing event information.
 * So, the event information is the system default push, not allowed to cancel. */

  bool register_event_info(RobotEventCallback event_cb, void *arg);


  /*
   * Define the function pointer for the waypoint information push */
  void realtime_waypoint_cb(const aubo_robot_namespace::wayPoint_S *wayPoint, void *arg);


  /// The IO module ///

  /*
   * Get the status information of the one or multiple interface board I/O */

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

};

} // namespace
#endif
