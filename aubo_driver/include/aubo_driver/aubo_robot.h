#ifndef AUBO_AUBO_ROBOT_H
#define AUBO_AUBO_ROBOT_H
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
// roscpp
#include <ros/ros.h>
#include <ros/console.h>
// xmlrpcpp
#include <XmlRpcValue.h>
#include <XmlRpcException.h>
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

  ServiceInterface service_interface;

  // Action Servers
  actionlib::SimpleActionServer<control_msgs::JointTrajectoryAction> joint_trajectory_asrv;

  // Diagnostic Info
  struct
  {
    bool arm_power_status;              // The switch status (on, off) of robot 48V power
    double arm_power_current;           // The current of robot 48V power
    double arm_power_voltage;           // The voltage of robot 48V power
    uint8 arm_canbus_status;            // 0x00: No error 0xff: CAN bus error

    bool remote_halt;                   // Remote halt signal
    bool soft_emergency;                // Robot soft emergency
    bool remote_emergency;              // Remote emergency sugnal

    bool robot_collision;               // Collision detection bit
    bool static_collision;              // Static collision detection switch
    uint8 joint_collision;              // Joint collision detection, each joint occupies 1 bit, 0-collision inexistence 1-collision existence

    bool force_control_mode;            // The flag bit of robot starting force control mode
    bool brake_status;                  // Brake status
    bool orpe_status;                   // The status bit of the software (ORPE)

    bool encoder_error;                 // Magnetic encoder error status
    bool encoder_lines_error;           // Optical-electricity encoders are not same, 0-no error, 1-error
    bool joint_error;                   // Joint error status
    uint8 tool_io_error;                // Tool error

    bool singularity_overspeed;         // The overspeed alarm of robot singularity
    bool robot_overcurrent;             // The alarm of robot current flow

    bool robot_mounting_pose_warning;   // The mounting position of the robot is wrong (Working on the force control only)

    uint16 can_buffer_size;             // The size of the mac buffer
    uint16 can_data_size;               // The valid data size of the mac buffer
    uint8 can_data_warning;             // The mac data interruption
  } robot_diagnostic;


  ros::Publisher ai_00_pub;
  ros::Publisher ai_01_pub;
  ros::Publisher ai_02_pub;
  ros::Publisher ai_03_pub;

  ros::Subscriber ao_00_sub;
  ros::Subscriber ao_01_sub;
  ros::Subscriber ao_02_sub;
  ros::Subscriber ao_03_sub;


  void refresh_cycle_cb(const ros::TimerEvent &ev)
  {
    // publish_digital_inputs();
    // publish_analog_inputs();
  }


  // void publish_digital_inputs()
  // {
  //   bool val;
  //   std_msgs::Bool msg;
  //
  //   get_digital_input("U_DI_00", val);
  //   msg.data = val;
  //   di_00_pub.publish(msg);
  //   get_digital_input("U_DI_01", val);
  //   msg.data = val;
  //   di_01_pub.publish(msg);
  //   get_digital_input("U_DI_02", val);
  //   msg.data = val;
  //   di_02_pub.publish(msg);
  //   get_digital_input("U_DI_03", val);
  //   msg.data = val;
  //   di_03_pub.publish(msg);
  //   get_digital_input("U_DI_04", val);
  //   msg.data = val;
  //   di_04_pub.publish(msg);
  //   get_digital_input("U_DI_05", val);
  //   msg.data = val;
  //   di_05_pub.publish(msg);
  //   get_digital_input("U_DI_06", val);
  //   msg.data = val;
  //   di_06_pub.publish(msg);
  //   get_digital_input("U_DI_07", val);
  //   msg.data = val;
  //   di_07_pub.publish(msg);
  //   get_digital_input("U_DI_10", val);
  //   msg.data = val;
  //   di_10_pub.publish(msg);
  //   get_digital_input("U_DI_11", val);
  //   msg.data = val;
  //   di_11_pub.publish(msg);
  //   get_digital_input("U_DI_12", val);
  //   msg.data = val;
  //   di_12_pub.publish(msg);
  //   get_digital_input("U_DI_13", val);
  //   msg.data = val;
  //   di_13_pub.publish(msg);
  //   get_digital_input("U_DI_14", val);
  //   msg.data = val;
  //   di_14_pub.publish(msg);
  //   get_digital_input("U_DI_15", val);
  //   msg.data = val;
  //   di_15_pub.publish(msg);
  //   get_digital_input("U_DI_16", val);
  //   msg.data = val;
  //   di_16_pub.publish(msg);
  //   get_digital_input("U_DI_17", val);
  //   msg.data = val;
  //   di_17_pub.publish(msg);
  // }


  // void publish_analog_inputs()
  // {
  //   double val;
  //   std_msgs::Float64 msg;
  //
  //   get_analog_input("VI0", val);
  //   msg.data = val;
  //   ai_00_pub.publish(msg);
  //   get_analog_input("VI1", val);
  //   msg.data = val;
  //   ai_01_pub.publish(msg);
  //   get_analog_input("VI2", val);
  //   msg.data = val;
  //   ai_02_pub.publish(msg);
  //   get_analog_input("VI3", val);
  //   msg.data = val;
  //   ai_03_pub.publish(msg);
  // }

  // void ao_00_cb(const std_msgs::Float64::ConstPtr &msg) { set_analog_output("VO1", msg->data); }
  // void ao_01_cb(const std_msgs::Float64::ConstPtr &msg) { set_analog_output("VO2", msg->data); }
  // void ao_02_cb(const std_msgs::Float64::ConstPtr &msg) { set_analog_output("VO3", msg->data); }
  // void ao_03_cb(const std_msgs::Float64::ConstPtr &msg) { set_analog_output("VO4", msg->data); }

public:

  ros::Publisher joint_state_pub;
  ros::Publisher tool_pose_pub;
  ros::Publisher digital_input_pub;
  ros::Publisher diagnostic_pub;

  std::vector<std::string> joint_names;


  AuboRobot(const ros::NodeHandle &node = ros::NodeHandle()) :
    node(node),
    joint_trajectory_asrv(node, "joint_trajectory", boost::bind(&aubo::AuboRobot::execute_trajectory, this, _1), false) { }


  void velocity_scaling_cb(const std_msgs::Float64::ConstPtr &msg)
  {
    double scaling_factor = msg->data;

    scaling_factor = std::max(0.0, scaling_factor);
    scaling_factor = std::min(scaling_factor, 1.0);

    std::vector<double> max_joint_vel;
    if (node.getParamCached("aubo/max_joint_velocity", max_joint_vel))
    {
      std::vector<double> joint_vel;
      joint_vel.resize(max_joint_vel.size());

      std::transform(max_joint_vel.begin(), max_joint_vel.end(), joint_vel.begin(), [scaling_factor](double value) { return scaling_factor * value; } );

      set_max_joint_velocity(joint_vel);
    }
  }


  void acceleration_scaling_cb(const std_msgs::Float64::ConstPtr &msg)
  {
    double scaling_factor = msg->data;

    scaling_factor = std::max(0.0, scaling_factor);
    scaling_factor = std::min(scaling_factor, 1.0);

    std::vector<double> max_joint_acc;
    if (node.getParamCached("aubo/max_joint_acceleration", max_joint_acc))
    {
      std::vector<double> joint_acc;
      joint_acc.resize(max_joint_acc.size());

      std::transform(max_joint_acc.begin(), max_joint_acc.end(), joint_acc.begin(), [scaling_factor](double value) { return scaling_factor * value; } );

      set_max_joint_acceleration(joint_acc);
    }
  }


  void digital_output_cb(const std_msgs::UInt16::ConstPtr &msg)
  {
    ROS_DEBUG("Digital Outputs: 0x%.4X", msg->data);
    set_digital_output("U_DO_00", (msg->data >> 0) & 0x01);
    set_digital_output("U_DO_01", (msg->data >> 1) & 0x01);
    set_digital_output("U_DO_02", (msg->data >> 2) & 0x01);
    set_digital_output("U_DO_03", (msg->data >> 3) & 0x01);
    set_digital_output("U_DO_04", (msg->data >> 4) & 0x01);
    set_digital_output("U_DO_05", (msg->data >> 5) & 0x01);
    set_digital_output("U_DO_06", (msg->data >> 6) & 0x01);
    set_digital_output("U_DO_07", (msg->data >> 7) & 0x01);
    set_digital_output("U_DO_10", (msg->data >> 8) & 0x01);
    set_digital_output("U_DO_11", (msg->data >> 9) & 0x01);
    set_digital_output("U_DO_12", (msg->data >> 10) & 0x01);
    set_digital_output("U_DO_13", (msg->data >> 11) & 0x01);
    set_digital_output("U_DO_14", (msg->data >> 12) & 0x01);
    set_digital_output("U_DO_15", (msg->data >> 13) & 0x01);
    set_digital_output("U_DO_16", (msg->data >> 14) & 0x01);
    set_digital_output("U_DO_17", (msg->data >> 15) & 0x01);
  }


  bool init(const std::vector<std::string> &joint_names)
  {
    this->joint_names = joint_names;

    // di_00_pub = node.advertise<std_msgs::Bool>("DI/00", 1);
    // di_01_pub = node.advertise<std_msgs::Bool>("DI/01", 1);
    // di_02_pub = node.advertise<std_msgs::Bool>("DI/02", 1);
    // di_03_pub = node.advertise<std_msgs::Bool>("DI/03", 1);
    // di_04_pub = node.advertise<std_msgs::Bool>("DI/04", 1);
    // di_05_pub = node.advertise<std_msgs::Bool>("DI/05", 1);
    // di_06_pub = node.advertise<std_msgs::Bool>("DI/06", 1);
    // di_07_pub = node.advertise<std_msgs::Bool>("DI/07", 1);
    //
    // di_10_pub = node.advertise<std_msgs::Bool>("DI/10", 1);
    // di_11_pub = node.advertise<std_msgs::Bool>("DI/11", 1);
    // di_12_pub = node.advertise<std_msgs::Bool>("DI/12", 1);
    // di_13_pub = node.advertise<std_msgs::Bool>("DI/13", 1);
    // di_14_pub = node.advertise<std_msgs::Bool>("DI/14", 1);
    // di_15_pub = node.advertise<std_msgs::Bool>("DI/15", 1);
    // di_16_pub = node.advertise<std_msgs::Bool>("DI/16", 1);
    // di_17_pub = node.advertise<std_msgs::Bool>("DI/17", 1);
    //
    // do_00_sub = node.subscribe<std_msgs::Bool>("DO/00", 1, &aubo::AuboRobot::do_00_cb, this);
    // do_01_sub = node.subscribe<std_msgs::Bool>("DO/01", 1, &aubo::AuboRobot::do_01_cb, this);
    // do_02_sub = node.subscribe<std_msgs::Bool>("DO/02", 1, &aubo::AuboRobot::do_02_cb, this);
    // do_03_sub = node.subscribe<std_msgs::Bool>("DO/03", 1, &aubo::AuboRobot::do_03_cb, this);
    // do_04_sub = node.subscribe<std_msgs::Bool>("DO/04", 1, &aubo::AuboRobot::do_04_cb, this);
    // do_05_sub = node.subscribe<std_msgs::Bool>("DO/05", 1, &aubo::AuboRobot::do_05_cb, this);
    // do_06_sub = node.subscribe<std_msgs::Bool>("DO/06", 1, &aubo::AuboRobot::do_06_cb, this);
    // do_07_sub = node.subscribe<std_msgs::Bool>("DO/07", 1, &aubo::AuboRobot::do_07_cb, this);
    // do_10_sub = node.subscribe<std_msgs::Bool>("DO/10", 1, &aubo::AuboRobot::do_10_cb, this);
    // do_11_sub = node.subscribe<std_msgs::Bool>("DO/11", 1, &aubo::AuboRobot::do_11_cb, this);
    // do_12_sub = node.subscribe<std_msgs::Bool>("DO/12", 1, &aubo::AuboRobot::do_12_cb, this);
    // do_13_sub = node.subscribe<std_msgs::Bool>("DO/13", 1, &aubo::AuboRobot::do_13_cb, this);
    // do_14_sub = node.subscribe<std_msgs::Bool>("DO/14", 1, &aubo::AuboRobot::do_14_cb, this);
    // do_15_sub = node.subscribe<std_msgs::Bool>("DO/15", 1, &aubo::AuboRobot::do_15_cb, this);
    // do_16_sub = node.subscribe<std_msgs::Bool>("DO/16", 1, &aubo::AuboRobot::do_16_cb, this);
    // do_17_sub = node.subscribe<std_msgs::Bool>("DO/17", 1, &aubo::AuboRobot::do_17_cb, this);
    //
    // ai_00_pub = node.advertise<std_msgs::Float64>("AI/00", 1);
    // ai_01_pub = node.advertise<std_msgs::Float64>("AI/01", 1);
    // ai_02_pub = node.advertise<std_msgs::Float64>("AI/02", 1);
    // ai_03_pub = node.advertise<std_msgs::Float64>("AI/03", 1);
    //
    // ao_00_sub = node.subscribe<std_msgs::Float64>("AO/00", 1, &aubo::AuboRobot::ao_00_cb, this);
    // ao_01_sub = node.subscribe<std_msgs::Float64>("AO/01", 1, &aubo::AuboRobot::ao_01_cb, this);
    // ao_02_sub = node.subscribe<std_msgs::Float64>("AO/02", 1, &aubo::AuboRobot::ao_02_cb, this);
    // ao_03_sub = node.subscribe<std_msgs::Float64>("AO/03", 1, &aubo::AuboRobot::ao_03_cb, this);

    return true;
  }


  bool init_robot()
  {
    // initialize movement profile
    if (init_profile())
    {
      ROS_DEBUG("Initialized robot movement profile");
    }
    else
    {
      ROS_ERROR("Failed to initialize robot movement profile");
      return false;
    }

    // set joint parameters
    std::vector<double> max_joint_vel;
    if (node.getParam("aubo/max_joint_velocity", max_joint_vel))
    {
      set_max_joint_velocity(max_joint_vel);
    }
    std::vector<double> max_joint_acc;
    if (node.getParam("aubo/max_joint_acceleration", max_joint_acc))
    {
      set_max_joint_acceleration(max_joint_acc);
    }

    // get joint parameters
    get_max_joint_velocity(max_joint_vel);
    std::stringstream vss;
    vss << "[ ";
    for (double val : max_joint_vel)
    {
      vss << val << " ";
    }
    vss << "] ";
    ROS_DEBUG_STREAM("max joint velocity: " << vss.str() << "[rad/s]");

    get_max_joint_acceleration(max_joint_acc);
    std::stringstream ass;
    ass << "[ ";
    for (double val : max_joint_acc)
    {
      ass << val << " ";
    }
    ass << "] ";
    ROS_DEBUG_STREAM("max joint acceleration: " << ass.str() << "[rad/s^2]");

    // set linear parameters
    double max_linear_vel;
    if (node.getParam("aubo/max_linear_velocity", max_linear_vel))
    {
      set_max_linear_velocity(max_linear_vel);
    }
    double max_linear_acc;
    if (node.getParam("aubo/max_linear_acceleration", max_linear_acc))
    {
      set_max_linear_acceleration(max_linear_acc);
    }
    double max_angular_vel;
    if (node.getParam("aubo/max_angular_velocity", max_angular_vel))
    {
      set_max_angular_velocity(max_angular_vel);
    }
    double max_angular_acc;
    if (node.getParam("aubo/max_angular_acceleration", max_angular_acc))
    {
      set_max_angular_acceleration(max_angular_acc);
    }

    // get linear parameters
    get_max_linear_velocity(max_linear_vel);
    ROS_DEBUG("max linear velocity: %.2f [m/s]", max_linear_vel);

    get_max_linear_acceleration(max_linear_acc);
    ROS_DEBUG("max linear acceleration: %.2f [m/s^2]", max_linear_acc);

    get_max_angular_velocity(max_angular_vel);
    ROS_DEBUG("max angular velocity: %.2f [rad/s]", max_angular_vel);

    get_max_angular_acceleration(max_angular_acc);
    ROS_DEBUG("max angular acceleration: %.2f [rad/s^2]", max_angular_acc);

    // blend radius
    double blend_radius;
    if (node.getParam("aubo/blend_radius", blend_radius))
    {
      set_blend_radius(blend_radius);
    }
    ROS_DEBUG("blend_radius: %.2f [m]", get_blend_radius());
  }


  bool start()
  {
    joint_trajectory_asrv.start();
    return true;
  }


  void execute_trajectory(const control_msgs::JointTrajectoryGoal::ConstPtr &goal)
  {
    if (move_track(goal->trajectory))
    {
      ROS_INFO("Trajectory executed succesfully.");
      joint_trajectory_asrv.setSucceeded();
    }
    else
    {
      ROS_ERROR("Failed to execute trajectory.");
      joint_trajectory_asrv.setAborted();
    }
  }

  /*
   * Establishing network connection with the manipulator sever */
  bool login(std::string host, int port = 8899, std::string username = "AUBO", std::string password = "123456");
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

  bool robot_startup(int collision_class = 6);
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

  bool set_blend_radius(double value);                // Set the blend radius.
  double get_blend_radius();                          // Get the blend radius.

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

  bool move_track(const trajectory_msgs::JointTrajectory &trajectory);

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
