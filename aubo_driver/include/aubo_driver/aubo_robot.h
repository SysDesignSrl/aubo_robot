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
// sensor_msgs
#include <sensor_msgs/JointState.h>
// trajectory_msgs
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
// control_msgs
#include <control_msgs/JointTrajectoryAction.h>
// aubo_driver
#include "aubo_driver/error_codes.h"
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
  bool init() {

    // init parameters
    if (!node.getParam("joint_names", joint_names)) {
      ROS_FATAL("Failed to get parameter: '%s'", "joint_names");
      return false;
    }

    collision_class = node.param<int>("aubo/collision_class", 6);
    blend_radius = node.param<double>("aubo/blend_radius", 0.02);


    // init provided services
    login_srv = node.advertiseService("login", &aubo::AuboRobot::login, this);
    logout_srv = node.advertiseService("logout", &aubo::AuboRobot::logout, this);
    robot_startup_srv = node.advertiseService("robot_startup", &aubo::AuboRobot::robot_startup, this);
    robot_shutdown_srv = node.advertiseService("robot_shutdown", &aubo::AuboRobot::robot_shutdown, this);
    init_profile_srv = node.advertiseService("init_profile", &aubo::AuboRobot::init_profile, this);

    // init published topics
    joint_state_pub = node.advertise<sensor_msgs::JointState>("joint_states", 100);

    // start action server
    joint_trajectory_act.start();

    // error_code = service_interface.robotServiceSetRealTimeRoadPointPush(true);
    // if (error_code != 0) {
    //   ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
    //   ROS_ERROR("Failed to enable joints real-time callback.");
    //   return false;
    // }
    //
    // error_code = service_interface.robotServiceRegisterRealTimeRoadPointCallback(AuboRobot::joints_real_time_callback);
    // if (error_code != 0) {
    //   ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
    //   ROS_ERROR("Failed to resister joints real-time callback.");
    //   return false;
    // }

    //ROS_INFO("Joints real-time callback registered.");
    return true;
  }


  /*
   * Establishing network connection with the manipulator sever */

   bool login(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {
     auto username = node.param<std::string>("login/username", "AUBO");
     auto password = node.param<std::string>("login/password", "123456");

     if (login(username, password)) {
       res.success = true;
       res.message = "Logged in.";
     }
     else {
       res.success = false;
       res.message = "Failed to log in.";
     }

     return true;
   }

  bool login(std::string username = "AUBO", std::string password = "123456") {
    int error_code;

    auto hostname = node.param<std::string>("tcp/hostname", "localhost");
    auto port = node.param<int>("tcp/port", 8899);

    error_code = service_interface.robotServiceLogin(hostname.c_str(), port, username.c_str(), password.c_str());
    if (error_code != 0) {
      ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
      ROS_ERROR("Failed to login to %s:%d", hostname.c_str(), port);
      return false;
    }

    ROS_INFO("Logged in to %s:%d", hostname.c_str(), port);
    return true;
  }


  /*
   * Disconnecting from the manipulator server */

  bool logout(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {
    if (logout()) {
      res.success = true;
      res.message = "Logged out.";
    }
    else {
      res.success = false;
      res.message = "Failed to log out.";
    }

    return true;
  }

  bool logout() {
    int error_code;

    error_code = service_interface.robotServiceLogout();
    if (error_code != 0) {
      ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
      return false;
    }

    return true;
  }


/* Initializing the manipulator, including power on, release the brake, set up
 * the collision class, set up the kinematics parameters.
 * This function needs a longtime to complete, so user can set its block mode
 * to adjust the return time of the function. When it is set to unblocked mode,
 * it will return immediately after calling function, the result will be notified by event.
 * When it is set to block mode, return value represents whether the interface
 * has been called successfully. */

 bool robot_startup(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {

   if (robot_startup()) {
     res.success = true;
     res.message = "Robot startup correctly.";
   }
   else {
     res.success = false;
     res.message = "Failed to startup the Robot.";
   }

   return true;
 }

  bool robot_startup() {
    int error_code;

    aubo_robot_namespace::ToolDynamicsParam tool_dynamics_param;
    tool_dynamics_param.positionX = 0.0;
    tool_dynamics_param.positionY = 0.0;
    tool_dynamics_param.positionZ = 0.0;
    tool_dynamics_param.payload = 0.0;
    tool_dynamics_param.toolInertia.xx = 0.0;
    tool_dynamics_param.toolInertia.xy = 0.0;
    tool_dynamics_param.toolInertia.xz = 0.0;
    tool_dynamics_param.toolInertia.yy = 0.0;
    tool_dynamics_param.toolInertia.yz = 0.0;
    tool_dynamics_param.toolInertia.zz = 0.0;

    aubo_robot_namespace::ROBOT_SERVICE_STATE result;

    error_code = service_interface.rootServiceRobotStartup(tool_dynamics_param, collision_class, true, true, 1000, result);
    if (error_code != 0) {
      ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
      return false;
    }

    switch (result)
    {
      case aubo_robot_namespace::ROBOT_SERVICE_STATE::ROBOT_SERVICE_READY:
        ROS_INFO("ROBOT SERVICE STATE: %s", "READY");
        break;
      case aubo_robot_namespace::ROBOT_SERVICE_STATE::ROBOT_SERVICE_STARTING:
        ROS_INFO("ROBOT SERVICE STATE: %s", "STARTING");
        break;
      case aubo_robot_namespace::ROBOT_SERVICE_STATE::ROBOT_SERVICE_WORKING:
        ROS_INFO("ROBOT SERVICE STATE: %s", "WORKING");
        break;
      case aubo_robot_namespace::ROBOT_SERVICE_STATE::ROBOT_SERVICE_CLOSING:
        ROS_INFO("ROBOT SERVICE STATE: %s", "CLOSING");
        break;
      case aubo_robot_namespace::ROBOT_SERVICE_STATE::ROBOT_SERVICE_CLOSED:
        ROS_INFO("ROBOT SERVICE STATE: %s", "CLOSED");
        break;
      case aubo_robot_namespace::ROBOT_SERVICE_STATE::ROBOT_SETVICE_FAULT_POWER:
        ROS_ERROR("ROBOT SERVICE STATE: %s", "FAULT POWER");
        break;
      case aubo_robot_namespace::ROBOT_SERVICE_STATE::ROBOT_SETVICE_FAULT_BRAKE:
        ROS_ERROR("ROBOT SERVICE STATE: %s", "FAULT BRAKE");
        break;
      case aubo_robot_namespace::ROBOT_SERVICE_STATE::ROBOT_SETVICE_FAULT_NO_ROBOT:
        ROS_ERROR("ROBOT SERVICE STATE: %s", "FAULT NO ROBOT");
        break;
    }

    return true;
  }


  /*
   * Shutdown the manipulator, including power off, hold the brake */

  bool robot_shutdown(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {

    if (robot_shutdown()) {
      res.success = true;
      res.message = "Robot shutdown correctly.";
    }
    else {
      res.success = false;
      res.message = "Failed to shutdown the Robot.";
    }

    return true;
  }

  bool robot_shutdown() {
    int error_code;

    error_code = service_interface.robotServiceRobotShutdown();
    if (error_code != 0) {
      ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
      return false;
    }

    return true;
  }


  /*
  * Initialize the movement property, set properties to default. */

  bool init_profile(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {

    if (init_profile()) {
      res.success = true;
      res.message = "Movement profile initialized correctly.";
    }
    else {
      res.success = false;
      res.message = "Failed to initialize movement profile.";
    }

    return true;
  }

  bool init_profile() {
    int error_code;

    error_code = service_interface.robotServiceInitGlobalMoveProfile();
    if (error_code != 0) {
      ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
      return false;
    }

    return true;
  }


  /* Set the maximum acceleration of each joint. */
  bool set_max_joint_acceleration(const std::vector<double> &value) {
    int error_code;

    aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
    std::copy(value.begin(), value.end(), jointMaxAcc.jointPara);

    error_code = service_interface.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);
    if (error_code != 0) {
      ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
      return false;
    }

    return true;
  }

  /* Get the maximum acceleration of each joint. */
  void get_max_joint_acceleration(std::vector<double> &result) {

    aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
    service_interface.robotServiceGetGlobalMoveJointMaxAcc(jointMaxAcc);

    result.resize(aubo_robot_namespace::ARM_DOF);
    std::copy(jointMaxAcc.jointPara, jointMaxAcc.jointPara + aubo_robot_namespace::ARM_DOF, result.begin());
  }


  /* Set the maximum velocity of each joint. */
  bool set_max_joint_velocity(const std::vector<double> &value) {
    int error_code;

    aubo_robot_namespace::JointVelcAccParam jointMaxVel;
    std::copy(value.begin(), value.end(), jointMaxVel.jointPara);

    error_code = service_interface.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVel);
    if (error_code != 0) {
      ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
      return false;
    }

    return true;
  }

  /* Get the maximum velocity of each joint. */
  void get_max_joint_velocity(std::vector<double> &result) {

    aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
    service_interface.robotServiceGetGlobalMoveJointMaxVelc(jointMaxAcc);

    result.resize(aubo_robot_namespace::ARM_DOF);
    std::copy(jointMaxAcc.jointPara, jointMaxAcc.jointPara + aubo_robot_namespace::ARM_DOF, result.begin());
  }


  /* Set the maximum linear acceleration of end-effector movement. */
  bool set_max_linear_acceleration(double value) {
    int error_code;

    error_code = service_interface.robotServiceSetGlobalMoveEndMaxLineAcc(value);
    if (error_code != 0) {
      ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
      return false;
    }

    return true;
  }

  /* Get the maximum linear acceleration of end-effector movement. */
  void get_max_linear_acceleration(double &result) {
    service_interface.robotServiceGetGlobalMoveEndMaxLineAcc(result);
  }


  /* Set the maximum linear velocity of end-effector movement. */
  bool set_max_linear_velocity(double value) {
    int error_code;

    error_code = service_interface.robotServiceSetGlobalMoveEndMaxLineVelc(value);
    if (error_code != 0) {
      ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
      return false;
    }

    return true;
  }

  /* Get the maximum linear velocity of end-effector movement. */
  void get_max_linear_velocity(double &result) {
    service_interface.robotServiceGetGlobalMoveEndMaxLineVelc(result);
  }


  /* Set the maximum angular acceleration of end-effector movement. */
  bool set_max_angular_acceleration(double value) {
    int error_code;

    error_code = service_interface.robotServiceSetGlobalMoveEndMaxAngleAcc(value);
    if (error_code != 0) {
      ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
      return false;
    }

    return true;
  }

  /* Get the maximum angular acceleration of end-effector movement. */
  void get_max_angular_acceleration(double &result) {
    service_interface.robotServiceGetGlobalMoveEndMaxAngleAcc(result);
  }


  /* Set the maximum angular velocity of end-effector movement. */
  bool set_max_angular_velocity(double value) {
    int error_code;

    error_code = service_interface.robotServiceSetGlobalMoveEndMaxAngleVelc(value);
    if (error_code != 0) {
      ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
      return false;
    }

    return true;
  }

  /* Get the maximum angular velocity of end-effector movement. */
  void get_max_angular_velocity(double &result) {
    service_interface.robotServiceGetGlobalMoveEndMaxAngleVelc(result);
  }


/* The manipulator moves to the target position through the joint movement
 * (Move Joint), the target position is described by the angle of each joint.
 * The maximum velocity and the maximum acceleration of the joint movement
 * should be set before calling it, and the offset attribute should be set if
 * the offset is used. The input has two option, one is the waypoint, or directly
 * gives the angle of each joint. */

  bool move_joint(const std::vector<double> &joint_pos) {
    int error_code;

    double jointAngle[aubo_robot_namespace::ARM_DOF];
    std::copy(joint_pos.cbegin(), joint_pos.cend(), jointAngle);

    error_code = service_interface.robotServiceJointMove(jointAngle, false);
    if (error_code != 0) {
      ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
      return false;
    }

    return true;
  }


/* The manipulator moves to the target position through the linear movement
 * (Move Line), the target position is described by the angle of each joint.
 * The maximum linear velocity and the maximum linear acceleration should be set
 * before calling it, and the offset attribute should be set if the offset is used.
 * The input has two option, one is the waypoint, or directly gives the angle of
 * each joint. */

  bool move_line(const std::vector<double> &joint_pos) {
    int error_code;

    double jointAngle[aubo_robot_namespace::ARM_DOF];
    std::copy(joint_pos.cbegin(), joint_pos.cend(), jointAngle);

    error_code = service_interface.robotServiceLineMove(jointAngle, false);
    if (error_code != 0) {
      ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
      return false;
    }

    return true;
  }


/* The track movement of the manipulator. This movement is belonged to different
 * movement type according to the different type of subMoveMode. */

  void move_track(const control_msgs::JointTrajectoryGoal::ConstPtr &goal) {
    int error_code;

    // clear waypoints
    service_interface.robotServiceClearGlobalWayPointVector();

    // add trajectory waypoints
    for (int i = 0; i < goal->trajectory.points.size(); i++) {

      std::vector<double> joint_pos;
      std::vector<double> joint_vel;

      auto sorted_extract = [&] (const trajectory_msgs::JointTrajectory &trajectory, int index)
      {
        joint_pos.resize(joint_names.size());
        joint_vel.resize(joint_names.size());

        for (int i=0; i < joint_names.size(); i++)
          for (int j=0; j < trajectory.joint_names.size(); j++)
            if (joint_names[i] == trajectory.joint_names[j])
            {
              joint_pos[i] = trajectory.points[index].positions[j];
              joint_vel[i] = trajectory.points[index].velocities[j];
            }
      };

      sorted_extract(goal->trajectory, i);

      error_code = service_interface.robotServiceAddGlobalWayPoint(joint_pos.data());
      if (error_code != 0) {
        ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
        ROS_ERROR("Failed to add trajectory waypoint to the robot.");
        joint_trajectory_act.setAborted();
      }
    }

    // set blend rodius
    error_code = service_interface.robotServiceSetGlobalBlendRadius(blend_radius);
    if (error_code != 0) {
      ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
      ROS_ERROR("Failed to set blend radius to: %.2f", blend_radius);
      joint_trajectory_act.setAborted();
    }

    // start trajectory execution
    error_code = service_interface.robotServiceTrackMove(aubo_robot_namespace::move_track::JIONT_CUBICSPLINE, false);
    if (error_code != 0) {
      ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
      ROS_ERROR("Failed to start executing trajectory.");
      joint_trajectory_act.setAborted();
    }

    ROS_INFO("Started trajectory execution succesfully.");
    joint_trajectory_act.setSucceeded();
  }


  /*
   * Get the joint angles of the manipulator */
  bool get_joint_angle(std::vector<double> &joint_pos) {
    int error_code;

    aubo_robot_namespace::JointParam jointParam;
    error_code = service_interface.robotServiceGetJointAngleInfo(jointParam);
    if (error_code != 0) {
      ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
      return false;
    }

    joint_pos.resize(aubo_robot_namespace::ARM_DOF);
    std::copy(jointParam.jointPos, jointParam.jointPos + aubo_robot_namespace::ARM_DOF, joint_pos.begin());
    return true;
  }


  /*
   * Get the waypoint information of the manipulator */
  void get_current_waypoint() {
    int error_code;

    aubo_robot_namespace::wayPoint_S wayPoint;
    error_code = service_interface.robotServiceGetCurrentWaypointInfo(wayPoint);
    if (error_code != 0) {
      ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
      ROS_ERROR("Failed to get current waypoint.");
      return;
    }

    std::vector<double> joint_pos(aubo_robot_namespace::ARM_DOF);
    std::copy(wayPoint.jointpos, wayPoint.jointpos + aubo_robot_namespace::ARM_DOF, joint_pos.begin());

    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.name = joint_names;
    joint_state.position = joint_pos;
    joint_state_pub.publish(joint_state);
  }

  /*
   * Get the diagnostic information of the manipulator */
  void print_diagnostic_info() {
    int error_code;

    aubo_robot_namespace::RobotDiagnosis robotDiagnosis;
    error_code = service_interface.robotServiceGetRobotDiagnosisInfo(robotDiagnosis);
    if (error_code != 0) {
      ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
      ROS_ERROR("Failed to get Robot Diagnosis Info.");
      return;
    }

    std::stringstream ss;
    ss << " Arm CAN Bus Status: " << robotDiagnosis.armCanbusStatus << std::endl;
    ss << " Arm Power Current: " << robotDiagnosis.armPowerCurrent << std::endl;
    ss << " Arm Power Voltage: " << robotDiagnosis.armPowerVoltage << std::endl;
    ss << " Arm Power Status: " << robotDiagnosis.armPowerStatus << std::endl;
    ss << " Controller Temperature: " << robotDiagnosis.contorllerTemp << std::endl;
    ss << " Controller Humidity: " << robotDiagnosis.contorllerHumidity << std::endl;
    ss << " Remote Halt: " << robotDiagnosis.remoteHalt << std::endl;
    ss << " Soft Emergency: " << robotDiagnosis.softEmergency << std::endl;
    ss << " Remote Emergency: " << robotDiagnosis.remoteEmergency << std::endl;
    ss << " Force Control Mode: " << robotDiagnosis.forceControlMode << std::endl;
    ss << " Brake Status: " << robotDiagnosis.brakeStuats << std::endl;
    ss << " Robot End Speed: " << robotDiagnosis.robotEndSpeed << std::endl;
    ss << " Robot Max Acceleration: " << robotDiagnosis.robotMaxAcc << std::endl;
    ss << " ORPE (Software) Status: " << robotDiagnosis.orpeStatus << std::endl;
    ss << " Enable Read Pose: " << robotDiagnosis.enableReadPose << std::endl;
    ss << " Robot Mounting Pose Changed: " << robotDiagnosis.robotMountingPoseChanged << std::endl;
    ss << " Encoder Error Status: " << robotDiagnosis.encoderErrorStatus << std::endl;
    ss << " Static Collision Detect: " << robotDiagnosis.staticCollisionDetect << std::endl;
    ss << " Joint Collision Detect: " << robotDiagnosis.jointCollisionDetect << std::endl;
    ss << " Encoder Lines Error: " << robotDiagnosis.encoderLinesError << std::endl;
    ss << " Joint Error Status: " << robotDiagnosis.jointErrorStatus << std::endl;
    ss << " Singularity Overspeed Alarm: " << robotDiagnosis.singularityOverSpeedAlarm << std::endl;
    ss << " Robot Current Alarm: " << robotDiagnosis.robotCurrentAlarm << std::endl;
    ss << " Tool IO Error: " << robotDiagnosis.toolIoError << std::endl;
    ss << " Robot Mounting Pose Warning: " << robotDiagnosis.robotMountingPoseWarning << std::endl;
    ss << " MAC Target Pos Buffer Size: " << robotDiagnosis.macTargetPosBufferSize << std::endl;
    ss << " MAC Target Pos Data Size: " << robotDiagnosis.macTargetPosDataSize << std::endl;
    ss << " MAC Data Interrupt Warning: " << robotDiagnosis.macDataInterruptWarning << std::endl;

    std::cout << "AUBO Robot Diagnostic Info:\n" << ss.str();
    //ROS_DEBUG_STREAM("AUBO Robot Diagnostic Info:\n" << ss.str());
  }


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

 bool register_waipoint_callback(RealTimeRoadPointCallback ptr, void *arg) {
   int error_code;

   error_code = service_interface.robotServiceRegisterRealTimeRoadPointCallback(ptr, arg);
   if (error_code != 0) {
     ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
     return false;
   }

   return true;
 }


/* Registers the callback function for obtaining the event information of the manipulator.
 * After registering the callback function, the server pushes the event information in real time.
 * Regarding the event information pushing, it does not provide the interface
 * for changing whether it is allowed the information to be pushed because many
 * important notifications of the manipulator are implemented by pushing event information.
 * So, the event information is the system default push, not allowed to cancel. */

  bool register_event_info(RobotEventCallback ptr, void *arg) {
    int error_code;

    error_code = service_interface.robotServiceRegisterRobotEventInfoCallback(ptr, arg);
    if (error_code != 0) {
      ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
      return false;
    }

    return true;
  }


  /*
   * Define the function pointer for the waypoint information push */
  void real_time_waypoint_callback(const aubo_robot_namespace::wayPoint_S *wayPoint, void *arg) {

    std::vector<double> joint_pos(aubo_robot_namespace::ARM_DOF);
    std::copy(wayPoint->jointpos, wayPoint->jointpos + aubo_robot_namespace::ARM_DOF, joint_pos.begin());

    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.name = joint_names;
    joint_state.position = joint_pos;
    joint_state_pub.publish(joint_state);
  }

};

} // namespace
#endif
