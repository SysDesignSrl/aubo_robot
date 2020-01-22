#ifndef AUBO_AUBO_ROBOT_H
#define AUBO_AUBO_ROBOT_H
// STL
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

public:

  AuboRobot(const ros::NodeHandle &node = ros::NodeHandle()) :
    node(node),
    joint_trajectory_act(node, "joint_trajectory", boost::bind(&aubo::AuboRobot::move_trajectory, this, _1), false) { }


/* */
  bool init() {
    int error_code;

    // init parameters
    if (!node.getParam("joint_names", joint_names)) {
      ROS_FATAL("Failed to get parameter: '%s'", "joint_names");
      return false;
    }

    // init provided services
    login_srv = node.advertiseService("login", &aubo::AuboRobot::login, this);
    logout_srv = node.advertiseService("logout", &aubo::AuboRobot::logout, this);
    robot_startup_srv = node.advertiseService("robot_startup", &aubo::AuboRobot::robot_startup, this);
    robot_shutdown_srv = node.advertiseService("robot_shutdown", &aubo::AuboRobot::robot_shutdown, this);
    init_profile_srv = node.advertiseService("init_motion_profile", &aubo::AuboRobot::init_profile, this);

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


/* Establishing network connection with the manipulator sever */
  bool
  login(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {
    int error_code;

    auto hostname = node.param<std::string>("tcp/hostname", "localhost");
    auto port = node.param<int>("tcp/port", 8899);
    auto username = node.param<std::string>("login/username", "AUBO");
    auto password = node.param<std::string>("login/password", "123456");

    error_code = service_interface.robotServiceLogin(hostname.c_str(), port, username.c_str(), password.c_str());
    if (error_code != 0) {
      ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
      ROS_ERROR("Failed to login to %s:%d", hostname.c_str(), port);
      ROS_ERROR("Username %s", username.c_str());
      ROS_ERROR("Password %s", password.c_str());
      res.success = false;
      res.message = "Failed to log in.";
      return false;
    }

    ROS_INFO("Logged in to %s:%d", hostname.c_str(), port);
    ROS_INFO("Username: %s", username.c_str());
    ROS_INFO("Password: %s", password.c_str());
    res.success = true;
    res.message = "Logged in.";
    return true;
  }


/* Disconnecting from the manipulator server */
  bool
  logout(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {
    int error_code;

    error_code = service_interface.robotServiceLogout();
    if (error_code != 0) {
      ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
      ROS_ERROR("Failed to logout.");
      res.success = false;
      res.message = "Failed to log out.";
      return false;
    }

    ROS_INFO("Logged out.");
    res.success = true;
    res.message = "Logged out.";
    return true;
  }


/* Initializing the manipulator, including power on, release the brake, set up
 * the collision class, set up the kinematics parameters.
 * This function needs a longtime to complete, so user can set its block mode
 * to adjust the return time of the function. When it is set to unblocked mode,
 * it will return immediately after calling function, the result will be notified by event.
 * When it is set to block mode, return value represents whether the interface
 * has been called successfully. */
  bool
  robot_startup(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {
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

    uint8 collision_class = 0x06;

    aubo_robot_namespace::ROBOT_SERVICE_STATE result;

    error_code = service_interface.rootServiceRobotStartup(tool_dynamics_param, collision_class, true, true, 1000, result);
    if (error_code != 0) {
      ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
      ROS_ERROR("Failed to startup the Robot.");
      res.success = false;
      res.message = "Failed to log out.";
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

    ROS_INFO("Robot startup correctly.");
    res.success = true;
    res.message = "Robot startup correctly.";
    return true;
  }


/* Shutdown the manipulator, including power off, hold the brake */
  bool
  robot_shutdown(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {
    int error_code;

    error_code = service_interface.robotServiceRobotShutdown();
    if (error_code != 0) {
      ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
      ROS_ERROR("Failed to shutdown the Robot.");
      res.success = false;
      res.message = "Failed to shutdown the Robot.";
      return false;
    }

    ROS_INFO("Robot shutdown correctly.");
    res.success = true;
    res.message = "Robot shutdown correctly.";
    return true;
  }


/* Initialize the movement property, set properties to default. */
  bool
  init_profile(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {
    int error_code;

    error_code = service_interface.robotServiceInitGlobalMoveProfile();
    if (error_code != 0) {
      ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
      ROS_ERROR("Failed to initialize movement profile.");
      res.success = false;
      res.message = "Failed to initialize motion profile.";
      return false;
    }

    ROS_INFO("Movement profile initialized correctly.");
    res.success = true;
    res.message = "Movement profile initialized correctly.";
    return true;
  }


/* */
  void
  move_trajectory(const control_msgs::JointTrajectoryGoal::ConstPtr &goal) {
    int error_code;

    // clear waypoints
    service_interface.robotServiceClearGlobalWayPointVector();

    // add waypoints
    for (const trajectory_msgs::JointTrajectoryPoint &trajectory_point : goal->trajectory.points) {
      double jointAngle[aubo_robot_namespace::ARM_DOF];
      std::copy(trajectory_point.positions.cbegin(), trajectory_point.positions.cend(), jointAngle);

      error_code = service_interface.robotServiceAddGlobalWayPoint(jointAngle);
      if (error_code != 0) {
        ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
        ROS_ERROR("Failed to add robot waypoint.");
        joint_trajectory_act.setAborted();
      }
    }

    // start executing trajectory
    error_code = service_interface.robotServiceTrackMove(aubo_robot_namespace::move_track::CARTESIAN_MOVEP, false);
    if (error_code != 0) {
      ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
      ROS_ERROR("Failed start executing trajectory.");
      joint_trajectory_act.setAborted();
    }

    ROS_INFO("Start executing trajectory...");
    joint_trajectory_act.setSucceeded();
  }


/* Define the function pointer for the waypoint information push */
  void
  joints_real_time_callback(const aubo_robot_namespace::wayPoint_S *wayPoint, void *arg) {

    std::vector<double> joint_pos(aubo_robot_namespace::ARM_DOF);
    std::copy(wayPoint->jointpos, wayPoint->jointpos + aubo_robot_namespace::ARM_DOF, joint_pos.begin());

    sensor_msgs::JointState joint_state;
    joint_state.name = joint_names;
    joint_state.position = joint_pos;
    joint_state_pub.publish(joint_state);
  }


/* Get the joint angle of the manipulator */
  void get_joint_angle() {
    int error_code;

    aubo_robot_namespace::JointParam jointParam;
    error_code = service_interface.robotServiceGetJointAngleInfo(jointParam);
    if (error_code != 0) {
      ROS_DEBUG("error_code: %d, %s", error_code, error_codes[error_code].c_str());
      ROS_ERROR("Failed to get current joint angles.");
      return;
    }

    std::vector<double> joint_pos(aubo_robot_namespace::ARM_DOF);
    std::copy(jointParam.jointPos, jointParam.jointPos + aubo_robot_namespace::ARM_DOF, joint_pos.begin());

    sensor_msgs::JointState joint_state;
    joint_state.name = joint_names;
    joint_state.position = joint_pos;
    joint_state_pub.publish(joint_state);
  }


/* Get the waypoint information of the manipulator */
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
    joint_state.name = joint_names;
    joint_state.position = joint_pos;
    joint_state_pub.publish(joint_state);
  }


};

} // namespace
#endif
