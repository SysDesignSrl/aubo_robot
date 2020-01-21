// STL
#include <string>
#include <vector>
#include <algorithm>
// roscpp
#include <ros/ros.h>
#include <ros/console.h>
// hardware_interface
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
// aubo_hardware_interface
#include "aubo_hardware_interface/aubo_driver.h"


namespace aubo_hardware_interface {

class AuboHardwareInterface : public hardware_interface::RobotHW {
private:
  aubo::AuboDriver aubo_driver;
  aubo_driver::ControlOption control_option;


  ros::NodeHandle node;

  int n_joints;

  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;

  std::vector<double> j_pos, j_pos_cmd;
  std::vector<double> j_vel, j_vel_cmd;
  std::vector<double> j_eff, j_eff_cmd;


public:
  double loop_hz;
  std::vector<std::string> joint_names;

  AuboHardwareInterface(const ros::NodeHandle &node) : node(node) {

    init();
  }


  bool init() {

    if (!node.getParam("/aubo/hardware_interface/loop_hz", loop_hz)) {
      ROS_ERROR("Failed to retrieve '/aubo/hardware_interface/loop_hz' parameter.");
    }

    if (!node.getParam("/aubo/hardware_interface/joints", joint_names)) {
      ROS_ERROR("Failed to retrieve '/aubo/hardware_interface/joints' parameter.");
    }

    n_joints = joint_names.size();

    j_pos.resize(n_joints, 0); j_pos_cmd.resize(n_joints, 0);
    j_vel.resize(n_joints, 0); j_vel_cmd.resize(n_joints, 0);
    j_eff.resize(n_joints, 0); j_eff_cmd.resize(n_joints, 0);

    for (int i=0; i < n_joints; i++) {

      hardware_interface::JointStateHandle jnt_state_handle(joint_names[i], &j_pos[i], &j_vel[i], &j_eff[i]);
      jnt_state_interface.registerHandle(jnt_state_handle);

      hardware_interface::JointHandle jnt_pos_handle(jnt_state_handle, &j_pos_cmd[i]);
      jnt_pos_interface.registerHandle(jnt_pos_handle);

      hardware_interface::JointHandle jnt_vel_handle(jnt_state_handle, &j_vel_cmd[i]);
      jnt_vel_interface.registerHandle(jnt_vel_handle);
    }

    registerInterface(&jnt_state_interface);
    registerInterface(&jnt_pos_interface);
    registerInterface(&jnt_vel_interface);
  }


  bool start() {
    int ret = -1;

    auto hostname = node.param<std::string>("/aubo_driver/tcp/hostname", "localhost");
    auto port = node.param<int>("/aubo_driver/tcp/port", 8899);

    // connect to the robot controller
    if (!aubo_driver.login(hostname, port)) {
      ros::param::set("/aubo_driver/robot_connected", false);
      ROS_WARN("Failed to connect to %s:%d", hostname.c_str(), port);
      return false;
    }

    ROS_INFO("Connected to %s:%d", hostname.c_str(), port);

    // switch to ros-controller
    int count = 2;
    do {
      ret = aubo_driver.robot_send_service.robotServiceEnterTcp2CanbusMode();
      count--;
    } while (ret != aubo_robot_namespace::InterfaceCallSuccCode && count < 0);

    if (ret == aubo_robot_namespace::InterfaceCallSuccCode) {
      control_option = aubo_driver::RosMoveIt;
      ROS_INFO("Switches to ros-controller successfully");
    }
    if (ret == aubo_robot_namespace::ErrCode_ResponseReturnError) {
      // already connect, disconnect first.
      ret = aubo_driver.robot_send_service.robotServiceLeaveTcp2CanbusMode();

      control_option = aubo_driver::AuboAPI;
      ROS_WARN("Failed to switch to ros-controller, the robot is still controlled by the robot controller!");
    }
  }


  void read() {
    int ret = -1;

    if (!aubo_driver.controller_connected_flag) {

    }

    /** Query the states of robot joints **/
    ret = aubo_driver.robot_receive_service.robotServiceGetCurrentWaypointInfo(aubo_driver.robot_state.wayPoint_);

    if (ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        for (int i=0; i < n_joints; i++) {
          j_pos[i] = aubo_driver.robot_state.wayPoint_.jointpos[i];
        }

        /** Get the buff size of thr rib **/
        //robot_receive_service_.robotServiceGetRobotDiagnosisInfo(rs.robot_diagnosis_info_);
        //rib_buffer_size_ = rs.robot_diagnosis_info_.macTargetPosDataSize;

//      robot_receive_service_.robotServiceGetRobotCurrentState(rs.state_);            // this is controlled by Robot Controller
//      robot_receive_service_.getErrDescByCode(rs.code_);
        // if (real_robot_exist_)
        // {
        //     // publish robot_status information to the controller action server.
        //     robot_status_.mode.val            = (int8)rs.robot_diagnosis_info_.orpeStatus;
        //     robot_status_.e_stopped.val       = (int8)(rs.robot_diagnosis_info_.softEmergency || emergency_stopped_);
        //     robot_status_.drives_powered.val  = (int8)rs.robot_diagnosis_info_.armPowerStatus;
        //     robot_status_.motion_possible.val = (int)(!start_move_);
        //     robot_status_.in_motion.val       = (int)start_move_;
        //     robot_status_.in_error.val        = (int)protective_stopped_;   //used for protective stop.
        //     robot_status_.error_code          = (int32)rs.robot_diagnosis_info_.singularityOverSpeedAlarm;
        // }
    }
    else if (ret == aubo_robot_namespace::ErrCode_SocketDisconnect)
    {
      /** Here we check the connection to satisfy the ROS-I specification **/
      /** Try to connect with the robot again **/
    }
  }


  void write() {
    int ret = -1;

    ret = aubo_driver.robot_send_service.robotServiceSetRobotPosData2Canbus(j_pos_cmd.data());
  }

};

} // namespace
