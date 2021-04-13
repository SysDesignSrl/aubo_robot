#ifndef AUBO_HARDWARE_INTERFACE_H
#define AUBO_HARDWARE_INTERFACE_H
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <functional>
// Unix
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <errno.h>
// roscpp
#include <ros/ros.h>
#include <ros/console.h>
// std_msgs
#include <std_msgs/Bool.h>
// std_srvs
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
// hardware_interface
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
// controller manager
#include <controller_manager/controller_manager.h>
// aubo_hardware_interface
#include "aubo_hardware_interface/aubo_robot.h"
#include "aubo_hardware_interface/time.h"


namespace aubo_hardware_interface {

void* control_loop(void* arg);


class AuboHW : public hardware_interface::RobotHW {
private:

  ros::NodeHandle node;

  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;

  std::vector<double> j_pos, j_pos_cmd, j_pos_off;
  std::vector<double> j_vel, j_vel_cmd;
  std::vector<double> j_eff, j_eff_cmd;

public:
  // Controller Manager
  controller_manager::ControllerManager controller_manager;
  bool reset_controllers = true;

  aubo::AuboRobot robot;

  bool ros_control = false;
  unsigned long t_cycle = 5000000U; long t_offset = 0;

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
  }
  robot_diagnostic;


  AuboHW(const ros::NodeHandle &node = ros::NodeHandle()) :
    node(node),
    controller_manager(this, node) { }


  bool init(double loop_hz, const std::vector<std::string> &joints, const std::vector<double> &joints_offset)
  {
    t_cycle = (1.0 / loop_hz) * 1000000000U;

    const int n_joints = joints.size();

    j_pos.resize(n_joints, 0.0); j_pos_cmd.resize(n_joints, 0.0);
    j_vel.resize(n_joints, 0.0); j_vel_cmd.resize(n_joints, 0.0);
    j_eff.resize(n_joints, 0.0); j_eff_cmd.resize(n_joints, 0.0);

    for (int i = 0; i < n_joints; i++)
    {
      hardware_interface::JointStateHandle jnt_state_handle(joints[i], &j_pos[i], &j_vel[i], &j_eff[i]);
      jnt_state_interface.registerHandle(jnt_state_handle);

      hardware_interface::JointHandle jnt_pos_handle(jnt_state_handle, &j_pos_cmd[i]);
      jnt_pos_interface.registerHandle(jnt_pos_handle);

      hardware_interface::JointHandle jnt_vel_handle(jnt_state_handle, &j_vel_cmd[i]);
      jnt_vel_interface.registerHandle(jnt_vel_handle);
    }

    registerInterface(&jnt_state_interface);
    registerInterface(&jnt_pos_interface);
    registerInterface(&jnt_vel_interface);

    j_pos_off = joints_offset;
    return true;
  }


  bool init_robot();

  bool login(std::string host,  unsigned int port);
  bool login(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  bool logout();
  bool logout(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  bool robot_startup();
  bool robot_startup(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  bool robot_shutdown();
  bool robot_shutdown(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  void print_diagnostic_info();
  bool print_diagnostic_info(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res);


  void ros_control_cb(const std_msgs::Bool::ConstPtr &msg)
  {
    if (msg->data && !ros_control)
    {
      if (robot.enable_tcp_canbus_mode())
      {
        ROS_INFO("Enabled TCP 2 CANbus Mode.");
      }
      else
      {
        ROS_ERROR("Failed to enable TCP 2 CANbus Mode.");
      }

      start_control_loop();
    }

    if (!msg->data && ros_control)
    {
      stop_control_loop();

      if (robot.disable_tcp_canbus_mode())
      {
        ROS_INFO("Disabled TCP 2 CANbus Mode.");
      }
      else
      {
        ROS_ERROR("Failed to disable TCP 2 CANbus Mode.");
      }
    }
  }


  bool start_control_loop()
  {
    pthread_t pthread;
    pthread_attr_t pthread_attr;

    errno = pthread_attr_init(&pthread_attr);
    if (errno != 0)
    {
      ROS_FATAL("pthread_attr_init");
      return false;
    }

    errno = pthread_attr_setinheritsched(&pthread_attr, PTHREAD_EXPLICIT_SCHED);
    if (errno != 0)
    {
      ROS_FATAL("pthread_attr_setschedpolicy");
      return false;
    }

    errno = pthread_attr_setschedpolicy(&pthread_attr, SCHED_FIFO);
    if (errno != 0)
    {
      ROS_FATAL("pthread_attr_setschedpolicy");
      return false;
    }

    sched_param sched_param
    {
      .sched_priority = 50
    };
    errno = pthread_attr_setschedparam(&pthread_attr, &sched_param);
    if (errno != 0)
    {
      ROS_FATAL("pthread_attr_setschedparam");
      return false;
    }

    errno = pthread_create(&pthread, &pthread_attr, &control_loop, this);
    if (errno != 0)
    {
      ROS_FATAL("pthread_create, errno: %d", errno);
      return false;
    }

    errno = pthread_attr_destroy(&pthread_attr);
    if (errno != 0)
    {
      ROS_FATAL("pthread_attr_destroy");
      return false;
    }

    return true;
  }


  void stop_control_loop()
  {
    ros_control = false;
  }


  void read(const ros::Time &time, const ros::Duration &period)
  {
    const int n_joints = j_pos.size();

    std::vector<double> joint_pos;
    joint_pos.resize(n_joints, 0.0);

    if (!robot.read(joint_pos))
    {
      reset_controllers = true;
      ROS_ERROR_THROTTLE(2.0, "Failed to read joint positions state from robot!");
    }

    // apply offset
    std::transform(joint_pos.begin(), joint_pos.end(), j_pos_off.begin(), j_pos.begin(), std::plus<double>());
  }


  void write(const ros::Time &time, const ros::Duration &period)
  {
    const int n_joints = j_pos_cmd.size();

    std::vector<double> joint_pos;
    joint_pos.resize(n_joints, 0.0);

    // apply offset
    std::transform(j_pos_cmd.begin(), j_pos_cmd.end(), j_pos_off.begin(), joint_pos.begin(), std::minus<double>());

    if (!robot.write(joint_pos))
    {
      reset_controllers = true;
      ROS_ERROR_THROTTLE(2.0, "Failed to write joint positions command to robot!");
    }
  }

};


inline void* control_loop(void* arg)
{
  aubo_hardware_interface::AuboHW* aubo_hw = (aubo_hardware_interface::AuboHW*)arg;
  aubo_hw->reset_controllers = true;

  struct timespec t_1, t;
  errno = clock_gettime(CLOCK_MONOTONIC, &t);
  if (errno != 0)
  {
    ROS_FATAL("clock_gettime");
    return NULL;
  }

  aubo_hw->ros_control = true;
  while (ros::ok() && aubo_hw->ros_control)
  {
    t_1 = t;
    aubo_hardware_interface::time::add_timespec(&t, aubo_hw->t_cycle + aubo_hw->t_offset);

    struct timespec t_left;
    errno = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, &t_left);
    if (errno != 0)
    {
      ROS_FATAL("clock_nanosleep");
      break;
    }

    struct timespec t_period;
    aubo_hardware_interface::time::diff_timespec(t, t_1, &t_period);

    const ros::Time now = ros::Time::now();
    const ros::Duration period(aubo_hardware_interface::time::to_sec(t_period));

    if (aubo_hw->robot.soft_emergency)
    {
      aubo_hw->robot_shutdown();
      aubo_hw->print_diagnostic_info();
      break;
    }

    if (aubo_hw->robot.remote_emergency)
    {
      aubo_hw->robot_shutdown();
      aubo_hw->print_diagnostic_info();
      break;
    }

    if (aubo_hw->robot.robot_collision)
    {
      aubo_hw->robot_shutdown();
      aubo_hw->print_diagnostic_info();
      break;
    }

    if (aubo_hw->robot.singularity_overspeed)
    {
      aubo_hw->robot_shutdown();
      aubo_hw->print_diagnostic_info();
      break;
    }

    if (aubo_hw->robot.robot_overcurrent)
    {
      aubo_hw->robot_shutdown();
      aubo_hw->print_diagnostic_info();
      break;
    }

    aubo_hw->read(now, period);
    aubo_hw->controller_manager.update(now, period, aubo_hw->reset_controllers);
    aubo_hw->reset_controllers = false;
    aubo_hw->write(now, period);
  }

  return NULL;
}


}  // namespace
#endif
