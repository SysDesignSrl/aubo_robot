#ifndef AUBO_AUBO_CONTROLLER_H
#define AUBO_AUBO_CONTROLLER_H
// STL
#include <sstream>
#include <string>
#include <vector>
#include <iterator>
#include <algorithm>
// roscpp
#include <ros/ros.h>
#include <ros/console.h>
// actionlib
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
// sensor_msgs
#include <sensor_msgs/JointState.h>
// trajectory_msgs
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
// control_msgs
#include <control_msgs/JointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>


namespace aubo {

class AuboController {
private:
  ros::NodeHandle node;

  ros::Subscriber joint_states_sub;

  actionlib::SimpleActionClient<control_msgs::JointTrajectoryAction> joint_trajectory_acli;
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> follow_joint_trajectory_asrv;


  sensor_msgs::JointState joint_state;

public:

  AuboController(const ros::NodeHandle &nh = ros::NodeHandle()) :
    node(nh),
    joint_trajectory_acli(node, "/aubo_driver/joint_trajectory"),
    follow_joint_trajectory_asrv(node, "follow_joint_trajectory", boost::bind(&aubo::AuboController::goal_callback, this, _1), false) {

    start();
  }


  void start() {

    joint_states_sub = node.subscribe("/aubo_driver/joint_states", 10, &aubo::AuboController::joint_states_callback, this);

    // wait aubo_driver action server
    while (ros::ok() && !joint_trajectory_acli.waitForServer(ros::Duration(5.0))) {
      ROS_INFO_THROTTLE(5.0, "Waiting for action server: '%s'...", "/aubo_driver/joint_trajectory");
    }

    follow_joint_trajectory_asrv.start();
  }


  void
  joint_states_callback(const sensor_msgs::JointState::ConstPtr &msg) {
    joint_state = *msg;
  }


  void
  goal_callback(const control_msgs::FollowJointTrajectoryGoal::ConstPtr &goal) {

    //
    control_msgs::JointTrajectoryGoal joint_trajectory_goal;
    joint_trajectory_goal.trajectory = goal->trajectory;
    joint_trajectory_acli.sendGoal(joint_trajectory_goal);

    auto start_time = ros::Time::now();
    for (const auto &joint_trajectory_pt : goal->trajectory.points) {

      //
      joint_trajectory_pt.time_from_start.sleep();
      ros::Time trajectory_time = start_time + joint_trajectory_pt.time_from_start;

      ROS_DEBUG("joint_state stamp: %.3fs", joint_state.header.stamp.toSec());
      ROS_DEBUG("trajectory_time: %.3fs", trajectory_time.toSec());


      std::stringstream desired_stream("[");
      for (int i=0; i < joint_trajectory_pt.positions.size(); i++) {
        if (i < joint_trajectory_pt.positions.size() -1)
          desired_stream << joint_trajectory_pt.positions[i] << ", ";
        else
          desired_stream << joint_trajectory_pt.positions[i] << " ]";
      }

      std::stringstream actual_stream("[");
      for (int i=0; i < joint_state.position.size(); i++) {
        if (i < joint_state.name.size() -1)
          actual_stream << joint_state.position[i] << ", ";
        else
          actual_stream << joint_state.position[i] << " ]";
      }

      ROS_DEBUG_STREAM("desired: " << desired_stream.str());
      ROS_DEBUG_STREAM("actual: " << actual_stream.str());
    }

    // //
    // auto is_completed = [&goal] (sensor_msgs::JointState joint_state) {
    //
    //   // get last trajectory point
    //   auto it = goal->trajectory.points.crend();
    //   trajectory_msgs::JointTrajectoryPoint last_trajectory_point = *(it++);
    //
    //   std::vector<double> jpos_goal = last_trajectory_point.positions;
    //   std::vector<double> jvel_goal = last_trajectory_point.velocities;
    //
    //   std::vector<double> jpos_curr = joint_state.position;
    //   std::vector<double> jvel_curr = joint_state.velocity;
    //
    //   for (int i=0; i < joint_state.name.size(); i++) {
    //
    //     if (joint_state.name[i] != goal->goal_tolerance[i].name) {
    //       ROS_ERROR("joint name mismatch!");
    //     }
    //
    //     double jpos_err = jpos_goal[i] - jpos_curr[i];
    //     double jvel_err = jvel_goal[i] - jvel_curr[i];
    //
    //     if (abs(jpos_err) > goal->goal_tolerance[i].position)
    //       return false;
    //     if (abs(jvel_err) > goal->goal_tolerance[i].velocity)
    //       return false;
    //   }
    //
    //   return true;
    // };
    //
    // // publish feedback
    // control_msgs::FollowJointTrajectoryFeedback feedback;
    // do {
    //
    // } while (!is_completed(joint_state) && follow_joint_trajectory_asrv.isActive());
    //
    // //
    // control_msgs::FollowJointTrajectoryResult result;
    // result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    // follow_joint_trajectory_asrv.setSucceeded(result);
  }

};

} // namespace
#endif
