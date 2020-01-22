#ifndef AUBO_AUBO_CONTROLLER_H
#define AUBO_AUBO_CONTROLLER_H
// STL
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

  ros::Publisher joint_trajectory_pub;

  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> follow_joint_trajectory_act;


  sensor_msgs::JointState joint_state;

public:

  AuboController(const ros::NodeHandle &node = ros::NodeHandle()) :
    node(node),
    follow_joint_trajectory_act(node, "follow_joint_trajectory", boost::bind(&aubo::AuboController::goal_callback, this, _1), false) {

    start();
  }


  void start() {
    joint_states_sub = node.subscribe("/aubo_driver/joint_states", 10, &aubo::AuboController::joint_states_callback, this);
    joint_trajectory_pub = node.advertise<trajectory_msgs::JointTrajectory>("/aubo_driver/joint_trajectory", 10);
    follow_joint_trajectory_act.start();
  }


  void
  joint_states_callback(const sensor_msgs::JointState::ConstPtr &msg) {
    joint_state = *msg;
  }


  void
  goal_callback(const control_msgs::FollowJointTrajectoryGoal::ConstPtr &goal) {

    //
    joint_trajectory_pub.publish(goal->trajectory);

    //
    auto is_completed = [&goal] (sensor_msgs::JointState joint_state) {

      // get last trajectory point
      auto it = goal->trajectory.points.crend();
      trajectory_msgs::JointTrajectoryPoint last_trajectory_point = *(it++);

      std::vector<double> jpos_goal = last_trajectory_point.positions;
      std::vector<double> jvel_goal = last_trajectory_point.velocities;

      std::vector<double> jpos_curr = joint_state.position;
      std::vector<double> jvel_curr = joint_state.velocity;

      for (int i=0; i < joint_state.name.size(); i++) {

        if (joint_state.name[i] != goal->goal_tolerance[i].name) {
          ROS_ERROR("joint name mismatch!");
        }

        double jpos_err = jpos_goal[i] - jpos_curr[i];
        double jvel_err = jvel_goal[i] - jvel_curr[i];

        if (abs(jpos_err) > goal->goal_tolerance[i].position)
          return false;
        if (abs(jvel_err) > goal->goal_tolerance[i].velocity)
          return false;
      }

      return true;
    };

    // publish feedback
    control_msgs::FollowJointTrajectoryFeedback feedback;
    do {

    } while (!is_completed(joint_state) && follow_joint_trajectory_act.isActive());

    //
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    follow_joint_trajectory_act.setSucceeded(result);
  }

};

} // namespace
#endif
