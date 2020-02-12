#ifndef AUBO_AUBO_CONTROLLER_H
#define AUBO_AUBO_CONTROLLER_H
// STL
#include <sstream>
#include <string>
#include <vector>
#include <iterator>
#include <functional>
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


  /* */
  void start() {

    // subscribe to aubo_driver joint_states
    joint_states_sub = node.subscribe("/aubo_driver/joint_states", 100, &aubo::AuboController::joint_states_callback, this);

    // wait for aubo_driver trajectory action server
    while (ros::ok() && !joint_trajectory_acli.waitForServer(ros::Duration(5.0))) {
      ROS_INFO_THROTTLE(5.0, "Waiting for action server: '%s'...", "/aubo_driver/joint_trajectory");
    }

    follow_joint_trajectory_asrv.start();
  }


  /* */
  void joint_states_callback(const sensor_msgs::JointState::ConstPtr &msg) {
    joint_state = *msg;
    // ROS_DEBUG("JointState update at %.3fs", msg->header.stamp.toSec());
  }


  /* */
  void goal_callback(const control_msgs::FollowJointTrajectoryGoal::ConstPtr &goal)
  {
    // goal
    control_msgs::JointTrajectoryGoal joint_trajectory_goal;
    joint_trajectory_goal.trajectory = goal->trajectory;
    // fire and forget
    auto state = joint_trajectory_acli.sendGoalAndWait(joint_trajectory_goal);

    if (state == actionlib::SimpleClientGoalState::StateEnum::PENDING)
    {
      std::string message = state.getText();
      ROS_WARN("Action '%s' %s: %s", "/aubo_driver/joint_trajectory", "PENDING", message.c_str());
      follow_joint_trajectory_asrv.setAborted();
      return;
    }
    if (state == actionlib::SimpleClientGoalState::StateEnum::ACTIVE)
    {
      std::string message = state.getText();
      ROS_WARN("Action '%s' %s: %s", "/aubo_driver/joint_trajectory", "ACTIVE", message.c_str());
      follow_joint_trajectory_asrv.setAborted();
      return;
    }
    if (state == actionlib::SimpleClientGoalState::StateEnum::PREEMPTED)
    {
      std::string message = state.getText();
      ROS_WARN("Action '%s' %s: %s", "/aubo_driver/joint_trajectory", "PREEMPTED", message.c_str());
      follow_joint_trajectory_asrv.setAborted();
      return;
    }
    if (state == actionlib::SimpleClientGoalState::StateEnum::LOST)
    {
      std::string message = state.getText();
      ROS_WARN("Action '%s' %s: %s", "/aubo_driver/joint_trajectory", "LOST", message.c_str());
      return;
    }
    if (state == actionlib::SimpleClientGoalState::StateEnum::REJECTED)
    {
      std::string message = state.getText();
      ROS_WARN("Action '%s' %s: %s", "/aubo_driver/joint_trajectory", "REJECTED", message.c_str());
      return;
    }
    if (state == actionlib::SimpleClientGoalState::StateEnum::ABORTED)
    {
      std::string message = state.getText();
      ROS_WARN("Action '%s' %s: %s", "/aubo_driver/joint_trajectory", "ABORTED", message.c_str());
      follow_joint_trajectory_asrv.setAborted();
      return;
    }
    if (state == actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED)
    {
      std::string message = state.getText();
      ROS_INFO("Action '%s' %s: %s", "/aubo_driver/joint_trajectory", "SUCCEDED", message.c_str());
    }

    // the trajectory is started
    ros::Time start_time = ros::Time::now();

    for (int i = 0; i < goal->trajectory.points.size(); i++)
    {
      ros::Time now = ros::Time::now();
      ros::Time trajectory_time = start_time + goal->trajectory.points[i].time_from_start;

      // sleep
      (trajectory_time - now).sleep();

      int n_joints = joint_state.name.size();

      std::vector<double> j_pos_cmd, j_pos, j_pos_err;
      std::vector<double> j_vel_cmd, j_vel, j_vel_err;

      j_pos.resize(n_joints);
      j_pos_cmd.resize(n_joints);
      j_pos_err.resize(n_joints);

      j_vel.resize(n_joints);
      j_vel_cmd.resize(n_joints);
      j_vel_err.resize(n_joints);

      auto sorted_extract = [&] (const trajectory_msgs::JointTrajectory &trajectory, int index)
      {
        for (int i=0; i < n_joints; i++)
          for (int j=0; j < trajectory.joint_names.size(); j++)
            if (joint_state.name[i] == trajectory.joint_names[j])
            {
              j_pos_cmd[i] = trajectory.points[index].positions[j];
              j_vel_cmd[i] = trajectory.points[index].velocities[j];
            }
      };

      // sort joint trajectory position command
      sorted_extract(goal->trajectory, i);
      // copy the current joint position state
      std::copy(joint_state.position.cbegin(), joint_state.position.cend(), j_pos.begin());
      // compute the position error
      std::transform(j_pos_cmd.cbegin(), j_pos_cmd.cend(), j_pos.cbegin(), j_pos_err.begin(), std::minus<double>());


      control_msgs::FollowJointTrajectoryFeedback feedback;
      feedback.header.stamp = start_time + goal->trajectory.points[i].time_from_start;
      feedback.joint_names = joint_state.name;
      feedback.desired.positions = j_pos_cmd;
      feedback.actual.positions = j_pos;
      feedback.error.positions = j_pos_err;
      follow_joint_trajectory_asrv.publishFeedback(feedback);
    }

    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    result.error_string = "Trajectory followed succesfully.";
    follow_joint_trajectory_asrv.setSucceeded(result);
  }

};

} // namespace
#endif
