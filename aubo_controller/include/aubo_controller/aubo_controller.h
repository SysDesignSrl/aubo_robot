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
  void goal_callback(const control_msgs::FollowJointTrajectoryGoal::ConstPtr &goal) {

    //
    control_msgs::JointTrajectoryGoal joint_trajectory_goal;
    joint_trajectory_goal.trajectory = goal->trajectory;
    joint_trajectory_acli.sendGoalAndWait(joint_trajectory_goal);

    if (joint_trajectory_acli.getState() != actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED) {
      follow_joint_trajectory_asrv.setAborted();
    }

    // the trajectory is started
    ros::Time start_time = ros::Time::now();

    for (int i = 0; i < goal->trajectory.points.size(); i++) {

      ros::Time now = ros::Time::now();
      ros::Time trajectory_time = start_time + goal->trajectory.points[i].time_from_start;

      // sleep
      (trajectory_time - now).sleep();


      std::vector<double> joint_pos_cmd, joint_pos_cur, joint_pos_err;
      std::vector<double> joint_vel_cmd, joint_vel_cur, joint_vel_err;

      joint_pos_cmd.resize(joint_state.name.size());
      joint_pos_cur.resize(joint_state.name.size());
      joint_pos_err.resize(joint_state.name.size());

      joint_vel_cmd.resize(joint_state.name.size());
      joint_vel_cur.resize(joint_state.name.size());
      joint_vel_err.resize(joint_state.name.size());

      auto sorted_extract = [&] (const trajectory_msgs::JointTrajectory &trajectory, int index)
      {
        for (int i=0; i < joint_state.name.size(); i++)
          for (int j=0; j < trajectory.joint_names.size(); j++)
            if (joint_state.name[i] == trajectory.joint_names[j])
            {
              joint_pos_cmd[i] = trajectory.points[index].positions[j];
              joint_vel_cmd[i] = trajectory.points[index].velocities[j];
            }
      };

      // sort joint trajectory position command
      sorted_extract(goal->trajectory, i);
      // copy the current joint position state
      std::copy(joint_state.position.cbegin(), joint_state.position.cend(), joint_pos_cur.begin());
      // compute the position error
      std::transform(joint_pos_cmd.cbegin(), joint_pos_cmd.cend(), joint_pos_cur.cbegin(), joint_pos_err.begin(), std::minus<double>());


      control_msgs::FollowJointTrajectoryFeedback feedback;
      feedback.header.stamp = start_time + goal->trajectory.points[i].time_from_start;
      feedback.joint_names = joint_state.name;
      feedback.desired.positions = joint_pos_cmd;
      feedback.actual.positions = joint_pos_cur;
      feedback.error.positions = joint_pos_err;
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
