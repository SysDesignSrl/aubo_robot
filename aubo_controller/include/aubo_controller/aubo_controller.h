#ifndef AUBO_AUBO_CONTROLLER_H
#define AUBO_AUBO_CONTROLLER_H
// STL
#include <sstream>
#include <cmath>
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
#include <control_msgs/JointTolerance.h>
#include <control_msgs/JointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>


namespace aubo {

class AuboController {
private:
  ros::NodeHandle node;

  std::vector<std::string> joint_names;

  std::vector<double> j_pos_cmd, j_pos, j_pos_err;
  std::vector<double> j_vel_cmd, j_vel, j_vel_err;


  ros::Subscriber joint_states_sub;

  actionlib::SimpleActionClient<control_msgs::JointTrajectoryAction> joint_trajectory_acli;
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> follow_joint_trajectory_asrv;


  sensor_msgs::JointState joint_state;

public:

  AuboController(const ros::NodeHandle &nh = ros::NodeHandle()) :
    node(nh),
    joint_trajectory_acli(node, "/aubo_driver/joint_trajectory"),
    follow_joint_trajectory_asrv(node, "follow_joint_trajectory", boost::bind(&aubo::AuboController::goal_callback, this, _1), false)
  {
    init();
    start();
  }


  bool init()
  {
    if (!node.getParam("joint_names", joint_names))
    {
      ROS_FATAL("Failed to get parameter: '%s'", "joint_names");
      return false;
    }

    int n_joints = joint_names.size();

    j_pos_cmd.resize(n_joints, 0.0);
    j_vel_cmd.resize(n_joints, 0.0);

    j_pos.resize(n_joints, 0.0);
    j_vel.resize(n_joints, 0.0);

    j_pos_err.resize(n_joints, 0.0);
    j_vel_err.resize(n_joints, 0.0);
  }


  void start()
  {
    // subscribe to aubo_driver joint_states
    joint_states_sub = node.subscribe("/aubo_driver/joint_states", 100, &aubo::AuboController::joint_states_callback, this);

    // wait for aubo_driver trajectory action server
    while (ros::ok() && !joint_trajectory_acli.waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO_THROTTLE(5.0, "Waiting Action Server: %s ...", "/aubo_driver/joint_trajectory");
    }

    follow_joint_trajectory_asrv.start();
  }


  /* */
  void joint_states_callback(const sensor_msgs::JointState::ConstPtr &msg)
  {
    // copy the current joint position state
    std::copy(msg->position.cbegin(), msg->position.cend(), j_pos.begin());
    std::copy(msg->velocity.cbegin(), msg->velocity.cend(), j_vel.begin());

    // compute the position error
    std::transform(j_pos_cmd.cbegin(), j_pos_cmd.cend(), j_pos.cbegin(), j_pos_err.begin(), std::minus<double>());
    std::transform(j_vel_cmd.cbegin(), j_vel_cmd.cend(), j_vel.cbegin(), j_vel_err.begin(), std::minus<double>());

    // ROS_DEBUG("JointState update at %.3fs", msg->header.stamp.toSec());
  }


  /* */
  void goal_callback(const control_msgs::FollowJointTrajectoryGoal::ConstPtr &goal)
  {
    // goal
    control_msgs::JointTrajectoryGoal joint_trajectory_goal;
    joint_trajectory_goal.trajectory = goal->trajectory;
    joint_trajectory_acli.sendGoal(joint_trajectory_goal);


    // execution trajectory is started
    ros::Time start_time = ros::Time::now();

    for (const trajectory_msgs::JointTrajectoryPoint &trajectory_pt : goal->trajectory.points)
    {
      //
      ros::Time now = ros::Time::now();
      ros::Time trajectory_time = start_time + trajectory_pt.time_from_start;
      ROS_DEBUG("trajectory time: %f", trajectory_time.toSec());

      // wait
      (trajectory_time - now).sleep();

      for (int i = 0; i < joint_names.size(); i++)
      {
        for (int j = 0; j < goal->trajectory.joint_names.size(); j++)
        {
          if (joint_names[i] == goal->trajectory.joint_names[j])
          {
            j_pos_cmd[i] = trajectory_pt.positions[j];
            // j_vel_cmd[i] = trajectory_pt.velocities[j];
            // j_acc_cmd[i] = trajectory_pt.accelerations[j];
          }
        }
      }

      // publish feedback
      control_msgs::FollowJointTrajectoryFeedback feedback;
      feedback.header.stamp = trajectory_time;
      feedback.joint_names = joint_names;
      feedback.desired.positions = j_pos_cmd;
      feedback.actual.positions = j_pos;
      feedback.error.positions = j_pos_err;
      follow_joint_trajectory_asrv.publishFeedback(feedback);

      // check path tolerance
      if (!check_path_tolerance(goal->path_tolerance))
      {
        control_msgs::FollowJointTrajectoryResult result;
        result.error_code = control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
        result.error_string = "";
        follow_joint_trajectory_asrv.setAborted(result);
        return;
      }
    }


    // wait
    ros::Duration time_tol = goal->goal_time_tolerance;
    ROS_DEBUG("Goal time tolerance: %.1fs", time_tol.toSec());
    time_tol.sleep();

    // check goal tolerances
    if (!check_goal_tolerance(goal->goal_tolerance))
    {
      control_msgs::FollowJointTrajectoryResult result;
      result.error_code = control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED;
      result.error_string = "";
      follow_joint_trajectory_asrv.setAborted(result);
      return;
    }


    // success
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    result.error_string = "Trajectory followed succesfully.";
    follow_joint_trajectory_asrv.setSucceeded(result);
  }


  bool check_path_tolerance(const std::vector<control_msgs::JointTolerance> &joint_tol)
  {
    for (int i = 0; i < joint_names.size(); i++)
    {
      const auto j_tol = std::find_if(joint_tol.cbegin(), joint_tol.cend(), [&](const control_msgs::JointTolerance &j_tol) { return j_tol.name == joint_names[i]; });

      if (j_tol == joint_tol.cend())
      {
        continue;
      }

      double j_pos_tol = j_tol->position;
      double j_vel_tol = j_tol->velocity;
      ROS_DEBUG("'%s' tol: %f %f", joint_names[i].c_str(), j_pos_tol, j_vel_tol);

      if (std::abs(j_pos_err[i]) > j_pos_tol)
      {
        ROS_WARN("Path position tolerance violated: '%s' err: %f > tol: %f", joint_names[i].c_str(), j_pos_err[i], j_pos_tol);
        return false;
      }
      if (std::abs(j_vel_err[i]) > j_vel_tol)
      {
        ROS_WARN("Path velocity tolerance violated: '%s' err: %f > tol: %f", joint_names[i].c_str(), j_vel_err[i], j_vel_tol);
        return false;
      }
    }

    return true;
  }


  bool check_goal_tolerance(const std::vector<control_msgs::JointTolerance> &joint_tol)
  {
    for (int i = 0; i < joint_names.size(); i++)
    {
      const auto j_tol = std::find_if(joint_tol.cbegin(), joint_tol.cend(), [&](const control_msgs::JointTolerance &j_tol) { return j_tol.name == joint_names[i]; });

      if (j_tol == joint_tol.cend())
      {
        continue;
      }

      double j_pos_tol = j_tol->position;
      double j_vel_tol = j_tol->velocity;
      ROS_DEBUG("'%s' tol: %f %f", joint_names[i].c_str(), j_pos_tol, j_vel_tol);

      if (std::abs(j_pos_err[i]) > j_pos_tol)
      {
        ROS_WARN("Goal position tolerance violated: '%s' err: %f > tol: %f", joint_names[i].c_str(), j_pos_err[i], j_pos_tol);
        return false;
      }
      if (std::abs(j_vel_err[i]) > j_vel_tol)
      {
        ROS_WARN("Goal velocity tolerance violated: '%s' err: %f > tol: %f", joint_names[i].c_str(), j_vel_err[i], j_vel_tol);
        return false;
      }
    }

    return true;
  }

};

} // namespace
#endif
