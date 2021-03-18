#ifndef AUBO_JOINT_TRAJECTORY_CONTROLLER_HANDLE_H
#define AUBO_JOINT_TRAJECTORY_CONTROLLER_HANDLE_H
#include <string>
#include <vector>
#include <map>
#include <memory>
// roscpp
#include <ros/ros.h>
#include <ros/console.h>
// actionlib
#include <actionlib/client/simple_action_client.h>
// moveit_core
#include <moveit/controller_manager/controller_manager.h>
// moveit_msgs
#include <moveit_msgs/RobotTrajectory.h>
// trajectory_msgs
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
// control_msgs
#include <control_msgs/JointTrajectoryAction.h>
// Boost
#include <boost/bind.hpp>
#include <boost/function.hpp>


namespace aubo_controller_manager {


class JointTrajectoryControllerHandle : public moveit_controller_manager::MoveItControllerHandle {
protected:
  ros::NodeHandle node;

  std::string action_ns;

  std::vector<std::string> joint_names;

  // Action Clients
  std::shared_ptr<actionlib::SimpleActionClient<control_msgs::JointTrajectoryAction>> joint_trajectory_acli;

  bool done;
  moveit_controller_manager::ExecutionStatus last_exec;


  void controllerActiveCallback()
  {
    ROS_DEBUG("Controller %s started execution ...", name_.c_str());
  }

  void controllerDoneCallback(const actionlib::SimpleClientGoalState &state, const control_msgs::JointTrajectoryResult::ConstPtr &result)
  {
    std::string state_name = state.toString();
    std::string state_description = state.getText();
    ROS_DEBUG("Controller %s is done with state: %s, %s", name_.c_str(), state_name.c_str(), state_description.c_str());

    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      this->last_exec = moveit_controller_manager::ExecutionStatus::SUCCEEDED;
    }
    else if (state == actionlib::SimpleClientGoalState::ABORTED)
    {
      this->last_exec = moveit_controller_manager::ExecutionStatus::ABORTED;
    }
    else if (state == actionlib::SimpleClientGoalState::PREEMPTED)
    {
      this->last_exec = moveit_controller_manager::ExecutionStatus::PREEMPTED;
    }
    else
    {
      this->last_exec = moveit_controller_manager::ExecutionStatus::FAILED;
    }

    this->done = true;
  }

public:

  JointTrajectoryControllerHandle(const std::string &name, const std::string &action_ns = "") : moveit_controller_manager::MoveItControllerHandle(name), action_ns(action_ns)
  {
    auto timeout = node.param<double>("trajectory_execution/controller_connection_timeout", 10.0);

    joint_trajectory_acli = std::make_shared<actionlib::SimpleActionClient<control_msgs::JointTrajectoryAction>>(name + "/" + action_ns);

    while (ros::ok() && !joint_trajectory_acli->waitForServer(ros::Duration(timeout)))
    {
      ROS_WARN("Waiting for Action Server: %s/%s to come up ...", name_.c_str(), action_ns.c_str());
      ros::Duration(1.0).sleep();
    }

    if (!joint_trajectory_acli->isServerConnected())
    {
      ROS_ERROR("Action client not connected to: %s/%s", name_.c_str(), action_ns.c_str());
      joint_trajectory_acli.reset();
    }

    this->last_exec = moveit_controller_manager::ExecutionStatus::SUCCEEDED;
  }


  bool sendTrajectory(const moveit_msgs::RobotTrajectory &trajectory) override
  {
    ROS_DEBUG("Sending trajectory to %s ...", name_.c_str());

    if (!joint_trajectory_acli)
    {
      return false;
    }

    if (!trajectory.multi_dof_joint_trajectory.points.empty())
    {
      ROS_WARN("%s cannot execute multi-dof trajectories.", name_.c_str());
    }

    if (done)
    {
      ROS_DEBUG("Sending trajectory to %s", name_.c_str());
    }
    else
    {
      ROS_DEBUG("Sending continuation for the currently executed trajectory to %s", name_.c_str());
    }

    control_msgs::JointTrajectoryGoal goal;
    goal.trajectory = trajectory.joint_trajectory;

    joint_trajectory_acli->sendGoal(goal,
      boost::bind(&JointTrajectoryControllerHandle::controllerDoneCallback, this, _1, _2),
      boost::bind(&JointTrajectoryControllerHandle::controllerActiveCallback, this));

    this->last_exec = moveit_controller_manager::ExecutionStatus::RUNNING;
    this->done = false;

    return true;
  }


  bool waitForExecution(const ros::Duration &timeout) override
  {
    if (joint_trajectory_acli && !done)
    {
      return joint_trajectory_acli->waitForResult(timeout);
    }
    else
    {
      return true;
    }
  }

  bool cancelExecution() override
  {
    if (!joint_trajectory_acli)
    {
      return false;
    }

    if (!done)
    {
      joint_trajectory_acli->cancelGoal();
      this->last_exec = moveit_controller_manager::ExecutionStatus::PREEMPTED;
      this->done = true;
    }

    return true;
  }

  moveit_controller_manager::ExecutionStatus getLastExecutionStatus() override
  {
    return this->last_exec;
  }

};

} // namespace
#endif
