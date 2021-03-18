#ifndef AUBO_CONTROLLER_MANAGER_H
#define AUBO_CONTROLLER_MANAGER_H
#include <string>
#include <vector>
#include <map>
#include <memory>
// roscpp
#include <ros/ros.h>
#include <ros/console.h>
// xmlrcpp
#include <XmlRpcValue.h>
#include <XmlRpcException.h>
// moveit_core
#include <moveit/controller_manager/controller_manager.h>
#include <moveit/utils/xmlrpc_casts.h>
// aubo_controller_manager
#include "aubo_controller_manager/joint_trajectory_controller_handle.h"
#include "aubo_controller_manager/gripper_command_controller_handle.h"


namespace aubo_controller_manager {


class AuboControllerManager : public moveit_controller_manager::MoveItControllerManager {
protected:
  ros::NodeHandle node;
  std::map<std::string,moveit_controller_manager::MoveItControllerHandlePtr> controllers;
  std::map<std::string,std::vector<std::string>> controllers_joints;
  std::map<std::string,moveit_controller_manager::MoveItControllerManager::ControllerState> controllers_state;

public:

  AuboControllerManager() : node("~")
  {
    //
    if (!node.hasParam("controller_list"))
    {
      std::string param_name = node.resolveName("controller_list");
      ROS_ERROR("Failed to retrieve %s parameter.", param_name.c_str());
      return;
    }

    //
    XmlRpc::XmlRpcValue controller_list;
    node.getParam("controller_list", controller_list);
    if (!moveit::core::isArray(controller_list))
    {
      std::string param_name = node.resolveName("controller_list");
      ROS_ERROR("Parameter %s is not an array", param_name.c_str());
      return;
    }

    //
    for (int i = 0; i < controller_list.size(); i++)
    {
      if (!moveit::core::isStruct(controller_list[i], { "name", "action_ns", "type", "joints" }))
      {
        ROS_ERROR("Controller[%d] have not name, action_ns, type and joints defined", i);
        continue;
      }

      try
      {
        const std::string name = controller_list[i]["name"];
        const std::string action_ns = controller_list[i]["action_ns"];
        const std::string type = controller_list[i]["type"];

        if (!moveit::core::isArray(controller_list[i]["joints"]))
        {
          ROS_ERROR("Controller '%s': joints is not an array", name.c_str());
          continue;
        }

        // joints
        std::vector<std::string> joints;
        for (int j = 0; j < controller_list[i]["joints"].size(); j++)
        {
          std::string joint = controller_list[i]["joints"][j];
          joints.push_back(joint);
        }
        controllers_joints[name] = joints;

        // state
        moveit_controller_manager::MoveItControllerManager::ControllerState state;
        state.default_ = controller_list[i].hasMember("default") ? (bool)controller_list[i]["default"] : false;
        state.active_ = true;
        controllers_state[name] = state;

        // handle
        if (type == "JointTrajectory")
        {
          controllers[name] = std::make_shared<aubo_controller_manager::JointTrajectoryControllerHandle>(name, action_ns);
        }
        else if (type == "GripperCommand")
        {
          controllers[name] = std::make_shared<aubo_controller_manager::GripperCommandControllerHandle>(name, action_ns);
        }
        else
        {
          ROS_ERROR("Controller '%s': type '%s' not supported", name.c_str(), type.c_str());
          continue;
        }

        ROS_INFO("Controller Manager: added controller: %s", name.c_str());
      }
      catch (const XmlRpc::XmlRpcException &ex)
      {
        const auto code = ex.getCode();
        const auto message = ex.getMessage();
        ROS_ERROR("Error while parsing controller information: %d, %s", code, message.c_str());
      }
    }
  }

  /*
   * Return a given named controller. */
  std::shared_ptr<moveit_controller_manager::MoveItControllerHandle> getControllerHandle(const std::string &name) override
  {
    std::map<std::string,moveit_controller_manager::MoveItControllerHandlePtr>::const_iterator it = controllers.find(name);
    if (it != controllers.end())
    {
      return it->second;
    }
    else
    {
      ROS_ERROR("No such controller: %s", name.c_str());
      return moveit_controller_manager::MoveItControllerHandlePtr();
    }
  }

  /*
   * Get the list of controller names. */
  void getControllersList(std::vector<std::string> &names) override
  {
    for (auto it = controllers.begin(); it != controllers.end(); it++)
    {
      names.push_back(it->first);
    }
  }

  /*
   * This plugin assumes that all controllers are already active --
   * and if they are not, well, it has no way to deal with it anyways! */
  void getActiveControllers(std::vector<std::string> &names) override
  {
    getControllersList(names);
  }

  /*
   * Controller must be loaded to be active, see comment above about active controllers... */
  void getLoadedControllers(std::vector<std::string> &names)
  {
    getControllersList(names);
  }

  /*
   * Get the list of joints that a controller can control. */
  void getControllerJoints(const std::string &name, std::vector<std::string> &joints) override
  {
    std::map<std::string,std::vector<std::string>>::const_iterator it = controllers_joints.find(name);
    if (it != controllers_joints.end())
    {
      joints = it->second;
    }
    else
    {
      ROS_WARN("The joints for controller '%s' are not known.", name.c_str());
      joints.clear();
    }
  }

  /*
   * Controllers are all active and default. */
  moveit_controller_manager::MoveItControllerManager::ControllerState getControllerState(const std::string &name) override
  {
    return controllers_state[name];
  }

  /*
   * Cannot switch our controllers */
  bool switchControllers(const std::vector<std::string> &activate, const std::vector<std::string> &deactivate) override
  {
    return false;
  }

};

} // namespace
#endif
