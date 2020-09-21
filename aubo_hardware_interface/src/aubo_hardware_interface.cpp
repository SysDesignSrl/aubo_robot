#include "aubo_hardware_interface/aubo_hardware_interface.h"


bool aubo_hardware_interface::AuboHW::init_robot()
{
  std::vector<double> max_joint_acc;
  if (node.getParam("aubo/max_joint_acceleration", max_joint_acc))
  {
    robot.set_max_joint_acceleration(max_joint_acc);
  }

  std::vector<double> max_joint_vel;
  if (node.getParam("aubo/max_joint_velocity", max_joint_vel))
  {
    robot.set_max_joint_velocity(max_joint_vel);
  }

  robot.get_max_joint_acceleration(max_joint_acc);
  {
    std::stringstream ss;
    ss << "[ ";
    for (double val : max_joint_acc)
    {
      ss << val << " ";
    }
    ss << "] ";
    ROS_DEBUG_STREAM("max joint acceleration: " << ss.str() << "[rad/s^2]");
  }

  robot.get_max_joint_velocity(max_joint_vel);
  {
    std::stringstream ss;
    ss << "[ ";
    for (double val : max_joint_vel)
    {
      ss << val << " ";
    }
    ss << "] ";
    ROS_DEBUG_STREAM("max joint velocity: " << ss.str() << "[rad/s]");
  }

  return true;
}


bool aubo_hardware_interface::AuboHW::login(std::string host,  unsigned int port = 8899)
{
  // Login
  if (robot.login(host, port))
  {
    node.setParam("connected", true);
    ROS_INFO("Connected to %s:%d", host.c_str(), port);
  }
  else
  {
    ROS_ERROR("Failed to connect to %s:%d", host.c_str(), port);
    return false;
  }

  if (!robot.register_event_info())
  {
    ROS_ERROR("Failed to register to robot events");
    return false;
  }

  if (!init_robot())
  {
    return false;
  }

  if (robot.enable_tcp_canbus_mode())
  {
    ROS_INFO("Enabled TCP 2 CANbus Mode.");
  }
  else
  {
    ROS_ERROR("Failed to enable TCP 2 CANbus Mode.");
  }

  return true;
}


bool aubo_hardware_interface::AuboHW::login(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
  auto host = node.param<std::string>("tcp/host", "localhost");
  auto port = node.param<int>("tcp/port", 8899);

  if (login(host, port))
  {
    res.success = true;
    res.message = "Logged in successfully.";
  }
  else
  {
    res.success = false;
    res.message = "Failed to log in.";
  }

  return true;
}


bool aubo_hardware_interface::AuboHW::logout()
{
  if (robot.disable_tcp_canbus_mode())
  {
    ROS_INFO("Disabled TCP 2 CANbus Mode.");
  }
  else
  {
    ROS_ERROR("Failed to disable TCP 2 CANbus Mode.");
  }

  if (robot.logout())
  {
    ROS_INFO("Logged out.");
    node.setParam("connected", false);
    return true;
  }
  else
  {
    ROS_ERROR("Failed to log out.");
    return false;
  }
}


bool aubo_hardware_interface::AuboHW::logout(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
  if (logout())
  {
    res.success = true;
    res.message = "Logged out.";
  }
  else
  {
    res.success = false;
    res.message = "Failed to log out.";
  }

  return true;
}


bool aubo_hardware_interface::AuboHW::robot_startup()
{
  XmlRpc::XmlRpcValue tool_dynamics;
  if (!node.getParam("aubo/tool_dynamics", tool_dynamics))
  {
    std::string param_name = node.resolveName("aubo/tool_dynamics");
    ROS_WARN("Failed to retrieve '%s' parameter.", param_name.c_str());
    return false;
  }

  int collision_class;
  if (!node.getParam("aubo/collision_class", collision_class))
  {
    std::string param_name = node.resolveName("aubo/collision_class");
    ROS_WARN("Failed to retrieve '%s' parameter.", param_name.c_str());
  }

  if (robot.robot_startup(tool_dynamics, collision_class))
  {
    ROS_INFO("Robot started up with collision class: %d", collision_class);
  }
  else
  {
    ROS_ERROR("Failed to startup the Robot.");
    return false;
  }

  reset_controllers = true;
  control_loop.start();

  return true;
}


bool aubo_hardware_interface::AuboHW::robot_startup(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
  if (robot_startup())
  {
    res.success = true;
    res.message = "Robot started up successfully.";
  }
  else
  {
    res.success = false;
    res.message = "Failed to startup the Robot.";
  }

  return true;
}


bool aubo_hardware_interface::AuboHW::robot_shutdown()
{
  reset_controllers = true;
  control_loop.stop();

  if (robot.robot_shutdown())
  {
    ROS_INFO("Robot shutted down correctly.");
    return true;
  }
  else
  {
    ROS_ERROR("Failed to shutdown the Robot.");
    return false;
  }
}


bool aubo_hardware_interface::AuboHW::robot_shutdown(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
  if (robot_shutdown())
  {
    res.success = true;
    res.message = "Robot shutted down successfully.";
  }
  else
  {
    res.success = false;
    res.message = "Failed to shutdown the Robot.";
  }

  return true;
}


bool aubo_hardware_interface::AuboHW::reset(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
  reset_controllers = true;

  res.success = true;
  res.message = "Robot shutted down successfully.";

  return true;
}
