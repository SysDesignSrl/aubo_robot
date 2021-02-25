// pluginlib
#include <pluginlib/class_list_macros.hpp>
// aubo_controller_manager
#include "aubo_controller_manager/aubo_controller_manager.h"
PLUGINLIB_EXPORT_CLASS(aubo_controller_manager::AuboControllerManager, moveit_controller_manager::MoveItControllerManager);
