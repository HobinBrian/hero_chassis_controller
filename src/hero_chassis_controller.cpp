#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace hero_chassis_controller 
{

bool HeroChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
                                 ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) 
{
  front_left_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
  front_right_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
  back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
  back_right_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");

  controller_nh.param("wheel_cmd", cmd_wheel_, 0.0);//从 yaml 读期望轮速

  if (!pid_front_left_.init(ros::NodeHandle(controller_nh, "pid_front_left")))
    return false;
  if (!pid_front_right_.init(ros::NodeHandle(controller_nh, "pid_front_right")))
    return false;
  if (!pid_back_left_.init(ros::NodeHandle(controller_nh, "pid_back_left")))
    return false;
  if (!pid_back_right_.init(ros::NodeHandle(controller_nh, "pid_back_right")))
    return false;

  return true;
}

void HeroChassisController::update(const ros::Time &time, const ros::Duration &period) 
{
  //用子命名空间初始化——去 yaml 里找 controller/hero_chassis_controller/pid_front_left/ 下的 p、i、d 等参数。​返回 false 就是参数没配全，spawner 会拒绝加载控制器
  front_left_joint_.setCommand(
      pid_front_left_.computeCommand(cmd_wheel_ - front_left_joint_.getVelocity(), period));
  front_right_joint_.setCommand(
      pid_front_right_.computeCommand(cmd_wheel_ - front_right_joint_.getVelocity(), period));
  back_left_joint_.setCommand(
      pid_back_left_.computeCommand(cmd_wheel_ - back_left_joint_.getVelocity(), period));
  back_right_joint_.setCommand(
      pid_back_right_.computeCommand(cmd_wheel_ - back_right_joint_.getVelocity(), period));
  
}

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController,
                       controller_interface::ControllerBase)
}// namespace hero_chassis_controller
