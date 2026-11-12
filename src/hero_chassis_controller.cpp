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

  //参数
  controller_nh.param("wheel_radius", wheel_radius_, 0.07625);
  controller_nh.param("wheel_base", wheel_base_, 0.4);
  controller_nh.param("wheel_track", wheel_track_, 0.4);
  k_ = (wheel_base_ + wheel_track_) / 2.0;

  if (!pid_front_left_.init(ros::NodeHandle(controller_nh, "pid_front_left")))
    return false;
  if (!pid_front_right_.init(ros::NodeHandle(controller_nh, "pid_front_right")))
    return false;
  if (!pid_back_left_.init(ros::NodeHandle(controller_nh, "pid_back_left")))
    return false;
  if (!pid_back_right_.init(ros::NodeHandle(controller_nh, "pid_back_right")))
    return false;


  cmd_vel_sub_ = root_nh.subscribe("/cmd_vel", 1,
                                   &HeroChassisController::cmdVelCallback, this);
  
  return true;
}

void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist &msg) 
{
  cmd_vel_ = msg;
}

void HeroChassisController::update(const ros::Time &time,const ros::Duration &period) 
{
  double vx = cmd_vel_.linear.x;
  double vy = cmd_vel_.linear.y;
  double omega = cmd_vel_.angular.z;

  double cmd_front_left = (vx - vy - k_ * omega) / wheel_radius_;
  double cmd_front_right = (vx + vy + k_ * omega) / wheel_radius_;
  double cmd_back_left = (vx + vy - k_ * omega) / wheel_radius_;
  double cmd_back_right = (vx - vy + k_ * omega) / wheel_radius_;

 front_left_joint_.setCommand
 (pid_front_left_.computeCommand(cmd_front_left - front_left_joint_.getVelocity(), period));
  front_right_joint_.setCommand
 (pid_front_right_.computeCommand(cmd_front_right - front_right_joint_.getVelocity(), period));
  back_left_joint_.setCommand
 (pid_back_left_.computeCommand(cmd_back_left - back_left_joint_.getVelocity(), period));
  back_right_joint_.setCommand
 (pid_back_right_.computeCommand(cmd_back_right - back_right_joint_.getVelocity(), period));
  
}

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController,
                       controller_interface::ControllerBase)
}// namespace hero_chassis_controller
