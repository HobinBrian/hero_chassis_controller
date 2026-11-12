#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace hero_chassis_controller 
{

bool HeroChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
                                 ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) 
{
  // 构造 tf 缓冲与监听器，global 速度模式下用它查询 odom -> base_link
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>();
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  // 速度模式：local = 底盘坐标系(base_link)，global = 世界坐标系(odom)
  std::string velocity_mode = "local";
  controller_nh.param<std::string>("velocity_mode", velocity_mode, "local");
  use_global_frame_ = (velocity_mode == "global");

  // 四个轮子的关节句柄
  front_left_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
  front_right_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
  back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
  back_right_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");

  // 底盘几何参数
  controller_nh.param("wheel_radius", wheel_radius_, 0.07625);
  controller_nh.param("wheel_base", wheel_base_, 0.4);
  controller_nh.param("wheel_track", wheel_track_, 0.4);
  k_ = (wheel_base_ + wheel_track_) / 2.0;

  ROS_INFO("[hero_chassis_controller] velocity_mode=%s r=%.5f wheel_base=%.3f wheel_track=%.3f k=%.3f",
           use_global_frame_ ? "global" : "local", wheel_radius_, wheel_base_, wheel_track_, k_);

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

  odom_pub_ = root_nh.advertise<nav_msgs::Odometry>("/odom", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>();
  
  return true;
}

//回调函数
void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist &msg) 
{
  cmd_vel_ = msg;
  last_cmd_time_ = ros::Time::now();
}

void HeroChassisController::update(const ros::Time &time,const ros::Duration &period) 
{
  //看门狗
  if ((time - last_cmd_time_).toSec() > 0.5) 
  {
    cmd_vel_ = geometry_msgs::Twist();   // 500ms 没收到新指令 → 归零
  }

  //逆运动学
  double vx = cmd_vel_.linear.x;
  double vy = cmd_vel_.linear.y;
  double omega = cmd_vel_.angular.z;

  if (use_global_frame_) 
  {
    double yaw = odom_yaw_;   // tf 还没就绪时用自己积分的朝向
    try 
    {
      auto tf_msg = tf_buffer_->lookupTransform("odom", "base_link", ros::Time(0));
      const auto &q = tf_msg.transform.rotation;
      yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                       1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    } 
    catch (const tf2::TransformException &ex) 
    {
      ROS_WARN_THROTTLE(1.0, "lookupTransform 失败，暂用里程计朝向: %s", ex.what());
    }

    // 世界系 → 车体系：绕 z 轴转 -yaw
    const double vx_world = vx;
    const double vy_world = vy;
    vx =  std::cos(yaw) * vx_world + std::sin(yaw) * vy_world;
    vy = -std::sin(yaw) * vx_world + std::cos(yaw) * vy_world;
  }

  double cmd_front_left = (vx - vy - k_ * omega) / wheel_radius_;
  double cmd_front_right = (vx + vy + k_ * omega) / wheel_radius_;
  double cmd_back_left = (vx + vy - k_ * omega) / wheel_radius_;
  double cmd_back_right = (vx - vy + k_ * omega) / wheel_radius_;

  // 四个轮子各自闭环：期望轮速 - 实测轮速 送 PID，输出力矩指令
  front_left_joint_.setCommand(
      pid_front_left_.computeCommand(cmd_front_left - front_left_joint_.getVelocity(), period));
  front_right_joint_.setCommand(
      pid_front_right_.computeCommand(cmd_front_right - front_right_joint_.getVelocity(), period));
  back_left_joint_.setCommand(
      pid_back_left_.computeCommand(cmd_back_left - back_left_joint_.getVelocity(), period));
  back_right_joint_.setCommand(
      pid_back_right_.computeCommand(cmd_back_right - back_right_joint_.getVelocity(), period));

  // 正运动学：四轮实测转速反解底盘速度
  const double vel_fl = front_left_joint_.getVelocity();
  const double vel_fr = front_right_joint_.getVelocity();
  const double vel_bl = back_left_joint_.getVelocity();
  const double vel_br = back_right_joint_.getVelocity();

  const double vx_meas = wheel_radius_ * (vel_fl + vel_fr + vel_bl + vel_br) / 4.0;
  const double vy_meas = wheel_radius_ * (-vel_fl + vel_fr + vel_bl - vel_br) / 4.0;
  const double omega_meas = wheel_radius_ * (-vel_fl + vel_fr - vel_bl + vel_br) / (4.0 * k_);

  // 位姿积分
  const double dt = period.toSec();
  odom_x_ += (vx_meas * std::cos(odom_yaw_) - vy_meas * std::sin(odom_yaw_)) * dt;
  odom_y_ += (vx_meas * std::sin(odom_yaw_) + vy_meas * std::cos(odom_yaw_)) * dt;
  odom_yaw_ += omega_meas * dt;

  if ((time - last_publish_).toSec() < 0.02) return;  // 50Hz 发布
  last_publish_ = time;

  geometry_msgs::TransformStamped transform;
  transform.header.stamp = time;
  transform.header.frame_id = "odom";
  transform.child_frame_id = "base_link";
  transform.transform.translation.x = odom_x_;
  transform.transform.translation.y = odom_y_;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, odom_yaw_);
  transform.transform.rotation.x = q.x();
  transform.transform.rotation.y = q.y();
  transform.transform.rotation.z = q.z();
  transform.transform.rotation.w = q.w();
  tf_broadcaster_->sendTransform(transform);

  nav_msgs::Odometry odom;
  odom.header.stamp = time;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  odom.pose.pose.position.x = odom_x_;
  odom.pose.pose.position.y = odom_y_;
  odom.pose.pose.orientation = transform.transform.rotation;
  odom.twist.twist.linear.x = vx_meas;
  odom.twist.twist.linear.y = vy_meas;
  odom.twist.twist.angular.z = omega_meas;
  odom_pub_.publish(odom);
}

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController,
                       controller_interface::ControllerBase)
}// namespace hero_chassis_controller
