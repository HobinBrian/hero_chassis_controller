#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <geometry_msgs/Twist.h>
#include <ros/subscriber.h> 
#include <control_toolbox/pid.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <memory>

//tf坐标变换
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

namespace hero_chassis_controller {

class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface> 
{
  public:
    HeroChassisController() = default;
    ~HeroChassisController() override = default;

    bool init(hardware_interface::EffortJointInterface *effort_joint_interface,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

    void update(const ros::Time &time, const ros::Duration &period) override;

    hardware_interface::JointHandle front_left_joint_, front_right_joint_, back_left_joint_, back_right_joint_;

  private:
    void cmdVelCallback(const geometry_msgs::Twist &msg);//订阅对象

    //数据
    ros::Subscriber cmd_vel_sub_;
    geometry_msgs::Twist cmd_vel_;
    ros::Time last_cmd_time_;//记录最后的指令时间截

    //控制器
    control_toolbox::Pid pid_front_left_, pid_front_right_, pid_back_left_, pid_back_right_;

    //参数
    double wheel_radius_{0.07625};
    double wheel_base_{0.4};
    double wheel_track_{0.4};
    double k_{0.4};

    //里程计与正运动学
    ros::Publisher odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    double odom_x_{0.0}, odom_y_{0.0}, odom_yaw_{0.0};
    ros::Time last_publish_;

    // tf 查询
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    bool use_global_frame_{false};
};
}// namespace hero_chassis_controller

#endif //HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
