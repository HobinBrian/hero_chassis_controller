#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <control_toolbox/pid.h>

namespace hero_chassis_controller {

class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
 public:
  HeroChassisController() = default;
  ~HeroChassisController() override = default;

  bool init(hardware_interface::EffortJointInterface *effort_joint_interface,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

  void update(const ros::Time &time, const ros::Duration &period) override;

  hardware_interface::JointHandle front_left_joint_, front_right_joint_, back_left_joint_, back_right_joint_;
  private:
    control_toolbox::Pid pid_front_left_, pid_front_right_, pid_back_left_, pid_back_right_;
    double cmd_wheel_{0.0};
};
}// namespace hero_chassis_controller

#endif //HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
