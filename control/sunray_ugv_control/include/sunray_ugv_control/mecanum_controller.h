#ifndef MECANUM_CONTROLLER_H
#define MECANUM_CONTROLLER_H

#include "sunray_ugv_control/ugv_controller.h"

namespace sunray_ugv_control {

class MecanumController : public UGVControllerBase {
public:
  explicit MecanumController(ros::NodeHandle& nh);
  ~MecanumController() override;

  geometry_msgs::Twist move_point(const sunray_msgs::UGVControlCMD& cmd) override;
  geometry_msgs::Twist move_velocity(const sunray_msgs::UGVControlCMD& cmd) override;
  bool supports_world_velocity() const override;
  bool supports_lateral_velocity() const override;
};

}  // namespace sunray_ugv_control

#endif  // MECANUM_CONTROLLER_H
