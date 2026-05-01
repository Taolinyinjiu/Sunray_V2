#ifndef DIFFERENTIAL_CONTROLLER_H
#define DIFFERENTIAL_CONTROLLER_H

#include "ugv_controller.h"

namespace sunray_ugv_control {

class DifferentialController : public UGVControllerBase {
public:
  explicit DifferentialController(ros::NodeHandle& nh);
  ~DifferentialController() override;

  geometry_msgs::Twist move_point(const sunray_msgs::UGVControlCMD& cmd) override;
  geometry_msgs::Twist move_velocity(const sunray_msgs::UGVControlCMD& cmd) override;
  bool supports_world_velocity() const override;
  bool supports_lateral_velocity() const override;

private:
  bool enable_reverse_;
  double reverse_angle_threshold_;
  double reverse_distance_threshold_;
  double reverse_speed_ratio_;
  double final_yaw_distance_threshold_;
};

}  // namespace sunray_ugv_control

#endif  // DIFFERENTIAL_CONTROLLER_H
