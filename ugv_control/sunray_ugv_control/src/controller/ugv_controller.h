#ifndef UGV_CONTROLLER_H
#define UGV_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sunray_msgs/UGVControlCMD.h>
#include <Eigen/Eigen>
#include "ugv_control_config.h"

namespace sunray_ugv_control {

class UGVController {
public:
  struct State {
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    double yaw;
  };

  virtual ~UGVController() {}

  virtual void set_current_state(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, double yaw) = 0;
  virtual geometry_msgs::Twist move_point(const sunray_msgs::UGVControlCMD& cmd) = 0;
  virtual geometry_msgs::Twist move_velocity(const sunray_msgs::UGVControlCMD& cmd) = 0;
  virtual bool supports_world_velocity() const = 0;
  virtual bool supports_lateral_velocity() const = 0;
};

class UGVControllerBase : public UGVController {
public:
  explicit UGVControllerBase(ros::NodeHandle& nh);
  ~UGVControllerBase() override;

  void set_current_state(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, double yaw) override;

protected:
  ros::NodeHandle nh_;
  State current_state_;
  UGVControllerParams params_;

  static double wrap_angle(double angle);
  static double clamp(double value, double lower, double upper);
  static Eigen::Vector2d world_to_body(const Eigen::Vector2d& world_vec, double yaw);

private:
  void init_params();
};

}  // namespace sunray_ugv_control

#endif  // UGV_CONTROLLER_H
