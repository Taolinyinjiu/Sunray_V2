#include "bspline_opt/uniform_bspline.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "traj_utils/Bspline.h"
#include "ego_planner_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>
#include <algorithm>
#include <cmath>

ros::Publisher pos_cmd_pub;

ego_planner_msgs::PositionCommand cmd;
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

using ego_planner::UniformBspline;

bool receive_traj_ = false;
vector<UniformBspline> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_;

// yaw control
double last_yaw_, last_yaw_dot_;
double time_forward_;
double odom_yaw_;
double traj_start_yaw_;
double traj_end_yaw_;
double filtered_target_yaw_;
double yaw_rate_max_;
double yaw_acc_max_;
double target_yaw_rate_max_;
double target_yaw_filter_time_constant_;
double yaw_dir_min_norm_;
double start_yaw_enter_thresh_;
double start_yaw_exit_thresh_;
double end_yaw_thresh_;
double start_yaw_timeout_;
bool have_odom_yaw_ = false;
bool yaw_initialized_ = false;
bool target_yaw_initialized_ = false;

enum YawExecState
{
  YAW_HOLD = 0,
  YAW_ALIGN_START = 1,
  YAW_TRACK_TRAJ = 2,
  YAW_ALIGN_END = 3
};

YawExecState yaw_exec_state_ = YAW_HOLD;
constexpr double kPi = 3.14159265358979323846;

double wrapAngle(const double yaw)
{
  double wrapped = yaw;
  while (wrapped > kPi)
    wrapped -= 2.0 * kPi;
  while (wrapped < -kPi)
    wrapped += 2.0 * kPi;
  return wrapped;
}

double angleDiff(const double target_yaw, const double current_yaw)
{
  return wrapAngle(target_yaw - current_yaw);
}

double clampAbs(const double value, const double limit)
{
  return std::max(-limit, std::min(limit, value));
}

double quaternionToYaw(const geometry_msgs::Quaternion &q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double getTrajectoryYaw(const double t_cur, const Eigen::Vector3d &pos, const double fallback_yaw)
{
  if (traj_.empty())
    return fallback_yaw;

  const double sample_t = std::max(0.0, std::min(t_cur, traj_duration_));
  const double lookahead_t = std::min(sample_t + time_forward_, traj_duration_);

  if (traj_.size() > 1)
  {
    Eigen::Vector3d lookahead_vel = traj_[1].evaluateDeBoorT(lookahead_t);
    if (lookahead_vel.head<2>().norm() > yaw_dir_min_norm_)
    {
      return std::atan2(lookahead_vel(1), lookahead_vel(0));
    }
  }

  Eigen::Vector3d dir = traj_[0].evaluateDeBoorT(lookahead_t) - pos;
  if (dir.head<2>().norm() > yaw_dir_min_norm_)
  {
    return std::atan2(dir(1), dir(0));
  }

  if (traj_.size() > 1)
  {
    Eigen::Vector3d vel = traj_[1].evaluateDeBoorT(sample_t);
    if (vel.head<2>().norm() > yaw_dir_min_norm_)
    {
      return std::atan2(vel(1), vel(0));
    }
  }

  return fallback_yaw;
}

double getTrajectoryStartYaw()
{
  const Eigen::Vector3d start_pos = traj_[0].evaluateDeBoorT(0.0);
  const double fallback_yaw = have_odom_yaw_ ? odom_yaw_ : last_yaw_;
  return getTrajectoryYaw(0.0, start_pos, fallback_yaw);
}

double getTrajectoryEndYaw()
{
  const double sample_t = std::max(0.0, traj_duration_ - std::min(time_forward_, traj_duration_));
  const Eigen::Vector3d sample_pos = traj_[0].evaluateDeBoorT(sample_t);
  return getTrajectoryYaw(sample_t, sample_pos, last_yaw_);
}

bool isTrajectoryActive(const ros::Time &time_now)
{
  if (!receive_traj_)
    return false;

  const double t_cur = (time_now - start_time_).toSec();
  return t_cur >= 0.0 && t_cur < traj_duration_;
}

double updateFilteredYawTarget(const double raw_target_yaw, const double dt)
{
  if (!target_yaw_initialized_)
  {
    filtered_target_yaw_ = wrapAngle(raw_target_yaw);
    target_yaw_initialized_ = true;
    return filtered_target_yaw_;
  }

  const double safe_dt = std::max(dt, 1e-3);
  const double alpha = safe_dt / std::max(target_yaw_filter_time_constant_ + safe_dt, safe_dt);
  double delta = alpha * angleDiff(raw_target_yaw, filtered_target_yaw_);
  delta = clampAbs(delta, target_yaw_rate_max_ * safe_dt);
  filtered_target_yaw_ = wrapAngle(filtered_target_yaw_ + delta);
  return filtered_target_yaw_;
}

std::pair<double, double> advanceYawToward(const double target_yaw, double dt)
{
  if (!yaw_initialized_)
  {
    last_yaw_ = wrapAngle(target_yaw);
    last_yaw_dot_ = 0.0;
    yaw_initialized_ = true;
    return std::make_pair(last_yaw_, last_yaw_dot_);
  }

  dt = std::max(dt, 1e-3);

  const double yaw_error = angleDiff(target_yaw, last_yaw_);
  double desired_yaw_dot = clampAbs(yaw_error / dt, yaw_rate_max_);
  const double yaw_dot_delta = desired_yaw_dot - last_yaw_dot_;
  desired_yaw_dot = last_yaw_dot_ + clampAbs(yaw_dot_delta, yaw_acc_max_ * dt);
  desired_yaw_dot = clampAbs(desired_yaw_dot, yaw_rate_max_);

  double yaw_step = desired_yaw_dot * dt;
  if (std::abs(yaw_step) > std::abs(yaw_error))
  {
    yaw_step = yaw_error;
    desired_yaw_dot = yaw_step / dt;
  }

  last_yaw_ = wrapAngle(last_yaw_ + yaw_step);
  last_yaw_dot_ = desired_yaw_dot;

  if (std::abs(angleDiff(target_yaw, last_yaw_)) < 1e-3 && std::abs(last_yaw_dot_) < 1e-2)
  {
    last_yaw_ = wrapAngle(target_yaw);
    last_yaw_dot_ = 0.0;
  }

  return std::make_pair(last_yaw_, last_yaw_dot_);
}

void odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
  odom_yaw_ = quaternionToYaw(msg->pose.pose.orientation);
  have_odom_yaw_ = true;

  if (!yaw_initialized_ || !receive_traj_)
  {
    last_yaw_ = odom_yaw_;
    last_yaw_dot_ = 0.0;
    filtered_target_yaw_ = odom_yaw_;
    yaw_initialized_ = true;
    target_yaw_initialized_ = true;
  }
}

void bsplineCallback(traj_utils::BsplineConstPtr msg)
{
  const ros::Time time_now = ros::Time::now();
  const bool was_tracking = isTrajectoryActive(time_now);

  // parse pos traj

  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());

  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i)
  {
    knots(i) = msg->knots[i];
  }

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  // parse yaw traj

  // Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  // for (int i = 0; i < msg->yaw_pts.size(); ++i) {
  //   yaw_pts(i, 0) = msg->yaw_pts[i];
  // }

  //UniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;

  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());

  traj_duration_ = traj_[0].getTimeSum();

  if (!yaw_initialized_ && have_odom_yaw_)
  {
    last_yaw_ = odom_yaw_;
    last_yaw_dot_ = 0.0;
    yaw_initialized_ = true;
  }

  traj_start_yaw_ = getTrajectoryStartYaw();
  traj_end_yaw_ = getTrajectoryEndYaw();

  if (!yaw_initialized_)
  {
    last_yaw_ = traj_start_yaw_;
    last_yaw_dot_ = 0.0;
    yaw_initialized_ = true;
  }

  const double start_yaw_error = std::abs(angleDiff(traj_start_yaw_, last_yaw_));
  yaw_exec_state_ = was_tracking ? YAW_TRACK_TRAJ : (start_yaw_error > start_yaw_enter_thresh_ ? YAW_ALIGN_START : YAW_TRACK_TRAJ);
  if (!target_yaw_initialized_)
  {
    filtered_target_yaw_ = last_yaw_;
    target_yaw_initialized_ = true;
  }
  receive_traj_ = true;
}

void cmdCallback(const ros::TimerEvent &e)
{
  /* no publishing before receive traj_ */
  if (!receive_traj_)
    return;

  ros::Time time_now = ros::Time::now();
  static ros::Time time_last(0);
  const double dt = time_last.isZero() ? 0.01 : std::max(1e-3, (time_now - time_last).toSec());
  double t_cur = (time_now - start_time_).toSec();

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), pos_f;
  std::pair<double, double> yaw_yawdot(0, 0);
  double raw_target_yaw = last_yaw_;

  if (t_cur < 0.0)
  {
    pos = traj_[0].evaluateDeBoorT(0.0);
    vel.setZero();
    acc.setZero();
    pos_f = pos;
    yaw_exec_state_ = YAW_ALIGN_START;
    raw_target_yaw = traj_start_yaw_;
  }
  else if (t_cur < traj_duration_)
  {
    pos = traj_[0].evaluateDeBoorT(t_cur);
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);

    double tf = std::min(traj_duration_, t_cur + 2.0);
    pos_f = traj_[0].evaluateDeBoorT(tf);

    const double tangent_yaw = getTrajectoryYaw(t_cur, pos, last_yaw_);

    if (yaw_exec_state_ == YAW_ALIGN_START)
    {
      const double start_yaw_error = std::abs(angleDiff(traj_start_yaw_, last_yaw_));
      if (start_yaw_error <= start_yaw_exit_thresh_ || t_cur >= start_yaw_timeout_)
      {
        yaw_exec_state_ = YAW_TRACK_TRAJ;
      }
    }

    if (yaw_exec_state_ == YAW_ALIGN_START)
    {
      raw_target_yaw = traj_start_yaw_;
    }
    else
    {
      yaw_exec_state_ = YAW_TRACK_TRAJ;
      raw_target_yaw = tangent_yaw;
    }
  }
  else if (t_cur >= traj_duration_)
  {
    /* hover when finish traj_ */
    pos = traj_[0].evaluateDeBoorT(traj_duration_);
    vel.setZero();
    acc.setZero();
    pos_f = pos;

    raw_target_yaw = traj_end_yaw_;
    const double end_yaw_error = std::abs(angleDiff(raw_target_yaw, last_yaw_));
    yaw_exec_state_ = end_yaw_error > end_yaw_thresh_ ? YAW_ALIGN_END : YAW_HOLD;
  }
  else
  {
    cout << "[Traj server]: invalid time." << endl;
  }

  const double target_yaw = updateFilteredYawTarget(raw_target_yaw, dt);
  yaw_yawdot = advanceYawToward(target_yaw, dt);
  time_last = time_now;

  cmd.header.stamp = time_now;
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = ego_planner_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);

  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);

  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);

  cmd.yaw = yaw_yawdot.first;
  cmd.yaw_dot = yaw_yawdot.second;

  pos_cmd_pub.publish(cmd);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  // ros::NodeHandle node;
  ros::NodeHandle nh("~");

  ros::Subscriber bspline_sub = nh.subscribe("planning/bspline", 10, bsplineCallback);
  ros::Subscriber odom_sub = nh.subscribe("odom", 10, odomCallback);
  pos_cmd_pub = nh.advertise<ego_planner_msgs::PositionCommand>("/position_cmd", 50);

  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);

  /* control parameter */
  cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];

  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];

  nh.param("traj_server/time_forward", time_forward_, -1.0);
  nh.param("traj_server/yaw_rate_max", yaw_rate_max_, kPi / 2.0);
  nh.param("traj_server/yaw_acc_max", yaw_acc_max_, kPi);
  nh.param("traj_server/target_yaw_rate_max", target_yaw_rate_max_, kPi / 1.5);
  nh.param("traj_server/target_yaw_filter_time_constant", target_yaw_filter_time_constant_, 0.15);
  nh.param("traj_server/yaw_dir_min_norm", yaw_dir_min_norm_, 0.1);
  nh.param("traj_server/start_yaw_enter_thresh", start_yaw_enter_thresh_, 30.0 * kPi / 180.0);
  nh.param("traj_server/start_yaw_exit_thresh", start_yaw_exit_thresh_, 8.0 * kPi / 180.0);
  nh.param("traj_server/end_yaw_thresh", end_yaw_thresh_, 5.0 * kPi / 180.0);
  nh.param("traj_server/start_yaw_timeout", start_yaw_timeout_, 0.6);
  last_yaw_ = 0.0;
  last_yaw_dot_ = 0.0;
  odom_yaw_ = 0.0;
  traj_start_yaw_ = 0.0;
  traj_end_yaw_ = 0.0;
  filtered_target_yaw_ = 0.0;

  ros::Duration(1.0).sleep();

  ROS_WARN("[Traj server]: ready.");

  ros::spin();

  return 0;
}
