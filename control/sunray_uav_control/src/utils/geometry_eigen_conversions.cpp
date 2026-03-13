#include "utils/geometry_eigen_conversions.hpp"

namespace uav_control {

Eigen::Quaterniond geometryToEigen(const geometry_msgs::Quaternion &vec_ros) {
  return Eigen::Quaterniond(vec_ros.w, vec_ros.x, vec_ros.y, vec_ros.z);
}

geometry_msgs::Quaternion eigenToGeometry(const Eigen::Quaterniond &vec_eigen) {
  geometry_msgs::Quaternion q;
  q.w = vec_eigen.w();
  q.x = vec_eigen.x();
  q.y = vec_eigen.y();
  q.z = vec_eigen.z();
  return q;
}

Eigen::Vector3d geometryToEigen(const geometry_msgs::Vector3 &vec_ros) {
  return Eigen::Vector3d(vec_ros.x, vec_ros.y, vec_ros.z);
}

Eigen::Vector3d geometryToEigen(const geometry_msgs::Point &vec_ros) {
  return Eigen::Vector3d(vec_ros.x, vec_ros.y, vec_ros.z);
}

Eigen::Vector3d geometryToEigen(const geometry_msgs::PointStamped &vec_ros) {
  return geometryToEigen(vec_ros.point);
}

geometry_msgs::Vector3 eigenToGeometry(const Eigen::Vector3d &vec_eigen) {
  geometry_msgs::Vector3 v;
  v.x = vec_eigen.x();
  v.y = vec_eigen.y();
  v.z = vec_eigen.z();
  return v;
}

geometry_msgs::Point vectorToPoint(const geometry_msgs::Vector3 &vector) {
  geometry_msgs::Point p;
  p.x = vector.x;
  p.y = vector.y;
  p.z = vector.z;
  return p;
}

Eigen::Affine3d geometryToEigen(const geometry_msgs::Pose &pose_ros) {
  Eigen::Affine3d tf = Eigen::Affine3d::Identity();
  tf.translation() = geometryToEigen(pose_ros.position);
  tf.linear() = geometryToEigen(pose_ros.orientation).toRotationMatrix();
  return tf;
}

Eigen::Affine3d geometryToEigen(const geometry_msgs::PoseStamped &pose_ros) {
  return geometryToEigen(pose_ros.pose);
}

} // namespace uav_control
