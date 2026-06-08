#include <memory>
#include <string>

#include <ros/ros.h>

#include <sunray_perception/tracking/trackers.h>

namespace sunray_perception
{
namespace
{

class TrackingNode
{
public:
  TrackingNode()
    : private_nh_("~")
  {
  }

  bool init()
  {
    std::string tracker_type;
    std::string camera_config_path;
    std::string tracking_config_path;

    if (!private_nh_.getParam("tracker_type", tracker_type) || tracker_type.empty())
    {
      ROS_ERROR("Missing required parameter ~tracker_type");
      return false;
    }
    if (!private_nh_.getParam("camera_config_path", camera_config_path) || camera_config_path.empty())
    {
      ROS_ERROR("Missing required parameter ~camera_config_path");
      return false;
    }
    if (!private_nh_.getParam("tracking_config_path", tracking_config_path) || tracking_config_path.empty())
    {
      ROS_ERROR("Missing required parameter ~tracking_config_path");
      return false;
    }

    std::string error;
    if (!loadCameraConfig(camera_config_path, &camera_, &error))
    {
      ROS_ERROR_STREAM("Failed to load camera config: " << error);
      return false;
    }

    tracker_ = createTracker(tracker_type);
    if (!tracker_)
    {
      ROS_ERROR_STREAM("Unsupported tracker_type: " << tracker_type);
      return false;
    }

    if (!tracker_->load(tracking_config_path, camera_, &error))
    {
      ROS_ERROR_STREAM("Failed to load tracker config: " << error);
      return false;
    }

    tracking_pub_ = nh_.advertise<Tracking>("/sunray_perception/tracking", 1);
    detections_sub_ = nh_.subscribe("/sunray_perception/detections", 1,
                                    &TrackingNode::detectionsCallback, this);

    ROS_INFO_STREAM("tracking_node started with tracker_type=" << tracker_type);
    return true;
  }

private:
  void detectionsCallback(const DetectionArrayConstPtr& detections_msg)
  {
    Tracking tracking;
    if (!tracker_->track(*detections_msg, &tracking))
    {
      ROS_WARN_THROTTLE(2.0, "No valid tracking target in current detections");
      return;
    }

    tracking_pub_.publish(tracking);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber detections_sub_;
  ros::Publisher tracking_pub_;
  CameraConfig camera_;
  std::unique_ptr<Tracker> tracker_;
};

}  // namespace
}  // namespace sunray_perception

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tracking_node");
  sunray_perception::TrackingNode node;
  if (!node.init())
  {
    return 1;
  }
  ros::spin();
  return 0;
}
