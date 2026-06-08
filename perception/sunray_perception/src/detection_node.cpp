#include <memory>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <sunray_perception/detection/detectors.h>

namespace sunray_perception
{
namespace
{

class DetectionNode
{
public:
  DetectionNode()
    : private_nh_("~"), image_transport_(nh_)
  {
  }

  bool init()
  {
    std::string detector_type;
    std::string image_topic;
    std::string config_path;
    std::string model_path;

    if (!private_nh_.getParam("detector_type", detector_type) || detector_type.empty())
    {
      ROS_ERROR("Missing required parameter ~detector_type");
      return false;
    }
    if (!private_nh_.getParam("image_topic", image_topic) || image_topic.empty())
    {
      ROS_ERROR("Missing required parameter ~image_topic");
      return false;
    }
    if (!private_nh_.getParam("config_path", config_path) || config_path.empty())
    {
      ROS_ERROR("Missing required parameter ~config_path");
      return false;
    }
    private_nh_.param<std::string>("model_path", model_path, "");

    detector_ = createDetector(detector_type);
    if (!detector_)
    {
      ROS_ERROR_STREAM("Unsupported detector_type: " << detector_type);
      return false;
    }

    std::string error;
    if (!detector_->load(config_path, model_path, &error))
    {
      ROS_ERROR_STREAM("Failed to load detector config: " << error);
      return false;
    }

    detections_pub_ = nh_.advertise<DetectionArray>("/sunray_perception/detections", 1);
    debug_pub_ = image_transport_.advertise("/sunray_perception/detection_image", 1);
    image_sub_ = image_transport_.subscribe(image_topic, 1, &DetectionNode::imageCallback, this);

    ROS_INFO_STREAM("detection_node started with detector_type=" << detector_type
                    << ", image_topic=" << image_topic);
    return true;
  }

private:
  void imageCallback(const sensor_msgs::ImageConstPtr& image_msg)
  {
    cv_bridge::CvImageConstPtr cv_image;
    try
    {
      cv_image = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (const cv_bridge::Exception& e)
    {
      ROS_ERROR_STREAM_THROTTLE(2.0, "cv_bridge conversion failed: " << e.what());
      return;
    }

    DetectionArray detections;
    cv::Mat debug_image;
    if (!detector_->detect(cv_image->image, image_msg->header, &detections, &debug_image))
    {
      ROS_ERROR_THROTTLE(2.0, "Detector failed");
      return;
    }

    detections_pub_.publish(detections);

    if (debug_pub_.getNumSubscribers() > 0)
    {
      cv_bridge::CvImage debug_msg;
      debug_msg.header = image_msg->header;
      debug_msg.encoding = sensor_msgs::image_encodings::BGR8;
      debug_msg.image = debug_image;
      debug_pub_.publish(debug_msg.toImageMsg());
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  image_transport::ImageTransport image_transport_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher debug_pub_;
  ros::Publisher detections_pub_;
  std::unique_ptr<Detector> detector_;
};

}  // namespace
}  // namespace sunray_perception

int main(int argc, char** argv)
{
  ros::init(argc, argv, "detection_node");
  sunray_perception::DetectionNode node;
  if (!node.init())
  {
    return 1;
  }
  ros::spin();
  return 0;
}
