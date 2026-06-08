#pragma once

#include <memory>
#include <string>

#include <opencv2/core.hpp>
#include <std_msgs/Header.h>

#include <sunray_perception/DetectionArray.h>

namespace sunray_perception
{

class Detector
{
public:
  virtual ~Detector() = default;

  virtual bool load(const std::string& config_path, const std::string& model_path, std::string* error) = 0;
  virtual bool detect(const cv::Mat& image, const std_msgs::Header& header,
                      DetectionArray* detections, cv::Mat* debug_image) = 0;
};

std::unique_ptr<Detector> createDetector(const std::string& detector_type);

}  // namespace sunray_perception
