#pragma once

#include <memory>
#include <string>

#include <sunray_perception/DetectionArray.h>
#include <sunray_perception/Tracking.h>
#include <sunray_perception/common/config_utils.h>

namespace sunray_perception
{

class Tracker
{
public:
  virtual ~Tracker() = default;

  virtual bool load(const std::string& tracking_config_path, const CameraConfig& camera,
                    std::string* error) = 0;
  virtual bool track(const DetectionArray& detections, Tracking* tracking) = 0;
};

std::unique_ptr<Tracker> createTracker(const std::string& tracker_type);

}  // namespace sunray_perception
