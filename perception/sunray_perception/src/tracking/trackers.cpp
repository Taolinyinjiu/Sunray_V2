#include <sunray_perception/tracking/trackers.h>

#include <algorithm>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <ros/package.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <sunray_perception/common/config_utils.h>

namespace sunray_perception
{
namespace
{

bool detectionToCorners(const Detection& detection, std::vector<cv::Point2f>* corners)
{
  if (!corners || detection.corners_x.size() != detection.corners_y.size() ||
      detection.corners_x.size() != 4)
  {
    return false;
  }

  corners->clear();
  corners->reserve(4);
  for (std::size_t i = 0; i < 4; ++i)
  {
    corners->emplace_back(static_cast<float>(detection.corners_x[i]),
                          static_cast<float>(detection.corners_y[i]));
  }
  return true;
}

void fillBaseTracking(const DetectionArray& detections, const CameraConfig& camera, Tracking* tracking)
{
  tracking->header = detections.header;
  tracking->height = camera.height;
  tracking->width = camera.width;
  tracking->fov_x = camera.fov_x;
  tracking->fov_y = camera.fov_y;
}

void fillPose(const cv::Mat& rotation_matrix, const cv::Mat& translation, Tracking* tracking)
{
  const cv::Vec3d rpy = rotationMatrixToRpy(rotation_matrix);
  tracking->mean_px = translation.at<double>(0);
  tracking->mean_py = translation.at<double>(1);
  tracking->mean_pz = translation.at<double>(2);
  tracking->mean_roll = rpy[0];
  tracking->mean_pitch = rpy[1];
  tracking->mean_yaw = rpy[2];
}

bool isArucoDetection(const Detection& detection)
{
  return detection.detector_type == "aruco";
}

std::string trim(const std::string& value)
{
  const std::string whitespace = " \t\r\n";
  const std::size_t begin = value.find_first_not_of(whitespace);
  if (begin == std::string::npos)
  {
    return "";
  }
  const std::size_t end = value.find_last_not_of(whitespace);
  return value.substr(begin, end - begin + 1);
}

std::vector<std::string> splitCsvLine(const std::string& line)
{
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ','))
  {
    fields.push_back(trim(field));
  }
  return fields;
}

bool parseInt(const std::string& text, int* value)
{
  if (!value)
  {
    return false;
  }

  try
  {
    std::size_t parsed = 0;
    const int result = std::stoi(text, &parsed);
    if (parsed != text.size())
    {
      return false;
    }
    *value = result;
    return true;
  }
  catch (const std::exception&)
  {
    return false;
  }
}

bool parseDouble(const std::string& text, double* value)
{
  if (!value)
  {
    return false;
  }

  try
  {
    std::size_t parsed = 0;
    const double result = std::stod(text, &parsed);
    if (parsed != text.size())
    {
      return false;
    }
    *value = result;
    return true;
  }
  catch (const std::exception&)
  {
    return false;
  }
}

std::string resolvePackagePath(const std::string& path)
{
  if (!path.empty() && path[0] == '/')
  {
    return path;
  }

  const std::string package_path = ros::package::getPath("sunray_perception");
  if (package_path.empty())
  {
    return path;
  }
  return package_path + "/" + path;
}

class SingleArucoTracker final : public Tracker
{
public:
  bool load(const std::string& tracking_config_path, const CameraConfig& camera,
            std::string* error) override
  {
    camera_ = camera;

    YAML::Node root;
    if (!loadYamlFile(tracking_config_path, &root, error))
    {
      return false;
    }

    if (!requireInt(root, "dictionaryId", &dictionary_id_, error) ||
        !requireInt(root, "markerId", &marker_id_, error) ||
        !requireDouble(root, "markerLength", &marker_length_, error))
    {
      return false;
    }

    if (marker_length_ <= 0.0)
    {
      if (error)
      {
        *error = "markerLength must be positive";
      }
      return false;
    }

    ROS_INFO_STREAM("Loaded SingleArucoTracker. markerId=" << marker_id_
                    << ", markerLength=" << marker_length_);
    return true;
  }

  bool track(const DetectionArray& detections, Tracking* tracking) override
  {
    if (!tracking)
    {
      return false;
    }

    const Detection* target = nullptr;
    for (const Detection& detection : detections.detections)
    {
      if (isArucoDetection(detection) && detection.class_id == marker_id_)
      {
        target = &detection;
        break;
      }
    }

    if (!target)
    {
      return false;
    }

    std::vector<cv::Point2f> image_points;
    if (!detectionToCorners(*target, &image_points))
    {
      ROS_WARN_THROTTLE(2.0, "single_aruco detection has invalid corners");
      return false;
    }

    const double half = marker_length_ / 2.0;
    std::vector<cv::Point3f> object_points = {
      cv::Point3f(static_cast<float>(-half), static_cast<float>(-half), 0.0f),
      cv::Point3f(static_cast<float>(half), static_cast<float>(-half), 0.0f),
      cv::Point3f(static_cast<float>(half), static_cast<float>(half), 0.0f),
      cv::Point3f(static_cast<float>(-half), static_cast<float>(half), 0.0f),
    };

    cv::Mat rvec;
    cv::Mat tvec;
    const bool ok = cv::solvePnP(object_points, image_points, camera_.camera_matrix,
                                 camera_.dist_coeffs, rvec, tvec);
    if (!ok)
    {
      return false;
    }

    cv::Mat rotation_matrix;
    cv::Rodrigues(rvec, rotation_matrix);
    fillBaseTracking(detections, camera_, tracking);
    fillPose(rotation_matrix, tvec, tracking);
    return true;
  }

private:
  CameraConfig camera_;
  int dictionary_id_ = 0;
  int marker_id_ = 0;
  double marker_length_ = 0.0;
};

class ArucoBoardTracker final : public Tracker
{
public:
  bool load(const std::string& tracking_config_path, const CameraConfig& camera,
            std::string* error) override
  {
    camera_ = camera;

    YAML::Node root;
    if (!loadYamlFile(tracking_config_path, &root, error))
    {
      return false;
    }

    std::string layout_offsets_path;
    if (!requireInt(root, "dictionaryId", &dictionary_id_, error) ||
        !requireString(root, "layoutOffsetsPath", &layout_offsets_path, error) ||
        !requireDouble(root, "smallMarkerLength", &small_marker_length_, error) ||
        !readSizeUnitScale(root["sizeUnitScale"], error))
    {
      return false;
    }

    if (small_marker_length_ <= 0.0)
    {
      if (error)
      {
        *error = "smallMarkerLength must be positive";
      }
      return false;
    }

    if (layout_offsets_path.empty())
    {
      if (error)
      {
        *error = "layoutOffsetsPath must be non-empty";
      }
      return false;
    }

    layout_offsets_path_ = resolvePackagePath(layout_offsets_path);
    if (!loadLayoutOffsets(layout_offsets_path_, error))
    {
      return false;
    }

    ROS_INFO_STREAM("Loaded ArucoBoardTracker. dictionaryId=" << dictionary_id_
                    << ", markers=" << marker_object_points_.size()
                    << ", layoutOffsetsPath=" << layout_offsets_path_);
    return true;
  }

  bool track(const DetectionArray& detections, Tracking* tracking) override
  {
    if (!tracking)
    {
      return false;
    }

    std::vector<cv::Point3f> object_points;
    std::vector<cv::Point2f> image_points;
    for (const Detection& detection : detections.detections)
    {
      if (!isArucoDetection(detection))
      {
        continue;
      }

      const auto object_points_for_marker = marker_object_points_.find(detection.class_id);
      if (object_points_for_marker == marker_object_points_.end())
      {
        continue;
      }

      std::vector<cv::Point2f> marker_corners;
      if (!detectionToCorners(detection, &marker_corners))
      {
        ROS_WARN_THROTTLE(2.0, "aruco_board detection has invalid corners");
        continue;
      }

      object_points.insert(object_points.end(),
                           object_points_for_marker->second.begin(),
                           object_points_for_marker->second.end());
      image_points.insert(image_points.end(), marker_corners.begin(), marker_corners.end());
    }

    if (object_points.size() < 4 || image_points.size() != object_points.size())
    {
      return false;
    }

    cv::Mat rvec;
    cv::Mat tvec;
    const bool ok = cv::solvePnP(object_points, image_points, camera_.camera_matrix,
                                 camera_.dist_coeffs, rvec, tvec, false,
                                 cv::SOLVEPNP_ITERATIVE);
    if (!ok)
    {
      return false;
    }

    cv::Mat rotation_board;
    cv::Rodrigues(rvec, rotation_board);

    cv::Mat axis_flip = (cv::Mat_<double>(3, 3) << 1.0, 0.0, 0.0,
                                                  0.0, -1.0, 0.0,
                                                  0.0, 0.0, -1.0);
    cv::Mat rotation_target = rotation_board * axis_flip;

    fillBaseTracking(detections, camera_, tracking);
    fillPose(rotation_target, tvec, tracking);
    return true;
  }

private:
  bool readSizeUnitScale(const YAML::Node& node, std::string* error)
  {
    if (!node || !node.IsMap())
    {
      if (error)
      {
        *error = "sizeUnitScale must be a map";
      }
      return false;
    }

    size_unit_scale_.clear();
    for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
    {
      std::string size_type;
      double scale = 0.0;
      try
      {
        size_type = it->first.as<std::string>();
        scale = it->second.as<double>();
      }
      catch (const YAML::Exception& e)
      {
        if (error)
        {
          *error = "invalid sizeUnitScale entry: " + std::string(e.what());
        }
        return false;
      }

      if (size_type.empty() || scale <= 0.0)
      {
        if (error)
        {
          *error = "sizeUnitScale entries must have non-empty names and positive scales";
        }
        return false;
      }
      size_unit_scale_[size_type] = scale;
    }

    if (size_unit_scale_.find("small") == size_unit_scale_.end() ||
        size_unit_scale_.find("big") == size_unit_scale_.end())
    {
      if (error)
      {
        *error = "sizeUnitScale must contain 'small' and 'big'";
      }
      return false;
    }

    return true;
  }

  bool loadLayoutOffsets(const std::string& path, std::string* error)
  {
    std::ifstream file(path);
    if (!file.is_open())
    {
      if (error)
      {
        *error = "failed to open layout offsets CSV: " + path;
      }
      return false;
    }

    std::string header_line;
    if (!std::getline(file, header_line))
    {
      if (error)
      {
        *error = "layout offsets CSV is empty: " + path;
      }
      return false;
    }

    const std::vector<std::string> headers = splitCsvLine(header_line);
    std::map<std::string, std::size_t> columns;
    for (std::size_t i = 0; i < headers.size(); ++i)
    {
      columns[headers[i]] = i;
    }

    const char* required_columns[] = { "id", "size_type", "x_offset_unit", "y_offset_unit" };
    for (const char* column : required_columns)
    {
      if (columns.find(column) == columns.end())
      {
        if (error)
        {
          *error = "layout offsets CSV missing required column: " + std::string(column);
        }
        return false;
      }
    }

    marker_object_points_.clear();
    std::set<int> seen_ids;
    std::string line;
    int row_number = 1;
    while (std::getline(file, line))
    {
      ++row_number;
      if (trim(line).empty())
      {
        continue;
      }

      const std::vector<std::string> fields = splitCsvLine(line);
      if (fields.size() < headers.size())
      {
        if (error)
        {
          *error = "layout offsets CSV row " + std::to_string(row_number) + " has too few columns";
        }
        return false;
      }

      int marker_id = 0;
      double x_offset_unit = 0.0;
      double y_offset_unit = 0.0;
      const std::string size_type = fields[columns["size_type"]];
      if (!parseInt(fields[columns["id"]], &marker_id) ||
          !parseDouble(fields[columns["x_offset_unit"]], &x_offset_unit) ||
          !parseDouble(fields[columns["y_offset_unit"]], &y_offset_unit))
      {
        if (error)
        {
          *error = "layout offsets CSV row " + std::to_string(row_number) + " has invalid numeric values";
        }
        return false;
      }

      if (!seen_ids.insert(marker_id).second)
      {
        if (error)
        {
          *error = "layout offsets CSV has duplicate marker id: " + std::to_string(marker_id);
        }
        return false;
      }

      const auto scale_it = size_unit_scale_.find(size_type);
      if (scale_it == size_unit_scale_.end())
      {
        if (error)
        {
          *error = "layout offsets CSV row " + std::to_string(row_number) +
                   " uses unknown size_type: " + size_type;
        }
        return false;
      }

      const double side = small_marker_length_ * scale_it->second;
      if (side <= 0.0)
      {
        if (error)
        {
          *error = "marker side length must be positive";
        }
        return false;
      }

      const double center_x = x_offset_unit * small_marker_length_;
      const double center_y = y_offset_unit * small_marker_length_;
      const double half = side / 2.0;
      marker_object_points_[marker_id] = {
        cv::Point3f(static_cast<float>(center_x - half), static_cast<float>(center_y - half), 0.0f),
        cv::Point3f(static_cast<float>(center_x + half), static_cast<float>(center_y - half), 0.0f),
        cv::Point3f(static_cast<float>(center_x + half), static_cast<float>(center_y + half), 0.0f),
        cv::Point3f(static_cast<float>(center_x - half), static_cast<float>(center_y + half), 0.0f),
      };
    }

    if (marker_object_points_.empty())
    {
      if (error)
      {
        *error = "layout offsets CSV contains no markers";
      }
      return false;
    }

    return true;
  }

  CameraConfig camera_;
  int dictionary_id_ = 0;
  double small_marker_length_ = 0.0;
  std::string layout_offsets_path_;
  std::map<std::string, double> size_unit_scale_;
  std::map<int, std::vector<cv::Point3f>> marker_object_points_;
};

class NpuTracker final : public Tracker
{
public:
  bool load(const std::string& tracking_config_path, const CameraConfig& camera,
            std::string* error) override
  {
    (void)tracking_config_path;
    (void)camera;
    (void)error;
    ROS_WARN("NpuTracker is a placeholder and will not publish valid tracking.");
    return true;
  }

  bool track(const DetectionArray& detections, Tracking* tracking) override
  {
    (void)detections;
    (void)tracking;
    ROS_WARN_THROTTLE(5.0, "NpuTracker is not implemented; publishing no tracking.");
    return false;
  }
};

}  // namespace

std::unique_ptr<Tracker> createTracker(const std::string& tracker_type)
{
  if (tracker_type == "single_aruco")
  {
    return std::unique_ptr<Tracker>(new SingleArucoTracker());
  }
  if (tracker_type == "aruco_board")
  {
    return std::unique_ptr<Tracker>(new ArucoBoardTracker());
  }
  if (tracker_type == "npu")
  {
    return std::unique_ptr<Tracker>(new NpuTracker());
  }
  return nullptr;
}

}  // namespace sunray_perception
