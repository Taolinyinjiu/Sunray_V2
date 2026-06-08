#include <sunray_perception/common/config_utils.h>

#include <cmath>
#include <sstream>

namespace sunray_perception
{
namespace
{

template <typename T>
bool requireScalar(const YAML::Node& root, const std::string& key, T* value, std::string* error)
{
  if (!root[key])
  {
    if (error)
    {
      *error = "missing required key '" + key + "'";
    }
    return false;
  }

  try
  {
    *value = root[key].as<T>();
  }
  catch (const YAML::Exception& e)
  {
    if (error)
    {
      *error = "invalid value for key '" + key + "': " + e.what();
    }
    return false;
  }
  return true;
}

template <typename T>
bool optionalScalar(const YAML::Node& root, const std::string& key, T* value, std::string* error)
{
  if (!root[key])
  {
    return true;
  }

  try
  {
    *value = root[key].as<T>();
  }
  catch (const YAML::Exception& e)
  {
    if (error)
    {
      *error = "invalid value for key '" + key + "': " + e.what();
    }
    return false;
  }
  return true;
}

bool readMatrix3x3(const YAML::Node& node, cv::Mat* matrix, std::string* error)
{
  if (!node || !node.IsSequence() || node.size() != 3)
  {
    if (error)
    {
      *error = "camera_matrix must be a 3x3 sequence";
    }
    return false;
  }

  *matrix = cv::Mat::zeros(3, 3, CV_64F);
  for (std::size_t row = 0; row < 3; ++row)
  {
    if (!node[row].IsSequence() || node[row].size() != 3)
    {
      if (error)
      {
        *error = "camera_matrix must be a 3x3 sequence";
      }
      return false;
    }
    for (std::size_t col = 0; col < 3; ++col)
    {
      try
      {
        matrix->at<double>(static_cast<int>(row), static_cast<int>(col)) = node[row][col].as<double>();
      }
      catch (const YAML::Exception& e)
      {
        if (error)
        {
          *error = "invalid camera_matrix value: " + std::string(e.what());
        }
        return false;
      }
    }
  }

  return true;
}

bool readDistortion(const YAML::Node& node, cv::Mat* dist_coeffs, std::string* error)
{
  if (!node || !node.IsSequence() || node.size() == 0)
  {
    if (error)
    {
      *error = "distortion must be a non-empty sequence";
    }
    return false;
  }

  *dist_coeffs = cv::Mat::zeros(1, static_cast<int>(node.size()), CV_64F);
  for (std::size_t i = 0; i < node.size(); ++i)
  {
    try
    {
      dist_coeffs->at<double>(0, static_cast<int>(i)) = node[i].as<double>();
    }
    catch (const YAML::Exception& e)
    {
      if (error)
      {
        *error = "invalid distortion value: " + std::string(e.what());
      }
      return false;
    }
  }

  return true;
}

}  // namespace

bool loadYamlFile(const std::string& path, YAML::Node* root, std::string* error)
{
  if (!root)
  {
    if (error)
    {
      *error = "internal error: null YAML root output";
    }
    return false;
  }

  try
  {
    *root = YAML::LoadFile(path);
  }
  catch (const YAML::Exception& e)
  {
    if (error)
    {
      *error = "failed to read '" + path + "': " + e.what();
    }
    return false;
  }

  if (!root->IsMap())
  {
    if (error)
    {
      *error = "YAML file '" + path + "' must contain a map at the root";
    }
    return false;
  }

  return true;
}

bool requireInt(const YAML::Node& root, const std::string& key, int* value, std::string* error)
{
  return requireScalar(root, key, value, error);
}

bool requireDouble(const YAML::Node& root, const std::string& key, double* value, std::string* error)
{
  return requireScalar(root, key, value, error);
}

bool requireString(const YAML::Node& root, const std::string& key, std::string* value, std::string* error)
{
  return requireScalar(root, key, value, error);
}

bool requireBool(const YAML::Node& root, const std::string& key, bool* value, std::string* error)
{
  return requireScalar(root, key, value, error);
}

bool optionalInt(const YAML::Node& root, const std::string& key, int* value, std::string* error)
{
  return optionalScalar(root, key, value, error);
}

bool optionalDouble(const YAML::Node& root, const std::string& key, double* value, std::string* error)
{
  return optionalScalar(root, key, value, error);
}

bool optionalFloat(const YAML::Node& root, const std::string& key, float* value, std::string* error)
{
  return optionalScalar(root, key, value, error);
}

bool optionalBool(const YAML::Node& root, const std::string& key, bool* value, std::string* error)
{
  return optionalScalar(root, key, value, error);
}

bool loadCameraConfig(const std::string& path, CameraConfig* config, std::string* error)
{
  if (!config)
  {
    if (error)
    {
      *error = "internal error: null camera config output";
    }
    return false;
  }

  YAML::Node root;
  if (!loadYamlFile(path, &root, error))
  {
    return false;
  }

  if (!requireInt(root, "image_width", &config->width, error) ||
      !requireInt(root, "image_height", &config->height, error) ||
      !readMatrix3x3(root["camera_matrix"], &config->camera_matrix, error) ||
      !readDistortion(root["distortion"], &config->dist_coeffs, error))
  {
    return false;
  }

  const double fx = config->camera_matrix.at<double>(0, 0);
  const double fy = config->camera_matrix.at<double>(1, 1);
  if (fx <= 0.0 || fy <= 0.0 || config->width <= 0 || config->height <= 0)
  {
    if (error)
    {
      *error = "camera intrinsics and image dimensions must be positive";
    }
    return false;
  }

  config->fov_x = 2.0 * std::atan(static_cast<double>(config->width) / (2.0 * fx));
  config->fov_y = 2.0 * std::atan(static_cast<double>(config->height) / (2.0 * fy));
  return true;
}

cv::Vec3d rotationMatrixToRpy(const cv::Mat& rotation_matrix)
{
  const double r00 = rotation_matrix.at<double>(0, 0);
  const double r10 = rotation_matrix.at<double>(1, 0);
  const double r20 = rotation_matrix.at<double>(2, 0);
  const double r21 = rotation_matrix.at<double>(2, 1);
  const double r22 = rotation_matrix.at<double>(2, 2);
  const double r01 = rotation_matrix.at<double>(0, 1);
  const double r11 = rotation_matrix.at<double>(1, 1);

  const double sy = std::sqrt(r00 * r00 + r10 * r10);
  const bool singular = sy < 1e-6;

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;

  if (!singular)
  {
    roll = std::atan2(r21, r22);
    pitch = std::atan2(-r20, sy);
    yaw = std::atan2(r10, r00);
  }
  else
  {
    roll = std::atan2(-r11, r01);
    pitch = std::atan2(-r20, sy);
    yaw = 0.0;
  }

  return cv::Vec3d(roll, pitch, yaw);
}

}  // namespace sunray_perception
