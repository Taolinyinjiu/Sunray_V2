#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <yaml-cpp/yaml.h>

namespace sunray_perception
{

struct CameraConfig
{
  int width = 0;
  int height = 0;
  cv::Mat camera_matrix;
  cv::Mat dist_coeffs;
  double fov_x = 0.0;
  double fov_y = 0.0;
};

bool loadYamlFile(const std::string& path, YAML::Node* root, std::string* error);

bool requireInt(const YAML::Node& root, const std::string& key, int* value, std::string* error);
bool requireDouble(const YAML::Node& root, const std::string& key, double* value, std::string* error);
bool requireString(const YAML::Node& root, const std::string& key, std::string* value, std::string* error);
bool requireBool(const YAML::Node& root, const std::string& key, bool* value, std::string* error);

bool optionalInt(const YAML::Node& root, const std::string& key, int* value, std::string* error);
bool optionalDouble(const YAML::Node& root, const std::string& key, double* value, std::string* error);
bool optionalFloat(const YAML::Node& root, const std::string& key, float* value, std::string* error);
bool optionalBool(const YAML::Node& root, const std::string& key, bool* value, std::string* error);

bool loadCameraConfig(const std::string& path, CameraConfig* config, std::string* error);
cv::Vec3d rotationMatrixToRpy(const cv::Mat& rotation_matrix);

}  // namespace sunray_perception
