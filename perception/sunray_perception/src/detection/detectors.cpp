#include <sunray_perception/detection/detectors.h>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <sunray_perception/common/config_utils.h>
#include <sunray_perception/detection/npu_backend.h>

namespace sunray_perception
{
namespace
{

class ArucoDetector final : public Detector
{
public:
  explicit ArucoDetector(std::string detector_type)
    : detector_type_(std::move(detector_type))
  {
  }

  bool load(const std::string& config_path, const std::string& model_path, std::string* error) override
  {
    (void)model_path;

    YAML::Node root;
    if (!loadYamlFile(config_path, &root, error))
    {
      return false;
    }

    int dictionary_id = 0;
    if (!requireInt(root, "dictionaryId", &dictionary_id, error))
    {
      return false;
    }

    dictionary_ = cv::aruco::getPredefinedDictionary(dictionary_id);
    parameters_ = cv::aruco::DetectorParameters::create();

    if (!optionalInt(root, "adaptiveThreshWinSizeMin", &parameters_->adaptiveThreshWinSizeMin, error) ||
        !optionalInt(root, "adaptiveThreshWinSizeMax", &parameters_->adaptiveThreshWinSizeMax, error) ||
        !optionalInt(root, "adaptiveThreshWinSizeStep", &parameters_->adaptiveThreshWinSizeStep, error) ||
        !optionalDouble(root, "adaptiveThreshConstant", &parameters_->adaptiveThreshConstant, error) ||
        !optionalDouble(root, "minMarkerPerimeterRate", &parameters_->minMarkerPerimeterRate, error) ||
        !optionalDouble(root, "maxMarkerPerimeterRate", &parameters_->maxMarkerPerimeterRate, error) ||
        !optionalDouble(root, "polygonalApproxAccuracyRate", &parameters_->polygonalApproxAccuracyRate, error) ||
        !optionalDouble(root, "minCornerDistanceRate", &parameters_->minCornerDistanceRate, error) ||
        !optionalInt(root, "minDistanceToBorder", &parameters_->minDistanceToBorder, error) ||
        !optionalDouble(root, "minMarkerDistanceRate", &parameters_->minMarkerDistanceRate, error) ||
        !optionalInt(root, "cornerRefinementMethod", &parameters_->cornerRefinementMethod, error) ||
        !optionalInt(root, "cornerRefinementWinSize", &parameters_->cornerRefinementWinSize, error) ||
        !optionalInt(root, "cornerRefinementMaxIterations", &parameters_->cornerRefinementMaxIterations, error) ||
        !optionalDouble(root, "cornerRefinementMinAccuracy", &parameters_->cornerRefinementMinAccuracy, error) ||
        !optionalInt(root, "markerBorderBits", &parameters_->markerBorderBits, error) ||
        !optionalInt(root, "perspectiveRemovePixelPerCell", &parameters_->perspectiveRemovePixelPerCell, error) ||
        !optionalDouble(root, "perspectiveRemoveIgnoredMarginPerCell",
                        &parameters_->perspectiveRemoveIgnoredMarginPerCell, error) ||
        !optionalDouble(root, "maxErroneousBitsInBorderRate", &parameters_->maxErroneousBitsInBorderRate, error) ||
        !optionalDouble(root, "minOtsuStdDev", &parameters_->minOtsuStdDev, error) ||
        !optionalDouble(root, "errorCorrectionRate", &parameters_->errorCorrectionRate, error) ||
        !optionalFloat(root, "aprilTagQuadDecimate", &parameters_->aprilTagQuadDecimate, error) ||
        !optionalFloat(root, "aprilTagQuadSigma", &parameters_->aprilTagQuadSigma, error) ||
        !optionalInt(root, "aprilTagMinClusterPixels", &parameters_->aprilTagMinClusterPixels, error) ||
        !optionalInt(root, "aprilTagMaxNmaxima", &parameters_->aprilTagMaxNmaxima, error) ||
        !optionalFloat(root, "aprilTagCriticalRad", &parameters_->aprilTagCriticalRad, error) ||
        !optionalFloat(root, "aprilTagMaxLineFitMse", &parameters_->aprilTagMaxLineFitMse, error) ||
        !optionalInt(root, "aprilTagMinWhiteBlackDiff", &parameters_->aprilTagMinWhiteBlackDiff, error) ||
        !optionalInt(root, "aprilTagDeglitch", &parameters_->aprilTagDeglitch, error) ||
        !optionalBool(root, "detectInvertedMarker", &parameters_->detectInvertedMarker, error))
    {
      return false;
    }

    ROS_INFO_STREAM("Loaded " << detector_type_ << " detector config: " << config_path);
    return true;
  }

  bool detect(const cv::Mat& image, const std_msgs::Header& header,
              DetectionArray* detections, cv::Mat* debug_image) override
  {
    if (!detections || !debug_image)
    {
      return false;
    }

    detections->header = header;
    detections->detections.clear();
    image.copyTo(*debug_image);

    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> rejected;
    cv::aruco::detectMarkers(image, dictionary_, corners, ids, parameters_, rejected);

    if (!ids.empty())
    {
      cv::aruco::drawDetectedMarkers(*debug_image, corners, ids);
    }

    detections->detections.reserve(ids.size());
    for (std::size_t i = 0; i < ids.size(); ++i)
    {
      Detection detection;
      detection.header = header;
      detection.detector_type = detector_type_;
      detection.class_id = ids[i];
      detection.class_name = "aruco_" + std::to_string(ids[i]);
      detection.score = 1.0;

      double min_x = corners[i][0].x;
      double max_x = corners[i][0].x;
      double min_y = corners[i][0].y;
      double max_y = corners[i][0].y;
      double sum_x = 0.0;
      double sum_y = 0.0;

      for (const cv::Point2f& point : corners[i])
      {
        detection.corners_x.push_back(point.x);
        detection.corners_y.push_back(point.y);
        min_x = std::min(min_x, static_cast<double>(point.x));
        max_x = std::max(max_x, static_cast<double>(point.x));
        min_y = std::min(min_y, static_cast<double>(point.y));
        max_y = std::max(max_y, static_cast<double>(point.y));
        sum_x += point.x;
        sum_y += point.y;
      }

      detection.center_x = sum_x / static_cast<double>(corners[i].size());
      detection.center_y = sum_y / static_cast<double>(corners[i].size());
      detection.width = max_x - min_x;
      detection.height = max_y - min_y;
      detections->detections.push_back(detection);
    }

    return true;
  }

private:
  std::string detector_type_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> parameters_;
};

class NpuDetector final : public Detector
{
public:
  bool load(const std::string& config_path, const std::string& model_path, std::string* error) override
  {
    if (model_path.empty())
    {
      if (error)
      {
        *error = "model_path is required for npu detector";
      }
      return false;
    }
    if (!fileExists(model_path))
    {
      if (error)
      {
        *error = "model file does not exist: " + model_path;
      }
      return false;
    }

    YAML::Node root;
    if (!loadYamlFile(config_path, &root, error))
    {
      return false;
    }

    if (!requireString(root, "backend_type", &backend_type_, error) ||
        !requireInt(root, "input_width", &input_width_, error) ||
        !requireInt(root, "input_height", &input_height_, error) ||
        !requireDouble(root, "confidence_threshold", &confidence_threshold_, error) ||
        !requireDouble(root, "nms_iou_threshold", &nms_iou_threshold_, error) ||
        !requireInt(root, "max_detections", &max_detections_, error))
    {
      return false;
    }

    if (backend_type_ != "rk3588" && backend_type_ != "orin_nx")
    {
      if (error)
      {
        *error = "backend_type must be 'rk3588' or 'orin_nx'";
      }
      return false;
    }

    if (input_width_ <= 0 || input_height_ <= 0)
    {
      if (error)
      {
        *error = "input_width and input_height must be positive";
      }
      return false;
    }
    if (confidence_threshold_ < 0.0 || confidence_threshold_ > 1.0 ||
        nms_iou_threshold_ < 0.0 || nms_iou_threshold_ > 1.0)
    {
      if (error)
      {
        *error = "confidence_threshold and nms_iou_threshold must be in [0, 1]";
      }
      return false;
    }
    if (max_detections_ <= 0)
    {
      if (error)
      {
        *error = "max_detections must be positive";
      }
      return false;
    }

    if (!readBgrColor(root, "letterbox_color_bgr", &letterbox_color_bgr_, error) ||
        !readBgrColor(root, "box_color_bgr", &box_color_bgr_, error) ||
        !readClassNames(root, error))
    {
      return false;
    }

    input_spec_.width = input_width_;
    input_spec_.height = input_height_;
    input_spec_.channels = 3;
    input_spec_.layout = backend_type_ == "rk3588" ? NpuTensorLayout::NHWC : NpuTensorLayout::NCHW;
    input_spec_.type = backend_type_ == "rk3588" ? NpuTensorType::UINT8 : NpuTensorType::FLOAT32;

    backend_ = createNpuBackend(backend_type_, error);
    if (!backend_)
    {
      return false;
    }
    if (!backend_->load(model_path, input_spec_, error))
    {
      return false;
    }
    input_spec_ = backend_->inputSpec();

    ROS_INFO_STREAM("Loaded npu detector: backend_type=" << backend_type_
                    << ", model_path=" << model_path
                    << ", input=" << input_width_ << "x" << input_height_);
    return true;
  }

  bool detect(const cv::Mat& image, const std_msgs::Header& header,
              DetectionArray* detections, cv::Mat* debug_image) override
  {
    if (!detections || !debug_image)
    {
      return false;
    }

    detections->header = header;
    detections->detections.clear();
    image.copyTo(*debug_image);

    LetterboxInfo letterbox;
    std::vector<unsigned char> input;
    preprocess(image, &input, &letterbox);

    std::vector<NpuTensor> outputs;
    std::string error;
    if (!backend_->infer(input.data(), input.size(), &outputs, &error))
    {
      ROS_ERROR_STREAM_THROTTLE(2.0, "NPU inference failed: " << error);
      return false;
    }

    std::vector<DecodedDetection> decoded;
    if (!decodeYolo26(outputs, image.cols, image.rows, letterbox, &decoded, &error))
    {
      ROS_ERROR_STREAM_THROTTLE(2.0, "NPU postprocess failed: " << error);
      return false;
    }

    detections->detections.reserve(decoded.size());
    for (const DecodedDetection& item : decoded)
    {
      Detection detection;
      detection.header = header;
      detection.detector_type = "npu";
      detection.class_id = item.class_id;
      detection.class_name = className(item.class_id);
      detection.score = item.score;
      detection.corners_x = { item.x1, item.x2, item.x2, item.x1 };
      detection.corners_y = { item.y1, item.y1, item.y2, item.y2 };
      detection.center_x = (item.x1 + item.x2) * 0.5;
      detection.center_y = (item.y1 + item.y2) * 0.5;
      detection.width = item.x2 - item.x1;
      detection.height = item.y2 - item.y1;
      detections->detections.push_back(detection);

      const cv::Point top_left(static_cast<int>(std::round(item.x1)), static_cast<int>(std::round(item.y1)));
      const cv::Point bottom_right(static_cast<int>(std::round(item.x2)), static_cast<int>(std::round(item.y2)));
      cv::rectangle(*debug_image, top_left, bottom_right, box_color_bgr_, 2);

      std::ostringstream label;
      label.setf(std::ios::fixed);
      label.precision(2);
      label << detection.class_name << " " << item.score;
      int baseline = 0;
      const cv::Size text_size = cv::getTextSize(label.str(), cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
      const int text_y = std::max(top_left.y, text_size.height + baseline);
      cv::rectangle(*debug_image,
                    cv::Point(top_left.x, text_y - text_size.height - baseline),
                    cv::Point(top_left.x + text_size.width, text_y + baseline),
                    box_color_bgr_, cv::FILLED);
      cv::putText(*debug_image, label.str(), cv::Point(top_left.x, text_y - baseline),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
    }
    return true;
  }

private:
  struct LetterboxInfo
  {
    double scale = 1.0;
    double pad_x = 0.0;
    double pad_y = 0.0;
  };

  struct DecodedDetection
  {
    int class_id = -1;
    double score = 0.0;
    double x1 = 0.0;
    double y1 = 0.0;
    double x2 = 0.0;
    double y2 = 0.0;
  };

  static bool fileExists(const std::string& path)
  {
    std::ifstream file(path.c_str(), std::ios::binary);
    return file.good();
  }

  static bool readBgrColor(const YAML::Node& root, const std::string& key, cv::Scalar* color, std::string* error)
  {
    if (!root[key] || !root[key].IsSequence() || root[key].size() != 3)
    {
      if (error)
      {
        *error = key + " must be a 3-element sequence";
      }
      return false;
    }

    int values[3] = { 0, 0, 0 };
    for (std::size_t i = 0; i < root[key].size(); ++i)
    {
      try
      {
        values[i] = root[key][i].as<int>();
      }
      catch (const YAML::Exception& e)
      {
        if (error)
        {
          *error = "invalid " + key + " value: " + std::string(e.what());
        }
        return false;
      }
      if (values[i] < 0 || values[i] > 255)
      {
        if (error)
        {
          *error = key + " values must be in [0, 255]";
        }
        return false;
      }
    }
    *color = cv::Scalar(values[0], values[1], values[2]);
    return true;
  }

  bool readClassNames(const YAML::Node& root, std::string* error)
  {
    if (!root["class_names"] || !root["class_names"].IsMap())
    {
      if (error)
      {
        *error = "class_names must be a map";
      }
      return false;
    }

    class_names_.clear();
    for (const auto& item : root["class_names"])
    {
      try
      {
        class_names_[item.first.as<int>()] = item.second.as<std::string>();
      }
      catch (const YAML::Exception& e)
      {
        if (error)
        {
          *error = "invalid class_names entry: " + std::string(e.what());
        }
        return false;
      }
    }
    return true;
  }

  void preprocess(const cv::Mat& image, std::vector<unsigned char>* input, LetterboxInfo* letterbox) const
  {
    const double scale = std::min(static_cast<double>(input_width_) / static_cast<double>(image.cols),
                                  static_cast<double>(input_height_) / static_cast<double>(image.rows));
    const int resized_width = static_cast<int>(std::round(static_cast<double>(image.cols) * scale));
    const int resized_height = static_cast<int>(std::round(static_cast<double>(image.rows) * scale));
    const int pad_left = (input_width_ - resized_width) / 2;
    const int pad_top = (input_height_ - resized_height) / 2;

    cv::Mat resized;
    cv::resize(image, resized, cv::Size(resized_width, resized_height), 0.0, 0.0, cv::INTER_LINEAR);

    cv::Mat letterboxed(input_height_, input_width_, CV_8UC3, letterbox_color_bgr_);
    resized.copyTo(letterboxed(cv::Rect(pad_left, pad_top, resized_width, resized_height)));

    cv::Mat rgb;
    cv::cvtColor(letterboxed, rgb, cv::COLOR_BGR2RGB);

    std::vector<float> normalized(static_cast<std::size_t>(input_width_) * input_height_ * 3, 0.0f);
    if (input_spec_.layout == NpuTensorLayout::NCHW)
    {
      const int channel_size = input_width_ * input_height_;
      for (int y = 0; y < input_height_; ++y)
      {
        const cv::Vec3b* row = rgb.ptr<cv::Vec3b>(y);
        for (int x = 0; x < input_width_; ++x)
        {
          for (int c = 0; c < 3; ++c)
          {
            normalized[static_cast<std::size_t>(c * channel_size + y * input_width_ + x)] =
                static_cast<float>(row[x][c]) / 255.0f;
          }
        }
      }
    }
    else
    {
      std::size_t index = 0;
      for (int y = 0; y < input_height_; ++y)
      {
        const cv::Vec3b* row = rgb.ptr<cv::Vec3b>(y);
        for (int x = 0; x < input_width_; ++x)
        {
          for (int c = 0; c < 3; ++c)
          {
            normalized[index++] = static_cast<float>(row[x][c]) / 255.0f;
          }
        }
      }
    }
    packInput(normalized, input);

    letterbox->scale = scale;
    letterbox->pad_x = pad_left;
    letterbox->pad_y = pad_top;
  }

  void packInput(const std::vector<float>& normalized, std::vector<unsigned char>* input) const
  {
    if (input_spec_.type == NpuTensorType::FLOAT32)
    {
      input->resize(normalized.size() * sizeof(float));
      std::memcpy(input->data(), normalized.data(), input->size());
      return;
    }
    if (input_spec_.type == NpuTensorType::FLOAT16)
    {
      input->resize(normalized.size() * sizeof(uint16_t));
      uint16_t* values = reinterpret_cast<uint16_t*>(input->data());
      for (std::size_t i = 0; i < normalized.size(); ++i)
      {
        values[i] = floatToHalf(normalized[i]);
      }
      return;
    }

    input->resize(normalized.size());
    for (std::size_t i = 0; i < normalized.size(); ++i)
    {
      input->at(i) = static_cast<unsigned char>(std::round(clip(normalized[i], 0.0, 1.0) * 255.0));
    }
  }

  static uint16_t floatToHalf(float value)
  {
    uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));

    const uint32_t sign = (bits >> 16) & 0x8000u;
    int exponent = static_cast<int>((bits >> 23) & 0xffu) - 127 + 15;
    uint32_t mantissa = bits & 0x7fffffu;

    if (exponent <= 0)
    {
      if (exponent < -10)
      {
        return static_cast<uint16_t>(sign);
      }
      mantissa = (mantissa | 0x800000u) >> static_cast<uint32_t>(1 - exponent);
      return static_cast<uint16_t>(sign | ((mantissa + 0x1000u) >> 13));
    }
    if (exponent >= 31)
    {
      return static_cast<uint16_t>(sign | 0x7c00u);
    }

    return static_cast<uint16_t>(sign | (static_cast<uint32_t>(exponent) << 10) |
                                 ((mantissa + 0x1000u) >> 13));
  }

  bool decodeYolo26(const std::vector<NpuTensor>& outputs, int image_width, int image_height,
                    const LetterboxInfo& letterbox, std::vector<DecodedDetection>* detections,
                    std::string* error) const
  {
    if (!detections)
    {
      if (error)
      {
        *error = "internal error: null decoded detection output";
      }
      return false;
    }
    if (outputs.empty())
    {
      if (error)
      {
        *error = "model returned no output tensors";
      }
      return false;
    }

    const NpuTensor* output = &outputs[0];
    if (output->shape.size() != 3 || output->shape[0] != 1 || output->shape[1] <= 4 ||
        output->shape[2] <= 0)
    {
      if (error)
      {
        std::ostringstream oss;
        oss << "expected YOLO26 one-to-many output shape (1, nc + 4, 8400), got "
            << shapeToString(output->shape);
        *error = oss.str();
      }
      return false;
    }

    const int channels = output->shape[1];
    const int candidates = output->shape[2];
    const int class_count = channels - 4;
    if (candidates != 8400)
    {
      ROS_WARN_STREAM_THROTTLE(5.0, "YOLO26 output candidate count is " << candidates
                               << ", expected 8400; decoding by provided shape.");
    }

    const std::size_t expected = static_cast<std::size_t>(channels) * candidates;
    if (output->data.size() < expected)
    {
      if (error)
      {
        std::ostringstream oss;
        oss << "output tensor data is too small: got " << output->data.size()
            << " floats, expected at least " << expected;
        *error = oss.str();
      }
      return false;
    }

    std::vector<DecodedDetection> candidates_out;
    for (int index = 0; index < candidates; ++index)
    {
      int best_class = 0;
      float best_score = output->data[static_cast<std::size_t>((4 + best_class) * candidates + index)];
      for (int class_id = 1; class_id < class_count; ++class_id)
      {
        const float score = output->data[static_cast<std::size_t>((4 + class_id) * candidates + index)];
        if (score > best_score)
        {
          best_score = score;
          best_class = class_id;
        }
      }

      if (best_score < confidence_threshold_)
      {
        continue;
      }

      const double cx = output->data[static_cast<std::size_t>(0 * candidates + index)];
      const double cy = output->data[static_cast<std::size_t>(1 * candidates + index)];
      const double width = output->data[static_cast<std::size_t>(2 * candidates + index)];
      const double height = output->data[static_cast<std::size_t>(3 * candidates + index)];

      DecodedDetection detection;
      detection.class_id = best_class;
      detection.score = best_score;
      detection.x1 = clip(((cx - width * 0.5) - letterbox.pad_x) / letterbox.scale, 0.0,
                          static_cast<double>(image_width));
      detection.y1 = clip(((cy - height * 0.5) - letterbox.pad_y) / letterbox.scale, 0.0,
                          static_cast<double>(image_height));
      detection.x2 = clip(((cx + width * 0.5) - letterbox.pad_x) / letterbox.scale, 0.0,
                          static_cast<double>(image_width));
      detection.y2 = clip(((cy + height * 0.5) - letterbox.pad_y) / letterbox.scale, 0.0,
                          static_cast<double>(image_height));
      if (detection.x2 <= detection.x1 || detection.y2 <= detection.y1)
      {
        continue;
      }
      candidates_out.push_back(detection);
    }

    std::sort(candidates_out.begin(), candidates_out.end(),
              [](const DecodedDetection& a, const DecodedDetection& b) {
                return a.score > b.score;
              });

    detections->clear();
    for (const DecodedDetection& candidate : candidates_out)
    {
      bool suppressed = false;
      for (const DecodedDetection& kept : *detections)
      {
        if (candidate.class_id == kept.class_id && iou(candidate, kept) > nms_iou_threshold_)
        {
          suppressed = true;
          break;
        }
      }
      if (!suppressed)
      {
        detections->push_back(candidate);
        if (static_cast<int>(detections->size()) >= max_detections_)
        {
          break;
        }
      }
    }

    return true;
  }

  static double clip(double value, double lower, double upper)
  {
    return std::max(lower, std::min(value, upper));
  }

  static double iou(const DecodedDetection& a, const DecodedDetection& b)
  {
    const double inter_x1 = std::max(a.x1, b.x1);
    const double inter_y1 = std::max(a.y1, b.y1);
    const double inter_x2 = std::min(a.x2, b.x2);
    const double inter_y2 = std::min(a.y2, b.y2);
    const double inter_w = std::max(0.0, inter_x2 - inter_x1);
    const double inter_h = std::max(0.0, inter_y2 - inter_y1);
    const double intersection = inter_w * inter_h;
    const double area_a = std::max(0.0, a.x2 - a.x1) * std::max(0.0, a.y2 - a.y1);
    const double area_b = std::max(0.0, b.x2 - b.x1) * std::max(0.0, b.y2 - b.y1);
    const double union_area = area_a + area_b - intersection;
    if (union_area <= 0.0)
    {
      return 0.0;
    }
    return intersection / union_area;
  }

  static std::string shapeToString(const std::vector<int>& shape)
  {
    std::ostringstream oss;
    oss << "(";
    for (std::size_t i = 0; i < shape.size(); ++i)
    {
      if (i > 0)
      {
        oss << ", ";
      }
      oss << shape[i];
    }
    oss << ")";
    return oss.str();
  }

  std::string className(int class_id) const
  {
    const auto it = class_names_.find(class_id);
    if (it != class_names_.end())
    {
      return it->second;
    }
    return "class_" + std::to_string(class_id);
  }

  std::string backend_type_;
  int input_width_ = 0;
  int input_height_ = 0;
  double confidence_threshold_ = 0.0;
  double nms_iou_threshold_ = 0.0;
  int max_detections_ = 0;
  cv::Scalar letterbox_color_bgr_;
  cv::Scalar box_color_bgr_;
  std::map<int, std::string> class_names_;
  NpuInputSpec input_spec_;
  std::unique_ptr<NpuBackend> backend_;
};

}  // namespace

std::unique_ptr<Detector> createDetector(const std::string& detector_type)
{
  if (detector_type == "aruco")
  {
    return std::unique_ptr<Detector>(new ArucoDetector("aruco"));
  }
  if (detector_type == "npu")
  {
    return std::unique_ptr<Detector>(new NpuDetector());
  }
  return nullptr;
}

}  // namespace sunray_perception
