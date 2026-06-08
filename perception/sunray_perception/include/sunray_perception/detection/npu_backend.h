#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace sunray_perception
{

enum class NpuTensorLayout
{
  NCHW,
  NHWC
};

enum class NpuTensorType
{
  FLOAT32,
  FLOAT16,
  UINT8
};

struct NpuInputSpec
{
  int width = 0;
  int height = 0;
  int channels = 3;
  NpuTensorLayout layout = NpuTensorLayout::NCHW;
  NpuTensorType type = NpuTensorType::FLOAT32;
};

struct NpuTensor
{
  std::vector<int> shape;
  std::vector<float> data;
};

class NpuBackend
{
public:
  virtual ~NpuBackend() = default;

  virtual bool load(const std::string& model_path, const NpuInputSpec& input_spec, std::string* error) = 0;
  virtual NpuInputSpec inputSpec() const = 0;
  virtual bool infer(const void* input_data, std::size_t input_bytes,
                     std::vector<NpuTensor>* outputs, std::string* error) = 0;
};

std::unique_ptr<NpuBackend> createNpuBackend(const std::string& backend_type, std::string* error);

}  // namespace sunray_perception
