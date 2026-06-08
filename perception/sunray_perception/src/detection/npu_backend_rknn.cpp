#include <sunray_perception/detection/npu_backend.h>

#include <algorithm>
#include <cstring>
#include <fstream>
#include <sstream>

#ifdef SUNRAY_WITH_RKNN
#include <rknn_api.h>
#endif

namespace sunray_perception
{
namespace
{

#ifndef SUNRAY_WITH_RKNN

class RknnBackend final : public NpuBackend
{
public:
  bool load(const std::string& model_path, const NpuInputSpec& input_spec, std::string* error) override
  {
    input_spec_ = input_spec;
    (void)model_path;
    if (error)
    {
      *error = "RKNN backend was requested but this package was built without WITH_RKNN=ON";
    }
    return false;
  }

  NpuInputSpec inputSpec() const override
  {
    return input_spec_;
  }

  bool infer(const void* input_data, std::size_t input_bytes,
             std::vector<NpuTensor>* outputs, std::string* error) override
  {
    (void)input_data;
    (void)input_bytes;
    (void)outputs;
    if (error)
    {
      *error = "RKNN backend is not available in this build";
    }
    return false;
  }

private:
  NpuInputSpec input_spec_;
};

#else

std::string rknnError(const std::string& action, int ret)
{
  std::ostringstream oss;
  oss << action << " failed, RKNN ret=" << ret;
  return oss.str();
}

bool readBinaryFile(const std::string& path, std::vector<unsigned char>* data, std::string* error)
{
  std::ifstream file(path.c_str(), std::ios::binary);
  if (!file)
  {
    if (error)
    {
      *error = "failed to open model file: " + path;
    }
    return false;
  }

  file.seekg(0, std::ios::end);
  const std::streamoff size = file.tellg();
  if (size <= 0)
  {
    if (error)
    {
      *error = "model file is empty: " + path;
    }
    return false;
  }

  data->resize(static_cast<std::size_t>(size));
  file.seekg(0, std::ios::beg);
  file.read(reinterpret_cast<char*>(data->data()), size);
  if (!file)
  {
    if (error)
    {
      *error = "failed to read model file: " + path;
    }
    return false;
  }
  return true;
}

class RknnBackend final : public NpuBackend
{
public:
  ~RknnBackend() override
  {
    if (ctx_ != 0)
    {
      rknn_destroy(ctx_);
    }
  }

  bool load(const std::string& model_path, const NpuInputSpec& input_spec, std::string* error) override
  {
    input_spec_ = input_spec;
    if (!readBinaryFile(model_path, &model_data_, error))
    {
      return false;
    }

    int ret = rknn_init(&ctx_, model_data_.data(), static_cast<uint32_t>(model_data_.size()), 0, nullptr);
    if (ret != RKNN_SUCC)
    {
      if (error)
      {
        *error = rknnError("rknn_init", ret);
      }
      return false;
    }

    ret = rknn_query(ctx_, RKNN_QUERY_IN_OUT_NUM, &io_num_, sizeof(io_num_));
    if (ret != RKNN_SUCC)
    {
      if (error)
      {
        *error = rknnError("RKNN_QUERY_IN_OUT_NUM", ret);
      }
      return false;
    }
    if (io_num_.n_input != 1 || io_num_.n_output < 1)
    {
      if (error)
      {
        *error = "RKNN model must have exactly one input and at least one output";
      }
      return false;
    }

    output_attrs_.resize(io_num_.n_output);
    for (uint32_t i = 0; i < io_num_.n_output; ++i)
    {
      output_attrs_[i].index = i;
      ret = rknn_query(ctx_, RKNN_QUERY_OUTPUT_ATTR, &output_attrs_[i], sizeof(output_attrs_[i]));
      if (ret != RKNN_SUCC)
      {
        if (error)
        {
          *error = rknnError("RKNN_QUERY_OUTPUT_ATTR", ret);
        }
        return false;
      }
    }

    return true;
  }

  NpuInputSpec inputSpec() const override
  {
    return input_spec_;
  }

  bool infer(const void* input_data, std::size_t input_bytes,
             std::vector<NpuTensor>* outputs, std::string* error) override
  {
    if (!outputs)
    {
      if (error)
      {
        *error = "internal error: null output tensor vector";
      }
      return false;
    }

    rknn_input input;
    memset(&input, 0, sizeof(input));
    input.index = 0;
    input.buf = const_cast<void*>(input_data);
    input.size = static_cast<uint32_t>(input_bytes);
    input.pass_through = 0;
    input.type = input_spec_.type == NpuTensorType::UINT8 ? RKNN_TENSOR_UINT8 : RKNN_TENSOR_FLOAT32;
    input.fmt = input_spec_.layout == NpuTensorLayout::NHWC ? RKNN_TENSOR_NHWC : RKNN_TENSOR_NCHW;

    int ret = rknn_inputs_set(ctx_, 1, &input);
    if (ret != RKNN_SUCC)
    {
      if (error)
      {
        *error = rknnError("rknn_inputs_set", ret);
      }
      return false;
    }

    ret = rknn_run(ctx_, nullptr);
    if (ret != RKNN_SUCC)
    {
      if (error)
      {
        *error = rknnError("rknn_run", ret);
      }
      return false;
    }

    std::vector<rknn_output> raw_outputs(io_num_.n_output);
    for (uint32_t i = 0; i < io_num_.n_output; ++i)
    {
      memset(&raw_outputs[i], 0, sizeof(raw_outputs[i]));
      raw_outputs[i].want_float = 1;
      raw_outputs[i].is_prealloc = 0;
    }

    ret = rknn_outputs_get(ctx_, io_num_.n_output, raw_outputs.data(), nullptr);
    if (ret != RKNN_SUCC)
    {
      if (error)
      {
        *error = rknnError("rknn_outputs_get", ret);
      }
      return false;
    }

    outputs->clear();
    outputs->reserve(raw_outputs.size());
    for (std::size_t i = 0; i < raw_outputs.size(); ++i)
    {
      NpuTensor tensor;
      for (uint32_t d = 0; d < output_attrs_[i].n_dims; ++d)
      {
        tensor.shape.push_back(output_attrs_[i].dims[d]);
      }
      const std::size_t element_count = raw_outputs[i].size / sizeof(float);
      const float* values = static_cast<const float*>(raw_outputs[i].buf);
      tensor.data.assign(values, values + element_count);
      outputs->push_back(std::move(tensor));
    }

    ret = rknn_outputs_release(ctx_, io_num_.n_output, raw_outputs.data());
    if (ret != RKNN_SUCC)
    {
      if (error)
      {
        *error = rknnError("rknn_outputs_release", ret);
      }
      return false;
    }

    return true;
  }

private:
  rknn_context ctx_ = 0;
  rknn_input_output_num io_num_;
  NpuInputSpec input_spec_;
  std::vector<unsigned char> model_data_;
  std::vector<rknn_tensor_attr> output_attrs_;
};

#endif

}  // namespace

std::unique_ptr<NpuBackend> createRknnBackend(std::string* error)
{
  (void)error;
  return std::unique_ptr<NpuBackend>(new RknnBackend());
}

}  // namespace sunray_perception
