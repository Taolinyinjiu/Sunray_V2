#include <sunray_perception/detection/npu_backend.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <numeric>
#include <sstream>

#ifdef SUNRAY_WITH_TENSORRT
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#endif

namespace sunray_perception
{
namespace
{

#ifndef SUNRAY_WITH_TENSORRT

class TensorRtBackend final : public NpuBackend
{
public:
  bool load(const std::string& model_path, const NpuInputSpec& input_spec, std::string* error) override
  {
    input_spec_ = input_spec;
    (void)model_path;
    if (error)
    {
      *error = "TensorRT backend was requested but this package was built without WITH_TENSORRT=ON";
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
      *error = "TensorRT backend is not available in this build";
    }
    return false;
  }

private:
  NpuInputSpec input_spec_;
};

#else

class TrtLogger final : public nvinfer1::ILogger
{
public:
  void log(Severity severity, const char* msg) noexcept override
  {
    if (severity <= Severity::kWARNING)
    {
      std::cerr << "[TensorRT] " << msg << std::endl;
    }
  }
};

struct DeviceBuffer
{
  void* ptr = nullptr;
  std::size_t bytes = 0;
};

bool readBinaryFile(const std::string& path, std::vector<char>* data, std::string* error)
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
  file.read(data->data(), size);
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

std::string cudaError(const std::string& action, cudaError_t ret)
{
  return action + " failed: " + cudaGetErrorString(ret);
}

std::size_t elementCount(const nvinfer1::Dims& dims)
{
  std::size_t count = 1;
  for (int i = 0; i < dims.nbDims; ++i)
  {
    count *= static_cast<std::size_t>(dims.d[i]);
  }
  return count;
}

std::size_t bytesPerElement(nvinfer1::DataType type)
{
  switch (type)
  {
    case nvinfer1::DataType::kFLOAT:
      return 4;
    case nvinfer1::DataType::kHALF:
      return 2;
    case nvinfer1::DataType::kINT8:
      return 1;
    case nvinfer1::DataType::kINT32:
      return 4;
    case nvinfer1::DataType::kBOOL:
      return 1;
  }
  return 0;
}

bool dataTypeToInputType(nvinfer1::DataType type, NpuTensorType* input_type, std::string* error)
{
  switch (type)
  {
    case nvinfer1::DataType::kFLOAT:
      *input_type = NpuTensorType::FLOAT32;
      return true;
    case nvinfer1::DataType::kHALF:
      *input_type = NpuTensorType::FLOAT16;
      return true;
    case nvinfer1::DataType::kINT8:
      *input_type = NpuTensorType::UINT8;
      return true;
    case nvinfer1::DataType::kINT32:
    case nvinfer1::DataType::kBOOL:
      break;
  }
  if (error)
  {
    *error = "unsupported TensorRT input binding data type";
  }
  return false;
}

std::vector<int> shapeFromDims(const nvinfer1::Dims& dims)
{
  std::vector<int> shape;
  shape.reserve(dims.nbDims);
  for (int i = 0; i < dims.nbDims; ++i)
  {
    shape.push_back(dims.d[i]);
  }
  return shape;
}

class TensorRtBackend final : public NpuBackend
{
public:
  ~TensorRtBackend() override
  {
    for (DeviceBuffer& buffer : buffers_)
    {
      if (buffer.ptr)
      {
        cudaFree(buffer.ptr);
      }
    }
    if (stream_)
    {
      cudaStreamDestroy(stream_);
    }
  }

  bool load(const std::string& model_path, const NpuInputSpec& input_spec, std::string* error) override
  {
    input_spec_ = input_spec;
    std::vector<char> engine_data;
    if (!readBinaryFile(model_path, &engine_data, error))
    {
      return false;
    }

    runtime_.reset(nvinfer1::createInferRuntime(logger_));
    if (!runtime_)
    {
      if (error)
      {
        *error = "createInferRuntime failed";
      }
      return false;
    }

    engine_.reset(runtime_->deserializeCudaEngine(engine_data.data(), engine_data.size()));
    if (!engine_)
    {
      if (error)
      {
        *error = "deserializeCudaEngine failed";
      }
      return false;
    }

    context_.reset(engine_->createExecutionContext());
    if (!context_)
    {
      if (error)
      {
        *error = "createExecutionContext failed";
      }
      return false;
    }

    const int nb_bindings = engine_->getNbBindings();
    if (nb_bindings < 2)
    {
      if (error)
      {
        *error = "TensorRT engine must have at least one input and one output binding";
      }
      return false;
    }

    input_binding_ = -1;
    output_bindings_.clear();
    for (int i = 0; i < nb_bindings; ++i)
    {
      if (engine_->bindingIsInput(i))
      {
        input_binding_ = i;
      }
      else
      {
        output_bindings_.push_back(i);
      }
    }

    if (input_binding_ < 0 || output_bindings_.empty())
    {
      if (error)
      {
        *error = "TensorRT engine bindings must include one input and at least one output";
      }
      return false;
    }
    if (!dataTypeToInputType(engine_->getBindingDataType(input_binding_), &input_spec_.type, error))
    {
      return false;
    }

    nvinfer1::Dims input_dims = engine_->getBindingDimensions(input_binding_);
    if (input_dims.nbDims == 4 && input_dims.d[0] < 0)
    {
      input_dims.d[0] = 1;
    }
    for (int i = 0; i < input_dims.nbDims; ++i)
    {
      if (input_dims.d[i] < 0)
      {
        if (i == input_dims.nbDims - 1)
        {
          input_dims.d[i] = input_spec.width;
        }
        else if (i == input_dims.nbDims - 2)
        {
          input_dims.d[i] = input_spec.height;
        }
        else if (i == input_dims.nbDims - 3)
        {
          input_dims.d[i] = input_spec.channels;
        }
        else
        {
          input_dims.d[i] = 1;
        }
      }
    }

    if (!context_->setBindingDimensions(input_binding_, input_dims))
    {
      if (error)
      {
        *error = "setBindingDimensions failed for TensorRT input";
      }
      return false;
    }

    buffers_.assign(nb_bindings, DeviceBuffer());
    host_output_shapes_.assign(nb_bindings, std::vector<int>());
    host_output_data_.assign(nb_bindings, std::vector<float>());

    for (int i = 0; i < nb_bindings; ++i)
    {
      const nvinfer1::Dims dims = context_->getBindingDimensions(i);
      const std::size_t bytes = elementCount(dims) * bytesPerElement(engine_->getBindingDataType(i));
      cudaError_t ret = cudaMalloc(&buffers_[i].ptr, bytes);
      if (ret != cudaSuccess)
      {
        if (error)
        {
          *error = cudaError("cudaMalloc", ret);
        }
        return false;
      }
      buffers_[i].bytes = bytes;
      if (!engine_->bindingIsInput(i))
      {
        host_output_shapes_[i] = shapeFromDims(dims);
        host_output_data_[i].resize(elementCount(dims));
      }
    }

    const cudaError_t ret = cudaStreamCreate(&stream_);
    if (ret != cudaSuccess)
    {
      if (error)
      {
        *error = cudaError("cudaStreamCreate", ret);
      }
      return false;
    }

    return true;
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
    if (input_bytes != buffers_[input_binding_].bytes)
    {
      if (error)
      {
        std::ostringstream oss;
        oss << "TensorRT input size mismatch: got " << input_bytes
            << " bytes, expected " << buffers_[input_binding_].bytes;
        *error = oss.str();
      }
      return false;
    }

    cudaError_t ret = cudaMemcpyAsync(buffers_[input_binding_].ptr, input_data, input_bytes,
                                      cudaMemcpyHostToDevice, stream_);
    if (ret != cudaSuccess)
    {
      if (error)
      {
        *error = cudaError("cudaMemcpyAsync input", ret);
      }
      return false;
    }

    std::vector<void*> binding_ptrs;
    binding_ptrs.reserve(buffers_.size());
    for (const DeviceBuffer& buffer : buffers_)
    {
      binding_ptrs.push_back(buffer.ptr);
    }

    if (!context_->enqueueV2(binding_ptrs.data(), stream_, nullptr))
    {
      if (error)
      {
        *error = "TensorRT enqueueV2 failed";
      }
      return false;
    }

    for (int binding : output_bindings_)
    {
      ret = cudaMemcpyAsync(host_output_data_[binding].data(), buffers_[binding].ptr, buffers_[binding].bytes,
                            cudaMemcpyDeviceToHost, stream_);
      if (ret != cudaSuccess)
      {
        if (error)
        {
          *error = cudaError("cudaMemcpyAsync output", ret);
        }
        return false;
      }
    }

    ret = cudaStreamSynchronize(stream_);
    if (ret != cudaSuccess)
    {
      if (error)
      {
        *error = cudaError("cudaStreamSynchronize", ret);
      }
      return false;
    }

    outputs->clear();
    outputs->reserve(output_bindings_.size());
    for (int binding : output_bindings_)
    {
      NpuTensor tensor;
      tensor.shape = host_output_shapes_[binding];
      tensor.data = host_output_data_[binding];
      outputs->push_back(std::move(tensor));
    }
    return true;
  }

  NpuInputSpec inputSpec() const override
  {
    return input_spec_;
  }

private:
  struct InferDeleter
  {
    template <typename T>
    void operator()(T* obj) const
    {
      if (obj)
      {
        obj->destroy();
      }
    }
  };

  TrtLogger logger_;
  NpuInputSpec input_spec_;
  std::unique_ptr<nvinfer1::IRuntime, InferDeleter> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine, InferDeleter> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext, InferDeleter> context_;
  int input_binding_ = -1;
  std::vector<int> output_bindings_;
  std::vector<DeviceBuffer> buffers_;
  std::vector<std::vector<int>> host_output_shapes_;
  std::vector<std::vector<float>> host_output_data_;
  cudaStream_t stream_ = nullptr;
};

#endif

}  // namespace

std::unique_ptr<NpuBackend> createTensorRtBackend(std::string* error)
{
  (void)error;
  return std::unique_ptr<NpuBackend>(new TensorRtBackend());
}

}  // namespace sunray_perception
