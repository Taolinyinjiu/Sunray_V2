#include <sunray_perception/detection/npu_backend.h>

namespace sunray_perception
{

std::unique_ptr<NpuBackend> createRknnBackend(std::string* error);
std::unique_ptr<NpuBackend> createTensorRtBackend(std::string* error);

std::unique_ptr<NpuBackend> createNpuBackend(const std::string& backend_type, std::string* error)
{
  if (backend_type == "rk3588")
  {
    return createRknnBackend(error);
  }
  if (backend_type == "orin_nx")
  {
    return createTensorRtBackend(error);
  }

  if (error)
  {
    *error = "backend_type must be 'rk3588' or 'orin_nx'";
  }
  return nullptr;
}

}  // namespace sunray_perception
