/**
 * @file px4_param_manager.h
 * @brief PX4 参数读写管理器。
 */

#pragma once

#include <ros/node_handle.h>
#include <ros/service_client.h>

#include <type_traits>

class PX4_ParamManager {
public:
  /** @brief 构造函数。 */
  explicit PX4_ParamManager(ros::NodeHandle nh);
  ~PX4_ParamManager() = default;
  // 写入PX4参数
  // 使用模板的方式实现对不同结构体的兼容，使用方式为
  // px4_param_types::EKF2_EV_CTRL ctrl;
  // manager.set_param(ctrl);
  template <typename T> bool set_param(const T &param) {
    using raw_type = typename T::raw_type;
    static_assert(std::is_same_v<decltype(param.encode()), raw_type>,
                  "encode() return type must match raw_type");

    return set_param_raw(T::param_name, param.encode());
  }
  // 读取PX4参数
  // 使用模板的方式实现对不同结构体的兼容，使用方式为
  // px4_param_decode::EKF2_EV_CTRL ctrl;
  // manager.read_param(&ctrl);
  template <typename T> bool read_param(T *param) {
    // 读保护,如果传入空指针返回false
    if (param == nullptr) {
      return false;
    }

    typename T::raw_type raw{};
    // 读取参数原始值
    if (!read_param_raw(T::param_name, &raw)) {
      return false;
    }
    // 解码
    param->decode(raw);
    return true;
  }

private:
  // 通过调用mavros提供的服务,向px4飞控写入参数
  bool set_param_raw(const char *name, int value);
  bool set_param_raw(const char *name, double value);
  // 通过调用mavros提供的服务,读取px4飞控指定的参数
  bool read_param_raw(const char *name, int *value);
  bool read_param_raw(const char *name, double *value);
	// 维护两个ros服务端
  ros::ServiceClient param_set_client_;
  ros::ServiceClient param_get_client_;
  bool initialized_ = false;
};
