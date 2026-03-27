#pragma once
#include "controller/base_controller.hpp"

class PX4_OriginController : public uav_control::BaseController {
  public:
    explicit PX4_OriginController(ros::NodeHandle nh) : uav_control::BaseController(nh) {}
};
