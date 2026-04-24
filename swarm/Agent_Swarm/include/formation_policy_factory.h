/*
本程序功能：
    1、定义 FormationPolicyFactory 工厂类，根据策略名称字符串创建对应的 FormationPolicy 实例
    2、支持 ring/line/column/v_shape/wedge/custom 六种策略
    3、未知名称默认返回 RingPolicy
*/
#pragma once

#include "formation_policy.h"
#include <memory>
#include <string>

namespace agent_swarm
{

// 阵型策略工厂
class FormationPolicyFactory
{
  public:
    // 按名称创建策略
    std::shared_ptr<FormationPolicy> create(const std::string &name) const;
};

} // namespace agent_swarm
