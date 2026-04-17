// 中文说明：编队策略工厂接口，按名称选择策略
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
