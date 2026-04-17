// 中文说明：编队策略工厂实现，映射策略名称到具体策略类
#include "formation_policy_factory.h"
#include "formation_policies.h"

namespace agent_swarm
{

std::shared_ptr<FormationPolicy> FormationPolicyFactory::create(const std::string &name) const
{
    if (name == "ring")
    {
        return std::make_shared<RingPolicy>();
    }
    if (name == "line")
    {
        return std::make_shared<LinePolicy>();
    }
    if (name == "column")
    {
        return std::make_shared<ColumnPolicy>();
    }
    if (name == "v_shape" || name == "v")
    {
        return std::make_shared<VFormationPolicy>();
    }
    if (name == "wedge")
    {
        return std::make_shared<WedgePolicy>();
    }
    if (name == "custom")
    {
        return std::make_shared<CustomPolicy>();
    }
    // 默认策略
    return std::make_shared<RingPolicy>();
}

} // namespace agent_swarm
