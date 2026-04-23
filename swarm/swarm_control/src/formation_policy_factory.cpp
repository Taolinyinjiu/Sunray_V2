/*
本程序功能：
    1、实现 FormationPolicyFactory::create，将策略名称映射到具体策略类实例
    2、支持 ring/line/column/v_shape(v)/wedge/custom 六种名称
    3、未知名称默认返回 RingPolicy
*/
#include "formation_policy_factory.h"
#include "formation_policies.h"

namespace swarm_control
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

} // namespace swarm_control
