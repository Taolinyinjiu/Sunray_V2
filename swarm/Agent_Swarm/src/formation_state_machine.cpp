/*
本程序功能：
    1、实现 FormationStateMachine::effectiveState，根据 leader/ORCA 数据新鲜度裁决实际状态
    2、INIT/TAKEOFF/LAND/HOVER 直接返回请求状态
    3、ORCA_RETURN_HOME 在 ORCA 超时时降级为 HOVER
    4、FORMATION 在 leader 或 ORCA 超时时降级为 HOVER（安全兜底）
*/
#include "formation_state_machine.h"

namespace agent_swarm
{

FormationStateMachine::FormationStateMachine() = default;

SwarmState FormationStateMachine::effectiveState(bool leader_ok, bool orca_ok) const
{
    // INIT 状态：节点刚启动，等待外部 TAKEOFF 指令
    if (requested_state_ == SwarmState::INIT)
    {
        return SwarmState::INIT;
    }
    if (requested_state_ == SwarmState::TAKEOFF || requested_state_ == SwarmState::LAND ||
        requested_state_ == SwarmState::HOVER)
    {
        return requested_state_;
    }
    if (requested_state_ == SwarmState::ORCA_RETURN_HOME)
    {
        return orca_ok ? SwarmState::ORCA_RETURN_HOME : SwarmState::HOVER;
    }
    // 安全兜底：leader 或 ORCA 超时则悬停
    if (!leader_ok || !orca_ok)
    {
        return SwarmState::HOVER;
    }
    return SwarmState::FORMATION;
}

void FormationStateMachine::setRequestedState(SwarmState state)
{
    requested_state_ = state;
}

} // namespace agent_swarm
