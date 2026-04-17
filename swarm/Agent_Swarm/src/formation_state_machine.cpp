// 中文说明：编队状态机实现，处理指令与安全兜底
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
