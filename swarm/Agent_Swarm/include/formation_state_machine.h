// 中文说明：编队状态机接口，定义状态与状态切换
#pragma once

namespace agent_swarm
{

// 编队状态枚举
enum class SwarmState
{
    INIT = 0,
    TAKEOFF = 1,
    LAND = 2,
    HOVER = 3,
    FORMATION = 4,
    ORCA_RETURN_HOME = 5
};

// 编队状态机
class FormationStateMachine
{
  public:
    FormationStateMachine();
    // 综合状态（含安全兜底）
    SwarmState effectiveState(bool leader_ok, bool orca_ok) const;
    // 请求状态
    SwarmState requestedState() const
    {
        return requested_state_;
    }
    // 直接设置状态
    void setRequestedState(SwarmState state);

  private:
    SwarmState requested_state_{SwarmState::INIT};
};

} // namespace agent_swarm
