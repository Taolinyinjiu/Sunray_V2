/*
本程序功能：
    1、定义 SwarmState 枚举：INIT、TAKEOFF、LAND、HOVER、FORMATION、ORCA_RETURN_HOME
    2、定义 FormationStateMachine 类，管理集群状态请求与安全兜底逻辑
    3、effectiveState() 根据 leader 和 ORCA 数据新鲜度裁决实际生效状态
    4、leader 或 ORCA 超时时自动降级到 HOVER 状态，防止失控
*/
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
