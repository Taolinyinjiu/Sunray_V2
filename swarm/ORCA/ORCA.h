/*
本文件功能：
    1、定义纯 C++ 的 ORCA 封装类，隔离上层业务与原生 RVO2 内核
    2、通过 API 提供初始化、设置 agent 状态、设置目标点、计算期望速度与读取 ORCA 输出速度
    3、不依赖 ROS，公开接口只使用基础数值类型，便于在不同模块中复用
*/
#pragma once

#include <memory>
#include <vector>

namespace RVO
{
class RVOSimulator;
}

namespace orca_swarm
{

/**
 * @brief 面向集群控制上层逻辑的 ORCA 算法封装类。
 *
 * 这个类不直接和 ROS 话题交互，只负责两件事：
 * 1. 管理每个 agent 的当前位置、当前速度和目标点；
 * 2. 调用底层 RVO2/ORCA 求解器，输出避碰修正后的速度指令。
 *
 * 典型使用流程：
 * 1. 调用 init(...) 初始化仿真器参数和 agent 数量；
 * 2. 每个控制周期通过 setAgentState(...) 更新所有 agent 的当前位置和速度；
 * 3. 通过 setAgentGoal(...) 设置目标点；
 * 4. 调用 GetOrcaVelCmd(...) 执行一次 ORCA 计算，并读取指定 agent 的输出速度。
 */
class ORCA
{
  public:
    ORCA();
    ~ORCA();

    /**
     * @brief 初始化 ORCA 求解器。
     *
     * @param agent_num 集群中 agent 的数量。
     * @param neighbor_dist 邻居搜索距离。某个 agent 只会考虑这个范围内的其他 agent 参与避碰。
     *        值越大，避碰更保守，但计算量也会更高。
     * @param time_horizon 预测时间窗口。ORCA 会假设在未来这段时间内可能发生碰撞并提前规避。
     *        值越大，避碰动作越早、越平滑，但也可能更保守。
     * @param radius agent 的等效碰撞半径。可以理解为“安全圆”的半径。
     * @param max_speed agent 允许的最大速度上限。
     * @param time_step ORCA 内部离散仿真的时间步长，通常与外部控制周期接近。
     * @param max_neighbors 每个 agent 最多纳入多少个邻居参与计算。
     *        小于等于 0 时，默认按 agent 总数自动设置。
     */
    void init(int agent_num,
              float neighbor_dist,
              float time_horizon,
              float radius,
              float max_speed,
              float time_step,
              int max_neighbors = -1);

    /**
     * @brief 更新指定 agent 的当前位置和当前速度。
     *
     * @param idx agent 索引，范围为 [0, agent_num-1]。
     * @param x 当前 x 坐标。
     * @param y 当前 y 坐标。
     * @param vx 当前 x 方向速度。
     * @param vy 当前 y 方向速度。
     * @return 索引合法时返回 true，否则返回 false。
     */
    bool setAgentState(int idx, double x, double y, double vx, double vy);

    /**
     * @brief 设置指定 agent 的目标点。
     *
     * 这里的目标点只用于计算“期望速度”，ORCA 再基于该期望速度进行避碰修正。
     *
     * @param idx agent 索引。
     * @param x 目标点 x 坐标。
     * @param y 目标点 y 坐标。
     * @return 索引合法时返回 true，否则返回 false。
     */
    bool setAgentGoal(int idx, double x, double y);

    /**
     * @brief 清除指定 agent 的目标点。
     *
     * 清除后，该 agent 的期望速度会被视为 0，等价于“当前没有主动机动目标”。
     *
     * @param idx agent 索引。
     * @return 索引合法时返回 true，否则返回 false。
     */
    bool clearAgentGoal(int idx);

    /**
     * @brief 添加一个圆形静态障碍物。
     *
     * 原生 RVO2 只支持线段/多边形障碍物，因此这里会把圆离散成凸多边形。
     * 建议在 init(...) 后、正式运行 GetOrcaVelCmd(...) 前设置障碍物。
     *
     * @param x 圆心 x 坐标。
     * @param y 圆心 y 坐标。
     * @param radius 圆半径，单位：米。
     * @param vertex_num 圆形近似多边形顶点数，越大越接近圆，但计算量也越高。
     * @return 障碍物成功加入并完成处理时返回 true，否则返回 false。
     */
    bool addCircleObstacle(double x, double y, double radius, int vertex_num = 24);

    /**
     * @brief 根据当前位置和目标点，计算某个 agent 的原始期望速度。
     *
     * 这个速度还没有经过避碰修正，只表示“如果周围没有其他 agent，
     * 该 agent 想朝哪个方向、以多大速度运动”。
     *
     * @param idx agent 索引。
     * @param pref_vx 输出的 x 方向期望速度。
     * @param pref_vy 输出的 y 方向期望速度。
     * @return 索引合法时返回 true，否则返回 false。
     */
    bool computePreferredVelocity(int idx, double &pref_vx, double &pref_vy) const;

    /**
     * @brief 执行一次 ORCA 计算，并读取指定 agent 的速度指令。
     *
     * 这个统一接口会：
     * 1. 把上层写入的状态同步到底层 RVO2 仿真器；
     * 2. 根据目标点计算每个 agent 的期望速度；
     * 3. 调用 ORCA 求解避碰后的速度指令；
     * 4. 返回指定 agent 的 ORCA 修正后速度。
     *
     * @param idx 需要读取输出速度的 agent 索引。
     * @param cmd_vx 输出的 x 方向速度指令。
     * @param cmd_vy 输出的 y 方向速度指令。
     * @return 初始化有效且索引合法时返回 true，否则返回 false。
     */
    bool GetOrcaVelCmd(int idx, double &cmd_vx, double &cmd_vy);

    /**
     * @brief 返回当前 ORCA 实例维护的 agent 数量。
     */
    int agent_num() const
    {
        return agent_num_;
    }

  private:
    /**
     * @brief 单个 agent 的缓存状态。
     *
     * 这些数据完全是上层控制逻辑可理解的物理量，不暴露底层 RVO2 的内部类型。
     */
    struct AgentData
    {
        double x{0.0};          // 当前 x 坐标
        double y{0.0};          // 当前 y 坐标
        double vx{0.0};         // 当前 x 方向速度
        double vy{0.0};         // 当前 y 方向速度
        double goal_x{0.0};     // 目标点 x 坐标
        double goal_y{0.0};     // 目标点 y 坐标
        bool goal_enabled{false}; // 当前是否存在有效目标点
        double pref_vx{0.0};    // 目标驱动下的原始期望速度 x 分量
        double pref_vy{0.0};    // 目标驱动下的原始期望速度 y 分量
        double cmd_vx{0.0};     // ORCA 修正后的输出速度 x 分量
        double cmd_vy{0.0};     // ORCA 修正后的输出速度 y 分量
        double deadlock_time{0.0}; // 有目标但 ORCA 输出速度长期过低的持续时间
    };

    // 判断 agent 索引是否在有效范围内。
    bool is_valid_index(int idx) const;

    int agent_num_{0};           // 当前管理的 agent 数量
    int max_neighbors_{0};       // 每个 agent 参与 ORCA 计算的最大邻居数量
    float neighbor_dist_{1.5f};  // 邻居搜索半径
    float time_horizon_{2.0f};   // 未来碰撞预测时间窗口
    float radius_{0.3f};         // agent 等效碰撞半径
    float max_speed_{0.5f};      // agent 最大速度上限
    float time_step_{0.1f};      // ORCA 内部离散时间步长

    std::vector<AgentData> agents_{};      // 上层可理解的 agent 状态缓存
    std::unique_ptr<RVO::RVOSimulator> sim_{}; // 在 init() 中直接创建的底层原生 ORCA/RVO2 仿真器
};

} // namespace orca_swarm
