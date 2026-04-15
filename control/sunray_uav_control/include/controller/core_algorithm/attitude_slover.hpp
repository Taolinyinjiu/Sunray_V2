/**
 * @brief 姿态控制模块，定义一个基类和两个子类，分别使用四元数和SO3表示姿态
 * 值得注意的是，本模块只在attitude -> bodyrate中使用，也就是说当控制器需要输出bodyrate时，调用本模块
 * 注意到两个姿态求解器，推力计算方式一致，并且计算得到的推力仍旧需要再做一次变换，因此拆出来单独实现
 */

#pragma once
#include <Eigen/Dense>

// ─────────────────────────────────────────────────────────────────────────
// 抽象基类：定义姿态子控制器的统一接口
// ─────────────────────────────────────────────────────────────────────────
class Attitude_Slover {
  public:
    virtual ~Attitude_Slover() = default;

    // 输入当前姿态、期望姿态、期望加速度，计算期望角速度和推力
    virtual void update(const Eigen::Quaterniond& curr_att,
                        const Eigen::Quaterniond& ref_att,
                        const Eigen::Vector3d& ref_acc) = 0;

    Eigen::Vector3d get_desired_rate() const {
        return desired_rate_;
    }

  protected:
    Eigen::Vector3d desired_rate_{Eigen::Vector3d::Zero()};  // 输出：期望机体角速度
};

// ─────────────────────────────────────────────────────────────────────────
// 四元数误差姿态控制（推荐首选，计算开销小，常规飞行足够精确）
// 参考：Brescianini et al., "Nonlinear quadrocopter attitude control", 2013
// ─────────────────────────────────────────────────────────────────────────
class Quaternion_Solver : public Attitude_Slover {
  public:
    explicit Quaternion_Solver(double attctrl_tau);
    void update(const Eigen::Quaterniond& curr_att,
                const Eigen::Quaterniond& ref_att,
                const Eigen::Vector3d& ref_acc) override;

  private:
    double attctrl_tau_{0.1};
};

// ─────────────────────────────────────────────────────────────────────────
// SO3 几何姿态控制（适合大角度特技飞行和高机动轨迹跟踪）
// 参考：Lee et al., "Geometric tracking control of a quadrotor UAV on SE(3)", 2010
// ─────────────────────────────────────────────────────────────────────────
class SO3_Solver : public Attitude_Slover {
  public:
    explicit SO3_Solver(double attctrl_tau);
    void update(const Eigen::Quaterniond& curr_att,
                const Eigen::Quaterniond& ref_att,
                const Eigen::Vector3d& ref_acc) override;

  private:
    double attctrl_tau_{0.1};
};
