#include "odom_filter/odom_kalman_filter.hpp"

#include <algorithm>
#include <cmath>

namespace control_common {
namespace {

// 本文件中的滤波变量约定：
// - x_：滤波器内部状态，顺序固定为 [px, py, pz, vx, vy, vz]^T。
// - P_：x_ 的状态协方差，表示当前状态估计的不确定度。
// - F/Q：预测阶段使用的状态转移矩阵和过程噪声协方差。
// - H/R：更新阶段使用的观测矩阵和测量噪声协方差。

// 协方差对角线最小值，避免数值误差把方差压到 0 或负数。
constexpr double kMinVariance = 1e-9;
// 最小有效采样间隔。重复时间戳或过小 dt 不做预测/更新，改为重新初始化。
constexpr double kMinDt = 1e-4;
// 最大有效采样间隔。超过该值视为断流、暂停或时间跳变后恢复，改为重新初始化。
constexpr double kMaxDt = 0.2;

}  // namespace

void OdomKalmanFilter::init(const OdomKalmanFilterParam_t& params) {
    // params_ 保存本滤波器运行所需的噪声参数：
    // process_noise_acc 控制预测模型允许的加速度扰动；
    // meas_noise_pos/meas_noise_vel 控制测量更新时对位置/速度观测的信任程度。
    // 参数合法性由配置加载层保证，这里不再做二次裁剪。
    params_ = params;
    reset();
}

void OdomKalmanFilter::reset() {
    // initialized_ 表示 x_/P_ 是否已经由有效里程计样本初始化。
    initialized_ = false;
    // x_ 是 6 维状态 [px, py, pz, vx, vy, vz]^T。
    x_.setZero();
    // P_ 是状态协方差；reset 后给单位阵，真正启用前会在 initialize() 中重设。
    P_.setIdentity();
    // last_timestamp_ 记录上一帧用于滤波的时间戳；
    // has_last_timestamp_ 表示 last_timestamp_ 是否已经可用于计算 dt。
    last_timestamp_ = ros::Time(0);
    has_last_timestamp_ = false;
}

UAVStateEstimate OdomKalmanFilter::update(const UAVStateEstimate& raw_odom) {
    // raw_odom 是本次输入的原始里程计样本。
    // 本函数只改写输出中的 position/velocity，timestamp/orientation/bodyrate 由 compose_output() 透传。

    // dt 只来自 UAVStateEstimate.timestamp。调用方若不信任消息 header stamp，
    // 应在构造 UAVStateEstimate 时写入接收时刻。
    // dt 是当前样本与上一有效时间戳之间的时间差，单位秒。
    double dt = 0.0;
    if (has_last_timestamp_) {
        dt = (raw_odom.timestamp - last_timestamp_).toSec();
    }
    // 先缓存本帧时间戳，用于下一次 update() 计算 dt。
    // has_last_timestamp_ 为 false 时，下一帧不会使用当前时间戳做预测间隔。
    last_timestamp_ = raw_odom.timestamp;
    has_last_timestamp_ = !raw_odom.timestamp.isZero();

    // 以下情况不使用上一状态外推：
    // - 首帧：还没有可预测的 x_/P_；
    // - 时间戳无效或 dt 非有限：时间轴不可用；
    // - dt 过小：重复时间戳或采样间隔小到预测增量没有意义；
    // - dt 过大：匀速模型跨越太长时间不可靠，按断流恢复处理。
    if (!initialized_ || raw_odom.timestamp.isZero() ||
        !std::isfinite(dt) || dt < kMinDt || dt > kMaxDt) {
        if (!initialize(raw_odom)) {
            reset();
            return raw_odom;
        }
        return compose_output(raw_odom);
    }
    // predict() 使用上一帧滤波状态和 dt 得到当前时刻的先验状态。
    predict(dt);
    // update_position_velocity() 使用当前里程计位置/速度测量修正先验状态。
    update_position_velocity(raw_odom.position, raw_odom.velocity);
    // 更新后检查 x_/P_ 是否仍然有限，并保护协方差对角线下限。
    enforce_numeric_safety(raw_odom);
    // 组装输出：滤波 position/velocity，orientation/bodyrate 透传。
    return compose_output(raw_odom);
}

bool OdomKalmanFilter::is_initialized() const {
    return initialized_;
}

const Eigen::Matrix<double, 6, 1>& OdomKalmanFilter::get_state() const {
    return x_;
}

const Eigen::Matrix<double, 6, 6>& OdomKalmanFilter::get_covariance() const {
    return P_;
}

bool OdomKalmanFilter::initialize(const UAVStateEstimate& raw_odom) {
    // v6 假设上游保证数据可信；这里仅做有限值防御，避免 NaN/Inf 污染状态。
    if (!raw_odom.position.allFinite() || !raw_odom.velocity.allFinite()) {
        return false;
    }

    // 初始状态直接取当前测量：位置写入 x_ 前 3 维，速度写入 x_ 后 3 维。
    x_.head<3>() = raw_odom.position;
    x_.tail<3>() = raw_odom.velocity;

    // P_ 表示初始状态的不确定度。
    // 因为首帧状态完全来自当前测量，所以位置/速度子块直接使用对应测量方差。
    P_.setZero();
    P_.topLeftCorner<3, 3>().diagonal().setConstant(
        params_.meas_noise_pos * params_.meas_noise_pos);
    P_.bottomRightCorner<3, 3>().diagonal().setConstant(
        params_.meas_noise_vel * params_.meas_noise_vel);

    // 到这里 x_/P_ 均已可用于后续 predict/update。
    initialized_ = true;
    return true;
}

void OdomKalmanFilter::predict(double dt) {
    // dt 是本次预测的时间步长，单位秒；调用者已保证它在有效范围内。

    // 匀速模型：
    // p(k+1) = p(k) + v(k) * dt
    // v(k+1) = v(k)
    // F 是状态转移矩阵。右上角 dt*I3 表示位置由速度积分得到。
    Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
    F.topRightCorner<3, 3>() = dt * Eigen::Matrix3d::Identity();

    // dt2/dt3 用于连续白噪声加速度模型的离散化 Q 矩阵。
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    // q 是连续白噪声加速度强度，对应参数 process_noise_acc。
    // q 越大，Q 越大，预测不确定度增长越快，测量更新时会更愿意跟随新观测。
    const double q = params_.process_noise_acc;

    // 连续白噪声加速度模型离散化后的 Q。
    // 左上：位置方差增量；右下：速度方差增量；非对角块：位置-速度相关项。
    Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
    Q.topLeftCorner<3, 3>() = (q * dt3 / 3.0) * Eigen::Matrix3d::Identity();
    Q.topRightCorner<3, 3>() = (q * dt2 / 2.0) * Eigen::Matrix3d::Identity();
    Q.bottomLeftCorner<3, 3>() = (q * dt2 / 2.0) * Eigen::Matrix3d::Identity();
    Q.bottomRightCorner<3, 3>() = (q * dt) * Eigen::Matrix3d::Identity();

    // x_ 预测：把上一状态推进到当前时间。
    x_ = F * x_;
    // P_ 预测：上一协方差经 F 传播，并叠加过程噪声 Q。
    P_ = F * P_ * F.transpose() + Q;
    // 消除浮点误差导致的轻微非对称。
    P_ = 0.5 * (P_ + P_.transpose());
}

bool OdomKalmanFilter::update_position_velocity(const Eigen::Vector3d& pos_meas,
                                                const Eigen::Vector3d& vel_meas) {
    // pos_meas/vel_meas 是本帧位置和速度观测，分别对应状态 x_ 的前 3 维和后 3 维。

    if (!pos_meas.allFinite() || !vel_meas.allFinite()) {
        return false;
    }

    // 固定 6 维观测：z = [p, v]，H = I6。
    // H 把状态 x_ 映射到观测空间；这里观测量与状态量一一对应。
    Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Identity();
    // z 是当前里程计测量向量，前 3 维位置，后 3 维速度。
    Eigen::Matrix<double, 6, 1> z;
    z.head<3>() = pos_meas;
    z.tail<3>() = vel_meas;

    // R 是测量噪声协方差。位置和速度各自使用各向同性方差。
    // R 越大表示越不信任对应观测，Kalman 增益会相应降低。
    Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Zero();
    R.topLeftCorner<3, 3>() =
        (params_.meas_noise_pos * params_.meas_noise_pos) * Eigen::Matrix3d::Identity();
    R.bottomRightCorner<3, 3>() =
        (params_.meas_noise_vel * params_.meas_noise_vel) * Eigen::Matrix3d::Identity();

    // innovation 是测量残差，表示测量值与当前预测状态之间的差。
    const Eigen::Matrix<double, 6, 1> innovation = z - H * x_;
    // S 是 innovation 协方差，用于计算 Kalman 增益。
    const Eigen::Matrix<double, 6, 6> S = H * P_ * H.transpose() + R;
    // PHt 是 P * H^T，单独保存便于构造 K，避免重复计算。
    const Eigen::Matrix<double, 6, 6> PHt = P_ * H.transpose();

    // ldlt 是 S 的矩阵分解器，用来解 K = P H^T S^-1。
    // 这样避免显式计算 S.inverse()，数值稳定性更好。
    Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt(S);
    if (ldlt.info() != Eigen::Success) {
        return false;
    }

    // K 是 Kalman 增益，决定本次测量残差有多少写回状态。
    const Eigen::Matrix<double, 6, 6> K = ldlt.solve(PHt.transpose()).transpose();
    if (!K.allFinite()) {
        return false;
    }

    // 用 Kalman 增益修正预测状态。
    x_ = x_ + K * innovation;

    // Joseph 形式比 P=(I-KH)P 更抗浮点误差，能更好保持半正定。
    // I 是 6 维单位阵，用于表达 Joseph 形式中的 (I - K H)。
    const Eigen::Matrix<double, 6, 6> I = Eigen::Matrix<double, 6, 6>::Identity();
    // KH 是 K*H，Joseph 形式更新协方差时复用。
    const Eigen::Matrix<double, 6, 6> KH = K * H;
    // P_ 更新后表示融合本帧测量后的后验状态协方差。
    P_ = (I - KH) * P_ * (I - KH).transpose() + K * R * K.transpose();
    P_ = 0.5 * (P_ + P_.transpose());

    for (int i = 0; i < 6; ++i) {
        // 防止数值误差把任一状态方差压成 0 或负数。
        P_(i, i) = std::max(P_(i, i), kMinVariance);
    }
    return true;
}

void OdomKalmanFilter::enforce_numeric_safety(const UAVStateEstimate& raw_odom) {
    // raw_odom 在这里作为故障恢复样本：若当前滤波状态已被 NaN/Inf 污染，
    // 尝试直接用本帧测量重新建立 x_/P_。

    // 一旦状态或协方差出现 NaN/Inf，尝试用当前测量重新初始化；
    // 如果当前测量也不可用，则 reset 后等待下一帧有效测量。
    if (!x_.allFinite() || !P_.allFinite()) {
        if (!initialize(raw_odom)) {
            reset();
        }
        return;
    }

    for (int i = 0; i < 6; ++i) {
        // i 遍历 6 个状态维度，逐项保护对应状态方差的最小正值。
        P_(i, i) = std::max(P_(i, i), kMinVariance);
    }
}

UAVStateEstimate OdomKalmanFilter::compose_output(const UAVStateEstimate& raw_odom) const {
    // 先复制原始里程计，保留 timestamp/orientation/bodyrate 等非滤波字段。
    UAVStateEstimate output = raw_odom;
    if (initialized_) {
        // output.position/output.velocity 是滤波后的控制输入；
        // 姿态和角速度不属于本滤波器状态，保持 raw_odom 原值。
        output.position = x_.head<3>();
        output.velocity = x_.tail<3>();
    }
    return output;
}

}  // namespace control_common
