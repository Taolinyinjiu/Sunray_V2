/*
本程序功能：
    1、Qt 无人车控制面板
    2、发布 UGVControlCMD
    3、订阅 UGVControlState 并显示无人车控制状态
*/
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sunray_msgs/UGVControlCMD.h>
#include <sunray_msgs/UGVControlState.h>

#include <QApplication>
#include <QColor>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMainWindow>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <cmath>
#include <mutex>
#include <string>

namespace
{
constexpr double kDegToRad = 0.017453292519943295;
constexpr double kRadToDeg = 57.29577951308232;

QDoubleSpinBox *makeSpin(const double value,
                         const double min_value,
                         const double max_value,
                         const double step,
                         const int decimals = 2)
{
    auto *spin = new QDoubleSpinBox();
    spin->setRange(min_value, max_value);
    spin->setDecimals(decimals);
    spin->setSingleStep(step);
    spin->setValue(value);
    return spin;
}

QString fsmName(const uint8_t state)
{
    switch (state)
    {
    case sunray_msgs::UGVControlState::FSM_INIT:
        return "INIT";
    case sunray_msgs::UGVControlState::FSM_HOLD:
        return "HOLD";
    case sunray_msgs::UGVControlState::FSM_RETURN:
        return "RETURN";
    case sunray_msgs::UGVControlState::FSM_MOVE:
        return "MOVE";
    default:
        return "UNKNOWN";
    }
}

QString cmdName(const uint8_t cmd)
{
    switch (cmd)
    {
    case sunray_msgs::UGVControlCMD::HOLD:
        return "HOLD";
    case sunray_msgs::UGVControlCMD::RETURN:
        return "RETURN";
    case sunray_msgs::UGVControlCMD::MOVE_POINT:
        return "MOVE_POINT";
    case sunray_msgs::UGVControlCMD::MOVE_VELOCITY:
        return "MOVE_VELOCITY";
    case sunray_msgs::UGVControlCMD::MOVE_VELOCITY_BODY:
        return "MOVE_VELOCITY_BODY";
    case sunray_msgs::UGVControlCMD::MOVE_WGS84:
        return "MOVE_WGS84";
    default:
        return "UNDEFINE";
    }
}

QString activeCmdText(const sunray_msgs::UGVControlCMD &cmd)
{
    switch (cmd.control_cmd)
    {
    case sunray_msgs::UGVControlCMD::MOVE_POINT:
        return QString("MOVE_POINT  x=%1  y=%2  yaw=%3 deg")
            .arg(cmd.desired_pos.x, 0, 'f', 2)
            .arg(cmd.desired_pos.y, 0, 'f', 2)
            .arg(cmd.desired_yaw * kRadToDeg, 0, 'f', 2);
    case sunray_msgs::UGVControlCMD::MOVE_VELOCITY:
        return QString("MOVE_VELOCITY  vx=%1  vy=%2  yaw=%3 deg")
            .arg(cmd.desired_vel.x, 0, 'f', 2)
            .arg(cmd.desired_vel.y, 0, 'f', 2)
            .arg(cmd.desired_yaw * kRadToDeg, 0, 'f', 2);
    case sunray_msgs::UGVControlCMD::MOVE_VELOCITY_BODY:
        return QString("MOVE_VELOCITY_BODY  vx=%1  vy=%2  wz=%3 deg/s")
            .arg(cmd.cmd_vel.linear.x, 0, 'f', 2)
            .arg(cmd.cmd_vel.linear.y, 0, 'f', 2)
            .arg(cmd.cmd_vel.angular.z * kRadToDeg, 0, 'f', 2);
    case sunray_msgs::UGVControlCMD::MOVE_WGS84:
        return QString("MOVE_WGS84  lat=%1  lon=%2  alt=%3")
            .arg(cmd.desired_wgs84_pos.latitude, 0, 'f', 7)
            .arg(cmd.desired_wgs84_pos.longitude, 0, 'f', 7)
            .arg(cmd.desired_wgs84_pos.altitude, 0, 'f', 2);
    default:
        return cmdName(cmd.control_cmd);
    }
}

double yawFromOdom(const nav_msgs::Odometry &odom)
{
    const auto &q = odom.pose.pose.orientation;
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

QString driveTypeName(const uint8_t drive_type)
{
    switch (drive_type)
    {
    case sunray_msgs::UGVControlState::DRIVE_MECANUM:
        return "mecanum";
    case sunray_msgs::UGVControlState::DRIVE_DIFFERENTIAL:
        return "differential";
    default:
        return "unknown";
    }
}

class UGVControlPanel : public QMainWindow
{
  public:
    explicit UGVControlPanel(ros::NodeHandle &nh, QWidget *parent = nullptr)
        : QMainWindow(parent), nh_(nh)
    {
        loadParams();
        setupRos();
        setupUi();
        applyStyle();

        refresh_timer_ = new QTimer(this);
        connect(refresh_timer_, &QTimer::timeout, this, [this]() { refreshState(); });
        refresh_timer_->start(200);

        setWindowTitle("Sunray UGV Control Panel");
        resize(980, 620);
    }

  private:
    void loadParams()
    {
        nh_.param("agent_name", agent_name_, std::string("ugv"));
        nh_.param("agent_id", agent_id_, 1);
        const std::string prefix = "/" + agent_name_ + std::to_string(agent_id_);
        nh_.param("cmd_topic", cmd_topic_, prefix + "/sunray/ugv_control/control_cmd");
        nh_.param("state_topic", state_topic_, prefix + "/sunray/ugv_control/control_state");
    }

    void setupRos()
    {
        cmd_pub_ = nh_.advertise<sunray_msgs::UGVControlCMD>(cmd_topic_, 10);
        state_sub_ = nh_.subscribe(state_topic_, 20, &UGVControlPanel::stateCallback, this);
    }

    void setupUi()
    {
        auto *central = new QWidget(this);
        auto *main_layout = new QVBoxLayout(central);
        main_layout->setContentsMargins(16, 16, 16, 16);
        main_layout->setSpacing(12);
        main_layout->addWidget(makeCommandGroup());
        main_layout->addWidget(makeStateGroup());
        main_layout->addWidget(makeLogGroup(), 1);
        setCentralWidget(central);
    }

    QWidget *makeCommandGroup()
    {
        auto *group = new QGroupBox("无人车控制");
        auto *layout = new QGridLayout(group);

        auto *hold_btn = new QPushButton("HOLD 停车");
        auto *return_btn = new QPushButton("RETURN 返航");
        connect(hold_btn, &QPushButton::clicked, this, [this]() { publishSimple(sunray_msgs::UGVControlCMD::HOLD); });
        connect(return_btn, &QPushButton::clicked, this, [this]() { publishSimple(sunray_msgs::UGVControlCMD::RETURN); });

        point_x_spin_ = makeSpin(0.0, -100.0, 100.0, 0.1);
        point_y_spin_ = makeSpin(0.0, -100.0, 100.0, 0.1);
        point_yaw_spin_ = makeSpin(0.0, -180.0, 180.0, 5.0);
        auto *point_btn = new QPushButton("发送 MOVE_POINT");
        connect(point_btn, &QPushButton::clicked, this, [this]() { publishMovePoint(); });

        body_vx_spin_ = makeSpin(0.0, -3.0, 3.0, 0.1);
        body_vy_spin_ = makeSpin(0.0, -3.0, 3.0, 0.1);
        body_wz_spin_ = makeSpin(0.0, -180.0, 180.0, 5.0);
        auto *body_vel_btn = new QPushButton("发送 MOVE_VELOCITY_BODY");
        connect(body_vel_btn, &QPushButton::clicked, this, [this]() { publishBodyVelocity(); });

        world_vx_spin_ = makeSpin(0.0, -3.0, 3.0, 0.1);
        world_vy_spin_ = makeSpin(0.0, -3.0, 3.0, 0.1);
        world_yaw_spin_ = makeSpin(0.0, -180.0, 180.0, 5.0);
        auto *world_vel_btn = new QPushButton("发送 MOVE_VELOCITY");
        connect(world_vel_btn, &QPushButton::clicked, this, [this]() { publishWorldVelocity(); });

        wgs84_lat_spin_ = makeSpin(0.0, -90.0, 90.0, 0.000001, 7);
        wgs84_lon_spin_ = makeSpin(0.0, -180.0, 180.0, 0.000001, 7);
        wgs84_alt_spin_ = makeSpin(0.0, -1000.0, 10000.0, 0.1);
        auto *wgs84_btn = new QPushButton("发送 MOVE_WGS84");
        connect(wgs84_btn, &QPushButton::clicked, this, [this]() { publishWgs84(); });

        layout->addWidget(new QLabel("快捷指令"), 0, 0);
        layout->addWidget(hold_btn, 0, 1);
        layout->addWidget(return_btn, 0, 2);

        layout->addWidget(new QLabel("目标点 x/y/yaw(deg)"), 1, 0);
        layout->addWidget(point_x_spin_, 1, 1);
        layout->addWidget(point_y_spin_, 1, 2);
        layout->addWidget(point_yaw_spin_, 1, 3);
        layout->addWidget(point_btn, 1, 4);

        layout->addWidget(new QLabel("车体系速度 vx/vy/wz(deg/s)"), 2, 0);
        layout->addWidget(body_vx_spin_, 2, 1);
        layout->addWidget(body_vy_spin_, 2, 2);
        layout->addWidget(body_wz_spin_, 2, 3);
        layout->addWidget(body_vel_btn, 2, 4);

        layout->addWidget(new QLabel("世界系速度 vx/vy/yaw(deg)"), 3, 0);
        layout->addWidget(world_vx_spin_, 3, 1);
        layout->addWidget(world_vy_spin_, 3, 2);
        layout->addWidget(world_yaw_spin_, 3, 3);
        layout->addWidget(world_vel_btn, 3, 4);

        layout->addWidget(new QLabel("WGS84 lat/lon/alt"), 4, 0);
        layout->addWidget(wgs84_lat_spin_, 4, 1);
        layout->addWidget(wgs84_lon_spin_, 4, 2);
        layout->addWidget(wgs84_alt_spin_, 4, 3);
        layout->addWidget(wgs84_btn, 4, 4);
        layout->setColumnStretch(5, 1);
        return group;
    }

    QWidget *makeStateGroup()
    {
        auto *group = new QGroupBox("运行状态");
        auto *layout = new QGridLayout(group);
        agent_label_ = new QLabel("-");
        input_label_ = new QLabel("-");
        pose_label_ = new QLabel("-");
        target_label_ = new QLabel("-");
        cmd_label_ = new QLabel("-");
        output_label_ = new QLabel("-");

        layout->addWidget(new QLabel("机器人"), 0, 0);
        layout->addWidget(agent_label_, 0, 1);
        layout->addWidget(new QLabel("输入状态"), 1, 0);
        layout->addWidget(input_label_, 1, 1);
        layout->addWidget(new QLabel("本机位姿"), 2, 0);
        layout->addWidget(pose_label_, 2, 1);
        layout->addWidget(new QLabel("控制目标"), 3, 0);
        layout->addWidget(target_label_, 3, 1);
        layout->addWidget(new QLabel("输入命令"), 4, 0);
        layout->addWidget(cmd_label_, 4, 1);
        layout->addWidget(new QLabel("输出速度"), 5, 0);
        layout->addWidget(output_label_, 5, 1);
        layout->setColumnStretch(1, 1);
        return group;
    }

    QWidget *makeLogGroup()
    {
        auto *group = new QGroupBox("操作日志");
        auto *layout = new QVBoxLayout(group);
        log_view_ = new QPlainTextEdit();
        log_view_->setReadOnly(true);
        log_view_->setMaximumBlockCount(120);
        layout->addWidget(log_view_);
        return group;
    }

    void applyStyle()
    {
        setStyleSheet(R"(
            QMainWindow, QWidget { background: #111b22; color: #edf4f2; font-size: 14px; }
            QGroupBox { background: #18262f; border: 1px solid #314b57; border-radius: 10px; margin-top: 16px; padding: 14px 12px 12px 12px; font-weight: 600; }
            QGroupBox::title { subcontrol-origin: margin; left: 14px; padding: 0 8px; color: #78dcca; }
            QDoubleSpinBox { background: #0d171e; color: #f6fbfa; border: 1px solid #36525f; border-radius: 7px; padding: 5px 8px; min-height: 26px; }
            QPushButton { background: #1d8b74; color: white; border: 0px; border-radius: 8px; padding: 8px 14px; font-weight: 600; }
            QPushButton:hover { background: #28a68d; }
            QLabel { color: #d8e8e4; }
            QPlainTextEdit { background: #0b141a; color: #cdeae2; border: 1px solid #314b57; border-radius: 8px; padding: 8px; font-family: "Monospace"; }
        )");
    }

    sunray_msgs::UGVControlCMD makeBaseCmd(const uint8_t control_cmd) const
    {
        sunray_msgs::UGVControlCMD cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.cmd_source = sunray_msgs::UGVControlCMD::SUNRAY_STATION;
        cmd.control_cmd = control_cmd;
        return cmd;
    }

    void publishSimple(const uint8_t control_cmd)
    {
        sunray_msgs::UGVControlCMD cmd = makeBaseCmd(control_cmd);
        cmd_pub_.publish(cmd);
        appendLog(QString("发布 %1 -> %2").arg(cmdName(control_cmd)).arg(QString::fromStdString(cmd_topic_)));
    }

    void publishMovePoint()
    {
        sunray_msgs::UGVControlCMD cmd = makeBaseCmd(sunray_msgs::UGVControlCMD::MOVE_POINT);
        cmd.desired_pos.x = point_x_spin_->value();
        cmd.desired_pos.y = point_y_spin_->value();
        cmd.desired_pos.z = 0.0;
        cmd.desired_yaw = point_yaw_spin_->value() * kDegToRad;
        cmd_pub_.publish(cmd);
        appendLog(QString("发布 MOVE_POINT x=%1 y=%2 yaw=%3deg")
                      .arg(point_x_spin_->value(), 0, 'f', 2)
                      .arg(point_y_spin_->value(), 0, 'f', 2)
                      .arg(point_yaw_spin_->value(), 0, 'f', 2));
    }

    void publishBodyVelocity()
    {
        sunray_msgs::UGVControlCMD cmd = makeBaseCmd(sunray_msgs::UGVControlCMD::MOVE_VELOCITY_BODY);
        cmd.cmd_vel.linear.x = body_vx_spin_->value();
        cmd.cmd_vel.linear.y = body_vy_spin_->value();
        cmd.cmd_vel.angular.z = body_wz_spin_->value() * kDegToRad;
        cmd_pub_.publish(cmd);
        appendLog(QString("发布 MOVE_VELOCITY_BODY vx=%1 vy=%2 wz=%3deg/s")
                      .arg(body_vx_spin_->value(), 0, 'f', 2)
                      .arg(body_vy_spin_->value(), 0, 'f', 2)
                      .arg(body_wz_spin_->value(), 0, 'f', 2));
    }

    void publishWorldVelocity()
    {
        sunray_msgs::UGVControlCMD cmd = makeBaseCmd(sunray_msgs::UGVControlCMD::MOVE_VELOCITY);
        cmd.desired_vel.x = world_vx_spin_->value();
        cmd.desired_vel.y = world_vy_spin_->value();
        cmd.desired_vel.z = 0.0;
        cmd.desired_yaw = world_yaw_spin_->value() * kDegToRad;
        cmd_pub_.publish(cmd);
        appendLog(QString("发布 MOVE_VELOCITY vx=%1 vy=%2 yaw=%3deg")
                      .arg(world_vx_spin_->value(), 0, 'f', 2)
                      .arg(world_vy_spin_->value(), 0, 'f', 2)
                      .arg(world_yaw_spin_->value(), 0, 'f', 2));
    }

    void publishWgs84()
    {
        sunray_msgs::UGVControlCMD cmd = makeBaseCmd(sunray_msgs::UGVControlCMD::MOVE_WGS84);
        cmd.desired_wgs84_pos.latitude = wgs84_lat_spin_->value();
        cmd.desired_wgs84_pos.longitude = wgs84_lon_spin_->value();
        cmd.desired_wgs84_pos.altitude = wgs84_alt_spin_->value();
        cmd_pub_.publish(cmd);
        appendLog(QString("发布 MOVE_WGS84 lat=%1 lon=%2 alt=%3")
                      .arg(wgs84_lat_spin_->value(), 0, 'f', 7)
                      .arg(wgs84_lon_spin_->value(), 0, 'f', 7)
                      .arg(wgs84_alt_spin_->value(), 0, 'f', 2));
    }

    void appendLog(const QString &text)
    {
        log_view_->appendPlainText(QString("[%1] %2")
                                       .arg(QString::number(ros::Time::now().toSec(), 'f', 2))
                                       .arg(text));
    }

    void stateCallback(const sunray_msgs::UGVControlState::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state_ = *msg;
        has_state_ = true;
    }

    void refreshState()
    {
        sunray_msgs::UGVControlState state;
        bool has_state = false;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            state = state_;
            has_state = has_state_;
        }

        if (!has_state)
        {
            agent_label_->setText("等待状态...");
            return;
        }

        const geometry_msgs::Point &pos = state.self_odom.pose.pose.position;
        const double yaw = yawFromOdom(state.self_odom);
        const QString agent_label =
            QString("/%1%2").arg(QString::fromStdString(state.agent_name)).arg(static_cast<int>(state.agent_id));

        agent_label_->setText(QString("%1  ID=%2  底盘=%3  FSM=%4")
                                  .arg(agent_label)
                                  .arg(static_cast<int>(state.agent_id))
                                  .arg(driveTypeName(state.drive_type))
                                  .arg(fsmName(state.fsm_state)));
        input_label_->setText(QString("odom=%1  cmd=%2  fence=%3")
                                  .arg(state.odom_valid ? "OK" : "BAD")
                                  .arg(state.control_cmd_valid ? "OK" : "BAD")
                                  .arg(state.inside_geo_fence ? "OK" : "BAD"));
        pose_label_->setText(QString("x=%1  y=%2  z=%3  yaw=%4 deg")
                                 .arg(pos.x, 0, 'f', 2)
                                 .arg(pos.y, 0, 'f', 2)
                                 .arg(pos.z, 0, 'f', 2)
                                 .arg(yaw * kRadToDeg, 0, 'f', 2));
        if (state.target_valid)
        {
            target_label_->setText(QString("x=%1  y=%2  yaw=%3 deg")
                                       .arg(state.target_pos.x, 0, 'f', 2)
                                       .arg(state.target_pos.y, 0, 'f', 2)
                                       .arg(state.target_yaw * kRadToDeg, 0, 'f', 2));
        }
        else
        {
            target_label_->setText("无明确目标点");
        }
        cmd_label_->setText(activeCmdText(state.active_ugv_control_cmd));
        output_label_->setText(QString("vx=%1  vy=%2  wz=%3")
                                   .arg(state.controller_cmd_vel.linear.x, 0, 'f', 2)
                                   .arg(state.controller_cmd_vel.linear.y, 0, 'f', 2)
                                   .arg(state.controller_cmd_vel.angular.z, 0, 'f', 2));
    }

    ros::NodeHandle nh_;
    ros::Publisher cmd_pub_;
    ros::Subscriber state_sub_;
    std::string agent_name_;
    int agent_id_{1};
    std::string cmd_topic_;
    std::string state_topic_;

    QDoubleSpinBox *point_x_spin_{nullptr};
    QDoubleSpinBox *point_y_spin_{nullptr};
    QDoubleSpinBox *point_yaw_spin_{nullptr};
    QDoubleSpinBox *body_vx_spin_{nullptr};
    QDoubleSpinBox *body_vy_spin_{nullptr};
    QDoubleSpinBox *body_wz_spin_{nullptr};
    QDoubleSpinBox *world_vx_spin_{nullptr};
    QDoubleSpinBox *world_vy_spin_{nullptr};
    QDoubleSpinBox *world_yaw_spin_{nullptr};
    QDoubleSpinBox *wgs84_lat_spin_{nullptr};
    QDoubleSpinBox *wgs84_lon_spin_{nullptr};
    QDoubleSpinBox *wgs84_alt_spin_{nullptr};
    QLabel *agent_label_{nullptr};
    QLabel *input_label_{nullptr};
    QLabel *pose_label_{nullptr};
    QLabel *target_label_{nullptr};
    QLabel *cmd_label_{nullptr};
    QLabel *output_label_{nullptr};
    QPlainTextEdit *log_view_{nullptr};
    QTimer *refresh_timer_{nullptr};

    std::mutex state_mutex_;
    sunray_msgs::UGVControlState state_;
    bool has_state_{false};
};
} // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ugv_control_panel_node");
    QApplication app(argc, argv);
    ros::NodeHandle nh("~");

    ros::AsyncSpinner spinner(2);
    spinner.start();

    UGVControlPanel panel(nh);
    panel.show();

    const int ret = app.exec();
    ros::shutdown();
    return ret;
}
