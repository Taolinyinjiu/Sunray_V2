/*
本程序功能：
    1、Qt 无人机控制面板
    2、发布 UAVControlCMD
    3、订阅 UAVControlFSMState 和 local_odom 并显示无人机状态
*/
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sunray_msgs/UAVControlCMD.h>
#include <sunray_msgs/UAVControlFSMState.h>
#include <visualization_msgs/MarkerArray.h>

#include <QApplication>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QLabel>
#include <QMainWindow>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
#include <QMouseEvent>

#include <cmath>
#include <deque>
#include <functional>
#include <mutex>
#include <string>

#include <OgrePlane.h>
#include <OgreRenderWindow.h>
#include <OgreVector3.h>
#include <rviz/display.h>
#include <rviz/geometry.h>
#include <rviz/render_panel.h>
#include <rviz/visualization_manager.h>

namespace
{
constexpr double kDegToRad = 0.017453292519943295;
constexpr double kRadToDeg = 57.29577951308232;

struct GoalVisual
{
    bool valid{false};
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double yaw{0.0};
    std::string label{};
};

std_msgs::ColorRGBA makeColor(const double r, const double g, const double b, const double a)
{
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}

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
    case sunray_msgs::UAVControlFSMState::FSM_OFF:
        return "OFF";
    case sunray_msgs::UAVControlFSMState::FSM_INIT:
        return "INIT";
    case sunray_msgs::UAVControlFSMState::FSM_TAKEOFF:
        return "TAKEOFF";
    case sunray_msgs::UAVControlFSMState::FSM_HOVER:
        return "HOVER";
    case sunray_msgs::UAVControlFSMState::FSM_RETURN:
        return "RETURN";
    case sunray_msgs::UAVControlFSMState::FSM_LAND:
        return "LAND";
    case sunray_msgs::UAVControlFSMState::FSM_MOVE:
        return "MOVE";
    case sunray_msgs::UAVControlFSMState::EMERGENCY_KILL:
        return "KILL";
    default:
        return "UNKNOWN";
    }
}

QString cmdName(const uint8_t cmd)
{
    switch (cmd)
    {
    case sunray_msgs::UAVControlCMD::TAKEOFF:
        return "TAKEOFF";
    case sunray_msgs::UAVControlCMD::LAND:
        return "LAND";
    case sunray_msgs::UAVControlCMD::RETURN:
        return "RETURN";
    case sunray_msgs::UAVControlCMD::KILL:
        return "KILL";
    case sunray_msgs::UAVControlCMD::HOVER:
        return "HOVER";
    case sunray_msgs::UAVControlCMD::MOVE_POINT:
        return "MOVE_POINT";
    case sunray_msgs::UAVControlCMD::MOVE_VELOCITY:
        return "MOVE_VELOCITY";
    case sunray_msgs::UAVControlCMD::MOVE_TRAJECTORY:
        return "MOVE_TRAJECTORY";
    case sunray_msgs::UAVControlCMD::MOVE_POINT_BODY:
        return "MOVE_POINT_BODY";
    case sunray_msgs::UAVControlCMD::MOVE_VELOCITY_BODY:
        return "MOVE_VELOCITY_BODY";
    case sunray_msgs::UAVControlCMD::MOVE_POINT_WGS84:
        return "MOVE_POINT_WGS84";
    default:
        return "UNDEFINE";
    }
}

QString yawModeName(const uint8_t mode)
{
    switch (mode)
    {
    case sunray_msgs::UAVControlCMD::KEEP_YAW:
        return "KEEP_YAW";
    case sunray_msgs::UAVControlCMD::SET_YAW:
        return "SET_YAW";
    case sunray_msgs::UAVControlCMD::SET_YAWRATE:
        return "SET_YAWRATE";
    default:
        return "UNKNOWN";
    }
}

double yawFromOdom(const nav_msgs::Odometry &odom)
{
    const auto &q = odom.pose.pose.orientation;
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

geometry_msgs::Point makePoint(const double x, const double y, const double z)
{
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

class ControlRenderPanel : public rviz::RenderPanel
{
  public:
    explicit ControlRenderPanel(QWidget *parent = nullptr)
        : rviz::RenderPanel(parent)
    {
    }

    void setPickCallback(const std::function<void(double, double)> &callback)
    {
        pick_callback_ = callback;
    }

    void setPickHeightProvider(const std::function<double()> &provider)
    {
        pick_height_provider_ = provider;
    }

  protected:
    void mousePressEvent(QMouseEvent *event) override
    {
        if (event != nullptr && event->button() == Qt::LeftButton &&
            (event->modifiers() & Qt::ControlModifier) && pick_callback_ != nullptr &&
            getRenderWindow() != nullptr)
        {
            Ogre::Viewport *viewport = getRenderWindow()->getViewport(0);
            if (viewport != nullptr)
            {
                Ogre::Plane plane(Ogre::Vector3::UNIT_Z,
                                  static_cast<Ogre::Real>(pick_height_provider_ != nullptr ? pick_height_provider_() : 0.0));
                Ogre::Vector3 intersection;
                if (rviz::getPointOnPlaneFromWindowXY(viewport, plane, event->x(), event->y(), intersection))
                {
                    pick_callback_(intersection.x, intersection.y);
                    event->accept();
                    return;
                }
            }
        }

        rviz::RenderPanel::mousePressEvent(event);
    }

  private:
    std::function<void(double, double)> pick_callback_{};
    std::function<double()> pick_height_provider_{};
};

class UAVControlPanel : public QMainWindow
{
  public:
    explicit UAVControlPanel(ros::NodeHandle &nh, QWidget *parent = nullptr)
        : QMainWindow(parent), nh_(nh)
    {
        loadParams();
        setupRos();
        setupUi();
        applyStyle();

        refresh_timer_ = new QTimer(this);
        connect(refresh_timer_, &QTimer::timeout, this, [this]() { refreshState(); });
        refresh_timer_->start(200);

        shutdown_timer_ = new QTimer(this);
        connect(shutdown_timer_, &QTimer::timeout, this, [this]() {
            if (!ros::ok())
            {
                close();
                qApp->quit();
            }
        });
        shutdown_timer_->start(100);

        setWindowTitle("Sunray UAV Control Panel");
        resize(1080, 700);
    }

  private:
    void loadParams()
    {
        nh_.param("uav_id", uav_id_, 1);
        const std::string prefix = "/uav" + std::to_string(uav_id_);
        nh_.param("cmd_topic", cmd_topic_, prefix + "/sunray/uav_control_cmd");
        nh_.param("fsm_state_topic", fsm_state_topic_, prefix + "/sunray/fsm/state");
        nh_.param("odom_topic", odom_topic_, prefix + "/sunray/localization/local_odom");
        nh_.param("rviz_frame_id", rviz_frame_id_, std::string("world"));
        nh_.param("marker_topic", marker_topic_, prefix + "/sunray/uav_control_panel/markers");
    }

    void setupRos()
    {
        cmd_pub_ = nh_.advertise<sunray_msgs::UAVControlCMD>(cmd_topic_, 10);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 10);
        fsm_state_sub_ = nh_.subscribe(fsm_state_topic_, 20, &UAVControlPanel::fsmStateCallback, this);
        odom_sub_ = nh_.subscribe(odom_topic_, 20, &UAVControlPanel::odomCallback, this);
    }

    void setupUi()
    {
        auto *central = new QWidget(this);
        auto *main_layout = new QHBoxLayout(central);
        main_layout->setContentsMargins(16, 16, 16, 16);
        main_layout->setSpacing(14);

        auto *left_widget = new QWidget(central);
        auto *left_layout = new QVBoxLayout(left_widget);
        left_layout->setContentsMargins(0, 0, 0, 0);
        left_layout->setSpacing(12);
        left_layout->addWidget(makeCommandGroup());
        left_layout->addWidget(makeStateGroup());
        left_layout->addWidget(makeLogGroup(), 1);

        auto *right_group = new QGroupBox("三维视图");
        auto *right_layout = new QVBoxLayout(right_group);
        right_layout->setContentsMargins(10, 18, 10, 10);
        setupRvizPanel();
        right_layout->addWidget(rviz_panel_);

        main_layout->addWidget(left_widget, 5);
        main_layout->addWidget(right_group, 6);
        setCentralWidget(central);
    }

    void setupRvizPanel()
    {
        rviz_panel_ = new ControlRenderPanel();
        rviz_panel_->setPickHeightProvider([this]() { return point_z_spin_ != nullptr ? point_z_spin_->value() : 0.0; });
        rviz_panel_->setPickCallback([this](const double x, const double y) {
            if (point_x_spin_ != nullptr && point_y_spin_ != nullptr)
            {
                point_x_spin_->setValue(x);
                point_y_spin_->setValue(y);
                appendLog(QString("RViz选点 -> x=%1 y=%2 z=%3")
                              .arg(x, 0, 'f', 2)
                              .arg(y, 0, 'f', 2)
                              .arg(point_z_spin_ != nullptr ? point_z_spin_->value() : 0.0, 0, 'f', 2));
                publishMovePoint();
            }
        });
        rviz_manager_ = new rviz::VisualizationManager(rviz_panel_);
        rviz_panel_->initialize(rviz_manager_->getSceneManager(), rviz_manager_);
        rviz_manager_->initialize();
        rviz_manager_->setFixedFrame(QString::fromStdString(rviz_frame_id_));
        rviz_manager_->startUpdate();

        rviz::Display *grid_display = rviz_manager_->createDisplay("rviz/Grid", "Grid", true);
        if (grid_display != nullptr)
        {
            grid_display->subProp("Plane Cell Count")->setValue(30);
            grid_display->subProp("Cell Size")->setValue(1.0);
            grid_display->subProp("Color")->setValue(QColor(90, 100, 110));
            grid_display->subProp("Alpha")->setValue(0.55);
        }

        rviz::Display *marker_display =
            rviz_manager_->createDisplay("rviz/MarkerArray", "PanelMarkers", true);
        if (marker_display != nullptr)
        {
            marker_display->subProp("Marker Topic")->setValue(QString::fromStdString(marker_topic_));
        }
    }

    QWidget *makeCommandGroup()
    {
        auto *group = new QGroupBox("无人机控制");
        auto *layout = new QGridLayout(group);

        auto *takeoff_btn = new QPushButton("TAKEOFF 起飞");
        auto *land_btn = new QPushButton("LAND 降落");
        auto *return_btn = new QPushButton("RETURN 返航");
        auto *hover_btn = new QPushButton("HOVER 悬停");
        auto *kill_btn = new QPushButton("KILL 急停");

        connect(takeoff_btn, &QPushButton::clicked, this, [this]() { publishSimple(sunray_msgs::UAVControlCMD::TAKEOFF); });
        connect(land_btn, &QPushButton::clicked, this, [this]() { publishSimple(sunray_msgs::UAVControlCMD::LAND); });
        connect(return_btn, &QPushButton::clicked, this, [this]() { publishSimple(sunray_msgs::UAVControlCMD::RETURN); });
        connect(hover_btn, &QPushButton::clicked, this, [this]() { publishSimple(sunray_msgs::UAVControlCMD::HOVER); });
        connect(kill_btn, &QPushButton::clicked, this, [this]() { publishSimple(sunray_msgs::UAVControlCMD::KILL); });

        point_x_spin_ = makeSpin(0.0, -100.0, 100.0, 0.1);
        point_y_spin_ = makeSpin(0.0, -100.0, 100.0, 0.1);
        point_z_spin_ = makeSpin(1.5, -10.0, 100.0, 0.1);
        point_yaw_spin_ = makeSpin(0.0, -180.0, 180.0, 5.0);
        auto *point_btn = new QPushButton("发送 MOVE_POINT");
        connect(point_btn, &QPushButton::clicked, this, [this]() { publishMovePoint(); });

        body_x_spin_ = makeSpin(0.0, -20.0, 20.0, 0.1);
        body_y_spin_ = makeSpin(0.0, -20.0, 20.0, 0.1);
        body_height_spin_ = makeSpin(1.5, -10.0, 100.0, 0.1);
        body_yaw_spin_ = makeSpin(0.0, -180.0, 180.0, 5.0);
        auto *body_point_btn = new QPushButton("发送 MOVE_POINT_BODY");
        connect(body_point_btn, &QPushButton::clicked, this, [this]() { publishMovePointBody(); });

        vel_x_spin_ = makeSpin(0.0, -5.0, 5.0, 0.1);
        vel_y_spin_ = makeSpin(0.0, -5.0, 5.0, 0.1);
        vel_z_spin_ = makeSpin(0.0, -5.0, 5.0, 0.1);
        vel_yaw_spin_ = makeSpin(0.0, -180.0, 180.0, 5.0);
        auto *vel_btn = new QPushButton("发送 MOVE_VELOCITY");
        connect(vel_btn, &QPushButton::clicked, this, [this]() { publishMoveVelocity(); });

        body_vx_spin_ = makeSpin(0.0, -5.0, 5.0, 0.1);
        body_vy_spin_ = makeSpin(0.0, -5.0, 5.0, 0.1);
        body_vh_spin_ = makeSpin(1.5, -10.0, 100.0, 0.1);
        body_vyaw_spin_ = makeSpin(0.0, -180.0, 180.0, 5.0);
        auto *body_vel_btn = new QPushButton("发送 MOVE_VELOCITY_BODY");
        connect(body_vel_btn, &QPushButton::clicked, this, [this]() { publishMoveVelocityBody(); });

        layout->addWidget(new QLabel("快捷指令"), 0, 0);
        layout->addWidget(takeoff_btn, 0, 1);
        layout->addWidget(land_btn, 0, 2);
        layout->addWidget(return_btn, 0, 3);
        layout->addWidget(hover_btn, 0, 4);
        layout->addWidget(kill_btn, 0, 5);

        layout->addWidget(new QLabel("目标点 x/y/z/yaw(deg)"), 1, 0);
        layout->addWidget(point_x_spin_, 1, 1);
        layout->addWidget(point_y_spin_, 1, 2);
        layout->addWidget(point_z_spin_, 1, 3);
        layout->addWidget(point_yaw_spin_, 1, 4);
        layout->addWidget(point_btn, 1, 5);

        layout->addWidget(new QLabel("机体系点 x/y/height/yaw(deg)"), 2, 0);
        layout->addWidget(body_x_spin_, 2, 1);
        layout->addWidget(body_y_spin_, 2, 2);
        layout->addWidget(body_height_spin_, 2, 3);
        layout->addWidget(body_yaw_spin_, 2, 4);
        layout->addWidget(body_point_btn, 2, 5);

        layout->addWidget(new QLabel("速度 vx/vy/vz/yaw(deg)"), 3, 0);
        layout->addWidget(vel_x_spin_, 3, 1);
        layout->addWidget(vel_y_spin_, 3, 2);
        layout->addWidget(vel_z_spin_, 3, 3);
        layout->addWidget(vel_yaw_spin_, 3, 4);
        layout->addWidget(vel_btn, 3, 5);

        layout->addWidget(new QLabel("机体系速度 vx/vy/height/yaw(deg)"), 4, 0);
        layout->addWidget(body_vx_spin_, 4, 1);
        layout->addWidget(body_vy_spin_, 4, 2);
        layout->addWidget(body_vh_spin_, 4, 3);
        layout->addWidget(body_vyaw_spin_, 4, 4);
        layout->addWidget(body_vel_btn, 4, 5);

        layout->setColumnStretch(6, 1);
        return group;
    }

    QWidget *makeStateGroup()
    {
        auto *group = new QGroupBox("运行状态");
        auto *layout = new QGridLayout(group);
        basic_label_ = new QLabel("-");
        pose_label_ = new QLabel("-");
        flight_label_ = new QLabel("-");
        cmd_label_ = new QLabel("-");
        topic_label_ = new QLabel("-");

        layout->addWidget(new QLabel("基本状态"), 0, 0);
        layout->addWidget(basic_label_, 0, 1);
        layout->addWidget(new QLabel("本机位姿"), 1, 0);
        layout->addWidget(pose_label_, 1, 1);
        layout->addWidget(new QLabel("飞行参数"), 2, 0);
        layout->addWidget(flight_label_, 2, 1);
        layout->addWidget(new QLabel("当前指令"), 3, 0);
        layout->addWidget(cmd_label_, 3, 1);
        layout->addWidget(new QLabel("话题信息"), 4, 0);
        layout->addWidget(topic_label_, 4, 1);
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

    sunray_msgs::UAVControlCMD makeBaseCmd(const uint8_t control_cmd) const
    {
        sunray_msgs::UAVControlCMD cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.cmd_source = sunray_msgs::UAVControlCMD::SUNRAY_STATION;
        cmd.control_cmd = control_cmd;
        cmd.yaw_mode = sunray_msgs::UAVControlCMD::SET_YAW;
        return cmd;
    }

    void publishSimple(const uint8_t control_cmd)
    {
        sunray_msgs::UAVControlCMD cmd = makeBaseCmd(control_cmd);
        if (control_cmd == sunray_msgs::UAVControlCMD::HOVER) {
            cmd.yaw_mode = sunray_msgs::UAVControlCMD::KEEP_YAW;
        }
        last_sent_yaw_mode_ = cmd.yaw_mode;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            last_sent_cmd_ = cmd;
        }
        cmd_pub_.publish(cmd);
        appendLog(QString("发布 %1 -> %2").arg(cmdName(control_cmd)).arg(QString::fromStdString(cmd_topic_)));
    }

    void publishMovePoint()
    {
        sunray_msgs::UAVControlCMD cmd = makeBaseCmd(sunray_msgs::UAVControlCMD::MOVE_POINT);
        cmd.desired_pos.x = point_x_spin_->value();
        cmd.desired_pos.y = point_y_spin_->value();
        cmd.desired_pos.z = point_z_spin_->value();
        cmd.desired_yaw = point_yaw_spin_->value() * kDegToRad;
        last_sent_yaw_mode_ = cmd.yaw_mode;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            last_sent_cmd_ = cmd;
        }
        cmd_pub_.publish(cmd);
        appendLog(QString("发布 MOVE_POINT x=%1 y=%2 z=%3 yaw=%4deg")
                      .arg(point_x_spin_->value(), 0, 'f', 2)
                      .arg(point_y_spin_->value(), 0, 'f', 2)
                      .arg(point_z_spin_->value(), 0, 'f', 2)
                      .arg(point_yaw_spin_->value(), 0, 'f', 2));
    }

    void publishMovePointBody()
    {
        sunray_msgs::UAVControlCMD cmd = makeBaseCmd(sunray_msgs::UAVControlCMD::MOVE_POINT_BODY);
        cmd.desired_body_xy_pos.x = body_x_spin_->value();
        cmd.desired_body_xy_pos.y = body_y_spin_->value();
        cmd.fixed_height = body_height_spin_->value();
        cmd.desired_yaw = body_yaw_spin_->value() * kDegToRad;
        last_sent_yaw_mode_ = cmd.yaw_mode;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            last_sent_cmd_ = cmd;
        }
        cmd_pub_.publish(cmd);
        appendLog(QString("发布 MOVE_POINT_BODY x=%1 y=%2 h=%3 yaw=%4deg")
                      .arg(body_x_spin_->value(), 0, 'f', 2)
                      .arg(body_y_spin_->value(), 0, 'f', 2)
                      .arg(body_height_spin_->value(), 0, 'f', 2)
                      .arg(body_yaw_spin_->value(), 0, 'f', 2));
    }

    void publishMoveVelocity()
    {
        sunray_msgs::UAVControlCMD cmd = makeBaseCmd(sunray_msgs::UAVControlCMD::MOVE_VELOCITY);
        cmd.desired_vel.x = vel_x_spin_->value();
        cmd.desired_vel.y = vel_y_spin_->value();
        cmd.desired_vel.z = vel_z_spin_->value();
        cmd.desired_yaw = vel_yaw_spin_->value() * kDegToRad;
        last_sent_yaw_mode_ = cmd.yaw_mode;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            last_sent_cmd_ = cmd;
        }
        cmd_pub_.publish(cmd);
        appendLog(QString("发布 MOVE_VELOCITY vx=%1 vy=%2 vz=%3 yaw=%4deg")
                      .arg(vel_x_spin_->value(), 0, 'f', 2)
                      .arg(vel_y_spin_->value(), 0, 'f', 2)
                      .arg(vel_z_spin_->value(), 0, 'f', 2)
                      .arg(vel_yaw_spin_->value(), 0, 'f', 2));
    }

    void publishMoveVelocityBody()
    {
        sunray_msgs::UAVControlCMD cmd = makeBaseCmd(sunray_msgs::UAVControlCMD::MOVE_VELOCITY_BODY);
        cmd.desired_body_xy_vel.x = body_vx_spin_->value();
        cmd.desired_body_xy_vel.y = body_vy_spin_->value();
        cmd.fixed_height = body_vh_spin_->value();
        cmd.desired_yaw = body_vyaw_spin_->value() * kDegToRad;
        last_sent_yaw_mode_ = cmd.yaw_mode;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            last_sent_cmd_ = cmd;
        }
        cmd_pub_.publish(cmd);
        appendLog(QString("发布 MOVE_VELOCITY_BODY vx=%1 vy=%2 h=%3 yaw=%4deg")
                      .arg(body_vx_spin_->value(), 0, 'f', 2)
                      .arg(body_vy_spin_->value(), 0, 'f', 2)
                      .arg(body_vh_spin_->value(), 0, 'f', 2)
                      .arg(body_vyaw_spin_->value(), 0, 'f', 2));
    }

    void appendLog(const QString &text)
    {
        log_view_->appendPlainText(QString("[%1] %2")
                                       .arg(QString::number(ros::Time::now().toSec(), 'f', 2))
                                       .arg(text));
    }

    void fsmStateCallback(const sunray_msgs::UAVControlFSMState::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        fsm_state_ = *msg;
        has_fsm_state_ = true;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        odom_ = *msg;
        has_odom_ = true;

        const geometry_msgs::Point &pos = odom_.pose.pose.position;
        if (trail_.empty() || distanceSq(trail_.back(), pos) > 1e-6)
        {
            trail_.push_back(pos);
            while (trail_.size() > 50)
            {
                trail_.pop_front();
            }
        }
    }

    void refreshState()
    {
        sunray_msgs::UAVControlFSMState fsm_state;
        nav_msgs::Odometry odom;
        sunray_msgs::UAVControlCMD last_sent_cmd;
        std::deque<geometry_msgs::Point> trail;
        bool has_fsm_state = false;
        bool has_odom = false;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            fsm_state = fsm_state_;
            odom = odom_;
            last_sent_cmd = last_sent_cmd_;
            trail = trail_;
            has_fsm_state = has_fsm_state_;
            has_odom = has_odom_;
        }

        basic_label_->setText(QString("/uav%1  FSM=%2").arg(uav_id_).arg(has_fsm_state ? fsmName(fsm_state.sunray_fsm_state) : "等待状态..."));

        if (has_odom)
        {
            const auto &pos = odom.pose.pose.position;
            const double yaw = yawFromOdom(odom);
            pose_label_->setText(QString("x=%1  y=%2  z=%3  yaw=%4 deg")
                                     .arg(pos.x, 0, 'f', 2)
                                     .arg(pos.y, 0, 'f', 2)
                                     .arg(pos.z, 0, 'f', 2)
                                     .arg(yaw * kRadToDeg, 0, 'f', 2));
        }
        else
        {
            pose_label_->setText("等待 local_odom...");
        }

        if (has_fsm_state)
        {
            flight_label_->setText(QString("takeoff_h=%1 m  takeoff_v=%2 m/s  land_type=%3  land_v=%4 m/s")
                                       .arg(fsm_state.takeoff_relative_height, 0, 'f', 2)
                                       .arg(fsm_state.takeoff_max_velocity, 0, 'f', 2)
                                       .arg(static_cast<int>(fsm_state.land_type))
                                       .arg(fsm_state.land_max_velocity, 0, 'f', 2));
            cmd_label_->setText(QString("%1  |  yaw_mode=%2")
                                    .arg(cmdName(fsm_state.control_cmd))
                                    .arg(yawModeName(last_sent_yaw_mode_)));
        }
        else
        {
            flight_label_->setText("等待 UAVControlFSMState...");
            cmd_label_->setText("-");
        }

        topic_label_->setText(QString("cmd: %1\nfsm: %2\nodom: %3")
                                  .arg(QString::fromStdString(cmd_topic_))
                                  .arg(QString::fromStdString(fsm_state_topic_))
                                  .arg(QString::fromStdString(odom_topic_)));

        publishVisualization(has_fsm_state, fsm_state, has_odom, odom, last_sent_cmd, trail);
    }

    double distanceSq(const geometry_msgs::Point &a, const geometry_msgs::Point &b) const
    {
        const double dx = a.x - b.x;
        const double dy = a.y - b.y;
        const double dz = a.z - b.z;
        return dx * dx + dy * dy + dz * dz;
    }

    GoalVisual resolveGoalVisual(const bool has_odom,
                                 const nav_msgs::Odometry &odom,
                                 const sunray_msgs::UAVControlCMD &cmd) const
    {
        GoalVisual goal;
        const double current_yaw = has_odom ? yawFromOdom(odom) : 0.0;
        const double cos_yaw = std::cos(current_yaw);
        const double sin_yaw = std::sin(current_yaw);

        switch (cmd.control_cmd)
        {
        case sunray_msgs::UAVControlCMD::MOVE_POINT:
            goal.valid = true;
            goal.x = cmd.desired_pos.x;
            goal.y = cmd.desired_pos.y;
            goal.z = cmd.desired_pos.z;
            goal.yaw = cmd.desired_yaw;
            goal.label = "MOVE_POINT";
            break;

        case sunray_msgs::UAVControlCMD::MOVE_POINT_BODY:
            if (!has_odom)
            {
                break;
            }
            goal.valid = true;
            goal.x = odom.pose.pose.position.x + cos_yaw * cmd.desired_body_xy_pos.x -
                     sin_yaw * cmd.desired_body_xy_pos.y;
            goal.y = odom.pose.pose.position.y + sin_yaw * cmd.desired_body_xy_pos.x +
                     cos_yaw * cmd.desired_body_xy_pos.y;
            goal.z = cmd.fixed_height;
            goal.yaw = cmd.desired_yaw;
            goal.label = "MOVE_POINT_BODY";
            break;

        case sunray_msgs::UAVControlCMD::MOVE_VELOCITY:
            if (!has_odom)
            {
                break;
            }
            goal.valid = true;
            goal.x = odom.pose.pose.position.x + cmd.desired_vel.x;
            goal.y = odom.pose.pose.position.y + cmd.desired_vel.y;
            goal.z = odom.pose.pose.position.z + cmd.desired_vel.z;
            goal.yaw = cmd.desired_yaw;
            goal.label = "MOVE_VELOCITY(1s)";
            break;

        case sunray_msgs::UAVControlCMD::MOVE_VELOCITY_BODY:
            if (!has_odom)
            {
                break;
            }
            goal.valid = true;
            goal.x = odom.pose.pose.position.x + cos_yaw * cmd.desired_body_xy_vel.x -
                     sin_yaw * cmd.desired_body_xy_vel.y;
            goal.y = odom.pose.pose.position.y + sin_yaw * cmd.desired_body_xy_vel.x +
                     cos_yaw * cmd.desired_body_xy_vel.y;
            goal.z = cmd.fixed_height;
            goal.yaw = cmd.desired_yaw;
            goal.label = "MOVE_VELOCITY_BODY(1s)";
            break;

        case sunray_msgs::UAVControlCMD::HOVER:
            if (!has_odom)
            {
                break;
            }
            goal.valid = true;
            goal.x = odom.pose.pose.position.x;
            goal.y = odom.pose.pose.position.y;
            goal.z = odom.pose.pose.position.z;
            goal.yaw = current_yaw;
            goal.label = "HOVER";
            break;

        case sunray_msgs::UAVControlCMD::RETURN:
            if (!has_odom)
            {
                break;
            }
            goal.valid = true;
            goal.x = odom.pose.pose.position.x;
            goal.y = odom.pose.pose.position.y;
            goal.z = 0.0;
            goal.yaw = current_yaw;
            goal.label = "RETURN";
            break;

        default:
            break;
        }

        return goal;
    }

    QString currentTaskText(const bool has_fsm_state,
                            const sunray_msgs::UAVControlFSMState &fsm_state,
                            const sunray_msgs::UAVControlCMD &cmd,
                            const GoalVisual &goal) const
    {
        QString task = has_fsm_state ? fsmName(fsm_state.sunray_fsm_state) : "WAIT_FSM";
        task += " | ";
        task += cmdName(cmd.control_cmd);
        if (goal.valid)
        {
            task += QString(" | goal=(%1,%2,%3)")
                        .arg(goal.x, 0, 'f', 2)
                        .arg(goal.y, 0, 'f', 2)
                        .arg(goal.z, 0, 'f', 2);
        }
        return task;
    }

    void publishVisualization(const bool has_fsm_state,
                              const sunray_msgs::UAVControlFSMState &fsm_state,
                              const bool has_odom,
                              const nav_msgs::Odometry &odom,
                              const sunray_msgs::UAVControlCMD &last_sent_cmd,
                              const std::deque<geometry_msgs::Point> &trail)
    {
        visualization_msgs::MarkerArray marker_array;
        const ros::Time stamp = ros::Time::now();
        const GoalVisual goal_visual = resolveGoalVisual(has_odom, odom, last_sent_cmd);

        visualization_msgs::Marker origin;
        origin.header.frame_id = rviz_frame_id_;
        origin.header.stamp = stamp;
        origin.ns = "world";
        origin.id = 0;
        origin.type = visualization_msgs::Marker::SPHERE;
        origin.action = visualization_msgs::Marker::ADD;
        origin.pose.orientation.w = 1.0;
        origin.scale.x = 0.15;
        origin.scale.y = 0.15;
        origin.scale.z = 0.15;
        origin.color = makeColor(1.0, 1.0, 1.0, 1.0);
        marker_array.markers.push_back(origin);

        visualization_msgs::Marker axis_x = origin;
        axis_x.id = 1;
        axis_x.type = visualization_msgs::Marker::ARROW;
        axis_x.scale.x = 0.04;
        axis_x.scale.y = 0.10;
        axis_x.scale.z = 0.12;
        axis_x.color = makeColor(1.0, 0.15, 0.15, 1.0);
        axis_x.points.resize(2);
        axis_x.points[1].x = 1.2;
        marker_array.markers.push_back(axis_x);

        visualization_msgs::Marker axis_y = axis_x;
        axis_y.id = 2;
        axis_y.color = makeColor(0.15, 1.0, 0.15, 1.0);
        axis_y.points[1].x = 0.0;
        axis_y.points[1].y = 1.2;
        marker_array.markers.push_back(axis_y);

        if (has_odom)
        {
            visualization_msgs::Marker uav = origin;
            uav.ns = "uav";
            uav.id = 10;
            uav.type = visualization_msgs::Marker::MESH_RESOURCE;
            uav.mesh_resource = "package://sunray_swarm_control/utils/meshes/uav.dae";
            uav.pose = odom.pose.pose;
            uav.scale.x = 1.0;
            uav.scale.y = 1.0;
            uav.scale.z = 1.0;
            uav.color = makeColor(0.20, 0.75, 1.0, 1.0);
            marker_array.markers.push_back(uav);

            visualization_msgs::Marker text = origin;
            text.ns = "uav";
            text.id = 11;
            text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text.pose.position = odom.pose.pose.position;
            text.pose.position.z += 0.45;
            text.scale.z = 0.25;
            text.color = makeColor(1.0, 1.0, 1.0, 1.0);
            text.text = "uav" + std::to_string(uav_id_);
            marker_array.markers.push_back(text);

            if (trail.size() >= 2)
            {
                visualization_msgs::Marker trail_marker = origin;
                trail_marker.ns = "trail";
                trail_marker.id = 12;
                trail_marker.type = visualization_msgs::Marker::LINE_STRIP;
                trail_marker.scale.x = 0.035;
                trail_marker.color = makeColor(0.15, 0.95, 0.95, 0.85);
                trail_marker.points.assign(trail.begin(), trail.end());
                marker_array.markers.push_back(trail_marker);
            }

            const geometry_msgs::Vector3 &vel = odom.twist.twist.linear;
            const double speed_sq = vel.x * vel.x + vel.y * vel.y + vel.z * vel.z;
            if (speed_sq > 1e-6)
            {
                visualization_msgs::Marker vel_arrow = origin;
                vel_arrow.ns = "velocity";
                vel_arrow.id = 13;
                vel_arrow.type = visualization_msgs::Marker::ARROW;
                vel_arrow.scale.x = 0.045;
                vel_arrow.scale.y = 0.10;
                vel_arrow.scale.z = 0.10;
                vel_arrow.color = makeColor(0.10, 0.55, 1.0, 0.95);
                vel_arrow.points.push_back(odom.pose.pose.position);
                vel_arrow.points.push_back(
                    makePoint(odom.pose.pose.position.x + vel.x,
                              odom.pose.pose.position.y + vel.y,
                              odom.pose.pose.position.z + vel.z));
                marker_array.markers.push_back(vel_arrow);
            }

            visualization_msgs::Marker task_text = origin;
            task_text.ns = "task";
            task_text.id = 14;
            task_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            task_text.pose.position = odom.pose.pose.position;
            task_text.pose.position.z += 0.78;
            task_text.scale.z = 0.20;
            task_text.color = makeColor(0.95, 0.95, 0.95, 1.0);
            task_text.text = currentTaskText(has_fsm_state, fsm_state, last_sent_cmd, goal_visual).toStdString();
            marker_array.markers.push_back(task_text);
        }

        if (goal_visual.valid)
        {
            visualization_msgs::Marker goal = origin;
            goal.ns = "goal";
            goal.id = 20;
            goal.type = visualization_msgs::Marker::ARROW;
            goal.scale.x = 0.05;
            goal.scale.y = 0.12;
            goal.scale.z = 0.12;
            goal.color = makeColor(1.0, 0.80, 0.15, 1.0);
            goal.points.push_back(makePoint(goal_visual.x, goal_visual.y, goal_visual.z));
            goal.points.push_back(makePoint(goal_visual.x + 0.45 * std::cos(goal_visual.yaw),
                                            goal_visual.y + 0.45 * std::sin(goal_visual.yaw),
                                            goal_visual.z));
            marker_array.markers.push_back(goal);

            visualization_msgs::Marker goal_text = origin;
            goal_text.ns = "goal";
            goal_text.id = 21;
            goal_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            goal_text.pose.position = makePoint(goal_visual.x, goal_visual.y, goal_visual.z + 0.35);
            goal_text.scale.z = 0.24;
            goal_text.color = makeColor(1.0, 0.92, 0.35, 1.0);
            goal_text.text = goal_visual.label;
            marker_array.markers.push_back(goal_text);
        }

        marker_pub_.publish(marker_array);
    }

    ros::NodeHandle nh_;
    ros::Publisher cmd_pub_;
    ros::Publisher marker_pub_;
    ros::Subscriber fsm_state_sub_;
    ros::Subscriber odom_sub_;
    int uav_id_{1};
    std::string cmd_topic_;
    std::string fsm_state_topic_;
    std::string odom_topic_;
    std::string rviz_frame_id_;
    std::string marker_topic_;

    QDoubleSpinBox *point_x_spin_{nullptr};
    QDoubleSpinBox *point_y_spin_{nullptr};
    QDoubleSpinBox *point_z_spin_{nullptr};
    QDoubleSpinBox *point_yaw_spin_{nullptr};
    QDoubleSpinBox *body_x_spin_{nullptr};
    QDoubleSpinBox *body_y_spin_{nullptr};
    QDoubleSpinBox *body_height_spin_{nullptr};
    QDoubleSpinBox *body_yaw_spin_{nullptr};
    QDoubleSpinBox *vel_x_spin_{nullptr};
    QDoubleSpinBox *vel_y_spin_{nullptr};
    QDoubleSpinBox *vel_z_spin_{nullptr};
    QDoubleSpinBox *vel_yaw_spin_{nullptr};
    QDoubleSpinBox *body_vx_spin_{nullptr};
    QDoubleSpinBox *body_vy_spin_{nullptr};
    QDoubleSpinBox *body_vh_spin_{nullptr};
    QDoubleSpinBox *body_vyaw_spin_{nullptr};
    QLabel *basic_label_{nullptr};
    QLabel *pose_label_{nullptr};
    QLabel *flight_label_{nullptr};
    QLabel *cmd_label_{nullptr};
    QLabel *topic_label_{nullptr};
    QPlainTextEdit *log_view_{nullptr};
    QTimer *refresh_timer_{nullptr};
    QTimer *shutdown_timer_{nullptr};
    ControlRenderPanel *rviz_panel_{nullptr};
    rviz::VisualizationManager *rviz_manager_{nullptr};

    std::mutex state_mutex_;
    sunray_msgs::UAVControlFSMState fsm_state_{};
    nav_msgs::Odometry odom_{};
    sunray_msgs::UAVControlCMD last_sent_cmd_{};
    std::deque<geometry_msgs::Point> trail_{};
    bool has_fsm_state_{false};
    bool has_odom_{false};
    uint8_t last_sent_yaw_mode_{sunray_msgs::UAVControlCMD::SET_YAW};
};
} // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_control_panel_node");
    QApplication app(argc, argv);
    ros::NodeHandle nh("~");

    ros::AsyncSpinner spinner(2);
    spinner.start();

    UAVControlPanel panel(nh);
    panel.show();

    const int ret = app.exec();
    ros::shutdown();
    return ret;
}
