/*
本程序功能：
    1、提供一个可通过 roslaunch 打开的 Qt 集群控制面板
    2、支持发布 UAVSwarmCMD / UGVSwarmCMD
    3、支持编辑常用 Formation 参数并查看 UAVSwarmState / UGVSwarmState 摘要
*/
#include "sunray_log.hpp"

#include <QAbstractItemView>
#include <QApplication>
#include <QColor>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QMainWindow>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QStackedWidget>
#include <QTableWidgetItem>
#include <QTableWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <algorithm>
#include <cmath>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sunray_msgs/Formation.h>
#include <sunray_msgs/UAVSwarmCMD.h>
#include <sunray_msgs/UAVSwarmState.h>
#include <sunray_msgs/UGVSwarmCMD.h>
#include <sunray_msgs/UGVSwarmState.h>

namespace
{

enum class Platform
{
    UAV = 0,
    UGV = 1
};

struct CachedState
{
    std::string platform;
    int agent_id{0};
    int swarm_num{0};
    bool self_odom_ready{false};
    bool peers_odom_ready{false};
    uint32_t ready_peer_num{0};
    uint8_t fsm_state{0};
    bool target_valid{false};
    geometry_msgs::Point target_pos{};
    double target_yaw{0.0};
    ros::Time receive_time{0.0};
};

QString formatDouble(const double value, const int precision = 2)
{
    return QString::number(value, 'f', precision);
}

double yawFromOdom(const nav_msgs::Odometry &odom)
{
    const geometry_msgs::Quaternion &q = odom.pose.pose.orientation;
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

const char *formationName(const uint8_t formation_type)
{
    switch (formation_type)
    {
    case sunray_msgs::Formation::STATIC_KEEP_FORMATION:
        return "STATIC_KEEP_FORMATION";
    case sunray_msgs::Formation::STATIC_FORMATION_LINE:
        return "STATIC_FORMATION_LINE";
    case sunray_msgs::Formation::STATIC_FORMATION_POLYGON:
        return "STATIC_FORMATION_POLYGON";
    case sunray_msgs::Formation::STATIC_FORMATION_RANDOM:
        return "STATIC_FORMATION_RANDOM";
    case sunray_msgs::Formation::STATIC_FORMATION_CUSTOM:
        return "STATIC_FORMATION_CUSTOM";
    case sunray_msgs::Formation::DYNAMIC_FORMATION_RING:
        return "DYNAMIC_FORMATION_RING";
    case sunray_msgs::Formation::DYNAMIC_FORMATION_POLYGON:
        return "DYNAMIC_FORMATION_POLYGON";
    case sunray_msgs::Formation::DYNAMIC_FORMATION_LEMNISCATE:
        return "DYNAMIC_FORMATION_LEMNISCATE";
    default:
        return "UNKNOWN";
    }
}

const char *uavStateName(const uint8_t state)
{
    switch (state)
    {
    case sunray_msgs::UAVSwarmState::INIT:
        return "INIT";
    case sunray_msgs::UAVSwarmState::TAKEOFF:
        return "TAKEOFF";
    case sunray_msgs::UAVSwarmState::LAND:
        return "LAND";
    case sunray_msgs::UAVSwarmState::RETURN_HOME:
        return "RETURN_HOME";
    case sunray_msgs::UAVSwarmState::ARRIVED:
        return "ARRIVED";
    case sunray_msgs::UAVSwarmState::SWARM_STATIC_FORMATION:
        return "STATIC_FORMATION";
    case sunray_msgs::UAVSwarmState::SWARM_DYNAMIC_FORMATION:
        return "DYNAMIC_FORMATION";
    case sunray_msgs::UAVSwarmState::SWARM_DYNAMIC_FORMATION_PREPARE:
        return "DYNAMIC_PREPARE";
    default:
        return "UNKNOWN";
    }
}

const char *ugvStateName(const uint8_t state)
{
    switch (state)
    {
    case sunray_msgs::UGVSwarmState::INIT:
        return "INIT";
    case sunray_msgs::UGVSwarmState::RETURN_HOME:
        return "RETURN_HOME";
    case sunray_msgs::UGVSwarmState::ARRIVED:
        return "ARRIVED";
    case sunray_msgs::UGVSwarmState::SWARM_STATIC_FORMATION:
        return "STATIC_FORMATION";
    case sunray_msgs::UGVSwarmState::SWARM_DYNAMIC_FORMATION:
        return "DYNAMIC_FORMATION";
    case sunray_msgs::UGVSwarmState::SWARM_DYNAMIC_FORMATION_PREPARE:
        return "DYNAMIC_PREPARE";
    default:
        return "UNKNOWN";
    }
}

QDoubleSpinBox *makeDoubleSpin(const double value,
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

class SwarmControlPanel : public QMainWindow
{
  public:
    explicit SwarmControlPanel(ros::NodeHandle &nh, QWidget *parent = nullptr)
        : QMainWindow(parent), nh_(nh)
    {
        loadParams();
        setupRos();
        setupUi();
        applyStyle();

        refresh_timer_ = new QTimer(this);
        connect(refresh_timer_, &QTimer::timeout, this, [this]() { refreshStateTable(); });
        refresh_timer_->start(200);

        setWindowTitle("Sunray Swarm Control Panel");
        resize(1180, 760);
    }

  private:
    void loadParams()
    {
        nh_.param("uav_swarm_cmd_topic", uav_swarm_cmd_topic_, std::string("/sunray/swarm/uav_swarm_cmd"));
        nh_.param("ugv_swarm_cmd_topic", ugv_swarm_cmd_topic_, std::string("/sunray/swarm/ugv_swarm_cmd"));
        nh_.param("uav_swarm_state_topic", uav_swarm_state_topic_, std::string("/sunray/swarm/uav_swarm_state"));
        nh_.param("ugv_swarm_state_topic", ugv_swarm_state_topic_, std::string("/sunray/swarm/ugv_swarm_state"));
        nh_.param("state_timeout", state_timeout_, 1.0);
    }

    void setupRos()
    {
        uav_cmd_pub_ = nh_.advertise<sunray_msgs::UAVSwarmCMD>(uav_swarm_cmd_topic_, 10);
        ugv_cmd_pub_ = nh_.advertise<sunray_msgs::UGVSwarmCMD>(ugv_swarm_cmd_topic_, 10);
        uav_state_sub_ =
            nh_.subscribe(uav_swarm_state_topic_, 100, &SwarmControlPanel::uavStateCallback, this);
        ugv_state_sub_ =
            nh_.subscribe(ugv_swarm_state_topic_, 100, &SwarmControlPanel::ugvStateCallback, this);
    }

    void setupUi()
    {
        auto *central = new QWidget(this);
        auto *main_layout = new QVBoxLayout(central);
        main_layout->setContentsMargins(16, 16, 16, 16);
        main_layout->setSpacing(12);
        main_layout->addWidget(makeCommandGroup());
        main_layout->addWidget(makeFormationGroup());
        main_layout->addWidget(makeStateGroup(), 1);
        main_layout->addWidget(makeLogGroup());
        setCentralWidget(central);
    }

    void applyStyle()
    {
        setStyleSheet(R"(
            QMainWindow, QWidget {
                background: #101820;
                color: #edf4f2;
                font-size: 14px;
            }
            QGroupBox {
                background: #16242d;
                border: 1px solid #28414c;
                border-radius: 10px;
                margin-top: 16px;
                padding: 14px 12px 12px 12px;
                font-weight: 600;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 14px;
                padding: 0 8px;
                color: #7ee2c3;
            }
            QLabel {
                color: #d7e6e2;
            }
            QLabel#hintLabel {
                color: #9fb8b1;
                background: #12202a;
                border: 1px solid #263f49;
                border-radius: 8px;
                padding: 8px 10px;
            }
            QComboBox, QSpinBox, QDoubleSpinBox {
                background: #0f1b23;
                color: #f4fbf9;
                border: 1px solid #36525f;
                border-radius: 7px;
                padding: 5px 8px;
                min-height: 26px;
            }
            QComboBox::drop-down {
                border: 0px;
                width: 22px;
            }
            QPushButton {
                background: #1e8a75;
                color: #ffffff;
                border: 0px;
                border-radius: 8px;
                padding: 8px 14px;
                font-weight: 600;
            }
            QPushButton:hover {
                background: #27a58c;
            }
            QPushButton:pressed {
                background: #176b5c;
            }
            QTableWidget {
                background: #0f1b23;
                alternate-background-color: #13232b;
                gridline-color: #2b4651;
                border: 1px solid #28414c;
                border-radius: 8px;
            }
            QHeaderView::section {
                background: #20333d;
                color: #cdeae2;
                border: 0px;
                padding: 7px;
                font-weight: 600;
            }
            QPlainTextEdit {
                background: #0d171e;
                color: #cdeae2;
                border: 1px solid #28414c;
                border-radius: 8px;
                padding: 8px;
                font-family: "Monospace";
            }
        )");
    }

    QWidget *makeCommandGroup()
    {
        auto *group = new QGroupBox("控制对象与快捷指令");
        auto *layout = new QGridLayout(group);

        platform_combo_ = new QComboBox();
        platform_combo_->addItem("UAV", static_cast<int>(Platform::UAV));
        platform_combo_->addItem("UGV", static_cast<int>(Platform::UGV));
        connect(platform_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int) {
            onPlatformChanged();
        });

        target_id_spin_ = new QSpinBox();
        target_id_spin_->setRange(1, 99);
        target_id_spin_->setValue(99);
        target_id_spin_->setToolTip("99 表示广播到全部智能体");

        auto *takeoff_button = new QPushButton("UAV 起飞");
        auto *land_button = new QPushButton("UAV 降落");
        hold_button_ = new QPushButton("悬停 / HOLD");
        auto *return_button = new QPushButton("返航");

        connect(takeoff_button, &QPushButton::clicked, this, [this]() { publishUavSimple(sunray_msgs::UAVSwarmCMD::SWARM_TAKEOFF); });
        connect(land_button, &QPushButton::clicked, this, [this]() { publishUavSimple(sunray_msgs::UAVSwarmCMD::SWARM_LAND); });
        connect(hold_button_, &QPushButton::clicked, this, [this]() { publishHold(); });
        connect(return_button, &QPushButton::clicked, this, [this]() { publishReturn(); });

        layout->addWidget(new QLabel("平台"), 0, 0);
        layout->addWidget(platform_combo_, 0, 1);
        layout->addWidget(new QLabel("目标ID"), 0, 2);
        layout->addWidget(target_id_spin_, 0, 3);
        layout->addWidget(takeoff_button, 1, 0);
        layout->addWidget(land_button, 1, 1);
        layout->addWidget(hold_button_, 1, 2);
        layout->addWidget(return_button, 1, 3);
        layout->setColumnStretch(4, 1);
        return group;
    }

    QWidget *makeFormationGroup()
    {
        auto *group = new QGroupBox("阵型指令");
        auto *main_layout = new QVBoxLayout(group);
        main_layout->setSpacing(12);

        formation_combo_ = new QComboBox();
        formation_combo_->addItem("STATIC_KEEP_FORMATION", sunray_msgs::Formation::STATIC_KEEP_FORMATION);
        formation_combo_->addItem("STATIC_FORMATION_LINE", sunray_msgs::Formation::STATIC_FORMATION_LINE);
        formation_combo_->addItem("STATIC_FORMATION_POLYGON", sunray_msgs::Formation::STATIC_FORMATION_POLYGON);
        formation_combo_->addItem("STATIC_FORMATION_RANDOM", sunray_msgs::Formation::STATIC_FORMATION_RANDOM);
        formation_combo_->addItem("DYNAMIC_FORMATION_RING", sunray_msgs::Formation::DYNAMIC_FORMATION_RING);
        formation_combo_->addItem("DYNAMIC_FORMATION_POLYGON", sunray_msgs::Formation::DYNAMIC_FORMATION_POLYGON);
        formation_combo_->addItem("DYNAMIC_FORMATION_LEMNISCATE", sunray_msgs::Formation::DYNAMIC_FORMATION_LEMNISCATE);
        connect(formation_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int) {
            onFormationTypeChanged();
        });

        leader_x_spin_ = makeDoubleSpin(0.0, -100.0, 100.0, 0.1);
        leader_y_spin_ = makeDoubleSpin(0.0, -100.0, 100.0, 0.1);
        leader_z_spin_ = makeDoubleSpin(1.5, -10.0, 20.0, 0.1);
        leader_yaw_spin_ = makeDoubleSpin(0.0, -3.1416, 3.1416, 0.05, 3);
        dynamic_time_spin_ = makeDoubleSpin(10.0, 0.1, 300.0, 1.0);

        static_line_spacing_spin_ = makeDoubleSpin(1.5, 0.1, 20.0, 0.1);
        static_line_angle_spin_ = makeDoubleSpin(0.0, -180.0, 180.0, 5.0);
        static_polygon_spacing_spin_ = makeDoubleSpin(2.0, 0.1, 20.0, 0.1);
        dynamic_ring_radius_spin_ = makeDoubleSpin(2.0, 0.1, 30.0, 0.1);
        dynamic_ring_speed_spin_ = makeDoubleSpin(0.5, -5.0, 5.0, 0.1);
        dynamic_polygon_spacing_spin_ = makeDoubleSpin(2.0, 0.1, 20.0, 0.1);
        dynamic_polygon_speed_spin_ = makeDoubleSpin(0.5, -5.0, 5.0, 0.1);
        dynamic_lemniscate_x_radius_spin_ = makeDoubleSpin(3.0, 0.1, 30.0, 0.1);
        dynamic_lemniscate_y_radius_spin_ = makeDoubleSpin(2.0, 0.1, 30.0, 0.1);
        dynamic_lemniscate_speed_spin_ = makeDoubleSpin(0.5, -5.0, 5.0, 0.1);

        auto *send_button = new QPushButton("发送阵型指令");
        connect(send_button, &QPushButton::clicked, this, [this]() { publishFormation(); });

        auto *top_layout = new QHBoxLayout();
        top_layout->addWidget(new QLabel("阵型类型"));
        top_layout->addWidget(formation_combo_, 1);
        top_layout->addWidget(send_button);
        main_layout->addLayout(top_layout);

        auto *leader_group = new QGroupBox("虚拟 Leader");
        auto *leader_layout = new QGridLayout(leader_group);
        leader_layout->setHorizontalSpacing(10);
        leader_layout->addWidget(new QLabel("X / m"), 0, 0);
        leader_layout->addWidget(leader_x_spin_, 0, 1);
        leader_layout->addWidget(new QLabel("Y / m"), 0, 2);
        leader_layout->addWidget(leader_y_spin_, 0, 3);
        leader_layout->addWidget(new QLabel("Z / m"), 0, 4);
        leader_layout->addWidget(leader_z_spin_, 0, 5);
        leader_layout->addWidget(new QLabel("Yaw / rad"), 0, 6);
        leader_layout->addWidget(leader_yaw_spin_, 0, 7);
        main_layout->addWidget(leader_group);

        dynamic_time_group_ = new QGroupBox("动态阵型时间");
        auto *dynamic_time_layout = new QFormLayout(dynamic_time_group_);
        dynamic_time_layout->addRow("持续时间 / s", dynamic_time_spin_);
        dynamic_time_spin_->setToolTip("动态阵型持续时间，单位：秒");
        main_layout->addWidget(dynamic_time_group_);

        formation_param_stack_ = new QStackedWidget();
        formation_param_stack_->addWidget(makeFormationParamPage(
            "保持当前阵型：抓拍各智能体当前相对位置，只需要设置虚拟 Leader 位置。", {}));
        formation_param_stack_->addWidget(makeFormationParamPage(
            "静态直线阵型：所有智能体沿指定角度排成一条线。",
            {{"相邻间距 / m", static_line_spacing_spin_}, {"直线角度 / deg", static_line_angle_spin_}}));
        formation_param_stack_->addWidget(makeFormationParamPage(
            "静态多边形阵型：根据集群数量生成正 N 边形。",
            {{"边长 / m", static_polygon_spacing_spin_}}));
        formation_param_stack_->addWidget(makeFormationParamPage(
            "静态随机阵型：目标点由 formation 模块在安全区域内生成，只需要设置虚拟 Leader。", {}));
        formation_param_stack_->addWidget(makeFormationParamPage(
            "动态圆环阵型：智能体在圆周上连续运动。",
            {{"圆半径 / m", dynamic_ring_radius_spin_}, {"移动速度 / m/s", dynamic_ring_speed_spin_}}));
        formation_param_stack_->addWidget(makeFormationParamPage(
            "动态多边形阵型：智能体沿正 N 边形边线运动。",
            {{"边长 / m", dynamic_polygon_spacing_spin_}, {"移动速度 / m/s", dynamic_polygon_speed_spin_}}));
        formation_param_stack_->addWidget(makeFormationParamPage(
            "动态 8 字阵型：智能体沿 Lemniscate 轨迹运动。",
            {{"X 方向尺度 / m", dynamic_lemniscate_x_radius_spin_},
             {"Y 方向尺度 / m", dynamic_lemniscate_y_radius_spin_},
             {"移动速度 / m/s", dynamic_lemniscate_speed_spin_}}));
        main_layout->addWidget(formation_param_stack_);
        onFormationTypeChanged();

        return group;
    }

    QWidget *makeFormationParamPage(const QString &hint,
                                    const std::vector<std::pair<QString, QWidget *>> &fields)
    {
        auto *page = new QWidget();
        auto *layout = new QFormLayout(page);
        layout->setContentsMargins(4, 4, 4, 4);
        layout->setSpacing(10);

        auto *hint_label = new QLabel(hint);
        hint_label->setObjectName("hintLabel");
        hint_label->setWordWrap(true);
        layout->addRow(hint_label);

        for (const auto &field : fields)
        {
            layout->addRow(field.first, field.second);
        }

        return page;
    }

    QWidget *makeStateGroup()
    {
        auto *group = new QGroupBox("集群状态");
        auto *layout = new QVBoxLayout(group);
        state_table_ = new QTableWidget(0, 8);
        state_table_->setHorizontalHeaderLabels(
            {"平台", "ID", "FSM", "自机ODOM", "邻居ODOM", "目标点", "当前位置", "更新时间"});
        state_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        state_table_->verticalHeader()->setVisible(false);
        state_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
        state_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
        layout->addWidget(state_table_);
        return group;
    }

    QWidget *makeLogGroup()
    {
        auto *group = new QGroupBox("操作日志");
        auto *layout = new QVBoxLayout(group);
        log_view_ = new QPlainTextEdit();
        log_view_->setReadOnly(true);
        log_view_->setMaximumBlockCount(200);
        layout->addWidget(log_view_);
        return group;
    }

    Platform currentPlatform() const
    {
        return static_cast<Platform>(platform_combo_->currentData().toInt());
    }

    int targetId() const
    {
        return target_id_spin_->value();
    }

    sunray_msgs::Formation makeFormation() const
    {
        sunray_msgs::Formation cmd;
        cmd.formation_type = static_cast<uint8_t>(formation_combo_->currentData().toInt());
        cmd.leader_pos.x = leader_x_spin_->value();
        cmd.leader_pos.y = leader_y_spin_->value();
        cmd.leader_pos.z = leader_z_spin_->value();
        cmd.leader_yaw = static_cast<float>(leader_yaw_spin_->value());
        cmd.dynamic_time = static_cast<float>(dynamic_time_spin_->value());
        cmd.static_line_spacing = static_cast<float>(static_line_spacing_spin_->value());
        cmd.static_line_angle = static_cast<float>(static_line_angle_spin_->value());
        cmd.static_polygon_spacing = static_cast<float>(static_polygon_spacing_spin_->value());
        cmd.dynamic_ring_radius = static_cast<float>(dynamic_ring_radius_spin_->value());
        cmd.dynamic_ring_move_speed = static_cast<float>(dynamic_ring_speed_spin_->value());
        cmd.dynamic_polygon_spacing = static_cast<float>(dynamic_polygon_spacing_spin_->value());
        cmd.dynamic_polygon_move_speed = static_cast<float>(dynamic_polygon_speed_spin_->value());
        cmd.dynamic_lemniscate_x_radius = static_cast<float>(dynamic_lemniscate_x_radius_spin_->value());
        cmd.dynamic_lemniscate_y_radius = static_cast<float>(dynamic_lemniscate_y_radius_spin_->value());
        cmd.dynamic_lemniscate_move_speed = static_cast<float>(dynamic_lemniscate_speed_spin_->value());
        return cmd;
    }

    void publishUavSimple(const uint8_t swarm_cmd)
    {
        sunray_msgs::UAVSwarmCMD msg;
        msg.header.stamp = ros::Time::now();
        msg.cmd_source = sunray_msgs::UAVSwarmCMD::GROUND_STATION;
        msg.agent_id = static_cast<uint8_t>(targetId());
        msg.swarm_cmd = swarm_cmd;
        uav_cmd_pub_.publish(msg);
        appendLog(QString("发布 UAV 指令: cmd=%1 target=%2").arg(swarm_cmd).arg(targetId()));
    }

    void publishUgvSimple(const uint8_t swarm_cmd)
    {
        sunray_msgs::UGVSwarmCMD msg;
        msg.header.stamp = ros::Time::now();
        msg.cmd_source = sunray_msgs::UGVSwarmCMD::GROUND_STATION;
        msg.agent_id = static_cast<uint8_t>(targetId());
        msg.swarm_cmd = swarm_cmd;
        ugv_cmd_pub_.publish(msg);
        appendLog(QString("发布 UGV 指令: cmd=%1 target=%2").arg(swarm_cmd).arg(targetId()));
    }

    void publishHold()
    {
        if (currentPlatform() == Platform::UAV)
        {
            publishUavSimple(sunray_msgs::UAVSwarmCMD::SWARM_HOVER);
        }
        else
        {
            publishUgvSimple(sunray_msgs::UGVSwarmCMD::SWARM_HOLD);
        }
    }

    void publishReturn()
    {
        if (currentPlatform() == Platform::UAV)
        {
            publishUavSimple(sunray_msgs::UAVSwarmCMD::SWARM_RETURN);
        }
        else
        {
            publishUgvSimple(sunray_msgs::UGVSwarmCMD::SWARM_RETURN);
        }
    }

    void publishFormation()
    {
        sunray_msgs::Formation formation = makeFormation();
        formation.header.stamp = ros::Time::now();

        if (currentPlatform() == Platform::UAV)
        {
            sunray_msgs::UAVSwarmCMD msg;
            msg.header.stamp = ros::Time::now();
            msg.cmd_source = sunray_msgs::UAVSwarmCMD::GROUND_STATION;
            msg.agent_id = static_cast<uint8_t>(targetId());
            msg.swarm_cmd = sunray_msgs::UAVSwarmCMD::SWARM_FORMATION;
            msg.formation_cmd = formation;
            uav_cmd_pub_.publish(msg);
            appendLog(QString("发布 UAV 阵型: %1 target=%2").arg(formationName(formation.formation_type)).arg(targetId()));
        }
        else
        {
            sunray_msgs::UGVSwarmCMD msg;
            msg.header.stamp = ros::Time::now();
            msg.cmd_source = sunray_msgs::UGVSwarmCMD::GROUND_STATION;
            msg.agent_id = static_cast<uint8_t>(targetId());
            msg.swarm_cmd = sunray_msgs::UGVSwarmCMD::SWARM_FORMATION;
            msg.formation_cmd = formation;
            ugv_cmd_pub_.publish(msg);
            appendLog(QString("发布 UGV 阵型: %1 target=%2").arg(formationName(formation.formation_type)).arg(targetId()));
        }
    }

    void onPlatformChanged()
    {
        if (currentPlatform() == Platform::UAV)
        {
            hold_button_->setText("悬停");
            leader_z_spin_->setValue(1.5);
        }
        else
        {
            hold_button_->setText("HOLD");
            leader_z_spin_->setValue(0.0);
        }
    }

    void onFormationTypeChanged()
    {
        if (formation_param_stack_ != nullptr)
        {
            formation_param_stack_->setCurrentIndex(formation_combo_->currentIndex());
        }

        const uint8_t formation_type = static_cast<uint8_t>(formation_combo_->currentData().toInt());
        const bool is_dynamic = formation_type == sunray_msgs::Formation::DYNAMIC_FORMATION_RING ||
                                formation_type == sunray_msgs::Formation::DYNAMIC_FORMATION_POLYGON ||
                                formation_type == sunray_msgs::Formation::DYNAMIC_FORMATION_LEMNISCATE;
        if (dynamic_time_group_ != nullptr)
        {
            dynamic_time_group_->setVisible(is_dynamic);
        }
    }

    void appendLog(const QString &text)
    {
        log_view_->appendPlainText(QString("[%1] %2")
                                       .arg(QString::number(ros::Time::now().toSec(), 'f', 2))
                                       .arg(text));
        SUNRAY_INFO("{}", text.toStdString());
    }

    void uavStateCallback(const sunray_msgs::UAVSwarmState::ConstPtr &msg)
    {
        CachedState state;
        state.platform = "UAV";
        state.agent_id = msg->agent_id;
        state.swarm_num = static_cast<int>(msg->swarm_num);
        state.self_odom_ready = msg->self_odom_ready;
        state.peers_odom_ready = msg->peers_odom_ready;
        state.ready_peer_num = msg->ready_peer_num;
        state.fsm_state = msg->fsm_state;
        state.target_valid = msg->target_valid;
        state.target_pos = msg->target_pos;
        state.target_yaw = msg->target_yaw;
        state.receive_time = ros::Time::now();

        std::lock_guard<std::mutex> lock(state_mutex_);
        states_["UAV" + std::to_string(state.agent_id)] = state;
        current_pose_text_["UAV" + std::to_string(state.agent_id)] =
            poseSummary(msg->self_odom_ready, msg->self_odom);
    }

    void ugvStateCallback(const sunray_msgs::UGVSwarmState::ConstPtr &msg)
    {
        CachedState state;
        state.platform = "UGV";
        state.agent_id = msg->agent_id;
        state.swarm_num = static_cast<int>(msg->swarm_num);
        state.self_odom_ready = msg->self_odom_ready;
        state.peers_odom_ready = msg->peers_odom_ready;
        state.ready_peer_num = msg->ready_peer_num;
        state.fsm_state = msg->fsm_state;
        state.target_valid = msg->target_valid;
        state.target_pos = msg->target_pos;
        state.target_yaw = msg->target_yaw;
        state.receive_time = ros::Time::now();

        std::lock_guard<std::mutex> lock(state_mutex_);
        states_["UGV" + std::to_string(state.agent_id)] = state;
        current_pose_text_["UGV" + std::to_string(state.agent_id)] =
            poseSummary(msg->self_odom_ready, msg->self_odom);
    }

    static QString poseSummary(const bool ready, const nav_msgs::Odometry &odom)
    {
        if (!ready)
        {
            return "无";
        }

        const geometry_msgs::Point &p = odom.pose.pose.position;
        return QString("x=%1 y=%2 z=%3 yaw=%4deg")
            .arg(formatDouble(p.x))
            .arg(formatDouble(p.y))
            .arg(formatDouble(p.z))
            .arg(formatDouble(yawFromOdom(odom) * 180.0 / M_PI));
    }

    QString targetSummary(const CachedState &state) const
    {
        if (!state.target_valid)
        {
            return "无";
        }

        return QString("(%1,%2,%3) yaw=%4deg")
            .arg(formatDouble(state.target_pos.x))
            .arg(formatDouble(state.target_pos.y))
            .arg(formatDouble(state.target_pos.z))
            .arg(formatDouble(state.target_yaw * 180.0 / M_PI));
    }

    void refreshStateTable()
    {
        std::map<std::string, CachedState> states;
        std::map<std::string, QString> poses;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            states = states_;
            poses = current_pose_text_;
        }

        const ros::Time now = ros::Time::now();
        state_table_->setRowCount(static_cast<int>(states.size()));

        int row = 0;
        for (const auto &item : states)
        {
            const CachedState &state = item.second;
            const double age = (now - state.receive_time).toSec();
            const bool online = age <= state_timeout_;
            const QString fsm_name = state.platform == "UAV" ? uavStateName(state.fsm_state) : ugvStateName(state.fsm_state);

            setTableItem(row, 0, QString::fromStdString(state.platform), online);
            setTableItem(row, 1, QString::number(state.agent_id), online);
            setTableItem(row, 2, fsm_name, online);
            setTableItem(row, 3, state.self_odom_ready ? "OK" : "BAD", state.self_odom_ready && online);
            setTableItem(row, 4,
                         state.peers_odom_ready ? "OK" : QString("BAD(%1)").arg(state.ready_peer_num),
                         state.peers_odom_ready && online);
            setTableItem(row, 5, targetSummary(state), state.target_valid && online);
            setTableItem(row, 6, poses[item.first], state.self_odom_ready && online);
            setTableItem(row, 7, QString("%1s").arg(formatDouble(age)), online);
            ++row;
        }
    }

    void setTableItem(const int row, const int col, const QString &text, const bool ok)
    {
        auto *item = new QTableWidgetItem(text);
        item->setTextAlignment(Qt::AlignCenter);
        item->setBackground(ok ? QColor(34, 70, 46) : QColor(88, 42, 42));
        item->setForeground(QColor(235, 235, 235));
        state_table_->setItem(row, col, item);
    }

    ros::NodeHandle nh_;
    ros::Publisher uav_cmd_pub_;
    ros::Publisher ugv_cmd_pub_;
    ros::Subscriber uav_state_sub_;
    ros::Subscriber ugv_state_sub_;

    std::string uav_swarm_cmd_topic_;
    std::string ugv_swarm_cmd_topic_;
    std::string uav_swarm_state_topic_;
    std::string ugv_swarm_state_topic_;
    double state_timeout_{1.0};

    QComboBox *platform_combo_{nullptr};
    QSpinBox *target_id_spin_{nullptr};
    QPushButton *hold_button_{nullptr};
    QComboBox *formation_combo_{nullptr};
    QDoubleSpinBox *leader_x_spin_{nullptr};
    QDoubleSpinBox *leader_y_spin_{nullptr};
    QDoubleSpinBox *leader_z_spin_{nullptr};
    QDoubleSpinBox *leader_yaw_spin_{nullptr};
    QDoubleSpinBox *dynamic_time_spin_{nullptr};
    QWidget *dynamic_time_group_{nullptr};
    QStackedWidget *formation_param_stack_{nullptr};
    QDoubleSpinBox *static_line_spacing_spin_{nullptr};
    QDoubleSpinBox *static_line_angle_spin_{nullptr};
    QDoubleSpinBox *static_polygon_spacing_spin_{nullptr};
    QDoubleSpinBox *dynamic_ring_radius_spin_{nullptr};
    QDoubleSpinBox *dynamic_ring_speed_spin_{nullptr};
    QDoubleSpinBox *dynamic_polygon_spacing_spin_{nullptr};
    QDoubleSpinBox *dynamic_polygon_speed_spin_{nullptr};
    QDoubleSpinBox *dynamic_lemniscate_x_radius_spin_{nullptr};
    QDoubleSpinBox *dynamic_lemniscate_y_radius_spin_{nullptr};
    QDoubleSpinBox *dynamic_lemniscate_speed_spin_{nullptr};
    QTableWidget *state_table_{nullptr};
    QPlainTextEdit *log_view_{nullptr};
    QTimer *refresh_timer_{nullptr};

    std::mutex state_mutex_;
    std::map<std::string, CachedState> states_;
    std::map<std::string, QString> current_pose_text_;
};

} // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "swarm_control_panel_node");
    QApplication app(argc, argv);
    ros::NodeHandle nh("~");

    ros::AsyncSpinner spinner(2);
    spinner.start();

    SwarmControlPanel panel(nh);
    panel.show();

    const int ret = app.exec();
    ros::shutdown();
    return ret;
}
