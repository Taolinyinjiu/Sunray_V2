/*
本程序功能：
    1、Sunray 综合地面站：集中显示 UAV/UGV 控制、定位、规划、集群状态
    2、内嵌 RViz 三维态势视图，并发布 /sunray/monitor/markers
    3、支持向 UAV/UGV 单机控制器和 swarm 控制器发布常用指令
*/
#include <ros/ros.h>
#include <ros/package.h>
#include <topic_tools/shape_shifter.h>
#include <XmlRpcValue.h>

#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/ColorRGBA.h>
#include <sunray_msgs/Formation.h>
#include <sunray_msgs/OdomState.h>
#include <sunray_msgs/UAVControlCMD.h>
#include <sunray_msgs/UAVControlState.h>
#include <sunray_msgs/UAVPlanningState.h>
#include <sunray_msgs/UAVSwarmCMD.h>
#include <sunray_msgs/UAVSwarmState.h>
#include <sunray_msgs/UGVControlCMD.h>
#include <sunray_msgs/UGVControlState.h>
#include <sunray_msgs/UGVSwarmCMD.h>
#include <sunray_msgs/UGVSwarmState.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <QAbstractItemView>
#include <QApplication>
#include <QColor>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileInfo>
#include <QFormLayout>
#include <QGridLayout>
#include <QIcon>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QMainWindow>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSpinBox>
#include <QSplitter>
#include <QStackedWidget>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTabWidget>
#include <QTimer>
#include <QPixmap>
#include <QVBoxLayout>
#include <QWidget>

#include <rviz/display.h>
#include <rviz/render_panel.h>
#include <rviz/visualization_manager.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace
{

constexpr double kDegToRad = 0.017453292519943295;
constexpr double kRadToDeg = 57.29577951308232;
constexpr int kContinuousCommandIntervalMs = 200;

enum class Platform
{
    UAV = 0,
    UGV = 1
};

struct TopicConfig
{
    std::string name;
    std::string module;
    std::string agent_type;
    std::string agent_name;
    int agent_id{0};
    std::string topic;
};

struct TopicRuntime
{
    TopicConfig config;
    ros::Subscriber sub;
    ros::Time last_receive_time;
    ros::Time first_receive_time;
    uint64_t message_count{0};
    std::string datatype;
    std::string md5sum;
};

struct AgentConfig
{
    Platform platform{Platform::UAV};
    int id{1};
    std::string name;
    std::string ns;
    std::string control_state_topic;
    std::string control_cmd_topic;
    std::string odom_state_topic;
    std::string planning_state_topic;
};

struct AgentRuntime
{
    AgentConfig config;

    ros::Subscriber uav_control_sub;
    ros::Subscriber ugv_control_sub;
    ros::Subscriber odom_state_sub;
    ros::Subscriber planning_state_sub;

    bool has_uav_control{false};
    bool has_ugv_control{false};
    bool has_odom_state{false};
    bool has_planning_state{false};
    bool has_uav_swarm_state{false};
    bool has_ugv_swarm_state{false};

    sunray_msgs::UAVControlState uav_control_state;
    sunray_msgs::UGVControlState ugv_control_state;
    sunray_msgs::OdomState odom_state;
    sunray_msgs::UAVPlanningState planning_state;
    sunray_msgs::UAVSwarmState uav_swarm_state;
    sunray_msgs::UGVSwarmState ugv_swarm_state;

    ros::Time last_control_time;
    ros::Time last_odom_time;
    ros::Time last_planning_time;
    ros::Time last_swarm_time;
};

std::string xmlRpcString(const XmlRpc::XmlRpcValue &value, const std::string &fallback = "")
{
    return value.getType() == XmlRpc::XmlRpcValue::TypeString ? static_cast<std::string>(value) : fallback;
}

int xmlRpcInt(const XmlRpc::XmlRpcValue &value, const int fallback = 0)
{
    if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
        return static_cast<int>(value);
    }
    return fallback;
}

QString formatDouble(const double value, const int precision = 2)
{
    if (!std::isfinite(value))
    {
        return "nan";
    }
    return QString::number(value, 'f', precision);
}

QString secondsText(const double value)
{
    if (value < 0.0 || !std::isfinite(value))
    {
        return "--";
    }
    return QString("%1 s").arg(value, 0, 'f', 2);
}

QString hzText(const double value)
{
    if (value <= 0.0 || !std::isfinite(value))
    {
        return "--";
    }
    return QString("%1 Hz").arg(value, 0, 'f', 1);
}

QString boolText(const bool value)
{
    return value ? "OK" : "NO";
}

QString platformText(const Platform platform)
{
    return platform == Platform::UAV ? "UAV" : "UGV";
}

QTableWidgetItem *makeItem(const QString &text)
{
    auto *item = new QTableWidgetItem(text);
    item->setFlags(item->flags() & ~Qt::ItemIsEditable);
    return item;
}

void setItemText(QTableWidget *table, const int row, const int column, const QString &text)
{
    if (table == nullptr)
    {
        return;
    }
    QTableWidgetItem *item = table->item(row, column);
    if (item == nullptr)
    {
        item = makeItem(text);
        table->setItem(row, column, item);
    }
    else
    {
        item->setText(text);
    }
}

void setRowColor(QTableWidget *table, const int row, const QColor &background, const QColor &foreground)
{
    if (table == nullptr)
    {
        return;
    }
    for (int column = 0; column < table->columnCount(); ++column)
    {
        QTableWidgetItem *item = table->item(row, column);
        if (item != nullptr)
        {
            item->setBackground(background);
            item->setForeground(foreground);
        }
    }
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

double yawFromOdom(const nav_msgs::Odometry &odom)
{
    const geometry_msgs::Quaternion &q = odom.pose.pose.orientation;
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

std_msgs::ColorRGBA makeColor(const double r, const double g, const double b, const double a)
{
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}

QString pointText(const geometry_msgs::Point &point, const int precision = 2)
{
    return QString("(%1, %2, %3)")
        .arg(formatDouble(point.x, precision))
        .arg(formatDouble(point.y, precision))
        .arg(formatDouble(point.z, precision));
}

QString vectorText(const geometry_msgs::Vector3 &vec, const int precision = 2)
{
    return QString("(%1, %2, %3)")
        .arg(formatDouble(vec.x, precision))
        .arg(formatDouble(vec.y, precision))
        .arg(formatDouble(vec.z, precision));
}

QString poseText(const nav_msgs::Odometry &odom)
{
    const geometry_msgs::Point &pos = odom.pose.pose.position;
    return QString("x=%1 y=%2 z=%3 yaw=%4deg")
        .arg(formatDouble(pos.x))
        .arg(formatDouble(pos.y))
        .arg(formatDouble(pos.z))
        .arg(formatDouble(yawFromOdom(odom) * kRadToDeg, 1));
}

QString twistText(const nav_msgs::Odometry &odom)
{
    const geometry_msgs::Vector3 &linear = odom.twist.twist.linear;
    return QString("vx=%1 vy=%2 vz=%3")
        .arg(formatDouble(linear.x))
        .arg(formatDouble(linear.y))
        .arg(formatDouble(linear.z));
}

QString uavControlStateName(const uint8_t state)
{
    switch (state)
    {
    case sunray_msgs::UAVControlState::OFF:
        return "OFF";
    case sunray_msgs::UAVControlState::INIT:
        return "INIT";
    case sunray_msgs::UAVControlState::TAKEOFF:
        return "TAKEOFF";
    case sunray_msgs::UAVControlState::HOVER:
        return "HOVER";
    case sunray_msgs::UAVControlState::RETURN:
        return "RETURN";
    case sunray_msgs::UAVControlState::LAND:
        return "LAND";
    case sunray_msgs::UAVControlState::MOVE:
        return "MOVE";
    case sunray_msgs::UAVControlState::EMERGENCY_KILL:
        return "KILL";
    default:
        return "UNKNOWN";
    }
}

QString ugvControlStateName(const uint8_t state)
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

QString uavCmdName(const uint8_t cmd)
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

QString ugvCmdName(const uint8_t cmd)
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

QString planningStateName(const uint8_t state)
{
    switch (state)
    {
    case sunray_msgs::UAVPlanningState::OFF:
        return "OFF";
    case sunray_msgs::UAVPlanningState::INIT:
        return "INIT";
    case sunray_msgs::UAVPlanningState::TAKEOFF:
        return "TAKEOFF";
    case sunray_msgs::UAVPlanningState::LAND:
        return "LAND";
    case sunray_msgs::UAVPlanningState::PLANNING:
        return "PLANNING";
    case sunray_msgs::UAVPlanningState::ARRIVED:
        return "ARRIVED";
    case sunray_msgs::UAVPlanningState::PLAN_FAILED:
        return "PLAN_FAILED";
    default:
        return "UNKNOWN";
    }
}

QString uavSwarmStateName(const uint8_t state)
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

QString ugvSwarmStateName(const uint8_t state)
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

QString formationName(const uint8_t formation_type)
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

}  // namespace

class SunrayMonitorPanel : public QMainWindow
{
  public:
    explicit SunrayMonitorPanel(ros::NodeHandle &nh, QWidget *parent = nullptr)
        : QMainWindow(parent), nh_(nh)
    {
        loadParams();
        setupRos();
        setupUi();
        setupRviz();
        setupWindowBranding();
        applyStyle();

        refresh_timer_ = new QTimer(this);
        connect(refresh_timer_, &QTimer::timeout, this, [this]() {
            ros::spinOnce();
            refreshUi();
            publishMarkers();
            if (!ros::ok())
            {
                close();
                qApp->quit();
            }
        });
        refresh_timer_->start(static_cast<int>(1000.0 / std::max(1.0, refresh_hz_)));

        command_publish_timer_ = new QTimer(this);
        connect(command_publish_timer_, &QTimer::timeout, this, [this]() { publishActiveCommand(); });
        command_publish_timer_->start(kContinuousCommandIntervalMs);

        setWindowTitle("Sunray Ground Station");
        resize(1540, 920);
        appendLog("Sunray 综合地面站启动");
    }

    ~SunrayMonitorPanel() override
    {
        if (rviz_manager_ != nullptr)
        {
            delete rviz_manager_;
            rviz_manager_ = nullptr;
        }
    }

  private:
    void loadParams()
    {
        nh_.param("fixed_frame", fixed_frame_, std::string("world"));
        nh_.param("marker_topic", marker_topic_, std::string("/sunray/monitor/markers"));
        nh_.param("status_timeout", status_timeout_, 1.0);
        nh_.param("refresh_hz", refresh_hz_, 5.0);
        nh_.param("uav_name_prefix", uav_name_prefix_, std::string("uav"));
        nh_.param("ugv_name_prefix", ugv_name_prefix_, std::string("ugv"));
        nh_.param("uav_swarm_cmd_topic", uav_swarm_cmd_topic_, std::string("/sunray/swarm/uav_swarm_cmd"));
        nh_.param("ugv_swarm_cmd_topic", ugv_swarm_cmd_topic_, std::string("/sunray/swarm/ugv_swarm_cmd"));
        nh_.param("uav_swarm_state_topic", uav_swarm_state_topic_, std::string("/sunray/swarm/uav_swarm_state"));
        nh_.param("ugv_swarm_state_topic", ugv_swarm_state_topic_, std::string("/sunray/swarm/ugv_swarm_state"));

        loadAgentList("uav_agents", Platform::UAV, uav_name_prefix_, std::vector<int>{1});
        loadAgentList("ugv_agents", Platform::UGV, ugv_name_prefix_, std::vector<int>{1});
        loadStateTopics();
    }

    void loadAgentList(const std::string &param_name,
                       const Platform platform,
                       const std::string &prefix,
                       const std::vector<int> &fallback_ids)
    {
        XmlRpc::XmlRpcValue agent_list;
        if (!nh_.getParam(param_name, agent_list) ||
            agent_list.getType() != XmlRpc::XmlRpcValue::TypeArray ||
            agent_list.size() == 0)
        {
            for (size_t i = 0; i < fallback_ids.size(); ++i)
            {
                AgentRuntime runtime;
                runtime.config = makeDefaultAgentConfig(platform, fallback_ids[i], prefix);
                agents_.push_back(runtime);
            }
            return;
        }

        for (int i = 0; i < agent_list.size(); ++i)
        {
            AgentRuntime runtime;
            if (agent_list[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
            {
                runtime.config = makeDefaultAgentConfig(platform, static_cast<int>(agent_list[i]), prefix);
            }
            else if (agent_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct)
            {
                const int id = agent_list[i].hasMember("id") ? xmlRpcInt(agent_list[i]["id"], i + 1) : i + 1;
                runtime.config = makeDefaultAgentConfig(platform, id, prefix);
                if (agent_list[i].hasMember("name"))
                {
                    runtime.config.name = xmlRpcString(agent_list[i]["name"], runtime.config.name);
                }
                if (agent_list[i].hasMember("namespace"))
                {
                    runtime.config.ns = xmlRpcString(agent_list[i]["namespace"], runtime.config.ns);
                }
                if (agent_list[i].hasMember("control_state_topic"))
                {
                    runtime.config.control_state_topic =
                        xmlRpcString(agent_list[i]["control_state_topic"], runtime.config.control_state_topic);
                }
                if (agent_list[i].hasMember("control_cmd_topic"))
                {
                    runtime.config.control_cmd_topic =
                        xmlRpcString(agent_list[i]["control_cmd_topic"], runtime.config.control_cmd_topic);
                }
                if (agent_list[i].hasMember("odom_state_topic"))
                {
                    runtime.config.odom_state_topic =
                        xmlRpcString(agent_list[i]["odom_state_topic"], runtime.config.odom_state_topic);
                }
                if (agent_list[i].hasMember("planning_state_topic"))
                {
                    runtime.config.planning_state_topic =
                        xmlRpcString(agent_list[i]["planning_state_topic"], runtime.config.planning_state_topic);
                }
            }
            else
            {
                continue;
            }

            agents_.push_back(runtime);
        }
    }

    AgentConfig makeDefaultAgentConfig(const Platform platform, const int id, const std::string &prefix) const
    {
        AgentConfig config;
        config.platform = platform;
        config.id = id;
        config.name = prefix + std::to_string(id);
        config.ns = "/" + config.name;
        if (platform == Platform::UAV)
        {
            config.control_state_topic = config.ns + "/sunray/uav_control/control_state";
            config.control_cmd_topic = config.ns + "/sunray/uav_control/control_cmd";
            config.odom_state_topic = config.ns + "/sunray/localization/odom_state";
            config.planning_state_topic = config.ns + "/sunray/planning/planning_state";
        }
        else
        {
            config.control_state_topic = config.ns + "/sunray/ugv_control/control_state";
            config.control_cmd_topic = config.ns + "/sunray/ugv_control/control_cmd";
            config.odom_state_topic = config.ns + "/sunray/localization/odom_state";
        }
        return config;
    }

    void loadStateTopics()
    {
        XmlRpc::XmlRpcValue topic_list;
        if (!nh_.getParam("state_topics", topic_list) ||
            topic_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_WARN("sunray_monitor_panel_node: no valid ~state_topics config.");
            return;
        }

        for (int i = 0; i < topic_list.size(); ++i)
        {
            XmlRpc::XmlRpcValue entry = topic_list[i];
            if (entry.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
                !entry.hasMember("topic"))
            {
                continue;
            }

            TopicConfig config;
            config.name = xmlRpcString(entry["name"], "state_" + std::to_string(i));
            config.module = xmlRpcString(entry["module"], "UNKNOWN");
            config.agent_type = xmlRpcString(entry["agent_type"], "UNKNOWN");
            config.agent_name = xmlRpcString(entry["agent_name"], "");
            config.agent_id = entry.hasMember("agent_id") ? xmlRpcInt(entry["agent_id"], 0) : 0;
            config.topic = xmlRpcString(entry["topic"], "");
            if (config.topic.empty())
            {
                continue;
            }

            TopicRuntime runtime;
            runtime.config = config;
            topics_.push_back(runtime);
        }
    }

    void setupRos()
    {
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 2);
        uav_swarm_cmd_pub_ = nh_.advertise<sunray_msgs::UAVSwarmCMD>(uav_swarm_cmd_topic_, 10);
        ugv_swarm_cmd_pub_ = nh_.advertise<sunray_msgs::UGVSwarmCMD>(ugv_swarm_cmd_topic_, 10);
        uav_swarm_state_sub_ =
            nh_.subscribe(uav_swarm_state_topic_, 100, &SunrayMonitorPanel::uavSwarmStateCallback, this);
        ugv_swarm_state_sub_ =
            nh_.subscribe(ugv_swarm_state_topic_, 100, &SunrayMonitorPanel::ugvSwarmStateCallback, this);

        for (size_t i = 0; i < agents_.size(); ++i)
        {
            AgentRuntime &agent = agents_[i];
            if (agent.config.platform == Platform::UAV)
            {
                agent.uav_control_sub = nh_.subscribe<sunray_msgs::UAVControlState>(
                    agent.config.control_state_topic, 10,
                    [this, i](const sunray_msgs::UAVControlState::ConstPtr &msg) {
                        uavControlStateCallback(msg, i);
                    });
                agent.planning_state_sub = nh_.subscribe<sunray_msgs::UAVPlanningState>(
                    agent.config.planning_state_topic, 10,
                    [this, i](const sunray_msgs::UAVPlanningState::ConstPtr &msg) {
                        planningStateCallback(msg, i);
                    });
                uav_control_pubs_[agent.config.id] =
                    nh_.advertise<sunray_msgs::UAVControlCMD>(agent.config.control_cmd_topic, 10);
            }
            else
            {
                agent.ugv_control_sub = nh_.subscribe<sunray_msgs::UGVControlState>(
                    agent.config.control_state_topic, 10,
                    [this, i](const sunray_msgs::UGVControlState::ConstPtr &msg) {
                        ugvControlStateCallback(msg, i);
                    });
                ugv_control_pubs_[agent.config.id] =
                    nh_.advertise<sunray_msgs::UGVControlCMD>(agent.config.control_cmd_topic, 10);
            }

            agent.odom_state_sub = nh_.subscribe<sunray_msgs::OdomState>(
                agent.config.odom_state_topic, 10,
                [this, i](const sunray_msgs::OdomState::ConstPtr &msg) {
                    odomStateCallback(msg, i);
                });

            ROS_INFO("sunray_monitor_panel subscribe: %s", agent.config.control_state_topic.c_str());
            ROS_INFO("sunray_monitor_panel subscribe: %s", agent.config.odom_state_topic.c_str());
        }

        for (size_t i = 0; i < topics_.size(); ++i)
        {
            topics_[i].sub = nh_.subscribe<topic_tools::ShapeShifter>(
                topics_[i].config.topic, 20,
                [this, i](const topic_tools::ShapeShifter::ConstPtr &msg) {
                    topicStateCallback(msg, i);
                });
            ROS_INFO("sunray_monitor_panel topic health subscribe: %s", topics_[i].config.topic.c_str());
        }
    }

    void setupUi()
    {
        auto *central = new QWidget(this);
        auto *root = new QVBoxLayout(central);
        root->setContentsMargins(12, 10, 12, 12);
        root->setSpacing(8);

        auto *header_card = new QWidget();
        header_card->setObjectName("headerCard");
        auto *header_layout = new QHBoxLayout(header_card);
        header_layout->setContentsMargins(12, 8, 12, 8);
        header_layout->setSpacing(14);

        logo_label_ = new QLabel();
        logo_label_->setObjectName("logoLabel");
        logo_label_->setMinimumWidth(210);
        logo_label_->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
        header_layout->addWidget(logo_label_);

        auto *title = new QLabel("Sunray 综合地面站");
        title->setObjectName("titleLabel");
        header_layout->addWidget(title);
        header_layout->addStretch(1);
        summary_label_ = new QLabel("等待状态...");
        summary_label_->setObjectName("summaryLabel");
        header_layout->addWidget(summary_label_);
        root->addWidget(header_card);

        auto *splitter = new QSplitter(Qt::Horizontal, central);
        left_tabs_ = new QTabWidget(splitter);
        left_tabs_->addTab(buildOverviewTab(), "概览");
        left_tabs_->addTab(buildControlTab(), "控制");
        left_tabs_->addTab(buildDetailsTab(), "详情");
        splitter->addWidget(left_tabs_);
        splitter->addWidget(buildRvizWidget());
        splitter->setStretchFactor(0, 4);
        splitter->setStretchFactor(1, 6);
        splitter->setSizes(QList<int>() << 610 << 920);
        root->addWidget(splitter, 1);

        setCentralWidget(central);
    }

    QWidget *buildOverviewTab()
    {
        auto *page = new QWidget();
        auto *layout = new QVBoxLayout(page);
        layout->setContentsMargins(10, 10, 10, 10);
        layout->setSpacing(8);

        overview_label_ = new QLabel("等待 UAV/UGV 状态...");
        overview_label_->setObjectName("sectionLabel");
        layout->addWidget(overview_label_);

        agent_table_ = new QTableWidget(0, 10);
        agent_table_->setHorizontalHeaderLabels(QStringList()
                                                << "状态"
                                                << "平台"
                                                << "ID"
                                                << "控制"
                                                << "定位"
                                                << "位置"
                                                << "速度"
                                                << "目标"
                                                << "最近指令"
                                                << "更新");
        agent_table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
        agent_table_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
        agent_table_->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
        agent_table_->horizontalHeader()->setSectionResizeMode(3, QHeaderView::ResizeToContents);
        agent_table_->horizontalHeader()->setSectionResizeMode(4, QHeaderView::ResizeToContents);
        agent_table_->horizontalHeader()->setSectionResizeMode(5, QHeaderView::Stretch);
        agent_table_->horizontalHeader()->setSectionResizeMode(6, QHeaderView::ResizeToContents);
        agent_table_->horizontalHeader()->setSectionResizeMode(7, QHeaderView::ResizeToContents);
        agent_table_->horizontalHeader()->setSectionResizeMode(8, QHeaderView::ResizeToContents);
        agent_table_->horizontalHeader()->setSectionResizeMode(9, QHeaderView::ResizeToContents);
        agent_table_->verticalHeader()->setVisible(false);
        agent_table_->setAlternatingRowColors(true);
        agent_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
        agent_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
        layout->addWidget(agent_table_, 1);

        return page;
    }

    QWidget *buildControlTab()
    {
        auto *tabs = new QTabWidget();
        tabs->addTab(buildUavControlPage(), "无人机控制");
        tabs->addTab(buildUgvControlPage(), "无人车控制");
        tabs->addTab(buildClusterControlPage(), "集群控制");
        return tabs;
    }

    QWidget *buildUavControlPage()
    {
        return buildControlPage({buildUavQuickGroup(), buildUavMotionGroup()});
    }

    QWidget *buildUgvControlPage()
    {
        return buildControlPage({buildUgvQuickGroup(), buildUgvMotionGroup()});
    }

    QWidget *buildClusterControlPage()
    {
        return buildControlPage({buildUavSwarmGroup(), buildUgvSwarmGroup(), buildFormationGroup()});
    }

    QWidget *buildControlPage(const std::vector<QWidget *> &groups)
    {
        auto *scroll = new QScrollArea();
        scroll->setWidgetResizable(true);
        auto *content = new QWidget();
        auto *layout = new QVBoxLayout(content);
        layout->setContentsMargins(10, 10, 10, 10);
        layout->setSpacing(10);
        for (QWidget *group : groups)
        {
            layout->addWidget(group);
        }
        layout->addStretch(1);
        scroll->setWidget(content);
        return scroll;
    }

    QWidget *buildUavQuickGroup()
    {
        auto *group = new QGroupBox("UAV 快捷指令");
        auto *layout = new QGridLayout(group);
        layout->setHorizontalSpacing(8);
        layout->setVerticalSpacing(8);

        uav_id_spin_ = new QSpinBox();
        uav_id_spin_->setRange(1, 99);
        uav_id_spin_->setValue(1);

        auto *takeoff_btn = new QPushButton("起飞");
        auto *hover_btn = new QPushButton("悬停/HOLD");
        auto *land_btn = new QPushButton("降落");
        auto *return_btn = new QPushButton("返航");
        auto *kill_btn = new QPushButton("急停");
        kill_btn->setObjectName("dangerButton");

        connect(takeoff_btn, &QPushButton::clicked, this, [this]() { publishSingleTakeoff(); });
        connect(hover_btn, &QPushButton::clicked, this, [this]() { publishSingleHold(); });
        connect(land_btn, &QPushButton::clicked, this, [this]() { publishSingleLand(); });
        connect(return_btn, &QPushButton::clicked, this, [this]() { publishSingleReturn(); });
        connect(kill_btn, &QPushButton::clicked, this, [this]() { publishSingleKill(); });

        layout->addWidget(new QLabel("UAV ID"), 0, 0);
        layout->addWidget(uav_id_spin_, 0, 1);
        layout->addWidget(takeoff_btn, 1, 0);
        layout->addWidget(hover_btn, 1, 1);
        layout->addWidget(land_btn, 1, 2);
        layout->addWidget(return_btn, 1, 3);
        layout->addWidget(kill_btn, 1, 4);
        layout->setColumnStretch(5, 1);
        return group;
    }

    QWidget *buildUavMotionGroup()
    {
        auto *group = new QGroupBox("UAV 位置/速度");
        auto *layout = new QVBoxLayout(group);
        layout->setSpacing(8);

        auto *point_layout = new QGridLayout();
        uav_point_x_spin_ = makeSpin(0.0, -200.0, 200.0, 0.1);
        uav_point_y_spin_ = makeSpin(0.0, -200.0, 200.0, 0.1);
        uav_point_z_spin_ = makeSpin(1.2, -20.0, 50.0, 0.1);
        uav_point_yaw_spin_ = makeSpin(0.0, -180.0, 180.0, 5.0, 1);
        auto *point_btn = new QPushButton("发送位置点");
        connect(point_btn, &QPushButton::clicked, this, [this]() { publishMovePoint(); });

        point_layout->addWidget(new QLabel("X/m"), 0, 0);
        point_layout->addWidget(uav_point_x_spin_, 0, 1);
        point_layout->addWidget(new QLabel("Y/m"), 0, 2);
        point_layout->addWidget(uav_point_y_spin_, 0, 3);
        point_layout->addWidget(new QLabel("Z/m"), 1, 0);
        point_layout->addWidget(uav_point_z_spin_, 1, 1);
        point_layout->addWidget(new QLabel("Yaw/deg"), 1, 2);
        point_layout->addWidget(uav_point_yaw_spin_, 1, 3);
        point_layout->addWidget(point_btn, 0, 4, 2, 1);
        layout->addLayout(point_layout);

        auto *world_vel_layout = new QGridLayout();
        uav_world_vx_spin_ = makeSpin(0.0, -10.0, 10.0, 0.1);
        uav_world_vy_spin_ = makeSpin(0.0, -10.0, 10.0, 0.1);
        uav_world_vz_spin_ = makeSpin(0.0, -5.0, 5.0, 0.1);
        uav_world_height_spin_ = makeSpin(1.2, -1.0, 50.0, 0.1);
        uav_world_yaw_spin_ = makeSpin(0.0, -180.0, 180.0, 5.0, 1);
        auto *world_vel_btn = new QPushButton("世界速度 5Hz");
        connect(world_vel_btn, &QPushButton::clicked, this, [this]() { startWorldVelocity(); });

        world_vel_layout->addWidget(new QLabel("Vx"), 0, 0);
        world_vel_layout->addWidget(uav_world_vx_spin_, 0, 1);
        world_vel_layout->addWidget(new QLabel("Vy"), 0, 2);
        world_vel_layout->addWidget(uav_world_vy_spin_, 0, 3);
        world_vel_layout->addWidget(new QLabel("Vz"), 1, 0);
        world_vel_layout->addWidget(uav_world_vz_spin_, 1, 1);
        world_vel_layout->addWidget(new QLabel("H"), 1, 2);
        world_vel_layout->addWidget(uav_world_height_spin_, 1, 3);
        world_vel_layout->addWidget(new QLabel("Yaw/deg"), 2, 0);
        world_vel_layout->addWidget(uav_world_yaw_spin_, 2, 1);
        world_vel_layout->addWidget(world_vel_btn, 0, 4, 3, 1);
        layout->addLayout(world_vel_layout);

        auto *body_vel_layout = new QGridLayout();
        uav_body_vx_spin_ = makeSpin(0.0, -10.0, 10.0, 0.1);
        uav_body_vy_spin_ = makeSpin(0.0, -10.0, 10.0, 0.1);
        uav_body_height_spin_ = makeSpin(1.2, -1.0, 50.0, 0.1);
        uav_body_yaw_rate_spin_ = makeSpin(0.0, -180.0, 180.0, 5.0, 1);
        auto *body_vel_btn = new QPushButton("机体速度 5Hz");
        auto *stop_btn = new QPushButton("停止速度");
        stop_btn->setObjectName("warnButton");
        connect(body_vel_btn, &QPushButton::clicked, this, [this]() { startBodyVelocity(); });
        connect(stop_btn, &QPushButton::clicked, this, [this]() { stopContinuousCommand(); });

        body_vel_layout->addWidget(new QLabel("Body Vx"), 0, 0);
        body_vel_layout->addWidget(uav_body_vx_spin_, 0, 1);
        body_vel_layout->addWidget(new QLabel("Body Vy"), 0, 2);
        body_vel_layout->addWidget(uav_body_vy_spin_, 0, 3);
        body_vel_layout->addWidget(new QLabel("H"), 1, 0);
        body_vel_layout->addWidget(uav_body_height_spin_, 1, 1);
        body_vel_layout->addWidget(new QLabel("YawRate/deg"), 1, 2);
        body_vel_layout->addWidget(uav_body_yaw_rate_spin_, 1, 3);
        body_vel_layout->addWidget(body_vel_btn, 0, 4);
        body_vel_layout->addWidget(stop_btn, 1, 4);
        layout->addLayout(body_vel_layout);

        return group;
    }

    QWidget *buildUgvQuickGroup()
    {
        auto *group = new QGroupBox("UGV 快捷指令");
        auto *layout = new QGridLayout(group);
        layout->setHorizontalSpacing(8);
        layout->setVerticalSpacing(8);

        ugv_id_spin_ = new QSpinBox();
        ugv_id_spin_->setRange(1, 99);
        ugv_id_spin_->setValue(1);

        auto *hold_btn = new QPushButton("HOLD");
        auto *return_btn = new QPushButton("返航");
        connect(hold_btn, &QPushButton::clicked, this, [this]() { publishUgvHold(); });
        connect(return_btn, &QPushButton::clicked, this, [this]() { publishUgvReturn(); });

        layout->addWidget(new QLabel("UGV ID"), 0, 0);
        layout->addWidget(ugv_id_spin_, 0, 1);
        layout->addWidget(hold_btn, 1, 0);
        layout->addWidget(return_btn, 1, 1);
        layout->setColumnStretch(2, 1);
        return group;
    }

    QWidget *buildUgvMotionGroup()
    {
        auto *group = new QGroupBox("UGV 位置/速度");
        auto *layout = new QVBoxLayout(group);
        layout->setSpacing(8);

        auto *point_layout = new QGridLayout();
        ugv_point_x_spin_ = makeSpin(0.0, -200.0, 200.0, 0.1);
        ugv_point_y_spin_ = makeSpin(0.0, -200.0, 200.0, 0.1);
        ugv_point_yaw_spin_ = makeSpin(0.0, -180.0, 180.0, 5.0, 1);
        auto *point_btn = new QPushButton("发送位置点");
        connect(point_btn, &QPushButton::clicked, this, [this]() { publishUgvMovePoint(); });

        point_layout->addWidget(new QLabel("X/m"), 0, 0);
        point_layout->addWidget(ugv_point_x_spin_, 0, 1);
        point_layout->addWidget(new QLabel("Y/m"), 0, 2);
        point_layout->addWidget(ugv_point_y_spin_, 0, 3);
        point_layout->addWidget(new QLabel("Yaw/deg"), 1, 0);
        point_layout->addWidget(ugv_point_yaw_spin_, 1, 1);
        point_layout->addWidget(point_btn, 0, 4, 2, 1);
        layout->addLayout(point_layout);

        auto *world_vel_layout = new QGridLayout();
        ugv_world_vx_spin_ = makeSpin(0.0, -10.0, 10.0, 0.1);
        ugv_world_vy_spin_ = makeSpin(0.0, -10.0, 10.0, 0.1);
        ugv_world_yaw_spin_ = makeSpin(0.0, -180.0, 180.0, 5.0, 1);
        auto *world_vel_btn = new QPushButton("世界速度 5Hz");
        connect(world_vel_btn, &QPushButton::clicked, this, [this]() { startUgvWorldVelocity(); });

        world_vel_layout->addWidget(new QLabel("Vx"), 0, 0);
        world_vel_layout->addWidget(ugv_world_vx_spin_, 0, 1);
        world_vel_layout->addWidget(new QLabel("Vy"), 0, 2);
        world_vel_layout->addWidget(ugv_world_vy_spin_, 0, 3);
        world_vel_layout->addWidget(new QLabel("Yaw/deg"), 1, 0);
        world_vel_layout->addWidget(ugv_world_yaw_spin_, 1, 1);
        world_vel_layout->addWidget(world_vel_btn, 0, 4, 2, 1);
        layout->addLayout(world_vel_layout);

        auto *body_vel_layout = new QGridLayout();
        ugv_body_vx_spin_ = makeSpin(0.0, -10.0, 10.0, 0.1);
        ugv_body_vy_spin_ = makeSpin(0.0, -10.0, 10.0, 0.1);
        ugv_body_yaw_rate_spin_ = makeSpin(0.0, -180.0, 180.0, 5.0, 1);
        auto *body_vel_btn = new QPushButton("机体速度 5Hz");
        auto *stop_btn = new QPushButton("停止速度");
        stop_btn->setObjectName("warnButton");
        connect(body_vel_btn, &QPushButton::clicked, this, [this]() { startUgvBodyVelocity(); });
        connect(stop_btn, &QPushButton::clicked, this, [this]() { stopContinuousCommand(); });

        body_vel_layout->addWidget(new QLabel("Body Vx"), 0, 0);
        body_vel_layout->addWidget(ugv_body_vx_spin_, 0, 1);
        body_vel_layout->addWidget(new QLabel("Body Vy"), 0, 2);
        body_vel_layout->addWidget(ugv_body_vy_spin_, 0, 3);
        body_vel_layout->addWidget(new QLabel("YawRate/deg"), 1, 0);
        body_vel_layout->addWidget(ugv_body_yaw_rate_spin_, 1, 1);
        body_vel_layout->addWidget(body_vel_btn, 0, 4);
        body_vel_layout->addWidget(stop_btn, 1, 4);
        layout->addLayout(body_vel_layout);

        return group;
    }

    QWidget *buildUavSwarmGroup()
    {
        auto *group = new QGroupBox("UAV 集群");
        auto *layout = new QGridLayout(group);
        layout->setHorizontalSpacing(8);
        layout->setVerticalSpacing(8);

        uav_swarm_target_id_spin_ = new QSpinBox();
        uav_swarm_target_id_spin_->setRange(1, 99);
        uav_swarm_target_id_spin_->setValue(99);

        auto *swarm_takeoff_btn = new QPushButton("起飞");
        auto *swarm_hold_btn = new QPushButton("悬停/HOVER");
        auto *swarm_land_btn = new QPushButton("降落");
        auto *swarm_return_btn = new QPushButton("返航");
        connect(swarm_takeoff_btn, &QPushButton::clicked, this, [this]() { publishUavSwarmTakeoff(); });
        connect(swarm_hold_btn, &QPushButton::clicked, this, [this]() { publishUavSwarmHold(); });
        connect(swarm_land_btn, &QPushButton::clicked, this, [this]() { publishUavSwarmLand(); });
        connect(swarm_return_btn, &QPushButton::clicked, this, [this]() { publishUavSwarmReturn(); });

        layout->addWidget(new QLabel("目标ID"), 0, 0);
        layout->addWidget(uav_swarm_target_id_spin_, 0, 1);
        layout->addWidget(swarm_takeoff_btn, 1, 0);
        layout->addWidget(swarm_hold_btn, 1, 1);
        layout->addWidget(swarm_land_btn, 1, 2);
        layout->addWidget(swarm_return_btn, 1, 3);
        layout->setColumnStretch(4, 1);
        return group;
    }

    QWidget *buildUgvSwarmGroup()
    {
        auto *group = new QGroupBox("UGV 集群");
        auto *layout = new QGridLayout(group);
        layout->setHorizontalSpacing(8);
        layout->setVerticalSpacing(8);

        ugv_swarm_target_id_spin_ = new QSpinBox();
        ugv_swarm_target_id_spin_->setRange(1, 99);
        ugv_swarm_target_id_spin_->setValue(99);

        auto *swarm_hold_btn = new QPushButton("HOLD");
        auto *swarm_return_btn = new QPushButton("返航");
        connect(swarm_hold_btn, &QPushButton::clicked, this, [this]() { publishUgvSwarmHold(); });
        connect(swarm_return_btn, &QPushButton::clicked, this, [this]() { publishUgvSwarmReturn(); });

        layout->addWidget(new QLabel("目标ID"), 0, 0);
        layout->addWidget(ugv_swarm_target_id_spin_, 0, 1);
        layout->addWidget(swarm_hold_btn, 1, 0);
        layout->addWidget(swarm_return_btn, 1, 1);
        layout->setColumnStretch(2, 1);
        return group;
    }

    QWidget *buildFormationGroup()
    {
        auto *group = new QGroupBox("阵型指令");
        auto *layout = new QVBoxLayout(group);
        layout->setSpacing(8);

        auto *top_layout = new QHBoxLayout();
        formation_combo_ = new QComboBox();
        formation_combo_->addItem("STATIC_KEEP_FORMATION", sunray_msgs::Formation::STATIC_KEEP_FORMATION);
        formation_combo_->addItem("STATIC_FORMATION_LINE", sunray_msgs::Formation::STATIC_FORMATION_LINE);
        formation_combo_->addItem("STATIC_FORMATION_POLYGON", sunray_msgs::Formation::STATIC_FORMATION_POLYGON);
        formation_combo_->addItem("STATIC_FORMATION_RANDOM", sunray_msgs::Formation::STATIC_FORMATION_RANDOM);
        formation_combo_->addItem("DYNAMIC_FORMATION_RING", sunray_msgs::Formation::DYNAMIC_FORMATION_RING);
        formation_combo_->addItem("DYNAMIC_FORMATION_POLYGON", sunray_msgs::Formation::DYNAMIC_FORMATION_POLYGON);
        formation_combo_->addItem("DYNAMIC_FORMATION_LEMNISCATE", sunray_msgs::Formation::DYNAMIC_FORMATION_LEMNISCATE);
        connect(formation_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int) {
            onFormationChanged();
        });
        auto *uav_formation_btn = new QPushButton("发给 UAV");
        auto *ugv_formation_btn = new QPushButton("发给 UGV");
        connect(uav_formation_btn, &QPushButton::clicked, this, [this]() { publishUavFormation(); });
        connect(ugv_formation_btn, &QPushButton::clicked, this, [this]() { publishUgvFormation(); });
        top_layout->addWidget(new QLabel("阵型"));
        top_layout->addWidget(formation_combo_, 1);
        top_layout->addWidget(uav_formation_btn);
        top_layout->addWidget(ugv_formation_btn);
        layout->addLayout(top_layout);

        auto *leader_group = new QGroupBox("虚拟 Leader");
        auto *leader_layout = new QGridLayout(leader_group);
        formation_leader_x_spin_ = makeSpin(0.0, -200.0, 200.0, 0.1);
        formation_leader_y_spin_ = makeSpin(0.0, -200.0, 200.0, 0.1);
        formation_leader_z_spin_ = makeSpin(1.5, -20.0, 50.0, 0.1);
        formation_leader_yaw_spin_ = makeSpin(0.0, -180.0, 180.0, 5.0, 1);
        leader_layout->setHorizontalSpacing(8);
        leader_layout->setVerticalSpacing(8);
        leader_layout->addWidget(new QLabel("X leader_pos.x / m"), 0, 0);
        leader_layout->addWidget(formation_leader_x_spin_, 0, 1);
        leader_layout->addWidget(new QLabel("Y leader_pos.y / m"), 0, 2);
        leader_layout->addWidget(formation_leader_y_spin_, 0, 3);
        leader_layout->addWidget(new QLabel("Z leader_pos.z / m"), 1, 0);
        leader_layout->addWidget(formation_leader_z_spin_, 1, 1);
        leader_layout->addWidget(new QLabel("航向 leader_yaw / deg"), 1, 2);
        leader_layout->addWidget(formation_leader_yaw_spin_, 1, 3);
        leader_layout->setColumnStretch(4, 1);
        layout->addWidget(leader_group);

        formation_line_spacing_spin_ = makeSpin(2.0, 0.1, 30.0, 0.1);
        formation_line_angle_spin_ = makeSpin(0.0, -180.0, 180.0, 5.0, 1);
        formation_polygon_spacing_spin_ = makeSpin(2.0, 0.1, 30.0, 0.1);
        formation_dynamic_time_spin_ = makeSpin(20.0, 0.1, 600.0, 1.0, 1);
        formation_ring_radius_spin_ = makeSpin(3.0, 0.1, 50.0, 0.1);
        formation_ring_speed_spin_ = makeSpin(0.5, -10.0, 10.0, 0.1);
        formation_dynamic_polygon_spacing_spin_ = makeSpin(2.0, 0.1, 30.0, 0.1);
        formation_dynamic_polygon_speed_spin_ = makeSpin(0.5, -10.0, 10.0, 0.1);
        formation_lemniscate_x_spin_ = makeSpin(4.0, 0.1, 50.0, 0.1);
        formation_lemniscate_y_spin_ = makeSpin(2.5, 0.1, 50.0, 0.1);
        formation_lemniscate_speed_spin_ = makeSpin(0.5, -10.0, 10.0, 0.1);

        formation_dynamic_time_group_ = new QGroupBox("动态阵型时间");
        auto *time_layout = new QFormLayout(formation_dynamic_time_group_);
        time_layout->setContentsMargins(10, 10, 10, 10);
        time_layout->addRow("持续时间 dynamic_time / s", formation_dynamic_time_spin_);
        layout->addWidget(formation_dynamic_time_group_);

        formation_param_stack_ = new QStackedWidget();
        formation_param_stack_->addWidget(buildFormationParamPage(
            "当前相对队形 STATIC_KEEP_FORMATION",
            {}));
        formation_param_stack_->addWidget(buildFormationParamPage(
            "静态直线阵型 STATIC_FORMATION_LINE",
            {{"相邻间距 static_line_spacing / m", formation_line_spacing_spin_},
             {"直线角度 static_line_angle / deg", formation_line_angle_spin_}}));
        formation_param_stack_->addWidget(buildFormationParamPage(
            "静态多边形阵型 STATIC_FORMATION_POLYGON",
            {{"边长 static_polygon_spacing / m", formation_polygon_spacing_spin_}}));
        formation_param_stack_->addWidget(buildFormationParamPage(
            "静态随机阵型 STATIC_FORMATION_RANDOM",
            {}));
        formation_param_stack_->addWidget(buildFormationParamPage(
            "动态圆环阵型 DYNAMIC_FORMATION_RING",
            {{"圆半径 dynamic_ring_radius / m", formation_ring_radius_spin_},
             {"切向速度 dynamic_ring_move_speed / m/s", formation_ring_speed_spin_}}));
        formation_param_stack_->addWidget(buildFormationParamPage(
            "动态多边形阵型 DYNAMIC_FORMATION_POLYGON",
            {{"边长 dynamic_polygon_spacing / m", formation_dynamic_polygon_spacing_spin_},
             {"线速度 dynamic_polygon_move_speed / m/s", formation_dynamic_polygon_speed_spin_}}));
        formation_param_stack_->addWidget(buildFormationParamPage(
            "动态 8 字阵型 DYNAMIC_FORMATION_LEMNISCATE",
            {{"X 方向尺度 dynamic_lemniscate_x_radius / m", formation_lemniscate_x_spin_},
             {"Y 方向尺度 dynamic_lemniscate_y_radius / m", formation_lemniscate_y_spin_},
             {"等效线速度 dynamic_lemniscate_move_speed / m/s", formation_lemniscate_speed_spin_}}));
        layout->addWidget(formation_param_stack_);

        onFormationChanged();

        return group;
    }

    QWidget *buildFormationParamPage(const QString &title,
                                     const std::vector<std::pair<QString, QWidget *>> &fields)
    {
        auto *page = new QWidget();
        auto *layout = new QFormLayout(page);
        layout->setContentsMargins(4, 4, 4, 4);
        layout->setSpacing(8);

        auto *title_label = new QLabel(title);
        title_label->setObjectName("sectionLabel");
        layout->addRow(title_label);

        for (const auto &field : fields)
        {
            layout->addRow(field.first, field.second);
        }

        return page;
    }

    QWidget *buildDetailsTab()
    {
        auto *tabs = new QTabWidget();

        detail_text_ = new QPlainTextEdit();
        detail_text_->setReadOnly(true);
        detail_text_->setLineWrapMode(QPlainTextEdit::NoWrap);
        tabs->addTab(detail_text_, "状态字段");

        topic_table_ = new QTableWidget(0, 9);
        topic_table_->setHorizontalHeaderLabels(QStringList()
                                                << "状态"
                                                << "名称"
                                                << "模块"
                                                << "类型"
                                                << "ID"
                                                << "频率"
                                                << "延迟"
                                                << "消息类型"
                                                << "话题");
        topic_table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
        topic_table_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
        topic_table_->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
        topic_table_->horizontalHeader()->setSectionResizeMode(3, QHeaderView::ResizeToContents);
        topic_table_->horizontalHeader()->setSectionResizeMode(4, QHeaderView::ResizeToContents);
        topic_table_->horizontalHeader()->setSectionResizeMode(5, QHeaderView::ResizeToContents);
        topic_table_->horizontalHeader()->setSectionResizeMode(6, QHeaderView::ResizeToContents);
        topic_table_->horizontalHeader()->setSectionResizeMode(7, QHeaderView::ResizeToContents);
        topic_table_->horizontalHeader()->setSectionResizeMode(8, QHeaderView::Stretch);
        topic_table_->verticalHeader()->setVisible(false);
        topic_table_->setAlternatingRowColors(true);
        topic_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
        topic_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
        tabs->addTab(topic_table_, "话题健康");

        log_view_ = new QPlainTextEdit();
        log_view_->setReadOnly(true);
        log_view_->setMaximumBlockCount(300);
        tabs->addTab(log_view_, "操作日志");

        return tabs;
    }

    QWidget *buildRvizWidget()
    {
        auto *group = new QGroupBox("三维态势");
        auto *layout = new QVBoxLayout(group);
        layout->setContentsMargins(8, 14, 8, 8);
        layout->setSpacing(6);

        rviz_status_label_ = new QLabel(QString("FixedFrame: %1    Marker: %2")
                                            .arg(QString::fromStdString(fixed_frame_))
                                            .arg(QString::fromStdString(marker_topic_)));
        rviz_status_label_->setObjectName("sectionLabel");
        layout->addWidget(rviz_status_label_);

        rviz_panel_ = new rviz::RenderPanel(group);
        layout->addWidget(rviz_panel_, 1);
        return group;
    }

    void setupRviz()
    {
        rviz_manager_ = new rviz::VisualizationManager(rviz_panel_);
        rviz_panel_->initialize(rviz_manager_->getSceneManager(), rviz_manager_);
        rviz_manager_->initialize();
        rviz_manager_->startUpdate();
        rviz_manager_->setFixedFrame(QString::fromStdString(fixed_frame_));

        rviz::Display *grid = rviz_manager_->createDisplay("rviz/Grid", "Grid", true);
        if (grid != nullptr)
        {
            grid->subProp("Plane Cell Count")->setValue(24);
            grid->subProp("Cell Size")->setValue(1.0);
            grid->subProp("Color")->setValue(QColor(120, 130, 120));
            grid->subProp("Alpha")->setValue(0.55);
        }

        rviz::Display *tf = rviz_manager_->createDisplay("rviz/TF", "TF", true);
        if (tf != nullptr)
        {
            tf->subProp("Show Names")->setValue(true);
            tf->subProp("Marker Scale")->setValue(0.35);
        }

        rviz::Display *marker = rviz_manager_->createDisplay("rviz/MarkerArray", "MonitorMarkers", true);
        if (marker != nullptr)
        {
            marker->subProp("Marker Topic")->setValue(QString::fromStdString(marker_topic_));
        }
    }

    void setupWindowBranding()
    {
        const QString package_dir = QString::fromStdString(ros::package::getPath("simulation_tools"));
        const QString logo_path = package_dir + "/logo/yundrone_logo.png";
        const QString icon_path = package_dir + "/logo/yundrone_small_logo.jpeg";

        if (!package_dir.isEmpty() && QFileInfo::exists(icon_path))
        {
            setWindowIcon(QIcon(icon_path));
        }

        if (logo_label_ != nullptr)
        {
            if (!package_dir.isEmpty() && QFileInfo::exists(logo_path))
            {
                logo_label_->setPixmap(QPixmap(logo_path).scaledToHeight(56, Qt::SmoothTransformation));
            }
            else
            {
                logo_label_->setText("Yundrone");
            }
        }
    }

    void applyStyle()
    {
        setStyleSheet(R"(
            QMainWindow, QWidget {
                background: #eef2ed;
                color: #17211c;
                font-family: "Noto Sans CJK SC", "Microsoft YaHei", sans-serif;
                font-size: 13px;
            }
            QLabel#titleLabel {
                font-size: 23px;
                font-weight: 800;
                color: #13201a;
                padding: 2px 0 5px 0;
            }
            QLabel#summaryLabel, QLabel#sectionLabel {
                color: #4f6259;
                font-weight: 700;
            }
            QLabel#logoLabel {
                background: transparent;
            }
            QWidget#headerCard {
                background: #f8faf7;
                border: 1px solid #bac7bf;
                border-radius: 8px;
            }
            QGroupBox {
                background: #f8faf7;
                border: 1px solid #bac7bf;
                border-radius: 8px;
                margin-top: 12px;
                padding: 10px 8px 8px 8px;
                font-weight: 700;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 6px;
                color: #1f7a5b;
            }
            QTabWidget::pane {
                border: 1px solid #b9c7be;
                border-radius: 8px;
                background: #fbfcfa;
                top: -1px;
            }
            QTabBar::tab {
                background: #dfe8e2;
                color: #294239;
                padding: 8px 14px;
                border: 1px solid #b9c7be;
                border-bottom: 0;
                border-top-left-radius: 5px;
                border-top-right-radius: 5px;
            }
            QTabBar::tab:selected {
                background: #f8faf7;
                color: #15231d;
            }
            QScrollArea {
                border: 0;
            }
            QComboBox, QSpinBox, QDoubleSpinBox {
                background: #fbfcfa;
                color: #15211b;
                border: 1px solid #b9c7be;
                border-radius: 5px;
                padding: 4px 7px;
                min-height: 24px;
            }
            QPushButton {
                background: #1f7a5b;
                color: white;
                border: 0;
                border-radius: 5px;
                padding: 7px 10px;
                font-weight: 700;
            }
            QPushButton:hover {
                background: #28916c;
            }
            QPushButton:pressed {
                background: #18664b;
            }
            QPushButton#warnButton {
                background: #b9802e;
            }
            QPushButton#warnButton:hover {
                background: #d69537;
            }
            QPushButton#dangerButton {
                background: #bd4d45;
            }
            QPushButton#dangerButton:hover {
                background: #d85b52;
            }
            QTableWidget {
                background: #fbfcfa;
                alternate-background-color: #f0f4ef;
                gridline-color: #d6dfd8;
                border: 1px solid #b9c7be;
                border-radius: 8px;
                color: #18221c;
            }
            QHeaderView::section {
                background: #eef4ef;
                color: #2a4137;
                border: 0;
                border-bottom: 1px solid #b9c7be;
                padding: 6px;
                font-weight: 700;
            }
            QPlainTextEdit {
                background: #fbfcfa;
                color: #17211c;
                border: 1px solid #b9c7be;
                border-radius: 8px;
                padding: 8px;
                font-family: "JetBrains Mono", "DejaVu Sans Mono", monospace;
            }
            QSplitter::handle {
                background: #c8d3cc;
            }
        )");
    }

    void refreshUi()
    {
        refreshOverview();
        refreshTopicTable();
        refreshDetails();
    }

    void refreshOverview()
    {
        if (agent_table_ == nullptr)
        {
            return;
        }

        const ros::Time now = ros::Time::now();
        int online_count = 0;
        int odom_ok_count = 0;
        int uav_count = 0;
        int ugv_count = 0;

        agent_table_->setRowCount(static_cast<int>(agents_.size()));
        for (int row = 0; row < static_cast<int>(agents_.size()); ++row)
        {
            const AgentRuntime &agent = agents_[static_cast<size_t>(row)];
            if (agent.config.platform == Platform::UAV)
            {
                ++uav_count;
            }
            else
            {
                ++ugv_count;
            }

            const bool online = agentOnline(agent, now);
            const bool odom_ok = agentOdomOk(agent);
            if (online)
            {
                ++online_count;
            }
            if (odom_ok)
            {
                ++odom_ok_count;
            }

            setItemText(agent_table_, row, 0, online ? "在线" : "离线");
            setItemText(agent_table_, row, 1, platformText(agent.config.platform));
            setItemText(agent_table_, row, 2, QString::number(agent.config.id));
            setItemText(agent_table_, row, 3, controlStateText(agent));
            setItemText(agent_table_, row, 4, odom_ok ? "OK" : "NO");
            setItemText(agent_table_, row, 5, currentPoseText(agent));
            setItemText(agent_table_, row, 6, currentVelocityText(agent));
            setItemText(agent_table_, row, 7, targetText(agent));
            setItemText(agent_table_, row, 8, lastCommandText(agent));
            setItemText(agent_table_, row, 9, secondsText(agentAge(agent, now)));

            if (!online)
            {
                setRowColor(agent_table_, row, QColor(72, 37, 34), QColor(255, 210, 203));
            }
            else if (!odom_ok)
            {
                setRowColor(agent_table_, row, QColor(75, 58, 31), QColor(255, 226, 178));
            }
            else
            {
                setRowColor(agent_table_, row, QColor(32, 65, 49), QColor(217, 255, 235));
            }
        }

        const QString active_text = has_active_command_
                                        ? QString("持续指令: %1%2 %3")
                                              .arg(platformText(active_platform_))
                                              .arg(active_agent_id_)
                                              .arg(active_command_label_)
                                        : "持续指令: 无";
        const QString summary = QString("UAV %1  UGV %2  在线 %3/%4  定位 %5/%4  %6")
                                    .arg(uav_count)
                                    .arg(ugv_count)
                                    .arg(online_count)
                                    .arg(agents_.size())
                                    .arg(odom_ok_count)
                                    .arg(active_text);
        if (summary_label_ != nullptr)
        {
            summary_label_->setText(summary);
        }
        if (overview_label_ != nullptr)
        {
            overview_label_->setText(QString("核心状态  |  超时阈值 %1 s  |  FixedFrame %2")
                                         .arg(status_timeout_, 0, 'f', 2)
                                         .arg(QString::fromStdString(fixed_frame_)));
        }
    }

    void refreshTopicTable()
    {
        if (topic_table_ == nullptr)
        {
            return;
        }

        const ros::Time now = ros::Time::now();
        topic_table_->setRowCount(static_cast<int>(topics_.size()));
        for (int row = 0; row < static_cast<int>(topics_.size()); ++row)
        {
            const TopicRuntime &runtime = topics_[static_cast<size_t>(row)];
            const bool has_msg = runtime.message_count > 0 && !runtime.last_receive_time.isZero();
            const double age = has_msg ? (now - runtime.last_receive_time).toSec() : -1.0;
            const bool online = has_msg && age <= status_timeout_;
            const double duration = has_msg ? std::max(0.001, (now - runtime.first_receive_time).toSec()) : 0.0;
            const double hz = runtime.message_count > 1 ? static_cast<double>(runtime.message_count - 1) / duration : 0.0;
            const QString agent_id_text = runtime.config.agent_id > 0 ? QString::number(runtime.config.agent_id) : "-";

            setItemText(topic_table_, row, 0, online ? "在线" : "离线");
            setItemText(topic_table_, row, 1, QString::fromStdString(runtime.config.name));
            setItemText(topic_table_, row, 2, QString::fromStdString(runtime.config.module));
            setItemText(topic_table_, row, 3, QString::fromStdString(runtime.config.agent_type));
            setItemText(topic_table_, row, 4, agent_id_text);
            setItemText(topic_table_, row, 5, hzText(hz));
            setItemText(topic_table_, row, 6, secondsText(age));
            setItemText(topic_table_, row, 7, runtime.datatype.empty() ? "--" : QString::fromStdString(runtime.datatype));
            setItemText(topic_table_, row, 8, QString::fromStdString(runtime.config.topic));

            setRowColor(topic_table_,
                        row,
                        online ? QColor(32, 65, 49) : QColor(72, 37, 34),
                        online ? QColor(217, 255, 235) : QColor(255, 210, 203));
        }
    }

    void refreshDetails()
    {
        if (detail_text_ == nullptr)
        {
            return;
        }

        const ros::Time now = ros::Time::now();
        QString text;
        text += QString("Sunray Monitor\nfixed_frame=%1 marker_topic=%2 status_timeout=%3s\n\n")
                    .arg(QString::fromStdString(fixed_frame_))
                    .arg(QString::fromStdString(marker_topic_))
                    .arg(status_timeout_, 0, 'f', 2);

        for (size_t i = 0; i < agents_.size(); ++i)
        {
            const AgentRuntime &agent = agents_[i];
            text += QString("[%1%2] %3  age=%4\n")
                        .arg(platformText(agent.config.platform))
                        .arg(agent.config.id)
                        .arg(agentOnline(agent, now) ? "ONLINE" : "OFFLINE")
                        .arg(secondsText(agentAge(agent, now)));
            text += QString("  topics:\n");
            text += QString("    control_state: %1\n").arg(QString::fromStdString(agent.config.control_state_topic));
            text += QString("    control_cmd:   %1\n").arg(QString::fromStdString(agent.config.control_cmd_topic));
            text += QString("    odom_state:    %1\n").arg(QString::fromStdString(agent.config.odom_state_topic));
            if (agent.config.platform == Platform::UAV)
            {
                text += QString("    planning:      %1\n").arg(QString::fromStdString(agent.config.planning_state_topic));
            }

            text += QString("  control: %1\n").arg(controlDetailText(agent));
            text += QString("  odom:    %1\n").arg(odomDetailText(agent));
            text += QString("  swarm:   %1\n").arg(swarmDetailText(agent));
            if (agent.config.platform == Platform::UAV)
            {
                text += QString("  plan:    %1\n").arg(planningDetailText(agent));
            }
            text += "\n";
        }

        detail_text_->setPlainText(text);
    }

    bool agentOnline(const AgentRuntime &agent, const ros::Time &now) const
    {
        const double age = agentAge(agent, now);
        return age >= 0.0 && age <= status_timeout_;
    }

    double agentAge(const AgentRuntime &agent, const ros::Time &now) const
    {
        ros::Time latest;
        if (!agent.last_control_time.isZero())
        {
            latest = agent.last_control_time;
        }
        if (!agent.last_odom_time.isZero() && (latest.isZero() || agent.last_odom_time > latest))
        {
            latest = agent.last_odom_time;
        }
        if (!agent.last_swarm_time.isZero() && (latest.isZero() || agent.last_swarm_time > latest))
        {
            latest = agent.last_swarm_time;
        }
        if (!agent.last_planning_time.isZero() && (latest.isZero() || agent.last_planning_time > latest))
        {
            latest = agent.last_planning_time;
        }
        return latest.isZero() ? -1.0 : (now - latest).toSec();
    }

    bool agentOdomOk(const AgentRuntime &agent) const
    {
        nav_msgs::Odometry odom;
        return getAgentOdom(agent, odom);
    }

    bool getAgentOdom(const AgentRuntime &agent, nav_msgs::Odometry &odom) const
    {
        if (agent.config.platform == Platform::UGV && agent.has_ugv_control && agent.ugv_control_state.odom_valid)
        {
            odom = agent.ugv_control_state.self_odom;
            return true;
        }
        if (agent.has_odom_state && agent.odom_state.odometry_valid && !agent.odom_state.local_odom.header.stamp.isZero())
        {
            odom = agent.odom_state.local_odom;
            return true;
        }
        if (agent.config.platform == Platform::UAV && agent.has_uav_swarm_state && agent.uav_swarm_state.self_odom_ready)
        {
            odom = agent.uav_swarm_state.self_odom;
            return true;
        }
        if (agent.config.platform == Platform::UGV && agent.has_ugv_swarm_state && agent.ugv_swarm_state.self_odom_ready)
        {
            odom = agent.ugv_swarm_state.self_odom;
            return true;
        }
        return false;
    }

    QString currentPoseText(const AgentRuntime &agent) const
    {
        nav_msgs::Odometry odom;
        if (!getAgentOdom(agent, odom))
        {
            return "--";
        }
        return poseText(odom);
    }

    QString currentVelocityText(const AgentRuntime &agent) const
    {
        nav_msgs::Odometry odom;
        if (!getAgentOdom(agent, odom))
        {
            return "--";
        }
        return twistText(odom);
    }

    QString controlStateText(const AgentRuntime &agent) const
    {
        if (agent.config.platform == Platform::UAV)
        {
            if (!agent.has_uav_control)
            {
                return "--";
            }
            return uavControlStateName(agent.uav_control_state.control_state);
        }

        if (!agent.has_ugv_control)
        {
            return "--";
        }
        return ugvControlStateName(agent.ugv_control_state.fsm_state);
    }

    QString lastCommandText(const AgentRuntime &agent) const
    {
        if (agent.config.platform == Platform::UAV)
        {
            if (agent.has_uav_control)
            {
                return uavCmdName(agent.uav_control_state.last_cmd.control_cmd);
            }
            if (agent.has_uav_swarm_state)
            {
                return uavCmdName(agent.uav_swarm_state.uav_cmd.control_cmd);
            }
            return "--";
        }

        if (agent.has_ugv_control)
        {
            return ugvCmdName(agent.ugv_control_state.active_ugv_control_cmd.control_cmd);
        }
        if (agent.has_ugv_swarm_state)
        {
            return ugvCmdName(agent.ugv_swarm_state.ugv_cmd.control_cmd);
        }
        return "--";
    }

    bool getAgentTarget(const AgentRuntime &agent, geometry_msgs::Point &target) const
    {
        if (agent.config.platform == Platform::UGV && agent.has_ugv_control && agent.ugv_control_state.target_valid)
        {
            target = agent.ugv_control_state.target_pos;
            return true;
        }
        if (agent.config.platform == Platform::UAV && agent.has_uav_swarm_state && agent.uav_swarm_state.target_valid)
        {
            target = agent.uav_swarm_state.target_pos;
            return true;
        }
        if (agent.config.platform == Platform::UGV && agent.has_ugv_swarm_state && agent.ugv_swarm_state.target_valid)
        {
            target = agent.ugv_swarm_state.target_pos;
            return true;
        }
        if (agent.config.platform == Platform::UAV && agent.has_planning_state &&
            !agent.planning_state.planning_cmd.waypoints.empty())
        {
            target = agent.planning_state.planning_cmd.waypoints.back().position;
            return true;
        }
        if (agent.config.platform == Platform::UAV && agent.has_uav_control &&
            agent.uav_control_state.last_cmd.control_cmd == sunray_msgs::UAVControlCMD::MOVE_POINT)
        {
            target.x = agent.uav_control_state.last_cmd.desired_pos.x;
            target.y = agent.uav_control_state.last_cmd.desired_pos.y;
            target.z = agent.uav_control_state.last_cmd.desired_pos.z;
            return true;
        }
        return false;
    }

    QString targetText(const AgentRuntime &agent) const
    {
        geometry_msgs::Point target;
        if (!getAgentTarget(agent, target))
        {
            return "--";
        }
        return pointText(target);
    }

    QString controlDetailText(const AgentRuntime &agent) const
    {
        if (agent.config.platform == Platform::UAV)
        {
            if (!agent.has_uav_control)
            {
                return "no UAVControlState";
            }
            const sunray_msgs::UAVControlState &state = agent.uav_control_state;
            return QString("state=%1 last_cmd=%2 odom_lost=%3 odom_valid=%4 takeoff_h=%5 land_v=%6 home=%7")
                .arg(uavControlStateName(state.control_state))
                .arg(uavCmdName(state.last_cmd.control_cmd))
                .arg(boolText(state.odometry_lost))
                .arg(boolText(state.odometry_valid))
                .arg(formatDouble(state.takeoff_relative_height))
                .arg(formatDouble(state.land_max_velocity))
                .arg(vectorText(state.home_point));
        }

        if (!agent.has_ugv_control)
        {
            return "no UGVControlState";
        }
        const sunray_msgs::UGVControlState &state = agent.ugv_control_state;
        return QString("state=%1 cmd=%2 odom_valid=%3 cmd_valid=%4 fence=%5 target=%6 cmd_vel=(%7,%8,%9)")
            .arg(ugvControlStateName(state.fsm_state))
            .arg(ugvCmdName(state.active_ugv_control_cmd.control_cmd))
            .arg(boolText(state.odom_valid))
            .arg(boolText(state.control_cmd_valid))
            .arg(boolText(state.inside_geo_fence))
            .arg(state.target_valid ? pointText(state.target_pos) : "--")
            .arg(formatDouble(state.controller_cmd_vel.linear.x))
            .arg(formatDouble(state.controller_cmd_vel.linear.y))
            .arg(formatDouble(state.controller_cmd_vel.angular.z));
    }

    QString odomDetailText(const AgentRuntime &agent) const
    {
        if (!agent.has_odom_state)
        {
            return "no OdomState";
        }
        const sunray_msgs::OdomState &state = agent.odom_state;
        QString text = QString("valid=%1 hz=%2 source=%3 relocal=%4")
                           .arg(boolText(state.odometry_valid))
                           .arg(formatDouble(state.odometry_update_hz, 1))
                           .arg(state.external_source)
                           .arg(boolText(state.relocalization_valid));
        if (!state.local_odom.header.stamp.isZero())
        {
            text += QString(" local=[%1]").arg(poseText(state.local_odom));
        }
        if (!state.global_odom.header.stamp.isZero())
        {
            text += QString(" global=[%1]").arg(poseText(state.global_odom));
        }
        text += QString(" frame=(%1,%2,%3)")
                    .arg(QString::fromStdString(state.global_frame_name))
                    .arg(QString::fromStdString(state.local_frame_name))
                    .arg(QString::fromStdString(state.base_frame_name));
        return text;
    }

    QString swarmDetailText(const AgentRuntime &agent) const
    {
        if (agent.config.platform == Platform::UAV)
        {
            if (!agent.has_uav_swarm_state)
            {
                return "no UAVSwarmState";
            }
            const sunray_msgs::UAVSwarmState &state = agent.uav_swarm_state;
            return QString("state=%1 swarm_num=%2 self_odom=%3 peers=%4 ready_peer=%5 target=%6 cmd=%7")
                .arg(uavSwarmStateName(state.fsm_state))
                .arg(state.swarm_num)
                .arg(boolText(state.self_odom_ready))
                .arg(boolText(state.peers_odom_ready))
                .arg(state.ready_peer_num)
                .arg(state.target_valid ? pointText(state.target_pos) : "--")
                .arg(uavCmdName(state.uav_cmd.control_cmd));
        }

        if (!agent.has_ugv_swarm_state)
        {
            return "no UGVSwarmState";
        }
        const sunray_msgs::UGVSwarmState &state = agent.ugv_swarm_state;
        return QString("state=%1 swarm_num=%2 self_odom=%3 peers=%4 ready_peer=%5 target=%6 cmd=%7")
            .arg(ugvSwarmStateName(state.fsm_state))
            .arg(state.swarm_num)
            .arg(boolText(state.self_odom_ready))
            .arg(boolText(state.peers_odom_ready))
            .arg(state.ready_peer_num)
            .arg(state.target_valid ? pointText(state.target_pos) : "--")
            .arg(ugvCmdName(state.ugv_cmd.control_cmd));
    }

    QString planningDetailText(const AgentRuntime &agent) const
    {
        if (!agent.has_planning_state)
        {
            return "no UAVPlanningState";
        }
        const sunray_msgs::UAVPlanningState &state = agent.planning_state;
        QString text = QString("planner=%1 state=%2 cmd=%3 waypoints=%4 home=%5")
                           .arg(QString::fromStdString(state.planner_type_string))
                           .arg(planningStateName(state.sunray_planning_state))
                           .arg(state.planning_cmd.plan_cmd)
                           .arg(state.planning_cmd.waypoints.size())
                           .arg(vectorText(state.home_point));
        if (!state.planning_cmd.waypoints.empty())
        {
            text += QString(" last_goal=%1").arg(pointText(state.planning_cmd.waypoints.back().position));
        }
        return text;
    }

    std::string defaultControlCmdTopic(const Platform platform, const int id) const
    {
        const std::string prefix = platform == Platform::UAV ? uav_name_prefix_ : ugv_name_prefix_;
        if (platform == Platform::UAV)
        {
            return "/" + prefix + std::to_string(id) + "/sunray/uav_control/control_cmd";
        }
        return "/" + prefix + std::to_string(id) + "/sunray/ugv_control/control_cmd";
    }

    std::string configuredControlCmdTopic(const Platform platform, const int id) const
    {
        for (size_t i = 0; i < agents_.size(); ++i)
        {
            if (agents_[i].config.platform == platform && agents_[i].config.id == id)
            {
                return agents_[i].config.control_cmd_topic;
            }
        }
        return defaultControlCmdTopic(platform, id);
    }

    ros::Publisher &uavControlPublisher(const int id)
    {
        std::map<int, ros::Publisher>::iterator it = uav_control_pubs_.find(id);
        if (it == uav_control_pubs_.end())
        {
            it = uav_control_pubs_
                     .insert(std::make_pair(id, nh_.advertise<sunray_msgs::UAVControlCMD>(
                                                    configuredControlCmdTopic(Platform::UAV, id), 10)))
                     .first;
        }
        return it->second;
    }

    ros::Publisher &ugvControlPublisher(const int id)
    {
        std::map<int, ros::Publisher>::iterator it = ugv_control_pubs_.find(id);
        if (it == ugv_control_pubs_.end())
        {
            it = ugv_control_pubs_
                     .insert(std::make_pair(id, nh_.advertise<sunray_msgs::UGVControlCMD>(
                                                    configuredControlCmdTopic(Platform::UGV, id), 10)))
                     .first;
        }
        return it->second;
    }

    sunray_msgs::UAVControlCMD makeUavBaseCommand(const uint8_t command) const
    {
        sunray_msgs::UAVControlCMD cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.cmd_source = sunray_msgs::UAVControlCMD::SUNRAY_STATION;
        cmd.control_cmd = command;
        cmd.yaw_mode = sunray_msgs::UAVControlCMD::SET_YAW;
        return cmd;
    }

    sunray_msgs::UGVControlCMD makeUgvBaseCommand(const uint8_t command) const
    {
        sunray_msgs::UGVControlCMD cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.cmd_source = sunray_msgs::UGVControlCMD::SUNRAY_STATION;
        cmd.control_cmd = command;
        return cmd;
    }

    void publishUavCommand(const int id, sunray_msgs::UAVControlCMD cmd, const bool continuous, const QString &label)
    {
        cmd.header.stamp = ros::Time::now();
        if (continuous)
        {
            active_platform_ = Platform::UAV;
            active_agent_id_ = id;
            active_uav_cmd_ = cmd;
            has_active_command_ = true;
            active_command_label_ = label;
        }
        else
        {
            has_active_command_ = false;
        }
        uavControlPublisher(id).publish(cmd);
        appendLog(QString("发布 UAV%1 %2 -> %3")
                      .arg(id)
                      .arg(label)
                      .arg(QString::fromStdString(configuredControlCmdTopic(Platform::UAV, id))));
    }

    void publishUgvCommand(const int id, sunray_msgs::UGVControlCMD cmd, const bool continuous, const QString &label)
    {
        cmd.header.stamp = ros::Time::now();
        if (continuous)
        {
            active_platform_ = Platform::UGV;
            active_agent_id_ = id;
            active_ugv_cmd_ = cmd;
            has_active_command_ = true;
            active_command_label_ = label;
        }
        else
        {
            has_active_command_ = false;
        }
        ugvControlPublisher(id).publish(cmd);
        appendLog(QString("发布 UGV%1 %2 -> %3")
                      .arg(id)
                      .arg(label)
                      .arg(QString::fromStdString(configuredControlCmdTopic(Platform::UGV, id))));
    }

    void publishSingleTakeoff()
    {
        publishUavCommand(uavTargetId(),
                          makeUavBaseCommand(sunray_msgs::UAVControlCMD::TAKEOFF),
                          false,
                          "TAKEOFF");
    }

    void publishSingleHold()
    {
        sunray_msgs::UAVControlCMD cmd = makeUavBaseCommand(sunray_msgs::UAVControlCMD::HOVER);
        cmd.yaw_mode = sunray_msgs::UAVControlCMD::KEEP_YAW;
        publishUavCommand(uavTargetId(), cmd, false, "HOVER");
    }

    void publishSingleLand()
    {
        publishUavCommand(uavTargetId(),
                          makeUavBaseCommand(sunray_msgs::UAVControlCMD::LAND),
                          false,
                          "LAND");
    }

    void publishSingleReturn()
    {
        publishUavCommand(uavTargetId(),
                          makeUavBaseCommand(sunray_msgs::UAVControlCMD::RETURN),
                          false,
                          "RETURN");
    }

    void publishSingleKill()
    {
        publishUavCommand(uavTargetId(),
                          makeUavBaseCommand(sunray_msgs::UAVControlCMD::KILL),
                          false,
                          "KILL");
    }

    void publishUgvHold()
    {
        publishUgvCommand(ugvTargetId(),
                          makeUgvBaseCommand(sunray_msgs::UGVControlCMD::HOLD),
                          false,
                          "HOLD");
    }

    void publishUgvReturn()
    {
        publishUgvCommand(ugvTargetId(),
                          makeUgvBaseCommand(sunray_msgs::UGVControlCMD::RETURN),
                          false,
                          "RETURN");
    }

    void publishMovePoint()
    {
        sunray_msgs::UAVControlCMD cmd = makeUavBaseCommand(sunray_msgs::UAVControlCMD::MOVE_POINT);
        cmd.desired_pos.x = uav_point_x_spin_->value();
        cmd.desired_pos.y = uav_point_y_spin_->value();
        cmd.desired_pos.z = uav_point_z_spin_->value();
        cmd.desired_yaw = uav_point_yaw_spin_->value() * kDegToRad;
        publishUavCommand(uavTargetId(), cmd, false, "MOVE_POINT");
    }

    void publishUgvMovePoint()
    {
        sunray_msgs::UGVControlCMD cmd = makeUgvBaseCommand(sunray_msgs::UGVControlCMD::MOVE_POINT);
        cmd.desired_pos.x = ugv_point_x_spin_->value();
        cmd.desired_pos.y = ugv_point_y_spin_->value();
        cmd.desired_pos.z = 0.0;
        cmd.desired_yaw = ugv_point_yaw_spin_->value() * kDegToRad;
        publishUgvCommand(ugvTargetId(), cmd, false, "MOVE_POINT");
    }

    void startWorldVelocity()
    {
        sunray_msgs::UAVControlCMD cmd = makeUavBaseCommand(sunray_msgs::UAVControlCMD::MOVE_VELOCITY);
        cmd.desired_vel.x = uav_world_vx_spin_->value();
        cmd.desired_vel.y = uav_world_vy_spin_->value();
        cmd.desired_vel.z = uav_world_vz_spin_->value();
        cmd.fixed_height = uav_world_height_spin_->value();
        cmd.desired_yaw = uav_world_yaw_spin_->value() * kDegToRad;
        publishUavCommand(uavTargetId(), cmd, true, "MOVE_VELOCITY");
    }

    void startUgvWorldVelocity()
    {
        sunray_msgs::UGVControlCMD cmd = makeUgvBaseCommand(sunray_msgs::UGVControlCMD::MOVE_VELOCITY);
        cmd.desired_vel.x = ugv_world_vx_spin_->value();
        cmd.desired_vel.y = ugv_world_vy_spin_->value();
        cmd.desired_vel.z = 0.0;
        cmd.desired_yaw = ugv_world_yaw_spin_->value() * kDegToRad;
        publishUgvCommand(ugvTargetId(), cmd, true, "MOVE_VELOCITY");
    }

    void startBodyVelocity()
    {
        sunray_msgs::UAVControlCMD cmd = makeUavBaseCommand(sunray_msgs::UAVControlCMD::MOVE_VELOCITY_BODY);
        cmd.desired_body_xy_vel.x = uav_body_vx_spin_->value();
        cmd.desired_body_xy_vel.y = uav_body_vy_spin_->value();
        cmd.fixed_height = uav_body_height_spin_->value();
        cmd.yaw_mode = sunray_msgs::UAVControlCMD::SET_YAWRATE;
        cmd.desired_yaw_rate = uav_body_yaw_rate_spin_->value() * kDegToRad;
        publishUavCommand(uavTargetId(), cmd, true, "MOVE_VELOCITY_BODY");
    }

    void startUgvBodyVelocity()
    {
        sunray_msgs::UGVControlCMD cmd = makeUgvBaseCommand(sunray_msgs::UGVControlCMD::MOVE_VELOCITY_BODY);
        cmd.cmd_vel.linear.x = ugv_body_vx_spin_->value();
        cmd.cmd_vel.linear.y = ugv_body_vy_spin_->value();
        cmd.cmd_vel.angular.z = ugv_body_yaw_rate_spin_->value() * kDegToRad;
        publishUgvCommand(ugvTargetId(), cmd, true, "MOVE_VELOCITY_BODY");
    }

    void stopContinuousCommand()
    {
        const bool was_active = has_active_command_;
        has_active_command_ = false;
        appendLog(was_active ? "停止 5Hz 持续速度指令" : "当前没有持续速度指令");
    }

    void publishActiveCommand()
    {
        if (!has_active_command_)
        {
            return;
        }

        if (active_platform_ == Platform::UAV)
        {
            active_uav_cmd_.header.stamp = ros::Time::now();
            uavControlPublisher(active_agent_id_).publish(active_uav_cmd_);
        }
        else
        {
            active_ugv_cmd_.header.stamp = ros::Time::now();
            ugvControlPublisher(active_agent_id_).publish(active_ugv_cmd_);
        }
    }

    void publishUavSwarmTakeoff()
    {
        publishUavSwarmCommand(sunray_msgs::UAVSwarmCMD::SWARM_TAKEOFF, "SWARM_TAKEOFF", uavSwarmTargetId());
    }

    void publishUavSwarmHold()
    {
        publishUavSwarmCommand(sunray_msgs::UAVSwarmCMD::SWARM_HOVER, "SWARM_HOVER", uavSwarmTargetId());
    }

    void publishUavSwarmLand()
    {
        publishUavSwarmCommand(sunray_msgs::UAVSwarmCMD::SWARM_LAND, "SWARM_LAND", uavSwarmTargetId());
    }

    void publishUavSwarmReturn()
    {
        publishUavSwarmCommand(sunray_msgs::UAVSwarmCMD::SWARM_RETURN, "SWARM_RETURN", uavSwarmTargetId());
    }

    void publishUgvSwarmHold()
    {
        publishUgvSwarmCommand(sunray_msgs::UGVSwarmCMD::SWARM_HOLD, "SWARM_HOLD", ugvSwarmTargetId());
    }

    void publishUgvSwarmReturn()
    {
        publishUgvSwarmCommand(sunray_msgs::UGVSwarmCMD::SWARM_RETURN, "SWARM_RETURN", ugvSwarmTargetId());
    }

    int uavTargetId() const
    {
        return uav_id_spin_->value();
    }

    int ugvTargetId() const
    {
        return ugv_id_spin_->value();
    }

    int uavSwarmTargetId() const
    {
        return uav_swarm_target_id_spin_->value();
    }

    int ugvSwarmTargetId() const
    {
        return ugv_swarm_target_id_spin_->value();
    }

    void publishUavSwarmCommand(const uint8_t command, const QString &label, const int target_id)
    {
        sunray_msgs::UAVSwarmCMD msg;
        msg.header.stamp = ros::Time::now();
        msg.cmd_source = sunray_msgs::UAVSwarmCMD::GROUND_STATION;
        msg.agent_id = static_cast<uint8_t>(target_id);
        msg.swarm_cmd = command;
        uav_swarm_cmd_pub_.publish(msg);
        appendLog(QString("发布 UAV 集群 %1 target=%2 -> %3")
                      .arg(label)
                      .arg(target_id)
                      .arg(QString::fromStdString(uav_swarm_cmd_topic_)));
    }

    void publishUgvSwarmCommand(const uint8_t command, const QString &label, const int target_id)
    {
        sunray_msgs::UGVSwarmCMD msg;
        msg.header.stamp = ros::Time::now();
        msg.cmd_source = sunray_msgs::UGVSwarmCMD::GROUND_STATION;
        msg.agent_id = static_cast<uint8_t>(target_id);
        msg.swarm_cmd = command;
        ugv_swarm_cmd_pub_.publish(msg);
        appendLog(QString("发布 UGV 集群 %1 target=%2 -> %3")
                      .arg(label)
                      .arg(target_id)
                      .arg(QString::fromStdString(ugv_swarm_cmd_topic_)));
    }

    sunray_msgs::Formation makeFormationCommand() const
    {
        sunray_msgs::Formation formation;
        formation.header.stamp = ros::Time::now();
        formation.formation_type = static_cast<uint8_t>(formation_combo_->currentData().toInt());
        formation.leader_pos.x = formation_leader_x_spin_->value();
        formation.leader_pos.y = formation_leader_y_spin_->value();
        formation.leader_pos.z = formation_leader_z_spin_->value();
        formation.leader_yaw = static_cast<float>(formation_leader_yaw_spin_->value() * kDegToRad);
        formation.static_line_spacing = static_cast<float>(formation_line_spacing_spin_->value());
        formation.static_line_angle = static_cast<float>(formation_line_angle_spin_->value());
        formation.static_polygon_spacing = static_cast<float>(formation_polygon_spacing_spin_->value());
        formation.dynamic_time = static_cast<float>(formation_dynamic_time_spin_->value());
        formation.dynamic_ring_radius = static_cast<float>(formation_ring_radius_spin_->value());
        formation.dynamic_ring_move_speed = static_cast<float>(formation_ring_speed_spin_->value());
        formation.dynamic_polygon_spacing = static_cast<float>(formation_dynamic_polygon_spacing_spin_->value());
        formation.dynamic_polygon_move_speed = static_cast<float>(formation_dynamic_polygon_speed_spin_->value());
        formation.dynamic_lemniscate_x_radius = static_cast<float>(formation_lemniscate_x_spin_->value());
        formation.dynamic_lemniscate_y_radius = static_cast<float>(formation_lemniscate_y_spin_->value());
        formation.dynamic_lemniscate_move_speed = static_cast<float>(formation_lemniscate_speed_spin_->value());
        return formation;
    }

    void publishUavFormation()
    {
        const sunray_msgs::Formation formation = makeFormationCommand();
        sunray_msgs::UAVSwarmCMD msg;
        msg.header.stamp = ros::Time::now();
        msg.cmd_source = sunray_msgs::UAVSwarmCMD::GROUND_STATION;
        msg.agent_id = static_cast<uint8_t>(uavSwarmTargetId());
        msg.swarm_cmd = sunray_msgs::UAVSwarmCMD::SWARM_FORMATION;
        msg.formation_cmd = formation;
        uav_swarm_cmd_pub_.publish(msg);
        appendLog(QString("发布 UAV 阵型 %1 target=%2")
                      .arg(formationName(formation.formation_type))
                      .arg(uavSwarmTargetId()));
    }

    void publishUgvFormation()
    {
        const sunray_msgs::Formation formation = makeFormationCommand();
        sunray_msgs::UGVSwarmCMD msg;
        msg.header.stamp = ros::Time::now();
        msg.cmd_source = sunray_msgs::UGVSwarmCMD::GROUND_STATION;
        msg.agent_id = static_cast<uint8_t>(ugvSwarmTargetId());
        msg.swarm_cmd = sunray_msgs::UGVSwarmCMD::SWARM_FORMATION;
        msg.formation_cmd = formation;
        ugv_swarm_cmd_pub_.publish(msg);
        appendLog(QString("发布 UGV 阵型 %1 target=%2")
                      .arg(formationName(formation.formation_type))
                      .arg(ugvSwarmTargetId()));
    }

    void onFormationChanged()
    {
        if (formation_param_stack_ != nullptr)
        {
            formation_param_stack_->setCurrentIndex(formation_combo_->currentIndex());
        }

        const uint8_t formation_type = static_cast<uint8_t>(formation_combo_->currentData().toInt());
        const bool is_dynamic = formation_type == sunray_msgs::Formation::DYNAMIC_FORMATION_RING ||
                                formation_type == sunray_msgs::Formation::DYNAMIC_FORMATION_POLYGON ||
                                formation_type == sunray_msgs::Formation::DYNAMIC_FORMATION_LEMNISCATE;
        if (formation_dynamic_time_group_ != nullptr)
        {
            formation_dynamic_time_group_->setVisible(is_dynamic);
        }
    }

    void publishMarkers()
    {
        visualization_msgs::MarkerArray markers;
        visualization_msgs::Marker clear_marker;
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        markers.markers.push_back(clear_marker);

        const ros::Time now = ros::Time::now();
        int marker_id = 0;
        for (size_t i = 0; i < agents_.size(); ++i)
        {
            const AgentRuntime &agent = agents_[i];
            nav_msgs::Odometry odom;
            if (!getAgentOdom(agent, odom))
            {
                continue;
            }

            const bool is_uav = agent.config.platform == Platform::UAV;
            const std_msgs::ColorRGBA agent_color = is_uav ? makeColor(0.10, 0.72, 0.58, 0.90)
                                                           : makeColor(0.92, 0.58, 0.18, 0.90);
            const std_msgs::ColorRGBA target_color = is_uav ? makeColor(0.42, 0.82, 1.0, 0.85)
                                                            : makeColor(1.0, 0.78, 0.28, 0.85);

            visualization_msgs::Marker body;
            body.header.frame_id = fixed_frame_;
            body.header.stamp = now;
            body.ns = "sunray_monitor_agent";
            body.id = marker_id++;
            body.action = visualization_msgs::Marker::ADD;
            body.type = is_uav ? visualization_msgs::Marker::SPHERE : visualization_msgs::Marker::CUBE;
            body.pose = odom.pose.pose;
            body.scale.x = is_uav ? 0.45 : 0.55;
            body.scale.y = is_uav ? 0.45 : 0.38;
            body.scale.z = is_uav ? 0.20 : 0.28;
            body.color = agent_color;
            markers.markers.push_back(body);

            visualization_msgs::Marker arrow;
            arrow.header.frame_id = fixed_frame_;
            arrow.header.stamp = now;
            arrow.ns = "sunray_monitor_heading";
            arrow.id = marker_id++;
            arrow.action = visualization_msgs::Marker::ADD;
            arrow.type = visualization_msgs::Marker::ARROW;
            arrow.pose = odom.pose.pose;
            arrow.scale.x = is_uav ? 0.85 : 0.70;
            arrow.scale.y = 0.08;
            arrow.scale.z = 0.12;
            arrow.color = makeColor(agent_color.r, agent_color.g, agent_color.b, 0.95);
            markers.markers.push_back(arrow);

            visualization_msgs::Marker label;
            label.header.frame_id = fixed_frame_;
            label.header.stamp = now;
            label.ns = "sunray_monitor_label";
            label.id = marker_id++;
            label.action = visualization_msgs::Marker::ADD;
            label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            label.pose.position = odom.pose.pose.position;
            label.pose.position.z += is_uav ? 0.65 : 0.45;
            label.scale.z = 0.28;
            label.color = makeColor(0.96, 0.98, 0.94, 0.95);
            label.text = QString("%1%2 %3")
                             .arg(platformText(agent.config.platform))
                             .arg(agent.config.id)
                             .arg(controlStateText(agent))
                             .toStdString();
            markers.markers.push_back(label);

            geometry_msgs::Point target;
            if (getAgentTarget(agent, target))
            {
                visualization_msgs::Marker target_marker;
                target_marker.header.frame_id = fixed_frame_;
                target_marker.header.stamp = now;
                target_marker.ns = "sunray_monitor_target";
                target_marker.id = marker_id++;
                target_marker.action = visualization_msgs::Marker::ADD;
                target_marker.type = visualization_msgs::Marker::SPHERE;
                target_marker.pose.position = target;
                target_marker.pose.orientation.w = 1.0;
                target_marker.scale.x = 0.28;
                target_marker.scale.y = 0.28;
                target_marker.scale.z = 0.28;
                target_marker.color = target_color;
                markers.markers.push_back(target_marker);

                visualization_msgs::Marker line;
                line.header.frame_id = fixed_frame_;
                line.header.stamp = now;
                line.ns = "sunray_monitor_target_line";
                line.id = marker_id++;
                line.action = visualization_msgs::Marker::ADD;
                line.type = visualization_msgs::Marker::LINE_LIST;
                line.points.push_back(odom.pose.pose.position);
                line.points.push_back(target);
                line.scale.x = 0.035;
                line.color = makeColor(target_color.r, target_color.g, target_color.b, 0.70);
                markers.markers.push_back(line);
            }
        }

        marker_pub_.publish(markers);
    }

    void appendLog(const QString &text)
    {
        if (log_view_ != nullptr)
        {
            log_view_->appendPlainText(QString("[%1] %2")
                                           .arg(QString::number(ros::Time::now().toSec(), 'f', 2))
                                           .arg(text));
        }
        ROS_INFO("%s", text.toStdString().c_str());
    }

    int findAgentIndex(const Platform platform, const int id) const
    {
        for (size_t i = 0; i < agents_.size(); ++i)
        {
            if (agents_[i].config.platform == platform && agents_[i].config.id == id)
            {
                return static_cast<int>(i);
            }
        }
        return -1;
    }

    void uavControlStateCallback(const sunray_msgs::UAVControlState::ConstPtr &msg, const size_t idx)
    {
        if (idx >= agents_.size())
        {
            return;
        }
        AgentRuntime &agent = agents_[idx];
        agent.uav_control_state = *msg;
        agent.has_uav_control = true;
        agent.last_control_time = ros::Time::now();
    }

    void ugvControlStateCallback(const sunray_msgs::UGVControlState::ConstPtr &msg, const size_t idx)
    {
        if (idx >= agents_.size())
        {
            return;
        }
        AgentRuntime &agent = agents_[idx];
        agent.ugv_control_state = *msg;
        agent.has_ugv_control = true;
        agent.last_control_time = ros::Time::now();
    }

    void odomStateCallback(const sunray_msgs::OdomState::ConstPtr &msg, const size_t idx)
    {
        if (idx >= agents_.size())
        {
            return;
        }
        AgentRuntime &agent = agents_[idx];
        agent.odom_state = *msg;
        agent.has_odom_state = true;
        agent.last_odom_time = ros::Time::now();
    }

    void planningStateCallback(const sunray_msgs::UAVPlanningState::ConstPtr &msg, const size_t idx)
    {
        if (idx >= agents_.size())
        {
            return;
        }
        AgentRuntime &agent = agents_[idx];
        agent.planning_state = *msg;
        agent.has_planning_state = true;
        agent.last_planning_time = ros::Time::now();
    }

    void uavSwarmStateCallback(const sunray_msgs::UAVSwarmState::ConstPtr &msg)
    {
        const int idx = findAgentIndex(Platform::UAV, msg->agent_id);
        if (idx < 0)
        {
            return;
        }
        AgentRuntime &agent = agents_[static_cast<size_t>(idx)];
        agent.uav_swarm_state = *msg;
        agent.has_uav_swarm_state = true;
        agent.last_swarm_time = ros::Time::now();
    }

    void ugvSwarmStateCallback(const sunray_msgs::UGVSwarmState::ConstPtr &msg)
    {
        const int idx = findAgentIndex(Platform::UGV, msg->agent_id);
        if (idx < 0)
        {
            return;
        }
        AgentRuntime &agent = agents_[static_cast<size_t>(idx)];
        agent.ugv_swarm_state = *msg;
        agent.has_ugv_swarm_state = true;
        agent.last_swarm_time = ros::Time::now();
    }

    void topicStateCallback(const topic_tools::ShapeShifter::ConstPtr &msg, const size_t idx)
    {
        if (idx >= topics_.size())
        {
            return;
        }

        TopicRuntime &runtime = topics_[idx];
        const ros::Time now = ros::Time::now();
        if (runtime.message_count == 0)
        {
            runtime.first_receive_time = now;
        }
        runtime.last_receive_time = now;
        runtime.message_count++;
        runtime.datatype = msg->getDataType();
        runtime.md5sum = msg->getMD5Sum();
    }

    ros::NodeHandle nh_;
    std::string fixed_frame_{"world"};
    std::string marker_topic_{"/sunray/monitor/markers"};
    std::string uav_name_prefix_{"uav"};
    std::string ugv_name_prefix_{"ugv"};
    std::string uav_swarm_cmd_topic_{"/sunray/swarm/uav_swarm_cmd"};
    std::string ugv_swarm_cmd_topic_{"/sunray/swarm/ugv_swarm_cmd"};
    std::string uav_swarm_state_topic_{"/sunray/swarm/uav_swarm_state"};
    std::string ugv_swarm_state_topic_{"/sunray/swarm/ugv_swarm_state"};
    double status_timeout_{1.0};
    double refresh_hz_{5.0};

    std::vector<AgentRuntime> agents_;
    std::vector<TopicRuntime> topics_;
    std::map<int, ros::Publisher> uav_control_pubs_;
    std::map<int, ros::Publisher> ugv_control_pubs_;
    ros::Publisher marker_pub_;
    ros::Publisher uav_swarm_cmd_pub_;
    ros::Publisher ugv_swarm_cmd_pub_;
    ros::Subscriber uav_swarm_state_sub_;
    ros::Subscriber ugv_swarm_state_sub_;

    QTimer *refresh_timer_{nullptr};
    QTimer *command_publish_timer_{nullptr};
    QLabel *summary_label_{nullptr};
    QLabel *overview_label_{nullptr};
    QLabel *rviz_status_label_{nullptr};
    QLabel *logo_label_{nullptr};
    QTabWidget *left_tabs_{nullptr};
    QTableWidget *agent_table_{nullptr};
    QTableWidget *topic_table_{nullptr};
    QPlainTextEdit *detail_text_{nullptr};
    QPlainTextEdit *log_view_{nullptr};
    rviz::RenderPanel *rviz_panel_{nullptr};
    rviz::VisualizationManager *rviz_manager_{nullptr};

    QSpinBox *uav_id_spin_{nullptr};
    QDoubleSpinBox *uav_point_x_spin_{nullptr};
    QDoubleSpinBox *uav_point_y_spin_{nullptr};
    QDoubleSpinBox *uav_point_z_spin_{nullptr};
    QDoubleSpinBox *uav_point_yaw_spin_{nullptr};
    QDoubleSpinBox *uav_world_vx_spin_{nullptr};
    QDoubleSpinBox *uav_world_vy_spin_{nullptr};
    QDoubleSpinBox *uav_world_vz_spin_{nullptr};
    QDoubleSpinBox *uav_world_height_spin_{nullptr};
    QDoubleSpinBox *uav_world_yaw_spin_{nullptr};
    QDoubleSpinBox *uav_body_vx_spin_{nullptr};
    QDoubleSpinBox *uav_body_vy_spin_{nullptr};
    QDoubleSpinBox *uav_body_height_spin_{nullptr};
    QDoubleSpinBox *uav_body_yaw_rate_spin_{nullptr};

    QSpinBox *ugv_id_spin_{nullptr};
    QDoubleSpinBox *ugv_point_x_spin_{nullptr};
    QDoubleSpinBox *ugv_point_y_spin_{nullptr};
    QDoubleSpinBox *ugv_point_yaw_spin_{nullptr};
    QDoubleSpinBox *ugv_world_vx_spin_{nullptr};
    QDoubleSpinBox *ugv_world_vy_spin_{nullptr};
    QDoubleSpinBox *ugv_world_yaw_spin_{nullptr};
    QDoubleSpinBox *ugv_body_vx_spin_{nullptr};
    QDoubleSpinBox *ugv_body_vy_spin_{nullptr};
    QDoubleSpinBox *ugv_body_yaw_rate_spin_{nullptr};

    QSpinBox *uav_swarm_target_id_spin_{nullptr};
    QSpinBox *ugv_swarm_target_id_spin_{nullptr};
    QComboBox *formation_combo_{nullptr};
    QWidget *formation_dynamic_time_group_{nullptr};
    QStackedWidget *formation_param_stack_{nullptr};
    QDoubleSpinBox *formation_leader_x_spin_{nullptr};
    QDoubleSpinBox *formation_leader_y_spin_{nullptr};
    QDoubleSpinBox *formation_leader_z_spin_{nullptr};
    QDoubleSpinBox *formation_leader_yaw_spin_{nullptr};
    QDoubleSpinBox *formation_line_spacing_spin_{nullptr};
    QDoubleSpinBox *formation_line_angle_spin_{nullptr};
    QDoubleSpinBox *formation_polygon_spacing_spin_{nullptr};
    QDoubleSpinBox *formation_dynamic_time_spin_{nullptr};
    QDoubleSpinBox *formation_ring_radius_spin_{nullptr};
    QDoubleSpinBox *formation_ring_speed_spin_{nullptr};
    QDoubleSpinBox *formation_dynamic_polygon_spacing_spin_{nullptr};
    QDoubleSpinBox *formation_dynamic_polygon_speed_spin_{nullptr};
    QDoubleSpinBox *formation_lemniscate_x_spin_{nullptr};
    QDoubleSpinBox *formation_lemniscate_y_spin_{nullptr};
    QDoubleSpinBox *formation_lemniscate_speed_spin_{nullptr};

    bool has_active_command_{false};
    Platform active_platform_{Platform::UAV};
    int active_agent_id_{1};
    QString active_command_label_;
    sunray_msgs::UAVControlCMD active_uav_cmd_;
    sunray_msgs::UGVControlCMD active_ugv_cmd_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sunray_monitor_panel_node");
    QApplication app(argc, argv);
    ros::NodeHandle private_nh("~");

    SunrayMonitorPanel panel(private_nh);
    panel.show();

    return app.exec();
}
