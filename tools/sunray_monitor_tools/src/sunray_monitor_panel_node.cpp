#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <XmlRpcValue.h>

#include <QApplication>
#include <QColor>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QMainWindow>
#include <QSplitter>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <rviz/display.h>
#include <rviz/render_panel.h>
#include <rviz/visualization_manager.h>

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace
{

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

QString secondsText(const double value)
{
    if (value < 0.0)
    {
        return "--";
    }
    return QString("%1 s").arg(value, 0, 'f', 2);
}

QString hzText(const double value)
{
    if (value <= 0.0)
    {
        return "--";
    }
    return QString("%1 Hz").arg(value, 0, 'f', 1);
}

QTableWidgetItem *makeItem(const QString &text)
{
    auto *item = new QTableWidgetItem(text);
    item->setFlags(item->flags() & ~Qt::ItemIsEditable);
    return item;
}

void setRowColor(QTableWidget *table, const int row, const QColor &background, const QColor &foreground)
{
    if (!table)
    {
        return;
    }
    for (int column = 0; column < table->columnCount(); ++column)
    {
        QTableWidgetItem *item = table->item(row, column);
        if (item)
        {
            item->setBackground(background);
            item->setForeground(foreground);
        }
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

        refresh_timer_ = new QTimer(this);
        connect(refresh_timer_, &QTimer::timeout, this, [this]() {
            refreshTable();
            ros::spinOnce();
        });
        refresh_timer_->start(static_cast<int>(1000.0 / std::max(1.0, refresh_hz_)));

        setWindowTitle("Sunray Monitor Panel");
        resize(1420, 860);
    }

    ~SunrayMonitorPanel() override
    {
        if (rviz_manager_)
        {
            delete rviz_manager_;
            rviz_manager_ = nullptr;
        }
    }

  private:
    void loadParams()
    {
        nh_.param("fixed_frame", fixed_frame_, std::string("world"));
        nh_.param("status_timeout", status_timeout_, 1.0);
        nh_.param("refresh_hz", refresh_hz_, 5.0);

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
        for (size_t i = 0; i < topics_.size(); ++i)
        {
            topics_[i].sub = nh_.subscribe<topic_tools::ShapeShifter>(
                topics_[i].config.topic,
                20,
                [this, i](const topic_tools::ShapeShifter::ConstPtr &msg) {
                    stateCallback(msg, i);
                });
            ROS_INFO("sunray_monitor_panel subscribe: %s", topics_[i].config.topic.c_str());
        }
    }

    void setupUi()
    {
        auto *central = new QWidget();
        auto *root = new QVBoxLayout(central);

        auto *title = new QLabel("Sunray 状态监控面板");
        title->setObjectName("titleLabel");
        root->addWidget(title);

        auto *splitter = new QSplitter(Qt::Horizontal);
        splitter->addWidget(buildStatusPanel());

        rviz_container_ = new QWidget();
        auto *rviz_layout = new QVBoxLayout(rviz_container_);
        rviz_layout->setContentsMargins(0, 0, 0, 0);
        rviz_panel_ = new rviz::RenderPanel(rviz_container_);
        rviz_layout->addWidget(rviz_panel_);
        splitter->addWidget(rviz_container_);

        splitter->setStretchFactor(0, 3);
        splitter->setStretchFactor(1, 5);
        splitter->setSizes({520, 860});
        root->addWidget(splitter, 1);

        setCentralWidget(central);
        applyStyle();
    }

    QWidget *buildStatusPanel()
    {
        auto *box = new QGroupBox("状态话题监控");
        auto *layout = new QVBoxLayout(box);

        summary_label_ = new QLabel("等待状态话题...");
        summary_label_->setObjectName("stateLabel");
        layout->addWidget(summary_label_);

        table_ = new QTableWidget();
        table_->setColumnCount(9);
        table_->setHorizontalHeaderLabels(QStringList()
                                          << "状态"
                                          << "名称"
                                          << "模块"
                                          << "类型"
                                          << "ID"
                                          << "频率"
                                          << "延迟"
                                          << "消息类型"
                                          << "话题");
        table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
        table_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
        table_->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
        table_->horizontalHeader()->setSectionResizeMode(3, QHeaderView::ResizeToContents);
        table_->horizontalHeader()->setSectionResizeMode(4, QHeaderView::ResizeToContents);
        table_->horizontalHeader()->setSectionResizeMode(5, QHeaderView::ResizeToContents);
        table_->horizontalHeader()->setSectionResizeMode(6, QHeaderView::ResizeToContents);
        table_->horizontalHeader()->setSectionResizeMode(7, QHeaderView::ResizeToContents);
        table_->horizontalHeader()->setSectionResizeMode(8, QHeaderView::Stretch);
        table_->verticalHeader()->setVisible(false);
        table_->setSelectionBehavior(QAbstractItemView::SelectRows);
        table_->setAlternatingRowColors(true);
        table_->setRowCount(static_cast<int>(topics_.size()));

        for (int row = 0; row < static_cast<int>(topics_.size()); ++row)
        {
            for (int column = 0; column < table_->columnCount(); ++column)
            {
                table_->setItem(row, column, makeItem("--"));
            }
        }

        layout->addWidget(table_, 1);
        return box;
    }

    void setupRviz()
    {
        rviz_manager_ = new rviz::VisualizationManager(rviz_panel_);
        rviz_panel_->initialize(rviz_manager_->getSceneManager(), rviz_manager_);
        rviz_manager_->initialize();
        rviz_manager_->startUpdate();
        rviz_manager_->setFixedFrame(QString::fromStdString(fixed_frame_));

        rviz::Display *grid = rviz_manager_->createDisplay("rviz/Grid", "Grid", true);
        if (grid)
        {
            grid->subProp("Plane Cell Count")->setValue(20);
            grid->subProp("Cell Size")->setValue(1.0);
            grid->subProp("Color")->setValue(QColor(150, 160, 150));
            grid->subProp("Alpha")->setValue(0.5);
        }

        rviz::Display *tf = rviz_manager_->createDisplay("rviz/TF", "TF", true);
        if (tf)
        {
            tf->subProp("Show Names")->setValue(true);
            tf->subProp("Marker Scale")->setValue(0.35);
        }

        rviz::Display *marker = rviz_manager_->createDisplay("rviz/MarkerArray", "MonitorMarkers", true);
        if (marker)
        {
            marker->subProp("Marker Topic")->setValue("/sunray/monitor/markers");
        }
    }

    void applyStyle()
    {
        setStyleSheet(R"(
            QMainWindow, QWidget { background: #eef2ed; color: #17211c; font-family: "Noto Sans CJK SC", "Microsoft YaHei", sans-serif; font-size: 13px; }
            #titleLabel { font-size: 24px; font-weight: 800; color: #16352d; padding: 4px 0px 8px 2px; }
            #stateLabel { color: #16352d; font-weight: 700; }
            QGroupBox { border: 1px solid #bac7bf; border-radius: 10px; margin-top: 12px; background: #f8faf7; font-weight: 700; }
            QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0px 6px; color: #31584d; }
            QTableWidget { background: #fbfcfa; alternate-background-color: #f1f5f1; border: 1px solid #b9c7be; border-radius: 8px; gridline-color: #d5ded8; }
            QHeaderView::section { background: #dbe7df; color: #20362e; border: 0px; padding: 6px; font-weight: 700; }
        )");
    }

    void stateCallback(const topic_tools::ShapeShifter::ConstPtr &msg, const size_t idx)
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

    void refreshTable()
    {
        if (!table_)
        {
            return;
        }

        const ros::Time now = ros::Time::now();
        int online_count = 0;
        for (int row = 0; row < static_cast<int>(topics_.size()); ++row)
        {
            const TopicRuntime &runtime = topics_[static_cast<size_t>(row)];
            const bool has_msg = runtime.message_count > 0 && !runtime.last_receive_time.isZero();
            const double age = has_msg ? (now - runtime.last_receive_time).toSec() : -1.0;
            const bool online = has_msg && age <= status_timeout_;
            if (online)
            {
                online_count++;
            }

            const double duration = has_msg ? std::max(0.001, (now - runtime.first_receive_time).toSec()) : 0.0;
            const double hz = runtime.message_count > 1 ? static_cast<double>(runtime.message_count - 1) / duration : 0.0;
            const QString agent_id_text = runtime.config.agent_id > 0 ? QString::number(runtime.config.agent_id) : "-";

            table_->item(row, 0)->setText(online ? "在线" : "离线");
            table_->item(row, 1)->setText(QString::fromStdString(runtime.config.name));
            table_->item(row, 2)->setText(QString::fromStdString(runtime.config.module));
            table_->item(row, 3)->setText(QString::fromStdString(runtime.config.agent_type));
            table_->item(row, 4)->setText(agent_id_text);
            table_->item(row, 5)->setText(hzText(hz));
            table_->item(row, 6)->setText(secondsText(age));
            table_->item(row, 7)->setText(runtime.datatype.empty() ? "--" : QString::fromStdString(runtime.datatype));
            table_->item(row, 8)->setText(QString::fromStdString(runtime.config.topic));

            setRowColor(table_,
                        row,
                        online ? QColor(201, 235, 211) : QColor(255, 238, 226),
                        online ? QColor(18, 83, 48) : QColor(126, 54, 33));
        }

        if (summary_label_)
        {
            summary_label_->setText(QString("状态话题: %1  在线: %2  超时阈值: %3 s  FixedFrame: %4")
                                        .arg(topics_.size())
                                        .arg(online_count)
                                        .arg(status_timeout_, 0, 'f', 2)
                                        .arg(QString::fromStdString(fixed_frame_)));
        }
    }

    ros::NodeHandle nh_;
    std::string fixed_frame_{"world"};
    double status_timeout_{1.0};
    double refresh_hz_{5.0};
    std::vector<TopicRuntime> topics_;

    QTimer *refresh_timer_{nullptr};
    QLabel *summary_label_{nullptr};
    QTableWidget *table_{nullptr};
    QWidget *rviz_container_{nullptr};
    rviz::RenderPanel *rviz_panel_{nullptr};
    rviz::VisualizationManager *rviz_manager_{nullptr};
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
