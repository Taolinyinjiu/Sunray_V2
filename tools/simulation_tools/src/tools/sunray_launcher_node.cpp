#include <ros/master.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <XmlRpcValue.h>

#include <QApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QIcon>
#include <QLabel>
#include <QPixmap>
#include <QColor>
#include <QProcess>
#include <QProgressBar>
#include <QPushButton>
#include <QSizePolicy>
#include <QStringList>
#include <QSplitter>
#include <QStandardPaths>
#include <QTextCursor>
#include <QTextEdit>
#include <QTimer>
#include <QTreeWidget>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QWidget>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <sys/stat.h>
#include <unistd.h>

namespace
{

constexpr int kLaunchIndexRole = Qt::UserRole + 1;
constexpr int kPidFileWarmupMsec = 3000;

struct LaunchItem
{
    std::string group;
    std::string title;
    std::string package;
    std::string launch;
    std::string description;
    std::vector<std::string> args;
};

struct LaunchGroup
{
    std::string name;
    std::vector<int> item_indices;
};

struct LaunchRuntime
{
    QString script_path;
    QString pid_file;
    QString done_file;
    QString command;
    QString terminal_program;
    qint64 terminal_pid{0};
    QDateTime start_time;
    bool running{false};
    bool stop_requested{false};
};

struct CpuSample
{
    unsigned long long idle{0};
    unsigned long long total{0};
    bool valid{false};
};

struct LaunchCommand
{
    std::string package_name;
    std::string launch_file;
    QStringList args;
};

std::string xmlRpcToString(const XmlRpc::XmlRpcValue &value, const std::string &fallback = "")
{
    if (value.getType() != XmlRpc::XmlRpcValue::TypeString)
    {
        return fallback;
    }
    return static_cast<std::string>(value);
}

std::vector<std::string> xmlRpcToStringList(const XmlRpc::XmlRpcValue &value)
{
    std::vector<std::string> result;
    if (value.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
        for (int i = 0; i < value.size(); ++i)
        {
            if (value[i].getType() == XmlRpc::XmlRpcValue::TypeString)
            {
                result.push_back(static_cast<std::string>(value[i]));
            }
        }
    }
    return result;
}

QString htmlEscape(const std::string &text)
{
    return QString::fromStdString(text).toHtmlEscaped();
}

QString shellQuote(const QString &text)
{
    QString escaped = text;
    escaped.replace("'", "'\"'\"'");
    return "'" + escaped + "'";
}

QString shellJoin(const QStringList &arguments)
{
    QStringList quoted_arguments;
    for (const QString &argument : arguments)
    {
        quoted_arguments << shellQuote(argument);
    }
    return quoted_arguments.join(" ");
}

QStringList splitCommandLine(const QString &command)
{
    QStringList tokens;
    QString current;
    bool in_single_quote = false;
    bool in_double_quote = false;
    bool escaping = false;

    for (const QChar ch : command)
    {
        if (escaping)
        {
            current.append(ch);
            escaping = false;
            continue;
        }
        if (ch == '\\' && !in_single_quote)
        {
            escaping = true;
            continue;
        }
        if (ch == '\'' && !in_double_quote)
        {
            in_single_quote = !in_single_quote;
            continue;
        }
        if (ch == '"' && !in_single_quote)
        {
            in_double_quote = !in_double_quote;
            continue;
        }
        if (ch.isSpace() && !in_single_quote && !in_double_quote)
        {
            if (!current.isEmpty())
            {
                tokens << current;
                current.clear();
            }
            continue;
        }
        current.append(ch);
    }

    if (escaping)
    {
        current.append('\\');
    }
    if (!current.isEmpty())
    {
        tokens << current;
    }
    return tokens;
}

CpuSample readCpuSample()
{
    std::ifstream file("/proc/stat");
    std::string label;
    unsigned long long user = 0;
    unsigned long long nice = 0;
    unsigned long long system = 0;
    unsigned long long idle = 0;
    unsigned long long iowait = 0;
    unsigned long long irq = 0;
    unsigned long long softirq = 0;
    unsigned long long steal = 0;

    if (!(file >> label) || label != "cpu")
    {
        return {};
    }
    file >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal;

    CpuSample sample;
    sample.idle = idle + iowait;
    sample.total = user + nice + system + sample.idle + irq + softirq + steal;
    sample.valid = sample.total > 0;
    return sample;
}

double computeCpuUsagePercent(const CpuSample &previous, const CpuSample &current)
{
    if (!previous.valid || !current.valid || current.total <= previous.total)
    {
        return -1.0;
    }

    const double total_delta = static_cast<double>(current.total - previous.total);
    const double idle_delta = static_cast<double>(current.idle - previous.idle);
    if (total_delta <= 0.0)
    {
        return -1.0;
    }
    return std::max(0.0, std::min(100.0, (1.0 - idle_delta / total_delta) * 100.0));
}

bool readMemoryUsage(double &used_percent, double &used_mb, double &total_mb)
{
    std::ifstream file("/proc/meminfo");
    std::string key;
    std::string unit;
    unsigned long long value_kb = 0;
    unsigned long long total_kb = 0;
    unsigned long long available_kb = 0;

    while (file >> key >> value_kb >> unit)
    {
        if (key == "MemTotal:")
        {
            total_kb = value_kb;
        }
        else if (key == "MemAvailable:")
        {
            available_kb = value_kb;
        }
    }

    if (total_kb == 0 || available_kb == 0 || available_kb > total_kb)
    {
        return false;
    }

    const unsigned long long used_kb = total_kb - available_kb;
    total_mb = static_cast<double>(total_kb) / 1024.0;
    used_mb = static_cast<double>(used_kb) / 1024.0;
    used_percent = static_cast<double>(used_kb) * 100.0 / static_cast<double>(total_kb);
    return true;
}

pid_t readPidFile(const QString &path)
{
    std::ifstream file(path.toStdString());
    long pid = 0;
    file >> pid;
    return pid > 0 ? static_cast<pid_t>(pid) : static_cast<pid_t>(-1);
}

bool isProcessAlive(const pid_t pid)
{
    if (pid <= 0)
    {
        return false;
    }
    if (::kill(pid, 0) == 0)
    {
        return true;
    }
    return errno == EPERM;
}

std::string readFirstLine(const QString &path)
{
    std::ifstream file(path.toStdString());
    std::string line;
    std::getline(file, line);
    return line;
}

QString sanitizeFileToken(QString text)
{
    text.replace("/", "_");
    text.replace("\\", "_");
    text.replace(" ", "_");
    text.replace(":", "_");
    return text;
}

} // namespace

class SunrayLauncherPanel : public QWidget
{
public:
    SunrayLauncherPanel(ros::NodeHandle private_nh, QWidget *parent = nullptr)
        : QWidget(parent), private_nh_(private_nh)
    {
        private_nh_.param("launcher_config", launcher_config_path_, std::string(""));

        loadLaunchItemsFromRosParam();
        buildUi();

        refresh_timer_ = new QTimer(this);
        connect(refresh_timer_, &QTimer::timeout, this, [this]() {
            refreshRuntimeUi();
            ros::spinOnce();
        });
        refresh_timer_->start(300);
    }

    ~SunrayLauncherPanel() override
    {
        stopAllLaunches();
    }

private:
    void loadLaunchItemsFromRosParam()
    {
        XmlRpc::XmlRpcValue groups;
        if (!private_nh_.getParam("launch_groups", groups) ||
            groups.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_WARN("simulation_tools: no valid ~launch_groups, launcher panel will start empty.");
            return;
        }

        for (int i = 0; i < groups.size(); ++i)
        {
            XmlRpc::XmlRpcValue group = groups[i];
            if (group.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
                !group.hasMember("name") || !group.hasMember("items"))
            {
                continue;
            }

            const std::string group_name = xmlRpcToString(group["name"], "unknown");
            XmlRpc::XmlRpcValue items = group["items"];
            if (items.getType() != XmlRpc::XmlRpcValue::TypeArray)
            {
                continue;
            }

            LaunchGroup launch_group;
            launch_group.name = group_name;
            for (int j = 0; j < items.size(); ++j)
            {
                XmlRpc::XmlRpcValue item_value = items[j];
                if (item_value.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
                    !item_value.hasMember("title") ||
                    !item_value.hasMember("package") ||
                    !item_value.hasMember("launch"))
                {
                    continue;
                }

                LaunchItem item;
                item.group = group_name;
                item.title = xmlRpcToString(item_value["title"]);
                item.package = xmlRpcToString(item_value["package"]);
                item.launch = xmlRpcToString(item_value["launch"]);
                if (item_value.hasMember("description"))
                {
                    item.description = xmlRpcToString(item_value["description"]);
                }
                if (item_value.hasMember("args"))
                {
                    item.args = xmlRpcToStringList(item_value["args"]);
                }
                launch_group.item_indices.push_back(static_cast<int>(launch_items_.size()));
                launch_items_.push_back(item);
            }
            launch_groups_.push_back(launch_group);
        }
    }

    void buildUi()
    {
        setWindowTitle("Sunray Launcher");
        const QString package_dir = QString::fromStdString(ros::package::getPath("simulation_tools"));
        const QString logo_path = package_dir + "/logo/yundrone_logo.png";
        const QString icon_path = package_dir + "/logo/yundrone_small_logo.jpeg";
        if (QFileInfo::exists(icon_path))
        {
            setWindowIcon(QIcon(icon_path));
        }
        resize(1380, 820);

        auto *root = new QVBoxLayout(this);
        root->setContentsMargins(14, 14, 14, 14);
        root->setSpacing(10);

        auto *main_splitter = new QSplitter(Qt::Horizontal);
        auto *left_panel = new QWidget();
        auto *left_layout = new QVBoxLayout(left_panel);
        left_layout->setContentsMargins(0, 0, 0, 0);
        left_layout->setSpacing(10);

        auto *header = new QWidget();
        auto *header_layout = new QHBoxLayout(header);
        header_layout->setContentsMargins(0, 0, 0, 0);
        header_layout->setSpacing(14);

        auto *logo = new QLabel();
        logo->setObjectName("logoLabel");
        if (QFileInfo::exists(logo_path))
        {
            logo->setPixmap(QPixmap(logo_path).scaledToHeight(56, Qt::SmoothTransformation));
        }
        logo->setMinimumWidth(210);
        logo->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);

        auto *title = new QLabel("Sunray启动器");
        title->setObjectName("titleLabel");
        header_layout->addWidget(logo);
        header_layout->addWidget(title);
        header_layout->addStretch();
        left_layout->addWidget(header);

        auto *splitter = new QSplitter(Qt::Vertical);
        splitter->addWidget(buildLaunchTree());
        splitter->addWidget(buildDetailPanel());
        splitter->addWidget(buildInfoPanel());
        splitter->setStretchFactor(0, 5);
        splitter->setStretchFactor(1, 1);
        splitter->setStretchFactor(2, 2);
        splitter->setSizes({470, 150, 220});
        left_layout->addWidget(splitter, 1);

        QWidget *monitor_panel = buildSystemMonitorPanel();
        main_splitter->addWidget(left_panel);
        main_splitter->addWidget(monitor_panel);
        main_splitter->setStretchFactor(0, 5);
        main_splitter->setStretchFactor(1, 2);
        main_splitter->setSizes({940, 360});
        root->addWidget(main_splitter, 1);

        setStyleSheet(R"(
            QWidget { background: #eef2ed; color: #17211c; font-family: "Noto Sans CJK SC", "Microsoft YaHei", sans-serif; font-size: 13px; }
            #titleLabel { font-size: 24px; font-weight: 800; color: #16352d; padding: 4px 0px 8px 2px; }
            QGroupBox { border: 1px solid #bac7bf; border-radius: 10px; margin-top: 12px; background: #f8faf7; font-weight: 700; }
            QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0px 6px; color: #31584d; }
            QTabWidget::pane { border: 1px solid #b9c7be; border-radius: 10px; background: #fbfcfa; top: -1px; }
            QTabBar::tab { background: #dde7e0; color: #31584d; border: 1px solid #b9c7be; border-bottom: 0px; padding: 8px 12px; margin-right: 2px; border-top-left-radius: 8px; border-top-right-radius: 8px; font-weight: 700; }
            QTabBar::tab:selected { background: #fbfcfa; color: #16352d; }
            QTabBar::tab:hover { background: #e9f1ec; }
            QTreeWidget { background: #fbfcfa; border: 0px; border-radius: 10px; outline: 0; }
            QTreeWidget::item { padding: 7px; }
            QTreeWidget::item:selected { background: #cfe7db; color: #11241e; }
            QTextEdit { background: #111815; color: #d8f3df; border-radius: 10px; border: 1px solid #23342d; font-family: "JetBrains Mono", "DejaVu Sans Mono", monospace; }
            QPushButton { background: #1f7a5b; color: white; border: 0px; border-radius: 8px; padding: 8px 14px; font-weight: 700; }
            QPushButton:hover { background: #28916c; }
            QPushButton:disabled { background: #9caaa3; color: #edf1ee; }
            QPushButton#stopButton { background: #b74e35; }
            QPushButton#stopButton:hover { background: #ce5b3e; }
            QPushButton#secondaryButton { background: #4d675d; }
            QPushButton#secondaryButton:hover { background: #5c7a6f; }
            QLabel#stateLabel { color: #16352d; font-weight: 700; }
            QLabel#mutedLabel { color: #63736b; }
        )");

        selectFirstLaunchItem();
    }

    QWidget *buildLaunchTree()
    {
        auto *box = new QGroupBox("模块列表");
        auto *layout = new QVBoxLayout(box);

        launch_tabs_ = new QTabWidget();
        for (const LaunchGroup &group : launch_groups_)
        {
            auto *tree = new QTreeWidget();
            tree->setColumnCount(4);
            tree->setHeaderLabels(QStringList() << "Launch" << "状态" << "Package" << "文件");
            tree->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
            tree->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
            tree->header()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
            tree->header()->setSectionResizeMode(3, QHeaderView::Stretch);

            for (const int idx : group.item_indices)
            {
                if (idx < 0 || idx >= static_cast<int>(launch_items_.size()))
                {
                    continue;
                }
                const LaunchItem &item = launch_items_[static_cast<size_t>(idx)];
                auto *tree_item = new QTreeWidgetItem(tree);
                tree_item->setText(0, QString::fromStdString(item.title));
                tree_item->setText(1, "未运行");
                tree_item->setText(2, QString::fromStdString(item.package));
                tree_item->setText(3, QString::fromStdString(item.launch));
                tree_item->setData(0, kLaunchIndexRole, idx);
                item_by_launch_index_[idx] = tree_item;
            }

            connect(tree, &QTreeWidget::currentItemChanged, this,
                    [this](QTreeWidgetItem *current, QTreeWidgetItem *) { onSelectionChanged(current); });

            launch_trees_.push_back(tree);
            launch_tabs_->addTab(tree, QString::fromStdString(group.name));
        }

        connect(launch_tabs_, &QTabWidget::currentChanged, this, [this](int tab_index) {
            if (tab_index < 0 || tab_index >= static_cast<int>(launch_trees_.size()))
            {
                onSelectionChanged(nullptr);
                return;
            }
            QTreeWidget *tree = launch_trees_[static_cast<size_t>(tab_index)];
            if (tree && tree->topLevelItemCount() > 0 && !tree->currentItem())
            {
                tree->setCurrentItem(tree->topLevelItem(0));
                return;
            }
            onSelectionChanged(tree ? tree->currentItem() : nullptr);
        });

        layout->addWidget(launch_tabs_);
        return box;
    }

    QWidget *buildDetailPanel()
    {
        auto *box = new QGroupBox("启动控制");
        auto *layout = new QVBoxLayout(box);

        selected_title_ = new QLabel("未选择 Launch");
        selected_title_->setObjectName("stateLabel");
        selected_title_->setWordWrap(false);
        selected_command_ = new QTextEdit();
        selected_command_->setPlaceholderText("roslaunch localization_fusion localization_fusion.launch source_id:=5 agent_name:=uav agent_id:=1");
        selected_command_->setMinimumHeight(38);
        selected_command_->setMaximumHeight(46);

        auto *top_row = new QHBoxLayout();
        start_btn_ = new QPushButton("启动");
        stop_btn_ = new QPushButton("停止");
        stop_btn_->setObjectName("stopButton");
        top_row->addWidget(selected_title_, 1);
        top_row->addWidget(start_btn_);
        top_row->addWidget(stop_btn_);

        connect(start_btn_, &QPushButton::clicked, this, [this]() { startSelectedLaunch(); });
        connect(stop_btn_, &QPushButton::clicked, this, [this]() { stopSelectedLaunch(); });

        layout->addLayout(top_row);
        layout->addWidget(new QLabel("启动命令（可编辑，最终执行此命令）"));
        layout->addWidget(selected_command_);
        updateButtons();
        return box;
    }

    QWidget *buildInfoPanel()
    {
        auto *box = new QGroupBox("INFO");
        auto *layout = new QVBoxLayout(box);

        log_view_ = new QTextEdit();
        log_view_->setReadOnly(true);
        layout->addWidget(log_view_);
        return box;
    }

    QWidget *buildSystemMonitorPanel()
    {
        auto *box = new QGroupBox("系统状态监控");
        auto *layout = new QGridLayout(box);
        box->setMinimumWidth(340);
        box->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
        layout->setContentsMargins(12, 18, 12, 12);
        layout->setVerticalSpacing(8);

        summary_label_ = new QLabel("配置项: --  运行中: --");
        summary_label_->setObjectName("stateLabel");
        cpu_label_ = new QLabel("CPU: --");
        memory_label_ = new QLabel("内存: --");
        ros_node_count_label_ = new QLabel("ROS nodes: --");

        cpu_bar_ = new QProgressBar();
        cpu_bar_->setRange(0, 100);
        cpu_bar_->setTextVisible(false);
        memory_bar_ = new QProgressBar();
        memory_bar_->setRange(0, 100);
        memory_bar_->setTextVisible(false);

        ros_nodes_view_ = new QTextEdit();
        ros_nodes_view_->setReadOnly(true);
        ros_nodes_view_->setMinimumWidth(310);
        ros_nodes_view_->setMinimumHeight(560);
        ros_nodes_view_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

        auto *button_row = new QHBoxLayout();
        auto *stop_all_btn = new QPushButton("停止全部 Launch");
        stop_all_btn->setObjectName("stopButton");
        auto *clear_log_btn = new QPushButton("清空日志");
        clear_log_btn->setObjectName("secondaryButton");
        button_row->addWidget(stop_all_btn);
        button_row->addWidget(clear_log_btn);

        connect(stop_all_btn, &QPushButton::clicked, this, [this]() { stopAllLaunches(); });
        connect(clear_log_btn, &QPushButton::clicked, this, [this]() {
            if (log_view_)
            {
                log_view_->clear();
            }
        });

        layout->addWidget(summary_label_, 0, 0, 1, 2);
        layout->addWidget(cpu_label_, 1, 0);
        layout->addWidget(cpu_bar_, 1, 1);
        layout->addWidget(memory_label_, 2, 0);
        layout->addWidget(memory_bar_, 2, 1);
        layout->addWidget(ros_node_count_label_, 3, 0, 1, 2);
        layout->addWidget(ros_nodes_view_, 4, 0, 1, 2);
        layout->addLayout(button_row, 5, 0, 1, 2);
        layout->setColumnStretch(1, 1);
        layout->setRowStretch(4, 1);
        return box;
    }

    int selectedLaunchIndex() const
    {
        QTreeWidgetItem *item = nullptr;
        if (launch_tabs_ && launch_tabs_->currentIndex() >= 0 &&
            launch_tabs_->currentIndex() < static_cast<int>(launch_trees_.size()))
        {
            QTreeWidget *tree = launch_trees_[static_cast<size_t>(launch_tabs_->currentIndex())];
            item = tree ? tree->currentItem() : nullptr;
        }
        if (!item)
        {
            return -1;
        }
        const QVariant value = item->data(0, kLaunchIndexRole);
        return value.isValid() ? value.toInt() : -1;
    }

    void selectFirstLaunchItem()
    {
        if (!launch_tabs_ || launch_trees_.empty())
        {
            onSelectionChanged(nullptr);
            return;
        }

        launch_tabs_->setCurrentIndex(0);
        QTreeWidget *tree = launch_trees_.front();
        if (tree && tree->topLevelItemCount() > 0)
        {
            tree->setCurrentItem(tree->topLevelItem(0));
        }
        else
        {
            onSelectionChanged(nullptr);
        }
    }

    void startSelectedLaunch()
    {
        const int idx = selectedLaunchIndex();
        if (idx >= 0)
        {
            startLaunch(idx);
        }
    }

    void stopSelectedLaunch()
    {
        const int idx = selectedLaunchIndex();
        if (idx >= 0)
        {
            stopLaunch(idx);
        }
    }

    void startLaunch(const int idx)
    {
        if (idx < 0 || idx >= static_cast<int>(launch_items_.size()))
        {
            return;
        }
        if (isRunning(idx))
        {
            appendLog("Launch 已在运行: " + launch_items_[static_cast<size_t>(idx)].title);
            return;
        }

        const LaunchItem &item = launch_items_[static_cast<size_t>(idx)];
        LaunchCommand command;
        if (!parseEditedCommand(command))
        {
            appendLog("启动失败: 启动命令格式错误，应为 roslaunch <package> <launch> [args...]");
            updateLaunchStatus(idx, "未运行");
            updateButtons();
            return;
        }

        const std::string launch_file = resolveLaunchFilePath(command.package_name, command.launch_file);
        if (!QFileInfo::exists(QString::fromStdString(launch_file)))
        {
            appendLog("启动失败: 找不到 launch 文件 " + launch_file);
            updateLaunchStatus(idx, "未运行");
            updateButtons();
            return;
        }

        const QStringList roslaunch_arguments = buildRoslaunchArguments(launch_file, command.args);
        LaunchRuntime runtime;
        runtime.command = shellJoin(QStringList() << "roslaunch" << roslaunch_arguments);
        const QString display_command = selected_command_ ? selected_command_->toPlainText().simplified() : runtime.command;
        runtime.start_time = QDateTime::currentDateTime();
        if (!prepareTerminalRuntime(idx, QString::fromStdString(item.title), runtime))
        {
            appendLog("启动失败: 无法创建 terminal wrapper。");
            updateLaunchStatus(idx, "未运行");
            updateButtons();
            return;
        }

        auto *process = new QProcess(this);
        const QStringList terminal_command = buildTerminalCommand(runtime);
        if (terminal_command.isEmpty())
        {
            appendLog("启动失败: 没有找到可用终端，请安装 gnome-terminal、konsole 或 xterm。");
            cleanupRuntimeFiles(runtime);
            updateLaunchStatus(idx, "未运行");
            updateButtons();
            return;
        }

        process->setProgram(terminal_command.first());
        process->setArguments(terminal_command.mid(1));
        connect(process,
                static_cast<void (QProcess::*)(int, QProcess::ExitStatus)>(&QProcess::finished),
                this,
                [this, process, idx](int exit_code, QProcess::ExitStatus exit_status) {
                    if (exit_code != 0 || exit_status != QProcess::NormalExit)
                    {
                        appendLog("终端启动器异常退出: " + launch_items_[static_cast<size_t>(idx)].title +
                                  " exit_code=" + std::to_string(exit_code) +
                                  " status=" + (exit_status == QProcess::NormalExit ? "normal" : "crashed"));
                    }
                    terminal_processes_.erase(idx);
                    process->deleteLater();
                    updateButtons();
                });

        launch_runtimes_[idx] = runtime;
        terminal_processes_[idx] = process;
        process->start();
        if (!process->waitForStarted(1500))
        {
            appendLog("启动失败: " + item.title + "  error=" + process->errorString().toStdString());
            terminal_processes_.erase(idx);
            process->deleteLater();
            cleanupRuntimeFiles(runtime);
            updateLaunchStatus(idx, "未运行");
            updateButtons();
            return;
        }

        launch_runtimes_[idx].terminal_pid = process->processId();
        launch_runtimes_[idx].running = true;
        updateLaunchStatus(idx, "运行 0s");
        appendLog("已在 terminal 中启动: " + display_command.toStdString());
        updateButtons();
    }

    QStringList buildRoslaunchArguments(const std::string &launch_file, const QStringList &edited_args) const
    {
        QStringList args;
        args << QString::fromStdString(launch_file);
        for (const QString &arg : edited_args)
        {
            args << arg;
        }
        return args;
    }

    QString buildDisplayCommand(const LaunchItem &item) const
    {
        QStringList args;
        args << "roslaunch"
             << QString::fromStdString(item.package)
             << QString::fromStdString(item.launch);
        for (const std::string &arg : item.args)
        {
            args << QString::fromStdString(arg);
        }
        return args.join(" ");
    }

    bool parseEditedCommand(LaunchCommand &command) const
    {
        if (!selected_command_)
        {
            return false;
        }

        const QStringList tokens = splitCommandLine(selected_command_->toPlainText().simplified());
        if (tokens.size() < 3 || tokens[0] != "roslaunch")
        {
            return false;
        }

        command.package_name = tokens[1].toStdString();
        command.launch_file = tokens[2].toStdString();
        command.args = tokens.mid(3);
        return !command.package_name.empty() && !command.launch_file.empty();
    }

    bool prepareTerminalRuntime(const int idx, const QString &title, LaunchRuntime &runtime) const
    {
        QDir runtime_dir(QDir::tempPath() + "/sunray_launcher_panel");
        if (!runtime_dir.exists() && !runtime_dir.mkpath("."))
        {
            return false;
        }

        const QString base_name = QString("launch_%1_%2_%3")
                                      .arg(idx)
                                      .arg(QDateTime::currentMSecsSinceEpoch())
                                      .arg(sanitizeFileToken(title));
        runtime.script_path = runtime_dir.filePath(base_name + ".sh");
        runtime.pid_file = runtime_dir.filePath(base_name + ".pid");
        runtime.done_file = runtime_dir.filePath(base_name + ".done");

        std::ofstream script(runtime.script_path.toStdString());
        if (!script)
        {
            return false;
        }

        script << "#!/usr/bin/env bash\n";
        script << "set +e\n";
        script << "rm -f " << shellQuote(runtime.pid_file).toStdString() << " "
               << shellQuote(runtime.done_file).toStdString() << "\n";
        script << "echo \"[Sunray] Launch: " << title.toStdString() << "\"\n";
        script << "echo \"[Sunray] Command: " << runtime.command.toStdString() << "\"\n";
        script << "echo\n";
        script << "echo \"$$\" > " << shellQuote(runtime.pid_file).toStdString() << "\n";
        script << "roslaunch_pid=\"\"\n";
        script << "cleanup() {\n";
        script << "  if [ -n \"$roslaunch_pid\" ] && kill -0 \"$roslaunch_pid\" 2>/dev/null; then\n";
        script << "    kill -INT \"$roslaunch_pid\" 2>/dev/null\n";
        script << "    wait \"$roslaunch_pid\" 2>/dev/null\n";
        script << "  fi\n";
        script << "}\n";
        script << "trap 'cleanup; exit 130' INT TERM HUP\n";
        script << "bash -lc " << shellQuote(runtime.command).toStdString() << " &\n";
        script << "roslaunch_pid=$!\n";
        script << "wait \"$roslaunch_pid\"\n";
        script << "exit_code=$?\n";
        script << "trap - INT TERM HUP\n";
        script << "echo \"$exit_code\" > " << shellQuote(runtime.done_file).toStdString() << "\n";
        script.close();

        if (::chmod(runtime.script_path.toStdString().c_str(), 0755) != 0)
        {
            return false;
        }
        return true;
    }

    QStringList buildTerminalCommand(LaunchRuntime &runtime) const
    {
        const QString title = "Sunray Launcher";
        if (!QStandardPaths::findExecutable("gnome-terminal").isEmpty())
        {
            runtime.terminal_program = "gnome-terminal";
            return QStringList() << "gnome-terminal"
                                 << "--title" << title
                                 << "--"
                                 << "bash"
                                 << "-lc"
                                 << shellQuote(runtime.script_path);
        }
        if (!QStandardPaths::findExecutable("konsole").isEmpty())
        {
            runtime.terminal_program = "konsole";
            return QStringList() << "konsole"
                                 << "--new-tab"
                                 << "-p" << QString("tabtitle=%1").arg(title)
                                 << "-e"
                                 << "bash"
                                 << "-lc"
                                 << shellQuote(runtime.script_path);
        }
        if (!QStandardPaths::findExecutable("xterm").isEmpty())
        {
            runtime.terminal_program = "xterm";
            return QStringList() << "xterm"
                                 << "-T" << title
                                 << "-e"
                                 << "bash"
                                 << "-lc"
                                 << shellQuote(runtime.script_path);
        }
        return {};
    }

    std::string resolveLaunchFilePath(const std::string &package_name, const std::string &launch_file) const
    {
        const QFileInfo launch_info(QString::fromStdString(launch_file));
        if (launch_info.isAbsolute())
        {
            return launch_info.absoluteFilePath().toStdString();
        }

        const std::string package_path = ros::package::getPath(package_name);
        if (package_path.empty())
        {
            return launch_file;
        }

        const QString package_dir = QString::fromStdString(package_path);
        const QFileInfo standard_launch_path(package_dir + "/launch/" + QString::fromStdString(launch_file));
        if (standard_launch_path.exists())
        {
            return standard_launch_path.absoluteFilePath().toStdString();
        }

        return QFileInfo(package_dir + "/" + QString::fromStdString(launch_file))
            .absoluteFilePath()
            .toStdString();
    }

    bool isRunning(const int idx) const
    {
        const auto it = launch_runtimes_.find(idx);
        if (it == launch_runtimes_.end() || !it->second.running)
        {
            return false;
        }

        const auto terminal_it = terminal_processes_.find(idx);
        if (terminal_it != terminal_processes_.end() &&
            terminal_it->second &&
            terminal_it->second->state() == QProcess::NotRunning)
        {
            return false;
        }

        const pid_t roslaunch_pid = readPidFile(it->second.pid_file);
        if (isProcessAlive(roslaunch_pid))
        {
            return true;
        }

        const bool pid_file_is_still_warming_up =
            roslaunch_pid <= 0 &&
            it->second.start_time.msecsTo(QDateTime::currentDateTime()) < kPidFileWarmupMsec &&
            !QFileInfo::exists(it->second.done_file);
        return pid_file_is_still_warming_up;
    }

    void stopLaunch(const int idx)
    {
        const auto it = launch_runtimes_.find(idx);
        if (it == launch_runtimes_.end())
        {
            return;
        }

        appendLog("停止 Launch: " + launch_items_[static_cast<size_t>(idx)].title);
        launch_runtimes_[idx].stop_requested = true;
        const pid_t roslaunch_pid = readPidFile(it->second.pid_file);
        if (roslaunch_pid > 0)
        {
            ::kill(-roslaunch_pid, SIGINT);
            ::kill(roslaunch_pid, SIGINT);
            appendLog("已发送 SIGINT 到 roslaunch 进程组: pid=" + std::to_string(roslaunch_pid));
            return;
        }

        const auto terminal_it = terminal_processes_.find(idx);
        if (terminal_it != terminal_processes_.end() && terminal_it->second)
        {
            terminal_it->second->terminate();
        }
    }

    void stopAllLaunches()
    {
        std::vector<int> running;
        for (const auto &entry : launch_runtimes_)
        {
            if (isRunning(entry.first))
            {
                running.push_back(entry.first);
            }
        }
        for (const int idx : running)
        {
            stopLaunch(idx);
        }
    }

    void cleanupRuntimeFiles(const LaunchRuntime &runtime) const
    {
        QFile::remove(runtime.script_path);
        QFile::remove(runtime.pid_file);
        QFile::remove(runtime.done_file);
    }

    void onSelectionChanged(QTreeWidgetItem *current)
    {
        int idx = -1;
        if (current)
        {
            const QVariant value = current->data(0, kLaunchIndexRole);
            if (value.isValid())
            {
                idx = value.toInt();
            }
        }
        if (idx >= 0 && idx < static_cast<int>(launch_items_.size()))
        {
            const LaunchItem &item = launch_items_[static_cast<size_t>(idx)];
            selected_title_->setText(QString("%1 | %2")
                                         .arg(QString::fromStdString(item.title))
                                         .arg(QString::fromStdString(item.description)));
            selected_command_->setPlainText(buildDisplayCommand(item));
        }
        else
        {
            selected_title_->setText("未选择 Launch");
            selected_command_->setPlainText("-");
        }
        updateButtons();
    }

    void refreshRuntimeUi()
    {
        int running_count = 0;
        std::vector<int> finished_launches;
        for (auto &entry : launch_runtimes_)
        {
            const int idx = entry.first;
            if (!isRunning(idx))
            {
                if (entry.second.running)
                {
                    const std::string exit_code = readFirstLine(entry.second.done_file);
                    updateLaunchStatus(idx, "未运行");
                    appendLog("Launch 结束: " + launch_items_[static_cast<size_t>(idx)].title +
                              (exit_code.empty() ? "" : " exit_code=" + exit_code));
                    entry.second.running = false;
                    finished_launches.push_back(idx);
                }
                continue;
            }

            ++running_count;
            const qint64 secs = entry.second.start_time.secsTo(QDateTime::currentDateTime());
            updateLaunchStatus(idx, QString("运行 %1s").arg(secs));
        }
        for (const int idx : finished_launches)
        {
            const auto it = launch_runtimes_.find(idx);
            if (it != launch_runtimes_.end())
            {
                cleanupRuntimeFiles(it->second);
                launch_runtimes_.erase(it);
            }
        }

        summary_label_->setText(QString("配置项: %1  运行中: %2")
                                    .arg(launch_items_.size())
                                    .arg(running_count));
        updateButtons();

        const QDateTime now = QDateTime::currentDateTime();
        if (!last_monitor_refresh_.isValid() || last_monitor_refresh_.msecsTo(now) >= 1000)
        {
            refreshSystemMonitor();
            last_monitor_refresh_ = now;
        }
    }

    void refreshSystemMonitor()
    {
        const CpuSample current_cpu_sample = readCpuSample();
        const double cpu_percent = computeCpuUsagePercent(last_cpu_sample_, current_cpu_sample);
        last_cpu_sample_ = current_cpu_sample;
        if (cpu_label_ && cpu_bar_)
        {
            if (cpu_percent >= 0.0)
            {
                cpu_label_->setText(QString("CPU: %1%").arg(cpu_percent, 0, 'f', 1));
                cpu_bar_->setValue(static_cast<int>(std::round(cpu_percent)));
            }
            else
            {
                cpu_label_->setText("CPU: 采样中");
                cpu_bar_->setValue(0);
            }
        }

        double memory_percent = 0.0;
        double memory_used_mb = 0.0;
        double memory_total_mb = 0.0;
        if (memory_label_ && memory_bar_)
        {
            if (readMemoryUsage(memory_percent, memory_used_mb, memory_total_mb))
            {
                memory_label_->setText(QString("内存: %1 / %2 MB  (%3%)")
                                           .arg(memory_used_mb, 0, 'f', 0)
                                           .arg(memory_total_mb, 0, 'f', 0)
                                           .arg(memory_percent, 0, 'f', 1));
                memory_bar_->setValue(static_cast<int>(std::round(memory_percent)));
            }
            else
            {
                memory_label_->setText("内存: 读取失败");
                memory_bar_->setValue(0);
            }
        }

        std::vector<std::string> running_nodes;
        const bool got_nodes = ros::master::getNodes(running_nodes);
        std::sort(running_nodes.begin(), running_nodes.end());
        if (ros_node_count_label_)
        {
            ros_node_count_label_->setText(got_nodes
                                               ? QString("ROS nodes: %1 个正在运行").arg(running_nodes.size())
                                               : "ROS nodes: 无法连接 ROS master");
        }
        if (ros_nodes_view_)
        {
            QStringList lines;
            if (got_nodes)
            {
                for (const std::string &node : running_nodes)
                {
                    lines << QString::fromStdString(node);
                }
            }
            else
            {
                lines << "无法从 ROS master 获取节点列表";
            }
            ros_nodes_view_->setPlainText(lines.join("\n"));
        }
    }

    void updateButtons()
    {
        const int idx = selectedLaunchIndex();
        const bool valid = idx >= 0;
        const bool running = valid && isRunning(idx);
        start_btn_->setEnabled(valid && !running);
        stop_btn_->setEnabled(running);
    }

    void updateLaunchStatus(const int idx, const QString &status)
    {
        const auto it = item_by_launch_index_.find(idx);
        if (it != item_by_launch_index_.end() && it->second)
        {
            it->second->setText(1, status);
            const bool running = status.startsWith("运行");
            const QColor row_background = running ? QColor(198, 235, 211) : QColor(251, 252, 250);
            const QColor row_foreground = running ? QColor(20, 84, 50) : QColor(23, 33, 28);
            for (int column = 0; column < it->second->columnCount(); ++column)
            {
                it->second->setBackground(column, row_background);
                it->second->setForeground(column, row_foreground);
            }
        }
    }

    void appendLog(const std::string &text)
    {
        if (!log_view_)
        {
            return;
        }
        const QString time = QDateTime::currentDateTime().toString("HH:mm:ss");
        log_view_->append(QString("<span style='color:#8ccfac'>[%1]</span> %2")
                              .arg(time)
                              .arg(htmlEscape(text)));
        log_view_->moveCursor(QTextCursor::End);
    }

    ros::NodeHandle private_nh_;

    std::string launcher_config_path_;

    std::vector<LaunchItem> launch_items_;
    std::vector<LaunchGroup> launch_groups_;
    std::map<int, QTreeWidgetItem *> item_by_launch_index_;
    std::map<int, QProcess *> terminal_processes_;
    std::map<int, LaunchRuntime> launch_runtimes_;

    QTimer *refresh_timer_{nullptr};
    QDateTime last_monitor_refresh_;
    CpuSample last_cpu_sample_;

    QTabWidget *launch_tabs_{nullptr};
    std::vector<QTreeWidget *> launch_trees_;
    QLabel *summary_label_{nullptr};
    QLabel *selected_title_{nullptr};
    QTextEdit *selected_command_{nullptr};
    QTextEdit *log_view_{nullptr};
    QLabel *cpu_label_{nullptr};
    QLabel *memory_label_{nullptr};
    QLabel *ros_node_count_label_{nullptr};
    QProgressBar *cpu_bar_{nullptr};
    QProgressBar *memory_bar_{nullptr};
    QTextEdit *ros_nodes_view_{nullptr};
    QPushButton *start_btn_{nullptr};
    QPushButton *stop_btn_{nullptr};
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sunray_launcher_node");
    QApplication app(argc, argv);
    ros::NodeHandle private_nh("~");

    SunrayLauncherPanel panel(private_nh);
    panel.show();

    return app.exec();
}
