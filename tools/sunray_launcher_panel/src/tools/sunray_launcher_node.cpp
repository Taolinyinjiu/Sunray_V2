#include <ros/master.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <XmlRpcValue.h>

#include <QApplication>
#include <QDateTime>
#include <QDir>
#include <QDirIterator>
#include <QFile>
#include <QFileInfo>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QIcon>
#include <QLabel>
#include <QFrame>
#include <QPainter>
#include <QPixmap>
#include <QColor>
#include <QProcess>
#include <QProgressBar>
#include <QPushButton>
#include <QSizePolicy>
#include <QStringList>
#include <QRegularExpression>
#include <QSplitter>
#include <QStandardPaths>
#include <QTabBar>
#include <QStyledItemDelegate>
#include <QStyleOptionViewItem>
#include <QTextCursor>
#include <QTextEdit>
#include <QTextOption>
#include <QTimer>
#include <QTreeWidget>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QWidget>
#include <QMouseEvent>

#include <algorithm>
#include <cerrno>
#include <cctype>
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
constexpr int kQuickLaunchIndexRole = Qt::UserRole + 2;
constexpr int kOriginalTitleRole = Qt::UserRole + 3;
constexpr int kBadgeStateRole = Qt::UserRole + 4;
constexpr int kPidFileWarmupMsec = 3000;

enum class BadgeState
{
    Idle = 0,
    Preview = 1,
    Running = 2,
};

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

struct QuickLaunchStep
{
    std::string ref;
    std::string title;
    std::string package;
    std::string launch;
    std::string command;
    std::vector<std::string> args;
    double delay_sec{0.0};
    int linked_launch_index{-1};
};

struct QuickLaunchGroup
{
    std::string title;
    std::string category{"sunray_sim"};
    std::string description;
    std::vector<QuickLaunchStep> steps;
};

struct ExternalWorkspace
{
    QString configured_path;
    QString root_path;
    QString devel_path;
    QStringList source_roots;
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

struct QuickLaunchRuntime
{
    LaunchRuntime terminal;
    QStringList step_script_paths;
    QStringList step_titles;
    QStringList step_commands;
    QStringList step_display_commands;
    std::vector<int> linked_launch_indices;
    bool running{false};
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

const QColor kSunrayRunningBg(96, 244, 109);
const QColor kSunrayRunningFg(6, 68, 31);
const QColor kSunrayPreviewBg(255, 232, 173);
const QColor kSunrayPreviewFg(102, 68, 10);
const QColor kSunraySelectedBg(224, 229, 226);
const QColor kSunraySelectedFg(22, 53, 45);
const QColor kSunrayNormalBg(251, 252, 250);
const QColor kSunrayNormalFg(23, 33, 28);

class LaunchTabBar : public QTabBar
{
public:
    explicit LaunchTabBar(QWidget *parent = nullptr) : QTabBar(parent) {}

    void setRunningCount(const int index, const int running_count)
    {
        running_counts_[index] = running_count;
        update();
    }

    void setPreviewCount(const int index, const int preview_count)
    {
        preview_counts_[index] = preview_count;
        update();
    }

protected:
    QSize tabSizeHint(const int index) const override
    {
        QFont tab_font = font();
        tab_font.setBold(true);
        QFontMetrics metrics(tab_font);
        const QSize text_size = metrics.size(Qt::TextSingleLine, tabText(index));
        return QSize(text_size.width() + 36, std::max(text_size.height() + 18, 36));
    }

    void paintEvent(QPaintEvent *) override
    {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing, true);

        for (int i = 0; i < count(); ++i)
        {
            const QRect rect = tabRect(i).adjusted(1, 2, -1, -2);
            const bool selected = (i == currentIndex());
            const int running_count = running_counts_.count(i) > 0 ? running_counts_.at(i) : 0;
            const int preview_count = preview_counts_.count(i) > 0 ? preview_counts_.at(i) : 0;

            const QColor background = running_count > 0
                                          ? kSunrayRunningBg
                                          : (preview_count > 0
                                                 ? kSunrayPreviewBg
                                                 : (selected ? kSunraySelectedBg : QColor(221, 231, 224)));
            const QColor text_color = running_count > 0
                                          ? kSunrayRunningFg
                                          : (preview_count > 0
                                                 ? kSunrayPreviewFg
                                                 : (selected ? kSunraySelectedFg : QColor(49, 88, 77)));

            painter.setPen(QPen(QColor(185, 199, 190), 1));
            painter.setBrush(background);
            painter.drawRoundedRect(rect, 8, 8);

            QFont tab_font = font();
            tab_font.setBold(true);
            painter.setFont(tab_font);
            painter.setPen(text_color);
            painter.drawText(rect.adjusted(16, 3, -16, -4), Qt::AlignCenter | Qt::TextSingleLine, tabText(i));
        }
    }

private:
    std::map<int, int> running_counts_;
    std::map<int, int> preview_counts_;
};

class LaunchTabWidget : public QTabWidget
{
public:
    explicit LaunchTabWidget(QWidget *parent = nullptr) : QTabWidget(parent) {}

    void useLaunchTabBar(QTabBar *tab_bar)
    {
        setTabBar(tab_bar);
    }
};

class ClearableTreeWidget : public QTreeWidget
{
public:
    explicit ClearableTreeWidget(QWidget *parent = nullptr) : QTreeWidget(parent) {}

protected:
    void mousePressEvent(QMouseEvent *event) override
    {
        QTreeWidgetItem *pressed_item = itemAt(event->pos());
        if (pressed_item && pressed_item == currentItem())
        {
            clearSelection();
            setCurrentItem(nullptr);
            return;
        }
        if (!pressed_item)
        {
            clearSelection();
            setCurrentItem(nullptr);
            return;
        }
        QTreeWidget::mousePressEvent(event);
    }
};

class StatusBadgeDelegate : public QStyledItemDelegate
{
public:
    explicit StatusBadgeDelegate(QObject *parent = nullptr) : QStyledItemDelegate(parent) {}

    QSize sizeHint(const QStyleOptionViewItem &option, const QModelIndex &index) const override
    {
        if (index.column() != 1)
        {
            return QStyledItemDelegate::sizeHint(option, index);
        }

        const QString text = index.data(Qt::DisplayRole).toString();
        QFont font = option.font;
        font.setBold(false);
        QFontMetrics metrics(font);
        const QSize base = metrics.size(Qt::TextSingleLine, text);
        return QSize(base.width() + 20, std::max(base.height() + 8, 22));
    }

    void paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const override
    {
        if (index.column() != 1)
        {
            QStyledItemDelegate::paint(painter, option, index);
            return;
        }

        const QString text = index.data(Qt::DisplayRole).toString();
        const int state = index.data(kBadgeStateRole).toInt();

        QColor background = kSunrayNormalBg;
        QColor foreground = kSunrayNormalFg;
        QColor border = QColor(190, 199, 194);
        switch (static_cast<BadgeState>(state))
        {
        case BadgeState::Running:
            background = kSunrayRunningBg;
            foreground = kSunrayRunningFg;
            border = QColor(86, 210, 99);
            break;
        case BadgeState::Preview:
            background = kSunrayPreviewBg;
            foreground = kSunrayPreviewFg;
            border = QColor(228, 190, 90);
            break;
        case BadgeState::Idle:
        default:
            background = QColor(232, 236, 233);
            foreground = QColor(47, 69, 60);
            border = QColor(197, 206, 201);
            break;
        }

        painter->save();
        painter->setRenderHint(QPainter::Antialiasing, true);
        painter->fillRect(option.rect, Qt::transparent);

        QFont font = option.font;
        font.setBold(false);
        painter->setFont(font);
        QFontMetrics metrics(font);
        const QSize text_size = metrics.size(Qt::TextSingleLine, text);
        const int padding_x = 10;
        const int padding_y = 4;
        const int badge_height = std::max(text_size.height() + padding_y, 20);
        const int badge_width = std::min(option.rect.width() - 8, text_size.width() + padding_x * 2);
        QRect badge_rect(0, 0, std::max(badge_width, 44), badge_height);
        badge_rect.moveCenter(option.rect.center());

        painter->setPen(QPen(border, 1));
        painter->setBrush(background);
        painter->drawRoundedRect(badge_rect, badge_rect.height() / 2.0, badge_rect.height() / 2.0);

        painter->setPen(foreground);
        painter->drawText(badge_rect.adjusted(8, 0, -8, 0), Qt::AlignCenter, text);
        painter->restore();
    }
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

QStringList uniquePathList(const QStringList &paths)
{
    QStringList unique_paths;
    for (const QString &path : paths)
    {
        const QString clean_path = QDir(path).absolutePath();
        if (QDir(clean_path).exists() && !unique_paths.contains(clean_path))
        {
            unique_paths << clean_path;
        }
    }
    return unique_paths;
}

QString shellEnvPrepend(const QStringList &paths, const QString &env_name)
{
    const QString prefix = uniquePathList(paths).join(":");
    if (prefix.isEmpty())
    {
        return "";
    }
    return shellQuote(prefix) + "${" + env_name + ":+:${" + env_name + "}}";
}

QString formatRosNodeNameForDisplay(const std::string &node)
{
    const QString text = QString::fromStdString(node);
    const QStringList parts = text.split('/', QString::SkipEmptyParts);
    QString display = text;
    if (parts.size() > 2 && text.size() > 28)
    {
        const QString first_line = "/" + parts[0] + "/" + parts[1];
        const QString rest = parts.mid(2).join("/");
        display = rest.isEmpty() ? first_line : first_line + "\n  " + rest;
    }

    if (display.startsWith('/'))
    {
        // Prevent Qt from breaking the line immediately after the leading slash.
        display.insert(1, QChar(0x2060));
    }
    return display;
}

QString expandUserPath(const std::string &path)
{
    QString expanded = QString::fromStdString(path).trimmed();
    if (expanded == "~")
    {
        return QDir::homePath();
    }
    if (expanded.startsWith("~/"))
    {
        return QDir::homePath() + expanded.mid(1);
    }
    return expanded;
}

void writeTerminalTitleKeeper(std::ofstream &script, const QString &title)
{
    // GNOME Terminal may refresh the tab title from the active child process.
    // Keep a lightweight OSC 0 refresher running so quick-launch tabs show the configured Sunray title.
    script << "sunray_terminal_title=" << shellQuote(title).toStdString() << "\n";
    script << "sunray_title_keeper_pid=\"\"\n";
    script << "set_sunray_terminal_title() {\n";
    script << "  printf '\\033]0;%s\\007' \"$sunray_terminal_title\"\n";
    script << "}\n";
    script << "start_sunray_title_keeper() {\n";
    script << "  while true; do\n";
    script << "    set_sunray_terminal_title\n";
    script << "    sleep 1\n";
    script << "  done &\n";
    script << "  sunray_title_keeper_pid=$!\n";
    script << "}\n";
    script << "stop_sunray_title_keeper() {\n";
    script << "  if [ -n \"${sunray_title_keeper_pid:-}\" ]; then\n";
    script << "    kill \"$sunray_title_keeper_pid\" 2>/dev/null\n";
    script << "    wait \"$sunray_title_keeper_pid\" 2>/dev/null\n";
    script << "    sunray_title_keeper_pid=\"\"\n";
    script << "  fi\n";
    script << "}\n";
    script << "set_sunray_terminal_title\n";
    script << "start_sunray_title_keeper\n";
}

std::string trimCopy(const std::string &text)
{
    size_t begin = 0;
    while (begin < text.size() && std::isspace(static_cast<unsigned char>(text[begin])))
    {
        ++begin;
    }

    size_t end = text.size();
    while (end > begin && std::isspace(static_cast<unsigned char>(text[end - 1])))
    {
        --end;
    }
    return text.substr(begin, end - begin);
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

std::vector<pid_t> readPidListFile(const QString &path)
{
    std::ifstream file(path.toStdString());
    std::vector<pid_t> pids;
    long pid = 0;
    while (file >> pid)
    {
        if (pid > 0)
        {
            pids.push_back(static_cast<pid_t>(pid));
        }
    }
    return pids;
}

QString sanitizeFileToken(QString text)
{
    text.replace("/", "_");
    text.replace("\\", "_");
    text.replace(" ", "_");
    text.replace(":", "_");
    return text;
}

std::string readPackageNameFromPackageXml(const QString &package_xml_path)
{
    std::ifstream file(package_xml_path.toStdString());
    if (!file)
    {
        return "";
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    const std::string content = buffer.str();
    const std::string begin_tag = "<name>";
    const std::string end_tag = "</name>";
    const size_t begin = content.find(begin_tag);
    if (begin == std::string::npos)
    {
        return "";
    }
    const size_t name_begin = begin + begin_tag.size();
    const size_t end = content.find(end_tag, name_begin);
    if (end == std::string::npos || end <= name_begin)
    {
        return "";
    }
    return trimCopy(content.substr(name_begin, end - name_begin));
}

} // namespace

class SunrayLauncherPanel : public QWidget
{
public:
    SunrayLauncherPanel(ros::NodeHandle private_nh, QWidget *parent = nullptr)
        : QWidget(parent), private_nh_(private_nh)
    {
        private_nh_.param("launch_groups_config", launch_groups_config_path_, std::string(""));
        private_nh_.param("quick_launch_groups_config", quick_launch_groups_config_path_, std::string(""));

        loadExternalWorkspacesFromRosParam();
        loadLaunchItemsFromRosParam();
        loadQuickLaunchGroupsFromRosParam();
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
    void loadExternalWorkspacesFromRosParam()
    {
        XmlRpc::XmlRpcValue workspaces;
        if (!private_nh_.getParam("external_workspaces", workspaces))
        {
            return;
        }

        const std::vector<std::string> workspace_paths = xmlRpcToStringList(workspaces);
        for (const std::string &configured_path : workspace_paths)
        {
            ExternalWorkspace workspace;
            workspace.configured_path = QString::fromStdString(configured_path);
            workspace.root_path = QDir(expandUserPath(configured_path)).absolutePath();
            if (!QDir(workspace.root_path).exists())
            {
                ROS_WARN("sunray_launcher_panel: ignore missing external workspace: %s",
                         configured_path.c_str());
                continue;
            }

            const QString devel_path = workspace.root_path + "/devel";
            if (QDir(devel_path).exists())
            {
                workspace.devel_path = QDir(devel_path).absolutePath();
            }

            workspace.source_roots = findExternalWorkspaceSourceRoots(workspace);
            scanExternalWorkspacePackages(workspace);
            external_workspaces_.push_back(workspace);

            ROS_INFO("sunray_launcher_panel: external workspace %s loaded, source_roots=%d",
                     workspace.configured_path.toStdString().c_str(),
                     workspace.source_roots.size());
        }
    }

    QStringList findExternalWorkspaceSourceRoots(const ExternalWorkspace &workspace) const
    {
        QStringList source_roots;
        const QString catkin_file = workspace.root_path + "/devel/.catkin";
        std::ifstream file(catkin_file.toStdString());
        if (file)
        {
            std::string root;
            while (std::getline(file, root, ';'))
            {
                const std::string clean_root = trimCopy(root);
                if (!clean_root.empty())
                {
                    source_roots << QString::fromStdString(clean_root);
                }
            }
        }

        const QString standard_src = workspace.root_path + "/src";
        if (QDir(standard_src).exists())
        {
            source_roots << standard_src;
        }
        return uniquePathList(source_roots);
    }

    void scanExternalWorkspacePackages(const ExternalWorkspace &workspace)
    {
        for (const QString &source_root : workspace.source_roots)
        {
            QDirIterator it(source_root,
                            QStringList() << "package.xml",
                            QDir::Files | QDir::NoSymLinks,
                            QDirIterator::Subdirectories);
            while (it.hasNext())
            {
                const QString package_xml_path = it.next();
                const std::string package_name = readPackageNameFromPackageXml(package_xml_path);
                if (package_name.empty())
                {
                    continue;
                }

                const QString package_path = QFileInfo(package_xml_path).absoluteDir().absolutePath();
                if (external_package_paths_.find(package_name) == external_package_paths_.end())
                {
                    external_package_paths_[package_name] = package_path.toStdString();
                }
            }
        }
    }

    void loadLaunchItemsFromRosParam()
    {
        XmlRpc::XmlRpcValue groups;
        if (!private_nh_.getParam("launch_groups", groups) ||
            groups.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_WARN("sunray_launcher_panel: no valid ~launch_groups, launcher panel will start empty.");
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

    void loadQuickLaunchGroupsFromRosParam()
    {
        XmlRpc::XmlRpcValue groups;
        if (!private_nh_.getParam("quick_launch_groups", groups) ||
            groups.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            return;
        }

        for (int i = 0; i < groups.size(); ++i)
        {
            XmlRpc::XmlRpcValue category_value = groups[i];
            if (category_value.getType() != XmlRpc::XmlRpcValue::TypeStruct)
            {
                continue;
            }

            if (category_value.hasMember("category") && category_value.hasMember("scripts"))
            {
                std::string category = trimCopy(xmlRpcToString(category_value["category"], "sunray_sim"));
                if (category.empty())
                {
                    category = "sunray_sim";
                }

                XmlRpc::XmlRpcValue scripts = category_value["scripts"];
                if (scripts.getType() != XmlRpc::XmlRpcValue::TypeArray)
                {
                    continue;
                }

                for (int j = 0; j < scripts.size(); ++j)
                {
                    QuickLaunchGroup group;
                    if (parseQuickLaunchGroup(scripts[j], category, group))
                    {
                        addQuickLaunchGroup(group);
                    }
                }
                continue;
            }

            // Backward compatibility for the old flat list format.
            QuickLaunchGroup group;
            if (parseQuickLaunchGroup(category_value, "sunray_sim", group))
            {
                addQuickLaunchGroup(group);
            }
        }
    }

    bool parseQuickLaunchGroup(XmlRpc::XmlRpcValue group_value,
                               const std::string &category_fallback,
                               QuickLaunchGroup &group)
    {
        if (group_value.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
            !group_value.hasMember("title") ||
            !group_value.hasMember("items"))
        {
            return false;
        }

        group = QuickLaunchGroup{};
        group.category = category_fallback.empty() ? "sunray_sim" : category_fallback;
        group.title = xmlRpcToString(group_value["title"]);
        if (group_value.hasMember("category"))
        {
            group.category = trimCopy(xmlRpcToString(group_value["category"], group.category));
        }
        if (group.category.empty())
        {
            group.category = "sunray_sim";
        }
        if (group_value.hasMember("description"))
        {
            group.description = xmlRpcToString(group_value["description"]);
        }

        XmlRpc::XmlRpcValue items = group_value["items"];
        if (items.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            return false;
        }

        for (int j = 0; j < items.size(); ++j)
        {
            XmlRpc::XmlRpcValue item_value = items[j];
            if (item_value.getType() != XmlRpc::XmlRpcValue::TypeStruct)
            {
                continue;
            }

            QuickLaunchStep step;
            if (item_value.hasMember("ref"))
            {
                step.ref = xmlRpcToString(item_value["ref"]);
            }
            if (item_value.hasMember("title"))
            {
                step.title = xmlRpcToString(item_value["title"]);
            }
            if (item_value.hasMember("package"))
            {
                step.package = xmlRpcToString(item_value["package"]);
            }
            if (item_value.hasMember("launch"))
            {
                step.launch = xmlRpcToString(item_value["launch"]);
            }
            if (item_value.hasMember("command"))
            {
                step.command = xmlRpcToString(item_value["command"]);
            }
            if (item_value.hasMember("args"))
            {
                step.args = xmlRpcToStringList(item_value["args"]);
            }
            if (item_value.hasMember("delay_sec") &&
                (item_value["delay_sec"].getType() == XmlRpc::XmlRpcValue::TypeDouble ||
                 item_value["delay_sec"].getType() == XmlRpc::XmlRpcValue::TypeInt))
            {
                step.delay_sec = static_cast<double>(item_value["delay_sec"]);
            }

            resolveQuickLaunchStep(step);
            if (!step.command.empty() || (!step.package.empty() && !step.launch.empty()))
            {
                group.steps.push_back(step);
            }
        }

        return !group.title.empty() && !group.steps.empty();
    }

    void addQuickLaunchGroup(const QuickLaunchGroup &group)
    {
        if (std::find(quick_launch_categories_.begin(),
                      quick_launch_categories_.end(),
                      group.category) == quick_launch_categories_.end())
        {
            quick_launch_categories_.push_back(group.category);
        }
        quick_launch_groups_.push_back(group);
    }

    void resolveQuickLaunchStep(QuickLaunchStep &step) const
    {
        if (step.ref.empty())
        {
            return;
        }

        const int idx = findLaunchItemByRef(step.ref);
        if (idx < 0)
        {
            ROS_WARN("sunray_launcher_panel: quick launch ref not found: %s", step.ref.c_str());
            return;
        }

        const LaunchItem &item = launch_items_[static_cast<size_t>(idx)];
        step.linked_launch_index = idx;
        if (step.title.empty())
        {
            step.title = item.title;
        }
        if (step.package.empty())
        {
            step.package = item.package;
        }
        if (step.launch.empty())
        {
            step.launch = item.launch;
        }
        if (step.args.empty())
        {
            step.args = item.args;
        }
    }

    int findLaunchItemByRef(const std::string &ref) const
    {
        const size_t slash_pos = ref.find('/');
        if (slash_pos == std::string::npos)
        {
            return -1;
        }

        const std::string group_name = ref.substr(0, slash_pos);
        const std::string title = ref.substr(slash_pos + 1);
        for (int i = 0; i < static_cast<int>(launch_items_.size()); ++i)
        {
            const LaunchItem &item = launch_items_[static_cast<size_t>(i)];
            if (item.group == group_name && item.title == title)
            {
                return i;
            }
        }
        return -1;
    }

    void buildUi()
    {
        setWindowTitle("Sunray Launcher");
        const QString package_dir = QString::fromStdString(ros::package::getPath("sunray_launcher_panel"));
        const QString logo_path = package_dir + "/logo/yundrone_logo.png";
        const QString icon_path = package_dir + "/logo/yundrone_small_logo.jpeg";
        if (QFileInfo::exists(icon_path))
        {
            setWindowIcon(QIcon(icon_path));
        }
        setMinimumSize(1360, 800);
        resize(1600, 920);

        auto *root = new QVBoxLayout(this);
        root->setContentsMargins(16, 16, 16, 16);
        root->setSpacing(12);

        auto *header = new QWidget();
        auto *header_layout = new QHBoxLayout(header);
        header_layout->setContentsMargins(0, 0, 0, 0);
        header_layout->setSpacing(16);

        auto *title = new QLabel("Sunray启动器");
        title->setObjectName("titleLabel");

        auto *logo = new QLabel();
        logo->setObjectName("logoLabel");
        if (QFileInfo::exists(logo_path))
        {
            logo->setPixmap(QPixmap(logo_path).scaledToHeight(56, Qt::SmoothTransformation));
        }
        logo->setMinimumWidth(230);
        logo->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

        header_layout->addWidget(title);
        header_layout->addStretch();
        header_layout->addWidget(logo);
        root->addWidget(header);

        auto *main_splitter = new QSplitter(Qt::Horizontal);
        auto *left_panel = new QWidget();
        auto *left_layout = new QVBoxLayout(left_panel);
        left_layout->setContentsMargins(0, 0, 0, 0);
        left_layout->setSpacing(12);

        auto *splitter = new QSplitter(Qt::Vertical);
        splitter->addWidget(buildLaunchArea());
        splitter->addWidget(buildInfoPanel());
        splitter->setCollapsible(0, false);
        splitter->setCollapsible(1, false);
        splitter->setStretchFactor(0, 7);
        splitter->setStretchFactor(1, 2);
        splitter->setSizes({700, 280});
        left_layout->addWidget(splitter, 1);

        QWidget *monitor_panel = buildSystemMonitorPanel();
        main_splitter->addWidget(left_panel);
        main_splitter->addWidget(monitor_panel);
        main_splitter->setCollapsible(0, false);
        main_splitter->setCollapsible(1, false);
        main_splitter->setStretchFactor(0, 8);
        main_splitter->setStretchFactor(1, 3);
        main_splitter->setSizes({1260, 380});
        root->addWidget(main_splitter, 1);

        setStyleSheet(R"(
            QWidget { background: #eef2ed; color: #17211c; font-family: "Noto Sans CJK SC", "Microsoft YaHei", sans-serif; font-size: 13px; }
            #titleLabel { font-size: 24px; font-weight: 800; color: #16352d; padding: 4px 0px 8px 2px; }
            QGroupBox { border: 1px solid #bac7bf; border-radius: 10px; margin-top: 12px; background: #f8faf7; font-weight: 700; }
            QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0px 6px; color: #31584d; }
            QTabWidget::pane { border: 1px solid #b9c7be; border-radius: 10px; background: #fbfcfa; top: -1px; }
            QTreeWidget { background: #fbfcfa; border: 0px; border-radius: 10px; outline: 0; }
            QTreeWidget::item { padding: 7px; }
            QTreeWidget::item:selected { background: #dfe4e1; color: #111815; }
            QTextEdit { background: #111815; color: #d8f3df; border-radius: 10px; border: 1px solid #23342d; font-family: "JetBrains Mono", "DejaVu Sans Mono", monospace; }
            QTextEdit#commandEdit { background: #111815; color: #f3fff6; border: 1px solid #23342d; border-radius: 8px; padding: 4px 8px; }
            QPushButton { background: #1f7a5b; color: white; border: 0px; border-radius: 8px; padding: 8px 14px; font-weight: 700; }
            QPushButton:hover { background: #28916c; }
            QPushButton:disabled { background: #9caaa3; color: #edf1ee; }
            QPushButton#stopButton { background: #b74e35; }
            QPushButton#stopButton:hover { background: #ce5b3e; }
            QPushButton#secondaryButton { background: #4d675d; }
            QPushButton#secondaryButton:hover { background: #5c7a6f; }
            QLabel#stateLabel { color: #16352d; font-weight: 700; }
            QLabel#mutedLabel { color: #63736b; }
            QLabel#commandPrompt { color: #1f7a5b; font-size: 20px; font-weight: 900; font-family: "JetBrains Mono", "DejaVu Sans Mono", monospace; padding: 0px 2px 2px 2px; }
        )");

        clearInitialSelection();
    }

    QWidget *buildLaunchArea()
    {
        auto *container = new QWidget();
        auto *layout = new QVBoxLayout(container);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->setSpacing(8);
        layout->addWidget(buildQuickLaunchPanel(), 1);
        layout->addWidget(buildLaunchTree(), 1);
        return container;
    }

    QWidget *buildQuickLaunchPanel()
    {
        auto *box = new QGroupBox("一键启动脚本");
        auto *layout = new QVBoxLayout(box);

        quick_launch_tabs_ = new LaunchTabWidget();
        quick_launch_tab_bar_ = new LaunchTabBar(quick_launch_tabs_);
        quick_launch_tabs_->useLaunchTabBar(quick_launch_tab_bar_);

        for (const std::string &category : quick_launch_categories_)
        {
            auto *tree = new ClearableTreeWidget();
            tree->setColumnCount(3);
            tree->setHeaderLabels(QStringList() << "标题" << "状态" << "功能描述");
            tree->setRootIsDecorated(false);
            tree->setIndentation(0);
            tree->setItemsExpandable(false);
            tree->setExpandsOnDoubleClick(false);
            tree->setAllColumnsShowFocus(true);
            tree->setItemDelegateForColumn(1, new StatusBadgeDelegate(tree));
            tree->setMinimumHeight(118);
            tree->header()->setMinimumSectionSize(88);
            tree->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
            tree->header()->setSectionResizeMode(1, QHeaderView::Fixed);
            tree->setColumnWidth(1, 96);
            tree->header()->setSectionResizeMode(2, QHeaderView::Stretch);

            for (int i = 0; i < static_cast<int>(quick_launch_groups_.size()); ++i)
            {
                const QuickLaunchGroup &group = quick_launch_groups_[static_cast<size_t>(i)];
                if (group.category != category)
                {
                    continue;
                }
                auto *item = new QTreeWidgetItem(tree);
                item->setText(0, QString::fromStdString(group.title));
                item->setText(1, "未运行");
                item->setText(2, QString::fromStdString(group.description));
                item->setData(0, kQuickLaunchIndexRole, i);
                item->setData(0, kOriginalTitleRole, QString::fromStdString(group.title));
                quick_item_by_index_[i] = item;
            }

            connect(tree, &QTreeWidget::currentItemChanged, this,
                    [this](QTreeWidgetItem *, QTreeWidgetItem *) {
                        updateLaunchPreviewHighlight();
                        updateGroupTabStatus();
                        updateQuickLaunchTabStatus();
                        updateButtons();
                    });

            quick_launch_trees_.push_back(tree);
            quick_launch_tabs_->addTab(tree, QString::fromStdString(category));
        }

        connect(quick_launch_tabs_, &QTabWidget::currentChanged, this,
                [this](int) {
                    updateLaunchPreviewHighlight();
                    updateGroupTabStatus();
                    updateQuickLaunchTabStatus();
                    updateButtons();
                });

        auto *button_row = new QHBoxLayout();
        quick_start_btn_ = new QPushButton("一键启动");
        quick_stop_btn_ = new QPushButton("一键停止");
        quick_stop_btn_->setObjectName("stopButton");
        auto *hint = new QLabel("使用 Ubuntu Terminal 多标签启动：每个 launch 一个 tab，按配置顺序延时启动");
        hint->setObjectName("mutedLabel");
        button_row->addWidget(hint, 1);
        button_row->addWidget(quick_start_btn_);
        button_row->addWidget(quick_stop_btn_);

        connect(quick_start_btn_, &QPushButton::clicked, this, [this]() { startSelectedQuickLaunch(); });
        connect(quick_stop_btn_, &QPushButton::clicked, this, [this]() { stopSelectedQuickLaunch(); });

        layout->addWidget(quick_launch_tabs_);
        layout->addLayout(button_row);
        return box;
    }

    QWidget *buildLaunchTree()
    {
        auto *box = new QGroupBox("启动文件列表");
        auto *layout = new QVBoxLayout(box);

        launch_tabs_ = new LaunchTabWidget();
        launch_tab_bar_ = new LaunchTabBar(launch_tabs_);
        launch_tabs_->useLaunchTabBar(launch_tab_bar_);
        for (const LaunchGroup &group : launch_groups_)
        {
            auto *tree = new ClearableTreeWidget();
            tree->setColumnCount(5);
            tree->setHeaderLabels(QStringList() << "标题" << "状态" << "ROS Package" << "launch文件名" << "功能描述");
            tree->setRootIsDecorated(false);
            tree->setIndentation(0);
            tree->setItemsExpandable(false);
            tree->setExpandsOnDoubleClick(false);
            tree->setAllColumnsShowFocus(true);
            tree->setItemDelegateForColumn(1, new StatusBadgeDelegate(tree));
            tree->header()->setMinimumSectionSize(88);
            tree->header()->setSectionResizeMode(0, QHeaderView::Interactive);
            tree->setColumnWidth(0, 180);
            tree->header()->setSectionResizeMode(1, QHeaderView::Fixed);
            tree->setColumnWidth(1, 96);
            tree->header()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
            tree->header()->setSectionResizeMode(3, QHeaderView::ResizeToContents);
            tree->header()->setSectionResizeMode(4, QHeaderView::Stretch);

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
                tree_item->setText(4, QString::fromStdString(item.description));
                tree_item->setData(0, kLaunchIndexRole, idx);
                tree_item->setData(0, kOriginalTitleRole, QString::fromStdString(item.title));
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
            onSelectionChanged(tree ? tree->currentItem() : nullptr);
        });

        selected_command_ = new QTextEdit();
        selected_command_->setPlaceholderText("roslaunch sunray_sim sunray_sim.launch");
        selected_command_->setMinimumHeight(34);
        selected_command_->setMaximumHeight(40);
        selected_command_->setObjectName("commandEdit");
        selected_command_->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

        auto *control_row = new QHBoxLayout();
        control_row->setSpacing(8);
        auto *command_prompt = new QLabel(">");
        command_prompt->setObjectName("commandPrompt");
        start_btn_ = new QPushButton("启动");
        stop_btn_ = new QPushButton("停止");
        stop_btn_->setObjectName("stopButton");
        control_row->addWidget(command_prompt);
        control_row->addWidget(selected_command_, 1);
        control_row->addWidget(start_btn_);
        control_row->addWidget(stop_btn_);

        connect(start_btn_, &QPushButton::clicked, this, [this]() { startSelectedLaunch(); });
        connect(stop_btn_, &QPushButton::clicked, this, [this]() { stopSelectedLaunch(); });

        layout->addWidget(launch_tabs_, 1);
        layout->addLayout(control_row);
        updateButtons();
        return box;
    }

    QWidget *buildInfoPanel()
    {
        auto *box = new QGroupBox("操作日志");
        auto *layout = new QVBoxLayout(box);

        auto *caption = new QLabel("启动 / 停止 / 预览 / 退出码");
        caption->setObjectName("mutedLabel");
        layout->addWidget(caption);

        log_view_ = new QTextEdit();
        log_view_->setReadOnly(true);
        log_view_->setFont(QFont("JetBrains Mono", 10));
        layout->addWidget(log_view_);
        return box;
    }

    QWidget *buildSystemMonitorPanel()
    {
        auto *box = new QGroupBox("系统状态监控");
        auto *layout = new QVBoxLayout(box);
        box->setMinimumWidth(320);
        box->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
        layout->setContentsMargins(12, 18, 12, 12);
        layout->setSpacing(10);

        auto *resource_box = new QGroupBox("资源监控");
        auto *resource_layout = new QGridLayout(resource_box);
        resource_layout->setContentsMargins(10, 14, 10, 10);
        resource_layout->setVerticalSpacing(8);

        cpu_label_ = new QLabel("CPU: --");
        memory_label_ = new QLabel("内存: --");

        cpu_bar_ = new QProgressBar();
        cpu_bar_->setRange(0, 100);
        cpu_bar_->setTextVisible(false);
        memory_bar_ = new QProgressBar();
        memory_bar_->setRange(0, 100);
        memory_bar_->setTextVisible(false);

        resource_layout->addWidget(cpu_label_, 0, 0);
        resource_layout->addWidget(cpu_bar_, 0, 1);
        resource_layout->addWidget(memory_label_, 1, 0);
        resource_layout->addWidget(memory_bar_, 1, 1);
        resource_layout->setColumnStretch(1, 1);

        auto *ros_box = new QGroupBox("ROS 节点");
        auto *ros_layout = new QVBoxLayout(ros_box);
        ros_layout->setContentsMargins(10, 14, 10, 10);
        ros_layout->setSpacing(8);

        auto *ros_state_row = new QHBoxLayout();
        ros_state_marker_ = new QFrame();
        ros_state_marker_->setFixedSize(12, 12);
        ros_state_marker_->setFrameShape(QFrame::StyledPanel);
        ros_state_marker_->setFrameShadow(QFrame::Plain);
        ros_state_marker_->setStyleSheet("background: #8f9d95; border-radius: 6px;");
        ros_node_count_label_ = new QLabel("ROS nodes: --");
        ros_state_text_ = new QLabel("状态: 未知");
        ros_state_text_->setObjectName("mutedLabel");
        ros_state_row->addWidget(ros_state_marker_, 0, Qt::AlignVCenter);
        ros_state_row->addWidget(ros_node_count_label_, 0);
        ros_state_row->addStretch(1);
        ros_state_row->addWidget(ros_state_text_, 0);

        ros_nodes_view_ = new QTextEdit();
        ros_nodes_view_->setReadOnly(true);
        ros_nodes_view_->setMinimumWidth(290);
        ros_nodes_view_->setMinimumHeight(260);
        ros_nodes_view_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        ros_nodes_view_->setLineWrapMode(QTextEdit::WidgetWidth);
        ros_nodes_view_->setWordWrapMode(QTextOption::WrapAtWordBoundaryOrAnywhere);
        ros_nodes_view_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        ros_nodes_view_->setFont(QFont("JetBrains Mono", 10));
        ros_layout->addLayout(ros_state_row);
        ros_layout->addWidget(ros_nodes_view_, 1);

        auto *button_row = new QHBoxLayout();
        auto *stop_all_btn = new QPushButton("停止全部 Launch");
        stop_all_btn->setObjectName("stopButton");
        button_row->addWidget(stop_all_btn);
        button_row->addStretch(1);

        connect(stop_all_btn, &QPushButton::clicked, this, [this]() { stopAllLaunches(); });

        layout->addWidget(resource_box);
        layout->addWidget(ros_box, 1);
        layout->addLayout(button_row);
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

    void clearInitialSelection()
    {
        for (QTreeWidget *tree : quick_launch_trees_)
        {
            if (tree)
            {
                tree->clearSelection();
                tree->setCurrentItem(nullptr);
            }
        }

        if (quick_launch_tabs_)
        {
            quick_launch_tabs_->setCurrentIndex(0);
        }

        for (QTreeWidget *tree : launch_trees_)
        {
            if (tree)
            {
                tree->clearSelection();
                tree->setCurrentItem(nullptr);
            }
        }

        if (launch_tabs_)
        {
            launch_tabs_->setCurrentIndex(0);
        }
        onSelectionChanged(nullptr);
        updateLaunchPreviewHighlight();
        updateGroupTabStatus();
        updateQuickLaunchTabStatus();
        updateButtons();
    }

    void startSelectedLaunch()
    {
        cacheSelectedLaunchCommand();
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
            const int quick_idx = findRunningQuickLaunchForLinkedLaunch(idx);
            if (quick_idx >= 0 && launch_runtimes_.find(idx) == launch_runtimes_.end())
            {
                stopQuickLaunch(quick_idx);
                return;
            }
            stopLaunch(idx);
        }
    }

    int selectedQuickLaunchIndex() const
    {
        if (!quick_launch_tabs_ || quick_launch_tabs_->currentIndex() < 0 ||
            quick_launch_tabs_->currentIndex() >= static_cast<int>(quick_launch_trees_.size()))
        {
            return -1;
        }

        QTreeWidget *tree = quick_launch_trees_[static_cast<size_t>(quick_launch_tabs_->currentIndex())];
        if (!tree || !tree->currentItem())
        {
            return -1;
        }
        const QVariant value = tree->currentItem()->data(0, kQuickLaunchIndexRole);
        return value.isValid() ? value.toInt() : -1;
    }

    void startSelectedQuickLaunch()
    {
        cacheSelectedLaunchCommand();
        const int idx = selectedQuickLaunchIndex();
        if (idx >= 0)
        {
            startQuickLaunch(idx);
        }
    }

    void stopSelectedQuickLaunch()
    {
        const int idx = selectedQuickLaunchIndex();
        if (idx >= 0)
        {
            stopQuickLaunch(idx);
        }
    }

    void startQuickLaunch(const int idx)
    {
        if (idx < 0 || idx >= static_cast<int>(quick_launch_groups_.size()))
        {
            return;
        }
        if (isQuickLaunchRunning(idx))
        {
            appendLog("[Sunray] 一键启动脚本已在运行: " + quick_launch_groups_[static_cast<size_t>(idx)].title);
            return;
        }
        const QuickLaunchGroup &group = quick_launch_groups_[static_cast<size_t>(idx)];
        QuickLaunchRuntime runtime;
        runtime.terminal.start_time = QDateTime::currentDateTime();
        runtime.linked_launch_indices = collectLinkedLaunchIndices(group);

        if (!prepareQuickLaunchRuntime(idx, group, runtime))
        {
            appendLog("[Sunray] 一键启动失败: 无法创建 terminal wrapper。");
            updateQuickLaunchStatus(idx, "未运行");
            updateButtons();
            return;
        }

        auto *process = new QProcess(this);
        const QStringList terminal_command = buildQuickTerminalCommand(runtime, group);
        if (terminal_command.isEmpty())
        {
            appendLog("[Sunray] 一键启动失败: 没有找到 gnome-terminal，无法使用 Ubuntu Terminal 多标签。");
            cleanupRuntimeFiles(runtime.terminal);
            updateQuickLaunchStatus(idx, "未运行");
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
                        appendLog("[Sunray] 一键启动 terminal 异常退出: " +
                                  quick_launch_groups_[static_cast<size_t>(idx)].title +
                                  " exit_code=" + std::to_string(exit_code) +
                                  " status=" + (exit_status == QProcess::NormalExit ? "normal" : "crashed"));
                    }
                    quick_terminal_processes_.erase(idx);
                    process->deleteLater();
                    updateButtons();
                });

        quick_launch_runtimes_[idx] = runtime;
        quick_terminal_processes_[idx] = process;
        process->start();
        if (!process->waitForStarted(1500))
        {
            appendLog("[Sunray] 一键启动失败: " + group.title + "  error=" + process->errorString().toStdString());
            quick_terminal_processes_.erase(idx);
            process->deleteLater();
            cleanupRuntimeFiles(runtime.terminal);
            updateQuickLaunchStatus(idx, "未运行");
            updateButtons();
            return;
        }

        quick_launch_runtimes_[idx].terminal.terminal_pid = process->processId();
        quick_launch_runtimes_[idx].terminal.running = true;
        quick_launch_runtimes_[idx].running = true;
        updateQuickLaunchStatus(idx, "运行 0s");
        markLinkedLaunchesRunning(quick_launch_runtimes_[idx], "运行(快速)");
        appendQuickLaunchStartLog(group, quick_launch_runtimes_[idx]);
        updateButtons();
    }

    void cacheSelectedLaunchCommand()
    {
        cacheLaunchCommand(current_selected_launch_index_);
    }

    void cacheLaunchCommand(const int idx)
    {
        if (!selected_command_ ||
            idx < 0 ||
            idx >= static_cast<int>(launch_items_.size()))
        {
            return;
        }

        const QString command = selected_command_->toPlainText().simplified();
        if (!command.isEmpty() && command != "-")
        {
            edited_launch_commands_[idx] = command;
        }
    }

    std::vector<int> collectLinkedLaunchIndices(const QuickLaunchGroup &group) const
    {
        std::vector<int> indices;
        for (const QuickLaunchStep &step : group.steps)
        {
            if (step.linked_launch_index >= 0 &&
                std::find(indices.begin(), indices.end(), step.linked_launch_index) == indices.end())
            {
                indices.push_back(step.linked_launch_index);
            }
        }
        return indices;
    }

    void markLinkedLaunchesRunning(const QuickLaunchRuntime &runtime, const QString &status)
    {
        for (const int linked_idx : runtime.linked_launch_indices)
        {
            updateLaunchStatus(linked_idx, status);
        }
        updateGroupTabStatus();
    }

    bool isLaunchActive(const int idx) const
    {
        if (isRunning(idx))
        {
            return true;
        }

        return findRunningQuickLaunchForLinkedLaunch(idx) >= 0;
    }

    int findRunningQuickLaunchForLinkedLaunch(const int linked_idx) const
    {
        for (const auto &entry : quick_launch_runtimes_)
        {
            if (!isQuickLaunchRunning(entry.first))
            {
                continue;
            }
            const std::vector<int> &indices = entry.second.linked_launch_indices;
            if (std::find(indices.begin(), indices.end(), linked_idx) != indices.end())
            {
                return entry.first;
            }
        }
        return -1;
    }

    void startLaunch(const int idx)
    {
        if (idx < 0 || idx >= static_cast<int>(launch_items_.size()))
        {
            return;
        }
        if (isRunning(idx))
        {
            appendLog("[Sunray] Launch 已在运行: " + launch_items_[static_cast<size_t>(idx)].title);
            return;
        }

        const LaunchItem &item = launch_items_[static_cast<size_t>(idx)];
        LaunchCommand command;
        if (!parseEditedCommand(command))
        {
            appendLog("[Sunray] 启动失败: 启动命令格式错误，应为 roslaunch <package> <launch> [args...]");
            updateLaunchStatus(idx, "未运行");
            updateGroupTabStatus();
            updateButtons();
            return;
        }

        const std::string launch_file = resolveLaunchFilePath(command.package_name, command.launch_file);
        if (!QFileInfo::exists(QString::fromStdString(launch_file)))
        {
            appendLog("[Sunray] 启动失败: 找不到 launch 文件 " + launch_file);
            updateLaunchStatus(idx, "未运行");
            updateGroupTabStatus();
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
            appendLog("[Sunray] 启动失败: 无法创建 terminal wrapper。");
            updateLaunchStatus(idx, "未运行");
            updateGroupTabStatus();
            updateButtons();
            return;
        }

        auto *process = new QProcess(this);
        const QStringList terminal_command = buildTerminalCommand(runtime);
        if (terminal_command.isEmpty())
        {
            appendLog("[Sunray] 启动失败: 没有找到可用终端，请安装 gnome-terminal、konsole 或 xterm。");
            cleanupRuntimeFiles(runtime);
            updateLaunchStatus(idx, "未运行");
            updateGroupTabStatus();
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
                        appendLog("[Sunray] 终端启动器异常退出: " + launch_items_[static_cast<size_t>(idx)].title +
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
            appendLog("[Sunray] 启动失败: " + item.title + "  error=" + process->errorString().toStdString());
            terminal_processes_.erase(idx);
            process->deleteLater();
            cleanupRuntimeFiles(runtime);
            updateLaunchStatus(idx, "未运行");
            updateGroupTabStatus();
            updateButtons();
            return;
        }

        launch_runtimes_[idx].terminal_pid = process->processId();
        launch_runtimes_[idx].running = true;
        updateLaunchStatus(idx, "运行 0s");
        updateGroupTabStatus();
        appendLog("[Sunray] Launch: " + item.title);
        appendLog("[Sunray] Command: " + display_command.toStdString());
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
        writeExternalWorkspaceEnvironment(script);
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

    bool prepareQuickLaunchRuntime(const int idx,
                                   const QuickLaunchGroup &group,
                                   QuickLaunchRuntime &runtime) const
    {
        QDir runtime_dir(QDir::tempPath() + "/sunray_launcher_panel");
        if (!runtime_dir.exists() && !runtime_dir.mkpath("."))
        {
            return false;
        }

        const QString base_name = QString("quick_%1_%2_%3")
                                      .arg(idx)
                                      .arg(QDateTime::currentMSecsSinceEpoch())
                                      .arg(sanitizeFileToken(QString::fromStdString(group.title)));
        runtime.terminal.script_path = runtime_dir.filePath(base_name);
        runtime.terminal.pid_file = runtime_dir.filePath(base_name + ".pid");
        runtime.terminal.done_file = runtime_dir.filePath(base_name + ".done");
        runtime.terminal.command = QString("quick_launch:%1").arg(QString::fromStdString(group.title));

        bool has_valid_step = false;
        int valid_step_index = 0;
        double start_delay_sec = 0.0;
        for (size_t i = 0; i < group.steps.size(); ++i)
        {
            const QuickLaunchStep &step = group.steps[i];
            const QString command = buildQuickStepRoslaunchCommand(step);
            const QString display_command = buildQuickStepDisplayCommand(step);
            if (command.isEmpty())
            {
                continue;
            }

            const QString step_script_path = quickStepScriptPath(runtime, valid_step_index);
            if (!writeQuickStepScript(step_script_path,
                                      group,
                                      step,
                                      command,
                                      display_command.isEmpty() ? command : display_command,
                                      valid_step_index,
                                      group.steps.size(),
                                      start_delay_sec))
            {
                return false;
            }
            runtime.step_script_paths << step_script_path;
            runtime.step_titles << QString::fromStdString(step.title.empty() ? step.package : step.title);
            runtime.step_commands << command;
            runtime.step_display_commands << (display_command.isEmpty() ? command : display_command);
            has_valid_step = true;
            ++valid_step_index;
            start_delay_sec += std::max(0.0, step.delay_sec);
        }

        if (!has_valid_step)
        {
            return false;
        }
        return true;
    }

    void appendQuickLaunchStartLog(const QuickLaunchGroup &group,
                                   const QuickLaunchRuntime &runtime)
    {
        appendLog("[Sunray] 一键启动脚本: " + group.title);
        for (int i = 0; i < runtime.step_titles.size(); ++i)
        {
            appendLog(QString("[Sunray] 当前 Terminal Tab %1/%2: %3")
                          .arg(i + 1)
                          .arg(runtime.step_titles.size())
                          .arg(runtime.step_titles[i])
                          .toStdString());
            if (i < runtime.step_commands.size() && !runtime.step_commands[i].isEmpty())
            {
                const QString command = i < runtime.step_display_commands.size()
                                            ? runtime.step_display_commands[i]
                                            : runtime.step_commands[i];
                appendLog("[Sunray] Command: " + command.toStdString());
            }
        }
    }

    QString quickStepScriptPath(const QuickLaunchRuntime &runtime, const int step_index) const
    {
        return QString("%1_step_%2.sh").arg(runtime.terminal.script_path).arg(step_index);
    }

    bool writeQuickStepScript(const QString &path,
                              const QuickLaunchGroup &group,
                              const QuickLaunchStep &step,
                              const QString &command,
                              const QString &display_command,
                              const int step_index,
                              const size_t step_count,
                              const double start_delay_sec) const
    {
        std::ofstream script(path.toStdString());
        if (!script)
        {
            return false;
        }

        const QString step_title = QString::fromStdString(step.title.empty() ? step.package : step.title);
        script << "#!/usr/bin/env bash\n";
        script << "set +e\n";
        writeTerminalTitleKeeper(script, step_title);
        if (step_index == 0)
        {
            script << "rm -f " << shellQuote(path.section("_step_", 0, 0) + ".pid").toStdString() << " "
                   << shellQuote(path.section("_step_", 0, 0) + ".done").toStdString() << "\n";
        }
        script << "echo \"$$\" >> " << shellQuote(path.section("_step_", 0, 0) + ".pid").toStdString() << "\n";
        if (start_delay_sec > 0.0)
        {
            script << "sleep " << start_delay_sec << "\n";
        }
        for (const QString &line : buildExternalWorkspaceEnvironmentLines())
        {
            script << line.toStdString() << "\n";
        }
        script << "clear\n";
        script << "echo " << shellQuote("============================================================").toStdString() << "\n";
        script << "echo " << shellQuote(QString("[Sunray] 一键启动脚本: %1").arg(QString::fromStdString(group.title))).toStdString() << "\n";
        script << "echo " << shellQuote(QString("[Sunray] 当前 Terminal Tab %1/%2: %3")
                                            .arg(step_index + 1)
                                            .arg(step_count)
                                            .arg(step_title))
                                      .toStdString()
               << "\n";
        script << "echo " << shellQuote("[Sunray] Command: " + display_command).toStdString() << "\n";
        script << "echo " << shellQuote("[Sunray] 停止整组: 使用启动器的“一键停止”按钮").toStdString() << "\n";
        script << "echo " << shellQuote("============================================================").toStdString() << "\n";
        script << "trap 'stop_sunray_title_keeper; exit 130' INT TERM HUP\n";
        script << command.toStdString() << "\n";
        script << "exit_code=$?\n";
        script << "stop_sunray_title_keeper\n";
        script << "set_sunray_terminal_title\n";
        script << "trap - INT TERM HUP\n";
        script << "echo \"$exit_code\" >> " << shellQuote(path.section("_step_", 0, 0) + ".done").toStdString() << "\n";
        script << "echo \"[Sunray] roslaunch exited with code ${exit_code}. Press Enter to keep/close this tab.\"\n";
        script << "read\n";
        script.close();

        return ::chmod(path.toStdString().c_str(), 0755) == 0;
    }

    QString buildRoslaunchCommand(const QuickLaunchStep &step) const
    {
        const std::string launch_file = resolveLaunchFilePath(step.package, step.launch);
        if (!QFileInfo::exists(QString::fromStdString(launch_file)))
        {
            return "";
        }

        QStringList step_args;
        for (const std::string &arg : step.args)
        {
            step_args << QString::fromStdString(arg);
        }
        return shellJoin(QStringList() << "roslaunch" << buildRoslaunchArguments(launch_file, step_args));
    }

    QString buildQuickStepRoslaunchCommand(const QuickLaunchStep &step) const
    {
        if (!step.command.empty())
        {
            return QString::fromStdString(step.command);
        }

        if (step.linked_launch_index >= 0)
        {
            const auto edited_it = edited_launch_commands_.find(step.linked_launch_index);
            if (edited_it != edited_launch_commands_.end() && !edited_it->second.trimmed().isEmpty())
            {
                return buildExecutableRoslaunchCommand(edited_it->second);
            }
        }

        return buildRoslaunchCommand(step);
    }

    QString buildQuickStepDisplayCommand(const QuickLaunchStep &step) const
    {
        if (!step.command.empty())
        {
            return QString::fromStdString(step.command).simplified();
        }

        if (step.linked_launch_index >= 0)
        {
            const auto edited_it = edited_launch_commands_.find(step.linked_launch_index);
            if (edited_it != edited_launch_commands_.end() && !edited_it->second.trimmed().isEmpty())
            {
                return edited_it->second.simplified();
            }
        }

        QStringList args;
        args << "roslaunch"
             << QString::fromStdString(step.package)
             << QString::fromStdString(step.launch);
        for (const std::string &arg : step.args)
        {
            args << QString::fromStdString(arg);
        }
        return args.join(" ");
    }

    QString buildExecutableRoslaunchCommand(const QString &display_command) const
    {
        const QStringList tokens = splitCommandLine(display_command.simplified());
        if (tokens.size() < 3 || tokens[0] != "roslaunch")
        {
            return "";
        }

        const std::string package_name = tokens[1].toStdString();
        const std::string launch_file_name = tokens[2].toStdString();
        const std::string launch_file = resolveLaunchFilePath(package_name, launch_file_name);
        if (!QFileInfo::exists(QString::fromStdString(launch_file)))
        {
            return "";
        }

        return shellJoin(QStringList() << "roslaunch" << buildRoslaunchArguments(launch_file, tokens.mid(3)));
    }

    QStringList buildQuickTerminalCommand(const QuickLaunchRuntime &runtime,
                                          const QuickLaunchGroup &group) const
    {
        if (QStandardPaths::findExecutable("gnome-terminal").isEmpty())
        {
            return {};
        }

        QStringList command;
        command << "gnome-terminal";

        for (int i = 0; i < runtime.step_script_paths.size(); ++i)
        {
            const QString title = i < runtime.step_titles.size()
                                      ? runtime.step_titles[i]
                                      : QString("launch_%1").arg(i + 1);
            command << (i == 0 ? "--window" : "--tab")
                    << "--title" << title
                    << "--command" << QString("bash -lc %1").arg(shellQuote(runtime.step_script_paths[i]));
        }
        return command;
    }

    void writeExternalWorkspaceEnvironment(std::ofstream &script) const
    {
        const QStringList lines = buildExternalWorkspaceEnvironmentLines();
        for (const QString &line : lines)
        {
            script << line.toStdString() << "\n";
        }
    }

    QStringList buildExternalWorkspaceEnvironmentLines() const
    {
        QStringList lines;
        QStringList source_roots;
        QStringList devel_paths;
        for (const ExternalWorkspace &workspace : external_workspaces_)
        {
            source_roots << workspace.source_roots;
            if (!workspace.devel_path.isEmpty())
            {
                devel_paths << workspace.devel_path;
            }
        }

        source_roots = uniquePathList(source_roots);
        devel_paths = uniquePathList(devel_paths);
        if (source_roots.isEmpty() && devel_paths.isEmpty())
        {
            return lines;
        }

        const QString ros_package_path = source_roots.join(":");
        if (!ros_package_path.isEmpty())
        {
            lines << "export ROS_PACKAGE_PATH=" + shellEnvPrepend(source_roots, "ROS_PACKAGE_PATH");
        }

        const QString cmake_prefix_path = devel_paths.join(":");
        if (!cmake_prefix_path.isEmpty())
        {
            lines << "export CMAKE_PREFIX_PATH=" + shellEnvPrepend(devel_paths, "CMAKE_PREFIX_PATH");

            QStringList bin_paths;
            QStringList lib_paths;
            for (const QString &devel_path : devel_paths)
            {
                if (QDir(devel_path + "/bin").exists())
                {
                    bin_paths << devel_path + "/bin";
                }
                if (QDir(devel_path + "/lib").exists())
                {
                    lib_paths << devel_path + "/lib";
                }
            }
            if (!bin_paths.isEmpty())
            {
                lines << "export PATH=" + shellEnvPrepend(bin_paths, "PATH");
            }
            if (!lib_paths.isEmpty())
            {
                lines << "export LD_LIBRARY_PATH=" + shellEnvPrepend(lib_paths, "LD_LIBRARY_PATH");
            }

            QStringList python_paths;
            for (const QString &devel_path : devel_paths)
            {
                const QString python3_path = devel_path + "/lib/python3/dist-packages";
                const QString python2_path = devel_path + "/lib/python2.7/dist-packages";
                if (QDir(python3_path).exists())
                {
                    python_paths << python3_path;
                }
                if (QDir(python2_path).exists())
                {
                    python_paths << python2_path;
                }
            }
            if (!python_paths.isEmpty())
            {
                lines << "export PYTHONPATH=" + shellEnvPrepend(python_paths, "PYTHONPATH");
            }
        }

        lines << "rospack profile >/dev/null 2>&1 || true";
        return lines;
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

        const std::string package_path = resolvePackagePath(package_name);
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

    std::string resolvePackagePath(const std::string &package_name) const
    {
        const auto external_it = external_package_paths_.find(package_name);
        if (external_it != external_package_paths_.end())
        {
            return external_it->second;
        }

        const std::string package_path = ros::package::getPath(package_name);
        if (!package_path.empty())
        {
            return package_path;
        }
        return "";
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

    bool isQuickLaunchRunning(const int idx) const
    {
        const auto it = quick_launch_runtimes_.find(idx);
        if (it == quick_launch_runtimes_.end() || !it->second.running)
        {
            return false;
        }

        const std::vector<pid_t> pids = readPidListFile(it->second.terminal.pid_file);
        for (const pid_t pid : pids)
        {
            if (isProcessAlive(pid))
            {
                return true;
            }
        }

        const bool pid_file_is_still_warming_up =
            pids.empty() &&
            it->second.terminal.start_time.msecsTo(QDateTime::currentDateTime()) < kPidFileWarmupMsec &&
            !QFileInfo::exists(it->second.terminal.done_file);
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

    void stopQuickLaunch(const int idx)
    {
        const auto it = quick_launch_runtimes_.find(idx);
        if (it == quick_launch_runtimes_.end())
        {
            return;
        }

        appendLog("[Sunray] 一键停止脚本: " + quick_launch_groups_[static_cast<size_t>(idx)].title);
        const std::vector<pid_t> pids = readPidListFile(it->second.terminal.pid_file);
        for (const pid_t pid : pids)
        {
            ::kill(-pid, SIGINT);
            ::kill(pid, SIGINT);
        }

        const auto terminal_it = quick_terminal_processes_.find(idx);
        if (terminal_it != quick_terminal_processes_.end() && terminal_it->second)
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

        std::vector<int> quick_running;
        for (const auto &entry : quick_launch_runtimes_)
        {
            if (isQuickLaunchRunning(entry.first))
            {
                quick_running.push_back(entry.first);
            }
        }
        for (const int idx : quick_running)
        {
            stopQuickLaunch(idx);
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
        cacheLaunchCommand(current_selected_launch_index_);

        int idx = -1;
        if (current)
        {
            const QVariant value = current->data(0, kLaunchIndexRole);
            if (value.isValid())
            {
                idx = value.toInt();
            }
        }
        current_selected_launch_index_ = idx;

        if (idx >= 0 && idx < static_cast<int>(launch_items_.size()))
        {
            const LaunchItem &item = launch_items_[static_cast<size_t>(idx)];
            const auto edited_it = edited_launch_commands_.find(idx);
            selected_command_->setPlainText(edited_it != edited_launch_commands_.end()
                                                ? edited_it->second
                                                : buildDisplayCommand(item));
        }
        else
        {
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

        int quick_running_count = 0;
        std::vector<int> finished_quick_launches;
        for (auto &entry : quick_launch_runtimes_)
        {
            const int idx = entry.first;
            if (!isQuickLaunchRunning(idx))
            {
                if (entry.second.running)
                {
                    const std::string exit_code = readFirstLine(entry.second.terminal.done_file);
                    updateQuickLaunchStatus(idx, "未运行");
                    markLinkedLaunchesRunning(entry.second, "未运行");
                    appendLog("[Sunray] 一键启动脚本结束: " + quick_launch_groups_[static_cast<size_t>(idx)].title +
                              (exit_code.empty() ? "" : " exit_code=" + exit_code));
                    entry.second.running = false;
                    entry.second.terminal.running = false;
                    finished_quick_launches.push_back(idx);
                }
                continue;
            }

            ++quick_running_count;
            const qint64 secs = entry.second.terminal.start_time.secsTo(QDateTime::currentDateTime());
            updateQuickLaunchStatus(idx, QString("运行 %1s").arg(secs));
            markLinkedLaunchesRunning(entry.second, "运行(快速)");
        }
        for (const int idx : finished_quick_launches)
        {
            const auto it = quick_launch_runtimes_.find(idx);
            if (it != quick_launch_runtimes_.end())
            {
                cleanupRuntimeFiles(it->second.terminal);
                quick_launch_runtimes_.erase(it);
            }
        }

        updateGroupTabStatus();
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
        if (ros_state_marker_)
        {
            if (got_nodes)
            {
                ros_state_marker_->setStyleSheet("background: #60f46d; border-radius: 6px;");
            }
            else
            {
                ros_state_marker_->setStyleSheet("background: #9caaa3; border-radius: 6px;");
            }
        }
        if (ros_state_text_)
        {
            ros_state_text_->setText(got_nodes ? "状态: 在线" : "状态: 离线");
        }
        if (ros_nodes_view_)
        {
            QStringList lines;
            if (got_nodes)
            {
                for (const std::string &node : running_nodes)
                {
                    lines << formatRosNodeNameForDisplay(node);
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
        const bool running = valid && isLaunchActive(idx);
        start_btn_->setEnabled(valid && !running);
        stop_btn_->setEnabled(running);

        const int quick_idx = selectedQuickLaunchIndex();
        const bool quick_valid = quick_idx >= 0;
        const bool quick_running = quick_valid && isQuickLaunchRunning(quick_idx);
        if (quick_start_btn_)
        {
            quick_start_btn_->setEnabled(quick_valid && !quick_running);
        }
        if (quick_stop_btn_)
        {
            quick_stop_btn_->setEnabled(quick_running);
        }
    }

    std::vector<int> selectedQuickLaunchPreviewIndices() const
    {
        const int quick_idx = selectedQuickLaunchIndex();
        if (quick_idx < 0 || quick_idx >= static_cast<int>(quick_launch_groups_.size()))
        {
            return {};
        }
        return collectLinkedLaunchIndices(quick_launch_groups_[static_cast<size_t>(quick_idx)]);
    }

    bool isLaunchInSelectedQuickPreview(const int idx) const
    {
        const std::vector<int> preview_indices = selectedQuickLaunchPreviewIndices();
        return std::find(preview_indices.begin(), preview_indices.end(), idx) != preview_indices.end();
    }

    bool isLaunchSelected(const int idx) const
    {
        return current_selected_launch_index_ == idx;
    }

    QString originalTitleOf(const QTreeWidgetItem *item) const
    {
        if (!item)
        {
            return "";
        }
        const QVariant value = item->data(0, kOriginalTitleRole);
        if (value.isValid())
        {
            return value.toString();
        }
        return item->text(0).trimmed().remove(QRegularExpression("^[▌▎\\s]+"));
    }

    void updateLaunchStatus(const int idx, const QString &status)
    {
        const auto it = item_by_launch_index_.find(idx);
        if (it != item_by_launch_index_.end() && it->second)
        {
            const bool running = status.startsWith("运行");
            const bool preview = status.startsWith("预览");
            it->second->setData(1, kBadgeStateRole, running ? static_cast<int>(BadgeState::Running)
                                                            : (preview ? static_cast<int>(BadgeState::Preview)
                                                                       : static_cast<int>(BadgeState::Idle)));
            it->second->setText(1, running ? "运行" : (preview ? "预览" : "未运行"));
            applyLaunchRowStyle(idx);
        }
    }

    void applyLaunchRowStyle(const int idx)
    {
        const auto it = item_by_launch_index_.find(idx);
        if (it == item_by_launch_index_.end() || !it->second)
        {
            return;
        }

        const QString status = it->second->text(1);
        const bool running = status.startsWith("运行");
        const bool preview = !running && isLaunchInSelectedQuickPreview(idx);
        const bool selected = isLaunchSelected(idx);
        const QColor row_background = running
                                          ? kSunrayRunningBg
                                          : (preview ? kSunrayPreviewBg : (selected ? kSunraySelectedBg : kSunrayNormalBg));
        const QColor row_foreground = running
                                          ? kSunrayRunningFg
                                          : (preview ? kSunrayPreviewFg : (selected ? kSunraySelectedFg : kSunrayNormalFg));
        for (int column = 0; column < it->second->columnCount(); ++column)
        {
            it->second->setBackground(column, row_background);
            it->second->setForeground(column, row_foreground);
            QFont font = it->second->font(column);
            font.setBold(selected || running);
            it->second->setFont(column, font);
        }
        const QString title = originalTitleOf(it->second);
        const QString marker = running ? "▌" : (preview ? "▎" : (selected ? "▌" : " "));
        it->second->setText(0, QString("%1 %2").arg(marker, title));
    }

    void updateLaunchPreviewHighlight()
    {
        const int quick_idx = selectedQuickLaunchIndex();
        for (const auto &entry : item_by_launch_index_)
        {
            applyLaunchRowStyle(entry.first);
        }
        for (const auto &entry : quick_item_by_index_)
        {
            if (!entry.second)
            {
                continue;
            }
            const bool selected = quick_idx == entry.first;
            const bool running = entry.second->text(1).startsWith("运行");
            const QColor background = running ? kSunrayRunningBg : (selected ? kSunraySelectedBg : kSunrayNormalBg);
            const QColor foreground = running ? kSunrayRunningFg : (selected ? kSunraySelectedFg : kSunrayNormalFg);
            for (int column = 0; column < entry.second->columnCount(); ++column)
            {
                entry.second->setBackground(column, background);
                entry.second->setForeground(column, foreground);
                QFont font = entry.second->font(column);
                font.setBold(selected || running);
                entry.second->setFont(column, font);
            }
            const QString title = originalTitleOf(entry.second);
            const QString marker = running ? "▌" : (selected ? "▌" : " ");
            entry.second->setText(0, QString("%1 %2").arg(marker, title));
        }
    }

    void updateQuickLaunchStatus(const int idx, const QString &status)
    {
        const auto it = quick_item_by_index_.find(idx);
        if (it != quick_item_by_index_.end() && it->second)
        {
            const bool running = status.startsWith("运行");
            const bool preview = status.startsWith("预览");
            it->second->setData(1, kBadgeStateRole, running ? static_cast<int>(BadgeState::Running)
                                                            : (preview ? static_cast<int>(BadgeState::Preview)
                                                                       : static_cast<int>(BadgeState::Idle)));
            it->second->setText(1, running ? "运行" : (preview ? "预览" : "未运行"));
            const QColor row_background = running ? kSunrayRunningBg : QColor(251, 252, 250);
            const QColor row_foreground = running ? kSunrayRunningFg : QColor(23, 33, 28);
            for (int column = 0; column < it->second->columnCount(); ++column)
            {
                it->second->setBackground(column, row_background);
                it->second->setForeground(column, row_foreground);
            }
            updateLaunchPreviewHighlight();
            updateQuickLaunchTabStatus();
        }
    }

    int runningCountForQuickCategory(const std::string &category) const
    {
        int running_count = 0;
        for (int i = 0; i < static_cast<int>(quick_launch_groups_.size()); ++i)
        {
            const QuickLaunchGroup &group = quick_launch_groups_[static_cast<size_t>(i)];
            if (group.category == category && isQuickLaunchRunning(i))
            {
                ++running_count;
            }
        }
        return running_count;
    }

    int previewCountForQuickCategory(const std::string &category) const
    {
        const int selected_idx = selectedQuickLaunchIndex();
        if (selected_idx < 0 || selected_idx >= static_cast<int>(quick_launch_groups_.size()))
        {
            return 0;
        }

        const QuickLaunchGroup &group = quick_launch_groups_[static_cast<size_t>(selected_idx)];
        return group.category == category && !isQuickLaunchRunning(selected_idx) ? 1 : 0;
    }

    void updateQuickLaunchTabStatus()
    {
        if (!quick_launch_tabs_)
        {
            return;
        }

        for (int i = 0; i < static_cast<int>(quick_launch_categories_.size()) && i < quick_launch_tabs_->count(); ++i)
        {
            const std::string &category = quick_launch_categories_[static_cast<size_t>(i)];
            const int running_count = runningCountForQuickCategory(category);
            const int preview_count = previewCountForQuickCategory(category);
            QString tab_text = QString::fromStdString(category);
            quick_launch_tabs_->setTabText(i, tab_text);
            if (quick_launch_tab_bar_)
            {
                quick_launch_tab_bar_->setRunningCount(i, running_count);
                quick_launch_tab_bar_->setPreviewCount(i, preview_count);
            }
        }
    }

    int runningCountForGroup(const LaunchGroup &group) const
    {
        int running_count = 0;
        for (const int idx : group.item_indices)
        {
            if (isLaunchActive(idx))
            {
                ++running_count;
            }
        }
        return running_count;
    }

    int previewCountForGroup(const LaunchGroup &group) const
    {
        const std::vector<int> preview_indices = selectedQuickLaunchPreviewIndices();
        int preview_count = 0;
        for (const int idx : group.item_indices)
        {
            if (std::find(preview_indices.begin(), preview_indices.end(), idx) != preview_indices.end() &&
                !isLaunchActive(idx))
            {
                ++preview_count;
            }
        }
        return preview_count;
    }

    void updateGroupTabStatus()
    {
        if (!launch_tabs_)
        {
            return;
        }

        for (int i = 0; i < static_cast<int>(launch_groups_.size()) && i < launch_tabs_->count(); ++i)
        {
            const LaunchGroup &group = launch_groups_[static_cast<size_t>(i)];
            const int running_count = runningCountForGroup(group);
            const int preview_count = previewCountForGroup(group);
            QString tab_text = QString::fromStdString(group.name);
            if (running_count > 0)
            {
                tab_text += QString("-%1").arg(running_count);
            }
            else if (preview_count > 0)
            {
                tab_text += QString("-%1").arg(preview_count);
            }
            launch_tabs_->setTabText(i, tab_text);
            if (launch_tab_bar_)
            {
                launch_tab_bar_->setRunningCount(i, running_count);
                launch_tab_bar_->setPreviewCount(i, preview_count);
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

    std::string launch_groups_config_path_;
    std::string quick_launch_groups_config_path_;

    std::vector<ExternalWorkspace> external_workspaces_;
    std::map<std::string, std::string> external_package_paths_;

    std::vector<LaunchItem> launch_items_;
    std::vector<LaunchGroup> launch_groups_;
    std::vector<QuickLaunchGroup> quick_launch_groups_;
    std::vector<std::string> quick_launch_categories_;
    std::map<int, QTreeWidgetItem *> item_by_launch_index_;
    std::map<int, QTreeWidgetItem *> quick_item_by_index_;
    std::map<int, QProcess *> terminal_processes_;
    std::map<int, LaunchRuntime> launch_runtimes_;
    std::map<int, QProcess *> quick_terminal_processes_;
    std::map<int, QuickLaunchRuntime> quick_launch_runtimes_;
    std::map<int, QString> edited_launch_commands_;
    int current_selected_launch_index_{-1};

    QTimer *refresh_timer_{nullptr};
    QDateTime last_monitor_refresh_;
    CpuSample last_cpu_sample_;

    LaunchTabWidget *launch_tabs_{nullptr};
    LaunchTabBar *launch_tab_bar_{nullptr};
    LaunchTabWidget *quick_launch_tabs_{nullptr};
    LaunchTabBar *quick_launch_tab_bar_{nullptr};
    std::vector<QTreeWidget *> launch_trees_;
    std::vector<QTreeWidget *> quick_launch_trees_;
    QTextEdit *selected_command_{nullptr};
    QTextEdit *log_view_{nullptr};
    QLabel *cpu_label_{nullptr};
    QLabel *memory_label_{nullptr};
    QLabel *ros_node_count_label_{nullptr};
    QFrame *ros_state_marker_{nullptr};
    QLabel *ros_state_text_{nullptr};
    QProgressBar *cpu_bar_{nullptr};
    QProgressBar *memory_bar_{nullptr};
    QTextEdit *ros_nodes_view_{nullptr};
    QPushButton *start_btn_{nullptr};
    QPushButton *stop_btn_{nullptr};
    QPushButton *quick_start_btn_{nullptr};
    QPushButton *quick_stop_btn_{nullptr};
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
