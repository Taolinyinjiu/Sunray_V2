#include <algorithm>
#include <atomic>
#include <chrono>
#include <clocale>
#include <cstdint>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <mavros_msgs/FileClose.h>
#include <mavros_msgs/FileEntry.h>
#include <mavros_msgs/FileList.h>
#include <mavros_msgs/FileOpen.h>
#include <mavros_msgs/FileRead.h>
#include <mavros_msgs/FileRemove.h>
#include <ros/package.h>
#include <ros/master.h>
#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <fcntl.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include "ftxui/component/component.hpp"
#include "ftxui/component/event.hpp"
#include "ftxui/component/screen_interactive.hpp"
#include "ftxui/dom/elements.hpp"
#include "ftxui/screen/color.hpp"
#include "ftxui/screen/terminal.hpp"

namespace {

using ftxui::CatchEvent;
using ftxui::Color;
using ftxui::Component;
using ftxui::Element;
using ftxui::Elements;
using ftxui::Event;
using ftxui::Renderer;
using ftxui::ScreenInteractive;
using ftxui::EQUAL;
using ftxui::HEIGHT;
using ftxui::LESS_THAN;
using ftxui::WIDTH;
using ftxui::align_right;
using ftxui::bgcolor;
using ftxui::bold;
using ftxui::border;
using ftxui::center;
using ftxui::color;
using ftxui::dim;
using ftxui::filler;
using ftxui::flex;
using ftxui::focus;
using ftxui::hbox;
using ftxui::paragraph;
using ftxui::separator;
using ftxui::separatorLight;
using ftxui::size;
using ftxui::text;
using ftxui::vbox;
using ftxui::vscroll_indicator;
using ftxui::window;
using ftxui::yframe;

constexpr uint64_t kFtpReadChunkSize = 239U * 18U - 1U;

std::string normalize_namespace(std::string value) {
  if (value.empty()) {
    return "";
  }
  if (value.front() != '/') {
    value.insert(value.begin(), '/');
  }
  while (value.size() > 1 && value.back() == '/') {
    value.pop_back();
  }
  return value;
}

bool ends_with_path_token(const std::string& value, const std::string& suffix) {
  if (value.size() < suffix.size()) {
    return false;
  }
  return value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

std::string join_remote_path(const std::string& parent, const std::string& child) {
  if (parent.empty() || parent == "/") {
    return "/" + child;
  }
  if (parent.back() == '/') {
    return parent + child;
  }
  return parent + "/" + child;
}

std::string trim_trailing_slashes(std::string value) {
  while (value.size() > 1 && value.back() == '/') {
    value.pop_back();
  }
  return value;
}

bool ends_with(const std::string& value, const std::string& suffix) {
  if (suffix.empty()) {
    return true;
  }
  if (value.size() < suffix.size()) {
    return false;
  }
  return std::equal(suffix.rbegin(), suffix.rend(), value.rbegin());
}

std::string format_bytes(uint64_t bytes) {
  const char* units[] = {"B", "KB", "MB", "GB"};
  double value = static_cast<double>(bytes);
  size_t unit_index = 0;
  while (value >= 1024.0 && unit_index + 1 < std::size(units)) {
    value /= 1024.0;
    ++unit_index;
  }

  std::ostringstream stream;
  if (unit_index == 0) {
    stream << bytes << " " << units[unit_index];
  } else {
    stream << std::fixed << std::setprecision(value >= 10.0 ? 1 : 2) << value
           << " " << units[unit_index];
  }
  return stream.str();
}

std::string format_percent(uint64_t done, uint64_t total) {
  if (total == 0) {
    return "0%";
  }
  const double percent =
      std::min(100.0, (static_cast<double>(done) * 100.0) /
                          static_cast<double>(total));
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(1) << percent << "%";
  return stream.str();
}

std::string format_time(std::chrono::system_clock::time_point time_point) {
  if (time_point.time_since_epoch().count() == 0) {
    return "-";
  }

  std::time_t raw_time = std::chrono::system_clock::to_time_t(time_point);
  std::tm local_time{};
  localtime_r(&raw_time, &local_time);

  char buffer[32] = {};
  std::strftime(buffer, sizeof(buffer), "%H:%M:%S", &local_time);
  return buffer;
}

std::string errno_message(int remote_errno) {
  if (remote_errno == 0) {
    return "remote errno 0";
  }
  return "remote errno " + std::to_string(remote_errno);
}

std::string shell_log_path() {
  return "/tmp/sunray_flight_logger_mavros.log";
}

std::string detect_repo_root() {
  std::string package_path = ros::package::getPath("sunray_flight_logger");
  if (!package_path.empty()) {
    std::filesystem::path path(package_path);
    for (int depth = 0; depth < 8 && !path.empty(); ++depth) {
      if (std::filesystem::exists(path / "build.sh") &&
          std::filesystem::exists(path / "tools")) {
        return path.string();
      }
      path = path.parent_path();
    }
  }

  std::filesystem::path path = std::filesystem::current_path();
  for (int depth = 0; depth < 8 && !path.empty(); ++depth) {
    if (std::filesystem::exists(path / "build.sh") &&
        std::filesystem::exists(path / "tools")) {
      return path.string();
    }
    path = path.parent_path();
  }

  return std::filesystem::current_path().string();
}

std::filesystem::path default_download_root() {
  return std::filesystem::path(detect_repo_root()) / "sunray_logs" / "flight_log";
}

std::string relative_log_path(const std::string& remote_path,
                              const std::string& log_root) {
  std::string root = trim_trailing_slashes(log_root);
  if (!root.empty() && remote_path.rfind(root + "/", 0) == 0) {
    return remote_path.substr(root.size() + 1);
  }

  if (!remote_path.empty() && remote_path.front() == '/') {
    return remote_path.substr(1);
  }
  return remote_path;
}

class TerminalGuard {
 public:
  TerminalGuard() = default;
  ~TerminalGuard() {
    std::cout << "\033[?25h"
              << "\033[0m"
              << "\033[?1000l"
              << "\033[?1002l"
              << "\033[?1003l"
              << "\033[?2004l"
              << std::flush;
  }

  TerminalGuard(const TerminalGuard&) = delete;
  TerminalGuard& operator=(const TerminalGuard&) = delete;
};

struct FlightLogInfo {
  int id = 0;
  std::string name;
  std::string remote_path;
  uint64_t size = 0;
  bool selected = false;
};

struct OperationState {
  bool running = false;
  std::string name;
  std::string current_file;
  uint64_t current_done = 0;
  uint64_t current_total = 0;
  int completed = 0;
  int total = 0;
};

struct FtpServiceSnapshot {
  std::set<std::string> services;
  std::vector<std::string> mavros_nodes;
};

class SunrayFlightLoggerTui {
 public:
  explicit SunrayFlightLoggerTui(ros::NodeHandle private_nh)
      : private_nh_(std::move(private_nh)) {
    private_nh_.param<std::string>("mavros_ns", preferred_mavros_ns_, "");
    private_nh_.param<std::string>("log_root", log_root_, "/fs/microsd/log");
    private_nh_.param<std::string>("download_root", download_root_param_, "");
    private_nh_.param<std::string>("log_extension_filter", extension_filter_,
                                   ".ulg");
    private_nh_.param<double>("refresh_period_sec", refresh_period_sec_, 10.0);
    private_nh_.param<int>("max_scan_depth", max_scan_depth_, 6);
    private_nh_.param<bool>("auto_start_mavros", auto_start_mavros_, true);
    private_nh_.param<double>("mavros_start_wait_sec", mavros_start_wait_sec_,
                              18.0);
    private_nh_.param<std::string>("mavros_launch_package",
                                   mavros_launch_package_, "sunray_mavros");
    private_nh_.param<std::string>("mavros_launch_file", mavros_launch_file_,
                                   "mavros.launch");
    private_nh_.param<std::string>("agent_name", agent_name_, "uav");
    private_nh_.param<int>("agent_id", agent_id_, 1);
    private_nh_.param<std::string>("fcu_url", fcu_url_, "/dev/ttyACM0:921600");
    private_nh_.param<std::string>("gcs_ip", gcs_ip_, "0.0.0.0");

    preferred_mavros_ns_ = normalize_namespace(preferred_mavros_ns_);
    log_root_ = trim_trailing_slashes(log_root_);
    if (download_root_param_.empty()) {
      download_root_ = default_download_root();
    } else {
      download_root_ = std::filesystem::path(download_root_param_);
    }

    if (!preferred_mavros_ns_.empty()) {
      configure_ftp_clients(preferred_mavros_ns_);
    }
  }

  ~SunrayFlightLoggerTui() {
    stop_refresh_.store(true);
    if (refresh_thread_.joinable()) {
      refresh_thread_.join();
    }
    if (operation_thread_.joinable()) {
      operation_thread_.join();
    }
    stop_started_mavros_launch();
  }

  int run() {
    TerminalGuard guard;
    refresh_logs(preferred_mavros_ns_.empty()
                     ? "正在自动发现 MAVROS ..."
                     : "正在连接 " + preferred_mavros_ns_ + " ...",
                 true);

    ScreenInteractive screen = ScreenInteractive::Fullscreen();
    screen_.store(&screen);

    auto component = create_component(screen);
    stop_refresh_.store(false);
    refresh_thread_ = std::thread([this]() { refresh_loop(); });

    screen.Loop(component);

    stop_refresh_.store(true);
    cancel_operation_.store(true);
    if (refresh_thread_.joinable()) {
      refresh_thread_.join();
    }
    if (operation_thread_.joinable()) {
      operation_thread_.join();
    }
    screen_.store(nullptr);
    return 0;
  }

 private:
  Component create_component(ScreenInteractive& screen) {
    auto renderer = Renderer([this]() { return render(); });

    return CatchEvent(renderer, [this, &screen](Event event) {
      if (event == Event::Escape || event == Event::Character('q') ||
          event == Event::Character('Q')) {
        screen.Exit();
        return true;
      }
      if (event == Event::ArrowUp || event == Event::Character('k')) {
        move_selection(-1);
        return true;
      }
      if (event == Event::ArrowDown || event == Event::Character('j')) {
        move_selection(1);
        return true;
      }
      if (event == Event::PageUp) {
        move_selection(-10);
        return true;
      }
      if (event == Event::PageDown) {
        move_selection(10);
        return true;
      }
      if (event == Event::Home) {
        move_selection_to_edge(true);
        return true;
      }
      if (event == Event::End) {
        move_selection_to_edge(false);
        return true;
      }
      if (event == Event::Character(' ')) {
        toggle_selected_log();
        return true;
      }
      if (event == Event::Character('a') || event == Event::Character('A')) {
        toggle_all_logs();
        return true;
      }
      if (event == Event::Return || event == Event::F1) {
        start_download_selected();
        return true;
      }
      if (event == Event::Character('d')) {
        start_delete_selected();
        return true;
      }
      if (event == Event::Character('D')) {
        start_delete_all();
        return true;
      }
      if (event == Event::F5 || event == Event::Character('r') ||
          event == Event::Character('R')) {
        if (operation_running_locked()) {
          std::lock_guard<std::recursive_mutex> lock(state_mutex_);
          status_message_ = "任务执行中，暂不刷新";
          status_is_error_ = true;
          return true;
        }
        refresh_logs("手动刷新完成", false);
        return true;
      }
      return false;
    });
  }

  Element render() {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    auto terminal = ftxui::Terminal::Size();
    const int width = terminal.dimx;
    const int height = terminal.dimy;

    if (width < 92 || height < 24) {
      return vbox({
                 filler(),
                 text("Sunray Flight Logger") | bold | center,
                 separatorLight(),
                 text("当前终端窗口太小，至少需要 92x24。") |
                     color(Color::Red) | bold | center,
                 text("请继续放大终端窗口。") | color(Color::GrayLight) |
                     center,
                 filler(),
                 text("Esc / q 退出") | center,
             }) |
             border;
    }

    const int sidebar_width = std::max(34, std::min(50, width * 34 / 100));
    const int detail_height = std::max(7, std::min(12, height / 3));

    Element header =
        hbox({
            text(" Sunray Flight Logger ") | bold | color(Color::Black) |
                bgcolor(Color::Cyan),
            text(" "),
            text(mavros_ns_.empty() ? "自动发现 MAVROS" : mavros_ns_) |
                color(Color::Yellow) | bold,
            text("  日志目录: ") | color(Color::GrayLight),
            text(log_root_) | color(Color::White),
            filler(),
            text("最后刷新 " + format_time(last_refresh_wall_time_)) |
                color(Color::GrayLight),
            text(" "),
        });

    Element status = text("状态: " + status_message_) |
                     color(status_is_error_ ? Color::Red : Color::GrayLight);

    Element content =
        hbox({
            render_log_panel() | flex,
            render_side_panel() | size(WIDTH, EQUAL, sidebar_width),
        }) |
        flex;

    Element key_guide =
        text("↑/↓ 选择   Space 多选   a 全选/取消   Enter/F1 下载选中   d 删除选中   D 删除全部   F5/r 刷新   Esc/q 退出") |
        color(Color::GrayLight) | center;

    return vbox({
               header,
               status,
               separator(),
               content,
               separatorLight(),
               render_detail_panel() | size(HEIGHT, EQUAL, detail_height),
               separatorLight(),
               key_guide,
           }) |
           border;
  }

  Element render_log_panel() const {
    Element body;
    if (logs_.empty()) {
      body = vbox({
          text("飞控日志") | bold | center,
          separatorLight(),
          filler(),
          text("没有发现日志文件") | dim | center,
          text("确认 PX4 已挂载 SD 卡，且 MAVROS FTP 插件可用") |
              color(Color::GrayLight) | center,
          filler(),
      });
    } else {
      body = vbox({
          hbox({
              text("飞控日志") | bold | center | flex,
              text(" " + std::to_string(selected_count_locked()) + "/" +
                   std::to_string(logs_.size()) + " 已选 ") |
                  color(Color::Yellow),
          }),
          separatorLight(),
          render_log_rows_locked() | yframe | vscroll_indicator | flex,
      });
    }

    return body | border | color(Color::Cyan);
  }

  Element render_log_rows_locked() const {
    Elements rows;
    rows.push_back(hbox({
        text("  ") | size(WIDTH, EQUAL, 3),
        text("ID") | bold | size(WIDTH, EQUAL, 5),
        text("Name") | bold | flex,
        text("Size") | bold | align_right | size(WIDTH, EQUAL, 12),
    }) | color(Color::GrayLight));
    rows.push_back(separatorLight());

    for (int index = 0; index < static_cast<int>(logs_.size()); ++index) {
      const FlightLogInfo& log = logs_[index];
      const bool focused_row = index == log_selection_;
      const std::string marker = log.selected ? "[x]" : "[ ]";

      Element row =
          hbox({
              text(marker + " ") | size(WIDTH, EQUAL, 4),
              text(std::to_string(log.id)) | size(WIDTH, EQUAL, 5),
              text(log.name) | flex,
              text(format_bytes(log.size)) | align_right |
                  size(WIDTH, EQUAL, 12),
          });

      if (focused_row) {
        row = row | bgcolor(Color::Blue) | color(Color::White) | bold | focus;
      } else if (log.selected) {
        row = row | color(Color::Yellow) | bold;
      } else {
        row = row | color(Color::White);
      }
      rows.push_back(row);
    }
    return vbox(rows);
  }

  Element render_side_panel() const {
    const uint64_t total_bytes = total_log_bytes_locked();
    Elements operation_rows;
    if (operation_.running) {
      operation_rows.push_back(text(operation_.name) | bold | color(Color::Yellow));
      operation_rows.push_back(text(operation_.current_file.empty()
                                        ? "-"
                                        : operation_.current_file) |
                               color(Color::White));
      operation_rows.push_back(
          text(std::to_string(operation_.completed) + "/" +
               std::to_string(operation_.total) + " files") |
          color(Color::GrayLight));
      operation_rows.push_back(
          text(format_bytes(operation_.current_done) + " / " +
               format_bytes(operation_.current_total) + " (" +
               format_percent(operation_.current_done, operation_.current_total) +
               ")") |
          color(Color::GrayLight));
    } else {
      operation_rows.push_back(text("空闲") | color(Color::Green) | bold);
    }

    return window(
        text("操作") | bold,
        vbox({
            hbox({text("日志数量: ") | bold,
                  text(std::to_string(logs_.size())) | align_right}),
            hbox({text("已选择: ") | bold,
                  text(std::to_string(selected_count_locked())) | align_right}),
            hbox({text("总大小: ") | bold,
                  text(format_bytes(total_bytes)) | align_right}),
            separatorLight(),
            text("默认下载路径") | bold | color(Color::Cyan),
            paragraph(download_root_.string()) | color(Color::GrayLight),
            separatorLight(),
            text("MAVROS") | bold | color(Color::Cyan),
            paragraph(mavros_ns_.empty() ? "等待自动发现" : mavros_ns_) |
                color(Color::GrayLight),
            text(mavros_launch_started_
                     ? ("本工具已启动 " + mavros_launch_package_ + "/" +
                        mavros_launch_file_)
                     : "使用已有节点或等待自动启动") |
                color(Color::GrayLight),
            separatorLight(),
            text("按钮") | bold | color(Color::Cyan),
            render_button_row("Enter", "下载选中", Color::Green),
            render_button_row("d", "删除选中", Color::Red),
            render_button_row("D", "清除全部", Color::Red),
            render_button_row("F5", "刷新", Color::Blue),
            separatorLight(),
            text("当前任务") | bold | color(Color::Cyan),
            vbox(operation_rows),
        }));
  }

  Element render_button_row(const std::string& key, const std::string& label,
                            Color color_value) const {
    return hbox({
        text("[" + key + "] ") | color(color_value) | bold,
        text(label) | color(Color::White),
    });
  }

  Element render_detail_panel() const {
    const FlightLogInfo* log = selected_log_locked();
    if (log == nullptr) {
      return window(text("日志详情") | bold,
                    vbox({text("没有选中的日志") | dim}));
    }

    return window(
        text("日志详情") | bold,
        vbox({
            hbox({text("ID: ") | bold, text(std::to_string(log->id)),
                  text("   名字: ") | bold, text(log->name) | flex,
                  text("大小: ") | bold, text(format_bytes(log->size))}),
            hbox({text("远端路径: ") | bold,
                  paragraph(log->remote_path) | color(Color::GrayLight) | flex}),
            hbox({text("本地保存: ") | bold,
                  paragraph(local_path_for_log(*log).string()) |
                      color(Color::GrayLight) | flex}),
        }));
  }

  void refresh_loop() {
    const auto period = std::chrono::milliseconds(
        std::max(1, static_cast<int>(refresh_period_sec_ * 1000.0)));
    while (!stop_refresh_.load() && ros::ok()) {
      std::this_thread::sleep_for(period);
      if (stop_refresh_.load() || !ros::ok()) {
        break;
      }

      bool operation_running = false;
      {
        std::lock_guard<std::recursive_mutex> lock(state_mutex_);
        operation_running = operation_.running;
      }
      if (!operation_running) {
        refresh_logs("", false);
        post_refresh_event();
      }
    }
  }

  void post_refresh_event() {
    ScreenInteractive* screen = screen_.load();
    if (screen != nullptr) {
      screen->PostEvent(Event::Custom);
    }
  }

  void configure_ftp_clients(const std::string& mavros_ns) {
    mavros_ns_ = normalize_namespace(mavros_ns);
    list_client_ =
        root_nh_.serviceClient<mavros_msgs::FileList>(mavros_ns_ + "/ftp/list");
    open_client_ =
        root_nh_.serviceClient<mavros_msgs::FileOpen>(mavros_ns_ + "/ftp/open");
    read_client_ =
        root_nh_.serviceClient<mavros_msgs::FileRead>(mavros_ns_ + "/ftp/read");
    close_client_ =
        root_nh_.serviceClient<mavros_msgs::FileClose>(mavros_ns_ + "/ftp/close");
    remove_client_ =
        root_nh_.serviceClient<mavros_msgs::FileRemove>(mavros_ns_ + "/ftp/remove");
  }

  bool ftp_services_available(double timeout_sec, std::string* missing_service) {
    if (mavros_ns_.empty()) {
      if (missing_service != nullptr) {
        *missing_service = "MAVROS namespace";
      }
      return false;
    }

    const ros::Duration timeout(timeout_sec);
    struct ClientCheck {
      ros::ServiceClient* client;
      const char* name;
    };
    const std::vector<ClientCheck> checks = {
        {&list_client_, "/ftp/list"},   {&open_client_, "/ftp/open"},
        {&read_client_, "/ftp/read"},   {&close_client_, "/ftp/close"},
        {&remove_client_, "/ftp/remove"}};

    for (const ClientCheck& check : checks) {
      if (!check.client->waitForExistence(timeout)) {
        if (missing_service != nullptr) {
          *missing_service = mavros_ns_ + check.name;
        }
        return false;
      }
    }
    return true;
  }

  void refresh_logs(const std::string& status_if_success, bool initializing) {
    std::lock_guard<std::mutex> service_lock(service_mutex_);

    std::string missing_service;
    if (!ensure_mavros_ftp_ready(initializing, &missing_service)) {
      std::lock_guard<std::recursive_mutex> state_lock(state_mutex_);
      status_message_ = missing_service;
      status_is_error_ = true;
      return;
    }

    std::string error_message;
    std::vector<FlightLogInfo> refreshed_logs;
    if (!scan_directory(log_root_, 0, &refreshed_logs, &error_message)) {
      std::lock_guard<std::recursive_mutex> state_lock(state_mutex_);
      status_message_ = "刷新失败: " + error_message;
      status_is_error_ = true;
      return;
    }

    std::sort(refreshed_logs.begin(), refreshed_logs.end(),
              [](const FlightLogInfo& lhs, const FlightLogInfo& rhs) {
                return lhs.remote_path < rhs.remote_path;
              });
    for (int index = 0; index < static_cast<int>(refreshed_logs.size()); ++index) {
      refreshed_logs[index].id = index + 1;
    }

    std::lock_guard<std::recursive_mutex> state_lock(state_mutex_);
    std::set<std::string> previously_selected;
    for (const FlightLogInfo& log : logs_) {
      if (log.selected) {
        previously_selected.insert(log.remote_path);
      }
    }
    const std::string old_path =
        selected_log_locked() == nullptr ? "" : selected_log_locked()->remote_path;

    logs_ = std::move(refreshed_logs);
    for (FlightLogInfo& log : logs_) {
      log.selected = previously_selected.count(log.remote_path) > 0;
    }
    restore_selection_locked(old_path);

    last_refresh_wall_time_ = std::chrono::system_clock::now();
    if (!status_if_success.empty()) {
      status_message_ = status_if_success;
    } else {
      status_message_ = "刷新成功，共 " + std::to_string(logs_.size()) + " 个日志";
    }
    status_is_error_ = false;
  }

  bool scan_directory(const std::string& remote_dir, int depth,
                      std::vector<FlightLogInfo>* logs,
                      std::string* error_message) {
    if (depth > max_scan_depth_) {
      return true;
    }

    mavros_msgs::FileList list_srv;
    list_srv.request.dir_path = remote_dir;
    if (!list_client_.call(list_srv)) {
      *error_message = "无法调用 " + mavros_ns_ + "/ftp/list";
      return false;
    }
    if (!list_srv.response.success) {
      *error_message = remote_dir + " list failed: " +
                       errno_message(list_srv.response.r_errno);
      return false;
    }

    for (const mavros_msgs::FileEntry& entry : list_srv.response.list) {
      if (entry.name.empty() || entry.name == "." || entry.name == "..") {
        continue;
      }

      const std::string path = join_remote_path(remote_dir, entry.name);
      if (entry.type == mavros_msgs::FileEntry::TYPE_DIRECTORY) {
        if (!scan_directory(path, depth + 1, logs, error_message)) {
          return false;
        }
        continue;
      }

      if (entry.type != mavros_msgs::FileEntry::TYPE_FILE) {
        continue;
      }
      if (!ends_with(entry.name, extension_filter_)) {
        continue;
      }

      FlightLogInfo log;
      log.name = entry.name;
      log.remote_path = path;
      log.size = entry.size;
      logs->push_back(std::move(log));
    }
    return true;
  }

  void move_selection(int delta) {
    if (delta == 0) {
      return;
    }

    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    if (logs_.empty()) {
      return;
    }
    log_selection_ =
        std::max(0, std::min(log_selection_ + delta,
                             static_cast<int>(logs_.size()) - 1));
  }

  void move_selection_to_edge(bool first) {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    if (logs_.empty()) {
      log_selection_ = 0;
      return;
    }
    log_selection_ = first ? 0 : static_cast<int>(logs_.size()) - 1;
  }

  void toggle_selected_log() {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    FlightLogInfo* log = selected_log_mutable_locked();
    if (log == nullptr) {
      status_message_ = "没有可选择的日志";
      status_is_error_ = true;
      return;
    }
    log->selected = !log->selected;
    status_message_ =
        std::string(log->selected ? "已选择: " : "取消选择: ") + log->name;
    status_is_error_ = false;
  }

  void toggle_all_logs() {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    if (logs_.empty()) {
      status_message_ = "没有可选择的日志";
      status_is_error_ = true;
      return;
    }

    const bool select_all = selected_count_locked() != logs_.size();
    for (FlightLogInfo& log : logs_) {
      log.selected = select_all;
    }
    status_message_ = select_all ? "已全选日志" : "已取消全选";
    status_is_error_ = false;
  }

  void start_download_selected() {
    std::vector<FlightLogInfo> targets = selected_logs_snapshot();
    if (targets.empty()) {
      std::lock_guard<std::recursive_mutex> lock(state_mutex_);
      status_message_ = "请先选择要下载的日志";
      status_is_error_ = true;
      return;
    }
    start_operation("正在下载", targets, false);
  }

  void start_delete_selected() {
    std::vector<FlightLogInfo> targets = selected_logs_snapshot();
    if (targets.empty()) {
      std::lock_guard<std::recursive_mutex> lock(state_mutex_);
      status_message_ = "请先选择要删除的日志";
      status_is_error_ = true;
      return;
    }
    start_operation("正在删除", targets, true);
  }

  void start_delete_all() {
    std::vector<FlightLogInfo> targets;
    {
      std::lock_guard<std::recursive_mutex> lock(state_mutex_);
      targets = logs_;
    }
    if (targets.empty()) {
      std::lock_guard<std::recursive_mutex> lock(state_mutex_);
      status_message_ = "没有可删除的日志";
      status_is_error_ = true;
      return;
    }
    start_operation("正在清除全部", targets, true);
  }

  std::vector<FlightLogInfo> selected_logs_snapshot() const {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    std::vector<FlightLogInfo> targets;
    for (const FlightLogInfo& log : logs_) {
      if (log.selected) {
        targets.push_back(log);
      }
    }
    return targets;
  }

  void start_operation(const std::string& name, std::vector<FlightLogInfo> targets,
                       bool delete_operation) {
    {
      std::lock_guard<std::recursive_mutex> lock(state_mutex_);
      if (operation_.running) {
        status_message_ = "已有任务正在执行";
        status_is_error_ = true;
        return;
      }
    }

    if (operation_thread_.joinable()) {
      operation_thread_.join();
    }

    cancel_operation_.store(false);
    {
      std::lock_guard<std::recursive_mutex> lock(state_mutex_);
      operation_ = OperationState{};
      operation_.running = true;
      operation_.name = name;
      operation_.total = static_cast<int>(targets.size());
      status_message_ = name + "...";
      status_is_error_ = false;
    }
    post_refresh_event();

    operation_thread_ = std::thread([this, targets = std::move(targets),
                                     delete_operation]() {
      run_operation(targets, delete_operation);
    });
  }

  void run_operation(std::vector<FlightLogInfo> targets, bool delete_operation) {
    std::string error_message;
    bool ok = true;

    {
      std::lock_guard<std::mutex> service_lock(service_mutex_);
      for (int index = 0; index < static_cast<int>(targets.size()); ++index) {
        if (cancel_operation_.load() || !ros::ok()) {
          error_message = "任务已取消";
          ok = false;
          break;
        }
        const FlightLogInfo& log = targets[index];
        {
          std::lock_guard<std::recursive_mutex> lock(state_mutex_);
          operation_.current_file = log.name;
          operation_.current_done = 0;
          operation_.current_total = log.size;
          operation_.completed = index;
        }
        post_refresh_event();

        if (delete_operation) {
          ok = remove_remote_file(log, &error_message);
        } else {
          ok = download_remote_file(log, &error_message);
        }
        if (!ok) {
          break;
        }

        {
          std::lock_guard<std::recursive_mutex> lock(state_mutex_);
          operation_.current_done = log.size;
          operation_.completed = index + 1;
        }
        post_refresh_event();
      }
    }

    {
      std::lock_guard<std::recursive_mutex> lock(state_mutex_);
      operation_.running = false;
      operation_.current_file.clear();
      status_message_ =
          ok ? (delete_operation ? "删除完成" : "下载完成") : error_message;
      status_is_error_ = !ok;
    }

    if (ok) {
      refresh_logs(delete_operation ? "删除完成，列表已刷新"
                                    : "下载完成，列表已刷新",
                   false);
    }
    post_refresh_event();
  }

  bool remove_remote_file(const FlightLogInfo& log, std::string* error_message) {
    mavros_msgs::FileRemove remove_srv;
    remove_srv.request.file_path = log.remote_path;
    if (!remove_client_.call(remove_srv)) {
      *error_message = "无法调用 " + mavros_ns_ + "/ftp/remove";
      return false;
    }
    if (!remove_srv.response.success) {
      *error_message = "删除失败 " + log.remote_path + ": " +
                       errno_message(remove_srv.response.r_errno);
      return false;
    }
    return true;
  }

  bool download_remote_file(const FlightLogInfo& log, std::string* error_message) {
    const std::filesystem::path local_path = local_path_for_log(log);
    std::error_code fs_error;
    std::filesystem::create_directories(local_path.parent_path(), fs_error);
    if (fs_error) {
      *error_message = "无法创建目录 " + local_path.parent_path().string() + ": " +
                       fs_error.message();
      return false;
    }

    mavros_msgs::FileOpen open_srv;
    open_srv.request.file_path = log.remote_path;
    open_srv.request.mode = mavros_msgs::FileOpen::Request::MODE_READ;
    if (!open_client_.call(open_srv)) {
      *error_message = "无法调用 " + mavros_ns_ + "/ftp/open";
      return false;
    }
    if (!open_srv.response.success) {
      *error_message = "打开失败 " + log.remote_path + ": " +
                       errno_message(open_srv.response.r_errno);
      return false;
    }

    const uint64_t remote_size = open_srv.response.size == 0 ? log.size
                                                            : open_srv.response.size;
    {
      std::lock_guard<std::recursive_mutex> lock(state_mutex_);
      operation_.current_total = remote_size;
    }

    std::ofstream output(local_path, std::ios::binary | std::ios::trunc);
    if (!output.is_open()) {
      close_remote_file(log.remote_path);
      *error_message = "无法写入本地文件 " + local_path.string();
      return false;
    }

    uint64_t offset = 0;
    bool ok = true;
    while (ros::ok() && !cancel_operation_.load()) {
      mavros_msgs::FileRead read_srv;
      read_srv.request.file_path = log.remote_path;
      read_srv.request.offset = offset;
      read_srv.request.size = kFtpReadChunkSize;
      if (!read_client_.call(read_srv)) {
      *error_message = "无法调用 " + mavros_ns_ + "/ftp/read";
        ok = false;
        break;
      }
      if (!read_srv.response.success) {
        *error_message = "读取失败 " + log.remote_path + ": " +
                         errno_message(read_srv.response.r_errno);
        ok = false;
        break;
      }
      if (read_srv.response.data.empty()) {
        break;
      }

      output.write(
          reinterpret_cast<const char*>(read_srv.response.data.data()),
          static_cast<std::streamsize>(read_srv.response.data.size()));
      if (!output.good()) {
        *error_message = "写入失败 " + local_path.string();
        ok = false;
        break;
      }

      offset += read_srv.response.data.size();
      {
        std::lock_guard<std::recursive_mutex> lock(state_mutex_);
        operation_.current_done = offset;
      }
      post_refresh_event();

      if (remote_size > 0 && offset >= remote_size) {
        break;
      }
    }

    if (cancel_operation_.load() && ok) {
      *error_message = "任务已取消";
      ok = false;
    }

    output.close();
    if (!close_remote_file(log.remote_path) && ok) {
      *error_message = "关闭远端文件失败 " + log.remote_path;
      ok = false;
    }
    return ok;
  }

  bool close_remote_file(const std::string& remote_path) {
    mavros_msgs::FileClose close_srv;
    close_srv.request.file_path = remote_path;
    if (!close_client_.call(close_srv)) {
      return false;
    }
    return close_srv.response.success;
  }

  bool ensure_mavros_ftp_ready(bool initializing, std::string* status_message) {
    if (!preferred_mavros_ns_.empty()) {
      configure_ftp_clients(preferred_mavros_ns_);
      std::string missing_service;
      if (ftp_services_available(initializing ? 1.0 : 0.05, &missing_service)) {
        return true;
      }
    }

    std::string discovered_ns;
    if (discover_ftp_namespace(&discovered_ns)) {
      configure_ftp_clients(discovered_ns);
      return true;
    }

    std::vector<std::string> running_mavros_nodes;
    discover_mavros_nodes(&running_mavros_nodes);
    if (!running_mavros_nodes.empty()) {
      *status_message =
          "检测到 MAVROS 节点 " + running_mavros_nodes.front() +
          "，但未发现 FTP 服务；请确认 MAVROS 加载了 ftp 插件";
      return false;
    }

    if (!auto_start_mavros_) {
      *status_message = "未发现 MAVROS，且 auto_start_mavros=false";
      return false;
    }

    if (!mavros_launch_started_) {
      std::string launch_error;
      if (!start_sunray_mavros_launch(&launch_error)) {
        *status_message = launch_error;
        return false;
      }
    }

    const auto deadline =
        ros::Time::now() + ros::Duration(initializing ? mavros_start_wait_sec_ : 1.0);
    ros::Rate wait_rate(5.0);
    while (ros::ok() && ros::Time::now() < deadline) {
      if (discover_ftp_namespace(&discovered_ns)) {
        configure_ftp_clients(discovered_ns);
        return true;
      }
      wait_rate.sleep();
    }

    *status_message =
        "已启动 " + mavros_launch_package_ + "/" + mavros_launch_file_ +
        "，但仍未发现 MAVROS FTP 服务；日志: " + shell_log_path();
    return false;
  }

  FtpServiceSnapshot read_master_snapshot() const {
    FtpServiceSnapshot snapshot;
    std::vector<std::string> nodes;
    if (ros::master::getNodes(nodes)) {
      for (const std::string& node : nodes) {
        if (ends_with_path_token(node, "/mavros")) {
          snapshot.mavros_nodes.push_back(node);
        }
      }
      std::sort(snapshot.mavros_nodes.begin(), snapshot.mavros_nodes.end());
    }

    XmlRpc::XmlRpcValue args;
    XmlRpc::XmlRpcValue result;
    XmlRpc::XmlRpcValue payload;
    args[0] = ros::this_node::getName();
    if (ros::master::execute("getSystemState", args, result, payload, false) &&
        payload.getType() == XmlRpc::XmlRpcValue::TypeArray &&
        payload.size() >= 3) {
      const XmlRpc::XmlRpcValue& services = payload[2];
      if (services.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for (int index = 0; index < services.size(); ++index) {
          if (services[index].getType() != XmlRpc::XmlRpcValue::TypeArray ||
              services[index].size() < 1) {
            continue;
          }
          if (services[index][0].getType() == XmlRpc::XmlRpcValue::TypeString) {
            snapshot.services.insert(static_cast<std::string>(services[index][0]));
          }
        }
      }
    }

    return snapshot;
  }

  bool namespace_has_ftp_services(const FtpServiceSnapshot& snapshot,
                                  const std::string& ns) const {
    const std::string normalized = normalize_namespace(ns);
    if (normalized.empty()) {
      return false;
    }
    const std::vector<std::string> suffixes = {
        "/ftp/list", "/ftp/open", "/ftp/read", "/ftp/close", "/ftp/remove"};
    for (const std::string& suffix : suffixes) {
      if (snapshot.services.count(normalized + suffix) == 0) {
        return false;
      }
    }
    return true;
  }

  bool discover_ftp_namespace(std::string* mavros_ns) const {
    const FtpServiceSnapshot snapshot = read_master_snapshot();
    if (!preferred_mavros_ns_.empty() &&
        namespace_has_ftp_services(snapshot, preferred_mavros_ns_)) {
      *mavros_ns = preferred_mavros_ns_;
      return true;
    }

    std::vector<std::string> candidates;
    for (const std::string& service : snapshot.services) {
      const std::string suffix = "/ftp/list";
      if (!ends_with_path_token(service, suffix)) {
        continue;
      }
      candidates.push_back(service.substr(0, service.size() - suffix.size()));
    }
    std::sort(candidates.begin(), candidates.end());
    candidates.erase(std::unique(candidates.begin(), candidates.end()),
                     candidates.end());

    for (const std::string& candidate : candidates) {
      if (namespace_has_ftp_services(snapshot, candidate)) {
        *mavros_ns = candidate;
        return true;
      }
    }
    return false;
  }

  void discover_mavros_nodes(std::vector<std::string>* nodes) const {
    const FtpServiceSnapshot snapshot = read_master_snapshot();
    *nodes = snapshot.mavros_nodes;
  }

  bool start_sunray_mavros_launch(std::string* error_message) {
    if (mavros_launch_pid_ > 0) {
      return true;
    }

    const std::vector<std::string> args = {
        "roslaunch",
        mavros_launch_package_,
        mavros_launch_file_,
        "agent_name:=" + agent_name_,
        "agent_id:=" + std::to_string(agent_id_),
        "fcu_url:=" + fcu_url_,
        "gcs_ip:=" + gcs_ip_};

    const pid_t pid = fork();
    if (pid < 0) {
      *error_message = "无法 fork roslaunch 进程";
      return false;
    }

    if (pid == 0) {
      setsid();
      const int log_fd =
          open(shell_log_path().c_str(), O_CREAT | O_WRONLY | O_APPEND, 0644);
      if (log_fd >= 0) {
        dup2(log_fd, STDOUT_FILENO);
        dup2(log_fd, STDERR_FILENO);
        close(log_fd);
      }

      std::vector<char*> argv;
      argv.reserve(args.size() + 1);
      for (const std::string& arg : args) {
        argv.push_back(const_cast<char*>(arg.c_str()));
      }
      argv.push_back(nullptr);
      execvp("roslaunch", argv.data());
      _exit(127);
    }

    mavros_launch_pid_ = pid;
    mavros_launch_started_ = true;
    return true;
  }

  void stop_started_mavros_launch() {
    if (!mavros_launch_started_ || mavros_launch_pid_ <= 0) {
      return;
    }

    kill(-mavros_launch_pid_, SIGTERM);
    for (int attempt = 0; attempt < 20; ++attempt) {
      const pid_t result = waitpid(mavros_launch_pid_, nullptr, WNOHANG);
      if (result == mavros_launch_pid_ || result < 0) {
        mavros_launch_pid_ = -1;
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    kill(-mavros_launch_pid_, SIGKILL);
    waitpid(mavros_launch_pid_, nullptr, 0);
    mavros_launch_pid_ = -1;
  }

  void restore_selection_locked(const std::string& old_path) {
    if (!old_path.empty()) {
      for (int index = 0; index < static_cast<int>(logs_.size()); ++index) {
        if (logs_[index].remote_path == old_path) {
          log_selection_ = index;
          return;
        }
      }
    }
    clamp_selection_locked();
  }

  void clamp_selection_locked() {
    if (logs_.empty()) {
      log_selection_ = 0;
      return;
    }
    log_selection_ = std::max(
        0, std::min(log_selection_, static_cast<int>(logs_.size()) - 1));
  }

  const FlightLogInfo* selected_log_locked() const {
    if (logs_.empty() || log_selection_ < 0 ||
        log_selection_ >= static_cast<int>(logs_.size())) {
      return nullptr;
    }
    return &logs_[log_selection_];
  }

  FlightLogInfo* selected_log_mutable_locked() {
    if (logs_.empty() || log_selection_ < 0 ||
        log_selection_ >= static_cast<int>(logs_.size())) {
      return nullptr;
    }
    return &logs_[log_selection_];
  }

  bool operation_running_locked() const {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    return operation_.running;
  }

  size_t selected_count_locked() const {
    return static_cast<size_t>(std::count_if(
        logs_.begin(), logs_.end(),
        [](const FlightLogInfo& log) { return log.selected; }));
  }

  uint64_t total_log_bytes_locked() const {
    uint64_t total = 0;
    for (const FlightLogInfo& log : logs_) {
      total += log.size;
    }
    return total;
  }

  std::filesystem::path local_path_for_log(const FlightLogInfo& log) const {
    const std::string relative_path = relative_log_path(log.remote_path, log_root_);
    return download_root_ / std::filesystem::path(relative_path);
  }

  ros::NodeHandle private_nh_;
  ros::NodeHandle root_nh_;
  std::string preferred_mavros_ns_;
  std::string mavros_ns_;
  std::string log_root_;
  std::string download_root_param_;
  std::string extension_filter_;
  double refresh_period_sec_ = 10.0;
  int max_scan_depth_ = 6;
  bool auto_start_mavros_ = true;
  double mavros_start_wait_sec_ = 18.0;
  std::string mavros_launch_package_ = "sunray_mavros";
  std::string mavros_launch_file_ = "mavros.launch";
  std::string agent_name_ = "uav";
  int agent_id_ = 1;
  std::string fcu_url_ = "/dev/ttyACM0:921600";
  std::string gcs_ip_ = "0.0.0.0";
  std::filesystem::path download_root_;
  bool mavros_launch_started_ = false;
  pid_t mavros_launch_pid_ = -1;

  ros::ServiceClient list_client_;
  ros::ServiceClient open_client_;
  ros::ServiceClient read_client_;
  ros::ServiceClient close_client_;
  ros::ServiceClient remove_client_;

  mutable std::recursive_mutex state_mutex_;
  std::mutex service_mutex_;
  std::atomic<bool> stop_refresh_{false};
  std::thread refresh_thread_;
  std::thread operation_thread_;
  std::atomic<ScreenInteractive*> screen_{nullptr};
  std::atomic<bool> cancel_operation_{false};

  std::vector<FlightLogInfo> logs_;
  int log_selection_ = 0;
  bool status_is_error_ = false;
  std::string status_message_ = "正在初始化...";
  std::chrono::system_clock::time_point last_refresh_wall_time_;
  OperationState operation_;
};

}  // namespace

int main(int argc, char** argv) {
  std::setlocale(LC_ALL, "");
  ros::init(argc, argv, "sunray_flight_logger_tui");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::NodeHandle private_nh("~");
  SunrayFlightLoggerTui app(private_nh);
  return app.run();
}
