#include <algorithm>
#include <atomic>
#include <chrono>
#include <clocale>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <ros/ros.h>

#include <sunray_msgs/GetFeatures.h>
#include <sunray_msgs/ListFeatures.h>
#include <sunray_msgs/StartFeature.h>
#include <sunray_msgs/StopFeature.h>
#include <sunray_msgs/SystemInfo.h>

#include "ftxui/component/component.hpp"
#include "ftxui/component/component_options.hpp"
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
using ftxui::gauge;
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

std::string join_strings(const std::vector<std::string>& values,
                         const std::string& separator_text) {
  if (values.empty()) {
    return "";
  }
  std::ostringstream stream;
  for (size_t index = 0; index < values.size(); ++index) {
    if (index > 0) {
      stream << separator_text;
    }
    stream << values[index];
  }
  return stream.str();
}

std::string format_float(float value, int precision = 1) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
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

std::string normalize_namespace(std::string value) {
  if (value.empty()) {
    return "";
  }
  while (value.size() > 1 && value.back() == '/') {
    value.pop_back();
  }
  return value;
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

struct FeatureInfo {
  std::string name;
  std::string group = "未分组";
  bool running = false;
  bool auto_start = false;
  float stop_timeout_sec = 0.0F;
  std::string description;
  std::string message;
  std::vector<std::string> depends_on;
  std::vector<std::string> preview_units;
  std::vector<std::string> preview_commands;
};

struct GroupInfo {
  std::string name;
  std::vector<int> feature_indices;
  int running_count = 0;
};

struct SystemInfoSnapshot {
  bool valid = false;
  float cpu_percent = 0.0F;
  float memory_percent = 0.0F;
  std::vector<std::string> active_ros_nodes;
  std::chrono::steady_clock::time_point received_at;
};

class ScriptsManageTui {
 public:
  explicit ScriptsManageTui(ros::NodeHandle private_nh)
      : private_nh_(std::move(private_nh)) {
    private_nh_.param<std::string>("system_ns", system_ns_, "/sunray_system");
    private_nh_.param<std::string>("system_info_topic", system_info_topic_,
                                   "/sunray/system_info");
    private_nh_.param<double>("refresh_period_sec", refresh_period_sec_, 5.0);
    private_nh_.param<bool>("start_with_terminal", start_with_terminal_, true);
    system_ns_ = normalize_namespace(system_ns_);

    ros::NodeHandle nh;
    list_client_ =
        nh.serviceClient<sunray_msgs::ListFeatures>(system_ns_ + "/list_features");
    get_client_ =
        nh.serviceClient<sunray_msgs::GetFeatures>(system_ns_ + "/get_features");
    start_client_ =
        nh.serviceClient<sunray_msgs::StartFeature>(system_ns_ + "/start_feature");
    stop_client_ =
        nh.serviceClient<sunray_msgs::StopFeature>(system_ns_ + "/stop_feature");
    system_info_sub_ = nh.subscribe(system_info_topic_, 1,
                                    &ScriptsManageTui::on_system_info, this);
  }

  int run() {
    TerminalGuard guard;
    refresh_features("正在连接 " + system_ns_ + " ...", true);

    ScreenInteractive screen = ScreenInteractive::Fullscreen();
    screen_.store(&screen);

    auto component = create_component(screen);
    stop_refresh_.store(false);
    refresh_thread_ = std::thread([this]() { refresh_loop(); });

    screen.Loop(component);

    stop_refresh_.store(true);
    if (refresh_thread_.joinable()) {
      refresh_thread_.join();
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
      if (event == Event::F1) {
        toggle_terminal_mode();
        return true;
      }
      if (event == Event::F2 || event == Event::Return) {
        start_selected_feature();
        return true;
      }
      if (event == Event::F3) {
        stop_selected_feature();
        return true;
      }
      if (event == Event::F5) {
        refresh_features("手动刷新完成", false);
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
        move_selection(-8);
        return true;
      }
      if (event == Event::PageDown) {
        move_selection(8);
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
      if (event == Event::Tab) {
        pane_focus_ = (pane_focus_ + 1) % 2;
        return true;
      }
      if (event == Event::ArrowLeft) {
        pane_focus_ = 0;
        return true;
      }
      if (event == Event::ArrowRight) {
        pane_focus_ = 1;
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

    if (width < 82 || height < 24) {
      return vbox({
                 filler(),
                 text("Sunray Scripts Manage") | bold | center,
                 separatorLight(),
                 text("当前终端窗口太小，至少需要 82x24。") | color(Color::Red) |
                     bold | center,
                 text("请继续放大终端窗口。") | color(Color::GrayLight) | center,
                 filler(),
                 text("Esc / q 退出") | center,
             }) |
             border;
    }

    const int sidebar_width = std::max(30, std::min(56, width * 30 / 100));
    const int main_width = std::max(42, width - sidebar_width - 6);
    const int list_width = std::max(24, std::min(42, main_width * 36 / 100));
    const int content_height = std::max(12, height - 6);
    const int min_list_height = 8;
    const int max_detail_height = std::max(8, content_height - min_list_height);
    const int wanted_detail_height = estimate_detail_height_locked(main_width);
    const int detail_height =
        std::max(8, std::min(wanted_detail_height, max_detail_height));

    Element left_panel = render_group_panel(list_width);
    Element right_panel = render_feature_panel();

    Element header = hbox({
        text(" Sunray Scripts Manage ") | bold | color(Color::Black) |
            bgcolor(Color::Cyan),
        text(" "),
        text(start_with_terminal_ ? "终端启动" : "后台启动") | bold |
            color(start_with_terminal_ ? Color::Yellow : Color::Green),
        filler(),
        text("最后刷新 " + format_time(last_refresh_wall_time_)) |
            color(Color::GrayLight),
        text(" "),
    });

    Element status = text("状态: " + status_message_) |
                     color(status_is_error_ ? Color::Red : Color::GrayLight);

    Element columns =
        hbox({
            left_panel | size(WIDTH, LESS_THAN, list_width) | flex,
            right_panel | flex,
        }) |
        flex;

    Element main_area =
        vbox({
            columns,
            separatorLight(),
            render_detail_panel() | size(HEIGHT, EQUAL, detail_height),
        }) |
        flex;

    Element content =
        hbox({
            main_area,
            render_system_panel() | size(WIDTH, EQUAL, sidebar_width),
        }) |
        flex;

    Element key_guide =
        text("↑/↓ 选择   ←/→/Tab 切换焦点   Enter/F2 启动   F3 停止   F1 启动模式   F5 刷新   Esc/q 退出") |
        color(Color::GrayLight) | center;

    return vbox({
               header,
	               status,
	               separator(),
	               content,
	               separatorLight(),
	               key_guide,
	           }) |
	           border;
  }

  Element render_group_panel(int list_width) const {
    Element body = group_entries_.empty()
                       ? vbox({
                             text("功能分组") | bold | center,
                             separatorLight(),
                             text("等待功能列表...") | dim | center,
                         })
                       : vbox({
                             text("功能分组") | bold | center,
                             separatorLight(),
                             render_group_rows_locked() | yframe |
                                 vscroll_indicator | flex,
                         });
    Element panel = body | border;
    if (pane_focus_ == 0) {
      panel = panel | color(Color::Cyan);
    }
    return panel | size(WIDTH, LESS_THAN, list_width);
  }

  Element render_feature_panel() const {
    Element body;
    if (feature_entries_.empty()) {
      body = vbox({
          text("功能列表") | bold | center,
          separatorLight(),
          text("当前分组没有功能") | dim | center,
      });
    } else {
      body = vbox({
          text("功能列表 - " + selected_group_name_locked()) | bold | center,
          separatorLight(),
          render_feature_rows_locked() | yframe | vscroll_indicator | flex,
      });
    }

    Element panel = body | border;
    if (pane_focus_ == 1) {
      panel = panel | color(Color::Cyan);
    }
    return panel;
  }

  Element render_group_rows_locked() const {
    Elements rows;
    for (int index = 0; index < static_cast<int>(groups_.size()); ++index) {
      const GroupInfo& group = groups_[index];
      const bool selected = index == group_selection_;
      const bool focused_row = selected && pane_focus_ == 0;
      const bool any_running = group.running_count > 0;
      const std::string marker = any_running ? "● " : "○ ";
      const std::string counter = " " + std::to_string(group.running_count) +
                                  "/" +
                                  std::to_string(group.feature_indices.size()) +
                                  " ";

      Element row = hbox({
          text(marker),
          text(group.name) | flex,
          text(counter),
      });

      if (focused_row) {
        row = row | bgcolor(Color::Blue) | color(Color::White) | bold | focus;
      } else if (selected) {
        row = row | bgcolor(Color::RGB(35, 35, 35)) | color(Color::White) |
              focus;
      } else if (any_running) {
        row = row | color(Color::Green);
      } else {
        row = row | color(Color::GrayLight);
      }
      rows.push_back(row);
    }
    return vbox(rows);
  }

  Element render_feature_rows_locked() const {
    Elements rows;
    for (int visible_index = 0;
         visible_index < static_cast<int>(visible_feature_indices_.size());
         ++visible_index) {
      const FeatureInfo* feature = feature_for_visible_index_locked(visible_index);
      if (feature == nullptr) {
        continue;
      }

      const bool selected = visible_index == feature_selection_;
      const bool focused_row = selected && pane_focus_ == 1;
      const std::string marker = feature->running ? "● " : "○ ";
      const std::string badge = feature->running ? " ON " : " OFF ";

      Element badge_element =
          text(badge) | bgcolor(feature->running ? Color::Green : Color::Red) |
          color(Color::Black) | bold;
      Element row = hbox({
          text(marker) | color(feature->running ? Color::Green : Color::Red),
          text(feature->name) | flex,
          badge_element,
      });

      if (focused_row) {
        row = row | bgcolor(Color::Blue) | color(Color::White) | bold | focus;
      } else if (selected) {
        row = row | bgcolor(Color::RGB(35, 35, 35)) | color(Color::White) |
              focus;
      } else {
        row = row | color(Color::White);
      }
      rows.push_back(row);
    }
    return vbox(rows);
  }

  Element render_detail_panel() const {
    const FeatureInfo* feature = selected_feature_locked();
    if (feature == nullptr) {
      return window(text("当前功能详情") | bold,
                    vbox({text("没有选中的功能") | dim}));
    }

    Elements preview_rows;
    if (feature->preview_units.empty()) {
      preview_rows.push_back(text("- (无)") | dim);
    } else {
      for (size_t index = 0; index < feature->preview_units.size(); ++index) {
        preview_rows.push_back(text("- " + feature->preview_units[index]) |
                               color(Color::Yellow));
        if (index < feature->preview_commands.size()) {
          preview_rows.push_back(paragraph("  " + feature->preview_commands[index]) |
                                 color(Color::GrayLight));
        }
      }
    }

    Element running =
        text(feature->running ? "运行中" : "未运行") |
        color(feature->running ? Color::Green : Color::Red) | bold;

    return window(
        text("当前功能详情") | bold,
        vbox({
            hbox({text("名称: ") | bold, text(feature->name)}),
            hbox({text("分组: ") | bold, text(feature->group)}),
            hbox({text("状态: ") | bold, running, text("    自动启动: ") | bold,
                  text(feature->auto_start ? "是" : "否")}),
            hbox({text("停止超时: ") | bold,
                  text(format_float(feature->stop_timeout_sec) + "s"),
                  text("    依赖功能: ") | bold,
                  text(feature->depends_on.empty()
                           ? "(无)"
                           : join_strings(feature->depends_on, ", "))}),
            hbox({text("描述: ") | bold,
                  paragraph(feature->description.empty() ? "-" : feature->description) |
                      flex}),
            separatorLight(),
            text("启动预览") | bold | color(Color::Cyan),
            vbox(preview_rows) | yframe | vscroll_indicator | flex,
        }));
  }

  int estimate_wrapped_lines(const std::string& text_value, int width) const {
    const int safe_width = std::max(12, width);
    return std::max<int>(
        1, static_cast<int>((text_value.size() + safe_width - 1) / safe_width));
  }

  int estimate_detail_height_locked(int main_width) const {
    const FeatureInfo* feature = selected_feature_locked();
    if (feature == nullptr) {
      return 5;
    }

    const int detail_width = std::max(24, main_width - 8);
    int lines = 2;  // window border
    lines += 4;     // name/group/status/dependency rows
    lines += estimate_wrapped_lines(feature->description.empty()
                                        ? "-"
                                        : feature->description,
                                    detail_width - 8);
    lines += 2;  // separator + "启动预览"

    if (feature->preview_units.empty()) {
      lines += 1;
    } else {
      for (size_t index = 0; index < feature->preview_units.size(); ++index) {
        lines += 1;
        if (index < feature->preview_commands.size() &&
            !feature->preview_commands[index].empty()) {
          lines += estimate_wrapped_lines(feature->preview_commands[index],
                                          detail_width - 2);
        }
      }
    }

    return lines + 1;
  }

  Element render_system_panel() const {
    if (!system_info_.valid) {
      return window(text("系统信息") | bold,
                    vbox({text("等待 " + system_info_topic_ + " ...") |
                          color(Color::Yellow)}));
    }

    Elements node_rows;
    for (int index = 0;
         index < static_cast<int>(system_info_.active_ros_nodes.size());
         ++index) {
      node_rows.push_back(text("- " + system_info_.active_ros_nodes[index]) |
                          color(Color::GrayLight));
    }
    if (node_rows.empty()) {
      node_rows.push_back(text("(无活跃节点)") | dim);
    }

    const auto age_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - system_info_.received_at)
            .count();
    const float cpu_progress =
        std::max(0.0F, std::min(1.0F, system_info_.cpu_percent / 100.0F));
    const float mem_progress =
        std::max(0.0F, std::min(1.0F, system_info_.memory_percent / 100.0F));

    return window(
        text("系统信息") | bold,
        vbox({
            hbox({text("CPU    ") | bold,
                  text(format_float(system_info_.cpu_percent) + "%") |
                      align_right}),
            gauge(cpu_progress),
            hbox({text("内存   ") | bold,
                  text(format_float(system_info_.memory_percent) + "%") |
                      align_right}),
            gauge(mem_progress),
            separatorLight(),
            text("活跃 ROS 节点 (" +
                 std::to_string(system_info_.active_ros_nodes.size()) + ")") |
                bold | color(Color::Cyan),
            vbox(node_rows) | yframe | vscroll_indicator | flex,
            text("更新: " + std::to_string(age_ms / 1000.0) + "s 前") | dim,
        }));
  }

  void refresh_loop() {
    const auto period =
        std::chrono::milliseconds(static_cast<int>(refresh_period_sec_ * 1000.0));
    while (!stop_refresh_.load() && ros::ok()) {
      std::this_thread::sleep_for(period);
      if (stop_refresh_.load() || !ros::ok()) {
        break;
      }
      refresh_features("", false);
      post_refresh_event();
    }
  }

  void post_refresh_event() {
    ScreenInteractive* screen = screen_.load();
    if (screen != nullptr) {
      screen->PostEvent(Event::Custom);
    }
  }

  void on_system_info(const sunray_msgs::SystemInfo::ConstPtr& message) {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    system_info_.valid = true;
    system_info_.cpu_percent = message->cpu_percent;
    system_info_.memory_percent = message->memory_percent;
    system_info_.active_ros_nodes = message->active_ros_nodes;
    system_info_.received_at = std::chrono::steady_clock::now();
  }

  void refresh_features(const std::string& status_if_success, bool initializing) {
    std::lock_guard<std::mutex> service_lock(service_mutex_);

    if (!list_client_.waitForExistence(ros::Duration(initializing ? 0.8 : 0.05))) {
      std::lock_guard<std::recursive_mutex> state_lock(state_mutex_);
      status_message_ = "等待服务: " + system_ns_ + "/list_features";
      status_is_error_ = true;
      return;
    }

    sunray_msgs::ListFeatures list_srv;
    if (!list_client_.call(list_srv)) {
      std::lock_guard<std::recursive_mutex> state_lock(state_mutex_);
      status_message_ = "刷新失败: 无法调用 " + system_ns_ + "/list_features";
      status_is_error_ = true;
      return;
    }

    std::vector<FeatureInfo> refreshed_features;
    refreshed_features.reserve(list_srv.response.feature_names.size());

    for (const std::string& name : list_srv.response.feature_names) {
      sunray_msgs::GetFeatures get_srv;
      get_srv.request.feature_name = name;

      FeatureInfo feature;
      feature.name = name;
      if (get_client_.call(get_srv) && get_srv.response.success) {
        feature.name = get_srv.response.name.empty() ? name : get_srv.response.name;
        feature.group =
            get_srv.response.group.empty() ? "未分组" : get_srv.response.group;
        feature.running = get_srv.response.running;
        feature.auto_start = get_srv.response.auto_start;
        feature.stop_timeout_sec = get_srv.response.stop_timeout_sec;
        feature.description = get_srv.response.description;
        feature.message = get_srv.response.message;
        feature.depends_on = get_srv.response.depends_on;
        feature.preview_units = get_srv.response.start_preview_units;
        feature.preview_commands = get_srv.response.start_preview_commands;
      } else {
        feature.group = "查询失败";
        feature.description = "无法获取该功能详情";
        feature.message = "get_features failed";
      }
      refreshed_features.push_back(std::move(feature));
    }

    std::sort(refreshed_features.begin(), refreshed_features.end(),
              [](const FeatureInfo& lhs, const FeatureInfo& rhs) {
                if (lhs.group != rhs.group) {
                  return lhs.group < rhs.group;
                }
                return lhs.name < rhs.name;
              });

    std::lock_guard<std::recursive_mutex> state_lock(state_mutex_);
    const std::string old_group = selected_group_name_locked();
    const std::string old_feature =
        selected_feature_locked() == nullptr ? "" : selected_feature_locked()->name;

    features_ = std::move(refreshed_features);
    rebuild_groups_locked();
    restore_selection_locked(old_group, old_feature);

    last_refresh_wall_time_ = std::chrono::system_clock::now();
    if (!status_if_success.empty()) {
      status_message_ = status_if_success;
    } else {
      status_message_ = "刷新成功，共 " + std::to_string(features_.size()) +
                        " 个功能";
    }
    status_is_error_ = false;
  }

  void start_selected_feature() {
    const FeatureInfo* selected = nullptr;
    std::string feature_name;
    bool terminal_mode = false;
    {
      std::lock_guard<std::recursive_mutex> lock(state_mutex_);
      selected = selected_feature_locked();
      if (selected == nullptr) {
        status_message_ = "没有可启动的功能";
        status_is_error_ = true;
        return;
      }
      feature_name = selected->name;
      terminal_mode = start_with_terminal_;
      status_message_ = "正在启动: " + feature_name;
      status_is_error_ = false;
    }

    sunray_msgs::StartFeature start_srv;
    start_srv.request.feature_name = feature_name;
    start_srv.request.override_args.clear();
    start_srv.request.restart_if_running = false;
    start_srv.request.start_with_terminal = terminal_mode;

    std::string result_message;
    bool ok = false;
    {
      std::lock_guard<std::mutex> service_lock(service_mutex_);
      if (start_client_.call(start_srv)) {
        ok = start_srv.response.success;
        result_message = start_srv.response.message;
      } else {
        result_message = "无法调用 start_feature 服务";
      }
    }

    refresh_features(result_message.empty()
                         ? (ok ? "启动成功: " + feature_name
                               : "启动失败: " + feature_name)
                         : result_message,
                     false);
    {
      std::lock_guard<std::recursive_mutex> lock(state_mutex_);
      status_is_error_ = !ok;
    }
  }

  void stop_selected_feature() {
    const FeatureInfo* selected = nullptr;
    std::string feature_name;
    {
      std::lock_guard<std::recursive_mutex> lock(state_mutex_);
      selected = selected_feature_locked();
      if (selected == nullptr) {
        status_message_ = "没有可停止的功能";
        status_is_error_ = true;
        return;
      }
      feature_name = selected->name;
      status_message_ = "正在停止: " + feature_name;
      status_is_error_ = false;
    }

    sunray_msgs::StopFeature stop_srv;
    stop_srv.request.feature_name = feature_name;
    stop_srv.request.force = false;

    std::string result_message;
    bool ok = false;
    {
      std::lock_guard<std::mutex> service_lock(service_mutex_);
      if (stop_client_.call(stop_srv)) {
        ok = stop_srv.response.success;
        result_message = stop_srv.response.message;
      } else {
        result_message = "无法调用 stop_feature 服务";
      }
    }

    refresh_features(result_message.empty()
                         ? (ok ? "停止成功: " + feature_name
                               : "停止失败: " + feature_name)
                         : result_message,
                     false);
    {
      std::lock_guard<std::recursive_mutex> lock(state_mutex_);
      status_is_error_ = !ok;
    }
  }

  void toggle_terminal_mode() {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    start_with_terminal_ = !start_with_terminal_;
    status_message_ = std::string("启动模式: ") +
                      (start_with_terminal_ ? "终端启动" : "后台启动");
    status_is_error_ = false;
  }

  void move_selection(int delta) {
    if (delta == 0) {
      return;
    }

    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    if (pane_focus_ == 0) {
      if (groups_.empty()) {
        return;
      }
      const int old_selection = group_selection_;
      group_selection_ = std::max(
          0, std::min(group_selection_ + delta,
                      static_cast<int>(groups_.size()) - 1));
      if (group_selection_ != old_selection) {
        rebuild_visible_features_locked(true);
        status_message_ = "分组: " + selected_group_name_locked();
        status_is_error_ = false;
      }
      return;
    }

    if (visible_feature_indices_.empty()) {
      return;
    }
    feature_selection_ =
        std::max(0, std::min(feature_selection_ + delta,
                             static_cast<int>(visible_feature_indices_.size()) -
                                 1));
  }

  void move_selection_to_edge(bool first) {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    if (pane_focus_ == 0) {
      if (groups_.empty()) {
        return;
      }
      const int old_selection = group_selection_;
      group_selection_ = first ? 0 : static_cast<int>(groups_.size()) - 1;
      if (group_selection_ != old_selection) {
        rebuild_visible_features_locked(true);
        status_message_ = "分组: " + selected_group_name_locked();
        status_is_error_ = false;
      }
      return;
    }

    if (visible_feature_indices_.empty()) {
      return;
    }
    feature_selection_ =
        first ? 0 : static_cast<int>(visible_feature_indices_.size()) - 1;
  }

  void rebuild_groups_locked() {
    groups_.clear();
    group_entries_.clear();

    std::map<std::string, GroupInfo> grouped;
    for (int index = 0; index < static_cast<int>(features_.size()); ++index) {
      GroupInfo& group = grouped[features_[index].group];
      group.name = features_[index].group;
      group.feature_indices.push_back(index);
      if (features_[index].running) {
        group.running_count += 1;
      }
    }

    for (auto& item : grouped) {
      groups_.push_back(std::move(item.second));
    }

    for (const GroupInfo& group : groups_) {
      group_entries_.push_back(group.name);
    }

    clamp_group_selection_locked();
    rebuild_visible_features_locked(false);
  }

  void restore_selection_locked(const std::string& old_group,
                                const std::string& old_feature) {
    if (!old_group.empty()) {
      for (int index = 0; index < static_cast<int>(groups_.size()); ++index) {
        if (groups_[index].name == old_group) {
          group_selection_ = index;
          break;
        }
      }
    }
    clamp_group_selection_locked();
    rebuild_visible_features_locked(false);

    if (!old_feature.empty()) {
      for (int index = 0; index < static_cast<int>(visible_feature_indices_.size());
           ++index) {
        const int feature_index = visible_feature_indices_[index];
        if (feature_index >= 0 &&
            feature_index < static_cast<int>(features_.size()) &&
            features_[feature_index].name == old_feature) {
          feature_selection_ = index;
          break;
        }
      }
    }
    clamp_feature_selection_locked();
  }

  void rebuild_visible_features_locked(bool reset_selection) {
    feature_entries_.clear();
    visible_feature_indices_.clear();
    if (groups_.empty()) {
      feature_selection_ = 0;
      return;
    }

    clamp_group_selection_locked();
    for (int feature_index : groups_[group_selection_].feature_indices) {
      if (feature_index >= 0 && feature_index < static_cast<int>(features_.size())) {
        visible_feature_indices_.push_back(feature_index);
        feature_entries_.push_back(features_[feature_index].name);
      }
    }

    if (reset_selection) {
      feature_selection_ = 0;
    }
    clamp_feature_selection_locked();
  }

  void clamp_group_selection_locked() {
    if (groups_.empty()) {
      group_selection_ = 0;
      return;
    }
    group_selection_ =
        std::max(0, std::min(group_selection_, static_cast<int>(groups_.size()) - 1));
  }

  void clamp_feature_selection_locked() {
    if (visible_feature_indices_.empty()) {
      feature_selection_ = 0;
      return;
    }
    feature_selection_ = std::max(
        0, std::min(feature_selection_,
                    static_cast<int>(visible_feature_indices_.size()) - 1));
  }

  std::string selected_group_name_locked() const {
    if (groups_.empty() || group_selection_ < 0 ||
        group_selection_ >= static_cast<int>(groups_.size())) {
      return "-";
    }
    return groups_[group_selection_].name;
  }

  const FeatureInfo* feature_for_visible_index_locked(int visible_index) const {
    if (visible_index < 0 ||
        visible_index >= static_cast<int>(visible_feature_indices_.size())) {
      return nullptr;
    }
    const int feature_index = visible_feature_indices_[visible_index];
    if (feature_index < 0 || feature_index >= static_cast<int>(features_.size())) {
      return nullptr;
    }
    return &features_[feature_index];
  }

  const FeatureInfo* selected_feature_locked() const {
    return feature_for_visible_index_locked(feature_selection_);
  }

  ros::NodeHandle private_nh_;
  std::string system_ns_;
  std::string system_info_topic_;
  double refresh_period_sec_ = 5.0;

  ros::ServiceClient list_client_;
  ros::ServiceClient get_client_;
  ros::ServiceClient start_client_;
  ros::ServiceClient stop_client_;
  ros::Subscriber system_info_sub_;

  mutable std::recursive_mutex state_mutex_;
  std::mutex service_mutex_;
  std::atomic<bool> stop_refresh_{false};
  std::thread refresh_thread_;
  std::atomic<ScreenInteractive*> screen_{nullptr};

  std::vector<FeatureInfo> features_;
  std::vector<GroupInfo> groups_;
  std::vector<int> visible_feature_indices_;
  std::vector<std::string> group_entries_;
  std::vector<std::string> feature_entries_;

  int pane_focus_ = 0;
  int group_selection_ = 0;
  int feature_selection_ = 0;
  bool start_with_terminal_ = true;
  bool status_is_error_ = false;
  std::string status_message_ = "正在初始化...";
  std::chrono::system_clock::time_point last_refresh_wall_time_;
  SystemInfoSnapshot system_info_;

};

}  // namespace

int main(int argc, char** argv) {
  std::setlocale(LC_ALL, "");
  ros::init(argc, argv, "scripts_manage_tui");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::NodeHandle private_nh("~");
  ScriptsManageTui app(private_nh);
  return app.run();
}
