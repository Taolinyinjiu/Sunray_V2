#include "tui_render.hpp"
#include "ftxui/component/component.hpp"
#include "ftxui/component/component_options.hpp"
#include "ftxui/component/animation.hpp"
#include "ftxui/component/event.hpp"
#include "ftxui/dom/elements.hpp"
#include "tui_terminal.hpp"
#include <filesystem>
#include <thread>
#include <cstdlib>
#include <sys/wait.h>

#ifdef __APPLE__
#include <mach-o/dyld.h>
#endif

using namespace ftxui;

namespace sunray_tui {

// 按钮组件创建逻辑（从 create_component 中提取）
void UIRenderer::create_buttons() {
  // ========== 组件化底部按钮 ==========
  // 主操作按钮：构建模式为开始编译构建，检查模式为检查依赖
  auto start_opt = ButtonOption::Animated();
  // 自定义外观：与清除按钮统一hover逻辑，错误提示改为点击触发
  start_opt.transform = [this](const EntryState &s) {
    const bool has_selection = !state_.view.selected_modules.empty();
    // 第一阶段：通过统一高亮管理器决定唯一高亮
    const bool hover_like = highlight_mgr_.is_highlighted(InteractiveId::Start());
    const int inner_width = 18;

    // 点击未选择模块后，短暂显示“请选择模块”并以红底提示
    const bool warn = state_.build_warning_flash_active;

    std::string raw = warn ? "请选择模块" : std::string(s.label);
    if ((int)raw.size() > inner_width)
      raw = raw.substr(0, inner_width);
    Element inner = text(raw) | center | size(WIDTH, EQUAL, inner_width);
    Element full = hbox({text("["), inner, text("]")});

    if (warn) {
      full = full | bold | bgcolor(Color::Red) | color(Color::White);
    } else if (hover_like) {
      // 与清除按钮一致的hover效果
      full = full | bold | bgcolor(Color::RGB(60, 60, 60)) | color(Color::White);
    } else if (has_selection) {
      full = full | bold | color(Color::Blue);
    } else {
      // 无选择时常态为置灰
      full = full | bold | color(Color::GrayDark) | dim;
    }

    return full;
  };
  start_button_ = Button(
      state_.app_mode == AppMode::Check ? "检查依赖" : "开始编译构建",
      [this] {
        // 仅当选择了模块才触发
        if (!state_.view.selected_modules.empty()) {
          state_.handle_build_button();
        } else {
          state_.trigger_build_warning_flash();
          animation::RequestAnimationFrame();
        }
        // 点击后重置按钮 hover 与键盘焦点，避免灰底滞留
        start_button_hovered_ = false;
        resolve_button_hovered_ = false;
        clear_button_hovered_ = false;
        state_.build_button_focused = false;
      },
      start_opt);
  // 去除 Hoverable，hover 高亮由统一管理器 + 反射 Box 决定

  if (state_.app_mode == AppMode::Check) {
    auto resolve_opt = ButtonOption::Animated();
    resolve_opt.transform = [this](const EntryState &s) {
      const bool has_selection = !state_.view.selected_modules.empty();
      const bool hover_like =
          highlight_mgr_.is_highlighted(InteractiveId::Resolve());
      const int inner_width = 18;
      const bool warn = state_.build_warning_flash_active;

      std::string raw = warn ? "请选择模块" : std::string(s.label);
      if ((int)raw.size() > inner_width)
        raw = raw.substr(0, inner_width);
      Element inner = text(raw) | center | size(WIDTH, EQUAL, inner_width);
      Element full = hbox({text("["), inner, text("]")});

      if (warn) {
        full = full | bold | bgcolor(Color::Red) | color(Color::White);
      } else if (hover_like) {
        full =
            full | bold | bgcolor(Color::RGB(60, 60, 60)) | color(Color::White);
      } else if (has_selection) {
        full = full | bold | color(Color::Green);
      } else {
        full = full | bold | color(Color::GrayDark) | dim;
      }

      return full;
    };

    resolve_button_ = Button(
        "解决依赖",
        [this] {
          if (!state_.view.selected_modules.empty()) {
            state_.handle_resolve_button();
          } else {
            state_.trigger_build_warning_flash();
            animation::RequestAnimationFrame();
          }
          start_button_hovered_ = false;
          resolve_button_hovered_ = false;
          clear_button_hovered_ = false;
          state_.build_button_focused = false;
        },
        resolve_opt);
  }

  // 清除输出按钮：仅清空检查面板，不触碰构建系统。
  auto clear_opt = ButtonOption::Animated();
  clear_opt.transform = [this](const EntryState &s) {
    // 第一阶段：通过统一高亮管理器决定唯一高亮
    const bool hover_like = highlight_mgr_.is_highlighted(InteractiveId::Clear());
    const int inner_width = 18;

    std::string raw;
    switch (clear_state_) {
    case CleanState::Idle:
      raw = std::string(s.label);
      break;
    case CleanState::Running:
      raw = "清除输出..";
      break;
    case CleanState::Success:
      raw = "已清除";
      break;
    case CleanState::Error:
      raw = "失败";
      break;
    }
    if ((int)raw.size() > inner_width)
      raw = raw.substr(0, inner_width);
    Element inner = text(raw) | center | size(WIDTH, EQUAL, inner_width);
    Element full = hbox({text("["), inner, text("]")});

    if (clear_state_ == CleanState::Running) {
      full =
          full | bold | bgcolor(Color::RGB(60, 60, 60)) | color(Color::White);
    } else if (clear_state_ == CleanState::Success) {
      if (hover_like) {
        // 成功状态hover：保持绿色但添加背景
        full = full | bold | bgcolor(Color::RGB(40, 80, 40)) | color(Color::White);
      } else {
        full = full | bold | color(Color::Green);
      }
    } else if (clear_state_ == CleanState::Error) {
      if (hover_like) {
        // 错误状态hover：保持红色但添加背景
        full = full | bold | bgcolor(Color::RGB(80, 40, 40)) | color(Color::White);
      } else {
        full = full | bold | color(Color::Red);
      }
    } else {
      // 空闲状态 - hover 时灰底白字，默认不着色（使用终端默认前景色）
      if (hover_like) {
        full = full | bold | bgcolor(Color::RGB(60, 60, 60)) | color(Color::White);
      } else {
        full = full | bold;
      }
    }
    return full;
  };

  clear_button_ = Button(
      "清除输出",
      [this] {
        state_.check_log_lines.clear();
        state_.check_task_title = "执行过程";
        state_.check_task_running = false;
        state_.check_task_exit_code = 0;
        clear_state_ = CleanState::Success;
        clear_success_frames_remaining_ = 30;
        start_button_hovered_ = false;
        resolve_button_hovered_ = false;
        clear_button_hovered_ = false;
        state_.build_button_focused = false;
        animation::RequestAnimationFrame();
      },
      clear_opt);
  // 去除 Hoverable，hover 高亮由统一管理器 + 反射 Box 决定

  if (state_.app_mode == AppMode::Check) {
    buttons_row_ =
        Container::Horizontal({start_button_, resolve_button_, clear_button_});
  } else {
    buttons_row_ = Container::Horizontal({start_button_, clear_button_});
  }
}

// 触发"清除输出"按钮对应的动作（与鼠标点击一致）
void UIRenderer::trigger_clear_build_clean() {
  // 这个功能已经在clear_button_的回调中实现
  // 这里提供一个编程接口来触发相同的动作
  if (clear_button_ && clear_state_ != CleanState::Running) {
    // 触发清除按钮的点击事件
    clear_button_->OnEvent(Event::Return);
  }
}

} // namespace sunray_tui
