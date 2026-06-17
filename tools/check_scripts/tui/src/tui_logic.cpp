#include "tui_logic.hpp"
#include "input_cleaner.hpp"
#include "tui_reset.hpp"
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <array>
#include <stdexcept>
#include <sys/wait.h>
#include <termios.h>
#include <unistd.h>


#ifdef __APPLE__
#include <mach-o/dyld.h>
#endif

namespace sunray_tui {

UILogic::UILogic(UIState &state) : state_(state), renderer_(state) {}

int UILogic::run() {
  try {
    return renderer_.run_with_action_callbacks(
        [this]() { this->execute_primary_action(); },
        [this]() { this->execute_resolve(); });
  } catch (const std::runtime_error &e) {
    if (std::string(e.what()) == "User requested exit")
      return 0;
    std::cerr << "UI错误: " << e.what() << std::endl;
    return 1;
  } catch (const std::exception &e) {
    std::cerr << "UI错误: " << e.what() << std::endl;
    return 1;
  }
}

std::vector<std::string> UILogic::get_selected_modules() const {
  if (state_.interaction_manager) {
    return state_.interaction_manager->get_explicitly_selected_modules();
  }
  return {state_.view.selected_modules.begin(),
          state_.view.selected_modules.end()};
}

std::string
UILogic::format_cli_arguments(const std::vector<std::string> &modules) const {
  std::string args;
  for (size_t i = 0; i < modules.size(); ++i) {
    if (i > 0)
      args += " ";
    args += shell_quote(modules[i]);
  }
  return args;
}

std::string UILogic::shell_quote(const std::string &value) const {
  std::string quoted = "'";
  for (char ch : value) {
    if (ch == '\'') {
      quoted += "'\\''";
    } else {
      quoted += ch;
    }
  }
  quoted += "'";
  return quoted;
}

void UILogic::append_check_log_line(const std::string &line) {
  constexpr size_t max_lines = 240;
  state_.check_log_lines.push_back(line);
  if (state_.check_log_lines.size() > max_lines) {
    state_.check_log_lines.erase(
        state_.check_log_lines.begin(),
        state_.check_log_lines.begin() +
            static_cast<long>(state_.check_log_lines.size() - max_lines));
  }
}

int UILogic::run_command_in_check_panel(const std::string &title,
                                        const std::string &command) {
  state_.check_task_title = title;
  state_.check_task_running = true;
  state_.check_task_exit_code = 0;
  state_.check_log_lines.clear();
  append_check_log_line("$ " + command);

  std::array<char, 512> buffer{};
  FILE *pipe = popen((command + " 2>&1").c_str(), "r");
  if (!pipe) {
    state_.check_task_running = false;
    state_.check_task_exit_code = 1;
    append_check_log_line("无法启动命令");
    return 1;
  }

  std::string pending;
  while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe)) {
    pending += buffer.data();
    size_t pos = 0;
    while ((pos = pending.find('\n')) != std::string::npos) {
      std::string line = pending.substr(0, pos);
      if (!line.empty() && line.back() == '\r') {
        line.pop_back();
      }
      append_check_log_line(line);
      pending.erase(0, pos + 1);
    }
  }
  if (!pending.empty()) {
    if (pending.back() == '\r') {
      pending.pop_back();
    }
    append_check_log_line(pending);
  }

  int status = pclose(pipe);
  int exit_code = 1;
  if (status != -1) {
    if (WIFEXITED(status)) {
      exit_code = WEXITSTATUS(status);
    } else if (WIFSIGNALED(status)) {
      exit_code = 128 + WTERMSIG(status);
    }
  }

  state_.check_task_running = false;
  state_.check_task_exit_code = exit_code;
  append_check_log_line(exit_code == 0 ? "执行完成" :
                                      "执行失败，退出码: " +
                                          std::to_string(exit_code));
  if (exit_code != 0) {
    append_check_log_line(
        "如果解决脚本需要交互式 sudo，请在终端运行 ./check.sh --resolve <module>");
  }
  return exit_code;
}

void UILogic::display_transition_message(const std::string &message) const {
  std::cout << "\n" << message << "\n" << std::flush;
}

std::string UILogic::get_project_root_dir() const {
  try {
    std::filesystem::path exe_path;
#ifdef __APPLE__
    char exe_path_buf[PATH_MAX];
    uint32_t size = sizeof(exe_path_buf);
    if (_NSGetExecutablePath(exe_path_buf, &size) == 0) {
      exe_path = std::filesystem::canonical(exe_path_buf);
    } else {
      throw std::runtime_error("无法获取可执行文件路径");
    }
#else
    exe_path = std::filesystem::canonical("/proc/self/exe");
#endif
    std::filesystem::path project_root =
        exe_path.parent_path().parent_path().parent_path();
    if (std::filesystem::exists(project_root / "check.sh"))
      return project_root.string();

    for (const auto &root_path : {"../../../", "../../", "../", "./"}) {
      std::filesystem::path test_path = std::filesystem::canonical(root_path);
      if (std::filesystem::exists(test_path / "check.sh"))
        return test_path.string();
    }
  } catch (const std::exception &e) {
    std::cerr << "路径解析异常: " << e.what() << std::endl;
  }
  return std::filesystem::current_path().string();
}

void UILogic::execute_build() {
  execute_check();
}

void UILogic::execute_check() {
  auto selected_modules = get_selected_modules();
  if (selected_modules.empty()) {
    state_.check_task_title = "检查依赖";
    state_.check_task_running = false;
    state_.check_task_exit_code = 1;
    state_.check_log_lines.clear();
    append_check_log_line("错误: 没有选择任何模块进行依赖检查");
    return;
  }

  save_current_selection();

  std::string project_root = get_project_root_dir();
  std::string check_script = project_root + "/check.sh";
  if (!std::filesystem::exists(check_script)) {
    state_.check_task_title = "检查依赖";
    state_.check_task_running = false;
    state_.check_task_exit_code = 1;
    state_.check_log_lines.clear();
    append_check_log_line("错误: 找不到依赖检查脚本 " + check_script);
    append_check_log_line("项目根目录: " + project_root);
    return;
  }

  std::string full_cmd =
      shell_quote(check_script) + " --from-tui " +
      format_cli_arguments(selected_modules);
  run_command_in_check_panel("检查依赖", full_cmd);
}

void UILogic::execute_resolve() {
  auto selected_modules = get_selected_modules();
  if (selected_modules.empty()) {
    state_.check_task_title = "解决依赖";
    state_.check_task_running = false;
    state_.check_task_exit_code = 1;
    state_.check_log_lines.clear();
    append_check_log_line("错误: 没有选择任何模块进行依赖解决");
    return;
  }

  save_current_selection();

  std::string project_root = get_project_root_dir();
  std::string installer_script = project_root + "/install_sunray_dependency.sh";
  if (!std::filesystem::exists(installer_script)) {
    state_.check_task_title = "解决依赖";
    state_.check_task_running = false;
    state_.check_task_exit_code = 1;
    state_.check_log_lines.clear();
    append_check_log_line("错误: 找不到依赖安装脚本 " + installer_script);
    append_check_log_line("项目根目录: " + project_root);
    return;
  }

  std::string full_cmd = shell_quote(installer_script);
  for (const auto &module : selected_modules) {
    full_cmd += " --module " + shell_quote(module);
  }

  state_.check_task_title = "解决依赖";
  state_.check_task_running = false;
  state_.check_task_exit_code = 0;
  state_.check_log_lines.clear();
  append_check_log_line("依赖安装可能需要交互式 sudo，TUI 不直接执行安装脚本。");
  append_check_log_line("请退出 TUI 后在终端执行:");
  append_check_log_line(full_cmd);
}

void UILogic::execute_primary_action() {
  if (state_.app_mode == AppMode::Check) {
    execute_check();
    return;
  }
  execute_build();
}

void UILogic::save_current_selection() const {
  auto selected_modules = get_selected_modules();
  if (selected_modules.empty()) {
    return;
  }

  try {
    std::string project_root = get_project_root_dir();
    std::string file_path = project_root + "/" + LAST_SELECTION_FILE;

    // 使用临时文件确保原子性写入
    std::string temp_file_path = file_path + ".tmp";
    std::ofstream temp_file(temp_file_path);
    if (!temp_file.is_open()) {
      std::cerr << "警告: 无法创建临时文件保存选择: " << temp_file_path
                << std::endl;
      return;
    }

    // 写入文件头和选择的模块
    temp_file << "# Sunray构建系统 - 上次模块选择\n";
    temp_file << "# 生成时间: " << std::time(nullptr) << "\n";
    temp_file << "# 格式: 每行一个模块名\n\n";

    for (const auto &module : selected_modules) {
      temp_file << module << "\n";
    }

    temp_file.close();

    // 原子性移动文件
    std::filesystem::rename(temp_file_path, file_path);

  } catch (const std::exception &e) {
    std::cerr << "警告: 保存模块选择时出错: " << e.what() << std::endl;
  }
}

void UILogic::load_last_selection() {
  try {
    std::string project_root = get_project_root_dir();
    std::string file_path = project_root + "/" + LAST_SELECTION_FILE;

    std::ifstream file(file_path);
    if (!file.is_open()) {
      std::cout << "提示: 未找到上次选择记录，使用默认选择\n";
      return;
    }

    std::vector<std::string> loaded_modules;
    std::string line;
    while (std::getline(file, line)) {
      // 跳过注释行和空行
      if (line.empty() || line[0] == '#') {
        continue;
      }

      // 清理模块名并添加
      line.erase(0, line.find_first_not_of(" \t"));
      line.erase(line.find_last_not_of(" \t") + 1);
      if (!line.empty()) {
        loaded_modules.push_back(line);
      }
    }

    if (loaded_modules.empty()) {
      std::cout << "提示: 上次选择记录为空，使用默认选择\n";
      return;
    }

    state_.view.selected_modules.clear();
    state_.view.selected_groups.clear();
    state_.view.active_group.clear();
    if (state_.interaction_manager) {
      state_.interaction_manager->set_explicit_selection(loaded_modules);
      for (const auto &module : state_.interaction_manager->get_selected_modules()) {
        state_.view.selected_modules.insert(module);
      }
    } else {
      for (const auto &module : loaded_modules) {
        state_.view.selected_modules.insert(module);
      }
    }

    std::cout << "已加载上次选择的 " << loaded_modules.size()
              << " 个模块，按空格开始构建\n";

  } catch (const std::exception &e) {
    std::cerr << "警告: 加载上次选择时出错: " << e.what() << std::endl;
  }
}

} // namespace sunray_tui
