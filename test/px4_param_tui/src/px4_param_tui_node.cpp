#include <ftxui/component/component.hpp>
#include <ftxui/component/component_options.hpp>
#include <ftxui/component/event.hpp>
#include <ftxui/component/screen_interactive.hpp>
#include <ftxui/dom/elements.hpp>
#include <ftxui/screen/terminal.hpp>

#include <px4_param_manager/px4_param_decode.h>
#include <px4_param_manager/px4_param_manager.h>
#include <px4_param_manager/px4_param_types.h>
#include <ros/ros.h>
#include <ros/service.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <ctime>
#include <deque>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <streambuf>
#include <string>
#include <thread>
#include <vector>

namespace {

using ftxui::Button;
using ftxui::CatchEvent;
using ftxui::Checkbox;
using ftxui::Color;
using ftxui::Component;
using ftxui::Element;
using ftxui::Event;
using ftxui::Input;
using ftxui::Menu;
using ftxui::Radiobox;
using ftxui::Renderer;
using ftxui::ScreenInteractive;
using ftxui::bold;
using ftxui::border;
using ftxui::bgcolor;
using ftxui::center;
using ftxui::color;
using ftxui::dim;
using ftxui::filler;
using ftxui::flex;
using ftxui::frame;
using ftxui::hbox;
using ftxui::paragraph;
using ftxui::separator;
using ftxui::size;
using ftxui::text;
using ftxui::vbox;
using ftxui::vscroll_indicator;

enum class ParamKind { kEnum, kBitmask, kReal };

struct ParamSpec {
  std::string name;
  ParamKind kind;
  std::vector<std::string> options;
  std::vector<int> option_values;
  std::string hint;
};

struct ParamWriteState {
  int enum_choice = 0;
  std::vector<std::shared_ptr<bool>> bit_choices;
  std::string real_input = "0.0";
};

std::string Trim(const std::string &s) {
  const auto begin = s.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = s.find_last_not_of(" \t\r\n");
  return s.substr(begin, end - begin + 1);
}

std::string ToLower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

std::string Join(const std::vector<std::string> &items, const std::string &sep) {
  if (items.empty()) {
    return "";
  }
  std::ostringstream oss;
  for (size_t i = 0; i < items.size(); ++i) {
    if (i > 0) {
      oss << sep;
    }
    oss << items[i];
  }
  return oss.str();
}

std::string NowTimestamp() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t t = std::chrono::system_clock::to_time_t(now);
  std::tm tm {};
  localtime_r(&t, &tm);
  std::ostringstream oss;
  oss << std::put_time(&tm, "%H:%M:%S");
  return oss.str();
}

class LogStore {
public:
  explicit LogStore(size_t max_lines) : max_lines_(max_lines) {}

  void Push(const std::string &line) {
    if (line.empty()) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    lines_.push_back(line);
    while (lines_.size() > max_lines_) {
      lines_.pop_front();
    }
  }

  void Clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    lines_.clear();
  }

  std::vector<std::string> Snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return {lines_.begin(), lines_.end()};
  }

private:
  size_t max_lines_ = 600;
  mutable std::mutex mutex_;
  std::deque<std::string> lines_;
};

class LineTeeBuffer : public std::streambuf {
public:
  using Callback = std::function<void(const std::string &)>;

  LineTeeBuffer(std::streambuf *downstream, Callback callback,
                bool forward_to_downstream)
      : downstream_(downstream), callback_(std::move(callback)),
        forward_to_downstream_(forward_to_downstream) {}

protected:
  int overflow(int ch) override {
    if (ch == traits_type::eof()) {
      return traits_type::not_eof(ch);
    }

    if (forward_to_downstream_ && downstream_) {
      downstream_->sputc(static_cast<char>(ch));
    }

    if (ch == '\n') {
      FlushLine();
    } else {
      line_buffer_.push_back(static_cast<char>(ch));
    }

    return ch;
  }

  int sync() override {
    if (forward_to_downstream_ && downstream_) {
      downstream_->pubsync();
    }
    FlushLine();
    return 0;
  }

private:
  void FlushLine() {
    if (!line_buffer_.empty() && callback_) {
      callback_(line_buffer_);
      line_buffer_.clear();
    }
  }

  std::streambuf *downstream_ = nullptr;
  Callback callback_;
  bool forward_to_downstream_ = false;
  std::string line_buffer_;
};

class ScopedStreamRedirect {
public:
  ScopedStreamRedirect(std::ostream &stream, LineTeeBuffer::Callback callback,
                       bool forward_to_downstream = false)
      : stream_(stream),
        original_(stream.rdbuf()),
        tee_buffer_(original_, std::move(callback), forward_to_downstream) {
    stream_.rdbuf(&tee_buffer_);
  }

  ~ScopedStreamRedirect() { stream_.rdbuf(original_); }

private:
  std::ostream &stream_;
  std::streambuf *original_ = nullptr;
  LineTeeBuffer tee_buffer_;
};

bool ParseDoubleStrict(const std::string &input, double *value_out) {
  if (value_out == nullptr) {
    return false;
  }
  const std::string trimmed = Trim(input);
  if (trimmed.empty()) {
    return false;
  }
  size_t consumed = 0;
  try {
    const double value = std::stod(trimmed, &consumed);
    if (consumed != trimmed.size()) {
      return false;
    }
    *value_out = value;
    return true;
  } catch (...) {
    return false;
  }
}

std::optional<int> ParseEnumRawValue(const ParamSpec &spec,
                                     const ParamWriteState &state) {
  if (state.enum_choice < 0 ||
      state.enum_choice >= static_cast<int>(spec.option_values.size())) {
    return std::nullopt;
  }
  return spec.option_values[state.enum_choice];
}

uint32_t BuildBitmaskRaw(const ParamSpec &spec, const ParamWriteState &state) {
  uint32_t raw = 0u;
  const size_t n =
      std::min(spec.option_values.size(), state.bit_choices.size());
  for (size_t i = 0; i < n; ++i) {
    if (state.bit_choices[i] != nullptr && *state.bit_choices[i]) {
      raw |= (1u << spec.option_values[i]);
    }
  }
  return raw;
}

std::string FormatEnabledByBits(const ParamSpec &spec, uint32_t raw) {
  std::vector<std::string> enabled;
  for (size_t i = 0; i < spec.option_values.size(); ++i) {
    const int bit = spec.option_values[i];
    if (bit >= 0 && bit < 31 && ((raw & (1u << bit)) != 0u)) {
      enabled.push_back(spec.options[i]);
    }
  }
  if (enabled.empty()) {
    return "none";
  }
  return Join(enabled, ", ");
}

std::vector<ParamSpec> BuildSpecs() {
  return {
      {"EKF2_HGT_REF",
       ParamKind::kEnum,
       {"Baro (0)", "Gnss (1)", "Range (2)", "Vision (3)"},
       {0, 1, 2, 3},
       "Single choice: EKF2 height source."},
      {"EKF2_EV_CTRL",
       ParamKind::kBitmask,
       {"Horizontal position (bit0)", "Vertical position (bit1)",
        "Velocity (bit2)", "Yaw (bit3)"},
       {0, 1, 2, 3},
       "Bitmask: external vision fusion controls."},
      {"EKF2_EV_DELAY",
       ParamKind::kReal,
       {},
       {},
       "Real value in ms."},
      {"EKF2_MAG_TYPE",
       ParamKind::kEnum,
       {"Auto (0)", "Mag_Yaw (1)", "None (5)"},
       {0, 1, 5},
       "Single choice: magnetometer fusion type."},
      {"EKF2_MAG_CHECK",
       ParamKind::kBitmask,
       {"Strength (bit0)", "Inclination (bit1)", "Wait2WMM (bit2)"},
       {0, 1, 2},
       "Bitmask: magnetometer checks."},
      {"EKF2_GPS_CTRL",
       ParamKind::kBitmask,
       {"Lon/Lat (bit0)", "Altitude (bit1)", "Velocity3D (bit2)",
        "Dual antenna heading (bit3)"},
       {0, 1, 2, 3},
       "Bitmask: GPS fusion controls."},
      {"EKF2_GPS_CHECK",
       ParamKind::kBitmask,
       {"Min sat count (bit0)", "Max PDOP (bit1)",
        "Max horizontal pos error (bit2)", "Max vertical pos error (bit3)",
        "Max speed error (bit4)", "Max horizontal pos rate (bit5)",
        "Max vertical pos rate (bit6)", "Max horizontal speed (bit7)",
        "Max vertical velocity discrepancy (bit8)"},
       {0, 1, 2, 3, 4, 5, 6, 7, 8},
       "Bitmask: GPS quality checks."},
      {"EKF2_GPS_DELAY",
       ParamKind::kReal,
       {},
       {},
       "Real value in ms."},
  };
}

std::vector<ParamWriteState> BuildWriteStates(const std::vector<ParamSpec> &specs) {
  std::vector<ParamWriteState> states(specs.size());
  for (size_t i = 0; i < specs.size(); ++i) {
    if (specs[i].kind == ParamKind::kBitmask) {
      states[i].bit_choices.reserve(specs[i].options.size());
      for (size_t j = 0; j < specs[i].options.size(); ++j) {
        states[i].bit_choices.push_back(std::make_shared<bool>(false));
      }
    }
  }
  return states;
}

bool WriteParam(PX4_ParamManager &manager, const ParamSpec &spec,
                const ParamWriteState &state, std::string *detail_out,
                std::string *err_out) {
  auto fail = [&](const std::string &msg) {
    if (err_out != nullptr) {
      *err_out = msg;
    }
    return false;
  };

  if (spec.name == "EKF2_HGT_REF") {
    const auto raw = ParseEnumRawValue(spec, state);
    if (!raw.has_value()) {
      return fail("invalid enum selection");
    }
    px4_param_types::EKF2_HGT_REF p;
    switch (*raw) {
    case 0:
      p.enable_baro();
      break;
    case 1:
      p.enable_gnss();
      break;
    case 2:
      p.enable_range();
      break;
    case 3:
      p.enable_vision();
      break;
    default:
      return fail("unsupported enum raw value");
    }
    const bool ok = manager.set_param(p);
    if (detail_out != nullptr) {
      *detail_out = "raw=" + std::to_string(*raw);
    }
    return ok;
  }

  if (spec.name == "EKF2_EV_CTRL") {
    px4_param_types::EKF2_EV_CTRL p;
    const uint32_t raw = BuildBitmaskRaw(spec, state);
    if ((raw & (1u << 0)) != 0u) {
      p.enable_Horizontalposition();
    }
    if ((raw & (1u << 1)) != 0u) {
      p.enable_Verticalposition();
    }
    if ((raw & (1u << 2)) != 0u) {
      p.enable_Velocity();
    }
    if ((raw & (1u << 3)) != 0u) {
      p.enable_Yaw();
    }
    const bool ok = manager.set_param(p);
    if (detail_out != nullptr) {
      *detail_out = "raw=" + std::to_string(static_cast<int>(raw));
    }
    return ok;
  }

  if (spec.name == "EKF2_EV_DELAY") {
    double value = 0.0;
    if (!ParseDoubleStrict(state.real_input, &value)) {
      return fail("invalid numeric input");
    }
    px4_param_types::EKF2_EV_DELAY p;
    p.set_delay_ms(value);
    const bool ok = manager.set_param(p);
    if (detail_out != nullptr) {
      *detail_out = "value=" + std::to_string(value);
    }
    return ok;
  }

  if (spec.name == "EKF2_MAG_TYPE") {
    const auto raw = ParseEnumRawValue(spec, state);
    if (!raw.has_value()) {
      return fail("invalid enum selection");
    }
    px4_param_types::EKF2_MAG_TYPE p;
    switch (*raw) {
    case 0:
      p.set_Auto();
      break;
    case 1:
      p.set_Yaw();
      break;
    case 5:
      p.set_None();
      break;
    default:
      return fail("unsupported enum raw value");
    }
    const bool ok = manager.set_param(p);
    if (detail_out != nullptr) {
      *detail_out = "raw=" + std::to_string(*raw);
    }
    return ok;
  }

  if (spec.name == "EKF2_MAG_CHECK") {
    px4_param_types::EKF2_MAG_CHECK p;
    const uint32_t raw = BuildBitmaskRaw(spec, state);
    if ((raw & (1u << 0)) != 0u) {
      p.enable_Strength();
    }
    if ((raw & (1u << 1)) != 0u) {
      p.enable_Inclination();
    }
    if ((raw & (1u << 2)) != 0u) {
      p.enable_Wait2WMM();
    }
    const bool ok = manager.set_param(p);
    if (detail_out != nullptr) {
      *detail_out = "raw=" + std::to_string(static_cast<int>(raw));
    }
    return ok;
  }

  if (spec.name == "EKF2_GPS_CTRL") {
    px4_param_types::EKF2_GPS_CTRL p;
    const uint32_t raw = BuildBitmaskRaw(spec, state);
    if ((raw & (1u << 0)) != 0u) {
      p.enable_Lon_Lat();
    }
    if ((raw & (1u << 1)) != 0u) {
      p.enable_Altitude();
    }
    if ((raw & (1u << 2)) != 0u) {
      p.enable_Velocity3D();
    }
    if ((raw & (1u << 3)) != 0u) {
      p.enable_Dual_AntennaHeading();
    }
    const bool ok = manager.set_param(p);
    if (detail_out != nullptr) {
      *detail_out = "raw=" + std::to_string(static_cast<int>(raw));
    }
    return ok;
  }

  if (spec.name == "EKF2_GPS_CHECK") {
    px4_param_types::EKF2_GPS_CHECK p;
    const uint32_t raw = BuildBitmaskRaw(spec, state);
    if ((raw & (1u << 0)) != 0u) {
      p.enable_Min_sat_count();
    }
    if ((raw & (1u << 1)) != 0u) {
      p.enable_Max_PDOP();
    }
    if ((raw & (1u << 2)) != 0u) {
      p.enable_Max_HorizontalPosition_Error();
    }
    if ((raw & (1u << 3)) != 0u) {
      p.enable_Max_VerticalPosition_Error();
    }
    if ((raw & (1u << 4)) != 0u) {
      p.enable_Max_Speed_Error();
    }
    if ((raw & (1u << 5)) != 0u) {
      p.enable_Max_HorizontalPosition_Rate();
    }
    if ((raw & (1u << 6)) != 0u) {
      p.enable_Max_VerticalPosition_Rate();
    }
    if ((raw & (1u << 7)) != 0u) {
      p.enable_Max_HorizontalSpeed();
    }
    if ((raw & (1u << 8)) != 0u) {
      p.enable_Max_VerticalVelocity_Discrepancy();
    }
    const bool ok = manager.set_param(p);
    if (detail_out != nullptr) {
      *detail_out = "raw=" + std::to_string(static_cast<int>(raw));
    }
    return ok;
  }

  if (spec.name == "EKF2_GPS_DELAY") {
    double value = 0.0;
    if (!ParseDoubleStrict(state.real_input, &value)) {
      return fail("invalid numeric input");
    }
    px4_param_types::EKF2_GPS_DELAY p;
    p.set_delay_ms(value);
    const bool ok = manager.set_param(p);
    if (detail_out != nullptr) {
      *detail_out = "value=" + std::to_string(value);
    }
    return ok;
  }

  return fail("unsupported parameter");
}

bool ReadParam(PX4_ParamManager &manager, const ParamSpec &spec,
               std::string *raw_out, std::string *decoded_out,
               std::string *err_out) {
  auto fail = [&](const std::string &msg) {
    if (err_out != nullptr) {
      *err_out = msg;
    }
    return false;
  };

  if (spec.name == "EKF2_HGT_REF") {
    px4_param_decode::EKF2_HGT_REF p;
    if (!manager.read_param(&p)) {
      return fail("service call failed");
    }
    if (raw_out != nullptr) {
      *raw_out = std::to_string(p.raw_value);
    }
    if (decoded_out != nullptr) {
      if (p.is_baro()) {
        *decoded_out = "Baro";
      } else if (p.is_gnss()) {
        *decoded_out = "Gnss";
      } else if (p.is_range()) {
        *decoded_out = "Range";
      } else if (p.is_vision()) {
        *decoded_out = "Vision";
      } else {
        *decoded_out = "Unknown";
      }
    }
    return true;
  }

  if (spec.name == "EKF2_EV_CTRL") {
    px4_param_decode::EKF2_EV_CTRL p;
    if (!manager.read_param(&p)) {
      return fail("service call failed");
    }
    if (raw_out != nullptr) {
      *raw_out = std::to_string(static_cast<int>(p.value));
    }
    if (decoded_out != nullptr) {
      std::vector<std::string> enabled;
      if (p.enable_horizontal_position()) {
        enabled.push_back("Horizontal position");
      }
      if (p.enable_vertical_position()) {
        enabled.push_back("Vertical position");
      }
      if (p.enable_velocity()) {
        enabled.push_back("Velocity");
      }
      if (p.enable_yaw()) {
        enabled.push_back("Yaw");
      }
      *decoded_out = enabled.empty() ? "none" : Join(enabled, ", ");
    }
    return true;
  }

  if (spec.name == "EKF2_EV_DELAY") {
    px4_param_decode::EKF2_EV_DELAY p;
    if (!manager.read_param(&p)) {
      return fail("service call failed");
    }
    if (raw_out != nullptr) {
      *raw_out = std::to_string(p.value);
    }
    if (decoded_out != nullptr) {
      *decoded_out = std::to_string(p.value) + " ms";
    }
    return true;
  }

  if (spec.name == "EKF2_MAG_TYPE") {
    px4_param_decode::EKF2_MAG_TYPE p;
    if (!manager.read_param(&p)) {
      return fail("service call failed");
    }
    if (raw_out != nullptr) {
      *raw_out = std::to_string(p.raw_value);
    }
    if (decoded_out != nullptr) {
      if (p.is_auto()) {
        *decoded_out = "Auto";
      } else if (p.is_yaw()) {
        *decoded_out = "Mag_Yaw";
      } else if (p.is_none()) {
        *decoded_out = "None";
      } else {
        *decoded_out = "Unknown";
      }
    }
    return true;
  }

  if (spec.name == "EKF2_MAG_CHECK") {
    px4_param_decode::EKF2_MAG_CHECK p;
    if (!manager.read_param(&p)) {
      return fail("service call failed");
    }
    const uint32_t raw = p.value;
    if (raw_out != nullptr) {
      *raw_out = std::to_string(static_cast<int>(raw));
    }
    if (decoded_out != nullptr) {
      *decoded_out = FormatEnabledByBits(spec, raw);
    }
    return true;
  }

  if (spec.name == "EKF2_GPS_CTRL") {
    px4_param_decode::EKF2_GPS_CTRL p;
    if (!manager.read_param(&p)) {
      return fail("service call failed");
    }
    const uint32_t raw = p.value;
    if (raw_out != nullptr) {
      *raw_out = std::to_string(static_cast<int>(raw));
    }
    if (decoded_out != nullptr) {
      *decoded_out = FormatEnabledByBits(spec, raw);
    }
    return true;
  }

  if (spec.name == "EKF2_GPS_CHECK") {
    px4_param_decode::EKF2_GPS_CHECK p;
    if (!manager.read_param(&p)) {
      return fail("service call failed");
    }
    const uint32_t raw = p.value;
    if (raw_out != nullptr) {
      *raw_out = std::to_string(static_cast<int>(raw));
    }
    if (decoded_out != nullptr) {
      *decoded_out = FormatEnabledByBits(spec, raw);
    }
    return true;
  }

  if (spec.name == "EKF2_GPS_DELAY") {
    px4_param_decode::EKF2_GPS_DELAY p;
    if (!manager.read_param(&p)) {
      return fail("service call failed");
    }
    if (raw_out != nullptr) {
      *raw_out = std::to_string(p.value);
    }
    if (decoded_out != nullptr) {
      *decoded_out = std::to_string(p.value) + " ms";
    }
    return true;
  }

  return fail("unsupported parameter");
}

} // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "px4_param_tui_node");
  ros::NodeHandle nh;

  PX4_ParamManager param_manager(nh);

  int uav_id = 1;
  std::string uav_name = "uav";
  nh.param("uav_id", uav_id, uav_id);
  nh.param("uav_name", uav_name, uav_name);
  const std::string mavros_ns = "/" + uav_name + std::to_string(uav_id) + "/mavros";
  const std::string set_service = mavros_ns + "/param/set";
  const std::string get_service = mavros_ns + "/param/get";

  LogStore logs(600);
  auto log_event = [&](const std::string &msg) {
    const std::string line = "[" + NowTimestamp() + "] " + msg;
    logs.Push(line);
  };

  ScopedStreamRedirect redirect_clog(std::clog, [&](const std::string &line) {
    logs.Push("[" + NowTimestamp() + "] [STDLOG] " + line);
  },
                                     false);

  const std::vector<ParamSpec> specs = BuildSpecs();
  std::vector<std::string> param_names;
  param_names.reserve(specs.size());
  for (const auto &spec : specs) {
    param_names.push_back(spec.name);
  }
  std::vector<ParamWriteState> write_states = BuildWriteStates(specs);

  int write_param_index = 0;
  int read_param_index = 0;
  int write_focus_index = 0;
  int read_focus_index = 0;
  int write_button_flash_ticks = 0;
  int read_button_flash_ticks = 0;
  std::string read_status = "No read operation yet.";
  bool read_status_ok = true;
  std::string read_raw = "-";
  std::string read_decoded = "-";

  auto make_param_menu = [&](int *selected, int *focused, Color active_color) {
    ftxui::MenuOption option = ftxui::MenuOption::Vertical();
    option.entries = &param_names;
    option.selected = selected;
    option.focused_entry = focused;
    option.entries_option.animated_colors.background.enabled = false;
    option.entries_option.animated_colors.foreground.enabled = false;
    option.entries_option.transform = [active_color](const ftxui::EntryState &state) {
      const std::string prefix = state.active ? "● " : "  ";
      Element e = text(prefix + state.label);
      if (state.active) {
        return e | bold | color(active_color);
      }
      return e | dim;
    };
    return Menu(option);
  };

  auto write_menu =
      make_param_menu(&write_param_index, &write_focus_index, Color::GreenLight);
  auto read_menu =
      make_param_menu(&read_param_index, &read_focus_index, Color::CyanLight);

  ftxui::InputOption real_input_option = ftxui::InputOption::Default();
  real_input_option.transform = [](ftxui::InputState state) {
    // Keep the original background even when focused.
    // We only use text style hints and avoid inverted/bright background.
    state.element |= color(Color::White);
    if (state.is_placeholder) {
      state.element |= dim;
    }
    if (state.focused) {
      state.element |= bold;
    }
    return state.element;
  };

  std::vector<Component> write_editors;
  write_editors.reserve(specs.size());
  for (size_t i = 0; i < specs.size(); ++i) {
    const auto &spec = specs[i];
    if (spec.kind == ParamKind::kEnum) {
      write_editors.push_back(Radiobox(&spec.options, &write_states[i].enum_choice));
      continue;
    }

    if (spec.kind == ParamKind::kBitmask) {
      std::vector<Component> checkboxes;
      checkboxes.reserve(spec.options.size());
      for (size_t j = 0; j < spec.options.size(); ++j) {
        checkboxes.push_back(
            Checkbox(spec.options[j], write_states[i].bit_choices[j].get()));
      }
      write_editors.push_back(ftxui::Container::Vertical(std::move(checkboxes)));
      continue;
    }

    if (spec.kind == ParamKind::kReal) {
      write_editors.push_back(
          Input(&write_states[i].real_input, "numeric value", real_input_option));
      continue;
    }

    write_editors.push_back(Renderer([] { return text("Unsupported editor type."); }));
  }
  auto write_editor_tab = ftxui::Container::Tab(write_editors, &write_param_index);

  ftxui::ButtonOption write_button_option = ftxui::ButtonOption::Simple();
  write_button_option.label = "写入参数";
  write_button_option.transform = [&](const ftxui::EntryState &) {
    Element e = text(" 写入参数 ") | border;
    if (write_button_flash_ticks > 0) {
      return e | bold | color(Color::Black) | bgcolor(Color::GreenLight);
    }
    return e;
  };
  write_button_option.on_click = [&] {
    write_button_flash_ticks = 4;
    const ParamSpec &spec = specs.at(write_param_index);
    if (!ros::service::waitForService(set_service, ros::Duration(0.25))) {
      log_event("WRITE failed for " + spec.name + ": set service unavailable.");
      return;
    }

    std::string detail;
    std::string error;
    const bool ok =
        WriteParam(param_manager, spec, write_states[write_param_index], &detail, &error);
    if (ok) {
      log_event("WRITE OK | " + spec.name + " | " + detail);
      std::clog << "write " << spec.name << " success: " << detail << std::endl;
    } else {
      log_event("WRITE FAILED | " + spec.name + " | " + error);
      std::clog << "write " << spec.name << " failed: " << error << std::endl;
    }
  };
  auto write_button = Button(write_button_option);

  ftxui::ButtonOption read_button_option = ftxui::ButtonOption::Simple();
  read_button_option.label = "读取参数";
  read_button_option.transform = [&](const ftxui::EntryState &) {
    Element e = text(" 读取参数 ") | border;
    if (read_button_flash_ticks > 0) {
      return e | bold | color(Color::Black) | bgcolor(Color::CyanLight);
    }
    return e;
  };
  read_button_option.on_click = [&] {
    read_button_flash_ticks = 4;
    const ParamSpec &spec = specs.at(read_param_index);
    if (!ros::service::waitForService(get_service, ros::Duration(0.25))) {
      read_status = "Service unavailable: " + get_service;
      read_status_ok = false;
      log_event("READ failed for " + spec.name + ": get service unavailable.");
      return;
    }

    std::string raw;
    std::string decoded;
    std::string error;
    const bool ok = ReadParam(param_manager, spec, &raw, &decoded, &error);
    if (ok) {
      read_raw = raw;
      read_decoded = decoded;
      read_status = "OK | " + spec.name;
      read_status_ok = true;
      log_event("READ OK | " + spec.name + " | raw=" + raw + " | decoded=" + decoded);
      std::clog << "read " << spec.name << " => raw=" << raw
                << ", decoded=" << decoded << std::endl;
    } else {
      read_raw = "-";
      read_decoded = "-";
      read_status = "FAILED | " + spec.name + " | " + error;
      read_status_ok = false;
      log_event("READ FAILED | " + spec.name + " | " + error);
      std::clog << "read " << spec.name << " failed: " << error << std::endl;
    }
  };
  auto read_button = Button(read_button_option);

  auto clear_logs_button = Button("Clear Logs", [&] {
    logs.Clear();
    log_event("Log panel cleared.");
  });

  auto left_panel_container =
      ftxui::Container::Vertical({write_menu, write_editor_tab, write_button});
  auto right_panel_container = ftxui::Container::Vertical({read_menu, read_button});
  auto top_container =
      ftxui::Container::Horizontal({left_panel_container, right_panel_container});
  auto root_container =
      ftxui::Container::Vertical({top_container, clear_logs_button});

  auto app = Renderer(root_container, [&] {
    if (write_button_flash_ticks > 0) {
      --write_button_flash_ticks;
    }
    if (read_button_flash_ticks > 0) {
      --read_button_flash_ticks;
    }

    const auto terminal = ftxui::Terminal::Size();
    const int term_w = std::max(terminal.dimx, 70);
    const int term_h = std::max(terminal.dimy, 24);

    int bottom_h = std::max(7, term_h / 3);
    int top_h = term_h - bottom_h;
    if (top_h < 12) {
      top_h = 12;
      bottom_h = std::max(7, term_h - top_h);
    }

    const int panel_w = std::max(30, term_w / 2 - 1);
    const int menu_h = std::max(6, top_h / 2 - 2);
    const int editor_h = std::max(4, top_h - menu_h - 8);

    const ParamSpec &current_write_spec = specs.at(write_param_index);

    Element left_panel = vbox({
                             text("Write Panel (mouse-first)") | bold | color(Color::Green),
                             separator(),
                             text("Select parameter:"),
                             write_menu->Render() | frame | vscroll_indicator |
                                 size(ftxui::HEIGHT, ftxui::EQUAL, menu_h),
                             separator(),
                             text("Set value:"),
                             write_editor_tab->Render() | frame | vscroll_indicator |
                                 size(ftxui::HEIGHT, ftxui::EQUAL, editor_h),
                             separator(),
                             write_button->Render() | center,
                             text("Hint: " + current_write_spec.hint) | dim,
                         }) |
                         border | size(ftxui::WIDTH, ftxui::EQUAL, panel_w) |
                         size(ftxui::HEIGHT, ftxui::EQUAL, top_h) | flex;

    Element read_value_panel = vbox({
                                  text("Raw value: " + read_raw),
                                  separator(),
                                  text("Decoded value:"),
                                  paragraph(read_decoded),
                              }) |
                              frame | vscroll_indicator |
                              size(ftxui::HEIGHT, ftxui::EQUAL, editor_h);

    Element right_panel = vbox({
                              text("Read Panel (mouse-first)") | bold | color(Color::Cyan),
                              separator(),
                              text("Select parameter:"),
                              read_menu->Render() | frame | vscroll_indicator |
                                  size(ftxui::HEIGHT, ftxui::EQUAL, menu_h),
                              separator(),
                              text("Value:"),
                              read_value_panel,
                              separator(),
                              read_button->Render() | center,
                              text("Status: " + read_status) |
                                  color(read_status_ok ? Color::GreenLight
                                                       : Color::RedLight),
                          }) |
                          border | size(ftxui::WIDTH, ftxui::EQUAL, panel_w) |
                          size(ftxui::HEIGHT, ftxui::EQUAL, top_h) | flex;

    std::vector<std::string> log_lines = logs.Snapshot();
    const int max_visible_logs = std::max(1, bottom_h - 4);
    const int begin =
        std::max(0, static_cast<int>(log_lines.size()) - max_visible_logs);
    std::vector<Element> visible_logs;
    visible_logs.reserve(static_cast<size_t>(max_visible_logs));
    for (int i = begin; i < static_cast<int>(log_lines.size()); ++i) {
      visible_logs.push_back(text(log_lines[static_cast<size_t>(i)]));
    }
    if (visible_logs.empty()) {
      visible_logs.push_back(text("(log is empty)") | dim);
    }

    Element bottom_panel = vbox({
                                hbox({
                                    text("Live Logs (std::clog redirect)") |
                                        bold,
                                    filler(),
                                    clear_logs_button->Render(),
                                }),
                                separator(),
                                vbox(std::move(visible_logs)) | frame | vscroll_indicator,
                            }) |
                            border | size(ftxui::HEIGHT, ftxui::EQUAL, bottom_h);

    return vbox({
        hbox({left_panel, right_panel}) | size(ftxui::HEIGHT, ftxui::EQUAL, top_h),
        bottom_panel,
    });
  });

  ScreenInteractive screen = ScreenInteractive::Fullscreen();
  std::atomic<bool> running {true};
  std::thread refresher([&] {
    while (running.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(120));
      screen.PostEvent(Event::Custom);
    }
  });

  auto app_with_exit = CatchEvent(app, [&](Event event) {
    if (event == Event::Character('q') || event == Event::Escape) {
      screen.ExitLoopClosure()();
      return true;
    }
    return false;
  });

  log_event("px4_param_tui started.");
  log_event("PX4 param services: " + set_service + " | " + get_service);
  log_event("Mouse-first usage: click lists, click options, click action buttons.");
  log_event("Keyboard fallback: q / Esc to quit.");

  screen.Loop(app_with_exit);
  running.store(false);
  if (refresher.joinable()) {
    refresher.join();
  }

  log_event("px4_param_tui exited.");
  return 0;
}
