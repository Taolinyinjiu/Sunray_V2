#include <ftxui/component/component.hpp>
#include <ftxui/component/component_options.hpp>
#include <ftxui/component/event.hpp>
#include <ftxui/component/screen_interactive.hpp>
#include <ftxui/dom/elements.hpp>
#include <ftxui/screen/terminal.hpp>

#include <lzma.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamSet.h>
#include <nlohmann/json.hpp>
#include <ros/ros.h>
#include <ros/service.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <ctime>
#include <deque>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <streambuf>
#include <string>
#include <thread>
#include <vector>

namespace {

using json = nlohmann::json;
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

enum class ParamKind { kEnum, kBitmask, kScalar };
enum class ValueType { kInt, kFloat, kBool, kUnknown };

struct ParamSpec {
  std::string name;
  ParamKind kind = ParamKind::kScalar;
  ValueType value_type = ValueType::kUnknown;
  std::vector<std::string> options;
  std::vector<int> option_values;
  std::string hint;
  std::string unit;
  bool has_min = false;
  double min_value = 0.0;
  bool has_max = false;
  double max_value = 0.0;
  bool has_default = false;
  double default_value = 0.0;
};

struct ParamWriteState {
  int enum_choice = 0;
  std::vector<std::shared_ptr<bool>> bit_choices;
  std::string scalar_input = "0";
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

std::string FormatDoubleCompact(double value) {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(6) << value;
  std::string s = oss.str();
  while (!s.empty() && s.back() == '0') {
    s.pop_back();
  }
  if (!s.empty() && s.back() == '.') {
    s.pop_back();
  }
  if (s.empty()) {
    return "0";
  }
  return s;
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

bool ParseIntStrict(const std::string &input, int *value_out) {
  if (value_out == nullptr) {
    return false;
  }
  const std::string trimmed = Trim(input);
  if (trimmed.empty()) {
    return false;
  }
  size_t consumed = 0;
  try {
    long long value = std::stoll(trimmed, &consumed, 10);
    if (consumed != trimmed.size()) {
      return false;
    }
    if (value < std::numeric_limits<int>::min() ||
        value > std::numeric_limits<int>::max()) {
      return false;
    }
    *value_out = static_cast<int>(value);
    return true;
  } catch (...) {
    return false;
  }
}

bool JsonToDouble(const json &j, double *out) {
  if (out == nullptr || j.is_null()) {
    return false;
  }
  try {
    if (j.is_number_float()) {
      *out = j.get<double>();
      return true;
    }
    if (j.is_number_integer()) {
      *out = static_cast<double>(j.get<long long>());
      return true;
    }
    if (j.is_number_unsigned()) {
      *out = static_cast<double>(j.get<unsigned long long>());
      return true;
    }
    if (j.is_string()) {
      return ParseDoubleStrict(j.get<std::string>(), out);
    }
  } catch (...) {
    return false;
  }
  return false;
}

bool JsonToInt(const json &j, int *out) {
  if (out == nullptr || j.is_null()) {
    return false;
  }
  try {
    if (j.is_number_integer()) {
      const auto v = j.get<long long>();
      if (v < std::numeric_limits<int>::min() ||
          v > std::numeric_limits<int>::max()) {
        return false;
      }
      *out = static_cast<int>(v);
      return true;
    }
    if (j.is_number_unsigned()) {
      const auto v = j.get<unsigned long long>();
      if (v > static_cast<unsigned long long>(std::numeric_limits<int>::max())) {
        return false;
      }
      *out = static_cast<int>(v);
      return true;
    }
    if (j.is_number_float()) {
      *out = static_cast<int>(j.get<double>());
      return true;
    }
    if (j.is_string()) {
      return ParseIntStrict(j.get<std::string>(), out);
    }
  } catch (...) {
    return false;
  }
  return false;
}

std::string ReadTextFile(const std::string &path) {
  std::ifstream ifs(path, std::ios::in);
  if (!ifs) {
    return "";
  }
  std::ostringstream oss;
  oss << ifs.rdbuf();
  return oss.str();
}

std::string ReadXzTextFile(const std::string &path, std::string *err_out) {
  std::ifstream ifs(path, std::ios::binary);
  if (!ifs) {
    if (err_out != nullptr) {
      *err_out = "failed to open xz file: " + path;
    }
    return "";
  }

  lzma_stream strm = LZMA_STREAM_INIT;
  lzma_ret ret = lzma_stream_decoder(&strm, UINT64_MAX, 0);
  if (ret != LZMA_OK) {
    if (err_out != nullptr) {
      *err_out = "lzma_stream_decoder init failed";
    }
    return "";
  }

  std::array<uint8_t, 1 << 14> in_buf {};
  std::array<uint8_t, 1 << 14> out_buf {};
  std::string output;
  output.reserve(2 << 20);

  bool done = false;
  bool eof = false;
  lzma_action action = LZMA_RUN;

  while (!done) {
    if (strm.avail_in == 0 && !eof) {
      ifs.read(reinterpret_cast<char *>(in_buf.data()),
               static_cast<std::streamsize>(in_buf.size()));
      const auto read_bytes = static_cast<size_t>(ifs.gcount());
      strm.next_in = in_buf.data();
      strm.avail_in = read_bytes;
      eof = ifs.eof();
      if (eof) {
        action = LZMA_FINISH;
      }
    }

    strm.next_out = out_buf.data();
    strm.avail_out = out_buf.size();
    ret = lzma_code(&strm, action);
    const size_t produced = out_buf.size() - strm.avail_out;
    if (produced > 0) {
      output.append(reinterpret_cast<char *>(out_buf.data()), produced);
    }

    if (ret == LZMA_STREAM_END) {
      done = true;
      break;
    }
    if (ret != LZMA_OK) {
      if (err_out != nullptr) {
        *err_out = "lzma decode failed, code=" + std::to_string(static_cast<int>(ret));
      }
      lzma_end(&strm);
      return "";
    }
    if (eof && strm.avail_in == 0 && produced == 0) {
      break;
    }
  }

  lzma_end(&strm);
  return output;
}

bool RangeCheck(const ParamSpec &spec, double value, std::string *err_out) {
  if (spec.has_min && value < spec.min_value) {
    if (err_out != nullptr) {
      *err_out = "below min: " + FormatDoubleCompact(spec.min_value);
    }
    return false;
  }
  if (spec.has_max && value > spec.max_value) {
    if (err_out != nullptr) {
      *err_out = "above max: " + FormatDoubleCompact(spec.max_value);
    }
    return false;
  }
  return true;
}

std::optional<int> ParseEnumRawValue(const ParamSpec &spec,
                                     const ParamWriteState &state) {
  if (state.enum_choice < 0 ||
      state.enum_choice >= static_cast<int>(spec.option_values.size())) {
    return std::nullopt;
  }
  return spec.option_values[static_cast<size_t>(state.enum_choice)];
}

uint32_t BuildBitmaskRaw(const ParamSpec &spec, const ParamWriteState &state) {
  uint32_t raw = 0u;
  const size_t n =
      std::min(spec.option_values.size(), state.bit_choices.size());
  for (size_t i = 0; i < n; ++i) {
    if (state.bit_choices[i] != nullptr && *state.bit_choices[i]) {
      const int bit = spec.option_values[i];
      if (bit >= 0 && bit < 31) {
        raw |= (1u << bit);
      }
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

std::string BuildHint(const ParamSpec &spec, const std::string &short_desc) {
  std::vector<std::string> parts;
  if (!short_desc.empty()) {
    parts.push_back(short_desc);
  }
  if (!spec.unit.empty()) {
    parts.push_back("unit=" + spec.unit);
  }
  if (spec.has_min || spec.has_max) {
    const std::string min_s = spec.has_min ? FormatDoubleCompact(spec.min_value) : "-inf";
    const std::string max_s = spec.has_max ? FormatDoubleCompact(spec.max_value) : "+inf";
    parts.push_back("range=[" + min_s + ", " + max_s + "]");
  }
  if (parts.empty()) {
    return "No hint.";
  }
  return Join(parts, " | ");
}

ValueType ParseValueType(const std::string &raw_type) {
  const std::string t = ToLower(raw_type);
  if (t.find("float") != std::string::npos || t.find("double") != std::string::npos) {
    return ValueType::kFloat;
  }
  if (t.find("bool") != std::string::npos) {
    return ValueType::kBool;
  }
  if (t.find("int") != std::string::npos) {
    return ValueType::kInt;
  }
  return ValueType::kUnknown;
}

int ResolveSelectedSpecIndex(const std::vector<int> &filtered_indices,
                             int menu_selected_index) {
  if (menu_selected_index < 0 ||
      menu_selected_index >= static_cast<int>(filtered_indices.size())) {
    return -1;
  }
  return filtered_indices[static_cast<size_t>(menu_selected_index)];
}

size_t RebuildFilteredMenuEntries(const std::vector<ParamSpec> &specs,
                                  const std::string &query,
                                  std::vector<int> *filtered_indices,
                                  std::vector<std::string> *filtered_names,
                                  int preferred_spec_index,
                                  int *menu_selected_index,
                                  int *menu_focused_index) {
  if (filtered_indices == nullptr || filtered_names == nullptr ||
      menu_selected_index == nullptr || menu_focused_index == nullptr) {
    return 0;
  }

  const int previous_selected_spec =
      ResolveSelectedSpecIndex(*filtered_indices, *menu_selected_index);
  const std::string key = ToLower(Trim(query));

  filtered_indices->clear();
  filtered_names->clear();
  filtered_indices->reserve(specs.size());
  filtered_names->reserve(specs.size());
  for (size_t i = 0; i < specs.size(); ++i) {
    const std::string name_lower = ToLower(specs[i].name);
    if (key.empty() || name_lower.find(key) != std::string::npos) {
      filtered_indices->push_back(static_cast<int>(i));
      filtered_names->push_back(specs[i].name);
    }
  }

  const size_t match_count = filtered_indices->size();
  if (match_count == 0) {
    filtered_indices->push_back(-1);
    filtered_names->push_back("(no matches)");
    *menu_selected_index = 0;
    *menu_focused_index = 0;
    return 0;
  }

  int target_spec_index = -1;
  if (previous_selected_spec >= 0) {
    target_spec_index = previous_selected_spec;
  } else if (preferred_spec_index >= 0) {
    target_spec_index = preferred_spec_index;
  }

  int target_menu_index = -1;
  if (target_spec_index >= 0) {
    for (size_t i = 0; i < filtered_indices->size(); ++i) {
      if ((*filtered_indices)[i] == target_spec_index) {
        target_menu_index = static_cast<int>(i);
        break;
      }
    }
  }

  if (target_menu_index < 0) {
    if (*menu_selected_index < 0) {
      *menu_selected_index = 0;
    }
    if (*menu_selected_index >= static_cast<int>(filtered_indices->size())) {
      *menu_selected_index = static_cast<int>(filtered_indices->size()) - 1;
    }
    target_menu_index = *menu_selected_index;
  }

  *menu_selected_index = target_menu_index;
  *menu_focused_index = target_menu_index;
  return match_count;
}

std::vector<ParamSpec> BuildSpecsFromMetadata(const std::string &metadata_path,
                                              std::string *err_out) {
  std::string text;
  if (metadata_path.size() >= 3 &&
      metadata_path.substr(metadata_path.size() - 3) == ".xz") {
    text = ReadXzTextFile(metadata_path, err_out);
  } else {
    text = ReadTextFile(metadata_path);
    if (text.empty() && err_out != nullptr) {
      *err_out = "failed to read metadata file: " + metadata_path;
    }
  }
  if (text.empty()) {
    return {};
  }

  json root;
  try {
    root = json::parse(text);
  } catch (const std::exception &e) {
    if (err_out != nullptr) {
      *err_out = std::string("json parse failed: ") + e.what();
    }
    return {};
  }

  if (!root.contains("parameters") || !root["parameters"].is_array()) {
    if (err_out != nullptr) {
      *err_out = "metadata missing 'parameters' array";
    }
    return {};
  }

  std::vector<ParamSpec> specs;
  specs.reserve(root["parameters"].size());
  for (const auto &entry : root["parameters"]) {
    if (!entry.is_object()) {
      continue;
    }
    if (!entry.contains("name") || !entry["name"].is_string()) {
      continue;
    }

    ParamSpec spec;
    spec.name = entry["name"].get<std::string>();
    if (spec.name.empty()) {
      continue;
    }

    const std::string raw_type = entry.value("type", "");
    spec.value_type = ParseValueType(raw_type);
    spec.unit = entry.value("units", "");

    double tmp = 0.0;
    if (entry.contains("min") && JsonToDouble(entry["min"], &tmp)) {
      spec.has_min = true;
      spec.min_value = tmp;
    }
    if (entry.contains("max") && JsonToDouble(entry["max"], &tmp)) {
      spec.has_max = true;
      spec.max_value = tmp;
    }
    if (entry.contains("default") && JsonToDouble(entry["default"], &tmp)) {
      spec.has_default = true;
      spec.default_value = tmp;
    }

    const std::string short_desc = entry.value("shortDesc", "");
    spec.hint = BuildHint(spec, short_desc);

    if (entry.contains("bitmask") && entry["bitmask"].is_array() &&
        !entry["bitmask"].empty()) {
      spec.kind = ParamKind::kBitmask;
      for (const auto &bit_item : entry["bitmask"]) {
        if (!bit_item.is_object()) {
          continue;
        }
        int bit_index = 0;
        if (!bit_item.contains("index") || !JsonToInt(bit_item["index"], &bit_index)) {
          continue;
        }
        const std::string desc = bit_item.value("description", "");
        spec.option_values.push_back(bit_index);
        spec.options.push_back(desc.empty() ? ("bit" + std::to_string(bit_index))
                                            : (desc + " (bit" + std::to_string(bit_index) + ")"));
      }
    } else if (entry.contains("values") && entry["values"].is_array() &&
               !entry["values"].empty()) {
      spec.kind = ParamKind::kEnum;
      for (const auto &value_item : entry["values"]) {
        if (!value_item.is_object()) {
          continue;
        }
        int value_code = 0;
        if (!value_item.contains("value") || !JsonToInt(value_item["value"], &value_code)) {
          continue;
        }
        const std::string desc = value_item.value("description", "");
        spec.option_values.push_back(value_code);
        spec.options.push_back(desc.empty() ? std::to_string(value_code)
                                            : (desc + " (" + std::to_string(value_code) + ")"));
      }
    } else {
      spec.kind = ParamKind::kScalar;
    }

    if (spec.kind == ParamKind::kScalar && spec.value_type == ValueType::kUnknown) {
      // Unknown scalar defaults to float write/read path.
      spec.value_type = ValueType::kFloat;
    }

    specs.push_back(std::move(spec));
  }

  std::sort(specs.begin(), specs.end(),
            [](const ParamSpec &a, const ParamSpec &b) { return a.name < b.name; });

  if (specs.empty() && err_out != nullptr) {
    *err_out = "no valid parameter entries found";
  }

  return specs;
}

std::vector<ParamWriteState> BuildWriteStates(const std::vector<ParamSpec> &specs) {
  std::vector<ParamWriteState> states(specs.size());
  for (size_t i = 0; i < specs.size(); ++i) {
    const auto &spec = specs[i];
    if (spec.kind == ParamKind::kBitmask) {
      states[i].bit_choices.reserve(spec.options.size());
      for (size_t j = 0; j < spec.options.size(); ++j) {
        const int bit = spec.option_values[j];
        bool enabled = false;
        if (spec.has_default && bit >= 0 && bit < 31) {
          const uint32_t raw = static_cast<uint32_t>(spec.default_value);
          enabled = ((raw & (1u << bit)) != 0u);
        }
        states[i].bit_choices.push_back(std::make_shared<bool>(enabled));
      }
      continue;
    }

    if (spec.kind == ParamKind::kEnum) {
      if (spec.has_default) {
        const int default_raw = static_cast<int>(spec.default_value);
        for (size_t j = 0; j < spec.option_values.size(); ++j) {
          if (spec.option_values[j] == default_raw) {
            states[i].enum_choice = static_cast<int>(j);
            break;
          }
        }
      }
      continue;
    }

    if (spec.has_default) {
      if (spec.value_type == ValueType::kInt || spec.value_type == ValueType::kBool) {
        states[i].scalar_input = std::to_string(static_cast<int>(spec.default_value));
      } else {
        states[i].scalar_input = FormatDoubleCompact(spec.default_value);
      }
    } else {
      states[i].scalar_input = (spec.value_type == ValueType::kFloat) ? "0.0" : "0";
    }
  }
  return states;
}

bool WriteParamGeneric(ros::ServiceClient &set_client, const ParamSpec &spec,
                       const ParamWriteState &state, std::string *detail_out,
                       std::string *err_out) {
  auto fail = [&](const std::string &msg) {
    if (err_out != nullptr) {
      *err_out = msg;
    }
    return false;
  };

  mavros_msgs::ParamSet srv;
  srv.request.param_id = spec.name;

  if (spec.kind == ParamKind::kEnum) {
    const auto raw = ParseEnumRawValue(spec, state);
    if (!raw.has_value()) {
      return fail("invalid enum selection");
    }
    if (!RangeCheck(spec, static_cast<double>(*raw), err_out)) {
      return false;
    }
    srv.request.value.integer = *raw;
    const bool ok = set_client.call(srv) && srv.response.success;
    if (detail_out != nullptr) {
      *detail_out = "raw=" + std::to_string(*raw);
    }
    return ok;
  }

  if (spec.kind == ParamKind::kBitmask) {
    const uint32_t raw = BuildBitmaskRaw(spec, state);
    if (!RangeCheck(spec, static_cast<double>(raw), err_out)) {
      return false;
    }
    srv.request.value.integer = static_cast<int>(raw);
    const bool ok = set_client.call(srv) && srv.response.success;
    if (detail_out != nullptr) {
      *detail_out = "raw=" + std::to_string(static_cast<int>(raw));
    }
    return ok;
  }

  if (spec.value_type == ValueType::kInt || spec.value_type == ValueType::kBool) {
    int raw = 0;
    if (!ParseIntStrict(state.scalar_input, &raw)) {
      return fail("invalid integer input");
    }
    if (!RangeCheck(spec, static_cast<double>(raw), err_out)) {
      return false;
    }
    srv.request.value.integer = raw;
    const bool ok = set_client.call(srv) && srv.response.success;
    if (detail_out != nullptr) {
      *detail_out = "value=" + std::to_string(raw);
    }
    return ok;
  }

  double value = 0.0;
  if (!ParseDoubleStrict(state.scalar_input, &value)) {
    return fail("invalid numeric input");
  }
  if (!RangeCheck(spec, value, err_out)) {
    return false;
  }
  srv.request.value.real = static_cast<float>(value);
  const bool ok = set_client.call(srv) && srv.response.success;
  if (detail_out != nullptr) {
    *detail_out = "value=" + FormatDoubleCompact(value);
  }
  return ok;
}

bool ReadParamGeneric(ros::ServiceClient &get_client, const ParamSpec &spec,
                      std::string *raw_out, std::string *decoded_out,
                      std::string *err_out) {
  auto fail = [&](const std::string &msg) {
    if (err_out != nullptr) {
      *err_out = msg;
    }
    return false;
  };

  mavros_msgs::ParamGet srv;
  srv.request.param_id = spec.name;
  if (!get_client.call(srv) || !srv.response.success) {
    return fail("service call failed");
  }

  if (spec.kind == ParamKind::kEnum ||
      spec.kind == ParamKind::kBitmask ||
      spec.value_type == ValueType::kInt || spec.value_type == ValueType::kBool) {
    const int raw = static_cast<int>(srv.response.value.integer);
    if (raw_out != nullptr) {
      *raw_out = std::to_string(raw);
    }

    if (decoded_out != nullptr) {
      if (spec.kind == ParamKind::kEnum) {
        std::string mapped = "Unknown";
        for (size_t i = 0; i < spec.option_values.size(); ++i) {
          if (spec.option_values[i] == raw) {
            mapped = spec.options[i];
            break;
          }
        }
        *decoded_out = mapped;
      } else if (spec.kind == ParamKind::kBitmask) {
        *decoded_out = FormatEnabledByBits(spec, static_cast<uint32_t>(raw));
      } else if (spec.value_type == ValueType::kBool) {
        *decoded_out = (raw == 0) ? "false" : "true";
      } else {
        *decoded_out = spec.unit.empty() ? std::to_string(raw)
                                         : (std::to_string(raw) + " " + spec.unit);
      }
    }
    return true;
  }

  const double raw = static_cast<double>(srv.response.value.real);
  if (raw_out != nullptr) {
    *raw_out = FormatDoubleCompact(raw);
  }
  if (decoded_out != nullptr) {
    const std::string value_s = FormatDoubleCompact(raw);
    *decoded_out = spec.unit.empty() ? value_s : (value_s + " " + spec.unit);
  }
  return true;
}

} // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "px4_param_tui_meta_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  int uav_id = 1;
  std::string uav_name = "uav";
  nh.param("uav_id", uav_id, uav_id);
  nh.param("uav_name", uav_name, uav_name);

  std::string metadata_path;
  pnh.param<std::string>("metadata_path", metadata_path, "");
  if (metadata_path.empty() && argc > 1 && argv[1] != nullptr) {
    metadata_path = argv[1];
  }
  if (metadata_path.empty()) {
    std::cerr << "metadata path is required. "
              << "Use _metadata_path:=/abs/path/parameters.json.xz or argv[1]." << std::endl;
    return 1;
  }

  const std::string mavros_ns = "/" + uav_name + std::to_string(uav_id) + "/mavros";
  const std::string set_service = mavros_ns + "/param/set";
  const std::string get_service = mavros_ns + "/param/get";
  auto param_set_client = nh.serviceClient<mavros_msgs::ParamSet>(set_service);
  auto param_get_client = nh.serviceClient<mavros_msgs::ParamGet>(get_service);

  LogStore logs(600);
  auto log_event = [&](const std::string &msg) {
    logs.Push("[" + NowTimestamp() + "] " + msg);
  };

  ScopedStreamRedirect redirect_clog(std::clog, [&](const std::string &line) {
    logs.Push("[" + NowTimestamp() + "] [STDLOG] " + line);
  },
                                     false);

  std::string load_error;
  const std::vector<ParamSpec> specs = BuildSpecsFromMetadata(metadata_path, &load_error);
  if (specs.empty()) {
    std::cerr << "failed to load metadata: " << load_error << std::endl;
    return 2;
  }

  std::vector<ParamWriteState> write_states = BuildWriteStates(specs);

  int write_param_index = 0;
  int read_param_index = 0;
  int write_menu_selected_index = 0;
  int read_menu_selected_index = 0;
  int write_focus_index = 0;
  int read_focus_index = 0;
  std::string write_search_text;
  std::string read_search_text;
  std::vector<int> write_filtered_indices;
  std::vector<int> read_filtered_indices;
  std::vector<std::string> write_menu_entries;
  std::vector<std::string> read_menu_entries;

  RebuildFilteredMenuEntries(specs, write_search_text, &write_filtered_indices,
                             &write_menu_entries, write_param_index,
                             &write_menu_selected_index, &write_focus_index);
  RebuildFilteredMenuEntries(specs, read_search_text, &read_filtered_indices,
                             &read_menu_entries, read_param_index,
                             &read_menu_selected_index, &read_focus_index);

  int write_button_flash_ticks = 0;
  int read_button_flash_ticks = 0;
  std::string read_status = "No read operation yet.";
  bool read_status_ok = true;
  std::string read_raw = "-";
  std::string read_decoded = "-";

  auto make_param_menu = [&](std::vector<std::string> *entries, int *selected,
                             int *focused, Color active_color) {
    ftxui::MenuOption option = ftxui::MenuOption::Vertical();
    option.entries = entries;
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

  auto write_menu = make_param_menu(&write_menu_entries, &write_menu_selected_index,
                                    &write_focus_index, Color::GreenLight);
  auto read_menu = make_param_menu(&read_menu_entries, &read_menu_selected_index,
                                   &read_focus_index, Color::CyanLight);

  ftxui::InputOption input_option = ftxui::InputOption::Default();
  input_option.transform = [](ftxui::InputState state) {
    state.element |= color(Color::White);
    if (state.is_placeholder) {
      state.element |= dim;
    }
    if (state.focused) {
      state.element |= bold;
    }
    return state.element;
  };

  ftxui::InputOption search_input_option = ftxui::InputOption::Default();
  search_input_option.transform = [](ftxui::InputState state) {
    state.element |= color(Color::White);
    if (state.is_placeholder) {
      state.element |= dim;
    }
    if (state.focused) {
      state.element |= bold;
    }
    return state.element;
  };

  auto write_search_input =
      Input(&write_search_text, "search parameter name", search_input_option);
  auto read_search_input =
      Input(&read_search_text, "search parameter name", search_input_option);

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
    std::string placeholder = (spec.value_type == ValueType::kFloat) ? "numeric value"
                                                                      : "integer value";
    write_editors.push_back(
        Input(&write_states[i].scalar_input, placeholder, input_option));
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
    const int selected_spec_index =
        ResolveSelectedSpecIndex(write_filtered_indices, write_menu_selected_index);
    if (selected_spec_index < 0) {
      log_event("WRITE skipped: no matched parameter in left panel.");
      return;
    }
    write_param_index = selected_spec_index;
    const ParamSpec &spec = specs.at(static_cast<size_t>(write_param_index));

    if (!ros::service::waitForService(set_service, ros::Duration(0.25))) {
      log_event("WRITE failed for " + spec.name + ": set service unavailable.");
      return;
    }

    std::string detail;
    std::string error;
    const bool ok = WriteParamGeneric(param_set_client, spec,
                                      write_states[static_cast<size_t>(write_param_index)],
                                      &detail, &error);
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
    const int selected_spec_index =
        ResolveSelectedSpecIndex(read_filtered_indices, read_menu_selected_index);
    if (selected_spec_index < 0) {
      read_status = "No matched parameter in right panel.";
      read_status_ok = false;
      log_event("READ skipped: no matched parameter in right panel.");
      return;
    }
    read_param_index = selected_spec_index;
    const ParamSpec &spec = specs.at(static_cast<size_t>(read_param_index));

    if (!ros::service::waitForService(get_service, ros::Duration(0.25))) {
      read_status = "Service unavailable: " + get_service;
      read_status_ok = false;
      log_event("READ failed for " + spec.name + ": get service unavailable.");
      return;
    }

    std::string raw;
    std::string decoded;
    std::string error;
    const bool ok =
        ReadParamGeneric(param_get_client, spec, &raw, &decoded, &error);
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

  auto left_panel_container = ftxui::Container::Vertical(
      {write_search_input, write_menu, write_editor_tab, write_button});
  auto right_panel_container = ftxui::Container::Vertical(
      {read_search_input, read_menu, read_button});
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

    const size_t write_match_count = RebuildFilteredMenuEntries(
        specs, write_search_text, &write_filtered_indices, &write_menu_entries,
        write_param_index, &write_menu_selected_index, &write_focus_index);
    const size_t read_match_count = RebuildFilteredMenuEntries(
        specs, read_search_text, &read_filtered_indices, &read_menu_entries,
        read_param_index, &read_menu_selected_index, &read_focus_index);

    const int selected_write_spec =
        ResolveSelectedSpecIndex(write_filtered_indices, write_menu_selected_index);
    const int selected_read_spec =
        ResolveSelectedSpecIndex(read_filtered_indices, read_menu_selected_index);
    if (selected_write_spec >= 0) {
      write_param_index = selected_write_spec;
    }
    if (selected_read_spec >= 0) {
      read_param_index = selected_read_spec;
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
    const int menu_h = std::max(5, top_h / 2 - 4);
    const int editor_h = std::max(4, top_h - menu_h - 10);

    const ParamSpec &current_write_spec =
        specs.at(static_cast<size_t>(write_param_index));

    Element left_panel = vbox({
                             text("Write Panel (mouse-first)") | bold | color(Color::Green),
                             separator(),
                             text("Search:"),
                             write_search_input->Render() | border,
                             text("Matches: " + std::to_string(write_match_count)) | dim,
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
                              text("Search:"),
                              read_search_input->Render() | border,
                              text("Matches: " + std::to_string(read_match_count)) | dim,
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
                                    text("Live Logs (std::clog redirect)") | bold,
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

  log_event("px4_param_tui_meta started.");
  log_event("metadata: " + metadata_path);
  log_event("loaded parameters: " + std::to_string(specs.size()));
  log_event("PX4 param services: " + set_service + " | " + get_service);
  log_event("Mouse-first usage: click lists, click options, click action buttons.");
  log_event("Keyboard fallback: q / Esc to quit.");

  screen.Loop(app_with_exit);
  running.store(false);
  if (refresher.joinable()) {
    refresher.join();
  }

  log_event("px4_param_tui_meta exited.");
  return 0;
}
