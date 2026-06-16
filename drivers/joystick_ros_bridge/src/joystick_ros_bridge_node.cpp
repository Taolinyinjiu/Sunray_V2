#include <fcntl.h>
#include <linux/joystick.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <mavros_msgs/OverrideRCIn.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace
{
constexpr float kDefaultAxisScale = 32767.0f;
constexpr size_t kRcChannelCount = 18;

int clampInt(const int value, const int low, const int high)
{
  return std::max(low, std::min(value, high));
}

double clampDouble(const double value, const double low, const double high)
{
  return std::max(low, std::min(value, high));
}

std::string xmlTypeName(const XmlRpc::XmlRpcValue::Type type)
{
  switch (type)
  {
    case XmlRpc::XmlRpcValue::TypeBoolean:
      return "bool";
    case XmlRpc::XmlRpcValue::TypeInt:
      return "int";
    case XmlRpc::XmlRpcValue::TypeDouble:
      return "double";
    case XmlRpc::XmlRpcValue::TypeString:
      return "string";
    case XmlRpc::XmlRpcValue::TypeArray:
      return "array";
    case XmlRpc::XmlRpcValue::TypeStruct:
      return "struct";
    default:
      return "unknown";
  }
}

bool xmlHasMember(XmlRpc::XmlRpcValue& value, const std::string& key)
{
  return value.getType() == XmlRpc::XmlRpcValue::TypeStruct && value.hasMember(key);
}

bool readIntMember(XmlRpc::XmlRpcValue& value, const std::string& key, int& out)
{
  if (!xmlHasMember(value, key))
  {
    return false;
  }

  XmlRpc::XmlRpcValue& member = value[key];
  if (member.getType() != XmlRpc::XmlRpcValue::TypeInt)
  {
    ROS_WARN("rc_channels member '%s' expected int, got %s", key.c_str(), xmlTypeName(member.getType()).c_str());
    return false;
  }

  out = static_cast<int>(member);
  return true;
}

bool readDoubleMember(XmlRpc::XmlRpcValue& value, const std::string& key, double& out)
{
  if (!xmlHasMember(value, key))
  {
    return false;
  }

  XmlRpc::XmlRpcValue& member = value[key];
  if (member.getType() == XmlRpc::XmlRpcValue::TypeDouble)
  {
    out = static_cast<double>(member);
    return true;
  }
  if (member.getType() == XmlRpc::XmlRpcValue::TypeInt)
  {
    out = static_cast<int>(member);
    return true;
  }

  ROS_WARN("rc_channels member '%s' expected number, got %s", key.c_str(), xmlTypeName(member.getType()).c_str());
  return false;
}

bool readStringMember(XmlRpc::XmlRpcValue& value, const std::string& key, std::string& out)
{
  if (!xmlHasMember(value, key))
  {
    return false;
  }

  XmlRpc::XmlRpcValue& member = value[key];
  if (member.getType() != XmlRpc::XmlRpcValue::TypeString)
  {
    ROS_WARN("rc_channels member '%s' expected string, got %s", key.c_str(), xmlTypeName(member.getType()).c_str());
    return false;
  }

  out = static_cast<std::string>(member);
  return true;
}

struct RcChannelConfig
{
  enum class SourceType
  {
    None,
    Axis,
    Button
  };

  SourceType type = SourceType::None;
  int source = -1;
  double in_min = -1.0;
  double in_max = 1.0;
  int out_min = 1000;
  int out_max = 2000;
  int button_low = 1000;
  int button_high = 2000;
  bool button_toggle = false;
  std::string label;
};

class JoystickRosBridge
{
public:
  JoystickRosBridge()
    : private_nh_("~")
  {
    private_nh_.param<std::string>("device", device_, "/dev/input/js0");
    private_nh_.param<std::string>("pub_joy_topic", pub_joy_topic_, "/joy");
    private_nh_.param<double>("publish_rate", publish_rate_, 100.0);
    private_nh_.param<double>("deadzone", deadzone_, 0.05);
    private_nh_.param<bool>("autorepeat", autorepeat_, true);
    private_nh_.param<int>("default_axes", default_axes_, 6);
    private_nh_.param<int>("default_buttons", default_buttons_, 13);

    private_nh_.param<bool>("enable_rc_override", enable_rc_override_, true);
    private_nh_.param<std::string>("rc_override_topic", rc_override_topic_, "/mavros/rc/override");
    private_nh_.param<int>("rc_default", rc_default_, 1000);
    private_nh_.param<int>("rc_min", rc_min_, 1000);
    private_nh_.param<int>("rc_max", rc_max_, 2000);
    private_nh_.param<int>("rc_neutral", rc_neutral_, 1500);
    private_nh_.param<int>("rc_button_low", rc_button_low_, 1000);
    private_nh_.param<int>("rc_button_high", rc_button_high_, 2000);

    private_nh_.param<bool>("print_status", print_status_, true);
    private_nh_.param<double>("print_rate", print_rate_, 10.0);

    private_nh_.getParam("pub_axis_indices", pub_axis_indices_);
    private_nh_.getParam("pub_button_indices", pub_button_indices_);

    normalizeParams();
    loadRcChannelConfig();

    joy_pub_ = nh_.advertise<sensor_msgs::Joy>(pub_joy_topic_, 10);
    if (enable_rc_override_)
    {
      rc_override_pub_ = nh_.advertise<mavros_msgs::OverrideRCIn>(rc_override_topic_, 10);
      ROS_WARN("RC override publishing is enabled on %s. Verify channel mapping before flying.",
               rc_override_topic_.c_str());
    }

    printStartupConfig();
  }

  ~JoystickRosBridge()
  {
    closeDevice();
  }

  bool openDevice()
  {
    closeDevice();

    fd_ = open(device_.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd_ < 0)
    {
      ROS_ERROR("Failed to open joystick device %s: %s", device_.c_str(), std::strerror(errno));
      return false;
    }

    char name[128] = {0};
    if (ioctl(fd_, JSIOCGNAME(sizeof(name)), name) < 0)
    {
      std::strncpy(name, "Unknown joystick", sizeof(name) - 1);
    }
    joystick_name_ = name;

    unsigned char axes = 0;
    unsigned char buttons = 0;
    if (ioctl(fd_, JSIOCGAXES, &axes) < 0)
    {
      axes = static_cast<unsigned char>(default_axes_);
      ROS_WARN("Failed to query joystick axis count, using default_axes=%d", default_axes_);
    }
    if (ioctl(fd_, JSIOCGBUTTONS, &buttons) < 0)
    {
      buttons = static_cast<unsigned char>(default_buttons_);
      ROS_WARN("Failed to query joystick button count, using default_buttons=%d", default_buttons_);
    }

    // Linux joystick 事件是增量上报的，所以这里先建立完整状态缓存。
    raw_axes_.assign(axes, 0.0f);
    raw_buttons_.assign(buttons, 0);
    last_buttons_.assign(buttons, 0);
    button_toggle_states_.assign(buttons, 0);
    rebuildJoyMessage();
    last_publish_time_ = ros::Time(0);
    dirty_ = true;

    ROS_INFO("Opened joystick '%s' at %s (%u axes, %u buttons), publishing %s",
             joystick_name_.c_str(), device_.c_str(), axes, buttons, pub_joy_topic_.c_str());
    return true;
  }

  void spin()
  {
    ros::Rate rate(publish_rate_);

    while (ros::ok())
    {
      if (fd_ < 0)
      {
        openDevice();
        ros::Duration(1.0).sleep();
        ros::spinOnce();
        continue;
      }

      if (!readAvailableEvents())
      {
        closeDevice();
        ros::Duration(1.0).sleep();
      }

      publishIfNeeded();
      ros::spinOnce();
      rate.sleep();
    }
  }

private:
  void normalizeParams()
  {
    if (publish_rate_ <= 0.0)
    {
      ROS_WARN("publish_rate must be positive, using 100.0 Hz");
      publish_rate_ = 100.0;
    }
    if (print_rate_ <= 0.0)
    {
      ROS_WARN("print_rate must be positive, using 10.0 Hz");
      print_rate_ = 10.0;
    }

    deadzone_ = std::max(0.0, std::min(deadzone_, 1.0));

    rc_min_ = clampInt(rc_min_, 800, 2200);
    rc_max_ = clampInt(rc_max_, 800, 2200);
    if (rc_min_ > rc_max_)
    {
      std::swap(rc_min_, rc_max_);
    }
    rc_neutral_ = clampInt(rc_neutral_, rc_min_, rc_max_);
    rc_default_ = clampInt(rc_default_, rc_min_, rc_max_);
    rc_button_low_ = clampInt(rc_button_low_, rc_min_, rc_max_);
    rc_button_high_ = clampInt(rc_button_high_, rc_min_, rc_max_);

    if (pub_axis_indices_.empty())
    {
      pub_axis_indices_ = {0, 1, 2, 3};
    }
    if (pub_button_indices_.empty())
    {
      pub_button_indices_ = {3, 0, 1, 2};
    }

    rc_channels_.fill(RcChannelConfig{});
    for (size_t i = 0; i < rc_values_.size(); ++i)
    {
      rc_values_[i] = rc_default_;
    }
  }

  void loadRcChannelConfig()
  {
    XmlRpc::XmlRpcValue configs;
    if (!private_nh_.getParam("rc_channels", configs))
    {
      ROS_WARN("No rc_channels config found; all RC channels will publish rc_default=%d", rc_default_);
      return;
    }

    if (configs.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_WARN("rc_channels must be a YAML list, got %s", xmlTypeName(configs.getType()).c_str());
      return;
    }

    for (int i = 0; i < configs.size(); ++i)
    {
      XmlRpc::XmlRpcValue& item = configs[i];
      if (item.getType() != XmlRpc::XmlRpcValue::TypeStruct)
      {
        ROS_WARN("rc_channels[%d] must be a struct, got %s", i, xmlTypeName(item.getType()).c_str());
        continue;
      }

      int channel = -1;
      std::string type;
      int source = -1;
      if (!readIntMember(item, "channel", channel) || !readStringMember(item, "type", type) ||
          !readIntMember(item, "source", source))
      {
        ROS_WARN("rc_channels[%d] missing required fields: channel/type/source", i);
        continue;
      }
      if (channel < 0 || channel >= static_cast<int>(kRcChannelCount))
      {
        ROS_WARN("rc_channels[%d] channel=%d out of range [0, 17]", i, channel);
        continue;
      }

      RcChannelConfig config;
      config.source = source;
      config.label = "RC" + std::to_string(channel + 1);
      if (xmlHasMember(item, "label"))
      {
        readStringMember(item, "label", config.label);
      }

      if (type == "axis")
      {
        config.type = RcChannelConfig::SourceType::Axis;
        readDoubleMember(item, "in_min", config.in_min);
        readDoubleMember(item, "in_max", config.in_max);
        readIntMember(item, "out_min", config.out_min);
        readIntMember(item, "out_max", config.out_max);
        config.out_min = clampInt(config.out_min, rc_min_, rc_max_);
        config.out_max = clampInt(config.out_max, rc_min_, rc_max_);
      }
      else if (type == "button")
      {
        config.type = RcChannelConfig::SourceType::Button;
        config.button_low = rc_button_low_;
        config.button_high = rc_button_high_;
        readIntMember(item, "low", config.button_low);
        readIntMember(item, "high", config.button_high);
        config.button_low = clampInt(config.button_low, rc_min_, rc_max_);
        config.button_high = clampInt(config.button_high, rc_min_, rc_max_);

        std::string mode = "momentary";
        readStringMember(item, "mode", mode);
        config.button_toggle = (mode == "toggle");
      }
      else
      {
        ROS_WARN("rc_channels[%d] unsupported type '%s'", i, type.c_str());
        continue;
      }

      rc_channels_[channel] = config;
    }
  }

  void closeDevice()
  {
    if (fd_ >= 0)
    {
      close(fd_);
      fd_ = -1;
    }
  }

  bool readAvailableEvents()
  {
    while (ros::ok())
    {
      fd_set set;
      FD_ZERO(&set);
      FD_SET(fd_, &set);

      timeval timeout;
      timeout.tv_sec = 0;
      timeout.tv_usec = 0;

      const int ready = select(fd_ + 1, &set, nullptr, nullptr, &timeout);
      if (ready < 0)
      {
        if (errno == EINTR)
        {
          continue;
        }
        ROS_ERROR("select() failed for %s: %s", device_.c_str(), std::strerror(errno));
        return false;
      }
      if (ready == 0)
      {
        return true;
      }

      js_event event;
      const ssize_t bytes = read(fd_, &event, sizeof(event));
      if (bytes == static_cast<ssize_t>(sizeof(event)))
      {
        handleEvent(event);
        continue;
      }

      if (bytes < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
      {
        return true;
      }

      ROS_ERROR("read() failed for %s: %s", device_.c_str(), std::strerror(errno));
      return false;
    }

    return true;
  }

  void handleEvent(const js_event& event)
  {
    const uint8_t type = event.type & ~JS_EVENT_INIT;

    if (type == JS_EVENT_AXIS)
    {
      if (event.number >= raw_axes_.size())
      {
        raw_axes_.resize(event.number + 1, 0.0f);
      }

      float value = static_cast<float>(event.value) / kDefaultAxisScale;
      value = std::max(-1.0f, std::min(value, 1.0f));
      if (std::abs(value) < deadzone_)
      {
        value = 0.0f;
      }

      raw_axes_[event.number] = value;
      dirty_ = true;
    }
    else if (type == JS_EVENT_BUTTON)
    {
      if (event.number >= raw_buttons_.size())
      {
        raw_buttons_.resize(event.number + 1, 0);
        last_buttons_.resize(event.number + 1, 0);
        button_toggle_states_.resize(event.number + 1, 0);
      }

      const int new_value = event.value ? 1 : 0;
      if (new_value == 1 && last_buttons_[event.number] == 0)
      {
        button_toggle_states_[event.number] = button_toggle_states_[event.number] ? 0 : 1;
      }
      raw_buttons_[event.number] = new_value;
      last_buttons_[event.number] = new_value;
      dirty_ = true;
    }
  }

  void publishIfNeeded()
  {
    const ros::Time now = ros::Time::now();
    const bool repeat_due = autorepeat_ && (last_publish_time_.isZero() ||
                                            (now - last_publish_time_).toSec() >= (1.0 / publish_rate_));

    if (!dirty_ && !repeat_due)
    {
      maybePrintStatus(false);
      return;
    }

    rebuildJoyMessage();
    joy_msg_.header.stamp = now;
    joy_pub_.publish(joy_msg_);

    if (enable_rc_override_)
    {
      publishRcOverride();
    }

    maybePrintStatus(true);
    last_publish_time_ = now;
    dirty_ = false;
  }

  void rebuildJoyMessage()
  {
    joy_msg_.axes.clear();
    joy_msg_.axes.reserve(pub_axis_indices_.size());
    for (const int axis_index : pub_axis_indices_)
    {
      joy_msg_.axes.push_back(getAxis(axis_index));
    }

    joy_msg_.buttons.clear();
    joy_msg_.buttons.reserve(pub_button_indices_.size());
    for (const int button_index : pub_button_indices_)
    {
      joy_msg_.buttons.push_back(getButton(button_index));
    }
  }

  float getAxis(const int axis_index) const
  {
    if (axis_index < 0 || static_cast<size_t>(axis_index) >= raw_axes_.size())
    {
      return 0.0f;
    }
    return raw_axes_[axis_index];
  }

  int getButton(const int button_index) const
  {
    if (button_index < 0 || static_cast<size_t>(button_index) >= raw_buttons_.size())
    {
      return 0;
    }
    return raw_buttons_[button_index];
  }

  int getButtonToggle(const int button_index) const
  {
    if (button_index < 0 || static_cast<size_t>(button_index) >= button_toggle_states_.size())
    {
      return 0;
    }
    return button_toggle_states_[button_index];
  }

  void publishRcOverride()
  {
    mavros_msgs::OverrideRCIn rc_msg;
    for (size_t channel = 0; channel < kRcChannelCount; ++channel)
    {
      rc_values_[channel] = computeRcValue(channel);
      rc_msg.channels[channel] = rc_values_[channel];
    }

    rc_override_pub_.publish(rc_msg);
  }

  uint16_t computeRcValue(const size_t channel) const
  {
    const RcChannelConfig& config = rc_channels_[channel];
    if (config.type == RcChannelConfig::SourceType::Axis)
    {
      return axisToPwm(getAxis(config.source), config);
    }
    if (config.type == RcChannelConfig::SourceType::Button)
    {
      const int button_value = config.button_toggle ? getButtonToggle(config.source) : getButton(config.source);
      return button_value ? config.button_high : config.button_low;
    }
    return static_cast<uint16_t>(rc_default_);
  }

  uint16_t axisToPwm(const float axis_value, const RcChannelConfig& config) const
  {
    if (std::abs(config.in_max - config.in_min) < 1e-6)
    {
      return static_cast<uint16_t>(rc_default_);
    }

    const double clamped = clampDouble(axis_value, std::min(config.in_min, config.in_max),
                                       std::max(config.in_min, config.in_max));
    const double ratio = (clamped - config.in_min) / (config.in_max - config.in_min);
    const double pwm = static_cast<double>(config.out_min) + ratio * (config.out_max - config.out_min);
    return static_cast<uint16_t>(clampInt(static_cast<int>(std::round(pwm)), rc_min_, rc_max_));
  }

  void maybePrintStatus(const bool published)
  {
    if (!print_status_)
    {
      return;
    }

    const ros::Time now = ros::Time::now();
    if (!last_print_time_.isZero() && (now - last_print_time_).toSec() < (1.0 / print_rate_))
    {
      return;
    }

    printStatus(published);
    last_print_time_ = now;
  }

  void printStartupConfig() const
  {
    std::ostringstream out;
    out << "\n========== joystick_ros_bridge 配置 =========="
        << "\n设备: " << device_
        << "\nJoy 话题: " << pub_joy_topic_
        << "\nRC Override: " << (enable_rc_override_ ? "开启" : "关闭")
        << "\nRC 话题: " << rc_override_topic_
        << "\n发布频率: " << publish_rate_ << " Hz"
        << "\n终端刷新: " << (print_status_ ? "开启" : "关闭") << " @ " << print_rate_ << " Hz"
        << "\nJoy 发布轴: " << joinInts(pub_axis_indices_)
        << "\nJoy 发布按键: " << joinInts(pub_button_indices_)
        << "\n未映射 RC 默认值: " << rc_default_
        << "\n=============================================\n";
    ROS_INFO_STREAM(out.str());
  }

  void printStatus(const bool /*published*/) const
  {
    std::ostringstream out;
    out << "\033[2J\033[H";
    out << "┌──────────────── joystick_ros_bridge 遥控状态 ────────────────┐\n";
    out << "│ 设备: " << std::left << std::setw(54) << truncate(joystick_name_.empty() ? device_ : joystick_name_, 54)
        << "│\n";
    out << "│ Joy话题: " << std::left << std::setw(21) << pub_joy_topic_ << " MAVROS RC: "
        << std::setw(21) << (enable_rc_override_ ? rc_override_topic_ : "disabled") << "│\n";
    out << "│ 发布频率: " << std::right << std::setw(6) << std::fixed << std::setprecision(1) << publish_rate_
        << " Hz        终端刷新: " << std::setw(5) << print_rate_ << " Hz                 │\n";
    out << "├────────────────────── Joy 原始数据 raw ──────────────────────┤\n";
    out << formatRawJoy();
    out << "├────────────────────── Joy 话题输出数据 ──────────────────────┤\n";
    out << formatPublishedJoy();
    out << "├──────────────── MAVROS RC 输出 PWM（前 8 通道） ──────────────┤\n";
    out << formatRcPrimaryChannels();
    out << "└───────────────────────────────────────────────────────────────┘\n";
    std::cout << out.str() << std::flush;
  }

  std::string formatRawJoy() const
  {
    std::ostringstream out;
    out << "│ 轴:   " << std::left << std::setw(56) << compactRawAxes() << "│\n";
    out << "│ 按钮: " << std::left << std::setw(56) << compactRawButtons() << "│\n";
    return out.str();
  }

  std::string formatPublishedJoy() const
  {
    std::ostringstream out;
    out << "│ axes:    " << std::left << std::setw(53) << compactFloatVector(joy_msg_.axes) << "│\n";
    out << "│ buttons: " << std::left << std::setw(53) << compactIntVector(joy_msg_.buttons) << "│\n";
    return out.str();
  }

  std::string formatRcPrimaryChannels() const
  {
    static const std::array<const char*, 8> labels = {
        "横滚", "俯仰", "油门", "偏航", "X键", "Y键", "A键", "B键"};

    std::ostringstream out;
    for (size_t row = 0; row < 2; ++row)
    {
      out << "│ ";
      for (size_t col = 0; col < 4; ++col)
      {
        const size_t i = row * 4 + col;
        std::ostringstream cell;
        cell << "RC" << (i + 1) << "(" << labels[i] << "):" << rc_values_[i];
        out << std::left << std::setw(17) << cell.str();
      }
      out << "  │\n";
    }
    return out.str();
  }

  std::string compactRawAxes() const
  {
    std::ostringstream out;
    out << "[";
    for (size_t i = 0; i < raw_axes_.size(); ++i)
    {
      if (i > 0)
      {
        out << ", ";
      }
      out << "a" << i << "=" << std::fixed << std::setprecision(2) << raw_axes_[i];
    }
    out << "]";
    return truncate(out.str(), 56);
  }

  std::string compactRawButtons() const
  {
    std::ostringstream out;
    out << "[";
    for (size_t i = 0; i < raw_buttons_.size(); ++i)
    {
      if (i > 0)
      {
        out << ", ";
      }
      out << "b" << i << "=" << raw_buttons_[i];
    }
    out << "]";
    return truncate(out.str(), 56);
  }

  static std::string truncate(const std::string& text, const size_t width)
  {
    if (text.size() <= width)
    {
      return text;
    }
    if (width <= 3)
    {
      return text.substr(0, width);
    }
    return text.substr(0, width - 3) + "...";
  }

  static std::string joinInts(const std::vector<int>& values)
  {
    std::ostringstream out;
    out << "[";
    for (size_t i = 0; i < values.size(); ++i)
    {
      if (i > 0)
      {
        out << ", ";
      }
      out << values[i];
    }
    out << "]";
    return out.str();
  }

  static std::string compactFloatVector(const std::vector<float>& values)
  {
    std::ostringstream out;
    out << "[";
    for (size_t i = 0; i < values.size(); ++i)
    {
      if (i > 0)
      {
        out << ", ";
      }
      out << std::fixed << std::setprecision(2) << values[i];
    }
    out << "]";
    return truncate(out.str(), 55);
  }

  static std::string compactIntVector(const std::vector<int32_t>& values)
  {
    std::ostringstream out;
    out << "[";
    for (size_t i = 0; i < values.size(); ++i)
    {
      if (i > 0)
      {
        out << ", ";
      }
      out << values[i];
    }
    out << "]";
    return truncate(out.str(), 54);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher joy_pub_;
  ros::Publisher rc_override_pub_;

  std::string device_;
  std::string joystick_name_;
  std::string pub_joy_topic_;
  double publish_rate_ = 100.0;
  double deadzone_ = 0.05;
  bool autorepeat_ = true;
  int default_axes_ = 6;
  int default_buttons_ = 13;

  bool enable_rc_override_ = true;
  std::string rc_override_topic_;
  int rc_default_ = 1000;
  int rc_min_ = 1000;
  int rc_max_ = 2000;
  int rc_neutral_ = 1500;
  int rc_button_low_ = 1000;
  int rc_button_high_ = 2000;
  bool print_status_ = true;
  double print_rate_ = 10.0;

  std::vector<int> pub_axis_indices_;
  std::vector<int> pub_button_indices_;
  std::array<RcChannelConfig, kRcChannelCount> rc_channels_;
  std::array<uint16_t, kRcChannelCount> rc_values_{};

  int fd_ = -1;
  std::vector<float> raw_axes_;
  std::vector<int> raw_buttons_;
  std::vector<int> last_buttons_;
  std::vector<int> button_toggle_states_;
  sensor_msgs::Joy joy_msg_;
  ros::Time last_publish_time_;
  ros::Time last_print_time_;
  bool dirty_ = false;
};
}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joystick_ros_bridge");

  JoystickRosBridge bridge;
  bridge.spin();

  return 0;
}
