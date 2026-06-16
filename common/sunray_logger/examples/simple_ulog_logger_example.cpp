#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <sunray_logger/records/odom_record.hpp>
#include <sunray_logger/ulog_logger.hpp>

// 这个示例演示两类日志数据的写入：
// 1. common/sunray_logger 已经提供的基础类型：sunray_logger::OdomRecord
// 2. 业务模块自己定义的复杂类型：ControlCmdRecord
//
// 实际模块接入时，通常把类似 ControlCmdRecord 的结构体放在模块自己的 include 目录中，
// 例如 sunray_uav_control/include/sunray_uav_control/log_records/control_cmd_record.hpp。
// common 只保存稳定通用类型，模块私有状态不要放进 common。
struct ControlCmdRecord {
    // ULog/PX4 风格的时序数据通常要求每条记录带 timestamp 字段。
    // 这里统一使用 uint64_t，单位为 microseconds，便于离线工具按时间轴绘图。
    uint64_t timestamp = 0;

    // 期望位置和期望速度，数组字段在 fields() 中写成 float[3]。
    float position_sp[3] = {0.0f, 0.0f, 0.0f};
    float velocity_sp[3] = {0.0f, 0.0f, 0.0f};

    // 期望偏航角，单位由业务模块自己约定；本示例按 rad 理解。
    float yaw_sp = 0.0f;

    // formatName() 是写入 ULog format 区域的消息类型名。
    // 同一个进程中，不同 record 的 formatName 应保持唯一且稳定。
    static const char* formatName() {
        return "sunray_control_cmd";
    }

    // fields() 描述 record 的二进制字段布局。
    // 字段顺序必须和 serialize() 中写入顺序完全一致，否则离线解析时字段会错位。
    static std::vector<sunray_logger::UlogField> fields() {
        return {
            {"uint64_t", "timestamp"},
            {"float[3]", "position_sp"},
            {"float[3]", "velocity_sp"},
            {"float", "yaw_sp"},
        };
    }

    // serialize() 把当前 record 按 fields() 声明的顺序写成二进制 payload。
    // 这里不要写字段名，只写字段值；字段名已经由 fields() 写入 ULog format。
    void serialize(sunray_logger::BinaryWriter& writer) const {
        writer.write(timestamp);
        writer.writeArray(position_sp, 3);
        writer.writeArray(velocity_sp, 3);
        writer.write(yaw_sp);
    }
};

int main(int argc, char** argv) {
    // ros::init 只初始化 ROS 客户端库。这个示例不创建 NodeHandle，也不连接 ROS master，
    // 因此可以作为普通可执行文件运行，用于快速验证 ULog 写文件流程。
    ros::init(argc, argv, "simple_ulog_logger_example");

    // standalone 场景下使用 ros::Time::now() 前需要显式初始化 ROS time。
    // 如果你的模块本来就创建了 ros::NodeHandle，这一行通常不是必须的。
    ros::Time::init();

    // 默认输出到 /tmp，也允许通过命令行第一个参数覆盖：
    //   devel/lib/sunray_logger/simple_ulog_logger_example /tmp/test.ulg
    std::string log_path = "/tmp/sunray_logger_example.ulg";
    if (argc > 1) {
        log_path = argv[1];
    }

    // UlogOptions 用于配置日志文件的公共信息和刷新策略。
    sunray_logger::UlogOptions options;

    // system_name 会写入 ULog info 区域，便于后续判断日志来源。
    options.system_name = "sunray_logger_example";

    // 示例程序数据量很小，开启 auto_flush 方便运行后立即看到完整文件。
    // 高频控制循环中建议关闭 auto_flush，在合适时机手动 flush，减少 IO 开销。
    options.auto_flush = true;

    // UlogLogger 是业务模块推荐使用的高层接口。
    // 典型调用顺序是：open -> advertise -> write -> flush/close。
    sunray_logger::UlogLogger logger;
    if (!logger.open(log_path, options)) {
        ROS_ERROR("failed to open ULog file '%s': %s", log_path.c_str(), logger.lastError().c_str());
        return 1;
    }

    // advertise<RecordT>() 会写入 RecordT 的 format 信息，并创建一个 typed topic handle。
    // 第一次 write() 之后 ULog header 会结束，之后不能再 advertise 新 topic；
    // 所以所有需要记录的 topic 应在开始写数据前统一注册。
    const auto odom_topic = logger.advertise<sunray_logger::OdomRecord>("odom");
    const auto control_topic = logger.advertise<ControlCmdRecord>("control_cmd");
    if (!odom_topic.valid() || !control_topic.valid()) {
        ROS_ERROR("failed to advertise example ULog topics");
        return 1;
    }

    for (int i = 0; i < 20; ++i) {
        // 构造一段简单轨迹数据，模拟模块循环中每个周期的状态快照。
        const float t = static_cast<float>(i) * 0.05f;

        // 记录时间统一转换成微秒整数。
        // 真实传感器/里程计数据优先使用 msg.header.stamp；
        // 模块内部事件或控制循环快照可以使用 ros::Time::now()。
        const uint64_t timestamp = sunray_logger::rosTimeToUs(ros::Time::now());

        // 写入 common 提供的基础里程计记录。
        sunray_logger::OdomRecord odom;
        odom.timestamp = timestamp;
        odom.position[0] = t;
        odom.position[1] = std::sin(t);
        odom.position[2] = 1.0f;
        odom.velocity[0] = 1.0f;
        odom.velocity[1] = std::cos(t);
        odom.velocity[2] = 0.0f;
        odom.orientation_q[0] = 1.0f;
        odom.orientation_q[1] = 0.0f;
        odom.orientation_q[2] = 0.0f;
        odom.orientation_q[3] = 0.0f;

        // 写入模块自定义控制指令记录。
        ControlCmdRecord cmd;
        cmd.timestamp = timestamp;
        cmd.position_sp[0] = t + 0.1f;
        cmd.position_sp[1] = std::sin(t + 0.1f);
        cmd.position_sp[2] = 1.2f;
        cmd.velocity_sp[0] = 1.0f;
        cmd.velocity_sp[1] = std::cos(t + 0.1f);
        cmd.velocity_sp[2] = 0.0f;
        cmd.yaw_sp = 0.1f * t;

        // write() 会检查 topic 是否有效，并检查 serialize() 的字节数是否和 fields() 匹配。
        // 如果 fields() 和 serialize() 不一致，这里会返回 false。
        if (!logger.write(odom_topic, odom) || !logger.write(control_topic, cmd)) {
            ROS_ERROR("failed to write example ULog data: %s", logger.lastError().c_str());
            return 1;
        }

        // 模拟 100 Hz 左右的写入节奏。真实模块中通常在控制循环或 ROS callback 中写。
        ros::Duration(0.01).sleep();
    }

    // 程序结束前主动 flush/close，确保数据落盘。
    // 析构函数也会 close，但显式关闭能让失败位置更清晰。
    logger.flush();
    logger.close();

    ROS_INFO("wrote example ULog file: %s", log_path.c_str());
    return 0;
}
