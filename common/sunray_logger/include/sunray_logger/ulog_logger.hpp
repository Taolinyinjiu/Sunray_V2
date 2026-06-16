#pragma once

#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>

#include <ros/console.h>

#include <sunray_logger/binary_writer.hpp>
#include <sunray_logger/time_utils.hpp>
#include <sunray_logger/ulog_record.hpp>
#include <sunray_logger/ulog_writer.hpp>

namespace sunray_logger {

template <typename RecordT>
class UlogTopic {
public:
    UlogTopic() = default;
    UlogTopic(uint16_t id, const std::string& name, const std::string& format_name)
        : id_(id), name_(name), format_name_(format_name), valid_(true) {}

    bool valid() const {
        return valid_;
    }

    uint16_t id() const {
        return id_;
    }

    const std::string& name() const {
        return name_;
    }

    const std::string& formatName() const {
        return format_name_;
    }

private:
    uint16_t id_ = 0;
    std::string name_;
    std::string format_name_;
    bool valid_ = false;
};

class UlogLogger {
public:
    UlogLogger() = default;
    ~UlogLogger() {
        close();
    }

    bool open(const std::string& path, const UlogOptions& options = UlogOptions()) {
        close();
        next_msg_id_ = 0;
        format_names_.clear();
        topic_names_.clear();
        return writer_.open(path, options, wallTimeNowToUs());
    }

    bool isOpen() const {
        return writer_.isOpen();
    }

    void flush() {
        writer_.flush();
    }

    void close() {
        writer_.close();
    }

    template <typename RecordT>
    UlogTopic<RecordT> advertise(const std::string& topic_name, uint8_t multi_id = 0) {
        validateRecordType<RecordT>();

        if (!writer_.isOpen()) {
            ROS_ERROR("sunray_logger: cannot advertise topic '%s' before opening a log file", topic_name.c_str());
            return UlogTopic<RecordT>();
        }

        if (writer_.hasCompletedHeader()) {
            ROS_ERROR("sunray_logger: cannot advertise topic '%s' after data writing started", topic_name.c_str());
            return UlogTopic<RecordT>();
        }

        if (topic_names_.count(topic_name) > 0U) {
            ROS_ERROR("sunray_logger: duplicate topic name '%s'", topic_name.c_str());
            return UlogTopic<RecordT>();
        }

        const std::string format_name = RecordT::formatName();
        if (format_names_.count(format_name) == 0U) {
            if (!writer_.writeFormat(format_name, RecordT::fields())) {
                ROS_ERROR("sunray_logger: failed to write format '%s': %s", format_name.c_str(),
                          writer_.lastError().c_str());
                return UlogTopic<RecordT>();
            }
            format_names_.insert(format_name);
        }

        const uint16_t msg_id = next_msg_id_++;
        if (!writer_.addLoggedMessage(msg_id, topic_name, format_name, multi_id)) {
            ROS_ERROR("sunray_logger: failed to add topic '%s': %s", topic_name.c_str(), writer_.lastError().c_str());
            return UlogTopic<RecordT>();
        }

        topic_names_.insert(topic_name);
        return UlogTopic<RecordT>(msg_id, topic_name, format_name);
    }

    template <typename RecordT>
    bool write(const UlogTopic<RecordT>& topic, const RecordT& record) {
        validateRecordType<RecordT>();

        if (!topic.valid()) {
            ROS_ERROR("sunray_logger: cannot write invalid ULog topic handle");
            return false;
        }

        if (!writer_.isOpen()) {
            ROS_ERROR("sunray_logger: cannot write topic '%s' because log file is not open", topic.name().c_str());
            return false;
        }

        if (!writer_.hasCompletedHeader() && !writer_.completeHeader()) {
            ROS_ERROR("sunray_logger: failed to complete ULog header before writing '%s': %s", topic.name().c_str(),
                      writer_.lastError().c_str());
            return false;
        }

        BinaryWriter payload_writer;
        record.serialize(payload_writer);

        const std::size_t expected_size = ulogRecordSize(RecordT::fields());
        if (payload_writer.size() != expected_size) {
            ROS_ERROR("sunray_logger: record '%s' serialized to %zu bytes, expected %zu bytes",
                      RecordT::formatName(), payload_writer.size(), expected_size);
            return false;
        }

        if (!writer_.writeData(topic.id(), payload_writer.data())) {
            ROS_ERROR("sunray_logger: failed to write topic '%s': %s", topic.name().c_str(), writer_.lastError().c_str());
            return false;
        }

        return true;
    }

    bool writeText(uint8_t log_level, uint64_t timestamp_us, const std::string& message) {
        if (!writer_.isOpen()) {
            return false;
        }
        if (!writer_.hasCompletedHeader() && !writer_.completeHeader()) {
            return false;
        }
        return writer_.writeTextMessage(log_level, timestamp_us, message);
    }

    std::string lastError() const {
        return writer_.lastError();
    }

private:
    UlogWriter writer_;
    uint16_t next_msg_id_ = 0;
    std::set<std::string> format_names_;
    std::set<std::string> topic_names_;
};

}  // namespace sunray_logger
