#pragma once

#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>

#include <ros/console.h>

#include <sunray_logger/binary_writer.hpp>
#include <sunray_logger/records/text_event_record.hpp>
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
        pending_subscriptions_.clear();
        subscriptions_written_ = false;
        text_event_topic_ = UlogTopic<TextEventRecord>();
        next_text_event_id_ = 1;
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
        pending_subscriptions_.push_back({msg_id, topic_name, format_name, multi_id});

        topic_names_.insert(topic_name);
        return UlogTopic<RecordT>(msg_id, topic_name, format_name);
    }

    UlogTopic<TextEventRecord> enableTextEventTopic(const std::string& topic_name = "sunray_text_event",
                                                    uint8_t multi_id = 0) {
        if (text_event_topic_.valid()) {
            return text_event_topic_;
        }

        text_event_topic_ = advertise<TextEventRecord>(topic_name, multi_id);
        return text_event_topic_;
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

        if (!ensureSubscriptionsWritten()) {
            ROS_ERROR("sunray_logger: failed to complete ULog subscriptions before writing '%s': %s", topic.name().c_str(),
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
        if (!ensureSubscriptionsWritten()) {
            return false;
        }
        return writer_.writeTextMessage(log_level, timestamp_us, message);
    }

    bool writeTextEvent(uint8_t log_level,
                        uint64_t timestamp_us,
                        uint32_t event_id,
                        uint32_t module_hash,
                        uint32_t message_hash) {
        if (!text_event_topic_.valid()) {
            return false;
        }

        TextEventRecord record;
        record.timestamp = timestamp_us;
        record.event_id = event_id;
        record.level = static_cast<uint32_t>(log_level);
        record.module_hash = module_hash;
        record.message_hash = message_hash;

        return write(text_event_topic_, record);
    }

    bool writeTextWithEvent(uint8_t log_level,
                            uint64_t timestamp_us,
                            const std::string& message,
                            uint32_t module_hash,
                            uint32_t message_hash,
                            uint32_t* event_id_out = nullptr) {
        const uint32_t event_id = next_text_event_id_++;
        if (event_id_out != nullptr) {
            *event_id_out = event_id;
        }

        const std::string event_message = "[E" + std::to_string(event_id) + "] " + message;
        const bool text_ok = writeText(log_level, timestamp_us, event_message);
        const bool event_ok = writeTextEvent(log_level, timestamp_us, event_id, module_hash, message_hash);

        return text_ok && (event_ok || !text_event_topic_.valid());
    }

    std::string lastError() const {
        return writer_.lastError();
    }

private:
    struct PendingSubscription {
        PendingSubscription(uint16_t id, const std::string& topic, const std::string& format, uint8_t multi)
            : msg_id(id), topic_name(topic), format_name(format), multi_id(multi) {}

        uint16_t msg_id = 0;
        std::string topic_name;
        std::string format_name;
        uint8_t multi_id = 0;
    };

    bool ensureSubscriptionsWritten() {
        if (subscriptions_written_) {
            return true;
        }

        for (const auto& subscription : pending_subscriptions_) {
            if (!writer_.addLoggedMessage(subscription.msg_id, subscription.topic_name, subscription.format_name,
                                          subscription.multi_id)) {
                return false;
            }
        }

        if (!writer_.completeHeader()) {
            return false;
        }

        subscriptions_written_ = true;
        return true;
    }

    UlogWriter writer_;
    uint16_t next_msg_id_ = 0;
    std::set<std::string> format_names_;
    std::set<std::string> topic_names_;
    std::vector<PendingSubscription> pending_subscriptions_;
    bool subscriptions_written_ = false;
    UlogTopic<TextEventRecord> text_event_topic_;
    uint32_t next_text_event_id_ = 1;
};

}  // namespace sunray_logger
