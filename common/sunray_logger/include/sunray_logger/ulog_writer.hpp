#pragma once

#include <cstdint>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include <sunray_logger/ulog_field.hpp>

namespace sunray_logger {

struct UlogOptions {
    std::string system_name = "sunray";
    bool auto_flush = false;
    std::size_t buffer_size = 1024 * 1024;
};

class UlogWriter {
public:
    UlogWriter();
    ~UlogWriter();

    bool open(const std::string& path, const UlogOptions& options = UlogOptions(), uint64_t start_timestamp_us = 0);
    bool isOpen() const;
    void flush();
    void close();

    bool writeFormat(const std::string& format_name, const std::vector<UlogField>& fields);
    bool completeHeader();
    bool addLoggedMessage(uint16_t msg_id, const std::string& topic_name, const std::string& format_name,
                          uint8_t multi_id = 0);
    bool writeData(uint16_t msg_id, const std::vector<uint8_t>& payload);
    bool writeInfo(const std::string& key, const std::string& value);
    bool writeTextMessage(uint8_t log_level, uint64_t timestamp_us, const std::string& message);

    bool hasCompletedHeader() const;
    const std::string& path() const;
    const UlogOptions& options() const;
    std::string lastError() const;

private:
    bool writeHeader(uint64_t start_timestamp_us);
    bool writeFlagBits();
    bool writeMessage(char type, const void* payload, std::size_t payload_size);
    bool writeMessage(char type, const std::vector<uint8_t>& payload);
    bool writeRaw(const void* data, std::size_t size);
    void setError(const std::string& error);
    static bool createParentDirectories(const std::string& file_path);

    std::ofstream stream_;
    std::string path_;
    UlogOptions options_;
    bool header_completed_ = false;
    std::string last_error_;
};

}  // namespace sunray_logger
