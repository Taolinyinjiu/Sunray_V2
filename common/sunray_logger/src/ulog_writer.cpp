#include <sunray_logger/ulog_writer.hpp>

#include <cerrno>
#include <cstring>
#include <sys/stat.h>
#include <sys/types.h>

#include <sunray_logger/binary_writer.hpp>
#include <sunray_logger/time_utils.hpp>

namespace sunray_logger {

namespace {

constexpr uint8_t kUlogMagic[] = {0x55, 0x4c, 0x6f, 0x67, 0x01, 0x12, 0x35};
constexpr uint8_t kUlogVersion = 1;

template <typename T>
void appendValue(std::vector<uint8_t>& payload, const T& value) {
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&value);
    payload.insert(payload.end(), bytes, bytes + sizeof(T));
}

}  // namespace

UlogWriter::UlogWriter() = default;

UlogWriter::~UlogWriter() {
    close();
}

bool UlogWriter::open(const std::string& path, const UlogOptions& options, uint64_t start_timestamp_us) {
    close();
    path_ = path;
    options_ = options;
    header_completed_ = false;
    last_error_.clear();

    if (path.empty()) {
        setError("empty log path");
        return false;
    }

    if (!createParentDirectories(path)) {
        setError("failed to create parent directory for log file '" + path + "'");
        return false;
    }

    stream_.open(path.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
    if (!stream_.is_open()) {
        setError("failed to open log file '" + path + "': " + std::strerror(errno));
        return false;
    }

    if (options_.buffer_size > 0U) {
        // std::ofstream does not expose a portable dynamic buffer resize API here.
        // The value is retained in options() for future async/buffered writer extensions.
    }

    if (!writeHeader(start_timestamp_us == 0U ? wallTimeNowToUs() : start_timestamp_us)) {
        close();
        return false;
    }

    if (!writeFlagBits()) {
        close();
        return false;
    }

    if (!options_.system_name.empty() && !writeInfo("sys_name", options_.system_name)) {
        close();
        return false;
    }

    return true;
}

bool UlogWriter::isOpen() const {
    return stream_.is_open();
}

void UlogWriter::flush() {
    if (stream_.is_open()) {
        stream_.flush();
    }
}

void UlogWriter::close() {
    if (stream_.is_open()) {
        stream_.flush();
        stream_.close();
    }
    header_completed_ = false;
}

bool UlogWriter::writeFormat(const std::string& format_name, const std::vector<UlogField>& fields) {
    if (!isOpen()) {
        setError("log file is not open");
        return false;
    }
    if (header_completed_) {
        setError("cannot write format after header is complete");
        return false;
    }
    if (format_name.empty()) {
        setError("empty format name");
        return false;
    }
    if (fields.empty()) {
        setError("format '" + format_name + "' has no fields");
        return false;
    }

    const std::string format = makeFormatString(format_name, fields);
    return writeMessage('F', format.data(), format.size());
}

bool UlogWriter::completeHeader() {
    if (!isOpen()) {
        setError("log file is not open");
        return false;
    }
    header_completed_ = true;
    return true;
}

bool UlogWriter::addLoggedMessage(uint16_t msg_id, const std::string& topic_name, const std::string& format_name,
                                  uint8_t multi_id) {
    if (!isOpen()) {
        setError("log file is not open");
        return false;
    }
    if (header_completed_) {
        setError("cannot add logged message after header is complete");
        return false;
    }
    if (topic_name.empty()) {
        setError("empty topic name");
        return false;
    }
    if (format_name.empty()) {
        setError("empty format name for topic '" + topic_name + "'");
        return false;
    }

    std::vector<uint8_t> payload;
    appendValue(payload, multi_id);
    appendValue(payload, msg_id);
    // ULog subscriptions reference the message format name. The public API
    // keeps topic_name as a module-side handle label.
    payload.insert(payload.end(), format_name.begin(), format_name.end());

    return writeMessage('A', payload);
}

bool UlogWriter::writeData(uint16_t msg_id, const std::vector<uint8_t>& payload) {
    if (!isOpen()) {
        setError("log file is not open");
        return false;
    }
    if (!header_completed_ && !completeHeader()) {
        return false;
    }

    std::vector<uint8_t> data_payload;
    appendValue(data_payload, msg_id);
    data_payload.insert(data_payload.end(), payload.begin(), payload.end());

    const bool ok = writeMessage('D', data_payload);
    if (ok && options_.auto_flush) {
        stream_.flush();
    }
    return ok;
}

bool UlogWriter::writeInfo(const std::string& key, const std::string& value) {
    if (!isOpen()) {
        setError("log file is not open");
        return false;
    }
    if (header_completed_) {
        setError("cannot write info after header is complete");
        return false;
    }
    if (key.empty()) {
        setError("empty info key");
        return false;
    }

    const std::string type_and_key = "char[" + std::to_string(value.size()) + "] " + key;
    if (type_and_key.size() > 255U) {
        setError("info key is too long: " + key);
        return false;
    }

    std::vector<uint8_t> payload;
    payload.push_back(static_cast<uint8_t>(type_and_key.size()));
    payload.insert(payload.end(), type_and_key.begin(), type_and_key.end());
    payload.insert(payload.end(), value.begin(), value.end());

    return writeMessage('I', payload);
}

bool UlogWriter::writeTextMessage(uint8_t log_level, uint64_t timestamp_us, const std::string& message) {
    if (!isOpen()) {
        setError("log file is not open");
        return false;
    }
    if (!header_completed_ && !completeHeader()) {
        return false;
    }

    std::vector<uint8_t> payload;
    appendValue(payload, log_level);
    appendValue(payload, timestamp_us);
    payload.insert(payload.end(), message.begin(), message.end());

    const bool ok = writeMessage('L', payload);
    if (ok && options_.auto_flush) {
        stream_.flush();
    }
    return ok;
}

bool UlogWriter::hasCompletedHeader() const {
    return header_completed_;
}

const std::string& UlogWriter::path() const {
    return path_;
}

const UlogOptions& UlogWriter::options() const {
    return options_;
}

std::string UlogWriter::lastError() const {
    return last_error_;
}

bool UlogWriter::writeHeader(uint64_t start_timestamp_us) {
    if (!writeRaw(kUlogMagic, sizeof(kUlogMagic))) {
        return false;
    }
    if (!writeRaw(&kUlogVersion, sizeof(kUlogVersion))) {
        return false;
    }
    return writeRaw(&start_timestamp_us, sizeof(start_timestamp_us));
}

bool UlogWriter::writeFlagBits() {
    std::vector<uint8_t> payload(8 + 8 + 3 * 8, 0);
    return writeMessage('B', payload);
}

bool UlogWriter::writeMessage(char type, const void* payload, std::size_t payload_size) {
    if (payload_size > UINT16_MAX) {
        setError("ULog message payload too large: " + std::to_string(payload_size));
        return false;
    }

    const uint16_t size = static_cast<uint16_t>(payload_size);
    if (!writeRaw(&size, sizeof(size))) {
        return false;
    }
    if (!writeRaw(&type, sizeof(type))) {
        return false;
    }
    if (payload_size > 0U && !writeRaw(payload, payload_size)) {
        return false;
    }
    return true;
}

bool UlogWriter::writeMessage(char type, const std::vector<uint8_t>& payload) {
    return writeMessage(type, payload.empty() ? nullptr : payload.data(), payload.size());
}

bool UlogWriter::writeRaw(const void* data, std::size_t size) {
    if (!stream_.is_open()) {
        setError("log file is not open");
        return false;
    }
    stream_.write(static_cast<const char*>(data), static_cast<std::streamsize>(size));
    if (!stream_) {
        setError("failed to write log file '" + path_ + "'");
        return false;
    }
    return true;
}

void UlogWriter::setError(const std::string& error) {
    last_error_ = error;
}

bool UlogWriter::createParentDirectories(const std::string& file_path) {
    const std::string::size_type slash = file_path.rfind('/');
    if (slash == std::string::npos || slash == 0U) {
        return true;
    }

    std::string current;
    const std::string parent = file_path.substr(0, slash);
    for (std::size_t i = 0; i < parent.size(); ++i) {
        current.push_back(parent[i]);
        if (parent[i] != '/' && i + 1 < parent.size()) {
            continue;
        }
        if (current.empty() || current == "/") {
            continue;
        }
        if (::mkdir(current.c_str(), 0755) != 0 && errno != EEXIST) {
            return false;
        }
    }

    if (::mkdir(parent.c_str(), 0755) != 0 && errno != EEXIST) {
        return false;
    }
    return true;
}

}  // namespace sunray_logger
