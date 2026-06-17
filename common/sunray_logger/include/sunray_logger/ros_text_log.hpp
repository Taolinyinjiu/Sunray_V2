#pragma once

#include <cstdarg>
#include <cstddef>
#include <cstdio>
#include <cstdint>
#include <string>
#include <vector>

#include <ros/console.h>
#include <ros/time.h>

#include <sunray_logger/time_utils.hpp>
#include <sunray_logger/ulog_logger.hpp>

namespace sunray_logger {

enum class TextLogLevel : uint8_t {
    Debug = 7,
    Info = 6,
    Warn = 4,
    Error = 3,
};

constexpr std::size_t kDefaultTextMessageMaxBytes = 1024U;
constexpr std::size_t kTextEventPrefixReservedBytes = 16U;

inline std::string truncateTextMessage(const std::string& message,
                                       std::size_t max_bytes = kDefaultTextMessageMaxBytes) {
    static const char kSuffix[] = "...[truncated]";
    const std::size_t suffix_size = sizeof(kSuffix) - 1U;

    if (message.size() <= max_bytes) {
        return message;
    }
    if (max_bytes <= suffix_size) {
        return message.substr(0, max_bytes);
    }

    return message.substr(0, max_bytes - suffix_size) + kSuffix;
}

inline std::string makeModuleTextMessage(const std::string& module, const std::string& message) {
    const std::size_t max_message_bytes = kDefaultTextMessageMaxBytes - kTextEventPrefixReservedBytes;
    if (module.empty()) {
        return truncateTextMessage(message, max_message_bytes);
    }

    return truncateTextMessage("[" + module + "] " + message, max_message_bytes);
}

inline uint32_t fnv1a32(const std::string& value) {
    uint32_t hash = 2166136261U;
    for (const char ch : value) {
        hash ^= static_cast<uint8_t>(ch);
        hash *= 16777619U;
    }
    return hash;
}

inline bool writeModuleText(UlogLogger* logger,
                            TextLogLevel level,
                            const std::string& module,
                            uint64_t timestamp_us,
                            const std::string& message) {
    if (logger == nullptr || !logger->isOpen()) {
        return false;
    }

    const std::string text = makeModuleTextMessage(module, message);
    return logger->writeTextWithEvent(static_cast<uint8_t>(level),
                                      timestamp_us,
                                      text,
                                      fnv1a32(module),
                                      fnv1a32(text));
}

inline bool writeModuleText(UlogLogger& logger,
                            TextLogLevel level,
                            const std::string& module,
                            uint64_t timestamp_us,
                            const std::string& message) {
    return writeModuleText(&logger, level, module, timestamp_us, message);
}

namespace detail {

inline std::string vformatText(const char* format, va_list args) {
    if (format == nullptr) {
        return std::string();
    }

    va_list size_args;
    va_copy(size_args, args);
    const int required_size = std::vsnprintf(nullptr, 0, format, size_args);
    va_end(size_args);

    if (required_size < 0) {
        return std::string(format);
    }

    std::vector<char> buffer(static_cast<std::size_t>(required_size) + 1U, '\0');
    const int written_size = std::vsnprintf(buffer.data(), buffer.size(), format, args);
    if (written_size < 0) {
        return std::string(format);
    }

    return std::string(buffer.data(), static_cast<std::size_t>(written_size));
}

inline void writeRosConsole(TextLogLevel level, const std::string& message) {
    switch (level) {
        case TextLogLevel::Debug:
            ROS_DEBUG("%s", message.c_str());
            break;
        case TextLogLevel::Info:
            ROS_INFO("%s", message.c_str());
            break;
        case TextLogLevel::Warn:
            ROS_WARN("%s", message.c_str());
            break;
        case TextLogLevel::Error:
            ROS_ERROR("%s", message.c_str());
            break;
    }
}

inline bool writeRosAndModuleTextV(UlogLogger* logger,
                                   TextLogLevel level,
                                   const std::string& module,
                                   const char* format,
                                   va_list args) {
    std::string formatted;
    try {
        formatted = vformatText(format, args);
    } catch (...) {
        formatted = format == nullptr ? std::string() : std::string(format);
    }

    const std::string text = makeModuleTextMessage(module, formatted);
    writeRosConsole(level, text);

    if (logger == nullptr || !logger->isOpen()) {
        return false;
    }

    return logger->writeTextWithEvent(static_cast<uint8_t>(level),
                                      rosTimeToUs(ros::Time::now()),
                                      text,
                                      fnv1a32(module),
                                      fnv1a32(text));
}

inline bool writeRosAndModuleText(UlogLogger* logger,
                                  TextLogLevel level,
                                  const std::string& module,
                                  const char* format,
                                  ...) {
    va_list args;
    va_start(args, format);
    const bool ok = writeRosAndModuleTextV(logger, level, module, format, args);
    va_end(args);
    return ok;
}

inline bool writeRosAndModuleText(UlogLogger& logger,
                                  TextLogLevel level,
                                  const std::string& module,
                                  const char* format,
                                  ...) {
    va_list args;
    va_start(args, format);
    const bool ok = writeRosAndModuleTextV(&logger, level, module, format, args);
    va_end(args);
    return ok;
}

}  // namespace detail

}  // namespace sunray_logger

#define SUNRAY_ULOG_DEBUG(logger, module, format, ...)                                                    \
    do {                                                                                                  \
        (void)::sunray_logger::detail::writeRosAndModuleText(                                             \
            (logger), ::sunray_logger::TextLogLevel::Debug, (module), (format), ##__VA_ARGS__);           \
    } while (0)

#define SUNRAY_ULOG_INFO(logger, module, format, ...)                                                     \
    do {                                                                                                  \
        (void)::sunray_logger::detail::writeRosAndModuleText(                                             \
            (logger), ::sunray_logger::TextLogLevel::Info, (module), (format), ##__VA_ARGS__);            \
    } while (0)

#define SUNRAY_ULOG_WARN(logger, module, format, ...)                                                     \
    do {                                                                                                  \
        (void)::sunray_logger::detail::writeRosAndModuleText(                                             \
            (logger), ::sunray_logger::TextLogLevel::Warn, (module), (format), ##__VA_ARGS__);            \
    } while (0)

#define SUNRAY_ULOG_ERROR(logger, module, format, ...)                                                    \
    do {                                                                                                  \
        (void)::sunray_logger::detail::writeRosAndModuleText(                                             \
            (logger), ::sunray_logger::TextLogLevel::Error, (module), (format), ##__VA_ARGS__);           \
    } while (0)
