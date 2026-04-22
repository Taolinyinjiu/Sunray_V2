#pragma once

#include "sunray_log.hpp"
#include <chrono>
#include <ctime>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

enum class SunrayPanelSeverity {
    INFO,
    WARN,
    ERROR
};

constexpr const char* kSunrayAnsiReset = "\033[0m";
constexpr const char* kSunrayAnsiLightCyan = "\033[1;36m";
constexpr const char* kSunrayAnsiGreen = "\033[32m";
constexpr const char* kSunrayAnsiRed = "\033[31m";

inline std::string sunray_colorize_console_text(const std::string& text, const char* color) {
    return std::string(color) + text + kSunrayAnsiReset;
}

inline spdlog::level::level_enum sunray_panel_level(SunrayPanelSeverity severity) {
    switch (severity) {
    case SunrayPanelSeverity::INFO:
        return spdlog::level::info;
    case SunrayPanelSeverity::WARN:
        return spdlog::level::warn;
    case SunrayPanelSeverity::ERROR:
        return spdlog::level::err;
    }
    return spdlog::level::info;
}

inline void sunray_write_panel_log(const std::shared_ptr<spdlog::logger>& logger,
                                   SunrayPanelSeverity severity,
                                   const std::string& plain_text,
                                   const std::string& console_text) {
    if (!logger) {
        return;
    }

    const spdlog::level::level_enum level = sunray_panel_level(severity);
    auto log_to_sink = [&](const spdlog::sink_ptr& sink, const std::string& text) {
        if (!sink || !sink->should_log(level)) {
            return;
        }
        const spdlog::string_view_t payload(text.data(), text.size());
        const spdlog::details::log_msg msg(logger->name(), level, payload);
        sink->log(msg);
        sink->flush();
    };

    auto& sinks = logger->sinks();
    if (!sinks.empty()) {
        log_to_sink(sinks.front(), console_text);
    }
    for (std::size_t i = 1; i < sinks.size(); ++i) {
        log_to_sink(sinks[i], plain_text);
    }
}

inline std::string sunray_parent_directory(const std::string& path) {
    const std::size_t pos = path.find_last_of('/');
    return (pos == std::string::npos) ? "" : path.substr(0, pos);
}

inline std::string sunray_make_startup_timestamp() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm tm_now;
    localtime_r(&now_time, &tm_now);

    const auto milliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    std::ostringstream oss;
    oss << std::put_time(&tm_now, "%Y%m%d_%H%M%S") << "_" << std::setfill('0') << std::setw(3)
        << milliseconds.count();
    return oss.str();
}
