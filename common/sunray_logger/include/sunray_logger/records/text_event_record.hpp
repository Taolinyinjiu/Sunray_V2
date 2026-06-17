#pragma once

#include <cstdint>
#include <vector>

#include <sunray_logger/binary_writer.hpp>
#include <sunray_logger/ulog_field.hpp>

namespace sunray_logger {

struct TextEventRecord {
    uint64_t timestamp = 0;
    uint32_t event_id = 0;
    uint32_t level = 0;
    uint32_t module_hash = 0;
    uint32_t message_hash = 0;

    static const char* formatName() {
        return "sunray_text_event";
    }

    static std::vector<UlogField> fields() {
        return {
            {"uint64_t", "timestamp"},
            {"uint32_t", "event_id"},
            {"uint32_t", "level"},
            {"uint32_t", "module_hash"},
            {"uint32_t", "message_hash"},
        };
    }

    void serialize(BinaryWriter& writer) const {
        writer.write(timestamp);
        writer.write(event_id);
        writer.write(level);
        writer.write(module_hash);
        writer.write(message_hash);
    }
};

}  // namespace sunray_logger
