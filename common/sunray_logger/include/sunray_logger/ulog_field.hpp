#pragma once

#include <cstddef>
#include <cstdint>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace sunray_logger {

struct UlogField {
    std::string type;
    std::string name;

    UlogField() = default;
    UlogField(const std::string& field_type, const std::string& field_name) : type(field_type), name(field_name) {}
};

inline std::string makeFormatString(const std::string& format_name, const std::vector<UlogField>& fields) {
    std::ostringstream stream;
    stream << format_name << ":";

    for (std::size_t i = 0; i < fields.size(); ++i) {
        if (i > 0) {
            stream << ";";
        }
        stream << fields[i].type << " " << fields[i].name;
    }

    return stream.str();
}

inline std::size_t scalarUlogTypeSize(const std::string& scalar_type) {
    if (scalar_type == "int8_t" || scalar_type == "uint8_t" || scalar_type == "char" || scalar_type == "bool") {
        return 1;
    }
    if (scalar_type == "int16_t" || scalar_type == "uint16_t") {
        return 2;
    }
    if (scalar_type == "int32_t" || scalar_type == "uint32_t" || scalar_type == "float") {
        return 4;
    }
    if (scalar_type == "int64_t" || scalar_type == "uint64_t" || scalar_type == "double") {
        return 8;
    }

    throw std::invalid_argument("unsupported ULog scalar type: " + scalar_type);
}

inline std::size_t ulogTypeSize(const std::string& type) {
    const std::string::size_type array_start = type.find('[');
    if (array_start == std::string::npos) {
        return scalarUlogTypeSize(type);
    }

    const std::string::size_type array_end = type.find(']', array_start + 1);
    if (array_end == std::string::npos) {
        throw std::invalid_argument("invalid ULog array type: " + type);
    }

    const std::string scalar_type = type.substr(0, array_start);
    const std::string count_text = type.substr(array_start + 1, array_end - array_start - 1);
    const std::size_t count = static_cast<std::size_t>(std::stoul(count_text));
    return scalarUlogTypeSize(scalar_type) * count;
}

inline std::size_t ulogRecordSize(const std::vector<UlogField>& fields) {
    std::size_t size = 0;
    for (const auto& field : fields) {
        size += ulogTypeSize(field.type);
    }
    return size;
}

}  // namespace sunray_logger
