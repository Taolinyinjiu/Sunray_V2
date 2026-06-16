#pragma once

#include <cstdint>
#include <cstring>
#include <string>
#include <type_traits>
#include <vector>

namespace sunray_logger {

class BinaryWriter {
public:
    BinaryWriter() = default;

    template <typename T>
    typename std::enable_if<std::is_arithmetic<T>::value || std::is_enum<T>::value, void>::type write(const T& value) {
        const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&value);
        buffer_.insert(buffer_.end(), bytes, bytes + sizeof(T));
    }

    template <typename T>
    void writeArray(const T* values, std::size_t count) {
        for (std::size_t i = 0; i < count; ++i) {
            write(values[i]);
        }
    }

    void writeBytes(const void* data, std::size_t size) {
        const uint8_t* bytes = static_cast<const uint8_t*>(data);
        buffer_.insert(buffer_.end(), bytes, bytes + size);
    }

    void writeStringBytes(const std::string& value) {
        writeBytes(value.data(), value.size());
    }

    const std::vector<uint8_t>& data() const {
        return buffer_;
    }

    std::vector<uint8_t>& data() {
        return buffer_;
    }

    std::size_t size() const {
        return buffer_.size();
    }

    bool empty() const {
        return buffer_.empty();
    }

    void clear() {
        buffer_.clear();
    }

private:
    std::vector<uint8_t> buffer_;
};

}  // namespace sunray_logger
