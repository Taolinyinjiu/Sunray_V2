#pragma once

#include <type_traits>
#include <vector>

#include <sunray_logger/binary_writer.hpp>
#include <sunray_logger/ulog_field.hpp>

namespace sunray_logger {

template <typename T>
class HasFormatName {
private:
    template <typename U>
    static auto test(int) -> decltype(U::formatName(), std::true_type());

    template <typename>
    static std::false_type test(...);

public:
    static const bool value = decltype(test<T>(0))::value;
};

template <typename T>
class HasFields {
private:
    template <typename U>
    static auto test(int) -> decltype(U::fields(), std::true_type());

    template <typename>
    static std::false_type test(...);

public:
    static const bool value = decltype(test<T>(0))::value;
};

template <typename T>
class HasSerialize {
private:
    template <typename U>
    static auto test(int) -> decltype(std::declval<const U>().serialize(std::declval<BinaryWriter&>()), std::true_type());

    template <typename>
    static std::false_type test(...);

public:
    static const bool value = decltype(test<T>(0))::value;
};

template <typename RecordT>
inline void validateRecordType() {
    static_assert(HasFormatName<RecordT>::value, "RecordT must provide static const char* formatName()");
    static_assert(HasFields<RecordT>::value, "RecordT must provide static std::vector<UlogField> fields()");
    static_assert(HasSerialize<RecordT>::value, "RecordT must provide void serialize(BinaryWriter&) const");
}

template <typename RecordT>
inline std::vector<uint8_t> serializeRecord(const RecordT& record) {
    validateRecordType<RecordT>();
    BinaryWriter writer;
    record.serialize(writer);
    return writer.data();
}

}  // namespace sunray_logger
