#pragma once

#include <string>
#include <tuple>

#include "custom_serizalization.hpp"
#include "nlohmann/json.hpp"

template <typename T>
concept Streamable = requires(std::ostream& os, T val) {
    { os << val } -> std::same_as<std::ostream&>;
};

// Helper type for member reflection
template <typename T, typename Class> struct Property {
    std::string_view name;
    T Class::*member_ptr;

    // Getter
    const T &get(const Class &obj) const {
        return obj.*member_ptr;
    }

    // Setter
    void set(Class &obj, const T &value) const {
        obj.*member_ptr = value;
    }
};

// Reflection macro for defining properties
#define REFLECT(...)                                                                                                   \
    struct IsStaticReflected {};                                                                                       \
    static constexpr auto staticReflect() {                                                                            \
        return std::make_tuple(__VA_ARGS__);                                                                           \
    }

template <size_t N> struct StringLiteral {
    constexpr StringLiteral(const char (&str)[N]) {
        std::copy_n(str, N, value);
    }
    char value[N];
};

template <typename T>
concept StdVector = requires {
    typename T::value_type;
    requires std::same_as<T, std::vector<typename T::value_type, typename T::allocator_type>>;
};

struct StaticReflect {
    // Function to set a property value
    template <typename T, typename ValueType>
    static void set_property(T &obj, std::string_view prop_name, const ValueType &value) {
        constexpr auto properties = T::staticReflect();
        std::apply([&](auto &&...props) { ((props.name == prop_name ? props.set(obj, value) : void()), ...); },
                   properties);
    }

    template <typename T, StringLiteral Name> static constexpr bool HasField() {
        constexpr auto properties = T::staticReflect();
        return std::apply([](auto &&...props) { return ((props.name == Name.value) || ...); }, properties);
    }

    // Serialize
    template <typename T>
    static nlohmann::json serialization(const T &obj) {
        constexpr auto properties = T::staticReflect();
        nlohmann::json json;

        std::apply([&](auto &&...props) {
            (..., (json[props.name] = serialize_field(props.get(obj))));
        }, properties);

        return json;
    }

private:
    template <typename FieldType>
    static nlohmann::json serialize_field(const FieldType& field) {
        if constexpr (Streamable<FieldType>) {
            return field;
        } else if constexpr (StdVector<FieldType>) {
            return serialize_vector(field);
        } else {
            return custom::custom_serialize(field);
        }
    }

    // Handle std::vector<T>
    template <typename T>
    static nlohmann::json serialize_vector(const std::vector<T>& vec) {
        nlohmann::json array = nlohmann::json::array();
        for (const auto& item : vec) {
            array.push_back(serialize_field(item));
        }
        return array;
    }
};