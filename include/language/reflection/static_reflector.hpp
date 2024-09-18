#pragma once

#include <string>
#include <tuple>

// Helper type for member reflection
template<typename T, typename Class>
struct Property {
    std::string_view name;
    T Class::*member_ptr;

    // Getter
    const T& get(const Class& obj) const {
        return obj.*member_ptr;
    }

    // Setter
    void set(Class& obj, const T& value) const {
        obj.*member_ptr = value;
    }
};

// Reflection macro for defining properties
#define REFLECT(...) \
    struct IsStaticReflected {};                          \
    static constexpr auto staticReflect() {               \
        return std::make_tuple(__VA_ARGS__);              \
    }

struct StaticReflect {
    // Function to set a property value
    template<typename T, typename ValueType>
    static void set_property(T& obj, std::string_view prop_name, const ValueType& value) {
        constexpr auto properties = T::staticReflect();
        std::apply([&](auto&&... props) {
            ((props.name == prop_name ? props.set(obj, value) : void()), ...);
        }, properties);
    }
};