#pragma once

#include <string>
#include <tuple>
#include <variant>

#include "custom_serizalization.hpp"
#include "nlohmann/json.hpp"

#include "language/template/string_literal.hpp"

template <typename T>
concept Streamable = requires(std::ostream &os, T val) {
    { os << val } -> std::same_as<std::ostream &>;
};

template <typename T>
concept IsStaticReflectedType = requires { typename T::IsStaticReflected; };

// Helper type for member reflection
template <typename T, typename Class> struct Property {
    using value_type = T;
    using PropertyTrait = void;

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

template <typename Ret, typename Class, typename... Args> struct Method {
    using return_type = Ret;
    using args_type = std::tuple<Args...>;
    using method_type = Ret (Class::*)(Args...);
    using const_method_type = Ret (Class::*)(Args...) const;

    using MethodTrait = void;

    std::string_view name;
    std::variant<method_type, const_method_type> method_ptr;

    // Call (non-const or const overload)
    template <typename... CallArgs> Ret call(Class &obj, CallArgs &&...args) const {
        return std::visit([&](auto ptr) { return (obj.*ptr)(std::forward<CallArgs>(args)...); }, method_ptr);
    }
};

template <typename T>
concept IsProperty = requires { typename T::PropertyTrait; };
template <typename T>
concept IsMethod = requires { typename T::MethodTrait; };

#define PROPERTY(name, ptr)                                                                                            \
    Property {                                                                                                         \
        #name, ptr                                                                                                     \
    }
#define METHOD(name, ptr)                                                                                              \
    Method {                                                                                                           \
        #name, ptr                                                                                                     \
    }

// Reflection macro for defining properties
#define REFLECT(...)                                                                                                   \
    struct IsStaticReflected {};                                                                                       \
    static constexpr auto staticReflect() {                                                                            \
        return std::make_tuple(__VA_ARGS__);                                                                           \
    }

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

    template <typename T, typename... Args>
    static std::optional<std::any> call(T &obj, std::string_view method_name, Args &&...args) {
        constexpr auto members = T::staticReflect();

        std::optional<std::any> result = std::nullopt;

        bool found = false;

        std::apply(
            [&](auto &&...member) {
                (..., ([&] {
                     using MemberType = std::decay_t<decltype(member)>;
                     if constexpr (IsMethod<MemberType>) {
                         if (member.name == method_name && !found) {
                             if constexpr (not std::same_as<typename MemberType::return_type, void>) {
                                 result = member.call(obj, std::forward<Args>(args)...);
                             } else {
                                 member.call(obj, std::forward<Args>(args)...);
                             }
                             found = true;
                         }
                     }
                 }()));
            },
            members);

        if (!found) {
            throw std::runtime_error("Method not found: " + std::string(method_name));
        }

        return result;
    }

    // Serialize
    template <typename T> static nlohmann::json serialization(const T &obj) {
        constexpr auto properties = T::staticReflect();
        nlohmann::json json;

        std::apply(
            [&](auto &&...props) {
                (..., ([&] {
                     if constexpr (IsProperty<std::decay_t<decltype(props)>>) {
                         json[props.name] = serialize_field(props.get(obj));
                     }
                 }()));
            },
            properties);

        return json;
    }

    // Deserialize
    template <typename T> static void deserialization(T &obj, const nlohmann::json &json) {
        constexpr auto properties = T::staticReflect();

        std::apply(
            [&](auto &&...props) {
                (..., (
                          [&] {
                              if constexpr (IsProperty<std::decay_t<decltype(props)>>) {
                                  if (json.contains(props.name)) {
                                      props.set(obj,
                                                deserialize_field<typename std::decay_t<decltype(props)>::value_type>(
                                                    json[props.name]));
                                  }
                              }
                          }(),
                          void()));
            },
            properties);
    }

  private:
    template <typename FieldType> static nlohmann::json serialize_field(const FieldType &field) {
        if constexpr (Streamable<FieldType>) {
            return field;
        } else if constexpr (StdVector<FieldType>) {
            return serialize_vector(field);
        } else {
            return custom::custom_serialize(field);
        }
    }

    // Handle std::vector<T>
    template <typename T> static nlohmann::json serialize_vector(const std::vector<T> &vec) {
        nlohmann::json array = nlohmann::json::array();
        for (const auto &item : vec) {
            if constexpr (IsStaticReflectedType<T>) {
                array.push_back(serialization(item));
            } else {
                array.push_back(serialize_field(item));
            }
        }
        return array;
    }

    template <typename FieldType> static FieldType deserialize_field(const nlohmann::json &json) {
        if constexpr (Streamable<FieldType>) {
            return json.get<FieldType>();
        } else if constexpr (StdVector<FieldType>) {
            return deserialize_vector<FieldType>(json);
        } else {
            return custom::custom_deserialize(json, std::type_identity<FieldType>{});
        }
    }

    // Handle std::vector<T>
    template <typename VecType> static VecType deserialize_vector(const nlohmann::json &json) {
        using ElementType = typename VecType::value_type;
        VecType vec;

        for (const auto &item : json) {
            if constexpr (IsStaticReflectedType<ElementType>) {
                ElementType element;
                deserialization(element, item);
                vec.push_back(std::move(element));
            } else {
                vec.push_back(deserialize_field<ElementType>(item));
            }
        }

        return vec;
    }
};