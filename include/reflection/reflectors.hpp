#pragma once

#include <iostream>
#include <functional>
#include <type_traits>
#include <optional>
#include <sstream>
#include <map>
#include <format>

#include "meta_programming/type_list.hpp"
#include "meta_programming/type_traits/is_function.hpp"

#include "meta_programming/type_traits/is_vector.hpp"
#include "meta_programming/type_traits/is_map.hpp"

#include <nlohmann/json.hpp>
using json = nlohmann::json;

using SerializableTypes = MetaProgramming::TypeList<int, float, std::string>;

template<typename T>
using SerializableContainerTypes = MetaProgramming::TypeList<std::vector<T>>;

template<typename T>
struct CustomReflector {};

template<typename T>
concept CustomReflectImpl = requires(T t, std::string str) {
    {T::ReflectImpl::serialize(t)} -> std::same_as<std::string>;
    {T::ReflectImpl::deserialize(str)} -> std::same_as<T>;
};

template<typename T>
concept CustomReflectable = requires(T t, std::string str) {
    CustomReflector<T>::serializable(t);
    {CustomReflector<T>::deserializable(str)} -> std::same_as<T>;
};

template<typename T>
concept Serializable = CustomReflectImpl<T> || MetaProgramming::TypeListFunctions::IsAnyOf<SerializableTypes, std::decay_t<T>>::value;

struct DispatchedTypeBase {
    void* dataPtr;

    virtual std::string serialize() = 0;
    virtual void deserialize(void* target_str, const std::string& str) = 0;
};

template<typename T>
struct DispatchedType: public DispatchedTypeBase {
    DispatchedType(void* ptr) {
        dataPtr = ptr;
    }

    std::string serialize_single() {
        auto ptr = reinterpret_cast<T*>(dataPtr);
        if constexpr (CustomReflectImpl<T>) {
            return T::ReflectImpl::serialize(*ptr);
        } else {
            /**
             * serialization of builtin types
             */
            auto& value = *ptr;
            if constexpr (std::is_same_v<T, int>) {
                return std::to_string(value);
            } else if constexpr (std::is_same_v<T, double>) {
                std::ostringstream oss;
                oss << value;
                return oss.str();
            } else if constexpr (std::is_same_v<T, float>) {
                std::ostringstream oss;
                oss << value;
                return oss.str();
            } else if constexpr (std::is_same_v<T, std::string>) {
                return value;  // Simple quote for JSON-like serialization
            } else {
                static_assert(std::is_same_v<T, void>, "Unsupported type");
                return "";
            }
        }
    }

    std::optional<T> deserialize_single(const std::string& str) {
        auto ptr = reinterpret_cast<T*>(dataPtr);
        if constexpr (CustomReflectImpl<T>) {
            return T::ReflectImpl::deserialize(str);
        } else {
            if constexpr (std::is_same_v<T, int>) {
                /**
                 * deserialization of builtin types
                 */
                if constexpr (std::is_same_v<T, int>) {
                    return std::stoi(str);
                } else if constexpr (std::is_same_v<T, double>) {
                    return std::stod(str);
                } else if constexpr (std::is_same_v<T, float>) {
                    return std::stof(str);
                } else if constexpr (std::is_same_v<T, std::string>) {
                    return str;
                    throw std::runtime_error("Invalid string format");
                } else {
                    static_assert(std::is_same_v<T, void>, "Unsupported type");
                    return std::nullopt;
                }
            }

            return std::nullopt;
        }
    }

    std::string serialize_vector() {
        if constexpr (is_vector_v<T>) {
            auto& vec = *reinterpret_cast<T*>(dataPtr);
            using value_type = T::value_type;
            std::ostringstream oss;
            oss << "[";
            for (size_t i = 0; i < vec.size(); ++i) {
                if (i > 0) oss << ", ";
                oss << DispatchedType<value_type>(&vec[i]).serialize();
            }
            oss << "]";

            return oss.str();
        }
        return "";
    }

    std::string serialize_map() {
        if constexpr (is_map_v<T>) {
            auto &map = *reinterpret_cast<T *>(dataPtr);

            using K = T::key_type;
            using V = T::mapped_type;

            std::ostringstream oss;
            oss << "{";
            bool first = true;
            for (const auto &[key, value]: map) {
                if (!first) oss << ", ";
                oss << DispatchedType<K>((void*)&key).serialize() << ": " << DispatchedType<V>((void *)&value).serialize();
                first = false;
            }
            oss << "}";
            return oss.str();
        }
        return "";
    }

    std::optional<T> deserialize_vector(const std::string& str) {
        if constexpr (is_vector_v<T>) {
            using value_type = T::value_type;

            json j = json::parse(str);
            std::vector<value_type> vec;
            for (const auto &item: j) {
                vec.push_back(DispatchedType<value_type>(nullptr).deserialize_single(item.dump()).value());
            }

            return vec;
        } else {
            return std::nullopt;
        }
    }

    std::optional<T> deserialize_map(const std::string& str) {
        if constexpr (is_map_v<T>) {
            using K = T::key_type;
            using V = T::mapped_type;

            json j = json::parse(str);
            std::map<K, V> m;
            for (auto it = j.begin(); it != j.end(); ++it) {
                K key = DispatchedType<K>(nullptr).deserialize_single(it.key().dump());
                V value = DispatchedType<V>(nullptr).deserialize_single(it.value().dump());
                m[key] = value;
            }
            return m;
        } else {
            return std::nullopt;
        }
    }

public:
    std::string serialize() {
        if constexpr (Serializable<T>) {
            // T is serializable
            return serialize_single();
        } else {
            // T is vector of some serializable
            if constexpr (is_vector_v<T>) {
                if constexpr (Serializable<T::value_type>) {
                    return serialize_vector();
                } else {
                    throw std::runtime_error("value of std::vector is not serializable");
                }
            }

            // T is map of two serializable
            if constexpr (is_map_v<T>) {
                if constexpr (Serializable<T::key_type> && Serializable<T::mapped_type>) {
                    return serialize_map();
                } else {
                    throw std::runtime_error("value or key of std::map is not serializable");
                }
            }

            throw std::runtime_error(std::format("{} {}", typeid(T).name(), "is not serializable"));
        }
    }

    void deserialize(void* target_ptr, const std::string& str) {

        auto& target = *reinterpret_cast<T*>(target_ptr);

        if constexpr (Serializable<T>) {
             target = deserialize_single(str).value();
        } else {
            // T is vector of some serializable
            if constexpr (is_vector_v<T>) {
                if constexpr (Serializable<T::value_type>) {
                    target.clear();
                    target = deserialize_vector(str).value();
                } else {
                    throw std::runtime_error("value of std::vector is not serializable");
                }
            }

            // T is map of two serializable
            if constexpr (is_map_v<T>) {
                std::map<int, double> t;
                using K = T::key_type;
                using V = T::value_type;
                if constexpr (Serializable<K> && Serializable<V>) {
                    target.clear();
                    target = deserialize_map(str).value();
                } else {
                    throw std::runtime_error("value or key of std::vector is not serializable");
                }
            }

            throw std::runtime_error(std::format("{} {}", typeid(T).name(), "is not serializable"));
        }
    }
};

// Type-erased wrapper that holds type information for both data members and functions
struct TypeErasedValue {
    std::function<const std::type_info &()> type_info_func;
    std::function<void *(void)> get_ptr_func;
    std::function<void(void)> call_func; // Callable for member functions
    DispatchedTypeBase* dispatched;
    TypeErasedValue() = default;

    // Constructor for data members
    template <typename T> TypeErasedValue(T *value) {
        type_info_func = [value]() -> const std::type_info & { return typeid(T); };
        get_ptr_func = [value]() -> void * { return static_cast<void *>(value); };
        dispatched = new DispatchedType<T>(value);
        call_func = nullptr; // Not a function
    }

    // Constructor for member functions
    template <typename R, typename C> TypeErasedValue(R (C::*func)(void), C *obj) {
        type_info_func = [func]() -> const std::type_info & { return typeid(func); };
        get_ptr_func = nullptr; // Not a data member
        call_func = [func, obj]() { (obj->*func)(); };
    }

    const std::type_info &type() const {
        return type_info_func();
    }

    void *get() const {
        return get_ptr_func ? get_ptr_func() : nullptr;
    }

    void call() const {
        if (call_func) {
            call_func();
        } else {
            std::cerr << "Error: Not a callable function." << std::endl;
        }
    }
};

using ReflectDataType = std::unordered_map<std::string, TypeErasedValue>;

struct Reflection {
    template <typename T> static auto eraseValueType(T &t) {
        return TypeErasedValue(&t);
    }

    template <IsFunction F, typename T> static auto eraseFunctionType(F f, T *t) {
        return TypeErasedValue(f, t);
    }

    template<typename T>
    static std::string serialize(T& t) {
        return DispatchedType<T>(&t).serialize();
    }

    template<typename T>
    static T deserialize(const std::string& str) {
        T t;
        DispatchedType<T>(nullptr).deserialize(&t, str);
        return t;
    }
};

struct TypeDispatcher {
    DispatchedTypeBase* dispath(void * ptr, type_info typeInfo) {
        /**
         * if the type is builtin type in SerializableTypes
         */
        auto result = tryDispatch(ptr, typeInfo, SerializableTypes{});
        if (result != nullptr) {
            return result;
        }

        /**
         * if the type is the std::vector of builtin types in SerializableTypes
         */
        result = tryDispatch(ptr, typeInfo, SerializableTypes::monad<std::vector>{});
        if (result != nullptr) {
            return result;
        }
    }

private:
    template <typename TargetType, typename... RemainingTypes>
    DispatchedTypeBase* tryRecoverType(void* ptr, type_info& typeInfo, MetaProgramming::TypeList<TargetType, RemainingTypes...>) {
        // Attempt to compare the type_info
        if (typeInfo == typeid(TargetType)) {
            // If successful, return a new DispatchedType with the casted pointer
            return new DispatchedType<TargetType>(static_cast<TargetType*>(ptr));
        } else {
            // Otherwise, recursively try the next type in the list
            return tryRecoverType(ptr, typeInfo, MetaProgramming::TypeList<RemainingTypes...>{});
        }
    }

    // Base case: when the type list is empty, return nullptr
    DispatchedTypeBase* tryRecoverType(void* /*ptr*/, type_info& /*typeInfo*/, MetaProgramming::TypeList<>) {
        return nullptr; // No more types to check
    }

    // Helper function to initiate the type list traversal
    template <typename... Types>
    DispatchedTypeBase* tryDispatch(void* ptr, type_info &typeInfo, MetaProgramming::TypeList<Types...> typeList) {
        return tryRecoverType(ptr, typeInfo, typeList);
    }
};

// Reflection base class
class Reflectable {
  public:
    virtual ReflectDataType reflect() = 0;

    std::string serialization() {
        json j;
        for (auto& [name, info]: reflect()) {
            j[name] = info.dispatched->serialize();
        }
        return j.dump();
    }

    void deserialization(json j) {
        for (auto& [name, info]: reflect()) {
            info.dispatched->deserialize(info.get(), j[name].dump());
        }
    }
};