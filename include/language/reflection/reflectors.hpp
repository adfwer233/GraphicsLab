#pragma once

#include <format>
#include <functional>
#include <iostream>
#include <map>
#include <optional>
#include <sstream>
#include <type_traits>
#include <typeinfo>

#include "language/meta_programming/type_list.hpp"
#include "language/meta_programming/type_traits/is_function.hpp"

#include "language/meta_programming/type_traits/is_map.hpp"
#include "language/meta_programming/type_traits/is_vector.hpp"

#include "nlohmann/json.hpp"

#include "builtin_reflectors.hpp"
#include "function_meta.hpp"

struct Reflectable;

template <typename T>
concept ReflectableDerived = std::is_base_of_v<Reflectable, T>;

template <typename... Args, std::size_t... I>
auto tupleFromAnyVector(const std::vector<std::any> &args, std::index_sequence<I...>) {
    return std::make_tuple(std::any_cast<Args>(args[I])...);
}

// General function to wrap member function calls dynamically
template <typename T, typename... Args> auto wrapMethod(T *obj, void (T::*method)(Args...)) {
    return [method, obj](const std::vector<std::any> &args) {
        if (args.size() != sizeof...(Args)) {
            throw std::runtime_error("Argument count mismatch!");
        }
        auto tup = tupleFromAnyVector<Args...>(args, std::index_sequence_for<Args...>{});
        std::apply([obj, method](Args... unpackedArgs) { (obj->*method)(unpackedArgs...); }, tup);
    };
}

// Type-erased wrapper that holds type information for both data members and functions
struct TypeErasedValue {
    std::function<const std::type_info &()> type_info_func;
    std::function<void *(void)> get_ptr_func;
    std::function<void(void)> call_func; // Callable for member functions

    std::optional<GraphicsLabReflection::GraphicsLabFunctionMeta> call_func_with_param_meta;
    std::function<void(const std::vector<std::any> &)> call_func_with_param;

    TypeErasedValue() = default;

    bool isReflectable = false;

    template <typename T> bool isObjectReflectable() {
        return std::is_base_of<Reflectable, T>::value;
    }

    // Constructor for data members
    template <typename T> TypeErasedValue(T *value) {
        type_info_func = [value]() -> const std::type_info & { return typeid(T); };
        get_ptr_func = [value]() -> void * { return static_cast<void *>(value); };

        isReflectable = isObjectReflectable<T>();

        call_func = nullptr; // Not a function
    }

    // Constructor for member functions
    template <typename R, typename C> TypeErasedValue(R (C::*func)(void), C *obj) {
        type_info_func = [func]() -> const std::type_info & { return typeid(func); };
        get_ptr_func = nullptr; // Not a data member
        call_func = [func, obj]() { (obj->*func)(); };
    }

    template <typename R, typename C, typename... Args>
    TypeErasedValue(R (C::*func)(Args...), C *obj, std::tuple<Args...> default_values, std::vector<std::string> names) {
        type_info_func = [func]() -> const std::type_info & { return typeid(func); };
        get_ptr_func = nullptr; // Not a data member
        call_func = nullptr;

        call_func_with_param_meta = GraphicsLabReflection::GraphicsLabFunctionMeta{};
        call_func_with_param_meta.value() =
            GraphicsLabReflection::MemberFunctionReflection::createFunctionMetaWithName<C, R, Args...>(
                func, default_values, names);

        call_func_with_param = wrapMethod(obj, func);
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

    template <typename... Args> void call_with_param(Args... args) const {
        std::vector<std::any> unpackedArgs;
        unpackedArgs.reserve(sizeof...(Args));
        unpackedArgs.emplace_back(args...);
        call_func_with_param(unpackedArgs);
    }
};

using ReflectDataType = std::unordered_map<std::string, TypeErasedValue>;

struct DynamicField {
    std::any value;
    std::function<const std::type_info &()> type_info_func;

    const std::type_info &type() const {
        return type_info_func();
    }

    template <typename T> explicit DynamicField(T v) : value(v) {
        type_info_func = []() -> const std::type_info & { return typeid(T); };
    }

    explicit DynamicField() = default;

    template <typename T> void set(T &v) {
        value = v;
    }

    template <typename T> T get() {
        return std::any_cast<T>(value);
    }
};

// Reflection base class
struct Reflectable {
  public:
    virtual ~Reflectable() = default;
    virtual ReflectDataType reflect() = 0;

    template <typename T> T getField(const std::string &name) {
        if (dynamicFields.contains(name)) {
            return dynamicFields[name].get<T>();
        }

        throw std::runtime_error("Field '" + name + "' not found!");
    }

    template <typename T> void setField(const std::string &name, const T &value) {
        if (dynamicFields.contains(name)) {
            dynamicFields[name].set(value);
        }
        throw std::runtime_error("Field '" + name + "' not found!");
    }

    template <typename T> void addField(const std::string &name, const T &value) {
        // dynamicFields[name] = DynamicField(value);
        dynamicFields.emplace(name, value);
    }

    std::map<std::string, DynamicField> dynamicFields;
};