#pragma once

#include "meta_programming/type_traits/is_function.hpp"

// Type-erased wrapper that holds type information for both data members and functions
struct TypeErasedValue {
    std::function<const std::type_info&()> type_info_func;
    std::function<void*(void)> get_ptr_func;
    std::function<void(void)> call_func;  // Callable for member functions

    TypeErasedValue() = default;

    // Constructor for data members
    template <typename T>
    TypeErasedValue(T* value) {
        type_info_func = [value]() -> const std::type_info& {
            return typeid(T);
        };
        get_ptr_func = [value]() -> void* {
            return static_cast<void*>(value);
        };
        call_func = nullptr;  // Not a function
    }

    // Constructor for member functions
    template <typename R, typename C>
    TypeErasedValue(R(C::*func)(void), C* obj) {
        type_info_func = [func]() -> const std::type_info& {
            return typeid(func);
        };
        get_ptr_func = nullptr;  // Not a data member
        call_func = [func, obj]() {
            (obj->*func)();
        };
    }

    const std::type_info& type() const {
        return type_info_func();
    }

    void* get() const {
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

class Reflection {
    template<typename T>
    static auto eraseValueType(T& t) {
        return TypeErasedValue(&t);
    }

    template<IsFunction F, typename T>
    static auto eraseFunctionType(F f, T* t) {
        return TypeErasedValue(f, t);
    }
};

// Reflection base class
class Reflectable {
  public:
    virtual ReflectDataType reflect() = 0;
};