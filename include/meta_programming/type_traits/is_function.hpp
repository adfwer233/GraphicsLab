#include <type_traits>

// Base template: false for all types
template <typename T>
struct is_function : std::false_type {};

// Specialization for regular functions
template <typename Ret, typename... Args>
struct is_function<Ret(Args...)> : std::true_type {};

// Specialization for function pointers
template <typename Ret, typename... Args>
struct is_function<Ret(*)(Args...)> : std::true_type {};

// Specialization for member functions
template <typename Ret, typename Class, typename... Args>
struct is_function<Ret(Class::*)(Args...)> : std::true_type {};

// Specialization for const member functions
template <typename Ret, typename Class, typename... Args>
struct is_function<Ret(Class::*)(Args...) const> : std::true_type {};

// Helper variable template for easy usage
template <typename T>
inline constexpr bool is_function_v = is_function<T>::value;

template <typename T>
concept IsFunction = is_function_v<T>;