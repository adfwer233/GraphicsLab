#pragma once

#include <tuple>
#include <numeric>

namespace MetaProgramming {

template <typename... Ts> struct TypeList {
    static constexpr int size = sizeof...(Ts);

    using type = TypeList;

    struct IsTypeList {};

    template <typename... T> using append = TypeList<Ts..., T...>;

    template <typename... T> using prepend = TypeList<T..., Ts...>;

    using to_ptr = TypeList<Ts*...>;

    template <template <typename...> typename T> using to = T<Ts...>;

    template <template <typename> typename T> using monad = TypeList<T<Ts>...>;
};

namespace TypeListFunctions {

template <typename TypeList>
concept TL = requires { typename TypeList::IsTypeList; };

// IndexOf

template <TL In, typename T> struct IndexOf;

template <typename... Ts>
struct index_of {};

template<typename T, typename First, typename... Rest>
struct index_of<T, First, Rest...> {
    static constexpr size_t value = std::is_same_v<T, First> ? 0 : 1 + index_of<T, Rest...>::value;
};

template<typename T>
struct index_of<T> {
    static constexpr size_t value = std::numeric_limits<size_t>::max();
};

template<typename T, typename... Ts> struct IndexOf<TypeList<Ts...>, T>: index_of<T, Ts...> {};

// IsAnyOf

template <TL In, typename T> struct IsAnyOf;

template <typename... Ts>
struct is_any_of {};

template<typename T, typename First, typename... Rest>
struct is_any_of<T, First, Rest...> {
    static constexpr bool value = std::is_same_v<T, First> || is_any_of<T, Rest...>::value;
};

template<typename T>
struct is_any_of<T> {
    static constexpr bool value = false;
};

template<typename T, typename... Ts>
struct IsAnyOf<TypeList<Ts...>, T> : is_any_of<T, Ts...> {};

// Map

template <TL In, template <typename> typename F> struct Map;

template <template <typename> typename F, typename... Ts>
struct Map<TypeList<Ts...>, F> : TypeList<typename F<Ts>::type...> {};

template <TL In, template <typename> typename F> using Map_t = typename Map<In, F>::type;

// Filter

template <TL In, template <typename> typename P, TL Out> struct Filter : Out {};

template <template <typename> class P, TL Out, typename H, typename... Ts>
struct Filter<TypeList<H, Ts...>, P, Out>
    : std::conditional_t<P<H>::value, Filter<TypeList<Ts...>, P, typename Out::template append<H>>,
                         Filter<TypeList<Ts...>, P, Out>> {};

template <TL In, template <typename> typename P> using Filter_t = typename Filter<In, P, TypeList<>>::type;

// Fold

template <typename T> struct Return {
    using type = T;
};

template <TL In, typename Init, template <typename, typename> typename Op> struct Fold : Return<Init> {};

template <typename ACC, template <typename, typename> class Op, typename H, typename... Ts>
struct Fold<TypeList<H, Ts...>, ACC, Op> : Fold<TypeList<Ts...>, typename Op<ACC, H>::type, Op> {};

template <TL In, typename Init, template <typename, typename> class Op> using Fold_t = Fold<In, Init, Op>::type;

template <typename List, typename T> struct Append;

template <typename... Ts, typename T> struct Append<TypeList<Ts...>, T> {
    typedef TypeList<Ts..., T> type;
};

} // namespace TypeListFunctions
} // namespace MetaProgramming