#pragma once

#include <array>

#include "type_list.hpp"

namespace MetaProgramming {

namespace details {

template <typename T, size_t dim, size_t... rest_dim> struct MultiDimensionalArrayImpl {
    using type = std::conditional_t<sizeof...(rest_dim) == 1, std::array<T, dim>,
                                    std::array<typename MultiDimensionalArrayImpl<T, rest_dim...>::type, dim>>;
};

template <typename T> struct MultiDimensionalArrayImpl<T, 0> {
    using type = size_t;
};
} // namespace details

template <typename T, size_t... dim> struct MultiDimensionalArray {
    // 0 is the dummy index
    using type = details::MultiDimensionalArrayImpl<T, dim..., 0>::type;
};
} // namespace MetaProgramming