#pragma once

#include <concepts>

#include "language/meta_programming/multi_dim_array.hpp"

template <std::floating_point T, size_t... DimLengths> class Polynomial {
    MetaProgramming::MultiDimensionalArray<T, DimLengths...> data;

    template <std::same_as<T>... Params> T evaluate(Params... params);

    template <size_t Dim, std::same_as<T>... Params> T evaluate_derivative(Params... params);

    template <size_t... DerivativeOrder, std::same_as<T>... Params>
    T evaluate_derivative_higher_order(Params... params);
};

#include "polynomial.hpp.impl"
