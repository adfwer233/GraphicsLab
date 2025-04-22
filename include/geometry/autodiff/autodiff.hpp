#pragma once

#include "autodiff/reverse/var.hpp"
#include "glm/glm.hpp"

namespace GraphicsLab::Geometry {

template <uint32_t dim> struct autodiff_vec {
    std::array<autodiff::var, dim> data;

    using value_type = autodiff::var;

    size_t size() const {
        return dim;
    }

    template <std::floating_point... Values> explicit autodiff_vec(Values... values) {
        static_assert(dim == sizeof...(values) || sizeof...(values) == 0);
        int i = 0;
        ((data[i] = values, i++), ...);
    }

    autodiff_vec() {
        for (auto &item : data)
            item = 0;
    }

    autodiff_vec<dim> operator+(const autodiff_vec<dim> &a);
    autodiff_vec<dim> operator-(const autodiff_vec<dim> &a);
    autodiff_vec<dim> operator*(const autodiff_vec<dim> &a);
    autodiff_vec<dim> operator/(const autodiff_vec<dim> &a);

    autodiff::var &operator[](size_t idx) {
        return data[idx];
    }

    autodiff::var operator[](size_t idx) const {
        return data[idx];
    }
};

struct autodiff_vec2 : public autodiff_vec<2> {
    template <std::floating_point... Values> explicit autodiff_vec2(Values... values) : autodiff_vec<2>(values...) {
    }

    autodiff_vec2(glm::vec2 vec) : autodiff_vec<2>(vec.x, vec.y) {
    }

    autodiff::var &x() {
        return data[0];
    }
    [[nodiscard]] autodiff::var x() const {
        return data[0];
    }

    autodiff::var &y() {
        return data[1];
    }
    [[nodiscard]] autodiff::var y() const {
        return data[1];
    }
};

struct autodiff_vec3 : public autodiff_vec<3> {
    template <std::floating_point... Values> explicit autodiff_vec3(Values... values) : autodiff_vec<3>(values...) {
    }

    autodiff_vec3(glm::vec3 vec3) : autodiff_vec<3>(vec3.x, vec3.y, vec3.z) {
    }

    autodiff::var &x() {
        return data[0];
    }
    [[nodiscard]] autodiff::var x() const {
        return data[0];
    }

    autodiff::var &y() {
        return data[1];
    }
    [[nodiscard]] autodiff::var y() const {
        return data[1];
    }

    autodiff::var &z() {
        return data[2];
    }
    [[nodiscard]] autodiff::var z() const {
        return data[2];
    }
};

struct autodiff_vec4 : public autodiff_vec<4> {};

template <uint32_t m, uint32_t n> struct autodiff_mat {

    std::array<std::array<autodiff::var, n>, m> data;

    autodiff::var &operator()(size_t idx1, size_t idx2) {
        return data[idx1][idx2];
    }
    autodiff::var operator()(size_t idx1, size_t idx2) const {
        return data[idx1][idx2];
    }
};

struct autodiff_mat2 : autodiff_mat<2, 2> {};

struct autodiff_mat3 : autodiff_mat<3, 3> {};


}