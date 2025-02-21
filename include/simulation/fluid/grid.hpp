#pragma once

#include <concepts>
#include <vector>

namespace GraphicsLab::Simulation {

template<typename T = float>
struct Grid2D {
    std::vector<T> data;
    int width, height;

    Grid2D(int width, int height) : width(width), height(height) {
        data.resize(width * height);
    }

    T& operator()(int x, int y) {
        return data[y * width + x];
    }

    const T& operator()(int x, int y) const {
        return data[y * width + x];
    }

    T& at(int x, int y) {
        return data.at(y * width + x);
    }

    const T& at(int x, int y) const {
        return data.at(y * width + x);
    }

    void fill(T value) {
        std::fill(data.begin(), data.end(), value);
    }
};

}