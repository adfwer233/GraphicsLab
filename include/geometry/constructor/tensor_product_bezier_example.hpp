#pragma once

#include "geometry/parametric/tensor_product_bezier.hpp"

namespace GraphicsLab::Geometry {

struct TensorProductBezierExample1 {
    static TensorProductBezier create() {
        using Point = TensorProductBezier::PointType;

        std::vector<std::vector<Point>> control_points = {{Point(0.0, 0.0, 0.0), Point(0.0, 1.0, 0.0)},
                                                          {Point(1.0, 0.0, 0.0), Point(1.0, 1.0, 0.0)}};

        TensorProductBezier surface(control_points);
        return surface;
    }
};

struct TensorProductBezierExample2 {
    static TensorProductBezier create() {
        using Point = TensorProductBezier::PointType;
        const std::vector<std::vector<Point>> control_points = {
            {Point(0.0, 0.0, 0.0), Point(0.0, 1.0, 0.0)},
            {Point(0.0, 0.0, 1.0), Point(0.0, 1.0, 1.0)},
            {Point(1.0, 0.0, 1.0), Point(1.0, 1.0, 1.0)},
            {Point(1.0, 0.0, 0.0), Point(1.0, 1.0, 0.0)},
            {Point(0.0, 0.0, 0.0), Point(0.0, 1.0, 0.0)},
        };
        return TensorProductBezier(control_points);
    }
};

}