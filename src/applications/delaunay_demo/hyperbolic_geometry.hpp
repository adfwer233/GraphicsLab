#pragma once

#include "glm/glm.hpp"
#include "mobius_transformation.hpp"

namespace GraphicsLab::Geometry {

/**
 * @brief Poincare Hyperbolic Disk
 */
struct HyperbolicLineSegment {
    using PointType = std::complex<float>;

    PointType start_;
    PointType end_;

    PointType center_;

    explicit HyperbolicLineSegment(PointType start, PointType end) : start_(start), end_(end) {
        if (std::norm(start - end) < 1e-3) {
            straight = true;
        }

        const float cross = start.real() * end.imag() - start.real() * end.imag();
        if (cross < 1e-5) {
            straight = true;
        }

        auto m1 = MobiusConstructor::move_to_origin(start);
        auto m2 = MobiusConstructor::rotate(-std::arg(m1(end)));
        transformation_ = m2.compose(m1);
        transformation_inverse_ = transformation_.inverse();
        end_transformed_ = transformation_(end_);
    }

    bool straight = false;

    PointType evaluate(float t) {
        auto res = end_transformed_ * t;
        return transformation_inverse_(res);
    }

private:
    Mobius transformation_, transformation_inverse_;
    PointType end_transformed_;
};

} // namespace GraphicsLab::Geometry