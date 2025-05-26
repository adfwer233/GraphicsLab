#pragma once

#include "glm/glm.hpp"

namespace GraphicsLab::Geometry {

/**
 * @brief Poincare Hyperbolic Disk
 */
struct HyperbolicLineSegment {
    using PointType = std::complex<float>;

    PointType start_;
    PointType end_;

    PointType center_;
    float angle_start_;
    float angle_end_;
    float radius_;

    explicit HyperbolicLineSegment(PointType start, PointType end) : start_(start), end_(end) {
        if (std::norm(start - end) < 1e-3) {
            straight = true;
        }

        const float cross = start.real() * end.imag() - start.real() * end.imag();
        if (cross < 1e-5) {
            straight = true;
        }

        auto a = start;
        auto b = end;
        if (not cross) {
            center_ = (a * (1.0f - std::norm(b) * std::norm(b)) - b * (1.0f - std::norm(a) * std::norm(a))) /
                      (a * std::conj(b) - std::conj(a) * b);
        }
    }

    bool straight = false;
};

} // namespace GraphicsLab::Geometry