#pragma once

#include <geometry/mesh/mesh.hpp>
#include <glm/glm.hpp>

namespace GraphicsLab::Geometry {
struct ParamCurve3D {
    virtual ~ParamCurve3D() = default;

    virtual glm::dvec3 evaluate(double t) const = 0;

    std::unique_ptr<CurveMesh3D> mesh = nullptr;
};

struct ParamCurve2D {
    struct DiscretizationCache {
        std::vector<glm::dvec2> points;
    };
    virtual ~ParamCurve2D() = default;

    virtual glm::dvec2 evaluate(double t) const = 0;

    std::unique_ptr<CurveMesh2D> mesh = nullptr;

    std::vector<glm::dvec2> discretize() {
        if (discretizationCache) {
            return discretizationCache->points;
        } else {
            discretizationCache = std::make_unique<DiscretizationCache>();
            for (int i = 0; i < 100; ++i) {
                double param = static_cast<double>(i) / 100.0;
                discretizationCache->points.push_back(evaluate(param));
            }
            return discretizationCache->points;
        }
    }

  protected:
    std::unique_ptr<DiscretizationCache> discretizationCache = nullptr;
};

template <size_t dim> using ParamCurveBase = std::conditional_t<dim == 3, ParamCurve3D, ParamCurve2D>;
} // namespace GraphicsLab::Geometry