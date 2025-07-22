#pragma once

#include <geometry/box/box.hpp>
#include <geometry/mesh/mesh.hpp>
#include <glm/glm.hpp>

namespace GraphicsLab::Geometry {

template <size_t dim> struct ParamCurveBase {
    using PointType = std::conditional_t<dim == 3, glm::dvec3, glm::dvec2>;
    using MeshType = std::conditional_t<dim == 3, CurveMesh3D, CurveMesh2D>;
    using BoxType = Box<dim, double>;

    struct DiscretizationCache {
        std::vector<PointType> points;
    };
    virtual ~ParamCurveBase() = default;

    virtual PointType evaluate(double t) const = 0;
    virtual PointType derivative(double t) const = 0;

    [[nodiscard]] virtual std::vector<std::pair<PointType, double>> sample (int n) const {
        std::vector<std::pair<PointType, double>> samples;

        for (int i = 0; i < n; i++) {
            double param = static_cast<double>(i) / static_cast<double>(n - 1);
            auto point = this->evaluate(param);
            samples.emplace_back(point, param);

        }

        return samples;
    }

    BoxType get_box() {
        return box;
    }

    std::unique_ptr<MeshType> mesh = nullptr;

    std::vector<PointType> discretize() {
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
    BoxType box;
};

using ParamCurve2D = ParamCurveBase<2>;
using ParamCurve3D = ParamCurveBase<3>;

} // namespace GraphicsLab::Geometry