#pragma once

#include "geometry/mesh/mesh.hpp"
#include "glm/glm.hpp"

namespace GraphicsLab::Geometry {

struct ParamSurface {
    virtual ~ParamSurface() = default;
    struct SurfaceTrait {};
    struct SceneTreeGeometryTypeTrait {};

    struct ParamSurfaceTrait {};

    using T = double;
    using ParamType = glm::vec<2, double>;
    using PointType = glm::vec<3, double>;
    using VectorType = glm::vec<3, double>;

    virtual PointType evaluate(const ParamType param) const = 0;
    virtual PointType normal(const ParamType param) const = 0;
    virtual std::pair<PointType, ParamType> project(const PointType point) const = 0;
    virtual bool test_point(const PointType point) const = 0;

    /**
     * @brief Sample n * m points uniformly
     */
    std::vector<std::pair<PointType, ParamType>> sample(int n, int m) const {
        std::vector<std::pair<PointType, ParamType>> result;

        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                double param_u = static_cast<double>(i) / static_cast<double>(n - 1);
                double param_v = static_cast<double>(j) / static_cast<double>(m - 1);
                result.emplace_back(evaluate({param_u, param_v}), ParamType{param_u, param_v});
            }
        }

        return result;
    }

    std::unique_ptr<Mesh3D> mesh = nullptr;
};

template <typename T>
concept ParamSurfaceTrait = requires { typename T::ParamSurfaceTrait; };

} // namespace GraphicsLab::Geometry