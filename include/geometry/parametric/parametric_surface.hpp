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

    ParamSurface() {}

    virtual PointType evaluate(const ParamType param) const = 0;
    virtual PointType normal(const ParamType param) const = 0;
    virtual std::pair<PointType, ParamType> project(const PointType point) const = 0;
    virtual bool test_point(const PointType point) const = 0;

    virtual bool is_singular(ParamType param) const {
        return false;
    }

    virtual std::pair<VectorType, VectorType> derivative(const ParamType) const {
        return {};
    }

    /**
     * @brief Sample n * m points uniformly
     */
    [[nodiscard]] std::vector<std::pair<PointType, ParamType>> sample(int n, int m) const {
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

    [[nodiscard]] ParamType move_param_to_std_domain(ParamType param) const {
        if (u_periodic) {
            const int shift = std::floor(param.x);
            param.x -= shift;
        }

        if (v_periodic) {
            const int shift = std::floor(param.y);
            param.y -= shift;
        }

        return param;
    }

    bool u_periodic = false;
    bool v_periodic = false;

    bool u_periodic_check() const {
        constexpr int sample_num = 10;

        for (int i = 0; i <= sample_num; i++) {
            double v = static_cast<double>(i) / sample_num;
            const auto pos1 = evaluate({0.0, v});
            const auto pos2 = evaluate({1.0, v});
            if (glm::distance(pos1, pos2) > 1e-6) {
                return false;
            }
        }
        return true;
    }

    bool v_periodic_check() const {
        constexpr int sample_num = 10;

        for (int i = 0; i <= sample_num; i++) {
            double u = static_cast<double>(i) / sample_num;
            const auto pos1 = evaluate({u, 0.0});
            const auto pos2 = evaluate({u, 1.0});

            if (glm::distance(pos1, pos2) > 1e-6) {
                return false;
            }
        }

        return true;
    }

    std::unique_ptr<Mesh3D> mesh = nullptr;
};

template <typename T>
concept ParamSurfaceTrait = requires { typename T::ParamSurfaceTrait; };

} // namespace GraphicsLab::Geometry