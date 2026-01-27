#pragma once

#include "cpptrace/cpptrace.hpp"
#include "spdlog/spdlog.h"

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
    virtual PointType second_derivative(double t) const = 0;

    virtual bool is_closed() const {
        auto start = evaluate(0);
        auto end = evaluate(1);

        return glm::distance(start, end) < 1e-6;
    }
    /**
     * @brief A simple Newton for general curve projection.
     * @param test_point
     * @param param_guess
     * @return
     *
     * @note
     *  Suppose the curve is c(t). Define f(t) = |C(t) - p|^2
     *  we calculate:
     *  f'(t) = 2(c(t) - p) \cdot c'(t)
     *  f''(t) = 2(c'(t) \cdot c'(t) + (c(t) - p) \cdot c''(t))
     *
     *  Then use the Newton-Raphson
     *
     *  t_{n + 1} = t_{n} - f'(t_n) / f'(t_{n + 1})
     */
    virtual std::pair<PointType, double> projection(PointType test_point, std::optional<double> param_guess) const {
        // @todo
        double t0 = 0.5;
        if (param_guess.has_value())
            t0 = param_guess.value();

        constexpr int max_iter = 10;

        auto closest_in_range = [&](double start, double end, double t) -> double {
            for (int i = 0; i < max_iter; i++) {
                PointType c = evaluate(t);
                PointType cp = derivative(t);
                PointType cpp = second_derivative(t);

                double f_prime = 2 * glm::dot(c - test_point, cp);
                double f_prime_prime = 2 * (glm::dot(c - test_point, cpp) + glm::dot(cp, cp));

                if (std::abs(f_prime_prime) < 1e-10) {
                    break;
                }

                double delta = f_prime / f_prime_prime;

                delta = std::clamp(delta, -0.3, 0.3);

                double t_next = t - delta;
                t_next = std::clamp(t_next, start, end);

                t = t_next;

                if (std::abs(delta) < 1e-8) {
                    break;
                }
            }

            return t;
        };

        std::vector<std::pair<double, double>> ranges{{0.0, 0.5}, {0.5, 1.0}};

        double res = -1;
        double distance = std::numeric_limits<double>::max();
        for (auto [s, t] : ranges) {
            auto t1 = (s + t) / 2;
            if (t0 > s and t0 < t) {
                t1 = t0;
            }

            auto t_newton = closest_in_range(s, t, t1);
            PointType cur = evaluate(t_newton);

            if (glm::distance(cur, test_point) < distance) {
                distance = glm::distance(cur, test_point);
                res = t_newton;
            }
        }
        // PointType pos = evaluate(t);
        // auto dis = glm::distance(pos, test_point);
        // if (glm::dot(test_point - pos, derivative(t)) > 1e-3) {
        //     throw cpptrace::logic_error("Curve projection failed.");
        // }

        return {evaluate(res), res};
    }

    [[nodiscard]] virtual std::vector<std::pair<PointType, double>> sample(int n) const {
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