#pragma once

#include <ranges>
#include "glm/glm.hpp"

#include "bezier_root_finder.hpp"

namespace GraphicsLab::Geometry {
template<typename T>
struct BezierClipping {
public:
    static uint32_t bezier_clipping(glm::vec<2, T> test_point, const std::vector<glm::vec<2, T>>& points) {
        bool quadrant1 = false, quadrant2 = false, quadrant3 = false, quadrant4 = false;

        for (auto &point : points) {
            if (point.x > test_point.x and point.y > test_point.y)
                quadrant1 = true;
            if (point.x < test_point.x and point.y > test_point.y)
                quadrant2 = true;
            if (point.x < test_point.x and point.y < test_point.y)
                quadrant3 = true;
            if (point.x > test_point.x and point.y < test_point.y)
                quadrant4 = true;
        }

        // if all points lies on the same side of the ray, return zero

        if (quadrant1 and quadrant2 and (not quadrant3) and (not quadrant4))
            return 0;
        if ((not quadrant1) and (not quadrant2) and quadrant3 and quadrant4)
            return 0;

        if ((not quadrant2) and (not quadrant3)) {
            auto tmp = (points.front().y - test_point.y) * (points.back().y - test_point.y);
            if (tmp > 0)
                return 0;
            if (tmp < 0)
                return 1;
        }

        if ((not quadrant1) and (not quadrant4)) {
            return 0;
        }

        std::vector<double> x_poly, y_poly;

        std::ranges::for_each(points, [&](glm::vec2 v) {
            x_poly.push_back(v.x);
            y_poly.push_back(v.y - test_point.y);
        });

        BezierRootFinder finder(std::move(y_poly));

        auto result = finder.get_roots();

        size_t valid_num =
                std::ranges::count_if(result, [&](double param) { return evaluate(param, x_poly) > test_point.x; });

        return valid_num % 2;
    }

private:
    static double evaluate(double param, std::vector<T> &coefficients) {
        T h = 1.0;
        T result = coefficients[0];
        T t = param;
        T u = 1 - t;
        uint32_t n = coefficients.size() - 1;
        uint32_t n1 = n + 1;
        if (param <= 0.5) {
            u = t / u;
            for (int k = 1; k <= n; k++) {
                h = h * u * (n1 - k);
                h = h / (k + h);
                T h1 = 1 - h;
                result = result * h1 + coefficients[k] * h;
            }
        } else {
            u = u / t;
            for (int k = 1; k <= n; k++) {
                h = h * (n1 - k);
                h = h / (k * u + h);
                T h1 = 1 - h;
                result = result * h1 + coefficients[k] * h;
            }
        }
        return result;
    }
};
}