#pragma once

#include "curve_curve_inter_result.hpp"

#include <geometry/parametric/bezier_curve_2d.hpp>

namespace GraphicsLab::Geometry {

struct BezierBezierIntersector2D {
    static CurveCurveIntersectionResult2D intersect(const BezierCurve2D &curve1, const BezierCurve2D &curve2) {
        CurveCurveIntersectionResult2D result;
        std::vector<std::pair<double, double>> intersections;

        intersect_recursive(curve1, curve2, 0.0, 1.0, 0.0, 1.0, intersections);

        for (auto [p1, p2] : intersections) {
            auto pos1 = curve1.evaluate(p1);
            auto pos2 = curve2.evaluate(p2);

            result.curve1_param.push_back(p1);
            result.curve2_param.push_back(p2);
            result.inter_points.push_back((pos1 + pos2) / 2.0);
        }

        return result;
    }

  private:
    static bool bbox_overlap(const BezierCurve2D &a, const BezierCurve2D &b) {
        auto [minA, maxA] = a.boundingBox();
        auto [minB, maxB] = b.boundingBox();
        return (maxA.x >= minB.x && minA.x <= maxB.x && maxA.y >= minB.y && minA.y <= maxB.y);
    }

    static void intersect_recursive(const BezierCurve2D &c1, const BezierCurve2D &c2, double t1a, double t1b,
                                    double t2a, double t2b, std::vector<std::pair<double, double>> &intersections,
                                    int depth = 0) {

        constexpr int MAX_DEPTH = 30;
        constexpr double EPSILON = 1e-6;
        if (depth > MAX_DEPTH || !bbox_overlap(c1, c2))
            return;

        glm::dvec2 p1 = c1.evaluate(0.5);
        glm::dvec2 p2 = c2.evaluate(0.5);

        if (glm::distance(p1, p2) < EPSILON) {
            intersections.emplace_back((t1a + t1b) / 2.0, (t2a + t2b) / 2.0);
            return;
        }

        auto [c1_left, c1_right] = c1.subdivide(0.5);
        auto [c2_left, c2_right] = c2.subdivide(0.5);

        double t1m = (t1a + t1b) / 2;
        double t2m = (t2a + t2b) / 2;

        intersect_recursive(c1_left, c2_left, t1a, t1m, t2a, t2m, intersections, depth + 1);
        intersect_recursive(c1_left, c2_right, t1a, t1m, t2m, t2b, intersections, depth + 1);
        intersect_recursive(c1_right, c2_left, t1m, t1b, t2a, t2m, intersections, depth + 1);
        intersect_recursive(c1_right, c2_right, t1m, t1b, t2m, t2b, intersections, depth + 1);
    }

    static bool segments_intersect(glm::dvec2 p1, glm::dvec2 p2, glm::dvec2 q1, glm::dvec2 q2) {
        auto cross = [](glm::dvec2 a, glm::dvec2 b) { return a.x * b.y - a.y * b.x; };

        glm::dvec2 r = p2 - p1;
        glm::dvec2 s = q2 - q1;
        glm::dvec2 qp = q1 - p1;

        double denom = cross(r, s);
        if (std::abs(denom) < 1e-10)
            return false; // Parallel

        double t = cross(qp, s) / denom;
        double u = cross(qp, r) / denom;

        return (t > 0 && t < 1 && u > 0 && u < 1);
    }
};

} // namespace GraphicsLab::Geometry