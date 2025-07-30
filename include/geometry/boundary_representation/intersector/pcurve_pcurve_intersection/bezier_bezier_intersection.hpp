#pragma once

#include "geometry/parametric/bezier_curve_2d.hpp"
#include "ppi_results.hpp"

namespace GraphicsLab::Geometry::BRep {

struct BezierBezierIntersector2D {
    static std::vector<PPIResult> intersect(const BezierCurve2D &curve1, const BezierCurve2D &curve2, const BRepPoint2 offset = BRepPoint2(0)) {
        std::vector<PPIResult> results;
        std::vector<std::pair<double, double>> intersections;

        if (is_point(curve1) or is_point(curve2)) {
            // if (glm::distance(curve1.evaluate(0), curve2.evaluate(0)) < 1e-3) {
            //     return {PPIResult{.param1 = 0, .param2 = 0, .inter_position = curve1.start_position()}};
            // } else {
            //     return {};
            // }
        }
        intersect_recursive(curve1, curve2, 0.0, 1.0, 0.0, 1.0, intersections, offset);

        for (auto [p1, p2] : intersections) {
            auto pos1 = curve1.evaluate(p1) + offset;
            auto pos2 = curve2.evaluate(p2);

            PPIResult result;

            result.param1 = p1;
            result.param2 = p2;
            result.inter_position = (pos1 + pos2) / 2.0;
            results.push_back(result);
        }

        return results;
    }

  private:
    static bool bbox_overlap(const BezierCurve2D &a, const BezierCurve2D &b, BRepPoint2 offset = BRepPoint2(0)) {
        auto [minA, maxA] = a.boundingBox();
        auto [minB, maxB] = b.boundingBox();

        minA += offset;
        maxA += offset;
        minA -= 1e-5;
        maxA += 1e-5;
        minB -= 1e-5;
        maxB += 1e-5;

        return (maxA.x >= minB.x && minA.x <= maxB.x && maxA.y >= minB.y && minA.y <= maxB.y);
    }

    static void intersect_recursive(const BezierCurve2D &c1, const BezierCurve2D &c2, double t1a, double t1b,
                                    double t2a, double t2b, std::vector<std::pair<double, double>> &intersections, const BRepPoint2 offset = BRepPoint2(0),
                                    int depth = 0) {

        constexpr int MAX_DEPTH = 30;
        constexpr double EPSILON = 1e-4;
        if (depth > MAX_DEPTH || !bbox_overlap(c1, c2, offset))
            return;

        glm::dvec2 p1 = c1.evaluate(0.5) + offset;
        glm::dvec2 p2 = c2.evaluate(0.5);

        if (glm::distance(p1, p2) < EPSILON) {
            intersections.emplace_back((t1a + t1b) / 2.0, (t2a + t2b) / 2.0);

            if (intersections.size() > 10) {
                int n = 0;
            }
            return;
        }

        auto [c1_left, c1_right] = c1.subdivide(0.5);
        auto [c2_left, c2_right] = c2.subdivide(0.5);

        double t1m = (t1a + t1b) / 2;
        double t2m = (t2a + t2b) / 2;

        intersect_recursive(c1_left, c2_left, t1a, t1m, t2a, t2m, intersections, offset, depth + 1);
        intersect_recursive(c1_left, c2_right, t1a, t1m, t2m, t2b, intersections, offset, depth + 1);
        intersect_recursive(c1_right, c2_left, t1m, t1b, t2a, t2m, intersections, offset, depth + 1);
        intersect_recursive(c1_right, c2_right, t1m, t1b, t2m, t2b, intersections, offset, depth + 1);
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

    static bool is_point(const BezierCurve2D &curve) {
        for (int i = 1; i < curve.control_points_.size(); ++i) {
            if (glm::distance(curve.control_points_[i], curve.control_points_[i - 1]) > 1e-3) {
                return false;
            }
        }
        return true;
    }
};

} // namespace GraphicsLab::Geometry::BRep