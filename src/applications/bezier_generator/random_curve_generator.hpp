#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <geometry/parametric/bezier_curve_2d.hpp>
#include <random>
#include <utility>
#include <vector>

namespace GraphicsLab {

struct Sampler {
    static double sample_uniform(double min, double max) {
        thread_local std::random_device rd;
        thread_local std::mt19937 gen(rd());
        thread_local std::uniform_real_distribution<double> dis(min, max);
        return dis(gen);
    }
};

struct RandomCurveGenerator {

    using PointType = glm::vec<2, double>;

    static bool bbox_overlap(const Geometry::BezierCurve2D &a, const Geometry::BezierCurve2D &b) {
        auto [minA, maxA] = a.boundingBox();
        auto [minB, maxB] = b.boundingBox();
        return maxA.x >= minB.x && minA.x <= maxB.x && maxA.y >= minB.y && minA.y <= maxB.y;
    }

    static void intersect_recursive(const Geometry::BezierCurve2D &c1, const Geometry::BezierCurve2D &c2, double t1a,
                                    double t1b, double t2a, double t2b,
                                    std::vector<std::pair<double, double>> &intersections, int depth = 0) {
        constexpr int MAX_DEPTH = 30;
        constexpr double EPSILON = 1e-6;
        if (depth > MAX_DEPTH || !bbox_overlap(c1, c2)) {
            return;
        }

        glm::dvec2 p1 = c1.evaluate(0.5);
        glm::dvec2 p2 = c2.evaluate(0.5);

        if (glm::distance(p1, p2) < EPSILON) {
            intersections.emplace_back((t1a + t1b) / 2.0, (t2a + t2b) / 2.0);
            return;
        }

        auto [c1_left, c1_right] = c1.subdivide(0.5);
        auto [c2_left, c2_right] = c2.subdivide(0.5);

        double t1m = (t1a + t1b) / 2.0;
        double t2m = (t2a + t2b) / 2.0;

        intersect_recursive(c1_left, c2_left, t1a, t1m, t2a, t2m, intersections, depth + 1);
        intersect_recursive(c1_left, c2_right, t1a, t1m, t2m, t2b, intersections, depth + 1);
        intersect_recursive(c1_right, c2_left, t1m, t1b, t2a, t2m, intersections, depth + 1);
        intersect_recursive(c1_right, c2_right, t1m, t1b, t2m, t2b, intersections, depth + 1);
    }

    static std::vector<std::pair<double, double>> intersect_bezier(const Geometry::BezierCurve2D &curve1,
                                                                   const Geometry::BezierCurve2D &curve2) {
        std::vector<std::pair<double, double>> intersections;
        intersect_recursive(curve1, curve2, 0.0, 1.0, 0.0, 1.0, intersections);
        return intersections;
    }

    static bool has_self_intersection(const Geometry::BezierCurve2D &curve) {
        auto points = curve.sample(100);
        auto segments_intersect = [](glm::dvec2 p1, glm::dvec2 p2, glm::dvec2 q1, glm::dvec2 q2) {
            auto cross = [](glm::dvec2 a, glm::dvec2 b) { return a.x * b.y - a.y * b.x; };

            glm::dvec2 r = p2 - p1;
            glm::dvec2 s = q2 - q1;
            glm::dvec2 qp = q1 - p1;

            double denom = cross(r, s);
            if (std::abs(denom) < 1e-10) {
                return false;
            }

            double t = cross(qp, s) / denom;
            double u = cross(qp, r) / denom;
            return (t > 0 && t < 1 && u > 0 && u < 1);
        };

        int n = points.size();
        for (int i = 0; i < n - 1; ++i) {
            for (int j = i + 2; j < n - 1; ++j) {
                if (std::abs(i - j) <= 1) {
                    continue;
                }

                if (segments_intersect(points[i], points[i + 1], points[j], points[j + 1])) {
                    return true;
                }
            }
        }

        return false;
    }

    static Geometry::BezierCurve2D generate_curve(int n, int m, int degree) {
        std::vector<PointType> control_points;
        PointType start_pos = {Sampler::sample_uniform(0.0, 1.0), Sampler::sample_uniform(0.0, 1.0)};
        PointType end_pos = start_pos + PointType{n, m};

        control_points.push_back(start_pos);
        for (int i = 1; i < degree; i++) {
            control_points.push_back({Sampler::sample_uniform(0.0, 1.0), Sampler::sample_uniform(0.0, 1.0)});
        }
        control_points.push_back(end_pos);

        Geometry::BezierCurve2D curve(std::move(control_points));
        return curve;
    }

    static Geometry::BezierCurve2D generate_simple_curve(int n, int m, int degree) {
        while (true) {
            auto c = generate_curve(n, m, degree);
            if (has_self_intersection(c)) {
                continue;
            }

            bool flag = false;
            const int repeat = 2 * std::max(std::abs(n), std::abs(m)) + 1;
            for (int i = 0; i < repeat; i++) {
                for (int j = 0; j < repeat; j++) {
                    glm::dvec2 offset{i, j};
                    if (i == 0 and j == 0) {
                        continue;
                    }
                    auto inter = intersect_bezier(c, c + offset);
                    if (not inter.empty()) {
                        for (std::size_t k = 0; k < inter.size(); ++k) {
                            auto p1 = inter[k].first;
                            auto p2 = inter[k].second;
                            if (p1 > 0.001 and p1 < 0.999 and p2 > 0.001 and p2 < 0.999) {
                                flag = true;
                            }
                        }
                        if (flag)
                            break;
                    }
                }
                if (flag)
                    break;
            }

            if (not flag) {
                return c;
            }
        }
    }

    static std::vector<Geometry::BezierCurve2D> generate_uniperiodic_curves() {
        int n = 3;

        std::vector<Geometry::BezierCurve2D> result;

        auto generate_new_curve = [&](int n, int m, int d) {
            auto c1 = generate_simple_curve(n, m, d);

            if (result.empty()) {
                result.push_back(c1);
            } else {
                while (true) {
                    bool intersected_with_previous = false;
                    for (auto &c : result) {
                        bool break_flag = false;
                        for (int k = -2; k <= 2; k++) {
                            auto inter = intersect_bezier(c1 + PointType{k, 0}, c);
                            if (not inter.empty()) {
                                intersected_with_previous = true;
                                break_flag = true;
                                break;
                            }
                        }
                        if (break_flag)
                            break;
                    }

                    if (not intersected_with_previous) {
                        result.push_back(c1);
                        break;
                    }

                    c1 = generate_simple_curve(n, m, d);
                }
            }

            return c1;
        };

        for (int i = 1; i <= n; i++) {
            generate_new_curve(1, 0, 5);
            generate_new_curve(-1, 0, 5);
        }

        std::ranges::sort(result,
                          [](const auto &a, const auto &b) { return a.start_position().y < b.start_position().y; });

        for (int i = 0; i < result.size(); i++) {
            if (i % 2 == 0 and result[i].start_position().x > result[i].end_position().x) {
                std::ranges::reverse(result[i].control_points_);
                result[i] = result[i] + PointType(1, 0);
            } else if (i % 2 == 1 and result[i].start_position().x < result[i].end_position().x) {
                std::ranges::reverse(result[i].control_points_);
                result[i] = result[i] - PointType(1, 0);
            }
        }

        for (auto &c : result) {
            auto end = c.end_position();
            // spdlog::info("{} {}", end.x, end.y);
        }

        return result;
    }

    static std::vector<Geometry::BezierCurve2D> generate_biperiodic_curves(int coeff1, int coeff2) {
        // int n = 3;

        std::vector<Geometry::BezierCurve2D> result;

        auto generate_new_curve = [&](int n, int m, int d) {
            auto c1 = generate_simple_curve(n, m, d);

            if (result.empty()) {
                result.push_back(c1);
            } else {
                while (true) {
                    bool intersected_with_previous = false;
                    for (auto &c : result) {
                        bool break_flag = false;
                        int repeat = 2 * std::max(std::abs(n), std::abs(m)) + 1;
                        for (int i = -repeat; i <= repeat; i++) {
                            for (int j = -repeat; j <= repeat; j++) {
                                auto inter = intersect_bezier(c1 + PointType{i, j}, c);
                                if (not inter.empty()) {
                                    intersected_with_previous = true;
                                    break_flag = true;
                                    break;
                                }
                            }
                            if (break_flag)
                                break;
                        }
                        if (break_flag)
                            break;
                    }

                    if (not intersected_with_previous) {
                        result.push_back(c1);
                        break;
                    }

                    // spdlog::info("regenerate");

                    c1 = generate_simple_curve(n, m, d);
                }
            }

            return c1;
        };

        for (int i = 1; i <= 1; i++) {
            generate_new_curve(coeff1, coeff2, 3);
            generate_new_curve(-coeff1, -coeff2, 3);
        }

        std::ranges::sort(result,
                          [](const auto &a, const auto &b) { return a.start_position().y < b.start_position().y; });

        // for (int i = 0; i < result.size(); i++) {
        //     if (i % 2 == 0 and result[i].start_position().x > result[i].end_position().x) {
        //         std::ranges::reverse(result[i].control_points_);
        //         result[i] = result[i] + PointType(1, 0);
        //     } else if (i % 2 == 1 and result[i].start_position().x < result[i].end_position().x) {
        //         std::ranges::reverse(result[i].control_points_);
        //         result[i] = result[i] - PointType(1, 0);
        //     }
        // }

        return result;
    }
};

} // namespace GraphicsLab