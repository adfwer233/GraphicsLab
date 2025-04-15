#pragma once

#include <geometry/parametric/bezier_curve_2d.hpp>
#include <random>

#include "geometry/parametric_intersector/bezier_2d_intersector.hpp"

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
            if (Geometry::Bezier2DIntersector::self_intersection(c)) {
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
                    auto inter = Geometry::Bezier2DIntersector::intersect(c, c + offset);
                    if (not inter.empty()) {
                        for (auto [p1, p2] : inter) {
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
                            auto inter = Geometry::Bezier2DIntersector::intersect(c1 + PointType{k, 0}, c);
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
                                auto inter = Geometry::Bezier2DIntersector::intersect(c1 + PointType{i, j}, c);
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