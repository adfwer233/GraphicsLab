#pragma once
#include <geometry/parametric/bezier_curve_2d.hpp>
#include <random>

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
        PointType start_pos = {Sampler::sample_uniform(0.0, 1.0) * 0, Sampler::sample_uniform(0.0, 1.0)};
        PointType end_pos = start_pos + PointType{1, m};

        control_points.push_back(start_pos);
        for (int i = 1; i < degree; i++) {
            control_points.push_back({Sampler::sample_uniform(0.0, 1.0), Sampler::sample_uniform(0.0, 1.0)});
        }
        control_points.push_back(end_pos);

        Geometry::BezierCurve2D curve(std::move(control_points));
        return curve;
    }

    static std::vector<Geometry::BezierCurve2D> generate_uniperiodic_curves() {
        int n = 3;

        std::vector<Geometry::BezierCurve2D> result;
        for (int i = 1; i <= n; i++) {
            result.push_back(generate_curve(1, 0, 3));
            result.push_back(generate_curve(-1, 0, 3));
        }
        return result;
    }
};

} // namespace GraphicsLab