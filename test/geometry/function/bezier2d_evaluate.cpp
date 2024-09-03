#include "gtest/gtest.h"

#include "geometry/curve/bezier_curve_2d.hpp"

#include "effolkronium/random.hpp"

using Random = effolkronium::random_static;

TEST(Bezier2DTest, EvaluationBenchmark) {
    std::vector<std::array<float, 2>> control_points {
        {0.0f, 0.0f}, {0.0f, 1.0f}, {1.0f, 1.0f}, {1.0f, 0.0f}, {2.0f, 0.0f}
    };

    BezierCurve2D curve(std::move(control_points));

    std::vector<float> params;

    for (int i = 0; i < 10000; i++) {
        params.push_back(Random::get(0.0f, 1.0f));
    }

    auto start1 = std::chrono::steady_clock::now();

    // de Casteljau's evaluation
    for (float param: params) {
        curve.evaluate(param);
    }
    auto end1 = std::chrono::steady_clock::now();

    auto start2 = std::chrono::steady_clock::now();
    // polynomial evaluation
    for (float param: params) {
        curve.evaluate_polynomial(param);
    }
    auto end2 = std::chrono::steady_clock::now();

    auto duration1 = end1 - start1;
    auto duration2 = end2 - start2;

    std::cout << std::chrono::duration_cast<std::chrono::nanoseconds>(duration1).count() << std::endl;
    std::cout << std::chrono::duration_cast<std::chrono::nanoseconds >(duration2).count() << std::endl;

    for (float param: params) {
        auto res1 = curve.evaluate_polynomial(param);
        auto res2  = curve.evaluate(param);
        EXPECT_TRUE(std::abs(res1.x - res2.x) < 1e-5 and std::abs(res1.y - res2.y) < 1e-5);
    }
}