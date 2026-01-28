#include "geometry/boundary_representation/allocator/brep_allocator.hpp"

#include "gtest/gtest.h"

#include "geometry/parametric_intersector/curve_curve_intersector_2d/bezier_bezier_intersection_2d.hpp"
#include "geometry/parametric_intersector/curve_curve_intersector_2d/line_param_intersector_2d.hpp"
#include "utils/sampler.hpp"

#include <geometry/parametric/parametric_curves/ellipse.hpp>

TEST(LineTest, Projection) {
    using namespace GraphicsLab;
    using namespace Geometry;

    constexpr int repeat = 100;

    for (int i = 0; i < repeat; i++) {
        BRep::BRepPoint2 start_point = Sampler::sampleUniformVec2();
        BRep::BRepPoint2 end_point = Sampler::sampleUniformVec2();

        const auto allocator = BRep::BRepAllocator::instance();
        const auto line = allocator->alloc_param_pcurve<StraightLine2D>(start_point, end_point);
        const BRep::BRepPoint2 test_point = Sampler::sampleUniformVec2();

        auto [line_proj, line_proj_param] = line->projection(test_point, std::nullopt);
        auto [proj, proj_param] = line->ParamCurve2D::projection(test_point, std::nullopt);

        const auto distance = glm::distance(line_proj, proj);
        const auto param_distance = glm::distance(line_proj_param, proj_param);

        EXPECT_NEAR(distance, 0, 1e-6);
        EXPECT_NEAR(param_distance, 0, 1e-6);
    }
}

TEST(EllipseTest, Projection) {
    using namespace GraphicsLab;
    using namespace Geometry;

    constexpr int repeat = 100;

    for (int i = 0; i < repeat; i++) {
        BRep::BRepPoint2 center(0);
        double a = Sampler::sampleUniform();
        double b = Sampler::sampleUniform();
        if (a < b) std::swap(a, b);

        const auto allocator = BRep::BRepAllocator::instance();
        const auto ellipse = allocator->alloc_param_pcurve<Ellipse2D>(center, a * BRep::BRepVector2(1.0, 0.0), b * BRep::BRepVector2(0.0, 1.0));
        const BRep::BRepPoint2 test_point = Sampler::sampleUniformVec2();

        auto [ellipse_proj, ellipse_proj_param] = ellipse->projection(test_point, std::nullopt);
        auto [proj, proj_param] = ellipse->ParamCurve2D::projection(test_point, std::nullopt);

        const auto distance = glm::distance(ellipse_proj, proj);
        const auto param_distance = glm::distance(ellipse_proj_param, proj_param);

        const double distance1 = glm::distance(ellipse_proj, test_point);
        const double distance2 = glm::distance(proj, test_point);

        spdlog::debug("projection result: {}, {}", distance1, distance2);
        spdlog::debug("projection param result: {}, {}", ellipse_proj_param, proj_param);

        if (distance1 > distance2) {
            EXPECT_NEAR(distance, 0, 1e-6);
            EXPECT_NEAR(param_distance, 0, 1e-6);
        }
    }
}
