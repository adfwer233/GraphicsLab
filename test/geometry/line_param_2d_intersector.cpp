#include "geometry/boundary_representation/allocator/brep_allocator.hpp"

#include "gtest/gtest.h"

#include "geometry/parametric_intersector/curve_curve_intersector_2d/bezier_bezier_intersection_2d.hpp"
#include "geometry/parametric_intersector/curve_curve_intersector_2d/line_param_intersector_2d.hpp"
#include "utils/sampler.hpp"

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