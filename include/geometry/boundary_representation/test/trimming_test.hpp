#pragma once
#include "geometry/boundary_representation/constructors/constructors.hpp"
#include "geometry/boundary_representation/topology/trimming_utils.hpp"
#include "geometry/parametric/bezier_curve_2d.hpp"
#include "test_base.hpp"

namespace GraphicsLab::Geometry::BRep {

struct TrimmingTestBase : TestBase {
    [[nodiscard]] std::string test_suite_name() const override {
        return "TrimmingTest";
    }

  protected:
    int ssi_counter = 0;
};

struct TrimmingTest1 : TrimmingTestBase {
    [[nodiscard]] std::string test_case_name() const override {
        return "TrimmingTest1";
    }

    void run_test() override {
        Face *rect = FaceConstructors::plane({0, 0, 0}, {1, 0, 0}, {0, 0, 1});

        auto allocator = BRepAllocator::instance();

        auto param_pc1 =
            allocator->alloc_param_pcurve<BezierCurve2D>(std::vector<BRepPoint2>{{0.8, 0.5}, {0.8, 0.8}, {0.5, 0.8}});
        auto param_pc2 =
            allocator->alloc_param_pcurve<BezierCurve2D>(std::vector<BRepPoint2>{{0.5, 0.8}, {0.2, 0.8}, {0.2, 0.5}});
        auto param_pc3 =
            allocator->alloc_param_pcurve<BezierCurve2D>(std::vector<BRepPoint2>{{0.2, 0.5}, {0.2, 0.2}, {0.5, 0.2}});
        auto param_pc4 =
            allocator->alloc_param_pcurve<BezierCurve2D>(std::vector<BRepPoint2>{{0.5, 0.2}, {0.8, 0.2}, {0.8, 0.5}});

        auto pc1 = TopologyUtils::create_pcurve_from_param_pcurve(param_pc1);
        auto pc2 = TopologyUtils::create_pcurve_from_param_pcurve(param_pc2);
        auto pc3 = TopologyUtils::create_pcurve_from_param_pcurve(param_pc3);
        auto pc4 = TopologyUtils::create_pcurve_from_param_pcurve(param_pc4);

        TrimmingUtils::TrimmingLoop trimming_loop;
        trimming_loop.pcurves = {pc1, pc2, pc3, pc4};
        auto trimming_result = TrimmingUtils::add_trimming_curve(rect, {trimming_loop});

        for (int i = 0; i < trimming_result.size(); i++) {
            faces[std::format("face_{}", i)] = trimming_result[i];
        }

        result = TestResult::Success;
    }
};

} // namespace GraphicsLab::Geometry::BRep

META_REGISTER_TYPE(GraphicsLab::Geometry::BRep::BRepTestRegisterTag, GraphicsLab::Geometry::BRep::TrimmingTest1)
