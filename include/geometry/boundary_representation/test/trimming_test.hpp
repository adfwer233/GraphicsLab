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

struct TurningTest1 : TrimmingTestBase {
    [[nodiscard]] std::string test_case_name() const override {
        return "TurningTest1";
    }

    void run_test() override {
        Face *face1 = FaceConstructors::torus({1, 0, 0}, 1.5, 0.6, {0, 1, 0}, {0, 0, 1});
        Face *face2 = FaceConstructors::torus({-1, 0, 0}, 1.5, 0.6, {0, 1, 0}, {0, 0, 1});

        faces["Face1"] = face1;
        faces["Face2"] = face2;

        auto inter_result = GeneralSurfaceSurfaceIntersection::solve(face1->geometry()->param_geometry(),
                                                                     face2->geometry()->param_geometry());

        for (int i = 0; auto inter : inter_result) {
            auto [tn, bd] = WNTrim::turning_number(inter.pcurve1);

            auto [wn, bd2] = WNTrim::winding_number(inter.pcurve1, BRepPoint2{0, 0});
            if (auto bspline = dynamic_cast<BSplineCurve2D *>(inter.pcurve1)) {
                if (not bspline->is_in_bezier_form())
                    bspline->insert_all_knots_to_bezier_form();
                auto bezier_curves = bspline->convert_to_bezier();
                if (i == 1) {
                    for (int j = 0; j < bezier_curves.size(); j++) {
                        auto c = new BezierCurve2D(bezier_curves[j]);
                        auto start_param = bspline->knots_[j * bspline->degree_ + 1];
                        auto end_param = bspline->knots_[(j + 1) * bspline->degree_ + 1];

                        // for (auto& pt: c->control_points_) {
                        //     pt = pt / (end_param - start_param) / 5.0;
                        // }
                        // c->update_bounds();

                        auto p1 = c->evaluate(0);
                        auto p2 = c->evaluate(0.5);
                        auto p3 = c->evaluate(1);

                        spdlog::info("{}, {}", start_param, end_param);

                        param_pcurve[std::format("dpc_{}_{}", i, j)] = c;
                    }
                }
            }

            spdlog::info("turning number {}, winding number {}", tn, wn);
            i++;
        }
    }
};

} // namespace GraphicsLab::Geometry::BRep

META_REGISTER_TYPE(GraphicsLab::Geometry::BRep::BRepTestRegisterTag, GraphicsLab::Geometry::BRep::TrimmingTest1)
META_REGISTER_TYPE(GraphicsLab::Geometry::BRep::BRepTestRegisterTag, GraphicsLab::Geometry::BRep::TurningTest1)
