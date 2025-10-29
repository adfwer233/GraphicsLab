#pragma once
#include "geometry/boundary_representation/boolean/boolean.hpp"
#include "geometry/boundary_representation/constructors/constructors.hpp"
#include "test_base.hpp"

namespace GraphicsLab::Geometry::BRep {
struct ConstructorTestBase : TestBase {
    [[nodiscard]] std::string test_suite_name() const override {
        return "ConstructorTest";
    }
};

struct CylinderFaceConstructorTest : ConstructorTestBase {
    [[nodiscard]] std::string test_case_name() const override {
        return "CylinderFaceConstructorTest";
    }

    void run_test() override {
        auto cylinder = BRep::FaceConstructors::cylinder({0, 0, 0}, {0, 1, 0}, {0.5, 0, 0}, 2);
        faces["cylinder"] = cylinder;
    }
};

struct CylinderBodyConstructorTest : ConstructorTestBase {
    [[nodiscard]] std::string test_case_name() const override {
        return "CylinderBodyConstructorTest";
    }

    void run_test() override {
        auto cylinder = BRep::BodyConstructors::cylinder({0, 0, 0}, {0, 1, 0}, {0.5, 0, 0}, 2);

        auto cylinder_faces = BRep::TopologyUtils::get_all_faces(cylinder);

        for (size_t i = 0; i < cylinder_faces.size(); i++) {
            faces[std::format("cylinder_{}", i)] = cylinder_faces[i];
        }
    }
};

struct TrimmedTorusConstructorTest : ConstructorTestBase {
    [[nodiscard]] std::string test_case_name() const override {
        return "TrimmedTorusConstructorTest";
    }

    void run_test() override {
        auto allocator = BRepAllocator::instance();
        auto torus = FaceConstructors::torus({0, 0, 0}, 1.5, 0.5, {0, 1, 0}, {1.0, 0.0, 0.0});

        auto param_pcurve1 = allocator->alloc_param_pcurve<StraightLine2D>(BRepPoint2{0.2, 0.0}, BRepPoint2{2.2, 3.0});
        auto param_pcurve2 = allocator->alloc_param_pcurve<StraightLine2D>(BRepPoint2{2.0, 3.0}, BRepPoint2{0.0, 0.0});
        auto pc1 = TopologyUtils::create_pcurve_from_param_pcurve(param_pcurve1);
        auto pc2 = TopologyUtils::create_pcurve_from_param_pcurve(param_pcurve2);

        auto param_curve1 =
            TopologyUtils::create_param_curve_from_pcurve(torus->geometry()->param_geometry(), param_pcurve1, 256, 32);
        auto parma_curve2 =
            TopologyUtils::create_param_curve_from_pcurve(torus->geometry()->param_geometry(), param_pcurve2, 256, 32);

        auto curve1 = TopologyUtils::create_curve_from_param_curve(param_curve1);
        auto curve2 = TopologyUtils::create_curve_from_param_curve(parma_curve2);

        auto edge1 = TopologyUtils::create_edge_from_curve(curve1);
        auto edge2 = TopologyUtils::create_edge_from_curve(curve2);

        auto coedge1 = TopologyUtils::create_coedge_from_edge(edge1);
        auto coedge2 = TopologyUtils::create_coedge_from_edge(edge2);

        coedge1->set_geometry(pc1);
        coedge2->set_geometry(pc2);

        auto lp1 = TopologyUtils::create_loop_from_coedge(coedge1);
        auto lp2 = TopologyUtils::create_loop_from_coedge(coedge2);

        lp1->set_next(lp2);

        lp1->set_face(torus);
        lp2->set_face(torus);

        torus->set_loop(lp1);

        auto f = [](GraphicsLab::Geometry::autodiff_vec2 param) -> GraphicsLab::Geometry::autodiff_vec3 {
            GraphicsLab::Geometry::autodiff_vec3 result;

            BRepPoint3 base{-4, 0, -4};
            BRepPoint3 dir1{8, 0, 0};
            BRepPoint3 dir2{0, 0, 8};

            auto u = 2 * 2 * std::numbers::pi * param.x();
            auto v = 2 * 2 * std::numbers::pi * param.y();
            result.x() = base.x + dir1.x * param.x();
            result.y() = 0.3 * sin(u) * sin(v);
            result.z() = base.z + dir2.z * param.y();

            return result;
        };

        Face *wave = FaceConstructors::explicit_surface(f);

        auto inter_results = FaceFaceIntersection::solve(torus, wave, false);

        edges["e1"] = edge1;
        edges["e2"] = edge2;

        faces["torus"] = torus;
        faces["wave"] = wave;

        int ffi_counter = 0;
        for (int i = 0; i < inter_results.size(); i++) {
            Curve *curve = TopologyUtils::create_curve_from_param_curve(inter_results[i].curve);
            Edge *edge = TopologyUtils::create_edge_from_curve(curve);
            edge->set_param_range(ParamRange{inter_results[i].curve_range});
            bool valid = inter_results[i].in_face1 and inter_results[i].in_face2;
            param_pcurve[std::format("ffi{}_pcurve1_{}_{}", ffi_counter, i, valid ? "valid" : "dropped")] =
                inter_results[i].pcurve1;

            // if (i == 2) {
            //     if (auto bspline = dynamic_cast<BSplineCurve2D*>(inter_results[i].pcurve1)) {
            //         if (not bspline->is_in_bezier_form()) bspline->insert_all_knots_to_bezier_form();
            //         auto beziers = bspline->convert_to_bezier();
            //         auto vec = new std::vector<BezierCurve2D>(std::move(beziers));
            //         for (int j = 0; j < vec->size(); j++) {
            //             param_pcurve[std::format("bezier_{}", j)] = &((*vec)[j]);
            //         }
            //     }
            //
            edges[std::format("ffi{}_edge_{}_{}", ffi_counter, i, valid ? "valid" : "dropped")] = edge;
            //
            //     break;
            // }
        }
        ffi_counter++;
    }
};

struct SphereConstructorTest : ConstructorTestBase {
    [[nodiscard]] std::string test_case_name() const override {
        return "SphereConstructorTest";
    }

    void run_test() override {
        auto sphere = BRep::FaceConstructors::sphere({0, 0, 0}, 1);
        faces["sphere"] = sphere;
    }
};

} // namespace GraphicsLab::Geometry::BRep

META_REGISTER_TYPE(GraphicsLab::Geometry::BRep::BRepTestRegisterTag,
                   GraphicsLab::Geometry::BRep::CylinderFaceConstructorTest)
META_REGISTER_TYPE(GraphicsLab::Geometry::BRep::BRepTestRegisterTag,
                   GraphicsLab::Geometry::BRep::CylinderBodyConstructorTest)
META_REGISTER_TYPE(GraphicsLab::Geometry::BRep::BRepTestRegisterTag, GraphicsLab::Geometry::BRep::SphereConstructorTest)
META_REGISTER_TYPE(GraphicsLab::Geometry::BRep::BRepTestRegisterTag,
                   GraphicsLab::Geometry::BRep::TrimmedTorusConstructorTest)
