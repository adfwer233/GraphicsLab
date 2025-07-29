#pragma once

#include "geometry/boundary_representation/constructors/constructors.hpp"
#include "geometry/boundary_representation/intersector/surface_surface_intersection/general_surface_surface_intersection.hpp"
#include "geometry/boundary_representation/intersector/topological_intersection/face_face_intersection.hpp"
#include "geometry/boundary_representation/topology/trimming_utils.hpp"
#include "test_base.hpp"

namespace GraphicsLab::Geometry::BRep {

struct IntersectionTestBase : TestBase {
    void save_ssi_results(const std::vector<SSIResult> &inter_result) {
        for (int i = 0; i < inter_result.size(); i++) {
            param_curve[std::format("ssi{}_curve_{}", ssi_counter, i)] = inter_result[i].inter_curve;
            param_pcurve[std::format("ssi{}_pcurve1_{}", ssi_counter, i)] = inter_result[i].pcurve1;
            param_pcurve[std::format("ssi{}_pcurve2_{}", ssi_counter, i)] = inter_result[i].pcurve2;
        }
        ssi_counter++;
    }

    void save_ffi_results(const std::vector<FFIResult> &inter_results) {
        for (int i = 0; i < inter_results.size(); i++) {
            Curve *curve = TopologyUtils::create_curve_from_param_curve(inter_results[i].curve);
            Edge *edge = TopologyUtils::create_edge_from_curve(curve);
            edge->set_param_range(ParamRange{inter_results[i].curve_range});
            edges[std::format("ffi{}_edge_{}", ffi_counter, i)] = edge;
        }
        ffi_counter++;
    }

    [[nodiscard]] std::string test_suite_name() const override {
        return "IntersectionTest";
    }

  protected:
    int ssi_counter = 0;
    int ffi_counter = 0;
};

struct PlaneIntersection1 : IntersectionTestBase {
    [[nodiscard]] std::string test_case_name() const override {
        return "PlaneIntersection1";
    }

    void run_test() override {
        Face *plane1 = FaceConstructors::plane({-2, 0, -2}, {4, 0, 0}, {0, 0, 4});
        Face *plane2 = FaceConstructors::plane({-1, -1, 0}, {2, 0, 0}, {0, 2, 0});

        auto inter_result = GeneralSurfaceSurfaceIntersection::solve(plane1->geometry()->param_geometry(),
                                                                     plane2->geometry()->param_geometry());

        faces["plane1"] = plane1;
        faces["plane2"] = plane2;

        save_ssi_results(inter_result);

        if (inter_result.size() != 1) {
            result = TestResult::Fail;
            return;
        }

        result = TestResult::Success;
    }
};

struct SpherePlaneIntersection1 : IntersectionTestBase {
    [[nodiscard]] std::string test_case_name() const override {
        return "SpherePlaneIntersection1";
    }

    void run_test() override {
        Face *sphere = FaceConstructors::sphere({0, 0, 0}, 0.9);
        Face *plane2 = FaceConstructors::plane({-1, -1, 0}, {2, 0, 0}, {0, 2, 0});

        auto inter_result = GeneralSurfaceSurfaceIntersection::solve(sphere->geometry()->param_geometry(),
                                                                     plane2->geometry()->param_geometry());

        faces["sphere"] = sphere;
        faces["plane2"] = plane2;

        save_ssi_results(inter_result);

        if (inter_result.size() != 1) {
            result = TestResult::Fail;
            return;
        }

        result = TestResult::Success;
    }
};

struct SpherePlaneIntersection2 : IntersectionTestBase {
    [[nodiscard]] std::string test_case_name() const override {
        return "SpherePlaneIntersection2";
    }

    void run_test() override {
        Face *sphere = FaceConstructors::sphere({0.1, 0.1, 0.1}, 1.0);
        Body *cube = BodyConstructors::cube({0, 0, 0}, {2, 2, 2});
        // Face *plane2 = FaceConstructors::plane({0, 0, 0}, {0, 2, 0}, {0, 0, 2});

        auto cube_faces = TopologyUtils::get_all_faces(cube);

        Face *plane2 = cube_faces[5];
        auto inter_result = GeneralSurfaceSurfaceIntersection::solve(sphere->geometry()->param_geometry(),
                                                                     plane2->geometry()->param_geometry());

        faces["sphere"] = sphere;
        faces["plane2"] = plane2;

        save_ssi_results(inter_result);

        if (inter_result.size() != 1) {
            result = TestResult::Fail;
            return;
        }

        result = TestResult::Success;
    }
};

struct TorusPlaneIntersection1 : IntersectionTestBase {
    [[nodiscard]] std::string test_case_name() const override {
        return "TorusPlaneIntersection1";
    }

    void run_test() override {
        Face *plane = FaceConstructors::plane({-2, 0, -2}, {4, 0, 0}, {0, 0, 4});
        Face *torus = FaceConstructors::torus({0, 0, 0}, 2, 0.5, {0, 1, 0}, {1, 0, 0});

        auto inter_result = GeneralSurfaceSurfaceIntersection::solve(plane->geometry()->param_geometry(),
                                                                     torus->geometry()->param_geometry());

        faces["plane"] = plane;
        faces["torus"] = torus;

        save_ssi_results(inter_result);

        if (inter_result.size() != 5) {
            result = TestResult::Fail;
        }
    }
};

struct TorusPlaneIntersection2 : IntersectionTestBase {
    [[nodiscard]] std::string test_case_name() const override {
        return "TorusPlaneIntersection2";
    }

    void run_test() override {
        Face *plane = FaceConstructors::plane({-4, 0, -4}, {8, 0, 0}, {0, 0, 8});
        Face *torus = FaceConstructors::torus({0, 0, 0}, 2, 0.5, {0, 1, 0}, {1, 0, 0});

        auto inter_result = GeneralSurfaceSurfaceIntersection::solve(plane->geometry()->param_geometry(),
                                                                     torus->geometry()->param_geometry());

        faces["plane"] = plane;
        faces["torus"] = torus;

        save_ssi_results(inter_result);

        if (inter_result.size() != 2) {
            result = TestResult::Fail;
        }
    }
};

struct TorusSplineIntersection : IntersectionTestBase {
    [[nodiscard]] std::string test_case_name() const override {
        return "TorusSplineIntersection";
    }

    void run_test() override {
        Face *spline = FaceConstructors::wave_spline({-4, 0, -4}, {8, 0, 0}, {0, 0, 8}, 5, 5, 1);
        Face *torus = FaceConstructors::torus({0, 0, 0}, 2, 0.5, {0, 1, 0}, {1, 0, 0});

        auto inter_result = GeneralSurfaceSurfaceIntersection::solve(spline->geometry()->param_geometry(),
                                                                     torus->geometry()->param_geometry());

        faces["spline"] = spline;
        faces["torus"] = torus;

        save_ssi_results(inter_result);

        if (inter_result.size() != 2) {
            result = TestResult::Fail;
        }
    }
};

struct TorusExplicitIntersection : IntersectionTestBase {
    [[nodiscard]] std::string test_case_name() const override {
        return "TorusExplicitIntersection";
    }

    void run_test() override {

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
        Face *torus = FaceConstructors::torus({0, 0, 0}, 2, 0.5, {0, 1, 0}, {1, 0, 0});

        auto inter_result = GeneralSurfaceSurfaceIntersection::solve(wave->geometry()->param_geometry(),
                                                                     torus->geometry()->param_geometry());

        faces["wave"] = wave;
        faces["torus"] = torus;

        save_ssi_results(inter_result);

        if (inter_result.size() != 2) {
            result = TestResult::Fail;
        }
    }
};

struct TorusTorusIntersection1 : IntersectionTestBase {
    [[nodiscard]] std::string test_case_name() const override {
        return "TorusTorusIntersection1";
    }

    void run_test() override {
        Face* face1 = FaceConstructors::torus({1, 0, 0}, 1.5, 0.6, {0, 1, 0}, {0, 0, 1});
        Face* face2 = FaceConstructors::torus({-1, 0, 0}, 1.5, 0.6, {0, 1, 0}, {0, 0, 1});

        faces["Face1"] = face1;
        faces["Face2"] = face2;

        auto inter_result = GeneralSurfaceSurfaceIntersection::solve(face1->geometry()->param_geometry(),
                                                             face2->geometry()->param_geometry());

        save_ssi_results(inter_result);
    }
};

struct FaceFaceIntersectionTest1 : IntersectionTestBase {
    [[nodiscard]] std::string test_case_name() const override {
        return "FaceFaceIntersectionTest1";
    }

    void run_test() override {
        auto f = [](GraphicsLab::Geometry::autodiff_vec2 param) -> GraphicsLab::Geometry::autodiff_vec3 {
            GraphicsLab::Geometry::autodiff_vec3 result;

            BRepPoint3 base{-2, 0, -2};
            BRepPoint3 dir1{4, 0, 0};
            BRepPoint3 dir2{0, 0, 4};

            auto u = 2 * 2 * std::numbers::pi * param.x();
            auto v = 2 * 2 * std::numbers::pi * param.y();
            result.x() = base.x + dir1.x * param.x();
            result.y() = 0.3 * sin(u) * sin(v);
            result.z() = base.z + dir2.z * param.y();

            return result;
        };

        // Face *rect = FaceConstructors::plane({-1, 0, -1}, {2, 0, 0}, {0, 0, 2});
        Face *rect = FaceConstructors::explicit_surface(f);
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

        Face *rect2 = FaceConstructors::plane({-3, -3, 0}, {6, 0, 0}, {0, 6, 0});

        for (int i = 0; i < trimming_result.size(); i++) {
            faces[std::format("face_{}", i)] = trimming_result[i];
        }
        faces["rect"] = rect2;

        auto ffi_result = FaceFaceIntersection::solve(trimming_result[1], rect2);

        if (ffi_result.size() != 1) {
            result = TestResult::Fail;
        }
        save_ffi_results(ffi_result);
        result = TestResult::Success;
    }
};

} // namespace GraphicsLab::Geometry::BRep

META_REGISTER_TYPE(GraphicsLab::Geometry::BRep::BRepTestRegisterTag, GraphicsLab::Geometry::BRep::PlaneIntersection1)
META_REGISTER_TYPE(GraphicsLab::Geometry::BRep::BRepTestRegisterTag,
                   GraphicsLab::Geometry::BRep::SpherePlaneIntersection1)
META_REGISTER_TYPE(GraphicsLab::Geometry::BRep::BRepTestRegisterTag,
                   GraphicsLab::Geometry::BRep::SpherePlaneIntersection2)
META_REGISTER_TYPE(GraphicsLab::Geometry::BRep::BRepTestRegisterTag,
                   GraphicsLab::Geometry::BRep::TorusPlaneIntersection1)
META_REGISTER_TYPE(GraphicsLab::Geometry::BRep::BRepTestRegisterTag,
                   GraphicsLab::Geometry::BRep::TorusPlaneIntersection2)
META_REGISTER_TYPE(GraphicsLab::Geometry::BRep::BRepTestRegisterTag,
                   GraphicsLab::Geometry::BRep::TorusSplineIntersection)
META_REGISTER_TYPE(GraphicsLab::Geometry::BRep::BRepTestRegisterTag,
                   GraphicsLab::Geometry::BRep::TorusExplicitIntersection)
META_REGISTER_TYPE(GraphicsLab::Geometry::BRep::BRepTestRegisterTag,
                   GraphicsLab::Geometry::BRep::FaceFaceIntersectionTest1)
