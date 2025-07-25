#pragma once
#include "geometry/boundary_representation/boolean/boolean.hpp"
#include "geometry/boundary_representation/constructors/constructors.hpp"
#include "test_base.hpp"

namespace GraphicsLab::Geometry::BRep {

struct BooleanTestBase : TestBase {
    [[nodiscard]] std::string test_suite_name() const override {
        return "BooleanTest";
    }
};

struct BreakFaceTest1 : BooleanTestBase {
    [[nodiscard]] std::string test_case_name() const override {
        return "BreakFaceTest1";
    }

    void run_test() override {
        Face *plane1 = FaceConstructors::plane({-2, 0, -2}, {4, 0, 0}, {0, 0, 4});
        Body *body = BodyConstructors::cube({-1, -1, -1}, {1, 1, 1});

        auto results = Boolean::break_face_by_intersection(plane1, body);

        auto cube_faces = TopologyUtils::get_all_faces(body);
        for (int i = 0; i < cube_faces.size(); i++) {
            faces[std::format("cube_face_{}", i)] = cube_faces[i];
        }
        for (int i = 0; i < results.size(); ++i) {
            faces[std::format("face_{}", i)] = results[i];
        }

        result = TestResult::Success;
    }
};
} // namespace GraphicsLab::Geometry::BRep