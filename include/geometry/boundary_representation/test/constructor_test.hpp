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

META_REGISTER_TYPE(GraphicsLab::Geometry::BRep::BRepTestRegisterTag, GraphicsLab::Geometry::BRep::CylinderFaceConstructorTest)
META_REGISTER_TYPE(GraphicsLab::Geometry::BRep::BRepTestRegisterTag, GraphicsLab::Geometry::BRep::SphereConstructorTest)
