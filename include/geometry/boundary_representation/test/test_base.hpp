#pragma once

#include "geometry/boundary_representation/brep_definition.hpp"
#include "language/meta_programming/type_register/type_register.hpp"

namespace GraphicsLab::Geometry::BRep {

/**
 * @brief Base class for intersection test cases.
 *
 * Test cases here are used to define the test data. We will use the in GTest project for auto check and
 * in the main program for visualization
 */
struct TestBase {
    enum class TestResult {
        Success,
        Fail,
        Skip
    };

    TestResult result = TestResult::Success;

    virtual ~TestBase() = default;

    std::map<std::string, Edge *> edges;
    std::map<std::string, Face *> faces;

    std::map<std::string, ParamCurve3D *> param_curve;
    std::map<std::string, ParamCurve2D *> param_pcurve;

    virtual void run_test() = 0;
    [[nodiscard]] virtual std::string test_suite_name() const = 0;
    [[nodiscard]] virtual std::string test_case_name() const = 0;
};

struct BRepTestRegisterTag {};

} // namespace GraphicsLab::Geometry::BRep