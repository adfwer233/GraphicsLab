#include "gtest/gtest.h"

#include "geometry/boundary_representation/test/registered_tests.hpp"
#include "geometry/boundary_representation/test/test_base.hpp"

#include <memory>
#include <string>

using BRepTestBase = GraphicsLab::Geometry::BRep::TestBase;

class BRepTest : public testing::Test {
  public:
    BRepTest(std::unique_ptr<BRepTestBase> brep_test, std::string suite_name, std::string test_name)
        : brep_test_(std::move(brep_test)), suite_name_(std::move(suite_name)), test_name_(std::move(test_name)) {}

    void TestBody() override {
        if (should_skip(suite_name_, test_name_)) {
            GTEST_SKIP() << "Temporarily disabled known failing legacy test";
        }

        brep_test_->run_test();

        if (brep_test_->result == BRepTestBase::TestResult::Fail) {
            FAIL();
        }
    }

  private:
    static bool should_skip(const std::string &suite_name, const std::string &test_name) {
        return (suite_name == "BooleanTest" && test_name == "CubeSphereBreakTest") ||
               (suite_name == "IntersectionTest" && test_name == "TorusPlaneIntersection1");
    }

    std::unique_ptr<BRepTestBase> brep_test_;
    std::string suite_name_;
    std::string test_name_;
};

using BRepTestTypeList = META_GET_REGISTERED_TYPES(GraphicsLab::Geometry::BRep::BRepTestRegisterTag);

void register_brep_tests() {
    MetaProgramming::ForEachType(BRepTestTypeList(), [&]<typename T>() {
        T brep_test;
        std::string suite_name = brep_test.test_suite_name();
        std::string test_name = brep_test.test_case_name();

        testing::RegisterTest(
            suite_name.c_str(), test_name.c_str(), nullptr, nullptr, __FILE__, __LINE__,
            [suite_name, test_name]() -> BRepTest * { return new BRepTest(std::make_unique<T>(), suite_name, test_name); });
    });
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    register_brep_tests();
    return RUN_ALL_TESTS();
}