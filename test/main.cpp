#include "gtest/gtest.h"

#include "geometry/boundary_representation/test/registered_tests.hpp"
#include "geometry/boundary_representation/test/test_base.hpp"

using BRepTestBase = GraphicsLab::Geometry::BRep::TestBase;

class BRepTest : public testing::Test {
public:
    explicit BRepTest(BRepTestBase* brep_test) : brep_test_(brep_test) {}
    void TestBody() override {
        brep_test_->run_test();

        if (brep_test_->result == BRepTestBase::TestResult::Fail) {
            FAIL();
        }
    }

private:
    BRepTestBase* brep_test_;
};

using BRepTestTypeList = META_GET_REGISTERED_TYPES(GraphicsLab::Geometry::BRep::BRepTestRegisterTag);

void register_brep_tests() {
    MetaProgramming::ForEachType(BRepTestTypeList(), [&]<typename T>() {
        T brep_test;
        std::string test_suite_name = brep_test.test_suite_name();
        std::string test_name = brep_test.test_case_name();

        testing::RegisterTest(test_suite_name.c_str(), test_name.c_str(),
            nullptr, nullptr, __FILE__, __LINE__,
            [=]() -> BRepTest* {return new BRepTest(new T()); });
    });
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    register_brep_tests();
    return RUN_ALL_TESTS();
}