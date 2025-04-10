#include "gtest/gtest.h"

#include "language/reflection/custom_serizalization.hpp"
#include "language/reflection/static_reflector.hpp"

#include "glm/vec3.hpp"
#include "spdlog/spdlog.h"

TEST(ReflectionTest, TestSerialization) {

    struct ReflectionStruct {
        int a;
        int b;

        REFLECT(Property{"a", &ReflectionStruct::a}, Property{"b", &ReflectionStruct::b})
    };

    ReflectionStruct reflectionStruct {
        .a = 1,
        .b = 2
    };

    auto test = StaticReflect::serialization(reflectionStruct);

    EXPECT_EQ(test.dump(), R"({"a":1,"b":2})");
}

TEST(ReflectionTest, TestVectorSerialization) {
    struct ReflectionStruct {
        std::vector<int> a;

        REFLECT(Property{"a", &ReflectionStruct::a})
    };

    ReflectionStruct reflectionStruct {};

    reflectionStruct.a = {1, 2, 3};

    auto test = StaticReflect::serialization(reflectionStruct);
    EXPECT_EQ(test.dump(), R"({"a":[1,2,3]})");
}

TEST(ReflectionTest, TestGLMVectorSerialization) {
    struct ReflectionStruct {
        glm::vec3 a;
        REFLECT(Property{"a", &ReflectionStruct::a})
    };

    ReflectionStruct reflectionStruct {};
    reflectionStruct.a = {1.0f, 2.0f, 3.0f};

    auto test = StaticReflect::serialization(reflectionStruct);
    EXPECT_EQ(test.dump(), R"({"a":{"x":1.0,"y":2.0,"z":3.0}})");
}

TEST(ReflectionTest, TestNestedSerialization) {
    struct A {
        int a = 1;
        double b = 2.0;

        REFLECT(Property{"a", &A::a}, Property{"b", &A::b})
    };
    struct ReflectionStructNested {
        std::vector<A> data;

        REFLECT(Property{"data", &ReflectionStructNested::data})
    };

    ReflectionStructNested reflectionStruct {};

    for (int i = 0; i < 3; i++) {
        reflectionStruct.data.push_back({});
    }
    auto test = StaticReflect::serialization(reflectionStruct);
    spdlog::info(test.dump());
    EXPECT_EQ(test.dump(), R"({"data":[{"a":1,"b":2.0},{"a":1,"b":2.0},{"a":1,"b":2.0}]})");
}