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

TEST(ReflectionTest, TestDeserialization) {
    struct ReflectionStruct {
        int a;
        int b;
        float c;
        REFLECT(Property{"a", &ReflectionStruct::a}, Property{"b", &ReflectionStruct::b}, Property{"c", &ReflectionStruct::c})
    };

    ReflectionStruct reflectionStruct {
        .a = 1,
        .b = 2,
        .c = 3.0f
    };

    auto test = StaticReflect::serialization(reflectionStruct);

    ReflectionStruct deserialization_result{};

    spdlog::info(test.dump());
    StaticReflect::deserialization(deserialization_result, test);

    EXPECT_EQ(reflectionStruct.a, deserialization_result.a);
    EXPECT_EQ(reflectionStruct.b, deserialization_result.b);
    EXPECT_FLOAT_EQ(reflectionStruct.c, deserialization_result.c);
}

TEST(ReflectionTest, TestGLMVectorDeserialization) {
    struct ReflectionStruct {
        glm::vec3 a;
        REFLECT(Property{"a", &ReflectionStruct::a})
    };

    ReflectionStruct reflectionStruct {};
    reflectionStruct.a = {1.0f, 2.0f, 3.0f};

    auto test = StaticReflect::serialization(reflectionStruct);
    spdlog::info(test.dump());

    ReflectionStruct deserialization_result{};
    StaticReflect::deserialization(deserialization_result, test);
    EXPECT_FLOAT_EQ(reflectionStruct.a.x, deserialization_result.a.x);
    EXPECT_FLOAT_EQ(reflectionStruct.a.y, deserialization_result.a.y);
    EXPECT_FLOAT_EQ(reflectionStruct.a.z, deserialization_result.a.z);
}

TEST(ReflectionTest, TestVectorDeserialization) {
    struct ReflectionStruct {
        std::vector<int> a;

        REFLECT(Property{"a", &ReflectionStruct::a})
    };

    ReflectionStruct reflectionStruct {};
    reflectionStruct.a = {1, 2, 3};
    auto test = StaticReflect::serialization(reflectionStruct);

    ReflectionStruct deserialization_result{};
    StaticReflect::deserialization(deserialization_result, test);
    EXPECT_EQ(reflectionStruct.a, deserialization_result.a);
}

TEST(ReflectionTest, TestNestedDeserialization) {
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

    ReflectionStructNested deserialization_result{};
    StaticReflect::deserialization(deserialization_result, test);

    EXPECT_EQ(deserialization_result.data.size(), reflectionStruct.data.size());
}

TEST(ReflectionTest, TestReflectedFunction) {
    struct A {
        int a = 1;
        void update() { a = 2; }

        REFLECT(
            PROPERTY(a, &A::a),
            Method<void, A>{"update", &A::update}
        )
    } a;

    auto tmp = StaticReflect::serialization(a);
    spdlog::info(tmp.dump());

    a.a = 2;
    StaticReflect::deserialization(a, tmp);
    EXPECT_EQ(a.a, 1);

    StaticReflect::call(a, "update");
    EXPECT_EQ(a.a, 2);
}