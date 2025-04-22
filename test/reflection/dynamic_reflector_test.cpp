#include "gtest/gtest.h"

#include "language/reflection/reflectors.hpp"

#include "glm/vec3.hpp"
#include "spdlog/spdlog.h"

TEST(DynamicRelflectionTest, TestReflectFunction) {
    struct TestStruct: public Reflectable {
        int a = 0;

        void update() {
            a = 1;
        }

        ReflectDataType reflect() override {
            return {
                {"a", TypeErasedValue(&a)},
                {"update", TypeErasedValue(&TestStruct::update, this)}
            };
        }
    };

    TestStruct testStruct;

    auto reflection = testStruct.reflect();

    reflection["update"].call_func();

    EXPECT_EQ(testStruct.a, 1);
}

TEST(DynamicReflectionTest, TestReflectFunctionWithParameter) {
    struct TestStruct: public Reflectable {
        int a = 0;

        void update(const int c) {
            a = c;
        }

        ReflectDataType reflect() override {
            return {
                    {"a", TypeErasedValue(&a)},
                    {"update", TypeErasedValue(&TestStruct::update, this, {1}, {"a"})}
            };
        }
    };

    TestStruct testStruct;
    auto reflection = testStruct.reflect();

    reflection["update"].call_func_with_param({12});
    EXPECT_EQ(testStruct.a, 12);
}

TEST(DynamicReflectionTest, TestReflectFunctionWithMultipleParameter) {
    struct TestStruct: public Reflectable {
        int a = 0;

        void update(const int b, const int c) {
            a = b * c;
        }

        ReflectDataType reflect() override {
            return {
                        {"a", TypeErasedValue(&a)},
                        {"update", TypeErasedValue(&TestStruct::update, this, {1, 2}, {"b", "c"})}
            };
        }
    };

    TestStruct testStruct;
    auto reflection = testStruct.reflect();

    reflection["update"].call_func_with_param({3, 4});
    EXPECT_EQ(testStruct.a, 12);
}

TEST(DynamicReflectionTest, TestAddField) {
    struct TestStruct: public Reflectable {
        ReflectDataType reflect() override {
            return {};
        }
    };

    TestStruct testStruct;

    testStruct.addField<int>("a", 10);
    const int res = testStruct.getField<int>("a");

    EXPECT_EQ(res, 10);
}