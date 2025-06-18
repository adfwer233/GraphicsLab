#include "language/crtp/auto_serialize_singleton.hpp"

#include "gtest/gtest.h"

TEST(AutoSerializeSingletonTest, TestAutoSerializeSingleton) {
    struct TestSingleton : AutoSerializeSingleton<TestSingleton, "TestSingleton"> {
        int value = 42;

        REFLECT(Property{"value", &TestSingleton::value})
    };

    auto &instance = TestSingleton::instance();
}