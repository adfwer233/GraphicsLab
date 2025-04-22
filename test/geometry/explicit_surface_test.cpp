#include "gtest/gtest.h"

#include "spdlog/spdlog.h"

#include "geometry/parametric/explicit_surface.hpp"

#include "geometry/autodiff/autodiff.hpp"


TEST(TestAutodiff, ReverseSin) {
    autodiff::var x(1.0);
    auto y = sin(x);
}