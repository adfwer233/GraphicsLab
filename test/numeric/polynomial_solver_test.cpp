#include "gtest/gtest.h"

#include "numeric/polynomial/polynomial_solver.hpp"
#include "numeric/polynomials.hpp"
#include "utils/sampler.hpp"

TEST(PolynomialSolverTest, QuadraticTest) {
    using namespace GraphicsLab::Numeric;

    GraphicsLab::RealPolynomial p1({1.0, 2.0, 1.0});

    auto cr1 = PolynomialSolver::find_complex_roots(p1);
    EXPECT_EQ(cr1.size(), 2);
    EXPECT_NEAR(cr1.front().real(), -1.0, 1e-9);
    EXPECT_NEAR(cr1.back().real(), -1.0, 1e-9);

    auto r1 = PolynomialSolver::find_real_roots_with_multiplicity(p1);

    EXPECT_EQ(r1.size(), 1);
    EXPECT_NEAR(r1.front().first, -1.0, 1e-9);
    EXPECT_EQ(r1.front().second, 2.0);
    int x = 0;
}