#include "gtest/gtest.h"

#include "numeric/polynomial/polynomial_solver.hpp"
#include "numeric/polynomials.hpp"
#include "utils/sampler.hpp"

using namespace GraphicsLab;
using namespace GraphicsLab::Numeric;

TEST(PolynomialSolverTest, QuadraticTest) {
    using namespace GraphicsLab::Numeric;

    RealPolynomial p1({1.0, 2.0, 1.0});

    auto cr1 = PolynomialSolver::find_complex_roots(p1);
    EXPECT_EQ(cr1.size(), 2);
    EXPECT_NEAR(cr1.front().real(), -1.0, 1e-9);
    EXPECT_NEAR(cr1.back().real(), -1.0, 1e-9);

    auto r1 = PolynomialSolver::find_real_roots_with_multiplicity(p1);

    EXPECT_EQ(r1.size(), 1);
    EXPECT_NEAR(r1.front().first, -1.0, 1e-9);
    EXPECT_EQ(r1.front().second, 2.0);
}

namespace {

constexpr double kTol = 1e-9;

inline void expect_near_unordered(
    std::vector<double> a,
    std::vector<double> b,
    double tol = kTol
) {
    ASSERT_EQ(a.size(), b.size());

    std::sort(a.begin(), a.end());
    std::sort(b.begin(), b.end());

    for (size_t i = 0; i < a.size(); ++i) {
        EXPECT_NEAR(a[i], b[i], tol);
    }
}

} // namespace

TEST(CubicPolynomialSolverTest, OneRealRoot) {
    // x^3 + x + 1 = 0
    RealPolynomial p({1.0, 1.0, 0.0, 1.0});

    auto roots = CubicPolynomialSolver::solve(p);
    ASSERT_EQ(roots.size(), 1u);

    EXPECT_NEAR(p.evaluate(roots[0]), 0.0, kTol);
}

TEST(CubicPolynomialSolverTest, OneRealRoot2) {
    // x^3 + 1 = 0
    RealPolynomial p({1.0, 0.0, 0.0, 1.0});
    auto roots = CubicPolynomialSolver::solve(p);
    ASSERT_EQ(roots.size(), 1u);
    EXPECT_NEAR(p.evaluate(roots[0]), 0.0, kTol);
}

TEST(CubicPolynomialSolverTest, ThreeRealRoots) {
    // (x - 1)(x - 2)(x - 3) = x^3 - 6x^2 + 11x - 6
    RealPolynomial p({-6.0, 11.0, -6.0, 1.0});

    auto roots = CubicPolynomialSolver::solve(p);
    expect_near_unordered(roots, {1.0, 2.0, 3.0});
}

TEST(CubicPolynomialSolverTest, MultipleRoot) {
    // (x - 1)^2 (x + 2) = x^3 + 0x^2 - 3x + 2
    RealPolynomial p({2.0, -3.0, 0.0, 1.0});

    auto roots = CubicPolynomialSolver::solve(p);
    expect_near_unordered(roots, {-2.0, 1.0});
}

TEST(QuarticPolynomialSolverTest, FourRealRoots) {
    // (x - 1)(x - 2)(x - 3)(x - 4)
    // = x^4 - 10x^3 + 35x^2 - 50x + 24
    RealPolynomial p({24.0, -50.0, 35.0, -10.0, 1.0});

    auto roots = QuarticPolynomialSolver::solve(p);
    expect_near_unordered(roots, {1.0, 2.0, 3.0, 4.0});
}

TEST(QuarticPolynomialSolverTest, TwoRealRoots) {
    // (x^2 - 1)(x^2 + 1) = x^4 - 1
    RealPolynomial p({-1.0, 0.0, 0.0, 0.0, 1.0});

    auto roots = QuarticPolynomialSolver::solve(p);
    expect_near_unordered(roots, {-1.0, 1.0});
}

TEST(QuarticPolynomialSolverTest, MultipleRoots) {
    // (x - 1)^4
    RealPolynomial p({1.0, -4.0, 6.0, -4.0, 1.0});

    auto roots = QuarticPolynomialSolver::solve(p);
    expect_near_unordered(roots, {1.0});
}

