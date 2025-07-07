#include "gtest/gtest.h"

#include "numeric/polynomials.hpp"
#include "utils/sampler.hpp"

TEST(PolynomialTest, ConstructorTest1) {
    GraphicsLab::RealPolynomial polynomial{1.0, 2.0, 3.0};
    double res = polynomial.evaluate(2);
    EXPECT_DOUBLE_EQ(res, 17.0);
}

TEST(PolynomialTest, ConverterTest1) {
    GraphicsLab::BernsteinPolynomial polynomial{1.0, 2.0, 3.0};
    GraphicsLab::RealPolynomial polynomial_power = polynomial.convert_to_power_basis();

    EXPECT_DOUBLE_EQ(polynomial.evaluate(0.5), polynomial_power.evaluate(0.5));
    EXPECT_DOUBLE_EQ(polynomial.evaluate(0.0), polynomial_power.evaluate(0.0));
    EXPECT_DOUBLE_EQ(polynomial.evaluate(1.0), polynomial_power.evaluate(1.0));
}

TEST(PolynomialTest, ConverterBatchTest) {
    using namespace GraphicsLab;

    for (int repeat = 0; repeat < 10; ++repeat) {
        for (int deg = 3; deg <= 10; deg ++) {
            std::vector<double> coeff;

            for (int i = 0; i <= deg; i++) {
                coeff.push_back(Sampler::sampleUniform());
            }

            BernsteinPolynomial polynomial_bernstein{coeff};
            RealPolynomial real_polynomial = polynomial_bernstein.convert_to_power_basis();

            for (int i = 0; i <= 10; i++) {
                double x = Sampler::sampleUniform();
                EXPECT_TRUE(std::abs(polynomial_bernstein.evaluate(x) - real_polynomial.evaluate(x)) < 1e-6);
            }
        }
    }

}