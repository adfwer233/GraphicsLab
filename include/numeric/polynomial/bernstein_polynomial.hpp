#pragma once
#include "real_polynomial.hpp"
#include "spdlog/spdlog.h"

#include <cmath>

namespace GraphicsLab {


struct BernsteinPolynomial {
    BernsteinPolynomial() = default;

    BernsteinPolynomial(const std::vector<double>& coeffs)
        : coefficients_(coeffs) {}

    BernsteinPolynomial(std::initializer_list<double> coeffs)
        : coefficients_(coeffs) {}

    // Degree of the polynomial
    size_t degree() const {
        return coefficients_.empty() ? 0 : coefficients_.size() - 1;
    }

    // Evaluate using de Casteljau's algorithm
    double evaluate(double x) const {
        std::vector<double> temp = coefficients_;
        size_t n = degree();
        for (size_t k = 1; k <= n; ++k) {
            for (size_t i = 0; i <= n - k; ++i) {
                temp[i] = (1 - x) * temp[i] + x * temp[i + 1];
            }
        }
        return temp[0];
    }

    // Convert to RealPolynomial (power basis)
    RealPolynomial convert_to_power_basis() const {
        size_t n = degree();
        std::vector<double> result(n + 1, 0.0);
        for (size_t i = 0; i <= n; ++i) {
            double coeff = coefficients_[i];
            for (size_t j = 0; j <= n - i; ++j) {
                double temp = coeff * binomial(n - i, j) * std::pow(-1.0, j) * binomial(n, i);
                result[j + i] += temp;
            }
        }
        return RealPolynomial(result);
    }

private:
    /**
     * coefficient of binom(n, i)(1 - x)^{n - i} x^{i}
     */
    std::vector<double> coefficients_;

    static double binomial(size_t n, size_t k) {
        if (k > n) return 0;
        if (k == 0 || k == n) return 1;
        double res = 1;
        for (size_t i = 1; i <= k; ++i)
            res = res * (n - (k - i)) / i;
        return res;
    }
};

}