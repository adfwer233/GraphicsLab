#pragma once
#include "../../../cmake-build-relwithdebinfo/_deps/cpptrace-src/include/cpptrace/cpptrace.hpp"
#include "real_polynomial.hpp"
#include "spdlog/spdlog.h"

#include <cmath>

namespace GraphicsLab::Numeric {

template<typename ValueType>
struct BezierPolynomial {
    using ParamType = double;

    explicit BezierPolynomial(const std::vector<ValueType>& coefficients): coefficients_(coefficients) {}

    ValueType evaluate(ParamType param) {
        double h = 1.0;
        ValueType result = coefficients_[0];
        double t = param;
        double u = 1 - t;
        uint32_t n = coefficients_.size() - 1;
        uint32_t n1 = n + 1;
        if (param <= 0.5) {
            u = t / u;
            for (uint32_t k = 1; k <= n; k++) {
                h = h * u * (n1 - k);
                h = h / (k + h);
                double h1 = 1 - h;
                result = result * h1 + coefficients_[k] * h;
            }
        } else {
            u = u / t;
            for (uint32_t k = 1; k <= n; k++) {
                h = h * (n1 - k);
                h = h / (k * u + h);
                double h1 = 1 - h;
                result = result * h1 + coefficients_[k] * h;
            }
        }
        return result;
    }

    BezierPolynomial derivative() const {
        std::vector<ValueType> coefficients(coefficients_.size() - 1);
        int n = coefficients_.size() - 1;
        for (int i = 0; i < n - 1; i++) {
            ValueType coeff = coefficients_[i + 1] - coefficients_[i];
            coefficients[i] = coeff * static_cast<ParamType>(n);
        }
        return BezierPolynomial{coefficients};
    }

    BezierPolynomial operator+(const BezierPolynomial& rhs) const {
        if (coefficients_.size() != rhs.coefficients_.size()) {
            throw cpptrace::runtime_error("[BezierPolynomial '+'] different degrees");
        }
        std::vector<ValueType> result(coefficients_.size());
        for (int i = 0; i < coefficients_.size(); ++i) {
            result[i] = coefficients_[i] + rhs.coefficients_[i];
        }
        return BezierPolynomial{result};
    }

    BezierPolynomial operator-(const BezierPolynomial& rhs) const {
        if (coefficients_.size() != rhs.coefficients_.size()) {
            throw cpptrace::runtime_error("[BezierPolynomial '+'] different degrees");
        }
        std::vector<ValueType> result(coefficients_.size());
        for (int i = 0; i < coefficients_.size(); ++i) {
            result[i] = coefficients_[i] - rhs.coefficients_[i];
        }
        return BezierPolynomial{result};
    }

private:
    std::vector<ValueType> coefficients_;
};

}