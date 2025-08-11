#pragma once

#include <algorithm>
#include <vector>

namespace GraphicsLab {

struct RealPolynomial {
    /**
     * Constructors
     */
    RealPolynomial() = default;

    explicit RealPolynomial(const std::vector<double> &coeffs) : coefficients_(coeffs) {
    }

    RealPolynomial(std::initializer_list<double> coeffs) : coefficients_(coeffs) {
    }

    [[nodiscard]] double evaluate(double x) const {
        double result = 0.0;
        double xn = 1.0; // x^0
        for (double c : coefficients_) {
            result += c * xn;
            xn *= x;
        }
        return result;
    }

    /**
     * Compute the derivative of the polynomial
     */
    [[nodiscard]] RealPolynomial derivative() const {
        if (coefficients_.size() <= 1) {
            // Derivative of a constant is zero
            return RealPolynomial({0.0});
        }

        std::vector<double> deriv(coefficients_.size() - 1);
        for (size_t i = 1; i < coefficients_.size(); ++i) {
            deriv[i - 1] = coefficients_[i] * static_cast<double>(i);
        }
        return RealPolynomial(deriv);
    }

    // Addition
    RealPolynomial operator+(const RealPolynomial &other) const {
        size_t n = std::max(coefficients_.size(), other.coefficients_.size());
        std::vector<double> result(n, 0.0);
        for (size_t i = 0; i < n; ++i) {
            if (i < coefficients_.size())
                result[i] += coefficients_[i];
            if (i < other.coefficients_.size())
                result[i] += other.coefficients_[i];
        }
        return RealPolynomial(result);
    }

    // Subtraction
    RealPolynomial operator-(const RealPolynomial &other) const {
        size_t n = std::max(coefficients_.size(), other.coefficients_.size());
        std::vector<double> result(n, 0.0);
        for (size_t i = 0; i < n; ++i) {
            if (i < coefficients_.size())
                result[i] += coefficients_[i];
            if (i < other.coefficients_.size())
                result[i] -= other.coefficients_[i];
        }
        return RealPolynomial(result);
    }

    // Multiplication
    RealPolynomial operator*(const RealPolynomial &other) const {
        size_t n = coefficients_.size() + other.coefficients_.size() - 1;
        std::vector<double> result(n, 0.0);
        for (size_t i = 0; i < coefficients_.size(); ++i)
            for (size_t j = 0; j < other.coefficients_.size(); ++j)
                result[i + j] += coefficients_[i] * other.coefficients_[j];
        return RealPolynomial(result);
    }

    [[nodiscard]] size_t degree() const {
        return coefficients_.empty() ? 0 : coefficients_.size() - 1;
    }

    [[nodiscard]] std::vector<double> get_coefficients() const {
        return coefficients_;
    }

    // Addition with scalar (p + scalar)
    RealPolynomial operator+(double scalar) const {
        std::vector<double> result = coefficients_;
        if (result.empty())
            result.push_back(scalar);
        else
            result[0] += scalar;
        return RealPolynomial(result);
    }

    // Subtraction with scalar (p - scalar)
    RealPolynomial operator-(double scalar) const {
        std::vector<double> result = coefficients_;
        if (result.empty())
            result.push_back(-scalar);
        else
            result[0] -= scalar;
        return RealPolynomial(result);
    }

    // Multiplication with scalar (p * scalar)
    RealPolynomial operator*(double scalar) const {
        std::vector<double> result = coefficients_;
        for (double &c : result)
            c *= scalar;
        return RealPolynomial(result);
    }

    // Division by scalar (p / scalar)
    RealPolynomial operator/(double scalar) const {
        std::vector<double> result = coefficients_;
        for (double &c : result)
            c /= scalar;
        return RealPolynomial(result);
    }

    friend RealPolynomial operator+(double scalar, const RealPolynomial &poly) {
        return poly + scalar;
    }

    friend RealPolynomial operator-(double scalar, const RealPolynomial &poly) {
        std::vector<double> result = poly.coefficients_;
        if (result.empty())
            result.push_back(scalar);
        else
            result[0] = scalar - result[0];
        for (size_t i = 1; i < result.size(); ++i)
            result[i] = -result[i];
        return RealPolynomial(result);
    }

    friend RealPolynomial operator*(double scalar, const RealPolynomial &poly) {
        return poly * scalar;
    }

  private:
    std::vector<double> coefficients_;
};

} // namespace GraphicsLab