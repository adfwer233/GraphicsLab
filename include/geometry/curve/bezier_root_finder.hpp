#pragma once

#include <cstdint>
#include <vector>

/**
 * @brief class to get the zeroes of a bezier polynomial
 *
 * @reference Revisiting the problem of zeros of univariate scalar Béziers
 */
struct BezierRootFinder {
  private:
    uint32_t degree;

    std::vector<double> coefficients_;
    std::vector<double> derivative_coefficients;

    /**
     * @brief evaluate the bezier polynomials
     * @return
     */
    double evaluate(double param);

    /**
     * @brief evaluate the derivative
     * @return
     */
    double evaluate_derivative(double param);

    double newton_method(double start);

    std::pair<std::vector<double>, std::vector<double>> subdivide(const std::vector<double> &c, float param) const;

    std::vector<double> get_roots(std::vector<double> &c, double a, double b);

  public:
    explicit BezierRootFinder(std::vector<double> &&coefficients) : coefficients_(coefficients) {
        degree = coefficients.size() - 1;
        for (int i = 1; i < degree; i++) {
            derivative_coefficients.push_back(degree * (coefficients[i] - coefficients[i - 1]));
        }
    }

    std::vector<double> get_roots();
};