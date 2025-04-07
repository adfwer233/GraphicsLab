#pragma once

#include <cstdint>
#include <iterator>
#include <vector>

namespace GraphicsLab::Geometry {

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
    double evaluate(double param) const {
        double h = 1.0;
        double result = coefficients_[0];
        double t = param;
        double u = 1 - t;
        uint32_t n = coefficients_.size() - 1;
        uint32_t n1 = n + 1;
        if (param <= 0.5) {
            u = t / u;
            for (int k = 1; k <= n; k++) {
                h = h * u * (n1 - k);
                h = h / (k + h);
                double h1 = 1 - h;
                result = result * h1 + coefficients_[k] * h;
            }
        } else {
            u = u / t;
            for (int k = 1; k <= n; k++) {
                h = h * (n1 - k);
                h = h / (k * u + h);
                double h1 = 1 - h;
                result = result * h1 + coefficients_[k] * h;
            }
        }
        return result;
    }

    /**
     * @brief evaluate the derivative
     * @return
     */
    double evaluate_derivative(double param) const {
        double h = 1.0;
        double result = derivative_coefficients[0];
        double t = param;
        double u = 1 - t;
        uint32_t n = derivative_coefficients.size() - 1;
        uint32_t n1 = n + 1;
        if (param <= 0.5) {
            u = t / u;
            for (int k = 1; k <= n; k++) {
                h = h * u * (n1 - k);
                h = h / (k + h);
                double h1 = 1 - h;
                result = result * h1 + derivative_coefficients[k] * h;
            }
        } else {
            u = u / t;
            for (int k = 1; k <= n; k++) {
                h = h * (n1 - k);
                h = h / (k * u + h);
                double h1 = 1 - h;
                result = result * h1 + derivative_coefficients[k] * h;
            }
        }
        return result;
    }

    double newton_method(double start) const {
        double xn = start;
        double xn1 = xn;
        double fn = 0, fn1 = 0, dfn = 0, delta = 0;
        double precision = 1e-8, min_deltax = 1e-8;
        int step = 0;
        int max_iterations = 100;
        do {
            fn = evaluate(xn);
            dfn = evaluate_derivative(xn);

            if (fabs(dfn) < 1e-6) {
                return fabs(fn) > precision ? NAN : xn;
            }
            xn1 = xn - fn / dfn;
            fn1 = evaluate(xn1);

            delta = fabs(xn1 - xn);
            step++;
            if (step > max_iterations) {
                return fabs(fn1) > precision ? NAN : xn;
            }
            xn = xn1;

        } while (fabs(fn1) > precision || delta > min_deltax);

        return xn1;
    }

    std::pair<std::vector<double>, std::vector<double>> subdivide(const std::vector<double> &c, float param) const {
        size_t n = c.size() - 1;

        std::vector current(c);
        std::vector<double> lhs, rhs;
        lhs.reserve(n);
        rhs.reserve(n);

        lhs.push_back(current.front());
        rhs.push_back(current.back());

        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n - i; j++) {
                current[j] = current[j] * (1 - param) + current[j + 1] * param;
            }
            lhs.push_back(current.front());
            rhs.push_back(current[n - i - 1]);
        }

        std::reverse(rhs.begin(), rhs.end());

        return {lhs, rhs};
    }

    std::vector<double> get_roots(const std::vector<double> &c, double a, double b) {

        bool has_positive = false, has_negative = false;
        for (double v : c) {
            if (v > 0)
                has_positive = true;
            if (v < 0)
                has_negative = true;
        }

        if ((not has_positive) or (not has_negative))
            return {};

        this->degree = c.size() - 1;
        this->coefficients_ = c;
        derivative_coefficients.clear();
        for (int i = 1; i <= degree; i++) {
            derivative_coefficients.push_back(degree * (coefficients_[i] - coefficients_[i - 1]));
        }

        double t0 = newton_method(0.5);

        if (t0 != NAN) {
            if (std::fabs(t0) < 1e-3 or std::fabs(t0 - 1) < 1e-3 or t0 > 1 or t0 < 0)
                t0 = NAN;
        }

        if (not std::isnan(t0)) {
            auto [cl, cr] = subdivide(c, t0);
            std::vector<double> cL, cR;
            std::copy(cl.cbegin(), cl.cend() - 1, std::back_inserter(cL));
            std::copy(cr.cbegin() + 1, cr.cend(), std::back_inserter(cR));

            int n = c.size() - 1;
            for (int i = 0; i < n; i++) {
                cL[i] *= (1.0 * n) / (n - i);
                cR[i] *= (1.0 * n) / (i + 1);
            }

            if (b - a < 1e-3)
                return {b * t0 + a * (1 - t0)};

            auto res1 = get_roots(cL, a, b * t0 + a * (1 - t0));
            auto res2 = get_roots(cR, b * t0 + a * (1 - t0), b);

            std::vector<double> result;

            std::ranges::copy(res1, std::back_inserter(result));
            result.push_back(b * t0 + a * (1 - t0));
            std::ranges::copy(res2, std::back_inserter(result));

            return result;
        } else {
            if (b - a < 1e-3)
                return {};
            t0 = (a + b) / 2;
            auto [cl, cr] = subdivide(c, t0);

            auto res1 = get_roots(cl, a, b * t0 + a * (1 - t0));
            auto res2 = get_roots(cr, b * t0 + a * (1 - t0), b);

            std::vector<double> result;
            std::ranges::copy(res1, std::back_inserter(result));
            std::ranges::copy(res2, std::back_inserter(result));

            return result;
        }
    }

  public:
    explicit BezierRootFinder(std::vector<double> &&coefficients) : coefficients_(coefficients) {
        degree = coefficients.size() - 1;
        for (int i = 1; i < degree; i++) {
            derivative_coefficients.push_back(degree * (coefficients[i] - coefficients[i - 1]));
        }
    }

    std::vector<double> get_roots() {
        return get_roots(coefficients_, 0.0, 1.0);
    }
};

} // namespace GraphicsLab::Geometry