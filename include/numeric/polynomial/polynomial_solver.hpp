#pragma once
#include "real_polynomial.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>

#include <numbers>
#include <vector>

namespace GraphicsLab::Numeric {

namespace detail {

inline double cbrt_real(double x) {
    if (x >= 0.0)
        return std::cbrt(x);
    return -std::cbrt(-x);
}

inline bool nearly_zero(double x, double eps = 1e-12) {
    return std::abs(x) < eps;
}

} // namespace detail

struct CubicPolynomialSolver {

    // Solve a3 x^3 + a2 x^2 + a1 x + a0 = 0
    static std::vector<double> solve(const RealPolynomial &p) {
        auto c = p.get_coefficients();
        if (c.size() < 4)
            return {};

        double a0 = c[0];
        double a1 = c[1];
        double a2 = c[2];
        double a3 = c[3];

        // Normalize
        double A = a2 / a3;
        double B = a1 / a3;
        double C = a0 / a3;

        // Depressed cubic: y^3 + py + q = 0
        double p_ = B - A * A / 3.0;
        double q_ = 2.0 * A * A * A / 27.0 - A * B / 3.0 + C;

        double D = (q_ * q_) / 4.0 + (p_ * p_ * p_) / 27.0;

        std::vector<double> roots;

        if (D > 0) {
            double u = detail::cbrt_real(-q_ / 2.0 + std::sqrt(D));
            double v = detail::cbrt_real(-q_ / 2.0 - std::sqrt(D));
            roots.push_back(u + v - A / 3.0);
        } else if (detail::nearly_zero(D)) {
            double u = detail::cbrt_real(-q_ / 2.0);
            roots.push_back(2.0 * u - A / 3.0);
            roots.push_back(-u - A / 3.0);
        } else {
            double r = std::sqrt(-p_ * p_ * p_ / 27.0);
            double phi = std::acos(-q_ / (2.0 * r));
            double t = 2.0 * std::sqrt(-p_ / 3.0);

            for (int k = 0; k < 3; ++k) {
                double y = t * std::cos((phi + 2.0 * std::numbers::pi * k) / 3.0);
                roots.push_back(y - A / 3.0);
            }
        }

        return roots;
    }
};

struct QuarticPolynomialSolver {

    // Solve a4 x^4 + a3 x^3 + a2 x^2 + a1 x + a0 = 0
    static std::vector<double> solve(const RealPolynomial &p) {
        auto c = p.get_coefficients();
        if (c.size() < 5)
            return {};

        double a0 = c[0];
        double a1 = c[1];
        double a2 = c[2];
        double a3 = c[3];
        double a4 = c[4];

        // Normalize
        double A = a3 / a4;
        double B = a2 / a4;
        double C = a1 / a4;
        double D = a0 / a4;

        // Depressed quartic: y^4 + p y^2 + q y + r = 0
        double p_ = B - 3.0 * A * A / 8.0;
        double q_ = A * A * A / 8.0 - A * B / 2.0 + C;
        double r_ = -3.0 * A * A * A * A / 256.0 + A * A * B / 16.0 - A * C / 4.0 + D;

        // Resolvent cubic
        RealPolynomial resolvent({-(q_ * q_) / 8.0, p_ * p_ / 4.0 - r_, p_, 1.0});

        auto z_roots = CubicPolynomialSolver::solve(resolvent);

        double z = z_roots[0];
        std::vector<double> roots;

        auto solve_quadratic = [&](double a, double b, double c) {
            double disc = b * b - 4.0 * a * c;
            if (detail::nearly_zero(disc)) {
                roots.push_back(-b / (2.0 * a));
                return;
            }
            if (disc < 0)
                return;
            double s = std::sqrt(disc);
            roots.push_back((-b + s) / (2.0 * a));
            roots.push_back((-b - s) / (2.0 * a));
        };

        if (detail::nearly_zero(z)) {
            double a = 1;
            double b = p_;
            double cc = r_;

            solve_quadratic(a, b, cc);
            auto tmp_roots = roots;
            roots.clear();
            for (auto r : tmp_roots) {
                if (detail::nearly_zero(r)) {
                    roots.push_back(0);
                } else if (r > 0) {
                    roots.push_back(std::sqrt(r));
                    roots.push_back(-std::sqrt(r));
                }
            }
        } else {
            double u = std::sqrt(2.0 * z);
            if (detail::nearly_zero(u))
                return {};

            double v = q_ / (2 * u);

            solve_quadratic(1.0, u, p_ / 2.0 + z - v);
            solve_quadratic(1.0, -u, p_ / 2.0 + z + v);
        }
        // Back substitution
        for (double &x : roots)
            x -= A / 4.0;

        return roots;
    }
};

struct PolynomialSolver {

    /**
     * @brief find all complex roots of the polynomial
     * @param polynomial
     *
     * @note we find the root with the companion matrix method.
     * @return
     */
    static std::vector<std::complex<double>> find_complex_roots(const RealPolynomial &polynomial) {
        int degree = polynomial.degree();
        std::vector<double> coeff = polynomial.get_coefficients();
        std::ranges::reverse(coeff);
        // Convert to Eigen vector
        Eigen::VectorXd c(degree + 1);
        for (int i = 0; i <= degree; ++i)
            c(i) = coeff[i];

        Eigen::MatrixXd companion = Eigen::MatrixXd::Zero(degree, degree);
        companion.block(1, 0, degree - 1, degree - 1) = Eigen::MatrixXd::Identity(degree - 1, degree - 1);
        companion.row(0) = -c.tail(degree).transpose();

        Eigen::EigenSolver<Eigen::MatrixXd> solver(companion, false);
        Eigen::VectorXcd eigenvalues = solver.eigenvalues();

        std::vector<std::complex<double>> roots(degree);
        for (int i = 0; i < degree; ++i)
            roots[i] = eigenvalues[i];

        return roots;
    }

    /**
     * @param polynomial
     * @param tol tolerance value to determine if a complex root is a real root.
     * @return real roots
     */
    static std::vector<double> find_real_roots(const RealPolynomial &polynomial, double tol = 1e-9) {
        std::vector<std::complex<double>> rootsC = find_complex_roots(polynomial);
        std::vector<double> roots;
        roots.reserve(rootsC.size());

        for (const auto &r : rootsC) {
            if (std::abs(r.imag()) < tol)
                roots.push_back(r.real());
        }
        return roots;
    }

    static std::vector<std::pair<double, int>> find_real_roots_with_multiplicity(const RealPolynomial &polynomial,
                                                                                 double real_tol = 1e-9,
                                                                                 double equal_tol = 1e-8) {
        std::vector<std::complex<double>> rootsC = find_complex_roots(polynomial);
        std::vector<double> realRoots;

        // Filter only real roots
        for (const auto &r : rootsC) {
            if (std::abs(r.imag()) < real_tol)
                realRoots.push_back(r.real());
        }

        // Sort roots to group duplicates
        std::sort(realRoots.begin(), realRoots.end());

        // Count multiplicities
        std::vector<std::pair<double, int>> uniqueRoots;
        for (size_t i = 0; i < realRoots.size();) {
            double val = realRoots[i];
            int count = 1;
            size_t j = i + 1;
            while (j < realRoots.size() && std::abs(realRoots[j] - val) < equal_tol) {
                ++count;
                ++j;
            }
            uniqueRoots.emplace_back(val, count);
            i = j;
        }

        return uniqueRoots;
    }
};

} // namespace GraphicsLab::Numeric