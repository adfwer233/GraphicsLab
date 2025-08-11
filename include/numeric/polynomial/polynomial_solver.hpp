#pragma once
#include "real_polynomial.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>

#include <vector>

namespace GraphicsLab::Numeric {

struct PolynomialSolver {

    /**
     * @brief find all complex roots of the polynomial
     * @param polynomial
     *
     * @note we find the root with the companion matrix method.
     * @return
     */
    static std::vector<std::complex<double>> find_complex_roots(const RealPolynomial& polynomial) {
        int degree = polynomial.degree();
        std::vector<double> coeff = polynomial.get_coefficients();
        std::ranges::reverse(coeff);
        // Convert to Eigen vector
        Eigen::VectorXd c(degree + 1);
        for (int i = 0; i <= degree; ++i) c(i) = coeff[i];

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
    static std::vector<double> find_real_roots(const RealPolynomial& polynomial, double tol = 1e-9) {
        std::vector<std::complex<double>> rootsC = find_complex_roots(polynomial);
        std::vector<double> roots;
        roots.reserve(rootsC.size());

        for (const auto &r : rootsC) {
            if (std::abs(r.imag()) < tol)
                roots.push_back(r.real());
        }
        return roots;
    }

    static std::vector<std::pair<double, int>> find_real_roots_with_multiplicity(const RealPolynomial& polynomial, double real_tol = 1e-9, double equal_tol = 1e-8) {
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

}