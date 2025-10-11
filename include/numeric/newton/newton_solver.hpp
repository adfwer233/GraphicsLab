#pragma once
#include <functional>
#include <optional>
#include <stdexcept>

namespace GraphicsLab::Numeric {

struct NewtonSolver {

    using OneVariableFunction = std::function<double(double)>;

    static std::vector<double> find_root(OneVariableFunction f, OneVariableFunction f_prime, double x_min, double x_max,
                                         std::optional<double> x_guess = std::nullopt, double tol = 1e-8,
                                         int max_iter = 100) {
        // Choose initial guess
        double x = x_guess.value_or((x_min + x_max) * 0.5);

        std::vector<double> iterates;
        iterates.push_back(x);

        for (int i = 0; i < max_iter; ++i) {
            double fx = f(x);
            double dfx = f_prime(x);

            if (std::abs(dfx) < 1e-14)
                throw std::runtime_error("Derivative too small — possible flat region.");

            double x_next = x - fx / dfx;

            // Keep the next guess within bounds
            if (x_next < x_min || x_next > x_max)
                x_next = std::clamp(x_next, x_min, x_max);

            iterates.push_back(x_next);

            if (std::abs(x_next - x) < tol || std::abs(fx) < tol)
                break;

            x = x_next;
        }

        return iterates;
    }
};

} // namespace GraphicsLab::Numeric