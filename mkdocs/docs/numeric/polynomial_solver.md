# Polynomial Solver

The numeric module includes polynomial representations and root-solving utilities used by geometry algorithms.

## Main Headers

- `include/numeric/polynomial/polynomial_solver.hpp`
- `include/numeric/polynomial/real_polynomial.hpp`
- `include/numeric/polynomial/bernstein_polynomial.hpp`
- `include/numeric/polynomial/bezier_polynomial.hpp`

## Typical Use Cases

- Solving projection and intersection equations
- Converting geometric constraints into root-finding problems
- Supporting Bezier and implicit equation workflows

## Tests

- `test/numeric/polynomial_solver_test.cpp`
- `test/numeric/polynomial_test.cpp`

## Notes

Choose the polynomial basis that matches your numerical step:

- Bernstein basis for Bezier-domain operations
- Standard real polynomial form for generic root solvers
