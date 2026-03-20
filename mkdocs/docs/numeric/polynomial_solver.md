# Polynomial Solver

The numeric module includes polynomial representations and root-solving utilities used by geometry algorithms.

## Prerequisites

Before this page:

- Review [Geometry](../geometry/introduction.md) to understand where root solving is used
- Complete [Get Started](../graphicslab/get_started.md) so you can build and run tests

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

## Expected Output

After this page, you should be able to:

- Select the appropriate polynomial representation for a geometry task
- Identify where solver APIs live and where they are tested
- Trace how numeric solving supports projection/intersection workflows

## Verify

- You can locate `polynomial_solver.hpp` and the related polynomial types
- You can locate numeric tests under `test/numeric/`
- You can describe one geometry scenario that maps to a root-finding step
