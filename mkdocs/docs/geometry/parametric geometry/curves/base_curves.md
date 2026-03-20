# Base Curves

The abstract parametric curve is defined by the struct `GraphicsLab::Geometry::ParamCurveBase`.

```cpp
template<size_t dim> requires (dim == 2 or dim == 3)
struct ParamCurveBase {
    // definition
};
```

In our system, a parametric curve is defined by a smooth mapping $c(t)$ from $t \in [0, 1]$ to $c(t) \in \mathbb{R}^3$. This mapping is specified by the abstract function:
```cpp
PointType ParamCurveBase::evaluate(double t) const = 0;
```

## Curve Evaluation

The following member functions are used to evaluate the curve, its derivative, and its second derivative:

```cpp
PointType ParamCurveBase::evaluate(double t) const = 0;
PointType ParamCurveBase::derivative(double t) const = 0;
PointType ParamCurveBase::second_derivative(double t) const = 0;
```

## Closed Curve

Curves may be closed ($c(0) = c(1)$), which can be checked by the following interface:

```cpp
bool ParamCurveBase::is_closed(double t) const;
```

If the distance between $c(0)$ and $c(1)$ is smaller than the system tolerance, the curve is reported as `closed`.

## General Projection Algorithm

To project a point to the curve, the following general algorithm is provided via this virtual function:
```cpp
virtual std::pair<PointType, double> ParamCurveBase::projection(PointType test_point, std::optional<double> param_guess) const
```

!!! note "Algorithm Details"

    Denote the test point as $p$.

    Define a function $f(t) = \vert c(t) - p \vert^2$. We compute an extremum of this function via Newton-Raphson, which is a root of $f'(t)$.

    $$
    t_{n + 1} = t_n - \frac{f'(t_n)}{f''(t_n)}
    $$

    However, there may be more than one extremum of $f$. For example, consider the test point $(2, 0)$ and a curve defined by the unit circle, which starts from $(1, 0+)$ and ends at $(1, 0-)$. To address this issue, we separately compute projection candidates in ranges $[0, 0.5]$ and $[0.5, 1]$, then return the better result. For more complex cases, the result depends on the initial guess value.