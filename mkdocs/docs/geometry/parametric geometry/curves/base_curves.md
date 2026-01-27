# Base Curves

The abstract parametric curces is defined by the struct `GraphicsLab::Geometry::ParamCurveBase`.

```cpp
template<size_t dim> requires (dim == 2 or dim == 3)
struct ParamCurveBase {
    // definition
};
```

In our system, a parametric curve is defined by a smooth mapping $c(t)$ from $t \in [0, 1]$ to $c(t) \in \mathbb{R}^3$, this mapping is specified by the abstract function
```cpp
PointType ParamCurveBase::evaluate(double t) const = 0;
```

## Curve Evaluation

Following member functions are used to evaluate the curve itself, the derivative, and the second derivative:

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

If the distance between $c(0)$ and $c(1)$ is smaller than the system tolerance, we report the curve is `closed`.

## General Projection Algorithm

To project one point to the curve, we implemented a general algorithm, which is provided in the following virtual function:
```cpp
virtual std::pair<PointType, double> ParamCurveBase::projection(PointType test_point, std::optional<double> param_guess) const
```

!!! note "Algorithm Details"

    Denote the test point as $p$.

    Define a function $f(t) = \vert c(t) - p \vert^2$. We compute the extreme point of this function via Newton-Raphson, which is the zero point of derivative $f'(t)$.

    $$
    t_{n + 1} = t_n - \frac{f'(t_n)}{f''(t_n)}
    $$

    However, there may be more than one extreme points of $f$. For example, consider the test point $(2, 0)$ and curve defined by the unit circle, which start from $(1, 0+)$ and end at $(1, 0-)$. To alley this issue, we seperately compute the projection point in range $[0, 0.5]$ and $[0.5, 1]$, the reture the best result. For more complex cases, the result is determinated by the initial guess value.