# Analytic Curves

## Straight Line

Our striaght line is defined in `StraightLine<dim>`. Each instance is specified by two points $p$ and $q$. The parameterization is 
$$
c(t) = tp + (1 - t)q
$$

## Ellipse

Ellipse (2D or 3D) is defined in `Ellipse<dim>`. Each instance has three members:
- Center $o$
- Major axis $a$
- Minor axis $b$

The parameterization is

$$
c(t) = o + a \cos(2\pi t) + b \sin(2 \pi t)
$$

### Projection Formulation

Consider a test point $(x_0, y_0)$ and its distance to $(a \cos \theta, b \sin \theta)$:

$$
d^2 = (a \cos \theta - x_0) ^2 + (b \sin \theta - y_0)^2
$$

Expand

$$
d^2 = f(\theta) = a^2 \cos^2 \theta + b^2 \sin^2 \theta - 2ax_0 \cos \theta - 2b y_0 \sin \theta + C
$$

The derivative

$$
f'(\theta) = 2(b^2 - a^2) \sin \theta \cos \theta - 2by_0 \cos \theta + 2ax_0 \sin \theta
$$

Let $t = \tan \theta / 2$, we have

$$
\sin \theta = \frac{2t}{1 + t^2} \quad \cos \theta = \frac{1 - t^2}{1 + t^2},
$$

substitute to the previous formula, $f'(\theta) = 0$ is equivalent to 

$$
4(b^2 - a^2)t(1 - t^2) - 2by_0(1 - t^4) + 4ax_0t(1 + t^2) = 0
$$

This is a quartic polynomial in $t$.