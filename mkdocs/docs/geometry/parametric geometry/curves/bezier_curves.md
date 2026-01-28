# Bezier Curves

Our bezier curve is defined by the class `BezierCurveBase<dim>`, and there are also some alias

```cpp
using BezierCurve2D = BezierCurveBase<2>;
using BezierCurve3D = BezierCurveBase<3>;
```

Rational Bezier curve is defined by the control points $P_k$ and weights $w_k$.

$$
c(t) = \frac{\sum_{k = 0}^n w_k P_k B_n^k(t)}{\sum_{k = 0}^n w_k B_n^k(t)},
$$
where $B_{n}^k(t) = \binom{n}{k} t^k(1 - t)^{n - k}$ is the Bernstein polynomial.

## Evaluation 


## Subdivision


## The First Derivative

The derivative of Bezier curve is still a Bezier curve. Firstly, we can compute the derivative of Bernstein polynomials

$$
\begin{aligned}
\frac{\mathrm{d}}{\mathrm{d} t} B_{n}^k(t) &= \binom{n}{k} kt^{k - 1}(1 - t)^{n - k} - \binom{n}{k} (n - k)t^k(1- t) ^{n - 1 - k} \\
&= n \binom{n - 1}{k - 1} t^{k - 1}(1 - t)^{n - k} - n \binom{n - 1}{k} t^k(1 - t)^{n - 1 - k}\\
&= n B_{n - 1}^{k - 1} - n B_{n - 1}^k
\end{aligned}    
$$

For the corner cases $k = 0$ and $k = n$, 

$$
\frac{\mathrm{d}}{\mathrm{d} t} B_{n}^0(t) = \frac{\mathrm{d}}{\mathrm{d} t} (1 - t)^n = - nB_{n - 1}^0 (t)
$$

$$
\frac{\mathrm{d}}{\mathrm{d} t} B_{n}^n(t) = \frac{\mathrm{d}}{\mathrm{d} t} t^n = n B_{n - 1}^n (t)
$$

A simple case: $w_k = 1$ for all $k$

$$
\begin{aligned}
    c'(t) &= n \sum_{k = 0}^n P_k (B_{n - 1}^{k - 1}(t) - B_{n - 1}^k(t)) \\
    &= n \sum_{k = 1}^n P_k B_{n - 1}^{k - 1}(t) - n \sum_{k = 0}^{n - 1}P_k B_{n - 1}^{k}(t) \\
    &= n \sum_{k = 0}^{n - 1} P_{k + 1} B_{n - 1}^{k}(t) - n \sum_{k = 0}^{n - 1} P_k B_{n - 1}^{k}(t) \\
    &= n \sum_{k = 0}^{n - 1} (P_{k + 1} - P_k) B_{n - 1}^k (t)
\end{aligned}
$$

Now we compute the general case, let $c(t) = \frac{f(t)}{g(t)}$, then the derivative is 

$$
c^\prime(t) = \frac{f'(t)g(t) - f(t)g'(t)}{g(t)^2} = \frac{f'(t) - g'(t) c(t)}{g(t)}
$$

$$
g'(t) = \frac{\mathrm{d}}{\mathrm{d} t} \sum_{k = 0}^n w_k B_n^k(t) = n \sum_{k = 0}^{n - 1} (w_{k + 1} - w_k) B_{n - 1}^k(t)
$$

$$
f'(t) = \frac{\mathrm{d}}{\mathrm{d} t} \sum_{k = 0}^n w_k P_k B_n^k(t) = n \sum_{k = 0}^{n - 1} (w_{k + 1} P_{k + 1} - w_k P_k) B_{n - 1}^k(t)
$$

## The Second Derivative

Now we derivative the formulation of the second derivative.

$$
\begin{aligned}
    c''(t) &= \frac{(f'' - g''c - g' c')g - g'(f' - g' c)}{g^2}  \\
    &= \frac{(f'' - g''c - g' c')g - g'c'g}{g^2}\\
    &= \frac{f'' - g''c - 2g' c'}{g}
\end{aligned}
$$

## Fitting


## Degree elevation


## Winding Number