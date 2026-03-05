# Plane — Torus Intersection

This note derives the analytic condition used by the `PlaneTorusIntersection` intersector and describes the practical algorithm implemented in the codebase.

## Torus parameterization

We use the (major, minor) angle parameterization
$$
S(u,v)=C+(R+r\cos v)\big(\cos u\,d_1+\sin u\,d_2\big)+r\sin v\,n,
$$
where

- $C$ is the torus center,
- $d_1,d_2$ are orthonormal directions spanning the major-circle plane,
- $n$ is the torus base normal (orthonormal to $d_1,d_2$),
- $R$ is the major radius and $r$ the minor radius,
- $u\in[0,2\pi)$ is the major-angle and $v\in[0,2\pi)$ is the minor-angle.

## Plane equation

A plane is given by a point $P_0$ and normal $n_p$; the plane constraint is
$$
n_p\cdot(x-P_0)=0 \quad\Longleftrightarrow\quad n_p\cdot x = n_p\cdot P_0.
$$

Substitute $x=S(u,v)$ and bring the constant terms to a single scalar $d' = n_p\cdot(C-P_0)$. Introduce
$$
a = n_p\cdot d_1,
\quad b = n_p\cdot d_2,
\quad c = n_p\cdot n.
$$
The plane constraint becomes
$$
(R + r\cos v)\,(a\cos u + b\sin u) + r\,c\sin v + d' = 0.
$$

## Reduce the $u$ dependence

Let $\alpha = \sqrt{a^2+b^2}$ and $\phi = \operatorname{atan2}(b,a)$. Then
$$
a\cos u + b\sin u = \alpha\cos(u-\phi).
$$
Therefore the intersection condition is
$$
\alpha(R + r\cos v)\cos(u-\phi) + r\,c\sin v + d' = 0.
$$
Solve for $\cos(u-\phi)$:
$$
\cos(u-\phi) = -\dfrac{r\,c\sin v + d'}{\alpha(R + r\cos v)} =: RHS(v).
$$

For a fixed $v$ the equation has solutions in $u$ iff $|RHS(v)|\le 1$. When valid,
$$
u = \phi \pm \arccos\big(RHS(v)\big).
$$

Thus intersection curves can be obtained by sweeping $v$ and solving for $u$ where the RHS is in $[-1,1]$.

## Degenerate / special cases

- If $\alpha\approx 0$ (i.e. $a\approx b\approx 0$), then the plane normal is parallel to the torus axis $n$. The equation reduces to
  $$r\,c\sin v + d' = 0,$$
   which is independent of $u$: each admissible $v$ yields an entire loop $u\in[0,2\pi)$ (a circle on the torus). Solve for $\sin v = -d'/(r c)$ when $c\neq 0$. This branch should be handled once (outside the $v$ sampling loop), otherwise the same circles are emitted repeatedly.
   In the current implementation, if $c\approx 0$ (or $r\approx 0$), the solver returns no curve for this degenerate configuration.

- If $RHS(v)$ is numerically slightly outside $[-1,1]$ due to rounding, clamp and treat as tangency (double root) when within tolerance.

- If $\alpha(R+r\cos v)$ is very small, skip that sample and split the traced branch to avoid unstable jumps.

## Algorithm implemented in PlaneTorusIntersection

1. Compute coefficients $a,b,c,d'$ and $\alpha=\sqrt{a^2+b^2}$.
2. If $\alpha\approx 0$, solve $\sin v = -d'/(rc)$ and emit one or two full-$u$ loops directly.
3. Otherwise sample the minor-angle $v$ densely (uniform grid). For each sample compute
   $$RHS(v) = -\dfrac{r\,c\sin v + d'}{\alpha(R + r\cos v)}.$$ 
4. If $|RHS(v)|\le 1$ compute two candidate major-angles
   $$u_{\pm}(v)=\phi\pm\arccos(RHS(v)).$$
   Record $(u_{\pm}(v),v)$ pairs as points on the intersection trace. If the two roots collapse (tangency), keep a single branch.
5. Split branches when validity is lost (no real root), when the denominator is near zero, or when a large $u$ jump indicates discontinuity.
6. For each branch, fit smooth BSpline curves to 3D points and to the corresponding pcurves (torus $(u,v)$ and plane $(s,t)$ parameters).

The sampling-and-fit approach recovers continuous intersection traces robustly for typical geometries. The analytic reduction above gives exact existence criteria per $v$ and exposes the tangency / degeneracy cases.

## Numerical considerations

- Use sufficient $v$ sampling resolution for thin tori or nearly-grazing planes.
- Guard against small $\alpha(R+r\cos v)$ before forming $RHS(v)$.
- For production robustness consider: adaptive sampling, marching along a validated trace instead of pure grid sampling, or root-finding in $v$ to locate segment endpoints precisely.

## Mapping to code

- See `include/geometry/boundary_representation/intersector/surface_surface_intersection/plane_torus_intersection.hpp` for the sampling and fitting implementation.
