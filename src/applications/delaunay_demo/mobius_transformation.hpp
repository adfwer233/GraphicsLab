#pragma once

#include <complex>

struct Mobius {
    using PointType = std::complex<float>;
    PointType a, b, c, d;

    PointType operator()(PointType z) const {
        return (a * z + b) / (c * z + d);
    }

    Mobius inverse() const {
        // Inverse of a Möbius transformation
        return {d, -b, -c, a};
    }

    // Compose with another: this ∘ other
    Mobius compose(const Mobius &other) const {
        return {a * other.a + b * other.c, a * other.b + b * other.d, c * other.a + d * other.c,
                c * other.b + d * other.d};
    }
};

// Construct Möbius transform that maps (z1, z2) → (-1, 1)
inline Mobius mobius_map_to_real_axis(Mobius::PointType z1, Mobius::PointType z2) {
    Mobius::PointType m = (z1 + z2) / (1.0f + std::conj(z1) * z2);

    Mobius M;
    M.a = 1.0;
    M.b = -m;
    M.c = std::conj(m);
    M.d = 1.0 - std::norm(m);

    return M;
}

struct MobiusConstructor {
    using PointType = std::complex<double>;
    static Mobius move_to_origin(PointType z) {
        Mobius result;
        result.a = 1.0;
        result.b = -z;
        result.c = -std::conj(z);
        result.d = 1.0;
        return result;
    }

    static Mobius rotate(double angle) {
        PointType r = std::polar(1.0, angle); // e^(i*angle)
        Mobius result;
        result.a = r;
        result.b = 0.0;
        result.c = 0.0;
        result.d = 1.0;
        return result;
    }
};

