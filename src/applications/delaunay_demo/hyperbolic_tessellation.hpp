#pragma once

#include "hyperbolic_geometry.hpp"

#include <cmath>
#include <complex>
#include <numbers>
#include <set>
#include <vector>

struct Mobius {
    using PointType = std::complex<double>;
    std::complex<double> a, b, c, d;

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
    Mobius::PointType m = (z1 + z2) / (1.0 + std::conj(z1) * z2);

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

struct HyperbolicPolygon {
    using PointType = std::complex<double>;
    using hash_type = std::pair<int, int>;

    PointType center;
    std::vector<PointType> vertices;

    auto hash() const -> hash_type {
        return {static_cast<int>(std::round(center.real() * 10000)),
                static_cast<int>(std::round(center.imag() * 10000))};
    }
};

struct HyperbolicTessellation {
    using PointType = std::complex<double>;

    int p, q;
    std::vector<HyperbolicPolygon> polygons;
    std::set<HyperbolicPolygon::hash_type> polygon_hash_set;

    explicit HyperbolicTessellation(int p, int q) : p(p), q(q) {
    }

    void create_initial_polygon() {
        double d = std::acosh((std::cos(std::numbers::pi / q)) / (std::sin(std::numbers::pi / p)));
        double r = std::tanh(d / 2.0);

        double pi = std::numbers::pi;
        r = std::sqrt((std::tan(pi / 2 - pi / q) - std::tan(pi / p)) / (std::tan(pi / 2 - pi / q) + std::tan(pi / p)));

        spdlog::critical("radius r {}", r);

        HyperbolicPolygon initial_polygon;
        initial_polygon.center = {0.0, 0.0};

        for (int i = 0; i < p; ++i) {
            double theta = 2 * std::numbers::pi * i / p;
            initial_polygon.vertices.emplace_back(r * std::polar(1.0, theta));
        }

        polygons.emplace_back(initial_polygon);
        polygon_hash_set.insert(initial_polygon.hash());
    }

    void create_polygon_tessellation(int depth) {
        create_initial_polygon();
        recurse(polygons.front(), depth);
        spdlog::info("polygon count {}", polygons.size());
    }

  private:
    void recurse(const HyperbolicPolygon polygon, int depth) {
        if (depth == 0)
            return;

        auto reflection_mobius = [&](PointType a, PointType b) {
            // translate a to the origin
            auto m1 = MobiusConstructor::move_to_origin(a);

            auto b1 = m1(b);
            auto m2 = MobiusConstructor::rotate(-std::arg(b1));
            return m2.compose(m1);
        };

        auto reflection_trans = [&](const PointType &z, Mobius &m) -> PointType {
            return m.inverse()(std::conj(m(z)));
        };

        for (int i = 0; i < polygon.vertices.size(); ++i) {
            const auto &a = polygon.vertices[i];
            const auto &b = polygon.vertices[(i + 1) % polygon.vertices.size()];

            auto mob = reflection_mobius(a, b);
            PointType new_center = reflection_trans(polygon.center, mob);

            // auto mid = (a + b) / 2.0;
            // spdlog::info("{} {} {} {} {} {}", polygon.center.real(), polygon.center.imag(), mid.real(), mid.imag(),
            // new_center.real(), new_center.imag()); spdlog::info("{} {}", new_center.real() / mid.real(),
            // new_center.imag() / mid.imag());
            HyperbolicPolygon new_polygon;
            new_polygon.center = new_center;

            for (const auto &v : polygon.vertices) {
                PointType reflected = reflection_trans(v, mob);
                new_polygon.vertices.emplace_back(reflected);
            }

            auto hash = new_polygon.hash();
            if (polygon_hash_set.contains(hash)) {
                spdlog::info("hash {} {}", hash.first, hash.second);
            }

            polygons.emplace_back(new_polygon);
            polygon_hash_set.insert(hash);
            recurse(new_polygon, depth - 1);
        }
    }
};
