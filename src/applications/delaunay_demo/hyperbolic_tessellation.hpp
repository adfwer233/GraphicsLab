#pragma once

#include "hyperbolic_geometry.hpp"
#include "mobius_transformation.hpp"
#include "utils/sampler.hpp"

#include <cmath>
#include <complex>
#include <numbers>
#include <queue>
#include <set>
#include <vector>

struct HyperbolicPolygon {
    using PointType = std::complex<float>;
    using hash_type = std::size_t;

    PointType center;
    std::vector<PointType> vertices;

    auto hash() const -> hash_type {
        auto x = static_cast<int>(std::round(center.real() * 10000));
        auto y = static_cast<int>(std::round(center.imag() * 10000));
        std::size_t h1 = std::hash<int>{}(x);
        std::size_t h2 = std::hash<int>{}(y);
        return h1 ^ (h2 << 1);
    }
};

struct HyperbolicTessellation {
    using PointType = std::complex<float>;

    int p, q;
    std::vector<HyperbolicPolygon> polygons;
    std::unordered_set<HyperbolicPolygon::hash_type> polygon_hash_set;

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

    CurveMesh2D create_curve_mesh_2d() const {
        CurveMesh2D curve_mesh;
        for (int id = 0; auto &poly : polygons) {
            glm::vec3 c = {GraphicsLab::Sampler::sampleUniform(), GraphicsLab::Sampler::sampleUniform(), GraphicsLab::Sampler::sampleUniform()};
            spdlog::critical("curve mesh {}", poly.vertices.size());
            for (int i = 0; i < poly.vertices.size(); ++i) {
                int j = (i + 1) % poly.vertices.size();
                auto start = poly.vertices[i];
                auto end = poly.vertices[j];

                GraphicsLab::Geometry::HyperbolicLineSegment edge(start, end);

                int n = 3;

                for (int k = 0; k <= n; k++) {
                    float param = 1.0f * k / n;
                    auto complex_pos = edge.evaluate(param);
                    curve_mesh.vertices.push_back({{complex_pos.real(), complex_pos.imag()}, c});

                    if (k > 0) {
                        curve_mesh.indices.emplace_back(curve_mesh.vertices.size() - 2, curve_mesh.vertices.size() - 1);
                    }
                }
            }

            spdlog::critical("curve mesh vertice {}, indices {}", curve_mesh.vertices.size(), curve_mesh.indices.size());

            id ++;
        }
        return curve_mesh;
    }
  private:
    void recurse(const HyperbolicPolygon start_polygon, int depth) {
        std::queue<std::pair<HyperbolicPolygon, int>> que;

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

        que.push({start_polygon, 0});

        while (not que.empty()) {
            auto [polygon, d] = que.front();
            que.pop();
            if (d >= depth) continue;
            for (int i = 0; i < polygon.vertices.size(); ++i) {
                const auto &a = polygon.vertices[i];
                const auto &b = polygon.vertices[(i + 1) % polygon.vertices.size()];

                auto mob = reflection_mobius(a, b);
                PointType new_center = reflection_trans(polygon.center, mob);

                HyperbolicPolygon new_polygon;
                new_polygon.center = new_center;

                for (const auto &v : polygon.vertices) {
                    PointType reflected = reflection_trans(v, mob);
                    new_polygon.vertices.emplace_back(reflected);
                }

                auto hash = new_polygon.hash();
                if (polygon_hash_set.contains(hash)) {
                    continue;
                }

                polygons.emplace_back(new_polygon);
                polygon_hash_set.insert(hash);

                que.push({new_polygon, d + 1});
            }
        }
    }
};
