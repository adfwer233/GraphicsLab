#pragma once
#include "geometry/mesh/mesh.hpp"
#include "igl/edges.h"
#include "spdlog/spdlog.h"

#include <unordered_set>

namespace GraphicsLab::Geometry {

struct ConvexHull3D {
    using PointType = glm::vec3;

    /**
     * DCEL data structure
     */

    struct HalfEdge;
    struct Face;

    struct Vertex {
        int id;
        PointType position;
        HalfEdge *edge = nullptr;
    };

    struct HalfEdge {
        Vertex *origin = nullptr;
        HalfEdge *twin = nullptr;
        HalfEdge *next = nullptr;
        Face *face = nullptr;
    };

    struct Face {
        HalfEdge *edge = nullptr;
        PointType normal;

        void compute_normal() {
            auto a = edge->origin->position;
            auto b = edge->next->origin->position;
            auto c = edge->next->next->origin->position;
            normal = glm::cross(b - a, c - a);
            normal = glm::normalize(normal);
        }

        bool is_visible(const PointType &p) {
            PointType a = edge->origin->position;
            return glm::dot(p - a, normal) > 1e-6;
        }
    };

    /**
     * Incremental 3D Convex hull
     */

    std::vector<std::unique_ptr<Vertex>> vertices;
    std::vector<std::unique_ptr<HalfEdge>> halfedges;
    std::vector<std::unique_ptr<Face>> faces;

    std::vector<bool> used;

    std::vector<TriangleIndex> make_indices() const {
        std::vector<TriangleIndex> indices;
        for (auto &f : faces) {
            int i = f.get()->edge->origin->id;
            int j = f.get()->edge->next->origin->id;
            int k = f.get()->edge->next->next->origin->id;
            indices.emplace_back(i, j, k);

            if (i == j or j == k or k == i) {
                int x = 0;
            }
        }
        return indices;
    }

    void build(const std::vector<PointType> &points) {
        if (points.size() < 4)
            return;

        used.resize(points.size(), false);
        create_initial_tet(points);

        for (size_t i = 0; i < points.size(); i++) {
            if (not used[i]) {
                auto v = std::make_unique<Vertex>();
                v->position = points[i];
                v->id = i;
                vertices.push_back(std::move(v));
                add_point(vertices.back().get());
            }
        }
    }

  private:
    struct pair_hash {
        template <class T1, class T2> std::size_t operator()(const std::pair<T1, T2> &p) const {
            auto h1 = std::hash<T1>()(p.first);
            auto h2 = std::hash<T2>()(p.second);
            return h1 ^ (h2 << 1);
        }
    };

    void make_face(int a, int b, int c) {
        Vertex *va = vertices[a].get(), *vb = vertices[b].get(), *vc = vertices[c].get();

        auto e1 = std::make_unique<HalfEdge>();
        auto e2 = std::make_unique<HalfEdge>();
        auto e3 = std::make_unique<HalfEdge>();

        e1->origin = va;
        e2->origin = vb;
        e3->origin = vc;

        e1->next = e2.get();
        e2->next = e3.get();
        e3->next = e1.get();

        auto f = std::make_unique<Face>();
        f->edge = e1.get();
        e1->face = f.get();
        e2->face = f.get();
        e3->face = f.get();

        f->compute_normal();

        halfedges.push_back(std::move(e1));
        halfedges.push_back(std::move(e2));
        halfedges.push_back(std::move(e3));

        faces.push_back(std::move(f));
    }

    void create_initial_tet(const std::vector<PointType> &points) {
        int i = 0, j = 1, k = 2, l = 3;

        for (int u = 0; u < 4; u++) {
            used[u] = true;
            auto v = std::make_unique<Vertex>();
            v->position = points[u];
            v->id = u;
            vertices.push_back(std::move(v));
        }

        Vertex *v0 = vertices[0].get();
        Vertex *v1 = vertices[1].get();
        Vertex *v2 = vertices[2].get();
        Vertex *v3 = vertices[3].get();

        // Ensure positive orientation
        if (glm::dot(glm::cross(v1->position - v0->position, v2->position - v0->position),
                     v3->position - v0->position) > 0) {
            std::swap(v1->position, v2->position);
            std::swap(v1->id, v2->id);
        }

        // Create 4 faces: (v0,v1,v2), (v0,v3,v1), (v0,v2,v3), (v1,v3,v2)
        struct FaceInfo {
            Vertex *a;
            Vertex *b;
            Vertex *c;
        };

        std::vector<FaceInfo> faceInfos = {{v0, v1, v2}, {v0, v3, v1}, {v0, v2, v3}, {v1, v3, v2}};

        // Each face has 3 half-edges, stored in a flat list
        std::vector<HalfEdge *> edgeTable;
        edgeTable.reserve(12); // 4 faces × 3 edges

        std::vector<HalfEdge *> localEdges;
        for (const auto &info : faceInfos) {
            auto e1 = std::make_unique<HalfEdge>();
            auto e2 = std::make_unique<HalfEdge>();
            auto e3 = std::make_unique<HalfEdge>();

            e1->origin = info.a;
            e2->origin = info.b;
            e3->origin = info.c;

            e1->next = e2.get();
            e2->next = e3.get();
            e3->next = e1.get();

            auto f = std::make_unique<Face>();
            f->edge = e1.get();
            e1->face = e2->face = e3->face = f.get();
            f->compute_normal();

            edgeTable.push_back(e1.get());
            edgeTable.push_back(e2.get());
            edgeTable.push_back(e3.get());

            halfedges.push_back(std::move(e1));
            halfedges.push_back(std::move(e2));
            halfedges.push_back(std::move(e3));
            faces.push_back(std::move(f));
        }

        spdlog::info("dir {}", faces[0]->is_visible(v3->position));
        spdlog::info("dir {}", faces[1]->is_visible(v2->position));
        spdlog::info("dir {}", faces[2]->is_visible(v1->position));
        spdlog::info("dir {}", faces[3]->is_visible(v0->position));

        // Create a map from (origin, dest) to edge for twin linking
        std::unordered_map<std::pair<Vertex *, Vertex *>, HalfEdge *, pair_hash> edgeMap;
        for (HalfEdge *e : edgeTable) {
            Vertex *from = e->origin;
            Vertex *to = e->next->origin;
            edgeMap[{from, to}] = e;
        }

        // Assign twins
        for (auto &[key, e] : edgeMap) {
            Vertex *from = key.first;
            Vertex *to = key.second;
            auto it = edgeMap.find({to, from});
            if (it != edgeMap.end()) {
                e->twin = it->second;
            }
        }
    }

    void fix_twin() {
        std::unordered_map<std::pair<Vertex *, Vertex *>, HalfEdge *, pair_hash> edgeMap;
        for (auto &e : halfedges) {
            Vertex *from = e->origin;
            Vertex *to = e->next->origin;
            edgeMap[{from, to}] = e.get();
        }

        for (auto &[key, e] : edgeMap) {
            Vertex *from = key.first;
            Vertex *to = key.second;

            if (auto it = edgeMap.find({to, from}); it != edgeMap.end()) {
                e->twin = it->second;
            }
        }
    }

    void add_point(Vertex *v) {
        std::unordered_set<Face *> visible_faces;

        for (auto &f : faces) {
            if (f->is_visible(v->position)) {
                visible_faces.insert(f.get());
            }
        }

        if (visible_faces.empty())
            return;

        std::unordered_set<HalfEdge *> silhouette_edge;

        for (Face *f : visible_faces) {
            HalfEdge *e = f->edge;
            do {
                HalfEdge *twin = e->twin;
                if (twin and not visible_faces.contains(twin->face)) {
                    silhouette_edge.insert(twin);
                }
                e = e->next;
            } while (e != f->edge);
        }

        auto isVisible = [&](const std::unique_ptr<Face> &f) { return visible_faces.contains(f.get()); };

        spdlog::info("is visible faces {}", visible_faces.size());
        std::erase_if(faces, isVisible);

        // Stitch new faces
        for (HalfEdge *boundary : silhouette_edge) {
            Vertex *a = boundary->twin->origin;
            Vertex *b = boundary->origin;

            auto he1 = std::make_unique<HalfEdge>();
            auto he2 = std::make_unique<HalfEdge>();
            auto he3 = std::make_unique<HalfEdge>();

            if (v == a or v == b or a == b) {
                int x = 0;
            }

            he1->origin = a;
            he2->origin = b;
            he3->origin = v;

            he1->next = he2.get();
            he2->next = he3.get();
            he3->next = he1.get();

            auto f = std::make_unique<Face>();
            f->edge = he1.get();

            he1->face = he2->face = he3->face = f.get();
            f->compute_normal();

            // Connect twin
            he2->twin = boundary;
            boundary->twin = he2.get();

            // Store
            halfedges.push_back(std::move(he1));
            halfedges.push_back(std::move(he2));
            halfedges.push_back(std::move(he3));
            faces.push_back(std::move(f));
        }

        fix_twin();
    }
};

} // namespace GraphicsLab::Geometry