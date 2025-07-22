#pragma once

#include "geometry/boundary_representation/brep_definition.hpp"
#include "spdlog/spdlog.h"

namespace GraphicsLab::Geometry::BRep {

struct TopologyUtils {

    static Coedge *get_coedge_of_given_face(const Edge *edge, const Face *face) {
        Coedge *start_coedge = edge->coedge();
        Coedge *coedge_iter = start_coedge;

        while (coedge_iter != nullptr) {
            if (coedge_iter->loop()->face() == face) {
                return coedge_iter;
            }
            coedge_iter = coedge_iter->partner();

            if (coedge_iter == start_coedge) {
                break;
            }
        }

        throw cpptrace::logic_error("The edge belongs no coedge referring to the face.");

        return nullptr;
    }

    static std::vector<Shell *> get_all_shells(const Body *body) {
        Shell *shell_start = body->shell();
        Shell *shell_iter = shell_start;
        std::vector<Shell *> shells;

        while (shell_iter != nullptr) {
            shells.push_back(shell_iter);
            shell_iter = shell_iter->next();
        }

        return shells;
    }

    static std::vector<Face *> get_all_faces(const Shell *shell) {
        Face *face_start = shell->face();
        Face *face_iter = face_start;
        std::vector<Face *> faces;

        while (face_iter != nullptr) {
            faces.push_back(face_iter);
            face_iter = face_iter->next();
        }

        return faces;
    }

    static std::vector<Face *> get_all_faces(const Body *body) {
        std::vector<Face *> faces;
        for (auto shell : get_all_shells(body)) {
            std::vector<Face *> shell_faces = get_all_faces(shell);
            std::ranges::copy(shell_faces, std::back_inserter(faces));
        }
        return faces;
    }

    static std::vector<Loop *> get_all_loops(const Face *face) {
        Loop *loop_iter = face->loop();
        std::vector<Loop *> loops;
        while (loop_iter != nullptr) {
            loops.push_back(loop_iter);
            loop_iter = loop_iter->next();
        }
        return loops;
    }

    static std::vector<Edge *> get_all_edges(const Loop *loop) {
        std::vector<Edge *> edges;
        Coedge *coedge_start = loop->coedge();
        Coedge *coedge_iter = coedge_start;

        while (coedge_iter != nullptr) {
            edges.push_back(coedge_iter->edge());

            coedge_iter = coedge_iter->next();
            if (coedge_iter == coedge_start or coedge_iter == nullptr)
                break;
        }

        return edges;
    }

    static std::vector<Edge *> get_all_edges(const Face *face) {
        std::vector<Edge *> edges;
        for (auto loop : get_all_loops(face)) {
            auto loop_edges = get_all_edges(loop);
            std::ranges::copy(loop_edges, std::back_inserter(edges));
        }
        return edges;
    }
};

} // namespace GraphicsLab::Geometry::BRep