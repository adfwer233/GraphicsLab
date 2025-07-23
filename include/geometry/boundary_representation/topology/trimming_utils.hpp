#pragma once

#include "cpptrace/exceptions.hpp"
#include "geometry/boundary_representation/allocator/brep_allocator.hpp"
#include "geometry/boundary_representation/brep_definition.hpp"
#include "topology_utils.hpp"

namespace GraphicsLab::Geometry::BRep {

/**
 * Given a trimmed surface, and a point on the underlying geometry surface,
 * determine if the point is contained in the trimmed surface.
 */
struct ContainmentQuery {

    enum class ContainmentResult {
        Inside,
        Outside,
        Boundary
    };

    /**
     * @brief containment query
     * @param face
     * @param test_point
     * @return containment query result.
     */
    static ContainmentResult contained(Face* face, BRepPoint2 test_point) {
        double wn = 0;

        for (Loop* loop: TopologyUtils::get_all_loops(face)) {
            wn += winding_number_loop(loop, test_point);
        }

        if (wn > std::numbers::pi) {
            return ContainmentResult::Inside;
        } else {
            return ContainmentResult::Outside;
        }
    }

    static bool is_hole(Loop* loop) {
        double wn = winding_number_loop(loop, std::nullopt);
        return wn < std::numbers::pi;
    }

    static double winding_number_loop(Loop* loop, std::optional<BRepPoint2> test_point_given) {
        std::vector<BRepPoint2> samples;

        if (loop->coedge() == nullptr) {
            throw cpptrace::logic_error("Loop has no coedge");
        }

        Coedge* coedge_start = loop->coedge();
        Coedge* coedge_iter = loop->coedge();

        constexpr int sample_per_pcurve = 5;

        while (true) {
            if (coedge_iter->geometry() == nullptr) {
                throw cpptrace::logic_error("coedge has no pcurve");
            }

            auto pcurve = coedge_iter->geometry();
            auto pcurve_param_start = pcurve->param_range().start();
            auto pcurve_param_end = pcurve->param_range().end();
            auto delta = (pcurve_param_end - pcurve_param_start) / sample_per_pcurve;
            std::vector<BRepPoint2> pcurve_samples;
            for (int i = 0; i <= sample_per_pcurve; i++) {
                double param = pcurve_param_start + i * delta;
                pcurve_samples.emplace_back(pcurve->param_geometry()->evaluate(param));
            }

            if (not coedge_iter->is_forward()) {
                std::ranges::reverse(pcurve_samples);
            }
            std::ranges::copy(pcurve_samples, std::back_inserter(samples));
            coedge_iter = coedge_iter->next();
            if (coedge_iter == coedge_start) break;
        }

        BRepPoint2 dir = samples[1] - samples[0];
        BRepPoint2 in_dir{-dir.y, dir.x};
        BRepPoint2 test_point = glm::mix(samples[0], samples[1], 0.5) + in_dir * 1e-3;

        if (test_point_given.has_value()) {
            test_point = test_point_given.value();
        }

        double wn = 0;
        for (int i = 1; i < samples.size(); i++) {
            wn += winding_number_line_segment(test_point, samples[i - 1], samples[i]);
        }

        return wn;
    }

    static double winding_number_line_segment(BRepPoint2 test_point, BRepPoint2 start_pos, BRepPoint2 end_pos) {
        auto d1 = glm::length(start_pos - test_point);
        auto d2 = glm::length(end_pos - test_point);

        auto v1 = glm::normalize(start_pos - test_point);
        auto v2 = glm::normalize(end_pos - test_point);
        auto outer = v1.x * v2.y - v1.y * v2.x;
        auto inner = glm::dot(v1, v2);

        auto acos_value = std::clamp(std::acos(inner), -1.0, 1.0);

        if (std::isnan(acos_value)) {
            // spdlog::info("NAN CASE");
            return 0;
        }

        return outer > 0 ? acos_value : -acos_value;
    }
};

struct TrimmingUtils {

    struct TrimmingLoop {
        std::vector<PCurve*> pcurves;
    };

    /**
     * @brief Trimming a surface with a set of loops
     * @param face
     * @param curves
     * @return A set of faces, split by given loops.
     */
    static std::vector<Face *> add_trimming_curve(Face *face, std::vector<TrimmingLoop> curves) {
        std::vector<Face*> faces;
        faces.push_back(face);

        for (int i = 0; i < curves.size(); i++) {
            PCurve* first_pcurve = curves[i].pcurves.front();
            BRepPoint2 test_point = first_pcurve->param_geometry()->evaluate(first_pcurve->param_range().get_mid());

            for (int j = 0; j < faces.size(); j++) {
                if (ContainmentQuery::contained(faces[j], test_point) == ContainmentQuery::ContainmentResult::Inside) {
                    Face* new_face = split_face_with_trimming_loop(faces[j], curves[i]);
                    faces.push_back(new_face);
                }
            }
        }
        return faces;
    }

private:
    static Face* split_face_with_trimming_loop(Face *face, TrimmingLoop trimming_loop) {
        auto allocator = BRepAllocator::instance();

        Face* new_face = allocator->alloc_face();

        auto [loop, loop_reverse] = create_loops_from_trimming_loop(face, trimming_loop);

        if (not ContainmentQuery::is_hole(loop)) {
            std::swap(loop, loop_reverse);
        }

        loop->set_face(face);
        loop_reverse->set_face(new_face);

        loop->set_next(face->loop());
        face->set_loop(loop);

        new_face->set_next(face->next());
        new_face->set_geometry(face->geometry());
        new_face->set_loop(loop_reverse);
        face->set_next(new_face);

        return new_face;
  }

  static std::pair<Loop*, Loop*> create_loops_from_trimming_loop(Face* face, TrimmingLoop trimming_loop) {
        std::vector<BRepPoint2> pcurve_start, pcurve_end;

        for (int i = 0; i < trimming_loop.pcurves.size(); i++) {
            auto param_pcurve = trimming_loop.pcurves[i]->param_geometry();
            auto param_range = trimming_loop.pcurves[i]->param_range();
            pcurve_start.emplace_back(param_pcurve->evaluate(param_range.start()));
            pcurve_end.emplace_back(param_pcurve->evaluate(param_range.end()));
        }

        std::vector<Curve*> curves;
        std::vector<Edge*> edges;
        std::vector<Coedge*> coedges;
        std::vector<Coedge*> reverse_coedges;

        for (int i = 0; i < trimming_loop.pcurves.size(); i++) {
            auto param_pcurve = trimming_loop.pcurves[i]->param_geometry();
            auto param_curve = TopologyUtils::create_param_curve_from_pcurve(face->geometry()->param_geometry(), param_pcurve);
            auto curve = TopologyUtils::create_curve_from_param_curve(param_curve);
            auto edge = TopologyUtils::create_edge_from_curve(curve);
            auto coedge = TopologyUtils::create_coedge_from_edge(edge);
            coedge->set_geometry(trimming_loop.pcurves[i]);
            edge->set_coedge(coedge);

            auto reverse_edge = TopologyUtils::create_edge_from_curve(curve);
            auto reverse_coedge = TopologyUtils::create_coedge_from_edge(reverse_edge);
            reverse_coedge->set_forward(false);
            reverse_coedge->set_geometry(trimming_loop.pcurves[i]);


            curves.emplace_back(curve);
            edges.emplace_back(edge);
            coedges.emplace_back(coedge);
            reverse_coedges.emplace_back(reverse_coedge);
        }

        for (int i = 1; i < trimming_loop.pcurves.size(); i++) {
            coedges[i - 1]->set_next(coedges[i]);
            reverse_coedges[i]->set_next(reverse_coedges[i - 1]);
        }
        coedges.back()->set_next(coedges.front());
        reverse_coedges.front()->set_next(reverse_coedges.back());

        Loop* loop = TopologyUtils::create_loop_from_coedge(coedges.front());
        Loop* reverse_loop = TopologyUtils::create_loop_from_coedge(reverse_coedges.back());

        return {loop, reverse_loop};
    }

    static BRepPoint2 sample_param_point_in_loop(Loop* loop) {
        PCurve* pcurve = loop->coedge()->geometry();
        ParamCurve2D* param_pcurve = pcurve->param_geometry();

        auto param = pcurve->param_range().get_mid();
        return param_pcurve->evaluate(param);
    }
};

}