#pragma once

#include <numbers>
#include <set>

#include "cpptrace/exceptions.hpp"
#include "geometry/boundary_representation/allocator/brep_allocator.hpp"
#include "geometry/boundary_representation/brep_definition.hpp"
#include "topology_utils.hpp"

#include "periodic_wn.hpp"
#include "geometry/parametric/torus.hpp"

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
    static ContainmentResult contained(const Face *face, BRepPoint2 test_point) {
        double wn = 0;

        if (face->loop() == nullptr) {
            return ContainmentResult::Inside;
        }

        std::set<Loop *> non_contractible;
        for (Loop *loop : TopologyUtils::get_all_loops(face)) {
            auto [p, q] = TopologyUtils::get_loop_homology(loop);
            if (p != 0 or q != 0) {
                non_contractible.insert(loop);
            }
        }

        // if (auto torus = dynamic_cast<Torus*>(face->geometry()->param_geometry())) {
        //     auto test_point_std = torus->move_param_to_std_domain(test_point);
        //
        //     auto [wn, bd] = WNTrim::torus_winding_number(face, test_point_std, std::nullopt);
        //     if (bd) return ContainmentResult::Boundary;
        //
        //     return wn > std::numbers::pi ? ContainmentResult::Inside : ContainmentResult::Outside;
        // }

        for (Loop *loop : TopologyUtils::get_all_loops(face)) {
            if (non_contractible.contains(loop))
                continue;

            wn += winding_number_loop(loop, test_point);
            auto loop_homo = TopologyUtils::get_loop_homology(loop);

            if (face->geometry()->param_geometry()->u_periodic) {
                if (loop_homo.first == 0 and loop_homo.second == 0) {
                    wn += winding_number_loop(loop, test_point - BRepVector2{1.0, 0.0});
                    wn += winding_number_loop(loop, test_point + BRepVector2{1.0, 0.0});
                }
            }

            if (face->geometry()->param_geometry()->v_periodic) {
                if (loop_homo.first == 0 and loop_homo.second == 0) {
                    wn += winding_number_loop(loop, test_point - BRepVector2{0.0, 1.0});
                    wn += winding_number_loop(loop, test_point + BRepVector2{0.0, 1.0});
                }
            }
        }

        if (not non_contractible.empty()) {
            Loop *lp1 = *non_contractible.begin();
            Loop *lp2 = *std::prev(non_contractible.end());

            auto pt = lp1->coedge()->geometry()->param_geometry()->evaluate(lp1->coedge()->param_range().get_mid());

            auto wn1 = winding_number_loop(lp2, pt);

            BRepVector2 lp1_offset(0);
            auto [p, q] = TopologyUtils::get_loop_homology(lp1);

            if (wn1 < 0) {
                if (p == 0) {
                    lp1_offset = {-1.0, 0.0};
                    if (winding_number_loop(lp2, pt + lp1_offset) < 0) {
                        lp1_offset = -lp1_offset;
                    }
                } else if (q == 0) {
                    lp1_offset = {0.0, 1.0};
                }
            }

            wn += winding_number_loop(lp1, test_point - lp1_offset);
            wn += winding_number_loop(lp2, test_point);

            if (face->geometry()->param_geometry()->u_periodic and p == 0) {
                wn += winding_number_loop(lp1, test_point + BRepVector2{1.0, 0.0} - lp1_offset);
                wn += winding_number_loop(lp2, test_point + BRepVector2{1.0, 0.0});
                wn += winding_number_loop(lp1, test_point + BRepVector2{-1.0, 0.0} - lp1_offset);
                wn += winding_number_loop(lp2, test_point + BRepVector2{-1.0, 0.0});
            }
        }

        if (wn > std::numbers::pi) {
            return ContainmentResult::Inside;
        } else {
            return ContainmentResult::Outside;
        }
    }

    static bool is_hole(Loop *loop) {
        double wn = winding_number_loop(loop, std::nullopt);
        return wn < std::numbers::pi;
    }

    static double winding_number_loop(Loop *loop, std::optional<BRepPoint2> test_point_given) {
        std::vector<BRepPoint2> samples;

        if (loop->coedge() == nullptr) {
            throw cpptrace::logic_error("Loop has no coedge");
        }

        Coedge *coedge_start = loop->coedge();
        Coedge *coedge_iter = loop->coedge();

        constexpr int sample_per_pcurve = 25;

        std::optional<BRepPoint2> last_end;
        BRepVector2 offset(0);

        while (coedge_iter != nullptr) {
            if (coedge_iter->geometry() == nullptr) {
                throw cpptrace::logic_error("coedge has no pcurve");
            }

            auto pcurve = coedge_iter->geometry();
            auto pcurve_param_start = coedge_iter->param_range().start();
            auto pcurve_param_end = coedge_iter->param_range().end();
            auto delta = (pcurve_param_end - pcurve_param_start) / sample_per_pcurve;
            std::vector<BRepPoint2> pcurve_samples;
            for (int i = 0; i <= sample_per_pcurve; i++) {
                double param = pcurve_param_start + i * delta;
                pcurve_samples.emplace_back(pcurve->param_geometry()->evaluate(param));
            }

            if (not pcurve->is_forward()) {
                std::ranges::reverse(pcurve_samples);
            }

            if (last_end.has_value()) {
                double offset_x = pcurve_samples.front().x - last_end->x;
                double offset_y = pcurve_samples.front().y - last_end->y;

                offset += BRepVector2{std::round(offset_x), std::round(offset_y)};
            }

            last_end = pcurve_samples.back();

            for (auto &s : pcurve_samples) {
                s -= offset;
            }

            std::ranges::copy(pcurve_samples, std::back_inserter(samples));
            coedge_iter = coedge_iter->next();
            if (coedge_iter == coedge_start)
                break;
        }

        BRepPoint2 dir = samples[sample_per_pcurve / 2] - samples[sample_per_pcurve / 2 - 1];
        BRepPoint2 in_dir{-dir.y, dir.x};
        BRepPoint2 test_point = glm::mix(samples[sample_per_pcurve / 2 - 1], samples[sample_per_pcurve / 2], 0.5) +
                                glm::normalize(in_dir) * 1e-3;

        if (test_point_given.has_value()) {
            test_point = test_point_given.value();
        }

        auto compute_wn = [&](BRepPoint2 p) -> double {
            double wn = 0;
            for (size_t i = 1; i < samples.size(); i++) {
                wn += winding_number_line_segment(p, samples[i - 1], samples[i]);
            }
            return wn;
        };

        double wn = compute_wn(test_point);

        if (loop->face() != nullptr) {
            auto [p, q] = TopologyUtils::get_loop_homology(loop);

            if (p != 0 or q != 0) {
                wn += compute_wn(test_point + BRepVector2{p, q});
                wn += compute_wn(test_point + BRepVector2{-p, -q});

                auto wn_seg = winding_number_line_segment(test_point, samples.front(), samples.back());
                auto wn_seg1 =
                    winding_number_line_segment(test_point + BRepVector2{p, q}, samples.front(), samples.back());
                auto wn_seg2 =
                    winding_number_line_segment(test_point + BRepVector2{-p, -q}, samples.front(), samples.back());

                if (wn_seg > 0)
                    wn += std::numbers::pi;
                if (wn_seg < 0)
                    wn -= std::numbers::pi;
                wn -= wn_seg + wn_seg1 + wn_seg2;
            }
        }

        return wn;
    }

    static double winding_number_line_segment(BRepPoint2 test_point, BRepPoint2 start_pos, BRepPoint2 end_pos) {
        auto v1 = glm::normalize(start_pos - test_point);
        auto v2 = glm::normalize(end_pos - test_point);
        auto outer = v1.x * v2.y - v1.y * v2.x;
        auto inner = glm::dot(v1, v2);

        auto acos_value = std::acos(std::clamp(inner, -0.999999999, 0.99999999));

        if (std::isnan(acos_value)) {
            spdlog::info("NAN CASE");
            return 0;
        }

        return outer > 0 ? acos_value : -acos_value;
    }
};

struct TrimmingUtils {

    struct TrimmingLoop {
        std::vector<PCurve *> pcurves;
    };

    /**
     * @brief Trimming a surface with a set of loops
     * @param face
     * @param curves
     * @return A set of faces, split by given loops.
     */
    static std::vector<Face *> add_trimming_curve(Face *face, std::vector<TrimmingLoop> curves) {
        std::vector<Face *> faces;
        faces.push_back(face);

        for (auto &curve : curves) {
            PCurve *first_pcurve = curve.pcurves.front();
            BRepPoint2 test_point = first_pcurve->param_geometry()->evaluate(0.5);

            // int cur_faces = faces.size();
            for (size_t j = 0; j < faces.size(); j++) {
                if (ContainmentQuery::contained(faces[j], test_point) == ContainmentQuery::ContainmentResult::Inside) {
                    Face *new_face = split_face_with_trimming_loop(faces[j], curve);
                    faces.push_back(new_face);
                    break;
                }
            }
        }
        return faces;
    }

  private:
    static Face *split_face_with_trimming_loop(Face *face, TrimmingLoop trimming_loop) {
        auto allocator = BRepAllocator::instance();

        Face *new_face = allocator->alloc_face();

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

    static std::pair<Loop *, Loop *> create_loops_from_trimming_loop(Face *face, TrimmingLoop trimming_loop) {
        std::vector<BRepPoint2> pcurve_start, pcurve_end;

        for (size_t i = 0; i < trimming_loop.pcurves.size(); i++) {
            auto param_pcurve = trimming_loop.pcurves[i]->param_geometry();
            ParamRange param_range{0, 1};
            pcurve_start.emplace_back(param_pcurve->evaluate(param_range.start()));
            pcurve_end.emplace_back(param_pcurve->evaluate(param_range.end()));
        }

        std::vector<Curve *> curves;
        std::vector<Edge *> edges;
        std::vector<Coedge *> coedges;
        std::vector<Coedge *> reverse_coedges;

        for (auto &pcurve : trimming_loop.pcurves) {
            auto param_pcurve = pcurve->param_geometry();
            auto param_curve =
                TopologyUtils::create_param_curve_from_pcurve(face->geometry()->param_geometry(), param_pcurve);
            auto curve = TopologyUtils::create_curve_from_param_curve(param_curve);
            auto edge = TopologyUtils::create_edge_from_curve(curve);
            auto coedge = TopologyUtils::create_coedge_from_edge(edge);
            coedge->set_geometry(pcurve);
            edge->set_coedge(coedge);

            auto reverse_edge = TopologyUtils::create_edge_from_curve(curve);
            auto reverse_coedge = TopologyUtils::create_coedge_from_edge(reverse_edge);
            reverse_coedge->set_forward(false);
            auto reverse_pcurve = TopologyUtils::create_pcurve_from_param_pcurve(pcurve->param_geometry());
            reverse_pcurve->set_forward(false);
            reverse_coedge->set_geometry(reverse_pcurve);

            curves.emplace_back(curve);
            edges.emplace_back(edge);
            coedges.emplace_back(coedge);
            reverse_coedges.emplace_back(reverse_coedge);
        }

        for (size_t i = 1; i < trimming_loop.pcurves.size(); i++) {
            coedges[i - 1]->set_next(coedges[i]);
            reverse_coedges[i]->set_next(reverse_coedges[i - 1]);
        }
        coedges.back()->set_next(coedges.front());
        reverse_coedges.front()->set_next(reverse_coedges.back());

        Loop *loop = TopologyUtils::create_loop_from_coedge(coedges.front());
        Loop *reverse_loop = TopologyUtils::create_loop_from_coedge(reverse_coedges.back());

        for (size_t i = 0; i < trimming_loop.pcurves.size(); i++) {
            coedges[i]->set_loop(loop);
            reverse_coedges[i]->set_loop(reverse_loop);
        }

        return {loop, reverse_loop};
    }

    static BRepPoint2 sample_param_point_in_loop(Loop *loop) {
        PCurve *pcurve = loop->coedge()->geometry();
        ParamCurve2D *param_pcurve = pcurve->param_geometry();

        auto param = loop->coedge()->param_range().get_mid();
        return param_pcurve->evaluate(param);
    }
};

} // namespace GraphicsLab::Geometry::BRep