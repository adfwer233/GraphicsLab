#pragma once

#include <numbers>
#include <set>

#include "cpptrace/exceptions.hpp"
#include "geometry/boundary_representation/allocator/brep_allocator.hpp"
#include "geometry/boundary_representation/brep_definition.hpp"
#include "geometry/parametric/bezier_curve_2d.hpp"
#include "geometry/parametric/bspline_curve_2d.hpp"

#include "topology_utils.hpp"

namespace GraphicsLab::Geometry::BRep {

struct WNTrim {

    /**
     * @brief winding number computation result, (double: wn, bool: is on-boundary)
     */
    using WNResult = std::pair<double, bool>;

    /**
     * @brief  A Loop converted to the parameter domain.
     */
    struct ParameterDomainLoop {
        Loop *loop;

        std::vector<ParamCurve2D *> pcurves;
        std::vector<std::pair<double, double>> param_ranges;
        std::vector<bool> is_forward;
        std::vector<BRepVector2> offsets;

        BRepPoint2 start_position;
        BRepPoint2 end_position;

        std::pair<int, int> homology_coefficients;

        explicit ParameterDomainLoop(Loop *t_loop) {
            loop = t_loop;
            Coedge *coedge_start = loop->coedge();
            Coedge *coedge_iter = loop->coedge();

            std::optional<BRepPoint2> last_end;
            BRepVector2 offset(0);
            int counter = 0;

            double total_wn = 0.0;
            /**
             * traverse all p-curves on the loop
             */
            while (coedge_iter != nullptr) {
                if (coedge_iter->geometry() == nullptr) {
                    throw cpptrace::logic_error("coedge has no pcurve");
                }

                auto pcurve = coedge_iter->geometry();
                auto pcurve_param_start = coedge_iter->param_range().start();
                auto pcurve_param_end = coedge_iter->param_range().end();

                auto pcurve_start_pos = pcurve->param_geometry()->evaluate(pcurve_param_start);
                auto pcurve_end_pos = pcurve->param_geometry()->evaluate(pcurve_param_end);

                if (not pcurve->is_forward()) {
                    std::swap(pcurve_start_pos, pcurve_end_pos);
                }

                if (last_end.has_value()) {
                    double offset_x = pcurve_start_pos.x - last_end->x;
                    double offset_y = pcurve_end_pos.x - last_end->y;

                    offset += BRepVector2{std::round(offset_x), std::round(offset_y)};
                }

                last_end = pcurve_end_pos;

                this->pcurves.push_back(pcurve->param_geometry());
                this->is_forward.push_back(pcurve->is_forward());
                this->param_ranges.emplace_back(pcurve_param_start, pcurve_param_end);
                this->offsets.push_back(offset);

                if (counter == 0) {
                    this->start_position = pcurve_start_pos;
                }

                coedge_iter = coedge_iter->next();
                counter += 1;
                if (coedge_iter == coedge_start or coedge_iter == nullptr) {
                    this->end_position = pcurve_end_pos;
                    break;
                }
            }

            this->homology_coefficients = TopologyUtils::get_loop_homology(t_loop);
        }
    };

    using LoopVector = std::vector<ParameterDomainLoop>;
    using LoopPair = std::pair<ParameterDomainLoop, ParameterDomainLoop>;
    using LoopPairVector = std::vector<LoopPair>;

    static WNResult winding_number(ParamCurve2D *pcurve, const BRepPoint2 &test_point) {
        if (auto line = dynamic_cast<StraightLine2D *>(pcurve)) {
            BezierCurve2D bezier({line->start_point, line->end_point});
            return bezier.winding_number(test_point, 1e-6);
        } else if (auto bezier = dynamic_cast<BezierCurve2D *>(pcurve)) {
            return bezier->winding_number(test_point, 1e-6);
        } else if (auto bspline = dynamic_cast<BSplineCurve2D *>(pcurve)) {
            BSplineCurve2D bs = *bspline;
            if (not bs.is_in_bezier_form())
                bs.insert_all_knots_to_bezier_form();
            auto bezier_curves = bs.convert_to_bezier();
            double total_wn = 0;
            for (auto &c : bezier_curves) {
                auto [wn, bd] = c.winding_number(test_point, 1e-6);
                total_wn += wn;
                if (bd)
                    return {wn, true};
            }
            return {total_wn, false};
        }
        return {0, false};
    }

    // static WNResult turning_number(ParamCurve2D *pcurve) {
    //     BRepPoint2 origin{0, 0};
    //
    //     if (auto line = dynamic_cast<StraightLine2D *>(pcurve)) {
    //         BezierCurve2D bezier({line->start_point, line->end_point});
    //         return bezier.derivative_curve().winding_number(origin, 1e-6);
    //     } else if (auto bezier = dynamic_cast<BezierCurve2D *>(pcurve)) {
    //         return bezier->derivative_curve().winding_number(origin, 1e-6);
    //     } else if (auto bspline = dynamic_cast<BSplineCurve2D *>(pcurve)) {
    //         BSplineCurve2D bs = *bspline;
    //         if (not bs.is_in_bezier_form())
    //             bs.insert_all_knots_to_bezier_form();
    //         auto bezier_curves = bs.convert_to_bezier();
    //
    //         bool isbezier = bs.is_in_bezier_form();
    //
    //         double total_wn = 0;
    //         for (auto &c : bezier_curves) {
    //             auto [wn, bd] = c.derivative_curve().winding_number(origin, 1e-6);
    //             total_wn += wn;
    //             if (bd)
    //                 return {wn, true};
    //         }
    //         return {total_wn, false};
    //     }
    //     return {0, false};
    // }

    using LoopClassificationResult = std::tuple<std::vector<int>, std::vector<int>, std::vector<int>>;

    /**
     * @brief Auxiliary function: classify loops via the "type classification theorem"
     * @param loops
     * @return
     */
    static auto create_classification_result(LoopVector &loops) -> LoopClassificationResult {
        int p = 0, q = 0;

        bool found = false;

        std::vector<int> type_a_loops, type_b_loops, type_c_loops;

        for (auto &l : loops) {
            if (l.homology_coefficients.first != 0 or l.homology_coefficients.second != 0) {
                p = l.homology_coefficients.first;
                q = l.homology_coefficients.second;

                found = true;
                break;
            }
        }

        if (not found) {
            for (int i = 0; i < loops.size(); ++i)
                type_c_loops.push_back(i);
        } else {
            for (int i = 0; i < loops.size(); ++i) {
                auto &l = loops[i];

                if (l.homology_coefficients.first == p and l.homology_coefficients.second == q) {
                    type_a_loops.push_back(i);
                } else if (l.homology_coefficients.first == -p and l.homology_coefficients.second == -q) {
                    type_b_loops.push_back(i);
                } else if (l.homology_coefficients.first == 0 and l.homology_coefficients.second == 0) {
                    type_c_loops.push_back(i);
                }
            }

            if (type_a_loops.size() != type_b_loops.size()) {
                throw cpptrace::runtime_error(
                    "Topology incorrect since there are different numbers of type A loop and type B loop");
            }
        }

        return std::make_tuple(type_a_loops, type_b_loops, type_c_loops);
    }

    /**
     * @brief compute loop winding number in parameter domain
     * @param loop
     * @param test_point
     * @return
     */
    static WNResult winding_number(ParameterDomainLoop &loop, const BRepPoint2 &test_point) {
        // if (loop->coedge() == nullptr) {
        //     throw cpptrace::logic_error("Loop has no coedge");
        // }

        Coedge *coedge_start = loop.loop->coedge();
        Coedge *coedge_iter = loop.loop->coedge();

        std::optional<BRepPoint2> last_end;
        BRepVector2 offset(0);

        double total_wn = 0.0;
        /**
         * traverse all p-curves on the loop
         */
        while (coedge_iter != nullptr) {
            if (coedge_iter->geometry() == nullptr) {
                throw cpptrace::logic_error("coedge has no pcurve");
            }

            auto pcurve = coedge_iter->geometry();
            auto pcurve_param_start = coedge_iter->param_range().start();
            auto pcurve_param_end = coedge_iter->param_range().end();

            auto pcurve_start_pos = pcurve->param_geometry()->evaluate(pcurve_param_start);
            auto pcurve_end_pos = pcurve->param_geometry()->evaluate(pcurve_param_end);

            WNResult wn = winding_number(pcurve->param_geometry(), test_point + offset);

            if (wn.second)
                return {0, true};
            if (not pcurve->is_forward()) {
                wn.first = -wn.first;

                std::swap(pcurve_start_pos, pcurve_end_pos);
            }

            if (last_end.has_value()) {
                double offset_x = pcurve_start_pos.x - last_end->x;
                double offset_y = pcurve_end_pos.x - last_end->y;

                offset += BRepVector2{std::round(offset_x), std::round(offset_y)};
            }

            last_end = pcurve_end_pos;
            total_wn += wn.first;

            coedge_iter = coedge_iter->next();
            if (coedge_iter == coedge_start)
                break;
        }

        return {total_wn, false};
    }

    static double winding_number_line(BRepPoint2 test_param, BRepPoint2 base_point, BRepPoint2 direction) {
        BRepVector2 base_to_test = test_param - base_point;
        double cross = direction.x * base_to_test.y - direction.y * base_to_test.x;
        double result =
            direction.x * base_to_test.y - direction.y * base_to_test.x > 0 ? std::numbers::pi : -std::numbers::pi;
        return result;
    }

    /**
     * @brief winding number respect to a line segment
     */
    static WNResult winding_number_line_segment(BRepPoint2 test_point, BRepPoint2 start_pos, BRepPoint2 end_pos) {
        auto d1 = glm::length(start_pos - test_point);
        auto d2 = glm::length(end_pos - test_point);

        if (d1 < 1e-6 or d2 < 1e-6)
            return {0, true};

        auto v1 = glm::normalize(start_pos - test_point);
        auto v2 = glm::normalize(end_pos - test_point);
        auto outer = v1.x * v2.y - v1.y * v2.x;
        auto inner = glm::dot(v1, v2);

        auto acos_value = std::acos(inner);

        return {outer > 0 ? acos_value : -acos_value, false};
    }

    /**
     * @brief compute lifted loop winding number
     * @param loop
     * @param test_point
     * @return
     */
    static WNResult periodically_extended_winding_number(ParameterDomainLoop &loop, const BRepPoint2 &test_point) {

        // temporally, use 3 replica
        double total_wn = 0.0;

        auto [p, q] = loop.homology_coefficients;

        total_wn += winding_number_line(test_point, loop.start_position, loop.end_position - loop.start_position);

        for (int i = -1; i <= 1; i++) {
            auto [wn, bd] = winding_number(loop, test_point + BRepVector2{p * i, q * i});
            if (bd)
                return {0, true};
            total_wn += wn;

            double wn_seg = winding_number_line_segment(test_point + BRepVector2{p * i, q * i}, loop.start_position,
                                                        loop.end_position)
                                .first;

            total_wn -= wn_seg;
        }

        return {total_wn, false};
    }

    /**
     * @brief compute winding number of a lifted loop pairs
     * @param loop_pair
     * @param test_point
     * @return
     */
    static WNResult periodically_extended_winding_number(LoopPair &loop_pair, const BRepPoint2 &test_point) {
        auto [wn1, bd1] = periodically_extended_winding_number(loop_pair.first, test_point);
        if (bd1)
            return {0, true};

        auto [wn2, bd2] = periodically_extended_winding_number(loop_pair.second, test_point);
        if (bd2)
            return {0, true};

        return {wn1 + wn2, false};
    }

    static WNResult sphere_covering_space_winding_number(LoopVector &loops, const BRepPoint2 &test_point) {
        if (loops.empty()) {
            throw cpptrace::runtime_error("Empty loop vector");
        }

        double total_wn = 0.0;

        int homology_p = 0, homology_q = 0;

        for (auto &loop : loops) {
            homology_p += loop.homology_coefficients.first;
            homology_q += loop.homology_coefficients.second;
        }

        if (homology_p != 0 or homology_q != 0) {
            total_wn += std::numbers::pi;
        }

        if (homology_p > 1 or homology_q > 1) {
            throw cpptrace::runtime_error("Incorrect topology");
        }

        auto [wn, bd] = cylinder_covering_space_winding_number(loops, test_point);
        if (bd)
            return {0, true};
        total_wn += wn;

        return {total_wn, false};
    }

    static WNResult cylinder_covering_space_winding_number(LoopVector &loops, const BRepPoint2 &test_point) {
        if (loops.empty()) {
            throw cpptrace::runtime_error("Empty loop vector");
        }

        double total_wn = 0.0;
        for (auto &loop : loops) {
            auto [wn, bd] = periodically_extended_winding_number(loop, test_point);
            if (bd)
                return {0, true};
            total_wn += wn;
        }

        return {total_wn, false};
    }

    /**
     * @brief compute winding number in the universal covering space of a torus
     * @param loop_pairs
     * @param test_point
     * @return
     */
    static WNResult torus_covering_space_winding_number(LoopPairVector &loop_pairs, const BRepPoint2 &test_point) {
        if (loop_pairs.empty()) {
            throw cpptrace::runtime_error("Empty loop pair vector");
        }

        auto [p, q] = loop_pairs.front().first.homology_coefficients;

        double total_wn = 0.0;
        for (auto &loop_pair : loop_pairs) {
            for (int i = 0; i < std::abs(p); i++) {
                for (int j = -std::abs(q); j < std::abs(q); j++) {
                    auto [wn, bd] = periodically_extended_winding_number(loop_pair, test_point - BRepVector2{i, j});
                    if (bd)
                        return {0, true};
                    total_wn += wn;
                }
            }
        }

        return {total_wn, false};
    }

    static WNResult torus_winding_number(const Face *face, const BRepPoint2 &test_point,
                                         std::optional<LoopPairVector> loop_pair_vec) {

        if (loop_pair_vec.has_value()) {
            return torus_covering_space_winding_number(loop_pair_vec.value(), test_point);
        } else {
            auto loops = TopologyUtils::get_all_loops(face);
            LoopVector domain_loops;
            for (auto l : loops) {
                domain_loops.emplace_back(l);
            }

            LoopClassificationResult type_classification = create_classification_result(domain_loops);
            LoopPairVector loop_pairs;

            auto [indices_a, indices_b, indices_c] = type_classification;

            for (int i = 0; i < indices_a.size(); i++) {
                loop_pairs.emplace_back(loops[indices_a[i]], loops[indices_b[i]]);
            }

            if (loop_pairs.empty()) {
                return {6.28, false};
            }

            return torus_covering_space_winding_number(loop_pairs, test_point);
        }
    }

    // static WNResult loop_turning_number(Loop *loop) {
    //     ParameterDomainLoop loop_domain(loop);
    //
    //     double result = 0.0;
    //
    //     for (size_t i = 0; i < loop_domain.pcurves.size(); i++) {
    //         auto [tn, bd] = turning_number(loop_domain.pcurves[i]);
    //         result += tn;
    //     }
    //
    //     return {result, false};
    // }
};

} // namespace GraphicsLab::Geometry::BRep