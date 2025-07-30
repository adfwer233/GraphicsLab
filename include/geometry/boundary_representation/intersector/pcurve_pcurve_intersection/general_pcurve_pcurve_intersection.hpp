#pragma once

#include "bezier_bezier_intersection.hpp"
#include "cpptrace/cpptrace.hpp"

#include "geometry/parametric/bezier_curve_2d.hpp"
#include "geometry/parametric/bspline_curve_2d.hpp"
#include "geometry/parametric/parametric_curves/parametric_curve.hpp"
#include "geometry/parametric/parametric_curves/straight_line.hpp"
#include "ppi_results.hpp"

namespace GraphicsLab::Geometry::BRep {

/**
 * @brief Intersector for pcurves.
 *
 * We convert all pcurves to the Bezier curves and solve the intersection via bezier clipping
 */
struct GeneralPCurvePCurveIntersection {
    static std::vector<PPIResult> solve(const ParamCurve2D *pcurve1, const ParamCurve2D *pcurve2, BRepPoint2 offset = BRepPoint2(0)) {
        return param_curves_intersection(pcurve1, pcurve2, offset);
    }

  private:
    static std::vector<PPIResult> param_curves_intersection(const ParamCurve2D *pcurve1, const ParamCurve2D *pcurve2, const BRepPoint2 offset) {
        std::vector<BezierCurve2D> bezier_segments_of_curve1;
        std::vector<BezierCurve2D> bezier_segments_of_curve2;

        auto convert_pcurve_to_bezier_curves = [](const ParamCurve2D *curve, std::vector<BezierCurve2D> &segments) {
            if (auto bezier_curve = dynamic_cast<const BezierCurve2D *>(curve)) {
                BezierCurve2D bezier_copy = *bezier_curve;
                segments.emplace_back(std::move(bezier_copy));
            } else if (auto line = dynamic_cast<const StraightLine2D *>(curve)) {
                BezierCurve2D bezier_line{{line->start_point, line->end_point}};
                segments.emplace_back(std::move(bezier_line));
            } else if (auto bspline = dynamic_cast<const BSplineCurve2D *>(curve)) {
                BSplineCurve2D bspline_copy = *bspline;
                bspline_copy.insert_all_knots_to_bezier_form();
                auto bezier_segments = bspline_copy.convert_to_bezier();
                std::ranges::copy(bezier_segments, std::back_inserter(segments));
            } else {
                throw cpptrace::logic_error("2d param curve can not be converted to bezier");
            }
        };

        convert_pcurve_to_bezier_curves(pcurve1, bezier_segments_of_curve1);
        convert_pcurve_to_bezier_curves(pcurve2, bezier_segments_of_curve2);

        std::vector<PPIResult> result;

        for (int i = 0; i < bezier_segments_of_curve1.size(); ++i) {
            for (int j = 0; j < bezier_segments_of_curve2.size(); ++j) {
                auto segment_inter =
                    BezierBezierIntersector2D::intersect(bezier_segments_of_curve1[i], bezier_segments_of_curve2[j], offset);

                for (auto inter : segment_inter) {
                    PPIResult res;
                    res.inter_position = inter.inter_position;
                    res.param1 = (i + inter.param1) / bezier_segments_of_curve1.size();
                    res.param2 = (j + inter.param2) / bezier_segments_of_curve2.size();

                    result.push_back(std::move(res));
                }
            }
        }

        return result;
    }
};

} // namespace GraphicsLab::Geometry::BRep