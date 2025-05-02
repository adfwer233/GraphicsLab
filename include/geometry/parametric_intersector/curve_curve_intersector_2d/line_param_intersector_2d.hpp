#pragma once

#include "curve_curve_inter_result.hpp"
#include "bezier_bezier_intersection_2d.hpp"

#include <geometry/parametric/bspline_curve_2d.hpp>
#include <geometry/parametric/parametric_curves/straight_line.hpp>

namespace GraphicsLab::Geometry {

struct LineParamIntersector2D {
    static CurveCurveIntersectionResult2D intersect(StraightLine2D &line, ParamCurve2D &curve) {
        return {};
    }
};

struct LineBezierParamIntersector2D {
    static CurveCurveIntersectionResult2D intersect(const StraightLine2D &line, const BezierCurve2D &curve) {
        BezierCurve2D l({line.start_point, line.end_point});
        return BezierBezierIntersector2D::intersect(l, curve);
    }
};

struct LineBSplineParamIntersector2D {
    static CurveCurveIntersectionResult2D intersect(const StraightLine2D &line, BSplineCurve2D & curve) {
        CurveCurveIntersectionResult2D result;

        curve.insert_all_knots_to_bezier_form();
        auto bezier_curves = curve.convert_to_bezier();

        for (int i = 0; i < bezier_curves.size(); i++) {
            double knot_begin = curve.knots_[curve.degree_ * i + 1];
            double knot_end = curve.knots_[curve.degree_ * (i + 1) + 1];

            auto line_bezier_result = LineBezierParamIntersector2D::intersect(line, bezier_curves[i]);

            for (size_t j = 0; j < line_bezier_result.curve1_param.size(); j++) {
                result.curve1_param.push_back(line_bezier_result.curve1_param[j]);
                double curve2_param = line_bezier_result.curve2_param[j];
                result.curve2_param.push_back(knot_begin * (1 - curve2_param) + knot_end * curve2_param);
                result.inter_points.push_back(line_bezier_result.inter_points[j]);

                // double final_param = knot_begin * (1 - curve2_param) + knot_end * curve2_param;
                // spdlog::info("knot: {} {}, param: {}", knot_begin, knot_end, final_param);
                // auto pos = curve.evaluate(final_param);
                // auto dis = glm::distance(pos, line_bezier_result.inter_points[j]);
                // auto dis2 = glm::distance(bezier_curves[i].evaluate(curve2_param), line_bezier_result.inter_points[j]);
                // spdlog::info("Curve curve intersection distance {} {}", dis, dis2);
            }
        }
        return result;
    }
};

} // namespace GraphicsLab::Geometry