#pragma once

#include "geometry/parametric/bezier_curve_2d.hpp"
#include "geometry/parametric/parametric_curves/parametric_curve.hpp"
#include "ppi_results.hpp"

namespace GraphicsLab::Geometry::BRep {

/**
 * @brief Intersector for pcurves.
 */
struct GeneralPCurvePCurveIntersection {
  private:
    std::vector<PPIResult> param_curves_intersection(const ParamCurve2D *pcurve1, const ParamCurve2D *pcurve2) {
        std::vector<BezierCurve2D> bezier_segments_of_curve1;
        std::vector<BezierCurve2D> bezier_segments_of_curve2;

        auto convert_pcurve_to_bezier_curves = [](const ParamCurve2D *curve, std::vector<BezierCurve2D> &segments) {

        };

        convert_pcurve_to_bezier_curves(pcurve1, bezier_segments_of_curve1);
        convert_pcurve_to_bezier_curves(pcurve2, bezier_segments_of_curve2);
    }
};

} // namespace GraphicsLab::Geometry::BRep