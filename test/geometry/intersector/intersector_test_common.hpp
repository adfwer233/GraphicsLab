#pragma once

#include "gtest/gtest.h"

#include "geometry/boundary_representation/base/vec_def.hpp"
#include "geometry/boundary_representation/intersector/curve_surface_intersection/csi_results.hpp"
#include "geometry/boundary_representation/intersector/pcurve_pcurve_intersection/ppi_results.hpp"
#include "geometry/boundary_representation/intersector/surface_surface_intersection/ssi_results.hpp"
#include "geometry/parametric/parametric_curves/parametric_curve.hpp"
#include "geometry/parametric/parametric_surface.hpp"

#include <algorithm>
#include <array>
#include <vector>

namespace GraphicsLab::Geometry::BRep::IntersectorTest {

inline constexpr double kTightTol = 1e-6;
inline constexpr double kDefaultTol = 1e-4;
inline constexpr double kLooseTol = 2e-2;

inline void expect_csi_residual_ok(const ParamCurve3D *curve, const ParamSurface *surface, const CSIResult &res,
                                   double tolerance = kDefaultTol) {
    ASSERT_NE(curve, nullptr);
    ASSERT_NE(surface, nullptr);

    const auto curve_pos = curve->evaluate(res.curve_parameter);
    const auto surf_pos = surface->evaluate(res.surface_parameter);

    EXPECT_NEAR(glm::distance(res.inter_position, curve_pos), 0.0, tolerance);
    EXPECT_NEAR(glm::distance(res.inter_position, surf_pos), 0.0, tolerance);
    EXPECT_NEAR(glm::distance(curve_pos, surf_pos), 0.0, tolerance);
}

inline void expect_ppi_residual_ok(const ParamCurve2D *curve1, const ParamCurve2D *curve2, const PPIResult &res,
                                   BRepPoint2 offset = BRepPoint2(0), double tolerance = kDefaultTol) {
    ASSERT_NE(curve1, nullptr);
    ASSERT_NE(curve2, nullptr);

    const auto p1 = curve1->evaluate(res.param1) + offset;
    const auto p2 = curve2->evaluate(res.param2);

    EXPECT_NEAR(glm::distance(res.inter_position, p1), 0.0, tolerance);
    EXPECT_NEAR(glm::distance(res.inter_position, p2), 0.0, tolerance);
    EXPECT_NEAR(glm::distance(p1, p2), 0.0, tolerance);
}

inline void expect_ssi_curve_residual_ok(const ParamSurface *surface1, const ParamSurface *surface2, const SSIResult &res,
                                         double tolerance = kLooseTol) {
    ASSERT_NE(surface1, nullptr);
    ASSERT_NE(surface2, nullptr);
    ASSERT_NE(res.inter_curve, nullptr);
    ASSERT_NE(res.pcurve1, nullptr);
    ASSERT_NE(res.pcurve2, nullptr);

    constexpr std::array<double, 5> sample_params{0.1, 0.3, 0.5, 0.7, 0.9};

    for (const double t : sample_params) {
        const auto curve_pos = res.inter_curve->evaluate(t);

        const auto proj1 = surface1->project(curve_pos).first;
        const auto proj2 = surface2->project(curve_pos).first;
        EXPECT_NEAR(glm::distance(curve_pos, proj1), 0.0, std::max(tolerance, kDefaultTol));
        EXPECT_NEAR(glm::distance(curve_pos, proj2), 0.0, std::max(tolerance, kDefaultTol));

        const auto uv1 = res.pcurve1->evaluate(t);
        const auto uv2 = res.pcurve2->evaluate(t);

        const auto pcurve1_pos = surface1->evaluate(uv1);
        const auto pcurve2_pos = surface2->evaluate(uv2);

        // pcurve parameterizations are not guaranteed to be synchronized with inter_curve's t.
        // Validate geometrically: each pcurve-sampled point should lie on the opposite surface.
        const auto pcurve1_on_s2 = surface2->project(pcurve1_pos).first;
        const auto pcurve2_on_s1 = surface1->project(pcurve2_pos).first;

        EXPECT_NEAR(glm::distance(pcurve1_pos, pcurve1_on_s2), 0.0, tolerance);
        EXPECT_NEAR(glm::distance(pcurve2_pos, pcurve2_on_s1), 0.0, tolerance);
    }
}

} // namespace GraphicsLab::Geometry::BRep::IntersectorTest




