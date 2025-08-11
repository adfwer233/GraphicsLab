#pragma once
#include "csi_results.hpp"
#include "geometry/parametric/parametric_curves/straight_line.hpp"
#include "geometry/parametric/torus.hpp"

namespace GraphicsLab::Geometry::BRep {

struct LineTorusIntersection {

    static std::vector<CSIResult> solve(const StraightLine3D *line, const Torus *torus, bool check_line_range = true) {
        std::vector<CSIResult> csi_results;

        return csi_results;
    }

};

}
