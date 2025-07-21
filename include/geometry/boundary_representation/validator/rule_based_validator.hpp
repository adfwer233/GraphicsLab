#pragma once

#include "geometry/boundary_representation/brep_definition.hpp"

namespace GraphicsLab::Geometry::BRep {

struct RuleBasedValidator {

    static bool validate(Face *face) {
        return true;
    }

    static bool validate(Body* body) {
        return true;
    }
};

}