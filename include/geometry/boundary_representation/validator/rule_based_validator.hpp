#pragma once

#include "geometry/boundary_representation/brep_definition.hpp"

namespace GraphicsLab::Geometry::BRep {

struct RuleBasedValidator {

    static bool validate(Face *face) {
        if (face == nullptr) {
             return false;
        }

        return true;
    }

    static bool validate(Body* body) {

        if (body == nullptr) {
            return false;
        }

        return true;
    }
};

}