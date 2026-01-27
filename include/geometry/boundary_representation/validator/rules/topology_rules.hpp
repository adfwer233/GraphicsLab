#pragma once

#include "geometry/boundary_representation/validator/validation_info.hpp"
#include "geometry/boundary_representation/brep_definition.hpp"

namespace GraphicsLab::Geometry::BRep {

struct TopologyValidationRule {
    static void validate(Face* face, ValidationInfo& info) {
        auto loops = TopologyUtils::get_all_loops(face);

        for (auto loop: loops) {
            if (loop->face() == nullptr) {
                info.error_messages.push_back("loop has no face");
            }
        }
    }
};

}