#pragma once

#include "geometry/boundary_representation/allocator/brep_allocator.hpp"
#include "geometry/boundary_representation/brep_definition.hpp"
#include "geometry/parametric/plane.hpp"

namespace GraphicsLab::Geometry::BRep {

struct FaceConstructors {

    static Face* plane(glm::vec3 base_pos, glm::vec3 direction1, glm::vec3 direction2) {
        auto allocator = BRepAllocator::instance();
        auto param_geo = allocator.alloc_param_surface<Plane>(base_pos, direction1, direction2);
        auto surface = allocator.alloc_surface();
        surface->set_param_geometry(param_geo);

        auto face = allocator.alloc_face();
        face->set_geometry(surface);

        return face;
    }

};

struct BodyConstructors {

    static Body *cube(glm::vec3 min_pos, glm::vec3 max_pos) {
        auto allocator = BRepAllocator::instance();

        // Plane* plane1 = allocator.alloc_surface<Plane>()

        return nullptr;
    }
};

}