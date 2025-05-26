#include "project.hpp"

ReflectDataType DelaunayDemoProject::reflect() {
    auto result = IGraphicsLabProject::reflect();
    result.emplace("tick", TypeErasedValue(&DelaunayDemoProject::tick, this));
    result.emplace("visualize_hyperbolic_tessellation",
                   TypeErasedValue(&DelaunayDemoProject::visualize_hyperbolic_tessellation, this));
    result.emplace("visualize_spherical_voronoi",
                   TypeErasedValue(&DelaunayDemoProject::visualize_spherical_voronoi, this));
    return result;
}