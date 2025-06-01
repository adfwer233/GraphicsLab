#include "project.hpp"

ReflectDataType DelaunayDemoProject::reflect() {
    auto result = IGraphicsLabProject::reflect();
    result.emplace("tick", TypeErasedValue(&DelaunayDemoProject::tick, this));
    result.emplace("visualize_hyperbolic_tessellation",
                   TypeErasedValue(&DelaunayDemoProject::visualize_hyperbolic_tessellation, this, {7, 3, 3}, {"p", "q", "depth"}));
    result.emplace("visualize_spherical_voronoi",
                   TypeErasedValue(&DelaunayDemoProject::visualize_spherical_voronoi, this, {64, 10}, {"#Vertices", "#Tessellation"}));
    return result;
}