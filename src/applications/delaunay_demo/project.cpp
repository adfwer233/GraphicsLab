#include "project.hpp"

ReflectDataType DelaunayDemoProject::reflect() {
    auto result = IGraphicsLabProject::reflect();
    result.clear();
    result.emplace(
        "visualize_hyperbolic_tessellation",
        TypeErasedValue(&DelaunayDemoProject::visualize_hyperbolic_tessellation, this, {7, 3, 3}, {"p", "q", "depth"}));
    result.emplace("visualize_spherical_voronoi", TypeErasedValue(&DelaunayDemoProject::visualize_spherical_voronoi,
                                                                  this, {64, 10}, {"#Vertices", "#Tessellation"}));

    result.emplace("visualize_hyperbolic_delaunay", TypeErasedValue(&DelaunayDemoProject::visualize_hyperbolic_delaunay, this));
    return result;
}