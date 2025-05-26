#include "project.hpp"

ReflectDataType DelaunayDemoProject::reflect() {
    auto result = IGraphicsLabProject::reflect();
    result.emplace("tick", TypeErasedValue(&DelaunayDemoProject::tick, this));
    result.emplace("visualize_hyperbolic_tessellation", TypeErasedValue(&DelaunayDemoProject::visualize_hyperbolic_tessellation, this));
    return result;
}