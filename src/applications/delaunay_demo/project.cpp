#include "project.hpp"

ReflectDataType DelaunayDemoProject::reflect() {
    auto result = IGraphicsLabProject::reflect();
    result.emplace("tick", TypeErasedValue(&DelaunayDemoProject::tick, this));
    return result;
}