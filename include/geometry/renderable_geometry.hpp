#pragma once

#include <concepts>

#include "language/meta_programming/type_register/type_register.hpp"

using namespace MetaProgramming;

struct MeshGeometryType {};
META_REGISTER_TYPE(RenderableGeometryTag, MeshGeometryType);

class GeometryBase {};

template <typename T>
concept RenderableGeometry = requires(T t) {
    typename T::IsRenderableGeometry;
    typename T::render_type;
    { t.getRenderModel() } -> std::same_as<typename T::render_type>;
};