#pragma once

#include "geometry/vertex/vertex2d.hpp"
#include "geometry/vertex/vertex3d.hpp"

template <typename T>
concept VklVertexType = std::is_same_v<T, Vertex3D> || std::is_same_v<T, Vertex2D> || std::is_same_v<T, Vertex2DRaw> ||
                        std::is_same_v<T, Vertex3DRaw> || std::is_same_v<T, DirectionalFieldVertex3D>;

template <typename T>
concept VklRenderable = requires(T t, VkCommandBuffer commandBuffer) {
    t.bind(commandBuffer);
    t.draw(commandBuffer);
};

// utils

template <typename T, typename... U>
concept IsAnyOf = (std::same_as<T, U> || ...);