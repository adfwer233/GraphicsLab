#pragma once

#include "glm/glm.hpp"
#include "vulkan/vulkan.h"

#include "vkl/scene_tree/vkl_camera.hpp"

#include "vkl/templates/vkl_concept.hpp"

struct PointLight {
    glm::vec4 position;
    glm::vec4 color;
};

struct GlobalUbo {
    glm::mat4 model;
    glm::mat4 view;
    glm::mat4 proj;

    PointLight pointLight;
    glm::vec3 cameraPos;
};

template <VklRenderable RenderableModel> struct FrameInfo {
    int frameIndex;
    float frameTime;
    VkCommandBuffer commandBuffer;
    Camera &camera;
    RenderableModel &model;
};