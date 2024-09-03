#pragma once

#include "vkl/core/vkl_offscreen_renderer.hpp"
#include "vkl/scene/vkl_scene.hpp"

#include "components/bezier_editor/bezier_editor_panel_ui.hpp"
#include "components/bezier_editor/bezier_editor_ui.hpp"
#include "components/geometry_processing_ui.hpp"
#include "components/material_ui.hpp"
#include "components/render_mode_ui.hpp"
#include "components/scene_info.hpp"
#include "components/scene_manager.hpp"
#include "components/scene_ui.hpp"
#include "components/uv_render_ui.hpp"
#include "ray_tracer/ray_picker.hpp"

#include "components/picking_ui.hpp"
#include "components/scene_render_ui.hpp"

#include <optional>

#include "vkl/system/compute_system/path_tracing_compute_system.hpp"

#include "geometry/curve/bezier_curve_2d.hpp"

#include "geometry_processing/variable_manager.hpp"

#include "ui_manager_register.hpp"

enum RenderMode {
    Raw,
    WireFrame,
    WithTexture,
    PathTracing
};

enum ShadingMode {
    PointLightShading,
    SolidShading,
    PureColor
};

class UIManager {
  private:
    VklScene &scene_;

    GET_REGISTERED_TYPES(UIManagerRegisteredTypeTag) uiManagerComponentList;

  public:
    VklDevice &device_;

    // used in scene ui
    float deltaTime;

    /**
     * picking result, used in picking UI and box rendering
     */
    std::optional<RayPicker::RayPickingResult> picking_result;

    /**
     * renderModel, used in render mode ui
     */
    RenderMode renderMode = RenderMode::Raw;

    /**
     * shading mode, point light or solid
     */
    ShadingMode shadingMode = ShadingMode::PointLightShading;

    /**
     * whether show the normal vector
     */
    bool showNormal = false;

    float normalStrength = 0.1;

    glm::vec3 normalColor{1.0, 0.0, 0.0};

    ModelLoader modelLoader = AssimpLoader;

    /**
     * path tracing result texture
     */
    VklTexture *renderResultTexture = nullptr;
    VkDescriptorSet pathTracingResTex = VK_NULL_HANDLE;

    /**
     * offline rendering target image
     */
    std::vector<VkImageView> *offscreenImageViews;
    VkSampler offscreenSampler;

    /**
     * uv rendering target resources
     */
    std::vector<VkImageView> *uvImageViews;
    VkSampler uvSampler;

    /**
     * bezier rending target resources
     */
    std::vector<VkImageView> *bezierImageViews;
    VkSampler bezierSampler;
    VklOffscreenRenderer *bezierRender;
    glm::vec2 bezier_shift{0.0f, 0.0f};

    /**
     * bezier editor curve rendering model
     */
    std::vector<std::unique_ptr<BezierCurve2D>> bezier_editor_curves;

    float bezier_zoom_in = 0.99f;

    GeometryVariableManager geometryVariableManager;

    std::unique_ptr<PathTracingComputeModel> pathTracingComputeModel_;
    std::unique_ptr<PathTracingComputeSystem> pathTracingComputeSystem_;

    uint32_t frameIndex;

    explicit UIManager(VklDevice &device, VklScene &scene);

    ~UIManager() {
        delete renderResultTexture;
    }

    void renderImgui();

    void resetBVH();

    void resetPathTracingCompute();

    void pickObject(float mouse_x_pos, float mouse_y_pos, float width, float height);
};