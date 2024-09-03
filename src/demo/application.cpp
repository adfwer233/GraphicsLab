#include "application.hpp"
#include "vkl/core/vkl_descriptor.hpp"
#include "vkl/core/vkl_image.hpp"
#include "vkl/scene/vkl_geometry_model.hpp"
#include "vkl/scene/vkl_scene.hpp"
#include "vkl/utils/vkl_box.hpp"
#include "vkl/utils/vkl_box_model.hpp"
#include "vkl/utils/vkl_curve_model.hpp"

#include "demo/utils/controller.hpp"
#include "vkl/system/render_system/line_render_system.hpp"
#include "vkl/system/render_system/normal_render_system.hpp"
#include "vkl/system/render_system/param_line_render_system.hpp"
#include "vkl/system/render_system/point_cloud_2d_render_system.hpp"
#include "vkl/system/render_system/simple_render_system.hpp"
#include "vkl/system/render_system/simple_wireframe_render_system.hpp"

#include "vkl/system/compute_system/base_compute_system.hpp"
#include "vkl/system/compute_system/path_tracing_compute_system.hpp"

#include "vkl/core/vkl_offscreen_renderer.hpp"

#include "vkl/render_graph/render_graph.hpp"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_vulkan.h"

#include <functional>
#include <random>

#include "ui/ui_manager.hpp"

#include "nlohmann/json.hpp"
using json = nlohmann::json;

static ImGui_ImplVulkanH_Window g_MainWindowData;

Application::~Application() {
}

void Application::run() {

    VklScene scene(device_, {0, 0, 10}, {0, 1, 0});

    auto boxModel = VklBoxModel3D(device_, getStandardBox3D());

    auto controller = KeyboardCameraController::instance();

    controller->setCamera(scene.camera);

    GLFWwindow *window = window_.getGLFWwindow();

    glfwSetCursorPosCallback(window, KeyboardCameraController::mouse_callback);
    glfwSetScrollCallback(window, KeyboardCameraController::scroll_callback);
    glfwSetMouseButtonCallback(window, KeyboardCameraController::mouse_button_callback);

    /** render system */
    SimpleRenderSystem renderSystem(
        device_, offscreenRenderer_.getSwapChainRenderPass(),
        {{std::format("{}/simple_shader.vert.spv", SHADER_DIR), VK_SHADER_STAGE_VERTEX_BIT},
         {std::format("{}/simple_shader.frag.spv", SHADER_DIR), VK_SHADER_STAGE_FRAGMENT_BIT}});

    SimpleRenderSystem rawRenderSystem(
        device_, offscreenRenderer_.getSwapChainRenderPass(),
        {{std::format("{}/simple_shader.vert.spv", SHADER_DIR), VK_SHADER_STAGE_VERTEX_BIT},
         {std::format("{}/point_light_shader.frag.spv", SHADER_DIR), VK_SHADER_STAGE_FRAGMENT_BIT}});

    SimpleWireFrameRenderSystem wireFrameRenderSystem(
        device_, offscreenRenderer_.getSwapChainRenderPass(),
        {{std::format("{}/simple_shader.vert.spv", SHADER_DIR), VK_SHADER_STAGE_VERTEX_BIT},
         {std::format("{}/point_light_shader.frag.spv", SHADER_DIR), VK_SHADER_STAGE_FRAGMENT_BIT}});

    LineRenderSystem lineRenderSystem(
        device_, offscreenRenderer_.getSwapChainRenderPass(),
        {{std::format("{}/line_shader.vert.spv", SHADER_DIR), VK_SHADER_STAGE_VERTEX_BIT},
         {std::format("{}/line_shader.frag.spv", SHADER_DIR), VK_SHADER_STAGE_FRAGMENT_BIT}});

    LineRenderSystem curveMeshRenderSystem(
        device_, offscreenRenderer_.getSwapChainRenderPass(),
        {{std::format("{}/curve_mesh_shader.vert.spv", SHADER_DIR), VK_SHADER_STAGE_VERTEX_BIT},
         {std::format("{}/line_shader.frag.spv", SHADER_DIR), VK_SHADER_STAGE_FRAGMENT_BIT}});

    ParamLineRenderSystem paramCurveRenderSystem(
        device_, uvRender_.getSwapChainRenderPass(),
        {{std::format("{}/param_curve_shader.vert.spv", SHADER_DIR), VK_SHADER_STAGE_VERTEX_BIT},
         {std::format("{}/param_curve_shader.frag.spv", SHADER_DIR), VK_SHADER_STAGE_FRAGMENT_BIT}});

    SimpleRenderSystem colorRenderSystem(
        device_, offscreenRenderer_.getSwapChainRenderPass(),
        {{std::format("{}/simple_shader.vert.spv", SHADER_DIR), VK_SHADER_STAGE_VERTEX_BIT},
         {std::format("{}/simple_color_shader.frag.spv", SHADER_DIR), VK_SHADER_STAGE_FRAGMENT_BIT}});

    NormalRenderSystem normalRenderSystem(
        device_, offscreenRenderer_.getSwapChainRenderPass(),
        {{std::format("{}/normal_shader.vert.spv", SHADER_DIR), VK_SHADER_STAGE_VERTEX_BIT},
         {std::format("{}/line_shader.frag.spv", SHADER_DIR), VK_SHADER_STAGE_FRAGMENT_BIT},
         {std::format("{}/normal_generation.geom.spv", SHADER_DIR), VK_SHADER_STAGE_GEOMETRY_BIT}});

    PointCloud2DRenderSystem pointCloud2DRenderSystem(
        device_, offscreenRenderer_.getSwapChainRenderPass(),
        {{std::format("{}/point_cloud_2d_shader.vert.spv", SHADER_DIR), VK_SHADER_STAGE_VERTEX_BIT},
         {std::format("{}/point_cloud_2d_shader.frag.spv", SHADER_DIR), VK_SHADER_STAGE_FRAGMENT_BIT}});

    float deltaTime = 0, lastFrame = 0;

    VkDescriptorPoolSize pool_sizes[] = {{VK_DESCRIPTOR_TYPE_SAMPLER, 1000},
                                         {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1000},
                                         {VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1000},
                                         {VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1000},
                                         {VK_DESCRIPTOR_TYPE_UNIFORM_TEXEL_BUFFER, 1000},
                                         {VK_DESCRIPTOR_TYPE_STORAGE_TEXEL_BUFFER, 1000},
                                         {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1000},
                                         {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1000},
                                         {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC, 1000},
                                         {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER_DYNAMIC, 1000},
                                         {VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT, 1000}};

    VkDescriptorPoolCreateInfo pool_info = {};
    pool_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    pool_info.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
    pool_info.maxSets = 1000;
    pool_info.poolSizeCount = (uint32_t)std::size(pool_sizes);
    pool_info.pPoolSizes = pool_sizes;

    VkDescriptorPool imguiPool;
    vkCreateDescriptorPool(device_.device(), &pool_info, nullptr, &imguiPool);

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard; // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;  // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    // ImGui::StyleColorsLight();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForVulkan(window, true);
    ImGui_ImplVulkan_InitInfo init_info = {};
    device_.fillImGuiInitInfo(init_info);
    init_info.DescriptorPool = imguiPool;
    init_info.RenderPass = renderer_.getSwapChainRenderPass();
    init_info.Subpass = 0;
    init_info.MinImageCount = 2;
    init_info.ImageCount = 2;
    init_info.MSAASamples = VK_SAMPLE_COUNT_1_BIT;

    ImGui_ImplVulkan_Init(&init_info);

    UIManager uiManager(device_, scene);

    VklTexture *renderRes = new VklTexture(device_, 1024, 1024, 4);

    device_.transitionImageLayout(renderRes->image_, VK_FORMAT_R8G8B8A8_SRGB, VK_IMAGE_LAYOUT_UNDEFINED,
                                  VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

    VklCurveModel2D::BuilderFromImmediateData parameterSpaceBoundaryBuilder;
    parameterSpaceBoundaryBuilder.vertices = {
        Vertex2D{{0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f}},
        Vertex2D{{0.0f, 1.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f}},
        Vertex2D{{1.0f, 1.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f}},
        Vertex2D{{1.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f}},
        Vertex2D{{0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f}},
        // Vertex2D{{1.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f}},
        // Vertex2D{{1.0f, 1.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f}},
        // Vertex2D{{2.0f, 1.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f}},
        // Vertex2D{{2.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f}},
        // Vertex2D{{1.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f}},
        // Vertex2D{{2.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f}},
        // Vertex2D{{2.0f, 1.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f}},
        // Vertex2D{{3.0f, 1.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f}},
        // Vertex2D{{3.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f}},
        // Vertex2D{{2.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f}},
    };

    VklCurveModel2D parameterSpaceBoundary(device_, parameterSpaceBoundaryBuilder);

    uiManager.renderResultTexture = renderRes;
    uiManager.offscreenImageViews = offscreenRenderer_.getImageView();
    uiManager.offscreenSampler = offscreenRenderer_.imageSampler;
    uiManager.uvImageViews = uvRender_.getImageView();
    uiManager.uvSampler = uvRender_.imageSampler;
    uiManager.bezierImageViews = bezierRenderer_.getImageView();
    uiManager.bezierSampler = bezierRenderer_.imageSampler;

    uiManager.bezierRender = &bezierRenderer_;

    controller->set_scene(scene);
    controller->setUIManager(uiManager);

    controller->actionCallBack = [&]() {
        if (uiManager.pathTracingComputeSystem_ != nullptr) {
            uiManager.pathTracingComputeSystem_->computeModel_.ubo.currentSample = 0;
        }
    };

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> distrib(0, 1.0);

    while (not window_.shouldClose()) {
        glfwPollEvents();

        ImGui_ImplVulkan_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        float currentFrame = static_cast<float>(glfwGetTime());
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        uiManager.deltaTime = deltaTime;

        controller->processInput(window_.getGLFWwindow(), deltaTime);

        int frameIndex = renderer_.getFrameIndex();
        uiManager.frameIndex = frameIndex;

        uiManager.renderImgui();

        if (uiManager.renderMode == PathTracing) {

            auto commandBuffer = renderer_.beginFrame();

            auto targetTexture = uiManager.pathTracingComputeModel_->getTargetTexture();
            auto accumulationTexture = uiManager.pathTracingComputeModel_->getAccumulationTexture();

            uiManager.pathTracingComputeSystem_->computeModel_.ubo.cameraZoom = scene.camera.zoom;
            uiManager.pathTracingComputeSystem_->computeModel_.ubo.cameraPosition = scene.camera.position;
            uiManager.pathTracingComputeSystem_->computeModel_.ubo.cameraUp = scene.camera.camera_up_axis;
            uiManager.pathTracingComputeSystem_->computeModel_.ubo.cameraFront = scene.camera.camera_front;
            uiManager.pathTracingComputeSystem_->computeModel_.ubo.currentSample += 1;
            uiManager.pathTracingComputeSystem_->computeModel_.ubo.rand1 = scene.camera.ratio;
            uiManager.pathTracingComputeSystem_->computeModel_.ubo.rand2 = distrib(gen);
            uiManager.pathTracingComputeSystem_->updateUniformBuffer(frameIndex);
            uiManager.pathTracingComputeSystem_->recordCommandBuffer(commandBuffer, targetTexture, accumulationTexture,
                                                                     renderRes->image_, frameIndex);

            renderer_.beginSwapChainRenderPass(commandBuffer);

            /* ImGui Rendering */
            ImGui::Render();
            ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), commandBuffer);

            renderer_.endSwapChainRenderPass(commandBuffer);

            if (vkEndCommandBuffer(commandBuffer) != VK_SUCCESS) {
                throw std::runtime_error("failed to record command buffer!");
            }

            std::vector<VkCommandBuffer> commandBuffers{commandBuffer};

            auto result = renderer_.swapChain_->submitCommandBuffers(commandBuffers, &renderer_.currentImageIndex);

            renderer_.endFrame();

        } else {

            auto commandBuffer = renderer_.beginFrame();
            renderer_.beginSwapChainRenderPass(commandBuffer);

            auto offscreenCommandBuffer = offscreenRenderer_.beginFrame();
            offscreenRenderer_.beginSwapChainRenderPass(offscreenCommandBuffer);

            auto uvCommandBuffer = uvRender_.beginFrame();
            uvRender_.beginSwapChainRenderPass(uvCommandBuffer);

            auto bezierCommandBuffer = bezierRenderer_.beginFrame();
            bezierRenderer_.beginSwapChainRenderPass(bezierCommandBuffer);

            GlobalUbo ubo{};

            ubo.view = scene.camera.get_view_transformation();
            ubo.proj = scene.camera.get_proj_transformation();
            ubo.model = glm::mat4(1.0f);
            ubo.pointLight = scene.pointLight;
            ubo.cameraPos = scene.camera.position;

            if (uiManager.shadingMode == SolidShading) {
                ubo.pointLight.position = glm::vec4(ubo.cameraPos, 1.0f);
            }

            // param line push constant

            ParamLineRenderSystemPushConstantData paramLineRenderSystemPushConstantData{
                .zoom = uiManager.bezier_zoom_in,
                .shift_x = uiManager.bezier_shift.x,
                .shift_y = uiManager.bezier_shift.y};
            ParamLineRenderSystemPushConstantList paramLineRenderSystemPushConstantList;
            paramLineRenderSystemPushConstantList.data[0] = paramLineRenderSystemPushConstantData;

            // render meshes
            for (auto &object_item : scene.objects) {
                for (auto model : object_item->models) {
                    ubo.model = object_item->getModelTransformation();

                    auto updateUniformBuffer = [&](VklDescriptorSetLayoutKey &key) {
                        if (model->uniformBuffers.contains(key)) {
                            model->uniformBuffers[key][frameIndex]->writeToBuffer(&ubo);
                            model->uniformBuffers[key][frameIndex]->flush();
                        }
                    };

                    FrameInfo<VklModel> modelFrameInfo{frameIndex, currentFrame, offscreenCommandBuffer, scene.camera,
                                                       *model};

                    if (uiManager.renderMode == Raw) {
                        if (uiManager.shadingMode == PointLightShading or uiManager.shadingMode == SolidShading) {
                            updateUniformBuffer(rawRenderSystem.descriptorSetLayout->descriptorSetLayoutKey);
                            rawRenderSystem.renderObject(modelFrameInfo);
                        } else if (uiManager.shadingMode == PureColor) {
                            updateUniformBuffer(colorRenderSystem.descriptorSetLayout->descriptorSetLayoutKey);
                            colorRenderSystem.renderObject(modelFrameInfo);
                        }
                    } else if (uiManager.renderMode == WireFrame) {
                        // modelFrameInfo.commandBuffer = uvCommandBuffer;
                        updateUniformBuffer(wireFrameRenderSystem.descriptorSetLayout->descriptorSetLayoutKey);
                        wireFrameRenderSystem.renderObject(modelFrameInfo);
                    } else if (uiManager.renderMode == WithTexture) {
                        if (model->textures_.empty()) {
                            updateUniformBuffer(rawRenderSystem.descriptorSetLayout->descriptorSetLayoutKey);
                            rawRenderSystem.renderObject(modelFrameInfo);
                        } else {
                            updateUniformBuffer(renderSystem.descriptorSetLayout->descriptorSetLayoutKey);
                            renderSystem.renderObject(modelFrameInfo);
                        }
                    }

                    if (uiManager.showNormal) {
                        NormalRenderSystemPushConstantData normalRenderSystemPushConstantData{};
                        normalRenderSystemPushConstantData.normalStrength = uiManager.normalStrength;
                        normalRenderSystemPushConstantData.normalColor = uiManager.normalColor;
                        NormalRenderSystemPushConstantDataList list;
                        list.data[0] = normalRenderSystemPushConstantData;
                        normalRenderSystem.renderObject(modelFrameInfo, list);
                    }

                    // visiting underlying geometry
                    std::visit(
                        [&](auto &&arg) {
                            using T = std::decay_t<decltype(arg)>;
                            if constexpr (std::is_same_v<T, MeshGeometryType>) {

                            } else if (std::is_same_v<T, TensorProductBezierSurface>) {
                                TensorProductBezierSurface &surf = arg;
                                auto modelBuffer = VklGeometryModelBuffer<TensorProductBezierSurface>::instance();
                                auto geometryModel = modelBuffer->getGeometryModel(device_, &surf);
                                for (auto &boundary3d : geometryModel->boundary_3d) {
                                    if (not boundary3d->uniformBuffers.empty()) {
                                        boundary3d
                                            ->uniformBuffers[curveMeshRenderSystem.descriptorSetLayout
                                                                 ->descriptorSetLayoutKey][frameIndex]
                                            ->writeToBuffer(&ubo);
                                        boundary3d
                                            ->uniformBuffers[curveMeshRenderSystem.descriptorSetLayout
                                                                 ->descriptorSetLayoutKey][frameIndex]
                                            ->flush();
                                    }
                                    FrameInfo<VklCurveModel3D> frameInfo{
                                        frameIndex, currentFrame, offscreenCommandBuffer, scene.camera, *boundary3d};
                                    curveMeshRenderSystem.renderObject(frameInfo);
                                }

                                for (auto &boundary2d : geometryModel->boundary_2d) {
                                    FrameInfo<VklCurveModel2D> frameInfo{frameIndex, currentFrame, uvCommandBuffer,
                                                                         scene.camera, *boundary2d};
                                    paramCurveRenderSystem.renderObject(frameInfo,
                                                                        paramLineRenderSystemPushConstantList);
                                }
                            }
                        },
                        model->underlyingGeometry);
                }
            }

            // render surfaces
            // for (auto &surf : scene.surfaces) {
            //     auto meshModel = surf->getMeshModel(device_);
            //
            //     meshModel->uniformBuffers[frameIndex]->writeToBuffer(&ubo);
            //     meshModel->uniformBuffers[frameIndex]->flush();
            //
            //     FrameInfo<VklModel> modelFrameInfo{frameIndex,
            //                                        currentFrame,
            //                                        offscreenCommandBuffer,
            //                                        scene.camera,
            //                                        &meshModel->descriptorSets[frameIndex],
            //                                        *meshModel};
            //
            //     if (uiManager.renderMode == Raw) {
            //         if (uiManager.shadingMode == PointLightShading or uiManager.shadingMode == SolidShading) {
            //             rawRenderSystem.renderObject(modelFrameInfo);
            //         } else if (uiManager.shadingMode == PureColor) {
            //             colorRenderSystem.renderObject(modelFrameInfo);
            //         }
            //     } else if (uiManager.renderMode == WireFrame) {
            //         // modelFrameInfo.commandBuffer = uvCommandBuffer;
            //         wireFrameRenderSystem.renderObject(modelFrameInfo);
            //     } else if (uiManager.renderMode == WithTexture) {
            //         if (meshModel->textures_.empty())
            //             rawRenderSystem.renderObject(modelFrameInfo);
            //         else
            //             renderSystem.renderObject(modelFrameInfo);
            //     }
            //
            //     if (uiManager.showNormal) {
            //         NormalRenderSystemPushConstantData normalRenderSystemPushConstantData{};
            //         normalRenderSystemPushConstantData.normalStrength = uiManager.normalStrength;
            //         normalRenderSystemPushConstantData.normalColor = uiManager.normalColor;
            //         NormalRenderSystemPushConstantDataList list;
            //         list.data[0] = normalRenderSystemPushConstantData;
            //         normalRenderSystem.renderObject(modelFrameInfo, list);
            //     }
            //
            //     auto boundary_meshes = surf->getBoundaryMeshModels(device_);
            //
            //     for (auto boundary : boundary_meshes) {
            //         boundary->uniformBuffers[frameIndex]->writeToBuffer(&ubo);
            //         boundary->uniformBuffers[frameIndex]->flush();
            //
            //         FrameInfo<TensorProductBezierSurface::boundary_render_type> boundaryModelFrameInfo{
            //             frameIndex,
            //             currentFrame,
            //             offscreenCommandBuffer,
            //             scene.camera,
            //             &boundary->descriptorSets[frameIndex],
            //             *boundary};
            //         curveMeshRenderSystem.renderObject(boundaryModelFrameInfo);
            //     }
            // }

            // rendering bezier curves
            if (controller->currentWidgets == BezierEditing) {

                FrameInfo<VklCurveModel2D> parameterSpaceBoundaryFrameInfo{
                    frameIndex, currentFrame, bezierCommandBuffer, scene.camera, parameterSpaceBoundary};

                // paramCurveRenderSystem.renderObject(parameterSpaceBoundaryFrameInfo,
                // paramLineRenderSystemPushConstantList);

                for (auto &bezier_editor_curve : uiManager.bezier_editor_curves) {
                    if (bezier_editor_curve != nullptr) {
                        auto modelBuffer = VklGeometryModelBuffer<BezierCurve2D>::instance();
                        auto curveMesh = modelBuffer->getGeometryModel(device_, bezier_editor_curve.get());

                        auto &model = curveMesh->controlPointsMesh;

                        PointCloud2DRenderSystemPushConstantData pointCloud2DRenderSystemPushConstantData{
                            .zoom = uiManager.bezier_zoom_in,
                            .shift_x = uiManager.bezier_shift.x,
                            .shift_y = uiManager.bezier_shift.y};
                        PointCloud2DRenderSystemPushConstantList pointCloud2DRenderSystemPushConstantList;
                        pointCloud2DRenderSystemPushConstantList.data[0] = pointCloud2DRenderSystemPushConstantData;

                        FrameInfo<VklPointCloud2D> modelFrameInfo{frameIndex, currentFrame, bezierCommandBuffer,
                                                                  scene.camera, *model};

                        pointCloud2DRenderSystem.renderObject(modelFrameInfo, pointCloud2DRenderSystemPushConstantList);

                        if (curveMesh->curveMesh != nullptr) {

                            FrameInfo<VklCurveModel2D> curveModelFrameInfo{
                                frameIndex, currentFrame, bezierCommandBuffer, scene.camera, *curveMesh->curveMesh};

                            paramCurveRenderSystem.renderObject(curveModelFrameInfo,
                                                                paramLineRenderSystemPushConstantList);
                        }

                        if (curveMesh->derivativeBoundMesh != nullptr) {
                            FrameInfo<VklCurveModel2D> curveModelFrameInfo{frameIndex, currentFrame,
                                                                           bezierCommandBuffer, scene.camera,
                                                                           *curveMesh->derivativeBoundMesh};

                            paramCurveRenderSystem.renderObject(curveModelFrameInfo,
                                                                paramLineRenderSystemPushConstantList);
                        }

                        if (curveMesh->extremePointMesh != nullptr) {
                            FrameInfo<VklPointCloud2D> curveModelFrameInfo{frameIndex, currentFrame,
                                                                           bezierCommandBuffer, scene.camera,
                                                                           *curveMesh->extremePointMesh};

                            // pointCloud2DRenderSystem.renderObject(curveModelFrameInfo,
                            // pointCloud2DRenderSystemPushConstantList);
                        }
                    }
                }
            }

            if (uiManager.picking_result.has_value()) {
                auto &object_picked = scene.objects[uiManager.picking_result->object_index];
                auto &model_picked = object_picked->models[uiManager.picking_result->model_index];
                auto box = model_picked->box;
                box.apply_transform(object_picked->getModelTransformation());
                auto box_trans = box.get_box_transformation();

                ubo.model = box_trans;
                if (not boxModel.uniformBuffers.empty()) {
                    boxModel.uniformBuffers[lineRenderSystem.descriptorSetLayout->descriptorSetLayoutKey][frameIndex]
                        ->writeToBuffer(&ubo);
                    boxModel.uniformBuffers[lineRenderSystem.descriptorSetLayout->descriptorSetLayoutKey][frameIndex]
                        ->flush();
                }
                FrameInfo<VklBoxModel3D> boxFrameInfo{frameIndex, currentFrame, offscreenCommandBuffer, scene.camera,
                                                      boxModel};
                lineRenderSystem.renderObject(boxFrameInfo);
            }

            bezierRenderer_.endSwapChainRenderPass(bezierCommandBuffer);
            bezierRenderer_.endFrame();

            uvRender_.endSwapChainRenderPass(uvCommandBuffer);
            uvRender_.endFrame();

            offscreenRenderer_.endSwapChainRenderPass(offscreenCommandBuffer);
            offscreenRenderer_.endFrame();

            /* ImGui Rendering */

            ImGui::Render();
            ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), commandBuffer);
            renderer_.endSwapChainRenderPass(commandBuffer);

            if (vkEndCommandBuffer(commandBuffer) != VK_SUCCESS) {
                throw std::runtime_error("failed to record command buffer!");
            }

            std::vector<VkCommandBuffer> commandBuffers{commandBuffer, offscreenCommandBuffer, uvCommandBuffer,
                                                        bezierCommandBuffer};

            auto result = renderer_.swapChain_->submitCommandBuffers(commandBuffers, &renderer_.currentImageIndex);

            if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR || window_.wasWindowResized()) {
                renderer_.windowUpdate();
            }

            renderer_.endFrame();
        }
    }

    vkDeviceWaitIdle(device_.device());

    ImGui_ImplVulkan_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    /** destroy imgui descriptor pool*/
    vkDestroyDescriptorPool(device_.device(), imguiPool, nullptr);

    delete VklGeometryModelBuffer<TensorProductBezierSurface>::instance();

    delete VklGeometryModelBuffer<BezierCurve2D>::instance();
}
