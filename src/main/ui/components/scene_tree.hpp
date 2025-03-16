#pragma once

#include "component.hpp"

#include "../../project/file_system.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "language/reflection/reflectors.hpp"
#include "project/file_system.hpp"
#include "project/project_manager.hpp"

#include "geometry/constructor/rectangle3d.hpp"

#include "nlohmann/json.hpp"
using json = nlohmann::json;

class SceneTreeComponent : public UIComponent {
  public:
    SceneTreeComponent(GraphicsLab::GraphicsLabInternalContext &context) : UIComponent(context) {
    }

    void render() final {
        ImGui::Begin("Scene Tree");
        {
            if (ImGui::Button("Load Obj File")) {
                std::string file = FileSystem::chooseFile();
                spdlog::info("Choose Obj File: {}", file);
                auto result = std::async(std::launch::async, [&]() {
                    std::scoped_lock scoped_lock(context_.sceneTree->sceneTreeMutex);
                    context_.sceneTree->importFromPath(file);
                    context_.sceneTree->addCameraNode("Camera 1", Camera({0, 0, 10}, {0, 1, 0}));
                    spdlog::info("Load Obj File: {}", file);
                });
                vkDeviceWaitIdle(context_.sceneTree->device_.device());
            }

            RenderTreeNode(context_.sceneTree->root.get());
        }
        ImGui::End();
    }

  private:
    void showReflectable(Reflectable *reflectableObject) {
        auto rfl = reflectableObject->reflect();
        for (auto &[key, value] : reflectableObject->reflect()) {
            if (value.get()) {
                if (value.type() == typeid(std::string)) {
                    auto str_ptr = reinterpret_cast<std::string *>(value.get());
                    ImGui::Text(std::format("{}: {}", key, *str_ptr).c_str());
                }
                if (value.type() == typeid(glm::vec3)) {
                    auto &vec = *reinterpret_cast<glm::vec3 *>(value.get());
                    ImGui::Text(std::format("{}: {} {} {}", key, vec.x, vec.y, vec.z).c_str());
                }
                if (value.type() == typeid(bool)) {
                    auto &boolval = *reinterpret_cast<bool *>(value.get());
                    ImGui::Checkbox(key.c_str(), &boolval);
                } else if (value.type() == typeid(glm::quat)) {
                    auto &quat = *reinterpret_cast<glm::quat *>(value.get());
                    ImGui::Text(std::format("{}: {} {} {} {}", key, quat.w, quat.x, quat.y, quat.z).c_str());
                }
                if (value.isReflectable) {
                    if (ImGui::TreeNode(key.c_str())) {
                        showReflectable(static_cast<Reflectable *>(value.get()));
                        ImGui::TreePop();
                    }
                }
            } else if (value.call_func != nullptr) {
                if (ImGui::Button(std::format("{}", key).c_str())) {
                    value.call();
                }
            } else if (value.call_func_with_param != nullptr) {
                if (ImGui::Button(std::format("{}", key).c_str())) {
                    for (int i = 0; auto &meta : value.function_with_pack->meta.arguments) {
                        spdlog::info("Param {}: {}", i++, meta.name);
                    }
                }
            }
        }
    }

    void RenderTreeNode(SceneTree::TreeNode *node) {
        if (!node)
            return;

        // Determine the label based on the node type
        std::string label;
        switch (node->type()) {
        case SceneTree::NodeType::InternalNode:
            label = "Internal Node";
            break;
        case SceneTree::NodeType::GeometryNode:
            label = "Geometry Node";
            break;
        case SceneTree::NodeType::LightSourceNode:
            label = "Light Node";
            break;
        case SceneTree::NodeType::CameraNode:
            label = "Camera Node";
            break;
        default:
            label = "Unknown Node";
            break;
        }

        // Create a tree node in ImGui
        if (ImGui::TreeNode(std::format("{}: {}", label, node->name).c_str())) {
            // If this is a GeometryNode, display additional details (like material info)
            if (node->type() == SceneTree::NodeType::GeometryNode) {

                if (auto mesh3d_node = dynamic_cast<SceneTree::GeometryNode<Mesh3D> *>(node)) {
                    ImGui::Text("Mesh3D: %s", mesh3d_node->name.c_str());
                }
            }

            if (node->type() == SceneTree::NodeType::CameraNode) {
                auto camera_node = reinterpret_cast<SceneTree::CameraNode *>(node);
                auto camera = camera_node->camera;
                ImGui::Text(std::format("Position {:.3f} {:.3f} {:.3f}", camera.position.x, camera.position.y,
                                        camera.position.z)
                                .c_str());
                ImGui::Text(std::format("Up {:.3f} {:.3f} {:.3f}", camera.camera_up_axis.x, camera.camera_up_axis.y,
                                        camera.camera_up_axis.z)
                                .c_str());
                ImGui::Text(std::format("Front {:.3f} {:.3f} {:.3f}", camera.camera_front.x, camera.camera_front.y,
                                        camera.camera_front.z)
                                .c_str());

                if (camera_node != context_.sceneTree->active_camera) {
                    if (ImGui::Button("Activate")) {
                        context_.sceneTree->active_camera = camera_node;
                    }
                }
            }

            showReflectable(node);

            // Recursively render child nodes
            for (const auto &child : node->children) {
                RenderTreeNode(child.get());
            }

            // Close the tree node
            ImGui::TreePop();
        }
    }
};

META_REGISTER_TYPE(MainComponentRegisterTag, SceneTreeComponent)