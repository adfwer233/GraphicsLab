#pragma once

#include "component.hpp"

#include "language/reflection/reflectors.hpp"

#include "project/file_system.hpp"
#include "project/project_manager.hpp"

#include "nlohmann/json.hpp"
using json = nlohmann::json;

class SceneTreeComponent : public UIComponent {
  public:
    SceneTreeComponent(SceneTree::VklSceneTree &sceneTree) : UIComponent(sceneTree) {
    }

    void render() final {
        ImGui::Begin("Scene Tree");
        { RenderTreeNode(sceneTree_.root.get()); }
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

                if (value.isReflectable) {
                    showReflectable(static_cast<Reflectable *>(value.get()));
                }
            } else {
                if (ImGui::Button(std::format("{}", key).c_str())) {
                    value.call();
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
                ImGui::Text(std::format("Front {:.3f} {:.3f} {:.3f}", camera.camera_up_axis.x, camera.camera_up_axis.y,
                                        camera.camera_up_axis.z)
                                .c_str());
                ImGui::Text(std::format("Up {:.3f} {:.3f} {:.3f}", camera.camera_front.x, camera.camera_front.y,
                                        camera.camera_front.z)
                                .c_str());

                if (camera_node != sceneTree_.active_camera) {
                    if (ImGui::Button("Activate")) {
                        sceneTree_.active_camera = camera_node;
                    }
                }
            }

            if (auto internal_node = dynamic_cast<SceneTree::InternalNode *>(node))
                showReflectable(internal_node);

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