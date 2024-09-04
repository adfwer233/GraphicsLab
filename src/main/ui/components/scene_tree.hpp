#pragma once

#include "component.hpp"

class SceneTreeComponent: public UIComponent {
  public:
    SceneTreeComponent(SceneTree::VklSceneTree &sceneTree): UIComponent(sceneTree) {

    }

    void render() final {
        ImGui::Begin("Scene Tree");
        {
            RenderTreeNode(sceneTree_.root.get());
        }
        ImGui::End();
    }

  private:

    void RenderTreeNode(SceneTree::TreeNode* node) {
        if (!node) return;

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
                if (auto mesh3d_node = dynamic_cast<SceneTree::GeometryNode<Mesh3D>*>(node)) {
                    ImGui::Text("Mesh3D: %s", mesh3d_node->name.c_str());
                }
                // Add more material properties if needed
            }

            // Recursively render child nodes
            for (const auto& child : node->children) {
                RenderTreeNode(child.get());
            }

            // Close the tree node
            ImGui::TreePop();
        }
    }
};

META_REGISTER_TYPE(MainComponentRegisterTag, SceneTreeComponent)