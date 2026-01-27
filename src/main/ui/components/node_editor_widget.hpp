#pragma once

#include "component.hpp"
#include "controller/controller.hpp"

#include "imgui_node_editor.h"

class NodeEditorWidget : public UIComponent {
  public:
    NodeEditorWidget(SceneTree::VklSceneTree &sceneTree, Controller &controller,
                     GraphicsLab::GraphicsLabInternalContext &appContext)
        : UIComponent(sceneTree), appContext_(appContext) {
        namespace ed = ax::NodeEditor;
        ed::Config config;
        config.SettingsFile = "node_editor_config.json";
        m_Context = ed::CreateEditor(&config);
    }

    void render() final {
        namespace ed = ax::NodeEditor;

        ImGui::Begin("Node Editor Demo");

        ed::SetCurrentEditor(m_Context);
        ed::Begin("My Editor", ImVec2(0.0, 0.0f));
        int uniqueId = 1;
        // Start drawing nodes.
        ed::BeginNode(uniqueId++);
        ImGui::Text("Node A");
        ed::BeginPin(uniqueId++, ed::PinKind::Input);
        ImGui::Text("-> In");
        ed::EndPin();
        ImGui::SameLine();
        ed::BeginPin(uniqueId++, ed::PinKind::Output);
        ImGui::Text("Out ->");
        ed::EndPin();
        ed::EndNode();

        for (auto &pass : appContext_.renderGraph->get_all_passes_generator()) {
            ed::BeginNode(uniqueId++);

            // Set a custom color for the node header
            // ed::Color(ed::GetNodeId(), ImColor(0.2f, 0.5f, 0.8f));

            // Title with larger font and bold style
            ImGui::TextColored(ImColor(1.0f, 1.0f, 1.0f), "%s", pass->get_name().data());

            // Create a horizontal layout for pins
            ImGui::BeginGroup();
            for (auto field : pass->render_pass_reflect()) {
                if (field.get_visibility() ==
                    GraphicsLab::RenderGraph::RenderPassReflection::Field::Visibility::Input) {
                    ed::BeginPin(uniqueId++, ed::PinKind::Input);
                    ImGui::Text("%s", field.get_name().c_str());
                    ed::EndPin();
                } else if (field.get_visibility() ==
                           GraphicsLab::RenderGraph::RenderPassReflection::Field::Visibility::Output) {
                    ed::BeginPin(uniqueId++, ed::PinKind::Output);
                    ImGui::Text("%s", field.get_name().c_str());
                    ed::EndPin();
                }
                ImGui::SameLine(0, 10); // Add some spacing between pins
            }
            ImGui::EndGroup();

            ed::EndNode();
        }

        ed::Link(0, 3, 5);

        ed::End();
        ed::SetCurrentEditor(nullptr);

        ImGui::End();
    }

  private:
    GraphicsLab::GraphicsLabInternalContext &appContext_;
    ax::NodeEditor::EditorContext *m_Context = nullptr;
};

META_REGISTER_TYPE(MainComponentRegisterTag, NodeEditorWidget)