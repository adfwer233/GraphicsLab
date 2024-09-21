#pragma once

#include "component.hpp"

#include "graphics_lab/utils/imgui_sink.hpp"

class LoggerWidgetComponent : public UIComponent {
  public:
    LoggerWidgetComponent(SceneTree::VklSceneTree &sceneTree) : UIComponent(sceneTree) {
    }

    void render() final {
        // Get the singleton instance of LogManager
        auto &logManager = LogManager::getInstance();

        // Begin ImGui window
        ImGui::Begin("Log Window");

        // Get the available size of the parent window
        ImVec2 available_size = ImGui::GetContentRegionAvail();

        // Make the log window scrollable and fit the size of the parent window
        ImGui::BeginChild("Scrolling", available_size, true, ImGuiWindowFlags_HorizontalScrollbar);

        // Retrieve and display logs
        const auto &logs = logManager.get_logs();
        for (const auto &log : logs) {
            ImGui::TextWrapped("%s", log.c_str());
        }

        ImGui::EndChild();
        ImGui::End();
    }
};

META_REGISTER_TYPE(MainComponentRegisterTag, LoggerWidgetComponent)