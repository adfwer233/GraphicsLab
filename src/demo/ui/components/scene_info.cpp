#include "scene_info.hpp"

#include <format>

#include "../ui_manager.hpp"

SceneInfoUI::SceneInfoUI(VklScene &scene, UIManager &uiManager) : scene_(scene), uiManager_(uiManager) {
}

void SceneInfoUI::renderImgui() {
    ImGui::Begin("Scene Data");

    ImGui::SeparatorText("Rendering Mode");

    for (int i = 0; i < scene_.objects.size(); i++) {
        ImGui::SeparatorText(std::format("Objcet {}", i).c_str());
        auto &obj = scene_.objects[i];
        for (auto j = 0; j < obj->models.size(); j++) {
            auto model = obj->models[j];
            std::visit(
                [](auto &&arg) {
                    using T = std::decay_t<decltype(arg)>;
                    if constexpr (std::is_same_v<T, MeshGeometryType>) {
                        ImGui::Text("Mesh Geometry");
                    } else if (std::is_same_v<T, TensorProductBezierSurface>) {
                        ImGui::Text("Tensor Product Bezier Surface");
                    } else {
                        ImGui::Text("Unknown Geometry");
                    }
                },
                model->underlyingGeometry);
        }
    }

    ImGui::End();
}
