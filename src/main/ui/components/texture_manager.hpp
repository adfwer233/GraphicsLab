#pragma once

#include "component.hpp"
#include "controller/ui_states.hpp"
#include "project/file_system.hpp"
#include "ui/render_resources.hpp"

class TextureManager : public UIComponent {
    UIState &uiState_;
    RenderResources &renderResources_;

  public:
    TextureManager(GraphicsLab::GraphicsLabInternalContext &context, UIState &uiState, RenderResources &renderResources)
        : UIComponent(context), uiState_(uiState), renderResources_(renderResources) {
    }

    void render() final {

        ImGui::Begin("Texture Manager");
        {
            if (ImGui::Button("Open Image")) {
                std::string path = FileSystem::chooseFile();
                context_.sceneTree->texture_manager.load_texture(path);
            }

            for (auto &[name, texture_meta] : context_.sceneTree->texture_manager.textures_metadata) {
                auto texture = context_.sceneTree->texture_manager.textures[texture_meta.texture_index].get();

                if (not renderResources_.textureImguiImages.contains(name)) {
                    renderResources_.textureImguiImages[name] = vkl::ImguiUtils::getImguiTextureFromVklTexture(texture);
                }

                ImGui::BeginGroup();
                ImGui::Image(reinterpret_cast<ImTextureID>(renderResources_.textureImguiImages[name].front()),
                             ImVec2(128, 128));
                ImGui::Text("%s", name.c_str());
                ImGui::EndGroup();

                ImGui::SameLine();
            }
        }

        ImGui::End();
    }
};

META_REGISTER_TYPE(MainComponentRegisterTag, TextureManager)