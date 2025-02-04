#pragma once

#include "component.hpp"
#include "controller/controller.hpp"

#include "geometry/parametric/sphere.hpp"
#include "geometry/parametric/tessellator.hpp"

class ConstructorWidget : public UIComponent {
public:
    ConstructorWidget(GraphicsLab::GraphicsLabInternalContext &context, Controller &controller) : UIComponent(context) {
    }

    void render() final {
        ImGui::Begin("Modelling");

        ImGui::Text("Constructors");

        if (ImGui::Button("Add Sphere")) {
            show_add_sphere_dialog = true;
        }

        ImGui::End();

        if (show_add_sphere_dialog) {
            render_add_sphere_dialog();
        }
    }

private:
    void render_add_sphere_dialog() {
        ImVec2 center = ImGui::GetMainViewport()->GetCenter();
        ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));

        ImGui::Begin("Input Dialog", &show_add_sphere_dialog,
                     ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse);
        ImGui::Text("Enter your arguments:");

        thread_local glm::vec3 sphere_center(0.0);
        thread_local double sphere_radius = 0.0;

        thread_local char name[64] = "default";

        ImGui::InputText("##InputText", name, sizeof(name));
        ImGui::InputFloat3("Center", glm::value_ptr(sphere_center));
        ImGui::InputDouble("Radius", &sphere_radius);


        if (ImGui::Button("OK")) {
            show_add_sphere_dialog = false;

            auto sphere = GraphicsLab::Geometry::Sphere(sphere_center, sphere_radius);
            GraphicsLab::Geometry::Tessellator::tessellate(sphere);
            // auto rect = RectangleConstructor::create(base_point, direction1, direction2, 50, 50);
            context_.sceneTree->addGeometryNode<GraphicsLab::Geometry::Sphere>(std::move(sphere), name);
        }

        ImGui::SameLine();

        if (ImGui::Button("Cancel")) {
            show_add_sphere_dialog = false;
        }
        ImGui::End();
    }

    bool show_add_sphere_dialog = false;
};

META_REGISTER_TYPE(MainComponentRegisterTag, ConstructorWidget)