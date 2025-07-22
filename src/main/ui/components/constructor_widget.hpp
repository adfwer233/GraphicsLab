#pragma once

#include "component.hpp"
#include "controller/controller.hpp"
#include "geometry/boundary_representation/constructors/constructors.hpp"
#include "geometry/boundary_representation/faceter/naive_faceter.hpp"

#include "utils/sampler.hpp"

#include "geometry/parametric/sphere.hpp"
#include "geometry/parametric/tessellator.hpp"
#include "geometry/parametric/torus.hpp"
#include "geometry/parametric_topology/brep_face.hpp"
#include "glm/gtc/type_ptr.hpp"

#include <geometry/constructor/explicit_surface_constructors.hpp>
#include <geometry/constructor/rectangle3d.hpp>
#include <geometry/constructor/tensor_product_bezier_example.hpp>
#include <geometry/parametric/explicit_surface.hpp>
#include <geometry/parametric/tensor_product_bezier.hpp>
#include <geometry/parametric_intersector/surface_surface_intersector.hpp>

namespace GraphicsLab::Geometry::BRep {
struct Body;
}
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

        if (ImGui::Button("Add Torus")) {
            show_add_torus_dialog = true;
        }

        if (ImGui::Button("Add Rectangle")) {
            show_add_rectangle_dialog = true;
        }

        if (ImGui::Button("Add bezier patch")) {
            auto surf = GraphicsLab::Geometry::TensorProductBezierExample2::create();
            GraphicsLab::Geometry::Tessellator::tessellate(surf);
            context_.sceneTree->addGeometryNode<GraphicsLab::Geometry::TensorProductBezier>(std::move(surf), "test");
        }

        if (ImGui::Button("Add Explicit")) {
            auto f = [](GraphicsLab::Geometry::autodiff_vec2 param) -> GraphicsLab::Geometry::autodiff_vec3 {
                GraphicsLab::Geometry::autodiff_vec3 result;

                double a = 2;
                double b = 1;

                auto v = (param.y() - 0.5) * 2;
                result.x() = a * cosh(v) * cos(2 * std::numbers::pi * param.x());
                result.y() = -b * sinh(v);
                result.z() = a * cosh(v) * sin(2 * std::numbers::pi * param.x());

                return result;
            };

            GraphicsLab::Geometry::ExplicitSurface surf(f);
            GraphicsLab::Geometry::Tessellator::tessellate(surf);
            context_.sceneTree->addGeometryNode<GraphicsLab::Geometry::ExplicitSurface>(std::move(surf),
                                                                                        "test explicit");
        }

        if (ImGui::Button("Add Random Point Cloud")) {
            PointCloud3D point_cloud;

            for (int i = 0; i < 100; i++) {
                point_cloud.vertices.emplace_back(GraphicsLab::Sampler::sampleUnitSphere<3>());
            }
            context_.sceneTree->addGeometryNode<PointCloud3D>(std::move(point_cloud), "point cloud");
        }

        if (ImGui::Button("Add intersection example")) {
            auto surf1 = GraphicsLab::Geometry::ExplicitSurfaceConstructor::createHyperboloid();
            GraphicsLab::Geometry::Torus torus({0.0, 0.0, 0.0}, 2.0, 0.5, {0.0, 1.0, 0.0}, {1.0, 0.0, 0.0});
            auto result = GraphicsLab::Geometry::SurfaceSurfaceIntersector::intersect_all(torus, surf1);

            PointCloud3D point_cloud;

            for (auto &trace : result.traces) {
                for (auto &p : trace) {
                    point_cloud.vertices.emplace_back(p.position);
                }
            }
            context_.sceneTree->addGeometryNode<PointCloud3D>(std::move(point_cloud), "point cloud");
        }

        ImGui::Text("BRep Constructors");

        if (ImGui::Button("Add Cube")) {
            using namespace GraphicsLab::Geometry;

            BRep::Body* cube = BRep::BodyConstructors::cube({0.0, 0.0, 0.0}, {1.0, 1.0, 1.0});
            auto faces = BRep::TopologyUtils::get_all_faces(cube);

            for (int i = 0; i < faces.size(); i++) {
                auto mesh = BRep::NaiveFaceter::naive_facet(faces[i], 3, 3);

                context_.sceneTree->addGeometryNode<Mesh3D>(std::move(mesh), std::format("Cube_face_{}", i));
            }
        }

        if (ImGui::Button("Add Explicit Surface")) {
            using namespace GraphicsLab::Geometry;

            auto f = [](autodiff_vec2 param) -> autodiff_vec3 {
                autodiff_vec3 result;

                double a = 2;
                double b = 1;

                auto v = (param.y() - 0.5) * 2;
                result.x() = a * cosh(v) *  cos(2 * std::numbers::pi * param.x());
                result.y() = -b * sinh(v);
                result.z() = a * cosh(v) * sin(2 * std::numbers::pi * param.x());

                return result;
            };
            BRep::Face* face = BRep::FaceConstructors::explicit_surface(f);

            auto mesh = BRep::NaiveFaceter::naive_facet(face, 3, 3);
            context_.sceneTree->addGeometryNode<Mesh3D>(std::move(mesh), std::format("Explicit_face"));

            auto edges = BRep::TopologyUtils::get_all_edges(face);
            for (int i = 0; i < edges.size(); i++) {
                auto curve_mesh = BRep::NaiveFaceter::naive_edge_facet(edges[i], 100);
                context_.sceneTree->addGeometryNode<CurveMesh3D>(std::move(curve_mesh), std::format("Explicit_edge_{}", i));
            }

            StraightLine3D line({-1, 0, -1}, {1, 0, 1});
            Tessellator::tessellate(line);

            CurveMesh3D curve_mesh = *line.mesh.get();
            context_.sceneTree->addGeometryNode<CurveMesh3D>(std::move(curve_mesh), std::format("Explicit_curve_intersect"));
        }

        ImGui::End();

        if (show_add_rectangle_dialog) {
            render_add_rectangle_dialog();
        }

        if (show_add_sphere_dialog) {
            render_add_sphere_dialog();
        }

        if (show_add_torus_dialog) {
            render_add_torus_dialog();
        }
    }

  private:
    void render_add_rectangle_dialog() {
        ImVec2 center = ImGui::GetMainViewport()->GetCenter();
        ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));

        ImGui::Begin("Input Dialog", &show_add_rectangle_dialog,
                     ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse);
        ImGui::Text("Enter your arguments:");

        thread_local glm::vec3 base_point(0.0f), direction1(0.0f), direction2(0.0f);
        thread_local char name[64] = "default";

        ImGui::InputText("##InputText", name, 64);
        ImGui::InputFloat3("base point", glm::value_ptr(base_point));
        ImGui::InputFloat3("direction1", glm::value_ptr(direction1));
        ImGui::InputFloat3("direction2", glm::value_ptr(direction2));

        if (ImGui::Button("OK")) {
            show_add_rectangle_dialog = false;

            auto rect = RectangleConstructor::create(base_point, direction1, direction2, 50, 50);
            context_.sceneTree->addGeometryNode<Mesh3D>(std::move(rect));

            spdlog::info("Add Rectangle {} Dialog OK", name);
            spdlog::info("base point: {} {} {}", base_point.x, base_point.y, base_point.z);
        }

        ImGui::SameLine();

        if (ImGui::Button("Cancel")) {
            show_add_rectangle_dialog = false;
        }

        ImGui::End();
    }

    void render_add_torus_dialog() {
        ImVec2 center = ImGui::GetMainViewport()->GetCenter();
        ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));

        ImGui::Begin("Input Dialog", &show_add_sphere_dialog,
                     ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse);
        ImGui::Text("Enter your arguments:");

        thread_local glm::vec3 torus_center(0.0), normal_direction(0, 1.0, 0.0), direction1(1, 0, 0);
        thread_local double major_radius = 2.0;
        thread_local double minor_radius = 1.0;

        thread_local char name[64] = "default";

        ImGui::InputText("##InputText", name, sizeof(name));
        ImGui::InputFloat3("Center", glm::value_ptr(torus_center));
        ImGui::InputFloat3("BaseNormal", glm::value_ptr(normal_direction));
        ImGui::InputFloat3("Direction", glm::value_ptr(direction1));
        ImGui::InputDouble("Major Radius", &major_radius);
        ImGui::InputDouble("Minor Radius", &minor_radius);

        if (ImGui::Button("OK")) {
            show_add_torus_dialog = false;

            auto torus =
                GraphicsLab::Geometry::Torus(torus_center, major_radius, minor_radius, normal_direction, direction1);
            GraphicsLab::Geometry::Tessellator::tessellate(torus);
            context_.sceneTree->addGeometryNode<GraphicsLab::Geometry::Torus>(std::move(torus), name);
        }

        ImGui::SameLine();

        if (ImGui::Button("Cancel")) {
            show_add_sphere_dialog = false;
        }
        ImGui::End();
    }

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

    bool show_add_rectangle_dialog = false;
    bool show_add_sphere_dialog = false;
    bool show_add_torus_dialog = false;
    // bool show_add_tensor_product_bezier_dialog = false;
};

META_REGISTER_TYPE(MainComponentRegisterTag, ConstructorWidget)