#pragma once

#include "graphics_lab/project.hpp"
#include "spdlog/spdlog.h"

#include "explicit_surface_examples.hpp"

#include <geometry/constructor/explicit_surface_constructors.hpp>
#include <geometry/parametric/tessellator.hpp>
#include <geometry/parametric_intersector/surface_surface_intersector.hpp>
#include <geometry/parametric_topology/brep_face.hpp>
#include <geometry/parametric_topology/brep_edge.hpp>
#include <geometry/parametric_topology/brep_coedge.hpp>
#include <geometry/parametric_topology/brep_loop.hpp>

struct VisualizationProject : IGraphicsLabProject {
    void tick() override {
        spdlog::info("tick in visualization project");
    }

    std::string name() override {
        return "Visualization";
    }

    void afterLoad() override {
        spdlog::info("project loaded");
    }

    void visualize_intersection() {
        std::scoped_lock lock(context.sceneTree->sceneTreeMutex);

        auto surf = GraphicsLab::Geometry::ExplicitSurfaceConstructor::createHyperboloid();
        GraphicsLab::Geometry::Tessellator::tessellate(surf);
        context.sceneTree->addGeometryNode<GraphicsLab::Geometry::ExplicitSurface>(std::move(surf), "test explicit");

        GraphicsLab::Geometry::Torus torus({0.0, 0.0, 0.0}, 2.0, 0.5, {0.0, 1.0, 0.0}, {1.0, 0.0, 0.0});
        GraphicsLab::Geometry::Tessellator::tessellate(torus);
        context.sceneTree->addGeometryNode<GraphicsLab::Geometry::Torus>(std::move(torus), "test torus");

        auto result = GraphicsLab::Geometry::SurfaceSurfaceIntersector::intersect_all(torus, surf);

        PointCloud3D point_cloud;

        for (auto &trace : result.traces) {
            for (auto &p : trace) {
                point_cloud.vertices.emplace_back(p.position);
            }
        }
        context.sceneTree->addGeometryNode<PointCloud3D>(std::move(point_cloud), "point cloud");
    }

    void visualize_deformed_torus() {
        auto surf = ExplicitSurfaceExample::createDeformedTorus();
        GraphicsLab::Geometry::Tessellator::tessellate(surf, 64, 64);
        context.sceneTree->addGeometryNode<GraphicsLab::Geometry::ExplicitSurface>(std::move(surf), "test explicit");
    }

    void intersection_demo2() {
        auto surf = ExplicitSurfaceExample::createDeformedTorus();
        // GraphicsLab::Geometry::Tessellator::tessellate(surf, 64, 64);
        // context.sceneTree->addGeometryNode<GraphicsLab::Geometry::ExplicitSurface>(std::move(surf), "test torus");

        auto surf2 = GraphicsLab::Geometry::ExplicitSurfaceConstructor::createHyperboloid(1.6, 2.0, 1.5);
        GraphicsLab::Geometry::Tessellator::tessellate(surf2);
        context.sceneTree->addGeometryNode<GraphicsLab::Geometry::ExplicitSurface>(std::move(surf2),
                                                                                   "test hyperboloid");

        auto result = GraphicsLab::Geometry::SurfaceSurfaceIntersector::intersect_all(surf, surf2);

        auto surface1 = new GraphicsLab::Geometry::ExplicitSurface(ExplicitSurfaceExample::createDeformedTorus());
        auto face1 = new GraphicsLab::Geometry::BRepFace();
        face1->surface = surface1;

        for (int i = 0; auto &trace : result.traces) {
            PointCloud3D point_cloud;
            PointCloud2D pcurve1;

            for (auto &p : trace) {
                point_cloud.vertices.emplace_back(p.position);
                pcurve1.vertices.emplace_back(p.param1);
            }

            auto edge = new GraphicsLab::Geometry::BRepEdge();
            edge->geometry = new GraphicsLab::Geometry::BSplineCurve3D(result.curve_list[i]);
            edge->orientation = GraphicsLab::Geometry::BRepEdge::Orientation::FORWARD;

            auto coedge1 = new GraphicsLab::Geometry::BRepCoedge();
            coedge1->geometry = new GraphicsLab::Geometry::BSplineCurve2D(result.pcurve_list1[i]);
            coedge1->orientation = GraphicsLab::Geometry::BRepCoedge::Orientation::FORWARD;
            coedge1->edge = edge;

            auto loop = new GraphicsLab::Geometry::BRepLoop();
            loop->coedges = {coedge1};
            loop->faces = {face1};

            face1->boundary.push_back(loop);

            auto start_pos = result.curve_list[i].evaluate(0);
            auto end_pos = result.curve_list[i].evaluate(1);
            spdlog::info("distance {}, {}", glm::distance(start_pos, end_pos), glm::distance(point_cloud.vertices.front().position, point_cloud.vertices.back().position));

            std::scoped_lock lock(context.sceneTree->sceneTreeMutex);

            GraphicsLab::Geometry::Tessellator::tessellate(result.curve_list[i], 1280);
            context.sceneTree->addGeometryNode<GraphicsLab::Geometry::BSplineCurve3D>(std::move(result.curve_list[i]), std::format("curve bs {}", i));

            GraphicsLab::Geometry::Tessellator::tessellate(result.pcurve_list1[i], 1280);
            context.sceneTree->addGeometryNode<GraphicsLab::Geometry::BSplineCurve2D>(std::move(result.pcurve_list1[i]), std::format("pcurve bs {}", i));

            // context.sceneTree->addGeometryNode<PointCloud3D>(std::move(point_cloud), std::format("curve {}", i));
            // context.sceneTree->addGeometryNode<PointCloud2D>(std::move(pcurve1), std::format("pcurve {}", i));
            i++;
        }

        GraphicsLab::Geometry::Tessellator::tessellate(face1);
        auto mesh2d_data = *face1->mesh2d;
        auto mesh_data = *face1->mesh;
        spdlog::info("test {}", mesh2d_data.vertices.size());

        context.sceneTree->addGeometryNode<Mesh3D>(std::move(mesh_data), "face 1");
        context.sceneTree->addGeometryNode<Mesh2D>(std::move(mesh2d_data), "param space");
        spdlog::info("tessellation finish");
    }

    ReflectDataType reflect() override {
        auto result = IGraphicsLabProject::reflect();
        result.emplace("tick", TypeErasedValue(&VisualizationProject::tick, this));
        result.emplace("visualize_intersection", TypeErasedValue(&VisualizationProject::visualize_intersection, this));
        result.emplace("visualize_deformed_torus",
                       TypeErasedValue(&VisualizationProject::visualize_deformed_torus, this));
        result.emplace("intersection_demo2", TypeErasedValue(&VisualizationProject::intersection_demo2, this));
        return result;
    }
};
