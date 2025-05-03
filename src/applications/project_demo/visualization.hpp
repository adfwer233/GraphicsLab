#pragma once

#include "graphics_lab/project.hpp"
#include "spdlog/spdlog.h"

#include "explicit_surface_examples.hpp"

#include <geometry/constructor/explicit_surface_constructors.hpp>
#include <geometry/parametric/tessellator.hpp>
#include <geometry/parametric/torus.hpp>
#include <geometry/parametric_intersector/curve_curve_intersector_2d/line_param_intersector_2d.hpp>
#include <geometry/parametric_intersector/surface_surface_intersector.hpp>
#include <geometry/parametric_topology/brep_coedge.hpp>
#include <geometry/parametric_topology/brep_edge.hpp>
#include <geometry/parametric_topology/brep_face.hpp>
#include <geometry/parametric_topology/brep_loop.hpp>
#include <utils/sampler.hpp>

#include "geometry/parametric_topology/trimming/boundary_cutting.hpp"

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
        auto surface2 = new GraphicsLab::Geometry::ExplicitSurface(
            GraphicsLab::Geometry::ExplicitSurfaceConstructor::createHyperboloid(1.6, 2.0, 1.5));
        auto face1 = new GraphicsLab::Geometry::BRepFace();
        auto face2 = new GraphicsLab::Geometry::BRepFace();

        face1->surface = surface1;
        face2->surface = surface2;

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
            coedge1->orientation = GraphicsLab::Geometry::BRepCoedge::Orientation::BACKWARD;
            coedge1->edge = edge;

            auto coedge2 = new GraphicsLab::Geometry::BRepCoedge();
            coedge2->geometry = new GraphicsLab::Geometry::BSplineCurve2D(result.pcurve_list2[i]);
            coedge2->orientation = GraphicsLab::Geometry::BRepCoedge::Orientation::FORWARD;
            coedge2->edge = edge;

            auto loop = new GraphicsLab::Geometry::BRepLoop();
            loop->coedges = {coedge1};
            loop->faces = {face1};
            face1->boundary.push_back(loop);

            auto loop2 = new GraphicsLab::Geometry::BRepLoop();
            loop2->coedges = {coedge2};
            loop2->faces = {face2};
            face2->boundary.push_back(loop2);

            auto start_pos = result.curve_list[i].evaluate(0);
            auto end_pos = result.curve_list[i].evaluate(1);
            spdlog::info("distance {}, {}", glm::distance(start_pos, end_pos),
                         glm::distance(point_cloud.vertices.front().position, point_cloud.vertices.back().position));

            std::scoped_lock lock(context.sceneTree->sceneTreeMutex);

            GraphicsLab::Geometry::Tessellator::tessellate(result.curve_list[i], 1280);
            context.sceneTree->addGeometryNode<GraphicsLab::Geometry::BSplineCurve3D>(std::move(result.curve_list[i]),
                                                                                      std::format("curve bs {}", i));

            GraphicsLab::Geometry::Tessellator::tessellate(result.pcurve_list1[i], 1280);
            context.sceneTree->addGeometryNode<GraphicsLab::Geometry::BSplineCurve2D>(std::move(result.pcurve_list1[i]),
                                                                                      std::format("pcurve1 bs {}", i));

            GraphicsLab::Geometry::Tessellator::tessellate(result.pcurve_list2[i], 1280);
            context.sceneTree->addGeometryNode<GraphicsLab::Geometry::BSplineCurve2D>(std::move(result.pcurve_list2[i]),
                                                                                      std::format("pcurve2 bs {}", i));

            // context.sceneTree->addGeometryNode<PointCloud3D>(std::move(point_cloud), std::format("curve {}", i));
            // context.sceneTree->addGeometryNode<PointCloud2D>(std::move(pcurve1), std::format("pcurve {}", i));
            i++;
        }

        GraphicsLab::Geometry::BoundaryCutting boundary_cutting(face1);
        boundary_cutting.cut_boundary();

        face2->trim_flag = false;
        GraphicsLab::Geometry::Tessellator::tessellate(face1);
        GraphicsLab::Geometry::Tessellator::tessellate(face2);
        auto mesh2d_data = *face1->mesh2d;
        auto mesh_data = *face1->mesh;
        auto mesh_data2 = *face2->mesh;
        spdlog::info("test {}", mesh_data.indices.size());
        spdlog::info("test {}", mesh_data2.indices.size());

        context.sceneTree->addGeometryNode<Mesh3D>(std::move(mesh_data), "face 1");
        context.sceneTree->addGeometryNode<Mesh3D>(std::move(mesh_data2), "face 2");
        context.sceneTree->addGeometryNode<Mesh2D>(std::move(mesh2d_data), "param space");
        spdlog::info("tessellation finish");

        int sample_on_face1 = 5000;
        int sample_on_face2 = 5000;

        PointCloud3D point_cloud1;
        for (int i = 0; i < sample_on_face1; i++) {
            glm::dvec2 param{GraphicsLab::Sampler::sampleUniform(), GraphicsLab::Sampler::sampleUniform()};
            auto pos = face1->surface->evaluate(param);
            glm::vec3 color{0.0, 1.0, 0.0};
            if (face1->contain(param)) {
                color = glm::vec3{1.0, 0.0, 0.0};
            }
            point_cloud1.vertices.emplace_back(pos, color);
        }

        PointCloud3D point_cloud2;
        for (int i = 0; i < sample_on_face2; i++) {
            glm::dvec2 param{GraphicsLab::Sampler::sampleUniform(), GraphicsLab::Sampler::sampleUniform()};
            auto pos = face2->surface->evaluate(param);
            glm::vec3 color{0.0, 1.0, 0.0};
            if (face2->contain(param)) {
                color = glm::vec3{1.0, 0.0, 0.0};
            }
            point_cloud2.vertices.emplace_back(pos, color);
        }

        context.sceneTree->addGeometryNode<PointCloud3D>(std::move(point_cloud1), "pc 1");
        context.sceneTree->addGeometryNode<PointCloud3D>(std::move(point_cloud2), "pc 2");
    }

    void intersection_demo3() {
        auto surf1 = ExplicitSurfaceExample::createDeformedTorus2({0.0, 0.0, 0.0});
        auto surf2 = ExplicitSurfaceExample::createDeformedTorus2({4.2, 0.0, 0.0}, {std::numbers::pi, 0.0});
        GraphicsLab::Geometry::Tessellator::tessellate(surf1, 64, 64);
        GraphicsLab::Geometry::Tessellator::tessellate(surf2, 64, 64);

        auto result = GraphicsLab::Geometry::SurfaceSurfaceIntersector::intersect_all(surf1, surf2);

        auto surface1 = new GraphicsLab::Geometry::ExplicitSurface(
            ExplicitSurfaceExample::createDeformedTorus2({0.0, 0.0, 0.0}));
        auto surface2 = new GraphicsLab::Geometry::ExplicitSurface(
            ExplicitSurfaceExample::createDeformedTorus2({4.2, 0.0, 0.0}, {std::numbers::pi, 0.0}));
        auto face1 = new GraphicsLab::Geometry::BRepFace();
        auto face2 = new GraphicsLab::Geometry::BRepFace();

        face1->surface = surface1;
        face2->surface = surface2;

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
            coedge1->orientation = GraphicsLab::Geometry::BRepCoedge::Orientation::BACKWARD;
            coedge1->edge = edge;

            auto coedge2 = new GraphicsLab::Geometry::BRepCoedge();
            coedge2->geometry = new GraphicsLab::Geometry::BSplineCurve2D(result.pcurve_list2[i]);
            coedge2->orientation = GraphicsLab::Geometry::BRepCoedge::Orientation::FORWARD;
            coedge2->edge = edge;

            auto loop = new GraphicsLab::Geometry::BRepLoop();
            loop->coedges = {coedge1};
            loop->faces = {face1};
            face1->boundary.push_back(loop);

            auto loop2 = new GraphicsLab::Geometry::BRepLoop();
            loop2->coedges = {coedge2};
            loop2->faces = {face2};
            face2->boundary.push_back(loop2);

            auto start_pos = result.curve_list[i].evaluate(0);
            auto end_pos = result.curve_list[i].evaluate(1);
            spdlog::info("distance {}, {}", glm::distance(start_pos, end_pos),
                         glm::distance(point_cloud.vertices.front().position, point_cloud.vertices.back().position));

            std::scoped_lock lock(context.sceneTree->sceneTreeMutex);

            GraphicsLab::Geometry::Tessellator::tessellate(result.curve_list[i], 1280);
            context.sceneTree->addGeometryNode<GraphicsLab::Geometry::BSplineCurve3D>(std::move(result.curve_list[i]),
                                                                                      std::format("curve bs {}", i));

            GraphicsLab::Geometry::Tessellator::tessellate(result.pcurve_list1[i], 1280);
            context.sceneTree->addGeometryNode<GraphicsLab::Geometry::BSplineCurve2D>(std::move(result.pcurve_list1[i]),
                                                                                      std::format("pcurve1 bs {}", i));

            GraphicsLab::Geometry::Tessellator::tessellate(result.pcurve_list2[i], 1280);
            context.sceneTree->addGeometryNode<GraphicsLab::Geometry::BSplineCurve2D>(std::move(result.pcurve_list2[i]),
                                                                                      std::format("pcurve2 bs {}", i));

            // context.sceneTree->addGeometryNode<PointCloud3D>(std::move(point_cloud), std::format("curve {}", i));
            // context.sceneTree->addGeometryNode<PointCloud2D>(std::move(pcurve1), std::format("pcurve {}", i));
            i++;
        }

        GraphicsLab::Geometry::BoundaryCutting boundary_cutting(face1);
        boundary_cutting.cut_boundary();

        face1->trim_flag = false;
        face2->trim_flag = false;
        GraphicsLab::Geometry::Tessellator::tessellate(face1);
        GraphicsLab::Geometry::Tessellator::tessellate(face2);
        auto mesh2d_data = *face1->mesh2d;
        auto mesh_data = *face1->mesh;
        auto mesh_data2 = *face2->mesh;
        spdlog::info("test {}", mesh_data.indices.size());
        spdlog::info("test {}", mesh_data2.indices.size());

        context.sceneTree->addGeometryNode<Mesh3D>(std::move(mesh_data), "face 1");
        context.sceneTree->addGeometryNode<Mesh3D>(std::move(mesh_data2), "face 2");
        context.sceneTree->addGeometryNode<Mesh2D>(std::move(mesh2d_data), "param space");
        spdlog::info("tessellation finish");

        int sample_on_face1 = 5000;
        int sample_on_face2 = 5000;

        PointCloud3D point_cloud1;
        for (int i = 0; i < sample_on_face1; i++) {
            glm::dvec2 param{GraphicsLab::Sampler::sampleUniform(), GraphicsLab::Sampler::sampleUniform()};
            auto pos = face1->surface->evaluate(param);
            glm::vec3 color{0.0, 1.0, 0.0};
            if (face1->contain(param)) {
                color = glm::vec3{1.0, 0.0, 0.0};
            }
            point_cloud1.vertices.emplace_back(pos, color);
        }

        PointCloud3D point_cloud2;
        for (int i = 0; i < sample_on_face2; i++) {
            glm::dvec2 param{GraphicsLab::Sampler::sampleUniform(), GraphicsLab::Sampler::sampleUniform()};
            auto pos = face2->surface->evaluate(param);
            glm::vec3 color{0.0, 1.0, 0.0};
            if (face2->contain(param)) {
                color = glm::vec3{1.0, 0.0, 0.0};
            }
            point_cloud2.vertices.emplace_back(pos, color);
        }

        context.sceneTree->addGeometryNode<PointCloud3D>(std::move(point_cloud1), "pc 1");
        context.sceneTree->addGeometryNode<PointCloud3D>(std::move(point_cloud2), "pc 2");
    }

    void convert_bezier_demo() {
        auto surf = ExplicitSurfaceExample::createDeformedTorus();
        auto surf2 = GraphicsLab::Geometry::ExplicitSurfaceConstructor::createHyperboloid(1.6, 2.0, 1.5);
        auto result = GraphicsLab::Geometry::SurfaceSurfaceIntersector::intersect_all(surf, surf2);

        auto curve = result.pcurve_list1.front();
        GraphicsLab::Geometry::Tessellator::tessellate(curve, 100);

        bool res1 = curve.is_in_bezier_form();
        curve.insert_all_knots_to_bezier_form();
        bool res2 = curve.is_in_bezier_form();

        auto bezier_curves = curve.convert_to_bezier();

        std::scoped_lock lock(context.sceneTree->sceneTreeMutex);
        for (int i = 0; i < bezier_curves.size(); i++) {
            GraphicsLab::Geometry::Tessellator::tessellate(bezier_curves[i], 100);
            context.sceneTree->addGeometryNode<GraphicsLab::Geometry::BezierCurve2D>(std::move(bezier_curves[i]),
                                                                                     std::format("curve bezier {}", i));
        }

        context.sceneTree->addGeometryNode<GraphicsLab::Geometry::BSplineCurve2D>(std::move(curve),
                                                                                  std::format("curve bspline"));
    }

    void boundary_cutting_example() {
        using namespace GraphicsLab::Geometry;
        auto surf = ExplicitSurfaceExample::createDeformedTorus();
        auto surf2 = GraphicsLab::Geometry::ExplicitSurfaceConstructor::createHyperboloid(1.6, 2.0, 1.5);
        auto result = GraphicsLab::Geometry::SurfaceSurfaceIntersector::intersect_all(surf, surf2);

        StraightLine2D straight_line{glm::dvec2{0.0, 0.0}, glm::dvec2{0.0, 1.0}};
        for (size_t i = 0; i < result.pcurve_list1.size(); i++) {
            auto inter_result = LineBSplineParamIntersector2D::intersect(straight_line, result.pcurve_list1[i]);
            for (auto pos : inter_result.inter_points) {
                spdlog::info("{} {}", pos.x, pos.y);
            }
        }
    }

    ReflectDataType reflect() override {
        auto result = IGraphicsLabProject::reflect();
        result.emplace("tick", TypeErasedValue(&VisualizationProject::tick, this));
        result.emplace("visualize_intersection", TypeErasedValue(&VisualizationProject::visualize_intersection, this));
        result.emplace("visualize_deformed_torus",
                       TypeErasedValue(&VisualizationProject::visualize_deformed_torus, this));
        result.emplace("intersection_demo2", TypeErasedValue(&VisualizationProject::intersection_demo2, this));
        result.emplace("intersection_demo3", TypeErasedValue(&VisualizationProject::intersection_demo3, this));
        result.emplace("convert_bezier_demo", TypeErasedValue(&VisualizationProject::convert_bezier_demo, this));
        result.emplace("boundary_cutting_example",
                       TypeErasedValue(&VisualizationProject::boundary_cutting_example, this));
        return result;
    }
};
