#pragma once

#include <pybind11/embed.h>

#include "geometry/mesh/mesh.hpp"

#include "pybind/bind_glm.hpp"

PYBIND11_EMBEDDED_MODULE(graphics_lab, m) {
    BindGLM::bind_glm(m);

    py::class_<Vertex3D>(m, "Vertex3D")
        .def(py::init<>())
        .def_readwrite("position", &Vertex3D::position)
        .def_readwrite("color", &Vertex3D::color)
        .def_readwrite("normal", &Vertex3D::normal)
        .def_readwrite("uv", &Vertex3D::uv);

    py::class_<Mesh3D>(m, "Mesh3D")
        .def(py::init<>())
        .def_readwrite("vertices", &Mesh3D::vertices)
        .def_readwrite("indices", &Mesh3D::indices);
}