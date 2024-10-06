#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/stl.h> // For STL types like std::vector
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp> // For glm::value_ptr
#include <iostream>

namespace py = pybind11;

struct BindGLM {
    static void bind_glm(py::module& m) {
        // Bind glm::vec2
        py::class_<glm::vec2>(m, "Vec2")
            .def(py::init<>())
            .def(py::init<float, float>())
            .def_readwrite("x", &glm::vec2::x)
            .def_readwrite("y", &glm::vec2::y)
            .def("__add__", [](const glm::vec2 &v1, const glm::vec2 &v2) { return v1 + v2; })
            .def("__sub__", [](const glm::vec2 &v1, const glm::vec2 &v2) { return v1 - v2; })
            .def("__mul__", [](const glm::vec2 &v, float scalar) { return v * scalar; })
            .def("__repr__", [](const glm::vec2 &v) { return "Vec2(" + std::to_string(v.x) + ", " + std::to_string(v.y) + ")"; });

        // Bind glm::vec3
        py::class_<glm::vec3>(m, "Vec3")
            .def(py::init<>())
            .def(py::init<float, float, float>())
            .def_readwrite("x", &glm::vec3::x)
            .def_readwrite("y", &glm::vec3::y)
            .def_readwrite("z", &glm::vec3::z)
            .def("__add__", [](const glm::vec3 &v1, const glm::vec3 &v2) { return v1 + v2; })
            .def("__sub__", [](const glm::vec3 &v1, const glm::vec3 &v2) { return v1 - v2; })
            .def("__mul__", [](const glm::vec3 &v, float scalar) { return v * scalar; })
            .def("__repr__", [](const glm::vec3 &v) { return "Vec3(" + std::to_string(v.x) + ", " + std::to_string(v.y) + ", " + std::to_string(v.z) + ")"; });

        // Bind glm::vec4
        py::class_<glm::vec4>(m, "Vec4")
            .def(py::init<>())
            .def(py::init<float, float, float, float>())
            .def_readwrite("x", &glm::vec4::x)
            .def_readwrite("y", &glm::vec4::y)
            .def_readwrite("z", &glm::vec4::z)
            .def_readwrite("w", &glm::vec4::w)
            .def("__add__", [](const glm::vec4 &v1, const glm::vec4 &v2) { return v1 + v2; })
            .def("__sub__", [](const glm::vec4 &v1, const glm::vec4 &v2) { return v1 - v2; })
            .def("__mul__", [](const glm::vec4 &v, float scalar) { return v * scalar; })
            .def("__repr__", [](const glm::vec4 &v) { return "Vec4(" + std::to_string(v.x) + ", " + std::to_string(v.y) + ", " + std::to_string(v.z) + ", " + std::to_string(v.w) + ")"; });

        // Bind glm::mat4
        py::class_<glm::mat4>(m, "Mat4")
            .def(py::init<>())
            .def(py::init<const glm::mat4 &>())
            .def("to_array", [](const glm::mat4 &m) {
                return std::vector<float>(glm::value_ptr(m), glm::value_ptr(m) + 16);
            })
            .def("__repr__", [](const glm::mat4 &m) {
                std::ostringstream oss;
                oss << "Mat4(";
                for (int i = 0; i < 4; ++i) {
                    for (int j = 0; j < 4; ++j) {
                        oss << m[i][j];
                        if (j < 3) oss << ", ";
                    }
                    if (i < 3) oss << "; ";
                }
                oss << ")";
                return oss.str();
            });
    }

};