#pragma once

template <typename T>
concept IsConstructor = requires { typename T::vertex_type; };