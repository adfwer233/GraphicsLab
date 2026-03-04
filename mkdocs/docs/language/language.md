# Language Utilities

The `language` module contains reusable C++ infrastructure used across GraphicsLab.

## Main Areas

- Reflection helpers (`include/language/reflection`)
- Parallel task utilities (`include/language/parallel`)
- Template/meta-programming support used by geometry and runtime tooling

These utilities are deliberately lightweight and header-driven, so most usage is visible directly in the include tree.

## Header Locations

- Reflection:
  - `include/language/reflection/static_reflector.hpp`
  - `include/language/reflection/reflectors.hpp`
  - `include/language/reflection/dynamic_fields.hpp`
- Parallel:
  - `include/language/parallel/thread_pool.hpp`
  - `include/language/parallel/task_runner.hpp`
