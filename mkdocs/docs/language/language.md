# Language Utilities

The `language` module contains reusable C++ infrastructure used across GraphicsLab.

## Prerequisites

Before exploring language utilities:

- Complete [Get Started](../graphicslab/get_started.md)
- Review [Create an Application](../graphicslab/create_application.md) for project-level context

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

## Expected Output

After this page, you should be able to:

- Identify where reflection and parallel utilities are implemented
- Decide whether a feature belongs in the language module versus geometry/rendering modules
- Navigate quickly to key header entry points for extension

## Verify

- You can locate reflection headers under `include/language/reflection/`
- You can locate parallel headers under `include/language/parallel/`
- You can explain one usage path from utility headers to higher-level modules
