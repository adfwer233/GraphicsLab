# Reflection

GraphicsLab uses a lightweight reflection layer to expose fields and metadata for editor, UI, and tooling workflows.

## Prerequisites

Before using reflection directly:

- Read [Language Utilities](language.md)
- Review [Create an Application](../graphicslab/create_application.md) for a practical `reflect()` usage pattern

## What It Enables

- Uniform access to object fields at runtime
- Generic UI generation for editable properties
- Reduced boilerplate in serialization-like paths

## Key Headers

- `include/language/reflection/static_reflector.hpp`
- `include/language/reflection/reflectors.hpp`
- `include/language/reflection/function_meta.hpp`
- `include/language/reflection/custom_serizalization.hpp`

## Design Notes

- Prefer compile-time registration where possible.
- Keep reflected metadata small to avoid unnecessary runtime overhead.
- Use reflection for tooling and inspection boundaries, not hot inner loops.

## Expected Output

After this page, you should be able to:

- Understand why reflection is used for UI/tooling in GraphicsLab
- Identify the core headers needed to expose fields and functions
- Add one reflected action to a project class (for example, a `visualize` button)

## Verify

- You can locate `reflectors.hpp` and `function_meta.hpp`
- You can find at least one `reflect()` override in the codebase
- You can describe what metadata is exposed through `TypeErasedValue`
