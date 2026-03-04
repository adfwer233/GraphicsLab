# Reflection

GraphicsLab uses a lightweight reflection layer to expose fields and metadata for editor, UI, and tooling workflows.

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
