# Parallel

GraphicsLab parallel helpers provide simple task execution primitives for CPU-side workloads.

## Components

- `ThreadPool`: fixed worker pool for queue-based tasks
- `TaskRunner`: convenience layer for launching/joining grouped work

## Usage Pattern

1. Create a pool with an appropriate worker count.
2. Enqueue independent jobs.
3. Synchronize at deterministic boundaries.

## Guidelines

- Submit coarse-grained work units to reduce scheduling overhead.
- Avoid sharing mutable state across tasks unless synchronization is explicit.
- Keep data ownership clear (move values into tasks where possible).

## Tests

Parallel behavior is validated in:

- `test/parallel/thread_pool_test.cpp`
