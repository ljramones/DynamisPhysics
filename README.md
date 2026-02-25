# DynamisPhysics

Dual-backend physics simulation library for the Dynamis engine ecosystem.
ODE4J and Jolt Physics implementations behind a clean API.

**Status:** 🚧 0.1.0 in development

## Modules

| Module | Purpose |
|--------|---------|
| `dynamisphysics-api` | Zero-dependency interfaces and value types. All consumers depend only on this. |
| `dynamisphysics-ode4j` | ODE4J backend implementation. Pure Java, JPMS-clean. |
| `dynamisphysics-jolt` | Jolt Physics backend implementation. High-performance, multi-threaded. |
| `dynamisphysics-test` | MockPhysicsWorld, PhysicsSimHarness, PhysicsAssertions. |
| `dynamisphysics-bench` | JMH benchmarks — ODE4J vs Jolt head-to-head comparison. |

## Requirements

- Java 25+
- Vectrix 1.10.12

## Architecture Deviations

Backend-specific, documented architecture deviations are tracked in
`ARCHITECTURE_NOTES.md`. Treat that file as authoritative when behavior differs
from the baseline architecture text.
