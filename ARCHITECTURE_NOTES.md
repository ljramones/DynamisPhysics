# Architecture Notes

## ODE4J Step Loop Variant (Documented Deviation)

The architecture intent is preserved: controller forces are applied before integration.

For ODE4J specifically, the effective step order is:

1. `forceAccumulator.flush()`
2. `vehicleSystem.stepAll(dt)`
3. `characterController.stepAll(dt, gravity)`
4. `OdeHelper.spaceCollide(...)`
5. `world.quickStep(dt)`
6. `constraintRegistry.checkBreakForces()`
7. `contactGroup.empty()`

Why this differs from a `quickStep(0)` post-collide phase:

- ODE4J rejects zero step sizes (`stepsize must be > 0`).
- Solving contacts in the same `quickStep(dt)` after collision generation is the ODE-safe equivalent.

Invariants kept:

- Vehicle/character control forces are part of the same substep solve.
- Break-force checks run after solve (when joint feedback is valid).
- Deterministic iteration/order constraints remain in place.
