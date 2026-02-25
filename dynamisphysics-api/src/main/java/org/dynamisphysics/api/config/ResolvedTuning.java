package org.dynamisphysics.api.config;

public record ResolvedTuning(
    PhysicsTuningProfile profile,
    boolean deterministic,
    int solverIterations,
    int threads,
    AllocatorMode allocatorMode,
    int allocatorBytes
) {
}
