package org.dynamisphysics.api.config;

public record PhysicsTuning(
    PhysicsTuningProfile profile,
    Boolean deterministic,
    Integer threads,
    AllocatorMode allocatorMode,
    Integer allocatorMb,
    Integer solverIterations
) {
    public static PhysicsTuning defaults() {
        return new PhysicsTuning(PhysicsTuningProfile.DEFAULT, null, null, null, null, null);
    }
}
