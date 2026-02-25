package org.dynamisphysics.api.config;

import org.vectrix.core.Vector3f;

public record PhysicsWorldConfig(
    PhysicsBackend backend,
    Vector3f gravity,
    float fixedTimeStep,
    int maxSubSteps,
    @Deprecated(forRemoval = false)
    int solverIterations,
    int maxBodies,
    int maxConstraints,
    BroadphaseType broadphase,
    @Deprecated(forRemoval = false)
    boolean deterministic,
    PhysicsTuning tuning
) {
    public PhysicsWorldConfig(
        PhysicsBackend backend,
        Vector3f gravity,
        float fixedTimeStep,
        int maxSubSteps,
        int solverIterations,
        int maxBodies,
        int maxConstraints,
        BroadphaseType broadphase,
        boolean deterministic
    ) {
        this(backend, gravity, fixedTimeStep, maxSubSteps, solverIterations, maxBodies, maxConstraints, broadphase, deterministic, null);
    }

    public static PhysicsWorldConfig defaults(PhysicsBackend backend) {
        return new PhysicsWorldConfig(
            backend,
            new Vector3f(0f, -9.81f, 0f),
            1f / 60f,
            4,
            10,
            65_536,
            16_384,
            BroadphaseType.BVH,
            true,
            PhysicsTuning.defaults()
        );
    }
}
