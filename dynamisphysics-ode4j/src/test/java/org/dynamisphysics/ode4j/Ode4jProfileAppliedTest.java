package org.dynamisphysics.ode4j;

import org.dynamisphysics.api.config.BroadphaseType;
import org.dynamisphysics.api.config.PhysicsBackend;
import org.dynamisphysics.api.config.PhysicsTuning;
import org.dynamisphysics.api.config.PhysicsTuningProfile;
import org.dynamisphysics.api.config.PhysicsWorldConfig;
import org.junit.jupiter.api.Test;
import org.vectrix.core.Vector3f;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class Ode4jProfileAppliedTest {

    @Test
    void perfProfileAppliesSolverIterations() {
        PhysicsWorldConfig cfg = new PhysicsWorldConfig(
            PhysicsBackend.ODE4J,
            new Vector3f(0f, -9.81f, 0f),
            1f / 60f,
            4,
            10,
            65_536,
            16_384,
            BroadphaseType.BVH,
            true,
            new PhysicsTuning(PhysicsTuningProfile.PERF, null, null, null, null, 8)
        );

        Ode4jPhysicsWorld world = Ode4jPhysicsWorld.create(cfg);
        try {
            assertEquals(8, world.resolvedTuningForTesting().solverIterations());
            assertTrue(world.resolvedTuningForTesting().profile() == PhysicsTuningProfile.PERF);
        } finally {
            world.destroy();
        }
    }

    @Test
    void deterministicProfileAppliesDeterministicMode() {
        PhysicsWorldConfig cfg = new PhysicsWorldConfig(
            PhysicsBackend.ODE4J,
            new Vector3f(0f, -9.81f, 0f),
            1f / 60f,
            4,
            10,
            65_536,
            16_384,
            BroadphaseType.BVH,
            false,
            new PhysicsTuning(PhysicsTuningProfile.DETERMINISTIC, true, null, null, null, null)
        );

        Ode4jPhysicsWorld world = Ode4jPhysicsWorld.create(cfg);
        try {
            assertTrue(world.resolvedTuningForTesting().deterministic());
        } finally {
            world.destroy();
        }
    }
}
