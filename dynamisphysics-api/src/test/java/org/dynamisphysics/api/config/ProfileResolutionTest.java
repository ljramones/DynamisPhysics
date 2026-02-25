package org.dynamisphysics.api.config;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.vectrix.core.Vector3f;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class ProfileResolutionTest {

    @AfterEach
    void clearProps() {
        clear("physics.profile");
        clear("physics.deterministic");
        clear("physics.solver.iterations");
        clear("jolt.threads");
        clear("jolt.alloc");
        clear("jolt.alloc.mb");
    }

    @Test
    void systemPropertyOverridesConfigAndProfile() {
        System.setProperty("jolt.threads", "8");
        PhysicsWorldConfig cfg = new PhysicsWorldConfig(
            PhysicsBackend.JOLT,
            new Vector3f(0f, -9.81f, 0f),
            1f / 60f,
            4,
            10,
            65_536,
            16_384,
            BroadphaseType.BVH,
            true,
            new PhysicsTuning(PhysicsTuningProfile.PERF, null, 4, null, null, null)
        );
        ResolvedTuning resolved = PhysicsTuningResolver.resolve(cfg);
        assertEquals(8, resolved.threads());
    }

    @Test
    void explicitTuningOverridesLegacy() {
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
            new PhysicsTuning(PhysicsTuningProfile.DEFAULT, true, null, null, null, null)
        );
        ResolvedTuning resolved = PhysicsTuningResolver.resolve(cfg);
        assertTrue(resolved.deterministic());
    }

    @Test
    void legacyAppliesWhenTuningFieldNull() {
        PhysicsWorldConfig cfg = new PhysicsWorldConfig(
            PhysicsBackend.ODE4J,
            new Vector3f(0f, -9.81f, 0f),
            1f / 60f,
            4,
            20,
            65_536,
            16_384,
            BroadphaseType.BVH,
            true,
            new PhysicsTuning(PhysicsTuningProfile.DEFAULT, null, null, null, null, null)
        );
        ResolvedTuning resolved = PhysicsTuningResolver.resolve(cfg);
        assertEquals(20, resolved.solverIterations());
    }

    @Test
    void profileDefaultsFallbackWhenTuningNull() {
        PhysicsWorldConfig cfg = new PhysicsWorldConfig(
            PhysicsBackend.JOLT,
            new Vector3f(0f, -9.81f, 0f),
            1f / 60f,
            4,
            10,
            65_536,
            16_384,
            BroadphaseType.BVH,
            true,
            null
        );
        ResolvedTuning resolved = PhysicsTuningResolver.resolve(cfg);
        assertEquals(PhysicsTuningProfile.DEFAULT, resolved.profile());
        assertTrue(resolved.deterministic());
        assertEquals(10, resolved.solverIterations());
        assertEquals(AllocatorMode.MALLOC, resolved.allocatorMode());
        assertEquals(64 * 1024 * 1024, resolved.allocatorBytes());
    }

    @Test
    void clampsInvalidRangesAndIgnoresInvalidProfile() {
        System.setProperty("jolt.threads", "0");
        System.setProperty("physics.solver.iterations", "999");
        System.setProperty("physics.profile", "NOPE");

        ResolvedTuning resolved = PhysicsTuningResolver.resolve(PhysicsWorldConfig.defaults(PhysicsBackend.JOLT));
        assertEquals(1, resolved.threads());
        assertEquals(64, resolved.solverIterations());
        assertEquals(PhysicsTuningProfile.DEFAULT, resolved.profile());
    }

    @Test
    void resolverIsIdempotentForSameInput() {
        PhysicsWorldConfig cfg = new PhysicsWorldConfig(
            PhysicsBackend.JOLT,
            new Vector3f(0f, -9.81f, 0f),
            1f / 60f,
            4,
            9,
            65_536,
            16_384,
            BroadphaseType.BVH,
            false,
            new PhysicsTuning(PhysicsTuningProfile.PERF, null, 6, AllocatorMode.IMPL, 128, 7)
        );
        ResolvedTuning a = PhysicsTuningResolver.resolve(cfg);
        ResolvedTuning b = PhysicsTuningResolver.resolve(cfg);
        assertEquals(a, b);
        assertFalse(a.deterministic());
        assertEquals(6, a.threads());
    }

    private static void clear(String name) {
        System.clearProperty(name);
    }
}
