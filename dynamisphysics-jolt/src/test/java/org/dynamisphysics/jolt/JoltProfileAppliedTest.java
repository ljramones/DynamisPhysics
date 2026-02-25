package org.dynamisphysics.jolt;

import org.dynamisphysics.api.config.AllocatorMode;
import org.dynamisphysics.api.config.BroadphaseType;
import org.dynamisphysics.api.config.PhysicsBackend;
import org.dynamisphysics.api.config.PhysicsTuning;
import org.dynamisphysics.api.config.PhysicsTuningProfile;
import org.dynamisphysics.api.config.PhysicsWorldConfig;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.vectrix.core.Vector3f;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class JoltProfileAppliedTest {

    @AfterEach
    void clearProps() {
        System.clearProperty("jolt.threads");
        System.clearProperty("physics.profile");
    }

    @Test
    void deterministicProfileResolvesSingleThread() {
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
            new PhysicsTuning(PhysicsTuningProfile.DETERMINISTIC, null, null, null, null, null)
        );
        JoltPhysicsWorld world = JoltPhysicsWorld.create(cfg);
        try {
            assertEquals(1, world.resolvedTuningForTesting().threads());
            assertTrue(world.resolvedTuningForTesting().deterministic());
        } finally {
            world.destroy();
        }
    }

    @Test
    void perfProfileUsesImplAllocatorDefault() {
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
            new PhysicsTuning(PhysicsTuningProfile.PERF, null, null, null, null, null)
        );
        JoltPhysicsWorld world = JoltPhysicsWorld.create(cfg);
        try {
            assertEquals(AllocatorMode.IMPL, world.resolvedTuningForTesting().allocatorMode());
            assertEquals(128 * 1024 * 1024, world.resolvedTuningForTesting().allocatorBytes());
        } finally {
            world.destroy();
        }
    }

    @Test
    void systemThreadOverrideWinsOverDeterministicProfile() {
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
            new PhysicsTuning(PhysicsTuningProfile.DETERMINISTIC, null, null, null, null, null)
        );
        JoltPhysicsWorld world = JoltPhysicsWorld.create(cfg);
        try {
            assertEquals(8, world.resolvedTuningForTesting().threads());
        } finally {
            world.destroy();
        }
    }
}
