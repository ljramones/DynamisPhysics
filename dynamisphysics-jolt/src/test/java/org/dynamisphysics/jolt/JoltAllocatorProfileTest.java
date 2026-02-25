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

class JoltAllocatorProfileTest {

    @AfterEach
    void clearProps() {
        System.clearProperty("jolt.threads");
        System.clearProperty("jolt.alloc");
        System.clearProperty("jolt.alloc.mb");
        System.clearProperty("physics.profile");
    }

    @Test
    void perfDefaultsUseImplAllocatorAndCappedAutoThreads() {
        PhysicsWorldConfig cfg = cfg(new PhysicsTuning(PhysicsTuningProfile.PERF, null, null, null, null, null), false);
        JoltPhysicsWorld world = JoltPhysicsWorld.create(cfg);
        try {
            assertEquals(AllocatorMode.IMPL, world.resolvedTuningForTesting().allocatorMode());
            assertEquals(128 * 1024 * 1024, world.resolvedTuningForTesting().allocatorBytes());
            assertEquals(Math.min(Runtime.getRuntime().availableProcessors(), 8), world.resolvedTuningForTesting().threads());
        } finally {
            world.destroy();
        }
    }

    @Test
    void safeAllocatorResolvesToMallocInJolt() {
        PhysicsWorldConfig cfg = cfg(new PhysicsTuning(PhysicsTuningProfile.DEFAULT, null, null, AllocatorMode.SAFE, 64, null), true);
        JoltPhysicsWorld world = JoltPhysicsWorld.create(cfg);
        try {
            assertEquals(AllocatorMode.MALLOC, world.resolvedTuningForTesting().allocatorMode());
        } finally {
            world.destroy();
        }
    }

    @Test
    void clampsAllocatorMbAndThreadsAndRespectsPrecedence() {
        System.setProperty("jolt.alloc.mb", "2048");
        System.setProperty("jolt.threads", "999");
        PhysicsWorldConfig cfg = cfg(new PhysicsTuning(PhysicsTuningProfile.PERF, null, 4, AllocatorMode.IMPL, 32, null), false);
        JoltPhysicsWorld world = JoltPhysicsWorld.create(cfg);
        try {
            assertEquals(512 * 1024 * 1024, world.resolvedTuningForTesting().allocatorBytes());
            assertEquals(64, world.resolvedTuningForTesting().threads());
        } finally {
            world.destroy();
        }
    }

    @Test
    void lowerBoundClampForAllocatorAndThreads() {
        System.setProperty("jolt.alloc.mb", "0");
        System.setProperty("jolt.threads", "0");
        PhysicsWorldConfig cfg = cfg(new PhysicsTuning(PhysicsTuningProfile.PERF, null, null, AllocatorMode.IMPL, null, null), false);
        JoltPhysicsWorld world = JoltPhysicsWorld.create(cfg);
        try {
            assertEquals(1 * 1024 * 1024, world.resolvedTuningForTesting().allocatorBytes());
            assertEquals(1, world.resolvedTuningForTesting().threads());
        } finally {
            world.destroy();
        }
    }

    @Test
    void systemOverridesConfigThreads() {
        System.setProperty("jolt.threads", "8");
        PhysicsWorldConfig cfg = cfg(new PhysicsTuning(PhysicsTuningProfile.DEFAULT, null, 4, null, null, null), false);
        JoltPhysicsWorld world = JoltPhysicsWorld.create(cfg);
        try {
            assertEquals(8, world.resolvedTuningForTesting().threads());
        } finally {
            world.destroy();
        }
    }

    private static PhysicsWorldConfig cfg(PhysicsTuning tuning, boolean deterministic) {
        return new PhysicsWorldConfig(
            PhysicsBackend.JOLT,
            new Vector3f(0f, -9.81f, 0f),
            1f / 60f,
            4,
            10,
            65_536,
            16_384,
            BroadphaseType.BVH,
            deterministic,
            tuning
        );
    }
}
