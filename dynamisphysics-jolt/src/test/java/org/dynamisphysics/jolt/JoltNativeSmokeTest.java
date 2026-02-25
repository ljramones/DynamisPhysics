package org.dynamisphysics.jolt;

import org.dynamisphysics.api.config.PhysicsBackend;
import org.dynamisphysics.api.config.PhysicsWorldConfig;
import org.dynamisphysics.api.world.PhysicsWorld;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;

class JoltNativeSmokeTest {

    @Test
    void joltNativeRuntimeInitialises() {
        assertDoesNotThrow(() -> {
            PhysicsWorld world = JoltPhysicsWorld.create(
                PhysicsWorldConfig.defaults(PhysicsBackend.JOLT)
            );
            world.destroy();
        });
    }

    @Test
    void joltWorldCanCreateDestroyRepeatedly() {
        assertDoesNotThrow(() -> {
            for (int i = 0; i < 4; i++) {
                PhysicsWorld world = JoltPhysicsWorld.create(
                    PhysicsWorldConfig.defaults(PhysicsBackend.JOLT)
                );
                world.destroy();
            }
        });
    }
}
