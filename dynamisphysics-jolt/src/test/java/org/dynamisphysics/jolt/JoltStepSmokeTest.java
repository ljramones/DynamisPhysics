package org.dynamisphysics.jolt;

import org.dynamiscollision.shapes.CollisionShape;
import org.dynamisphysics.api.body.BodyMode;
import org.dynamisphysics.api.body.RigidBodyConfig;
import org.dynamisphysics.api.config.PhysicsBackend;
import org.dynamisphysics.api.config.PhysicsWorldConfig;
import org.dynamisphysics.api.world.PhysicsWorld;
import org.junit.jupiter.api.Test;
import org.vectrix.core.Matrix4f;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;

class JoltStepSmokeTest {

    @Test
    void worldCreateSpawnStepDestroy() {
        assertDoesNotThrow(() -> {
            System.setProperty("jolt.trace", "true");
            System.err.println("JOLT-STEP-SMOKE create");
            PhysicsWorld world = JoltPhysicsWorld.create(
                PhysicsWorldConfig.defaults(PhysicsBackend.JOLT)
            );
            try {
                System.err.println("JOLT-STEP-SMOKE spawn-ground");
                world.spawnRigidBody(
                    RigidBodyConfig.builder(CollisionShape.box(20f, 0.5f, 20f), 0f)
                        .mode(BodyMode.STATIC)
                        .worldTransform(new Matrix4f().identity().translation(0f, -0.5f, 0f))
                        .build()
                );
                System.err.println("JOLT-STEP-SMOKE spawn-sphere");
                world.spawnRigidBody(
                    RigidBodyConfig.builder(CollisionShape.sphere(0.5f), 1f)
                        .worldTransform(new Matrix4f().identity().translation(0f, 4f, 0f))
                        .build()
                );
                System.err.println("JOLT-STEP-SMOKE step");
                world.step(1f / 60f, 1);
            } finally {
                System.err.println("JOLT-STEP-SMOKE destroy");
                world.destroy();
            }
        });
    }
}
