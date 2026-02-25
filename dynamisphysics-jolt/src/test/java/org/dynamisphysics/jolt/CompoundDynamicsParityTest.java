package org.dynamisphysics.jolt;

import org.dynamiscollision.shapes.CollisionShape;
import org.dynamisphysics.api.PhysicsWorldFactory;
import org.dynamisphysics.api.body.BodyMode;
import org.dynamisphysics.api.body.BodyState;
import org.dynamisphysics.api.body.RigidBodyConfig;
import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.config.PhysicsBackend;
import org.dynamisphysics.api.config.PhysicsWorldConfig;
import org.dynamisphysics.api.world.PhysicsWorld;
import org.dynamisphysics.ode4j.Ode4jBackendRegistrar;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
import org.vectrix.affine.Transformf;
import org.vectrix.core.Matrix4f;
import org.vectrix.core.Quaternionf;
import org.vectrix.core.Vector3f;

import java.util.List;
import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.assertTrue;

class CompoundDynamicsParityTest {

    @BeforeAll
    static void registerBackends() {
        new Ode4jBackendRegistrar();
        new JoltBackendRegistrar();
    }

    static Stream<PhysicsBackend> backends() {
        return Stream.of(PhysicsBackend.ODE4J, PhysicsBackend.JOLT);
    }

    @ParameterizedTest
    @MethodSource("backends")
    void comShiftImpulseInducesRotation(PhysicsBackend backend) {
        PhysicsWorld world = createZeroGravityWorld(backend);
        try {
            CollisionShape compound = comShiftedCompound();
            RigidBodyHandle body = world.spawnRigidBody(
                RigidBodyConfig.builder(compound, 10f)
                    .worldTransform(new Matrix4f().identity().translation(0f, 0f, 0f))
                    .build()
            );
            BodyState before = world.getBodyState(body);

            // Apply impulse at approximate geometric center (not COM).
            float approxComX = 7f / 9f; // from radii 0.3 and 0.6 at x=-1,+1
            Vector3f point = new Vector3f(before.position().x() - approxComX, before.position().y(), before.position().z());
            world.applyImpulse(body, new Vector3f(0f, 0f, 8f), point);
            step(world, 1);

            BodyState after = world.getBodyState(body);
            float ang = after.angularVelocity().length();
            assertTrue(ang > 0.000001f, "angular velocity too small: " + ang);
        } finally {
            world.destroy();
        }
    }

    @ParameterizedTest
    @MethodSource("backends")
    void wideCompoundAcceleratesLessUnderSameTorque(PhysicsBackend backend) {
        PhysicsWorld world = createZeroGravityWorld(backend);
        try {
            RigidBodyHandle compact = world.spawnRigidBody(
                RigidBodyConfig.builder(symmetricCompound(0.5f), 8f)
                    .worldTransform(new Matrix4f().identity().translation(-2f, 2f, 0f))
                    .build()
            );
            RigidBodyHandle wide = world.spawnRigidBody(
                RigidBodyConfig.builder(symmetricCompound(2.0f), 8f)
                    .worldTransform(new Matrix4f().identity().translation(2f, 2f, 0f))
                    .build()
            );

            for (int i = 0; i < 90; i++) {
                world.applyTorque(compact, new Vector3f(0f, 6f, 0f));
                world.applyTorque(wide, new Vector3f(0f, 6f, 0f));
                world.step(1f / 60f, 1);
            }

            float compactYaw = Math.abs(world.getBodyState(compact).angularVelocity().y());
            float wideYaw = Math.abs(world.getBodyState(wide).angularVelocity().y());
            assertTrue(compactYaw > wideYaw, "compactYaw=" + compactYaw + " wideYaw=" + wideYaw);
        } finally {
            world.destroy();
        }
    }

    private static PhysicsWorld createWorldWithGround(PhysicsBackend backend) {
        PhysicsWorld world = PhysicsWorldFactory.create(PhysicsWorldConfig.defaults(backend));
        world.spawnRigidBody(
            RigidBodyConfig.builder(CollisionShape.box(100f, 0.2f, 100f), 0f)
                .mode(BodyMode.STATIC)
                .worldTransform(new Matrix4f().identity().translation(0f, -0.2f, 0f))
                .build()
        );
        return world;
    }

    private static PhysicsWorld createZeroGravityWorld(PhysicsBackend backend) {
        PhysicsWorldConfig d = PhysicsWorldConfig.defaults(backend);
        return PhysicsWorldFactory.create(new PhysicsWorldConfig(
            backend,
            new Vector3f(0f, 0f, 0f),
            d.fixedTimeStep(),
            d.maxSubSteps(),
            d.solverIterations(),
            d.maxBodies(),
            d.maxConstraints(),
            d.broadphase(),
            d.deterministic()
        ));
    }

    private static CollisionShape comShiftedCompound() {
        return CollisionShape.compound(
            List.of(CollisionShape.sphere(0.3f), CollisionShape.sphere(0.6f)),
            List.of(local(-1f, 0f, 0f), local(1f, 0f, 0f))
        );
    }

    private static CollisionShape symmetricCompound(float offsetX) {
        return CollisionShape.compound(
            List.of(CollisionShape.sphere(0.4f), CollisionShape.sphere(0.4f)),
            List.of(local(-offsetX, 0f, 0f), local(offsetX, 0f, 0f))
        );
    }

    private static Transformf local(float x, float y, float z) {
        return new Transformf(
            new Vector3f(x, y, z),
            new Quaternionf(0f, 0f, 0f, 1f),
            new Vector3f(1f, 1f, 1f)
        );
    }

    private static void step(PhysicsWorld world, int n) {
        for (int i = 0; i < n; i++) {
            world.step(1f / 60f, 1);
        }
    }
}
