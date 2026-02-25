package org.dynamisphysics.jolt;

import org.dynamiscollision.shapes.CollisionShape;
import org.dynamisphysics.api.PhysicsWorldFactory;
import org.dynamisphysics.api.body.BodyMode;
import org.dynamisphysics.api.body.BodyState;
import org.dynamisphysics.api.body.RigidBodyConfig;
import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.config.PhysicsBackend;
import org.dynamisphysics.api.config.PhysicsWorldConfig;
import org.dynamisphysics.api.event.ContactEvent;
import org.dynamisphysics.api.event.PhysicsEvent;
import org.dynamisphysics.api.query.RaycastResult;
import org.dynamisphysics.api.world.PhysicsWorld;
import org.dynamisphysics.ode4j.Ode4jBackendRegistrar;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
import org.vectrix.core.Matrix4f;
import org.vectrix.core.Vector3f;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.Stream;

import static org.dynamisphysics.test.assertions.PhysicsAssertions.assertBodyFalling;
import static org.dynamisphysics.test.assertions.PhysicsAssertions.assertEventFired;
import static org.dynamisphysics.test.assertions.PhysicsAssertions.assertSnapshotNonEmpty;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

class BackendParityTest {

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
    void worldCreates(PhysicsBackend backend) {
        PhysicsWorld world = PhysicsWorldFactory.create(PhysicsWorldConfig.defaults(backend));
        assertNotNull(world);
        world.destroy();
    }

    @ParameterizedTest
    @MethodSource("backends")
    void sphereFallsOnGround(PhysicsBackend backend) {
        PhysicsWorld world = PhysicsWorldFactory.create(PhysicsWorldConfig.defaults(backend));
        try {
            spawnGround(world);
            RigidBodyHandle sphere = world.spawnRigidBody(
                RigidBodyConfig.builder(CollisionShape.sphere(0.5f), 1f)
                    .worldTransform(translation(0f, 4f, 0f))
                    .build()
            );
            BodyState before = world.getBodyState(sphere);
            for (int i = 0; i < 120; i++) {
                world.step(1f / 60f, 1);
            }
            BodyState after = world.getBodyState(sphere);
            assertBodyFalling(before, after);
        } finally {
            world.destroy();
        }
    }

    @ParameterizedTest
    @MethodSource("backends")
    void impulseChangesVelocity(PhysicsBackend backend) {
        PhysicsWorld world = PhysicsWorldFactory.create(PhysicsWorldConfig.defaults(backend));
        try {
            RigidBodyHandle sphere = world.spawnRigidBody(
                RigidBodyConfig.builder(CollisionShape.sphere(0.5f), 1f)
                    .worldTransform(translation(0f, 2f, 0f))
                    .build()
            );
            world.applyImpulse(sphere, new Vector3f(10f, 0f, 0f), new Vector3f());
            world.step(1f / 60f, 1);
            BodyState state = world.getBodyState(sphere);
            assertTrue(state.linearVelocity().x() > 0f);
        } finally {
            world.destroy();
        }
    }

    @ParameterizedTest
    @MethodSource("backends")
    void raycastFractionContract(PhysicsBackend backend) {
        PhysicsWorld world = PhysicsWorldFactory.create(PhysicsWorldConfig.defaults(backend));
        try {
            world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.box(2f, 0.1f, 2f), 0f)
                .mode(BodyMode.STATIC)
                .worldTransform(translation(0f, 0f, 0f))
                .build());

            Optional<RaycastResult> hit = world.raycastClosest(new Vector3f(0f, 10f, 0f), new Vector3f(0f, -1f, 0f),
                20f, -1);
            assertTrue(hit.isPresent());
            assertTrue(hit.get().fraction() >= 0f);
            assertTrue(hit.get().fraction() <= 1f);
        } finally {
            world.destroy();
        }
    }

    @ParameterizedTest
    @MethodSource("backends")
    void contactEventFiresOnCollision(PhysicsBackend backend) {
        PhysicsWorld world = PhysicsWorldFactory.create(PhysicsWorldConfig.defaults(backend));
        try {
            spawnGround(world);
            world.spawnRigidBody(
                RigidBodyConfig.builder(CollisionShape.sphere(0.5f), 1f)
                    .worldTransform(translation(0f, 1.1f, 0f))
                    .build()
            );
            List<PhysicsEvent> events = new ArrayList<>();
            for (int i = 0; i < 90; i++) {
                world.step(1f / 60f, 1);
                events.addAll(world.drainEvents());
            }
            assertEventFired(events, ContactEvent.class);
        } finally {
            world.destroy();
        }
    }

    @ParameterizedTest
    @MethodSource("backends")
    void snapshotRoundtripPreservesBodyCount(PhysicsBackend backend) {
        PhysicsWorld world = PhysicsWorldFactory.create(PhysicsWorldConfig.defaults(backend));
        try {
            spawnGround(world);
            world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.sphere(0.5f), 1f)
                .worldTransform(translation(0f, 2f, 0f)).build());
            world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.sphere(0.5f), 1f)
                .worldTransform(translation(1f, 2f, 0f)).build());
            byte[] snap = world.snapshot();
            assertSnapshotNonEmpty(snap);
            int countBefore = world.getStats().bodyCount();
            world.restore(snap);
            assertEquals(countBefore, world.getStats().bodyCount());
        } finally {
            world.destroy();
        }
    }

    private static void spawnGround(PhysicsWorld world) {
        world.spawnRigidBody(
            RigidBodyConfig.builder(CollisionShape.box(20f, 0.5f, 20f), 0f)
                .mode(BodyMode.STATIC)
                .worldTransform(translation(0f, -0.5f, 0f))
                .build()
        );
    }

    private static Matrix4f translation(float x, float y, float z) {
        return new Matrix4f().identity().translation(x, y, z);
    }
}
