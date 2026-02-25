package org.dynamisphysics.ode4j;

import org.dynamisphysics.api.PhysicsWorldFactory;
import org.dynamisphysics.api.body.BodyMode;
import org.dynamisphysics.api.body.BodyState;
import org.dynamisphysics.api.body.RigidBodyConfig;
import org.dynamisphysics.api.config.PhysicsBackend;
import org.dynamisphysics.api.config.PhysicsWorldConfig;
import org.dynamisphysics.api.event.ContactEvent;
import org.dynamisphysics.api.event.PhysicsEvent;
import org.dynamisphysics.api.world.PhysicsWorld;
import org.dynamiscollision.shapes.CollisionShape;
import org.vectrix.core.Matrix4f;
import org.vectrix.core.Quaternionf;
import org.vectrix.core.Vector3f;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.dynamisphysics.test.assertions.PhysicsAssertions.assertAlive;
import static org.dynamisphysics.test.assertions.PhysicsAssertions.assertBodyFalling;
import static org.dynamisphysics.test.assertions.PhysicsAssertions.assertDead;
import static org.dynamisphysics.test.assertions.PhysicsAssertions.assertEventFired;
import static org.dynamisphysics.test.assertions.PhysicsAssertions.assertStatsBodyCount;
import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

class Ode4jCoreTest {
    private PhysicsWorld world;

    @BeforeAll
    static void registerBackend() {
        PhysicsWorldFactory.register(PhysicsBackend.ODE4J, Ode4jPhysicsWorld::create);
    }

    @BeforeEach
    void setUp() {
        world = PhysicsWorldFactory.create(PhysicsWorldConfig.defaults(PhysicsBackend.ODE4J));
    }

    @AfterEach
    void tearDown() {
        world.destroy();
    }

    @Test void worldCreatesWithoutError() { assertNotNull(world); }

    @Test
    void sphereBodySpawnReturnsLiveHandle() {
        var h = world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.sphere(0.5f), 1f).build());
        assertAlive(h);
    }

    @Test
    void boxBodySpawnReturnsLiveHandle() {
        var h = world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.box(1f, 1f, 1f), 1f).build());
        assertAlive(h);
    }

    @Test
    void capsuleBodySpawnReturnsLiveHandle() {
        var h = world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.capsule(0.3f, 1f), 1f).build());
        assertAlive(h);
    }

    @Test
    void cylinderBodySpawnReturnsLiveHandle() {
        var h = world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.cylinder(0.3f, 1f), 1f).build());
        assertAlive(h);
    }

    @Test
    void staticBodyDoesNotMoveUnderGravity() {
        var h = world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.box(10f, 0.1f, 10f), 0f)
            .mode(BodyMode.STATIC)
            .worldTransform(new Matrix4f().translation(0f, 0f, 0f))
            .build());
        BodyState before = world.getBodyState(h);
        for (int i = 0; i < 60; i++) world.step(1f / 60f, 1);
        BodyState after = world.getBodyState(h);
        assertEquals(before.position().y(), after.position().y(), 0.001f, "Static body moved");
    }

    @Test
    void dynamicBodyFallsUnderGravity() {
        var h = world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.sphere(0.5f), 1f)
            .worldTransform(new Matrix4f().translation(0f, 10f, 0f))
            .build());
        BodyState before = world.getBodyState(h);
        for (int i = 0; i < 60; i++) world.step(1f / 60f, 1);
        BodyState after = world.getBodyState(h);
        assertBodyFalling(before, after);
    }

    @Test
    void impulseChangesLinearVelocity() {
        var h = world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.sphere(0.5f), 1f)
            .worldTransform(new Matrix4f().translation(0f, 5f, 0f))
            .build());
        world.applyImpulse(h, new Vector3f(10f, 0f, 0f), new Vector3f());
        world.step(1f / 60f, 1);
        BodyState state = world.getBodyState(h);
        assertTrue(state.linearVelocity().x() > 0f, "Impulse did not produce positive X velocity");
    }

    @Test
    void torqueChangesAngularVelocity() {
        var h = world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.box(1f, 1f, 1f), 1f)
            .worldTransform(new Matrix4f().translation(0f, 5f, 0f))
            .build());
        world.applyTorque(h, new Vector3f(0f, 10f, 0f));
        world.step(1f / 60f, 1);
        BodyState state = world.getBodyState(h);
        assertTrue(Math.abs(state.angularVelocity().y()) > 0f, "Torque did not produce angular velocity");
    }

    @Test
    void dynamicCompoundBodySupportsTorqueAndCollision() {
        world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.box(30f, 0.1f, 30f), 0f)
            .mode(BodyMode.STATIC)
            .worldTransform(new Matrix4f().identity().translation(0f, -0.1f, 0f))
            .build());

        CollisionShape compound = CollisionShape.compound(
            List.of(CollisionShape.sphere(0.4f), CollisionShape.sphere(0.4f)),
            List.of(
                new org.vectrix.affine.Transformf(
                    new Vector3f(-0.8f, 0f, 0f),
                    new Quaternionf(0f, 0f, 0f, 1f),
                    new Vector3f(1f, 1f, 1f)
                ),
                new org.vectrix.affine.Transformf(
                    new Vector3f(0.8f, 0f, 0f),
                    new Quaternionf(0f, 0f, 0f, 1f),
                    new Vector3f(1f, 1f, 1f)
                )
            )
        );

        var h = world.spawnRigidBody(RigidBodyConfig.builder(compound, 2f)
            .worldTransform(new Matrix4f().identity().translation(0f, 5f, 0f))
            .build());
        world.applyTorque(h, new Vector3f(0f, 8f, 0f));
        for (int i = 0; i < 120; i++) world.step(1f / 60f, 1);
        BodyState state = world.getBodyState(h);
        assertTrue(state.position().y() < 5f, "Compound did not fall");
        assertTrue(Math.abs(state.angularVelocity().y()) > 0f, "Compound torque had no effect");
    }

    @Test
    void destroyedBodyHandleIsNotAlive() {
        var h = world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.sphere(0.5f), 1f).build());
        world.destroyRigidBody(h);
        assertDead(h);
    }

    @Test
    void stepDoesNotThrow() {
        world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.sphere(0.5f), 1f).build());
        assertDoesNotThrow(() -> {
            for (int i = 0; i < 120; i++) world.step(1f / 60f);
        });
    }

    @Test
    void contactEventFiredOnSphereVsPlaneCollision() {
        world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.planeY(), 0f).mode(BodyMode.STATIC).build());
        world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.sphere(0.5f), 1f)
            .worldTransform(new Matrix4f().translation(0f, 0.4f, 0f))
            .build());
        List<PhysicsEvent> allEvents = new ArrayList<>();
        for (int i = 0; i < 30; i++) {
            world.step(1f / 60f);
            allEvents.addAll(world.drainEvents());
        }
        assertEventFired(allEvents, ContactEvent.class);
    }

    @Test
    void statsBodyCountMatchesSpawned() {
        world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.sphere(0.5f), 1f).build());
        world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.sphere(0.5f), 1f).build());
        assertStatsBodyCount(world.getStats(), 2);
    }
}
