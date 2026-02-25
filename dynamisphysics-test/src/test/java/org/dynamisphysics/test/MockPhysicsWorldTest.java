package org.dynamisphysics.test;

import org.dynamisphysics.api.body.BodyMode;
import org.dynamisphysics.api.body.BodyState;
import org.dynamisphysics.api.body.RigidBodyConfig;
import org.dynamisphysics.api.constraint.ConstraintDesc;
import org.dynamisphysics.api.constraint.ConstraintLimits;
import org.dynamisphysics.api.constraint.ConstraintMotor;
import org.dynamisphysics.api.constraint.ConstraintType;
import org.dynamisphysics.api.event.SleepEvent;
import org.dynamisphysics.test.mock.MockPhysicsWorld;
import org.dynamiscollision.shapes.CollisionShape;
import org.vectrix.core.Matrix4f;
import org.vectrix.core.Vector3f;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.dynamisphysics.test.assertions.PhysicsAssertions.assertContactFired;
import static org.dynamisphysics.test.assertions.PhysicsAssertions.assertEventFired;
import static org.dynamisphysics.test.assertions.PhysicsAssertions.assertPositionNear;
import static org.dynamisphysics.test.assertions.PhysicsAssertions.assertStatsBodyCount;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class MockPhysicsWorldTest {
    private MockPhysicsWorld world;

    @BeforeEach
    void setUp() {
        world = new MockPhysicsWorld();
    }

    @Test void stepIncreasesStepCount() { world.step(1f / 60f); world.step(1f / 60f); assertEquals(2, world.stepCount()); }
    @Test void stepWithSubstepsCountsEachSubstep() { world.step(1f / 60f, 4); assertEquals(4, world.stepCount()); }
    @Test void pauseStopsStepCounting() { world.pause(); world.step(1f / 60f); assertEquals(0, world.stepCount()); }
    @Test void resumeRestoresStepCounting() { world.pause(); world.resume(); world.step(1f / 60f); assertEquals(1, world.stepCount()); }

    @Test
    void spawnBodyTrackedInCount() {
        world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.sphere(1f), 1f).build());
        assertEquals(1, world.spawnedBodyCount());
    }

    @Test
    void spawnBodyHandleIsAlive() {
        var h = world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.sphere(1f), 1f).build());
        assertTrue(h.isAlive());
    }

    @Test
    void destroyBodyKillsHandle() {
        var h = world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.sphere(1f), 1f).build());
        world.destroyRigidBody(h);
        assertFalse(h.isAlive());
    }

    @Test
    void destroyBodyRemovesFromWorld() {
        var h = world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.sphere(1f), 1f).build());
        world.destroyRigidBody(h);
        assertFalse(world.hasBody(h));
    }

    @Test
    void getBodyStateReturnsPositionFromTransform() {
        var h = world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.sphere(1f), 1f)
            .worldTransform(new Matrix4f().translation(3f, 7f, 0f))
            .build());
        BodyState s = world.getBodyState(h);
        assertPositionNear(s, new Vector3f(3f, 7f, 0f), 0.01f);
    }

    @Test
    void injectEventAppearsInDrainedList() {
        var h = world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.sphere(1f), 1f).build());
        world.injectEvent(new SleepEvent(h));
        var events = world.drainEvents();
        assertEventFired(events, SleepEvent.class);
    }

    @Test
    void drainClearsQueue() {
        var h = world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.sphere(1f), 1f).build());
        world.injectEvent(new SleepEvent(h));
        world.drainEvents();
        assertEquals(0, world.drainEvents().size());
    }

    @Test
    void injectContactProducesContactEvent() {
        var a = world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.sphere(1f), 1f).build());
        var b = world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.sphere(1f), 1f).build());
        world.injectContact(a, b, new Vector3f(0f, 0f, 0f), 5f);
        assertContactFired(world.drainEvents(), a, b);
    }

    @Test
    void addConstraintReturnsLiveHandle() {
        var a = world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.sphere(1f), 1f).build());
        var b = world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.sphere(1f), 1f).build());
        var ch = world.addConstraint(new ConstraintDesc(
            ConstraintType.FIXED, a, b,
            new Vector3f(), new Vector3f(),
            new Vector3f(0f, 1f, 0f), new Vector3f(0f, 1f, 0f),
            ConstraintLimits.locked(), ConstraintMotor.off(), 0f, 0f));
        assertTrue(ch.isAlive());
    }

    @Test
    void removeConstraintKillsHandle() {
        var a = world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.sphere(1f), 1f).build());
        var b = world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.sphere(1f), 1f).build());
        var ch = world.addConstraint(new ConstraintDesc(
            ConstraintType.FIXED, a, b,
            new Vector3f(), new Vector3f(),
            new Vector3f(0f, 1f, 0f), new Vector3f(0f, 1f, 0f),
            ConstraintLimits.locked(), ConstraintMotor.off(), 0f, 0f));
        world.removeConstraint(ch);
        assertFalse(ch.isAlive());
    }

    @Test
    void applyImpulseTracked() {
        var h = world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.sphere(1f), 1f).build());
        world.applyImpulse(h, new Vector3f(1f, 0f, 0f), new Vector3f());
        assertEquals(1, world.applyImpulseCount());
    }

    @Test
    void statsBodyCountMatchesSpawned() {
        world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.sphere(1f), 1f).build());
        world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.sphere(1f), 1f).build());
        assertStatsBodyCount(world.getStats(), 2);
    }
}
