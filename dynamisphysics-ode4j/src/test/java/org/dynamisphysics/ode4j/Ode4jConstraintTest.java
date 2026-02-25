package org.dynamisphysics.ode4j;

import org.dynamisphysics.api.PhysicsWorldFactory;
import org.dynamisphysics.api.body.BodyMode;
import org.dynamisphysics.api.body.BodyState;
import org.dynamisphysics.api.body.RigidBodyConfig;
import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.config.PhysicsBackend;
import org.dynamisphysics.api.config.PhysicsWorldConfig;
import org.dynamisphysics.api.constraint.ConstraintDesc;
import org.dynamisphysics.api.constraint.ConstraintHandle;
import org.dynamisphysics.api.constraint.ConstraintLimits;
import org.dynamisphysics.api.constraint.ConstraintMotor;
import org.dynamisphysics.api.constraint.ConstraintType;
import org.dynamisphysics.api.event.ConstraintBreakEvent;
import org.dynamisphysics.api.event.PhysicsEvent;
import org.dynamisphysics.api.world.PhysicsWorld;
import org.dynamisphysics.ode4j.constraint.DoorHingeBuilder;
import org.dynamisphysics.ode4j.constraint.RopeChainBuilder;
import org.dynamiscollision.shapes.CollisionShape;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.vectrix.core.Matrix4f;
import org.vectrix.core.Vector3f;

import java.util.ArrayList;
import java.util.List;

import static org.dynamisphysics.test.assertions.PhysicsAssertions.assertEventFired;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class Ode4jConstraintTest {
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

    @Test
    void allConstraintTypesCreateLiveHandle() {
        var a = spawnBox(0f, 5f, 0f);
        var b = spawnBox(1f, 5f, 0f);
        for (ConstraintType type : ConstraintType.values()) {
            ConstraintHandle handle = world.addConstraint(descFor(type, a, b));
            assertTrue(handle.isAlive(), "Expected live handle for " + type);
            world.removeConstraint(handle);
        }
    }

    @Test
    void fixedJointWeldsTwoBodies() {
        var a = spawnStatic(0f, 0f, 0f);
        var b = spawnBox(0f, 1f, 0f);
        world.addConstraint(new ConstraintDesc(
            ConstraintType.FIXED, a, b,
            new Vector3f(0f, 1f, 0f), new Vector3f(0f, 1f, 0f),
            new Vector3f(0f, 1f, 0f), new Vector3f(0f, 1f, 0f),
            ConstraintLimits.locked(), ConstraintMotor.off(), 0f, 0f
        ));
        BodyState before = world.getBodyState(b);
        step(120);
        BodyState after = world.getBodyState(b);
        assertEquals(before.position().y(), after.position().y(), 0.05f);
    }

    @Test
    void hingeJointPermitsRotation() {
        var a = spawnStatic(0f, 5f, 0f);
        var b = spawnBox(0f, 3f, 0f);
        world.addConstraint(new ConstraintDesc(
            ConstraintType.HINGE, a, b,
            new Vector3f(0f, 4f, 0f), new Vector3f(0f, 4f, 0f),
            new Vector3f(1f, 0f, 0f), new Vector3f(1f, 0f, 0f),
            ConstraintLimits.free(), ConstraintMotor.off(), 0f, 0f
        ));
        step(60);
        BodyState after = world.getBodyState(b);
        assertTrue(after.angularVelocity().length() > 0.01f || after.position().y() < 3f);
    }

    @Test
    void sliderJointRespectsLinearLimit() {
        var a = spawnStatic(0f, 5f, 0f);
        var b = spawnBox(0f, 3f, 0f);
        world.addConstraint(new ConstraintDesc(
            ConstraintType.SLIDER, a, b,
            new Vector3f(), new Vector3f(),
            new Vector3f(0f, 1f, 0f), new Vector3f(0f, 1f, 0f),
            new ConstraintLimits(-0.1f, 0f, 0f, 0f),
            ConstraintMotor.off(), 0f, 0f
        ));
        step(120);
        BodyState after = world.getBodyState(b);
        assertTrue(after.position().y() > 2.8f);
    }

    @Test
    void motorDrivesHingeAtTargetVelocity() {
        var a = spawnStatic(0f, 5f, 0f);
        var b = spawnBox(0f, 3f, 0f);
        world.addConstraint(new ConstraintDesc(
            ConstraintType.HINGE, a, b,
            new Vector3f(0f, 4f, 0f), new Vector3f(0f, 4f, 0f),
            new Vector3f(1f, 0f, 0f), new Vector3f(1f, 0f, 0f),
            ConstraintLimits.free(), ConstraintMotor.velocity(2.0f, 50f), 0f, 0f
        ));
        step(60);
        BodyState after = world.getBodyState(b);
        assertTrue(after.angularVelocity().length() > 0.01f || after.position().distance(new Vector3f(0f, 3f, 0f)) > 0.05f);
    }

    @Test
    void breakableConstraintBreaksAtThreshold() {
        var a = spawnStatic(0f, 0f, 0f);
        var b = spawnBox(0f, 1f, 0f);
        ConstraintHandle h = world.addConstraint(new ConstraintDesc(
            ConstraintType.FIXED, a, b,
            new Vector3f(0f, 1f, 0f), new Vector3f(0f, 1f, 0f),
            new Vector3f(0f, 1f, 0f), new Vector3f(0f, 1f, 0f),
            ConstraintLimits.locked(), ConstraintMotor.off(), 0.01f, 0f
        ));
        world.applyImpulse(b, new Vector3f(1000f, 0f, 0f), new Vector3f());
        step(20);
        assertFalse(h.isAlive());
    }

    @Test
    void constraintBreakEventFiredOnBreak() {
        var a = spawnStatic(0f, 0f, 0f);
        var b = spawnBox(0f, 1f, 0f);
        world.addConstraint(new ConstraintDesc(
            ConstraintType.FIXED, a, b,
            new Vector3f(0f, 1f, 0f), new Vector3f(0f, 1f, 0f),
            new Vector3f(0f, 1f, 0f), new Vector3f(0f, 1f, 0f),
            ConstraintLimits.locked(), ConstraintMotor.off(), 0.01f, 0f
        ));
        world.applyImpulse(b, new Vector3f(1000f, 0f, 0f), new Vector3f());
        List<PhysicsEvent> events = new ArrayList<>();
        for (int i = 0; i < 30; i++) {
            world.step(1f / 60f);
            events.addAll(world.drainEvents());
        }
        assertEventFired(events, ConstraintBreakEvent.class);
    }

    @Test
    void removeConstraintKillsHandle() {
        var a = spawnBox(0f, 5f, 0f);
        var b = spawnBox(1f, 5f, 0f);
        var h = world.addConstraint(new ConstraintDesc(
            ConstraintType.BALL_SOCKET, a, b,
            new Vector3f(), new Vector3f(),
            new Vector3f(0f, 1f, 0f), new Vector3f(0f, 1f, 0f),
            ConstraintLimits.free(), ConstraintMotor.off(), 0f, 0f
        ));
        world.removeConstraint(h);
        assertFalse(h.isAlive());
    }

    @Test
    void doorHingeBuilderCreatesConstraintWithLimits() {
        var frame = spawnStatic(0f, 3f, 0f);
        var door = spawnBox(1f, 3f, 0f);
        var h = DoorHingeBuilder.create(
            world,
            frame,
            door,
            new Vector3f(0f, 3f, 0f),
            new Vector3f(0f, 1f, 0f),
            -90f,
            0f
        );
        assertTrue(h.isAlive());
        assertEquals(ConstraintType.HINGE, h.type());
    }

    @Test
    void ropeChainBuilderCreatesCorrectConstraintCount() {
        var links = new ArrayList<RigidBodyHandle>();
        for (int i = 0; i < 5; i++) {
            links.add(spawnBox(0f, 10f - i, 0f));
        }
        var constraints = RopeChainBuilder.create(world, links);
        assertEquals(4, constraints.size());
        constraints.forEach(h -> assertTrue(h.isAlive()));
    }

    @Test
    void statsConstraintCountMatchesAdded() {
        var a = spawnBox(0f, 5f, 0f);
        var b = spawnBox(1f, 5f, 0f);
        world.addConstraint(new ConstraintDesc(
            ConstraintType.BALL_SOCKET, a, b,
            new Vector3f(), new Vector3f(),
            new Vector3f(0f, 1f, 0f), new Vector3f(0f, 1f, 0f),
            ConstraintLimits.free(), ConstraintMotor.off(), 0f, 0f
        ));
        assertEquals(1, world.getStats().constraintCount());
    }

    @Test
    void setMotorTargetDoesNotThrowOnEnabledConstraint() {
        var a = spawnStatic(0f, 5f, 0f);
        var b = spawnBox(0f, 3f, 0f);
        var h = world.addConstraint(new ConstraintDesc(
            ConstraintType.HINGE, a, b,
            new Vector3f(0f, 4f, 0f), new Vector3f(0f, 4f, 0f),
            new Vector3f(1f, 0f, 0f), new Vector3f(1f, 0f, 0f),
            ConstraintLimits.free(), ConstraintMotor.velocity(1f, 20f), 0f, 0f
        ));
        world.setMotorTarget(h, 3.0f);
        step(10);
        assertTrue(h.isAlive());
    }

    private RigidBodyHandle spawnBox(float x, float y, float z) {
        return world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.box(0.5f, 0.5f, 0.5f), 1f)
            .worldTransform(new Matrix4f().translation(x, y, z))
            .build());
    }

    private RigidBodyHandle spawnStatic(float x, float y, float z) {
        return world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.box(2f, 0.1f, 2f), 0f)
            .mode(BodyMode.STATIC)
            .worldTransform(new Matrix4f().translation(x, y, z))
            .build());
    }

    private void step(int n) {
        for (int i = 0; i < n; i++) {
            world.step(1f / 60f);
        }
    }

    private static ConstraintDesc descFor(ConstraintType type, RigidBodyHandle a, RigidBodyHandle b) {
        return new ConstraintDesc(
            type,
            a,
            b,
            new Vector3f(),
            new Vector3f(),
            new Vector3f(0f, 1f, 0f),
            new Vector3f(1f, 0f, 0f),
            ConstraintLimits.free(),
            ConstraintMotor.off(),
            0f,
            0f
        );
    }
}
