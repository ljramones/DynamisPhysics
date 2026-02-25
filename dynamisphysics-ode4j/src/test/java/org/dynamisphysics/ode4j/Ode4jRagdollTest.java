package org.dynamisphysics.ode4j;

import org.dynamiscollision.shapes.CollisionShape;
import org.dynamisphysics.api.AnimisPose;
import org.dynamisphysics.api.PhysicsWorldFactory;
import org.dynamisphysics.api.RagdollBoneDesc;
import org.dynamisphysics.api.RagdollDescriptor;
import org.dynamisphysics.api.RagdollHandle;
import org.dynamisphysics.api.RagdollJointDesc;
import org.dynamisphysics.api.body.BodyState;
import org.dynamisphysics.api.config.PhysicsBackend;
import org.dynamisphysics.api.config.PhysicsWorldConfig;
import org.dynamisphysics.api.constraint.ConstraintLimits;
import org.dynamisphysics.api.constraint.ConstraintType;
import org.dynamisphysics.api.material.PhysicsMaterial;
import org.dynamisphysics.api.world.GetUpPoseHint;
import org.dynamisphysics.api.world.PhysicsWorld;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.vectrix.core.Matrix4f;
import org.vectrix.core.Quaternionf;
import org.vectrix.core.Vector3f;

import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;

import static org.dynamisphysics.test.assertions.PhysicsAssertions.assertBodyFalling;
import static org.dynamisphysics.test.assertions.PhysicsAssertions.assertPositionNear;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class Ode4jRagdollTest {
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
    void ragdollSpawnCreatesOneBodyPerBone() {
        RagdollHandle h = world.spawnRagdoll(simpleDescriptor(null), simplePose(0f));
        assertTrue(h.isAlive());
        assertTrue(world.getStats().bodyCount() >= 2);
    }

    @Test
    void ragdollAtAlphaZeroTracksAnimisPoseExactly() {
        RagdollHandle h = world.spawnRagdoll(simpleDescriptor(null), simplePose(0f));
        AnimisPose target = simplePose(3f);
        world.setRagdollBlendTarget(h, target, 0f);

        step(5);

        BodyState root = h.getBoneState("root");
        assertPositionNear(root, new Vector3f(0f, 3f, 0f), 0.05f);
    }

    @Test
    void ragdollAtAlphaOneFallsUnderGravity() {
        RagdollHandle h = world.spawnRagdoll(simpleDescriptor(null), simplePose(5f));
        world.activateRagdoll(h, 0f);

        BodyState before = h.getBoneState("root");
        step(60);
        BodyState after = h.getBoneState("root");

        assertBodyFalling(before, after);
    }

    @Test
    void pdControllerTorqueDirectedTowardTarget() {
        RagdollHandle h = world.spawnRagdoll(simpleDescriptor(null), rotatedPose(0f, 0f));
        world.setRagdollBlendTarget(h, rotatedPose(0f, 1.2f), 1f);

        step(20);

        BodyState root = h.getBoneState("root");
        assertTrue(root.angularVelocity().length() > 0.01f, "Expected PD controller to generate angular velocity");
    }

    @Test
    void deactivateRagdollRestoresKinematicTracking() {
        RagdollHandle h = world.spawnRagdoll(simpleDescriptor(null), simplePose(2f));
        world.activateRagdoll(h, 0f);
        step(30);

        world.deactivateRagdoll(h);
        world.setRagdollBlendTarget(h, simplePose(4f), 0f);
        step(5);

        assertPositionNear(h.getBoneState("root"), new Vector3f(0f, 4f, 0f), 0.05f);
    }

    @Test
    void getUpPoseHintEmittedWhenAllBonesAtRest() {
        AtomicInteger hints = new AtomicInteger();
        world.setGravity(new Vector3f(0f, 0f, 0f));
        RagdollHandle h = world.spawnRagdoll(simpleDescriptor(hint -> hints.incrementAndGet()), simplePose(1f));
        world.activateRagdoll(h, 0f);

        step(20);

        assertTrue(hints.get() > 0, "Expected get-up hint emission at rest");
    }

    @Test
    void blendAlphaHalfProducesPartialTorque() {
        RagdollHandle hHalf = world.spawnRagdoll(simpleDescriptor(null), rotatedPose(0f, 0f));
        world.setRagdollBlendTarget(hHalf, rotatedPose(0f, 1.2f), 0.5f);
        step(20);
        float halfAng = hHalf.getBoneState("root").angularVelocity().length();

        world.destroyRagdoll(hHalf);

        RagdollHandle hFull = world.spawnRagdoll(simpleDescriptor(null), rotatedPose(0f, 0f));
        world.setRagdollBlendTarget(hFull, rotatedPose(0f, 1.2f), 1f);
        step(20);
        float fullAng = hFull.getBoneState("root").angularVelocity().length();

        assertTrue(halfAng > 0.001f, "Expected non-zero angular response at alpha=0.5");
        assertTrue(fullAng > 0.001f, "Expected non-zero angular response at alpha=1");
        assertTrue(Math.abs(fullAng - halfAng) > 1e-4f, "Expected blend alpha to change response magnitude");
    }

    @Test
    void angularErrorZeroWhenOrientationsMatch() {
        Quaternionf q = new Quaternionf();
        Vector3f error = Quaternionf.angularError(q, q, new Vector3f());
        assertTrue(error.length() < 1e-6f);
    }

    @Test
    void destroyRagdollKillsHandle() {
        RagdollHandle h = world.spawnRagdoll(simpleDescriptor(null), simplePose(0f));
        world.destroyRagdoll(h);
        assertFalse(h.isAlive());
    }

    private void step(int steps) {
        for (int i = 0; i < steps; i++) {
            world.step(1f / 60f);
            world.drainEvents();
        }
    }

    private static AnimisPose simplePose(float y) {
        return new AnimisPose(Map.of(
            "root", new Matrix4f().identity().translation(0f, y, 0f),
            "spine", new Matrix4f().identity().translation(0f, y + 1f, 0f)
        ));
    }

    private static AnimisPose rotatedPose(float y, float yawRad) {
        Matrix4f root = new Matrix4f().identity().translation(0f, y, 0f).rotateY(yawRad);
        Matrix4f spine = new Matrix4f().identity().translation(0f, y + 1f, 0f).rotateY(yawRad);
        return new AnimisPose(Map.of("root", root, "spine", spine));
    }

    private static RagdollDescriptor simpleDescriptor(java.util.function.Consumer<GetUpPoseHint> hintListener) {
        return new RagdollDescriptor(
            List.of(
                new RagdollBoneDesc("root", CollisionShape.capsule(0.2f, 0.6f), 10f, new Vector3f(), 80f, 8f, 60f),
                new RagdollBoneDesc("spine", CollisionShape.capsule(0.18f, 0.5f), 8f, new Vector3f(), 70f, 7f, 50f)
            ),
            List.of(
                new RagdollJointDesc("root", "spine", ConstraintType.BALL_SOCKET, ConstraintLimits.free())
            ),
            18f,
            hintListener
        );
    }
}
