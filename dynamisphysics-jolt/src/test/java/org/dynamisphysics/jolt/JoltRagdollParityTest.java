package org.dynamisphysics.jolt;

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
import org.dynamisphysics.api.world.GetUpPoseHint;
import org.dynamisphysics.api.world.PhysicsWorld;
import org.dynamisphysics.ode4j.Ode4jBackendRegistrar;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
import org.vectrix.core.Matrix4f;
import org.vectrix.core.Vector3f;

import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class JoltRagdollParityTest {

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
    void ragdollSpawnCreatesOneBodyPerBone(PhysicsBackend backend) {
        PhysicsWorld world = PhysicsWorldFactory.create(PhysicsWorldConfig.defaults(backend));
        try {
            RagdollHandle h = world.spawnRagdoll(simpleDescriptor(null), simplePose(0f));
            assertTrue(h.isAlive());
            assertTrue(world.getStats().bodyCount() >= 2);
        } finally {
            world.destroy();
        }
    }

    @ParameterizedTest
    @MethodSource("backends")
    void ragdollAtAlphaZeroTracksAnimisPose(PhysicsBackend backend) {
        PhysicsWorld world = PhysicsWorldFactory.create(PhysicsWorldConfig.defaults(backend));
        try {
            RagdollHandle h = world.spawnRagdoll(simpleDescriptor(null), simplePose(0f));
            world.setRagdollBlendTarget(h, simplePose(3f), 0f);
            step(world, 5);
            BodyState root = h.getBoneState("root");
            assertTrue(java.lang.Math.abs(root.position().y() - 3f) < 0.1f);
        } finally {
            world.destroy();
        }
    }

    @ParameterizedTest
    @MethodSource("backends")
    void ragdollAtAlphaOneFallsUnderGravity(PhysicsBackend backend) {
        PhysicsWorld world = PhysicsWorldFactory.create(PhysicsWorldConfig.defaults(backend));
        try {
            RagdollHandle h = world.spawnRagdoll(simpleDescriptor(null), simplePose(5f));
            world.activateRagdoll(h, 0f);
            float beforeY = h.getBoneState("root").position().y();
            step(world, 60);
            float afterY = h.getBoneState("root").position().y();
            assertTrue(afterY < beforeY - 0.05f, "beforeY=" + beforeY + " afterY=" + afterY);
        } finally {
            world.destroy();
        }
    }

    @ParameterizedTest
    @MethodSource("backends")
    void pdControllerGeneratesAngularResponse(PhysicsBackend backend) {
        PhysicsWorld world = PhysicsWorldFactory.create(PhysicsWorldConfig.defaults(backend));
        try {
            RagdollHandle h = world.spawnRagdoll(simpleDescriptor(null), rotatedPose(0f, 0f));
            world.setRagdollBlendTarget(h, rotatedPose(0f, 1.2f), 1f);
            step(world, 20);
            assertTrue(h.getBoneState("root").angularVelocity().length() > 0.01f);
        } finally {
            world.destroy();
        }
    }

    @ParameterizedTest
    @MethodSource("backends")
    void getUpPoseHintEmittedAtRest(PhysicsBackend backend) {
        PhysicsWorld world = PhysicsWorldFactory.create(PhysicsWorldConfig.defaults(backend));
        try {
            AtomicInteger hints = new AtomicInteger();
            world.setGravity(new Vector3f(0f, 0f, 0f));
            RagdollHandle h = world.spawnRagdoll(simpleDescriptor(hint -> hints.incrementAndGet()), simplePose(1f));
            world.activateRagdoll(h, 0f);
            step(world, 40);
            assertTrue(hints.get() > 0, "expected get-up hint");
        } finally {
            world.destroy();
        }
    }

    @ParameterizedTest
    @MethodSource("backends")
    void destroyRagdollKillsHandle(PhysicsBackend backend) {
        PhysicsWorld world = PhysicsWorldFactory.create(PhysicsWorldConfig.defaults(backend));
        try {
            RagdollHandle h = world.spawnRagdoll(simpleDescriptor(null), simplePose(0f));
            world.destroyRagdoll(h);
            assertFalse(h.isAlive());
        } finally {
            world.destroy();
        }
    }

    private static void step(PhysicsWorld world, int steps) {
        for (int i = 0; i < steps; i++) {
            world.step(1f / 60f, 1);
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

