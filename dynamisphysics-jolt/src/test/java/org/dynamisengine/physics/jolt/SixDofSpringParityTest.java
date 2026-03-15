package org.dynamisengine.physics.jolt;

import org.dynamisengine.collision.shapes.CollisionShape;
import org.dynamisengine.physics.api.PhysicsWorldFactory;
import org.dynamisengine.physics.api.body.BodyMode;
import org.dynamisengine.physics.api.body.BodyState;
import org.dynamisengine.physics.api.body.RigidBodyConfig;
import org.dynamisengine.physics.api.body.RigidBodyHandle;
import org.dynamisengine.physics.api.config.BroadphaseType;
import org.dynamisengine.physics.api.config.PhysicsBackend;
import org.dynamisengine.physics.api.config.PhysicsWorldConfig;
import org.dynamisengine.physics.api.constraint.ConstraintDesc;
import org.dynamisengine.physics.api.constraint.ConstraintLimits;
import org.dynamisengine.physics.api.constraint.ConstraintMotor;
import org.dynamisengine.physics.api.constraint.ConstraintType;
import org.dynamisengine.physics.api.world.PhysicsWorld;
import org.dynamisengine.physics.ode4j.Ode4jBackendRegistrar;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
import org.dynamisengine.vectrix.core.Matrix4f;
import org.dynamisengine.vectrix.core.Vector3f;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.assertTrue;

class SixDofSpringParityTest {
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
    void springResponseSettles(PhysicsBackend backend) {
        PhysicsWorld world = createWorld(backend);
        try {
            RigidBodyHandle anchor = spawnStatic(world, 0f, 0f, 0f);
            RigidBodyHandle body = spawnDynamic(world, 0f, 2f, 0f);
            addSpring(world, anchor, body, 120f, 4f);
            float restY = world.getBodyState(body).position().y();
            world.applyImpulse(body, new Vector3f(0f, 6f, 0f), new Vector3f());

            List<Float> ys = new ArrayList<>();
            for (int i = 0; i < 600; i++) {
                world.step(1f / 60f, 1);
                ys.add(world.getBodyState(body).position().y() - restY);
            }

            float earlyPeak = 0f;
            float latePeak = 0f;
            for (int i = 0; i < 200; i++) {
                earlyPeak = java.lang.Math.max(earlyPeak, java.lang.Math.abs(ys.get(i)));
            }
            for (int i = ys.size() - 200; i < ys.size(); i++) {
                latePeak = java.lang.Math.max(latePeak, java.lang.Math.abs(ys.get(i)));
            }
            float finalAbs = java.lang.Math.abs(ys.get(ys.size() - 1));
            assertTrue(latePeak <= earlyPeak * 1.05f,
                "Expected bounded/settling response for " + backend + " early=" + earlyPeak + " late=" + latePeak);
            assertTrue(finalAbs <= earlyPeak,
                "Expected eventual settling for " + backend + " finalAbs=" + finalAbs + " early=" + earlyPeak);
        } finally {
            world.destroy();
        }
    }

    @ParameterizedTest
    @MethodSource("backends")
    void nearCriticalDampingConvergesWithoutLargeOvershoot(PhysicsBackend backend) {
        PhysicsWorld world = createWorld(backend);
        try {
            RigidBodyHandle anchor = spawnStatic(world, 0f, 0f, 0f);
            RigidBodyHandle body = spawnDynamic(world, 0f, 2f, 0f);
            addSpring(world, anchor, body, 120f, 22f);
            float restY = world.getBodyState(body).position().y();
            world.applyImpulse(body, new Vector3f(0f, 6f, 0f), new Vector3f());

            float minY = Float.MAX_VALUE;
            for (int i = 0; i < 360; i++) {
                world.step(1f / 60f, 1);
                BodyState state = world.getBodyState(body);
                minY = java.lang.Math.min(minY, state.position().y() - restY);
            }
            BodyState finalState = world.getBodyState(body);

            assertTrue(minY > -0.25f, "Expected limited overshoot for " + backend + ", minY=" + minY);
            assertTrue(java.lang.Math.abs(finalState.position().y() - restY) < 0.35f,
                "Expected convergence near rest for " + backend);
        } finally {
            world.destroy();
        }
    }

    private static PhysicsWorld createWorld(PhysicsBackend backend) {
        return PhysicsWorldFactory.create(new PhysicsWorldConfig(
            backend,
            new Vector3f(0f, 0f, 0f),
            1f / 60f,
            1,
            20,
            65_536,
            16_384,
            BroadphaseType.BVH,
            true
        ));
    }

    private static void addSpring(PhysicsWorld world, RigidBodyHandle a, RigidBodyHandle b, float k, float c) {
        world.addConstraint(new ConstraintDesc(
            ConstraintType.SIX_DOF_SPRING,
            a,
            b,
            new Vector3f(),
            new Vector3f(),
            new Vector3f(0f, 1f, 0f),
            new Vector3f(1f, 0f, 0f),
            new ConstraintLimits(-10f, 10f, -1f, 1f),
            new ConstraintMotor(true, 0f, 0f, k, c),
            0f,
            0f
        ));
    }

    private static RigidBodyHandle spawnDynamic(PhysicsWorld world, float x, float y, float z) {
        return world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.box(0.25f, 0.25f, 0.25f), 1f)
            .worldTransform(new Matrix4f().translation(x, y, z))
            .build());
    }

    private static RigidBodyHandle spawnStatic(PhysicsWorld world, float x, float y, float z) {
        return world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.box(0.5f, 0.5f, 0.5f), 0f)
            .mode(BodyMode.STATIC)
            .worldTransform(new Matrix4f().translation(x, y, z))
            .build());
    }
}
