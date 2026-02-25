package org.dynamisphysics.jolt;

import org.dynamiscollision.shapes.CollisionShape;
import org.dynamisphysics.api.PhysicsWorldFactory;
import org.dynamisphysics.api.body.BodyState;
import org.dynamisphysics.api.body.RigidBodyConfig;
import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.config.BroadphaseType;
import org.dynamisphysics.api.config.PhysicsBackend;
import org.dynamisphysics.api.config.PhysicsWorldConfig;
import org.dynamisphysics.api.constraint.MechanicalConstraintBuilders;
import org.dynamisphysics.api.world.PhysicsWorld;
import org.dynamisphysics.ode4j.Ode4jBackendRegistrar;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
import org.vectrix.core.Matrix4f;
import org.vectrix.core.Vector3f;

import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.assertTrue;

class MechanicalConstraintsParityTest {
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
    void gearCouplingTracksRatio(PhysicsBackend backend) {
        PhysicsWorld world = createWorld(backend);
        try {
            RigidBodyHandle a = spawnBody(world, 0f, 0f, 0f);
            RigidBodyHandle b = spawnBody(world, 0f, 2f, 0f);
            float ratio = 2f;
            world.addConstraint(MechanicalConstraintBuilders.gear(
                a, b, new Vector3f(0f, 1f, 0f), new Vector3f(0f, 1f, 0f), ratio
            ));
            world.setVelocity(a, new Vector3f(), new Vector3f(0f, 8f, 0f));
            world.setVelocity(b, new Vector3f(), new Vector3f());
            step(world, 180);

            float wA = world.getBodyState(a).angularVelocity().y();
            float wB = world.getBodyState(b).angularVelocity().y();
            assertTrue(java.lang.Math.abs(wA + ratio * wB) < 0.2f,
                "Expected gear coupling for " + backend + " got wA=" + wA + " wB=" + wB);
        } finally {
            world.destroy();
        }
    }

    @ParameterizedTest
    @MethodSource("backends")
    void rackPinionCouplesLinearAndAngularVelocity(PhysicsBackend backend) {
        PhysicsWorld world = createWorld(backend);
        try {
            RigidBodyHandle rack = spawnBody(world, 0f, 0f, 0f);
            RigidBodyHandle pinion = spawnBody(world, 0f, 2f, 0f);
            float ratio = 0.5f;
            world.addConstraint(MechanicalConstraintBuilders.rackPinion(
                rack, pinion, new Vector3f(1f, 0f, 0f), new Vector3f(0f, 1f, 0f), ratio
            ));
            world.setVelocity(rack, new Vector3f(4f, 0f, 0f), new Vector3f());
            world.setVelocity(pinion, new Vector3f(), new Vector3f());
            step(world, 180);

            BodyState rackState = world.getBodyState(rack);
            BodyState pinionState = world.getBodyState(pinion);
            float v = rackState.linearVelocity().x();
            float w = pinionState.angularVelocity().y();
            assertTrue(java.lang.Math.abs(v - ratio * w) < 0.25f,
                "Expected rack/pinion coupling for " + backend + " got v=" + v + " w=" + w);
        } finally {
            world.destroy();
        }
    }

    @ParameterizedTest
    @MethodSource("backends")
    void pulleyMaintainsLengthConstraint(PhysicsBackend backend) {
        PhysicsWorld world = createWorld(backend);
        try {
            RigidBodyHandle a = spawnBody(world, 0f, 2f, 0f);
            RigidBodyHandle b = spawnBody(world, 2f, 2f, 0f);
            Vector3f anchorA = new Vector3f(0f, 0f, 0f);
            Vector3f anchorB = new Vector3f(2f, 0f, 0f);
            float ropeLength = 4f;
            world.addConstraint(MechanicalConstraintBuilders.pulley(
                a, b, anchorA, anchorB, new Vector3f(0f, 1f, 0f), new Vector3f(0f, 1f, 0f), ropeLength, 1f
            ));

            world.applyImpulse(a, new Vector3f(0f, 5f, 0f), new Vector3f());
            step(world, 180);

            float dA = world.getBodyState(a).position().y() - anchorA.y();
            float dB = world.getBodyState(b).position().y() - anchorB.y();
            assertTrue(java.lang.Math.abs((dA + dB) - ropeLength) < 0.5f,
                "Expected pulley length conservation for " + backend + " got dA+dB=" + (dA + dB));
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

    private static RigidBodyHandle spawnBody(PhysicsWorld world, float x, float y, float z) {
        return world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.box(0.25f, 0.25f, 0.25f), 1f)
            .worldTransform(new Matrix4f().translation(x, y, z))
            .build());
    }

    private static void step(PhysicsWorld world, int frames) {
        for (int i = 0; i < frames; i++) {
            world.step(1f / 60f, 1);
        }
    }
}
