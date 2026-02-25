package org.dynamisphysics.ode4j;

import org.dynamiscollision.shapes.CollisionShape;
import org.dynamisphysics.api.PhysicsWorldFactory;
import org.dynamisphysics.api.body.BodyMode;
import org.dynamisphysics.api.body.RigidBodyConfig;
import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.config.PhysicsBackend;
import org.dynamisphysics.api.config.PhysicsWorldConfig;
import org.dynamisphysics.api.world.PhysicsWorld;
import org.dynamisphysics.api.world.PhysicsStats;
import org.dynamisphysics.ode4j.snapshot.Ode4jSnapshot;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.vectrix.core.Matrix4f;
import org.vectrix.core.Vector3f;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class Ode4jDeterminismTest {
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
    void snapshotRoundtripPreservesBodyCount() {
        spawnGround(world);
        spawnSpheres(world, 6);
        world.step(1f / 60f, 1);

        byte[] snap = world.snapshot();
        world.restore(snap);

        PhysicsStats stats = world.getStats();
        assertEquals(7, stats.bodyCount());
    }

    @Test
    void snapshotRoundtripPreservesBodyPositions() {
        spawnGround(world);
        spawnSpheres(world, 4);
        runSteps(world, 120);

        byte[] snap1 = world.snapshot();
        world.restore(snap1);
        byte[] snap2 = world.snapshot();

        Ode4jSnapshot.RestoredState s1 = Ode4jSnapshot.read(snap1);
        Ode4jSnapshot.RestoredState s2 = Ode4jSnapshot.read(snap2);
        assertEquals(s1.bodies().size(), s2.bodies().size());
        for (int i = 0; i < s1.bodies().size(); i++) {
            var b1 = s1.bodies().get(i);
            var b2 = s2.bodies().get(i);
            assertEquals(b1.bodyId(), b2.bodyId());
            assertEquals(b1.position().x(), b2.position().x(), 1e-5f);
            assertEquals(b1.position().y(), b2.position().y(), 1e-5f);
            assertEquals(b1.position().z(), b2.position().z(), 1e-5f);
        }
    }

    @Test
    void restoreAndContinueMatchesContinuousRun() {
        PhysicsWorld continuous = PhysicsWorldFactory.create(PhysicsWorldConfig.defaults(PhysicsBackend.ODE4J));
        PhysicsWorld checkpoint = PhysicsWorldFactory.create(PhysicsWorldConfig.defaults(PhysicsBackend.ODE4J));
        try {
            List<RigidBodyHandle> continuousBodies = spawnIsolatedSpheres(continuous, 20);
            List<RigidBodyHandle> checkpointBodies = spawnIsolatedSpheres(checkpoint, 20);

            for (int i = 0; i < 300; i++) {
                if (i == 10 || i == 120) {
                    applyDeterministicImpulses(continuous, continuousBodies);
                    applyDeterministicImpulses(checkpoint, checkpointBodies);
                }
                continuous.step(1f / 60f, 1);
                checkpoint.step(1f / 60f, 1);
            }

            byte[] midSnap = checkpoint.snapshot();
            checkpoint.restore(midSnap);

            for (int i = 0; i < 300; i++) {
                continuous.step(1f / 60f, 1);
                checkpoint.step(1f / 60f, 1);
            }

            Ode4jSnapshot.RestoredState expected = Ode4jSnapshot.read(continuous.snapshot());
            Ode4jSnapshot.RestoredState actual = Ode4jSnapshot.read(checkpoint.snapshot());
            assertEquals(expected.bodies().size(), actual.bodies().size());
            for (int i = 0; i < expected.bodies().size(); i++) {
                var a = expected.bodies().get(i);
                var b = actual.bodies().get(i);
                assertEquals(a.bodyId(), b.bodyId());
                assertEquals(a.position().x(), b.position().x(), 1e-3f);
                assertEquals(a.position().y(), b.position().y(), 1e-3f);
                assertEquals(a.position().z(), b.position().z(), 1e-3f);
                assertEquals(a.linearVelocity().x(), b.linearVelocity().x(), 1e-3f);
                assertEquals(a.linearVelocity().y(), b.linearVelocity().y(), 1e-3f);
                assertEquals(a.linearVelocity().z(), b.linearVelocity().z(), 1e-3f);
            }
        } finally {
            continuous.destroy();
            checkpoint.destroy();
        }
    }

    @Test
    void twoRunsSameInputsProduceBitIdenticalSnapshot600Steps() {
        byte[] a = runDeterministicScenario();
        byte[] b = runDeterministicScenario();
        assertArrayEquals(a, b, "Physics snapshots not bit-identical after 600 steps");
    }

    @Test
    void statsBodyCountMatchesSpawnMinusDestroy() {
        RigidBodyHandle a = world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.sphere(0.5f), 1f).build());
        world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.sphere(0.5f), 1f).build());
        world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.sphere(0.5f), 1f).build());
        world.destroyRigidBody(a);
        assertEquals(2, world.getStats().bodyCount());
    }

    @Test
    void statsActiveBodyCountDecreasesAsBodiesSleep() {
        spawnGround(world);
        world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.sphere(0.5f), 1f)
            .worldTransform(new Matrix4f().translation(0f, 2f, 0f))
            .build());

        runSteps(world, 600);
        PhysicsStats stats = world.getStats();
        assertTrue(stats.sleepingBodyCount() >= 1, "Expected at least one sleeping dynamic body");
        assertTrue(stats.activeBodyCount() <= 1, "Expected active dynamic bodies to decrease");
    }

    private static byte[] runDeterministicScenario() {
        PhysicsWorld world = PhysicsWorldFactory.create(PhysicsWorldConfig.defaults(PhysicsBackend.ODE4J));
        try {
            List<RigidBodyHandle> bodies = spawnIsolatedSpheres(world, 20);
            for (int i = 0; i < 600; i++) {
                if (i == 10 || i == 120) {
                    applyDeterministicImpulses(world, bodies);
                }
                world.step(1f / 60f, 1);
            }
            return world.snapshot();
        } finally {
            world.destroy();
        }
    }

    private static void applyDeterministicImpulses(PhysicsWorld world, List<RigidBodyHandle> bodies) {
        for (int i = 0; i < bodies.size(); i++) {
            float sx = (i % 2 == 0) ? 1f : -1f;
            Vector3f impulse = new Vector3f(5f * sx, 0f, 0.5f * i);
            world.applyImpulse(bodies.get(i), impulse, new Vector3f());
        }
    }

    private static void runSteps(PhysicsWorld world, int steps) {
        for (int i = 0; i < steps; i++) {
            world.step(1f / 60f, 1);
        }
    }

    private static void spawnGround(PhysicsWorld world) {
        world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.box(50f, 0.2f, 50f), 0f)
            .mode(BodyMode.STATIC)
            .worldTransform(new Matrix4f().translation(0f, -0.2f, 0f))
            .build());
    }

    private static List<RigidBodyHandle> spawnSpheres(PhysicsWorld world, int count) {
        List<RigidBodyHandle> handles = new ArrayList<>(count);
        for (int i = 0; i < count; i++) {
            handles.add(world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.sphere(0.5f), 1f)
                .worldTransform(new Matrix4f().translation((i % 5) * 1.5f, 2f + (i / 5), 0f))
                .build()));
        }
        return handles;
    }

    private static List<RigidBodyHandle> spawnIsolatedSpheres(PhysicsWorld world, int count) {
        List<RigidBodyHandle> handles = new ArrayList<>(count);
        for (int i = 0; i < count; i++) {
            handles.add(world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.sphere(0.5f), 1f)
                .worldTransform(new Matrix4f().translation(i * 20f, 50f + i, 0f))
                .build()));
        }
        return handles;
    }
}
