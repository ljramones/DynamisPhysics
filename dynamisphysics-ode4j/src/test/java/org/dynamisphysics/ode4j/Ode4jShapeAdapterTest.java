package org.dynamisphysics.ode4j;

import org.dynamisphysics.api.PhysicsWorldFactory;
import org.dynamisphysics.api.body.BodyMode;
import org.dynamisphysics.api.body.BodyState;
import org.dynamisphysics.api.body.RigidBodyConfig;
import org.dynamisphysics.api.config.PhysicsBackend;
import org.dynamisphysics.api.config.PhysicsWorldConfig;
import org.dynamisphysics.api.world.PhysicsWorld;
import org.dynamiscollision.shapes.CollisionShape;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.vectrix.affine.Transformf;
import org.vectrix.core.Matrix4f;
import org.vectrix.core.Quaternionf;
import org.vectrix.core.Vector3f;

import java.util.List;

import static org.dynamisphysics.test.assertions.PhysicsAssertions.assertAlive;
import static org.dynamisphysics.test.assertions.PhysicsAssertions.assertYAbove;
import static org.dynamisphysics.test.assertions.PhysicsAssertions.assertYBelow;
import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;

class Ode4jShapeAdapterTest {
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
    void sphereShapeSpawnsLiveBody() {
        var h = world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.sphere(0.5f), 1f).build());
        assertAlive(h);
    }

    @Test
    void boxShapeSpawnsLiveBody() {
        var h = world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.box(1f, 1f, 1f), 1f).build());
        assertAlive(h);
    }

    @Test
    void capsuleShapeSpawnsLiveBody() {
        var h = world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.capsule(0.3f, 1f), 1f).build());
        assertAlive(h);
    }

    @Test
    void cylinderShapeSpawnsLiveBody() {
        var h = world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.cylinder(0.3f, 1f), 1f).build());
        assertAlive(h);
    }

    @Test
    void planeShapeSpawnsStaticBody() {
        var h = world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.planeY(), 0f)
            .mode(BodyMode.STATIC)
            .build());
        assertAlive(h);
    }

    @Test
    void convexHullShapeSpawnsLiveBody() {
        float[] verts = {
            0f, 1f, 0f,
            -1f, -1f, 1f,
            1f, -1f, 1f,
            0f, -1f, -1f
        };
        int[] idx = {0, 1, 2, 0, 2, 3, 0, 3, 1, 1, 3, 2};
        var h = world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.convexHull(verts, idx), 1f)
            .worldTransform(new Matrix4f().translation(0f, 5f, 0f))
            .build());
        assertAlive(h);
    }

    @Test
    void triangleMeshShapeSpawnsStaticBody() {
        float[] verts = {
            -5f, 0f, -5f,
            5f, 0f, -5f,
            5f, 0f, 5f,
            -5f, 0f, 5f
        };
        int[] idx = {0, 2, 1, 0, 3, 2};
        var h = world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.triangleMesh(verts, idx), 0f)
            .mode(BodyMode.STATIC)
            .build());
        assertAlive(h);
    }

    @Test
    void heightfieldShapeSpawnsStaticBody() {
        int w = 4;
        int d = 4;
        float[] heights = new float[w * d];
        var h = world.spawnRigidBody(RigidBodyConfig.builder(
                CollisionShape.heightfield(heights, w, d, 10f, 10f, 5f), 0f)
            .mode(BodyMode.STATIC)
            .build());
        assertAlive(h);
    }

    @Test
    void compoundShapeSpawnsStaticBody() {
        List<CollisionShape> children = List.of(
            CollisionShape.sphere(0.3f),
            CollisionShape.box(0.5f, 0.5f, 0.5f)
        );
        List<Transformf> transforms = List.of(
            new Transformf(new Vector3f(0f, 0.5f, 0f), new Quaternionf(), new Vector3f(1f, 1f, 1f)),
            new Transformf(new Vector3f(0f, -0.5f, 0f), new Quaternionf(), new Vector3f(1f, 1f, 1f))
        );
        var h = world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.compound(children, transforms), 0f)
            .mode(BodyMode.STATIC)
            .build());
        assertAlive(h);
    }

    @Test
    void triangleMeshGroundStopsDynamicSphere() {
        float[] verts = {
            -10f, 0f, -10f,
            10f, 0f, -10f,
            10f, 0f, 10f,
            -10f, 0f, 10f
        };
        int[] idx = {0, 2, 1, 0, 3, 2};
        world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.triangleMesh(verts, idx), 0f)
            .mode(BodyMode.STATIC)
            .build());
        var sphere = world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.sphere(0.5f), 1f)
            .worldTransform(new Matrix4f().translation(0f, 3f, 0f))
            .build());
        for (int i = 0; i < 120; i++) {
            world.step(1f / 60f, 1);
        }
        BodyState state = world.getBodyState(sphere);
        assertYAbove(state, -0.5f);
        assertYBelow(state, 2f);
    }

    @Test
    void heightfieldGroundStopsDynamicSphere() {
        int w = 8;
        int d = 8;
        float[] heights = new float[w * d];
        world.spawnRigidBody(RigidBodyConfig.builder(
                CollisionShape.heightfield(heights, w, d, 16f, 16f, 2f), 0f)
            .mode(BodyMode.STATIC)
            .build());
        var sphere = world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.sphere(0.5f), 1f)
            .worldTransform(new Matrix4f().translation(4f, 5f, 4f))
            .build());
        for (int i = 0; i < 120; i++) {
            world.step(1f / 60f, 1);
        }
        BodyState state = world.getBodyState(sphere);
        assertYAbove(state, -0.5f);
        assertYBelow(state, 4f);
    }

    @Test
    void convexHullBodyStepsWithoutThrow() {
        float[] verts = {
            0f, 1f, 0f,
            -1f, -1f, 1f,
            1f, -1f, 1f,
            0f, -1f, -1f
        };
        int[] idx = {0, 1, 2, 0, 2, 3, 0, 3, 1, 1, 3, 2};
        world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.convexHull(verts, idx), 1f)
            .worldTransform(new Matrix4f().translation(0f, 5f, 0f))
            .build());
        assertDoesNotThrow(() -> world.step(1f / 60f, 1));
    }
}
