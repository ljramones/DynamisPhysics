package org.dynamisphysics.ode4j;

import org.dynamisphysics.api.PhysicsWorldFactory;
import org.dynamisphysics.api.body.BodyMode;
import org.dynamisphysics.api.body.RigidBodyConfig;
import org.dynamisphysics.api.config.PhysicsBackend;
import org.dynamisphysics.api.config.PhysicsWorldConfig;
import org.dynamisphysics.api.world.PhysicsWorld;
import org.dynamiscollision.shapes.CollisionShape;
import org.dynamisphysics.ode4j.shape.BvhTriangleMeshBuilder;
import org.dynamisphysics.ode4j.shape.ConvexHullShapeBuilder;
import org.dynamisphysics.ode4j.shape.RagdollCapsuleFitter;
import org.meshforge.core.attr.AttributeKey;
import org.meshforge.core.attr.AttributeSemantic;
import org.meshforge.core.attr.VertexFormat;
import org.meshforge.pack.buffer.PackedMesh;
import org.meshforge.pack.layout.VertexLayout;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.vectrix.affine.Transformf;
import org.vectrix.core.Matrix4f;
import org.vectrix.core.Quaternionf;
import org.vectrix.core.Vector3f;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.LinkedHashMap;
import java.util.List;

import static org.dynamisphysics.test.assertions.PhysicsAssertions.assertAlive;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

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
        int[] idx = {0, 1, 2, 0, 2, 3};
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
    void convexHullBuilderFromPackedMeshProducesShape() {
        PackedMesh mesh = packedQuad();
        var shape = ConvexHullShapeBuilder.fromPackedMesh(mesh);
        assertEquals(12, shape.vertices().length);
        assertTrue(shape.indices().length >= 3);
    }

    @Test
    void bvhBuilderFromPackedMeshProducesShape() {
        PackedMesh mesh = packedQuad();
        var shape = BvhTriangleMeshBuilder.fromPackedMesh(mesh);
        assertEquals(12, shape.vertices().length);
        assertEquals(6, shape.indices().length);
    }

    @Test
    void ragdollCapsuleFitterProducesReasonableDimensions() {
        PackedMesh mesh = packedPrism();
        var capsule = RagdollCapsuleFitter.fitToBoneYAxis(mesh);
        assertTrue(capsule.radius() > 0f);
        assertTrue(capsule.height() >= capsule.radius() * 2f);
    }

    private static PackedMesh packedQuad() {
        float[] pos = {
            -1f, 0f, -1f,
            1f, 0f, -1f,
            1f, 0f, 1f,
            -1f, 0f, 1f
        };
        int[] idx = {0, 1, 2, 0, 2, 3};
        return packedMesh(pos, idx);
    }

    private static PackedMesh packedPrism() {
        float[] pos = {
            -0.04f, 0.15f, -0.04f,
            0.04f, 0.15f, -0.04f,
            0.04f, 0.15f, 0.04f,
            -0.04f, 0.15f, 0.04f,
            -0.04f, -0.15f, -0.04f,
            0.04f, -0.15f, -0.04f,
            0.04f, -0.15f, 0.04f,
            -0.04f, -0.15f, 0.04f
        };
        int[] idx = {
            0, 1, 2, 0, 2, 3,
            4, 6, 5, 4, 7, 6,
            0, 4, 5, 0, 5, 1,
            1, 5, 6, 1, 6, 2,
            2, 6, 7, 2, 7, 3,
            3, 7, 4, 3, 4, 0
        };
        return packedMesh(pos, idx);
    }

    private static PackedMesh packedMesh(float[] positions, int[] indices) {
        ByteBuffer vb = ByteBuffer.allocateDirect(positions.length * Float.BYTES).order(ByteOrder.nativeOrder());
        for (float v : positions) {
            vb.putFloat(v);
        }
        vb.flip();

        ByteBuffer ib = ByteBuffer.allocateDirect(indices.length * Integer.BYTES).order(ByteOrder.nativeOrder());
        for (int idx : indices) {
            ib.putInt(idx);
        }
        ib.flip();

        AttributeKey positionKey = new AttributeKey(AttributeSemantic.POSITION, 0);
        var entries = new LinkedHashMap<AttributeKey, VertexLayout.Entry>();
        entries.put(positionKey, new VertexLayout.Entry(positionKey, VertexFormat.F32x3, 0));
        VertexLayout layout = new VertexLayout(3 * Float.BYTES, entries);

        PackedMesh.IndexBufferView indexView = new PackedMesh.IndexBufferView(
            PackedMesh.IndexType.UINT32, ib, indices.length
        );

        return new PackedMesh(
            layout,
            vb,
            indexView,
            List.of(new PackedMesh.SubmeshRange(0, indices.length, null))
        );
    }
}
