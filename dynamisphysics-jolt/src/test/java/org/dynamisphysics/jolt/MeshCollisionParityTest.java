package org.dynamisphysics.jolt;

import org.dynamisphysics.api.PhysicsWorldFactory;
import org.dynamisphysics.api.body.BodyMode;
import org.dynamisphysics.api.body.RigidBodyConfig;
import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.config.PhysicsBackend;
import org.dynamisphysics.api.config.PhysicsWorldConfig;
import org.dynamisphysics.api.world.PhysicsWorld;
import org.dynamisphysics.jolt.meshforge.MeshForgeCollisionShapes;
import org.dynamisphysics.ode4j.Ode4jBackendRegistrar;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;
import org.meshforge.api.Meshes;
import org.meshforge.core.attr.AttributeSemantic;
import org.meshforge.core.attr.VertexFormat;
import org.meshforge.core.attr.VertexSchema;
import org.meshforge.pack.buffer.PackedMesh;
import org.meshforge.pack.packer.MeshPacker;
import org.meshforge.pack.spec.PackSpec;
import org.vectrix.core.Matrix4f;

import static org.junit.jupiter.api.Assertions.assertTrue;

class MeshCollisionParityTest {
    private static final float DT = 1f / 60f;

    @BeforeAll
    static void registerBackends() {
        new Ode4jBackendRegistrar();
        new JoltBackendRegistrar();
    }

    @ParameterizedTest
    @EnumSource(value = PhysicsBackend.class, names = {"ODE4J", "JOLT"})
    void staticMeshGroundStopsSphere(PhysicsBackend backend) {
        PhysicsWorld world = PhysicsWorldFactory.create(PhysicsWorldConfig.defaults(backend));
        try {
            PackedMesh groundMesh = MeshPacker.pack(quadMeshData(40f), meshSpec());
            world.spawnRigidBody(RigidBodyConfig.builder(MeshForgeCollisionShapes.triangleMesh(groundMesh), 0f)
                .mode(BodyMode.STATIC)
                .worldTransform(new Matrix4f().identity().translation(0f, 0f, 0f))
                .build());

            RigidBodyHandle sphere = world.spawnRigidBody(RigidBodyConfig.builder(org.dynamiscollision.shapes.CollisionShape.sphere(0.5f), 1f)
                .worldTransform(new Matrix4f().identity().translation(0f, 4f, 0f))
                .build());
            float startY = world.getBodyState(sphere).position().y();
            for (int i = 0; i < 180; i++) {
                world.step(DT, 1);
            }
            float endY = world.getBodyState(sphere).position().y();
            assertTrue(endY < startY, "sphere did not fall");
            assertTrue(endY > -2f, "sphere tunneled through mesh ground");
        } finally {
            world.destroy();
        }
    }

    @ParameterizedTest
    @EnumSource(value = PhysicsBackend.class, names = {"ODE4J", "JOLT"})
    void convexHullDynamicInteractsWithGround(PhysicsBackend backend) {
        PhysicsWorld world = PhysicsWorldFactory.create(PhysicsWorldConfig.defaults(backend));
        try {
            world.spawnRigidBody(RigidBodyConfig.builder(org.dynamiscollision.shapes.CollisionShape.box(20f, 0.5f, 20f), 0f)
                .mode(BodyMode.STATIC)
                .worldTransform(new Matrix4f().identity().translation(0f, -0.5f, 0f))
                .build());

            PackedMesh hullMesh = MeshPacker.pack(Meshes.cube(1f), meshSpec());
            RigidBodyHandle hull = world.spawnRigidBody(RigidBodyConfig.builder(MeshForgeCollisionShapes.convexHull(hullMesh), 2f)
                .worldTransform(new Matrix4f().identity().translation(0f, 4f, 0f))
                .build());
            float startY = world.getBodyState(hull).position().y();
            for (int i = 0; i < 180; i++) {
                world.step(DT, 1);
            }
            float endY = world.getBodyState(hull).position().y();
            assertTrue(endY < startY, "convex hull did not fall");
            assertTrue(endY > -5f, "convex hull fell through ground");
        } finally {
            world.destroy();
        }
    }

    private static org.meshforge.core.mesh.MeshData quadMeshData(float halfExtent) {
        VertexSchema schema = VertexSchema.builder()
            .add(AttributeSemantic.POSITION, VertexFormat.F32x3)
            .build();
        float[] positions = {
            -halfExtent, 0f, -halfExtent,
             halfExtent, 0f, -halfExtent,
             halfExtent, 0f,  halfExtent,
            -halfExtent, 0f,  halfExtent
        };
        int[] indices = {
            0, 1, 2,
            0, 2, 3,
            2, 1, 0,
            3, 2, 0
        };
        return Meshes.writer(schema, 4, indices.length)
            .positions(positions)
            .indices(indices)
            .build();
    }

    private static PackSpec meshSpec() {
        return PackSpec.builder()
            .target(AttributeSemantic.POSITION, 0, VertexFormat.F32x3)
            .dropUnknownAttributes(true)
            .build();
    }
}
