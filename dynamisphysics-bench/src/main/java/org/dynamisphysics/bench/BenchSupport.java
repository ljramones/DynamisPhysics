package org.dynamisphysics.bench;

import org.dynamiscollision.shapes.CollisionShape;
import org.dynamisphysics.api.AnimisPose;
import org.dynamisphysics.api.PhysicsWorldFactory;
import org.dynamisphysics.api.RagdollBoneDesc;
import org.dynamisphysics.api.RagdollDescriptor;
import org.dynamisphysics.api.RagdollJointDesc;
import org.dynamisphysics.api.VehicleDescriptor;
import org.dynamisphysics.api.body.BodyMode;
import org.dynamisphysics.api.body.RigidBodyConfig;
import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.config.PhysicsBackend;
import org.dynamisphysics.api.config.PhysicsWorldConfig;
import org.dynamisphysics.api.constraint.ConstraintDesc;
import org.dynamisphysics.api.constraint.ConstraintLimits;
import org.dynamisphysics.api.constraint.ConstraintMotor;
import org.dynamisphysics.api.constraint.ConstraintType;
import org.dynamisphysics.api.constraint.MechanicalConstraintBuilders;
import org.dynamisphysics.api.material.PhysicsMaterial;
import org.dynamisphysics.api.world.PhysicsWorld;
import org.dynamisphysics.jolt.JoltBackendRegistrar;
import org.dynamisphysics.ode4j.Ode4jBackendRegistrar;
import org.meshforge.api.Meshes;
import org.meshforge.core.attr.AttributeSemantic;
import org.meshforge.core.attr.VertexFormat;
import org.meshforge.core.attr.VertexSchema;
import org.meshforge.pack.packer.MeshPacker;
import org.meshforge.pack.spec.PackSpec;
import org.vectrix.affine.Transformf;
import org.vectrix.core.Matrix4f;
import org.vectrix.core.Quaternionf;
import org.vectrix.core.Vector3f;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

final class BenchSupport {
    private static volatile boolean registered;

    private BenchSupport() {
    }

    static void ensureBackendsRegistered() {
        if (registered) {
            return;
        }
        synchronized (BenchSupport.class) {
            if (registered) {
                return;
            }
            new Ode4jBackendRegistrar();
            new JoltBackendRegistrar();
            registered = true;
        }
    }

    static PhysicsWorld createWorld(PhysicsBackend backend, boolean deterministic) {
        ensureBackendsRegistered();
        PhysicsWorldConfig cfg = new PhysicsWorldConfig(
            backend,
            PhysicsWorldConfig.defaults(backend).gravity(),
            1f / 60f,
            1,
            10,
            100_000,
            20_000,
            PhysicsWorldConfig.defaults(backend).broadphase(),
            deterministic
        );
        return PhysicsWorldFactory.create(cfg);
    }

    static RigidBodyHandle spawnGround(PhysicsWorld world) {
        return world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.box(200f, 1f, 200f), 0f)
            .mode(BodyMode.STATIC)
            .material(PhysicsMaterial.ASPHALT)
            .worldTransform(new Matrix4f().translation(0f, -1f, 0f))
            .build());
    }

    static RigidBodyHandle spawnMeshGround(PhysicsWorld world) {
        return world.spawnRigidBody(RigidBodyConfig.builder(createGroundMeshShape(200f), 0f)
            .mode(BodyMode.STATIC)
            .material(PhysicsMaterial.ASPHALT)
            .worldTransform(new Matrix4f().translation(0f, 0f, 0f))
            .build());
    }

    static List<RigidBodyHandle> spawnSphereGrid(PhysicsWorld world, int count, float radius) {
        List<RigidBodyHandle> handles = new ArrayList<>(count);
        int side = (int) java.lang.Math.ceil(java.lang.Math.sqrt(count));
        float spacing = radius * 2.25f;
        for (int i = 0; i < count; i++) {
            int x = i % side;
            int z = i / side;
            float px = (x - (side * 0.5f)) * spacing;
            float pz = (z - (side * 0.5f)) * spacing;
            float py = 4f + (i % 4) * (radius * 2.5f);
            handles.add(world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.sphere(radius), 1f)
                .worldTransform(new Matrix4f().translation(px, py, pz))
                .material(PhysicsMaterial.DEFAULT)
                .build()));
        }
        return handles;
    }

    static List<RigidBodyHandle> spawnRaycastTargets(PhysicsWorld world, int count) {
        List<RigidBodyHandle> handles = new ArrayList<>(count);
        int side = (int) java.lang.Math.ceil(java.lang.Math.sqrt(count));
        for (int i = 0; i < count; i++) {
            int x = i % side;
            int z = i / side;
            float px = (x - side / 2f) * 2.2f;
            float pz = (z - side / 2f) * 2.2f;
            handles.add(world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.box(0.5f, 0.5f, 0.5f), 0f)
                .mode(BodyMode.STATIC)
                .worldTransform(new Matrix4f().translation(px, 0.5f, pz))
                .build()));
        }
        return handles;
    }

    static CollisionShape buildCompoundCrossShape(int childrenPerCompound) {
        List<CollisionShape> children = new ArrayList<>(childrenPerCompound);
        List<Transformf> transforms = new ArrayList<>(childrenPerCompound);
        float spacing = 0.6f;
        for (int i = 0; i < childrenPerCompound; i++) {
            float angle = (float) (2.0 * java.lang.Math.PI * i / java.lang.Math.max(1, childrenPerCompound));
            float x = (float) java.lang.Math.cos(angle) * spacing;
            float z = (float) java.lang.Math.sin(angle) * spacing;
            children.add(CollisionShape.sphere(0.25f));
            transforms.add(new Transformf(
                new Vector3f(x, 0f, z),
                new Quaternionf(0f, 0f, 0f, 1f),
                new Vector3f(1f, 1f, 1f)
            ));
        }
        return CollisionShape.compound(children, transforms);
    }

    static List<RigidBodyHandle> spawnCompoundPile(PhysicsWorld world, int compoundCount, int childrenPerCompound) {
        List<RigidBodyHandle> handles = new ArrayList<>(compoundCount);
        CollisionShape shape = buildCompoundCrossShape(childrenPerCompound);
        int side = (int) java.lang.Math.ceil(java.lang.Math.sqrt(compoundCount));
        for (int i = 0; i < compoundCount; i++) {
            int x = i % side;
            int z = i / side;
            float px = (x - (side * 0.5f)) * 1.8f;
            float pz = (z - (side * 0.5f)) * 1.8f;
            float py = 3f + (i % 8) * 0.3f;
            handles.add(world.spawnRigidBody(RigidBodyConfig.builder(shape, 6f)
                .worldTransform(new Matrix4f().translation(px, py, pz))
                .material(PhysicsMaterial.DEFAULT)
                .build()));
        }
        return handles;
    }

    static List<RigidBodyHandle> spawnConstraintBodies(PhysicsWorld world, int bodyCount) {
        List<RigidBodyHandle> handles = new ArrayList<>(bodyCount);
        for (int i = 0; i < bodyCount; i++) {
            float x = (i % 40) * 0.6f - 12f;
            float z = (i / 40) * 0.8f - 8f;
            handles.add(world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.box(0.2f, 0.2f, 0.2f), 1f)
                .worldTransform(new Matrix4f().translation(x, 2.0f + (i % 4) * 0.15f, z))
                .build()));
        }
        return handles;
    }

    static ConstraintDesc springConstraint(RigidBodyHandle a, RigidBodyHandle b, float stiffness, float damping) {
        return new ConstraintDesc(
            ConstraintType.SIX_DOF_SPRING,
            a,
            b,
            new Vector3f(),
            new Vector3f(),
            new Vector3f(0f, 1f, 0f),
            new Vector3f(1f, 0f, 0f),
            new ConstraintLimits(-5f, 5f, -1f, 1f),
            new ConstraintMotor(true, 0f, 0f, stiffness, damping),
            0f,
            0f
        );
    }

    static void addMechanicalModule(PhysicsWorld world, RigidBodyHandle a, RigidBodyHandle b, int moduleIndex) {
        int mode = moduleIndex % 3;
        if (mode == 0) {
            world.addConstraint(MechanicalConstraintBuilders.gear(
                a, b, new Vector3f(0f, 1f, 0f), new Vector3f(0f, 1f, 0f), 2f
            ));
        } else if (mode == 1) {
            world.addConstraint(MechanicalConstraintBuilders.rackPinion(
                a, b, new Vector3f(1f, 0f, 0f), new Vector3f(0f, 1f, 0f), 0.5f
            ));
        } else {
            world.addConstraint(MechanicalConstraintBuilders.pulley(
                a, b,
                new Vector3f(0f, 0f, 0f), new Vector3f(2f, 0f, 0f),
                new Vector3f(0f, 1f, 0f), new Vector3f(0f, 1f, 0f),
                4f, 1f
            ));
        }
    }

    static RagdollDescriptor simpleRagdollDescriptor() {
        return new RagdollDescriptor(
            List.of(
                new RagdollBoneDesc("root", CollisionShape.capsule(0.2f, 0.6f), 10f, new Vector3f(), 80f, 8f, 60f),
                new RagdollBoneDesc("spine", CollisionShape.capsule(0.18f, 0.5f), 8f, new Vector3f(), 70f, 7f, 50f)
            ),
            List.of(
                new RagdollJointDesc("root", "spine", ConstraintType.BALL_SOCKET, ConstraintLimits.free())
            ),
            18f
        );
    }

    static AnimisPose simpleRagdollPose(float y) {
        return new AnimisPose(Map.of(
            "root", new Matrix4f().identity().translation(0f, y, 0f),
            "spine", new Matrix4f().identity().translation(0f, y + 1f, 0f)
        ));
    }

    static void warmStart(PhysicsWorld world, int steps) {
        for (int i = 0; i < steps; i++) {
            world.step(1f / 60f, 1);
        }
    }

    static VehicleDescriptor defaultVehicleDescriptor() {
        return VehicleDescriptor.simpleCar(CollisionShape.box(1f, 0.4f, 2.2f), 1200f);
    }

    private static CollisionShape createGroundMeshShape(float halfExtent) {
        VertexSchema schema = VertexSchema.builder()
            .add(AttributeSemantic.POSITION, VertexFormat.F32x3)
            .build();
        float[] positions = {
            -halfExtent, 0f, -halfExtent,
            halfExtent, 0f, -halfExtent,
            halfExtent, 0f, halfExtent,
            -halfExtent, 0f, halfExtent
        };
        int[] indices = {
            0, 1, 2,
            0, 2, 3,
            2, 1, 0,
            3, 2, 0
        };
        var mesh = Meshes.writer(schema, 4, indices.length)
            .positions(positions)
            .indices(indices)
            .build();
        var packed = MeshPacker.pack(mesh, PackSpec.builder()
            .target(AttributeSemantic.POSITION, 0, VertexFormat.F32x3)
            .dropUnknownAttributes(true)
            .build());
        return CollisionShape.triangleMesh(
            org.dynamisphysics.jolt.meshforge.MeshForgeExtractors.extractPositions(packed),
            org.dynamisphysics.jolt.meshforge.MeshForgeExtractors.extractIndices(packed)
        );
    }
}
