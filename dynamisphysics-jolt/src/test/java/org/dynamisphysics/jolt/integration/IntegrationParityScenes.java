package org.dynamisphysics.jolt.integration;

import org.dynamiscollision.shapes.CollisionShape;
import org.dynamisphysics.api.AnimisPose;
import org.dynamisphysics.api.CharacterDescriptor;
import org.dynamisphysics.api.CharacterHandle;
import org.dynamisphysics.api.PhysicsWorldFactory;
import org.dynamisphysics.api.RagdollBoneDesc;
import org.dynamisphysics.api.RagdollDescriptor;
import org.dynamisphysics.api.RagdollHandle;
import org.dynamisphysics.api.RagdollJointDesc;
import org.dynamisphysics.api.VehicleDescriptor;
import org.dynamisphysics.api.VehicleHandle;
import org.dynamisphysics.api.body.BodyMode;
import org.dynamisphysics.api.body.BodyState;
import org.dynamisphysics.api.body.RigidBodyConfig;
import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.config.BroadphaseType;
import org.dynamisphysics.api.config.PhysicsBackend;
import org.dynamisphysics.api.config.PhysicsWorldConfig;
import org.dynamisphysics.api.constraint.ConstraintDesc;
import org.dynamisphysics.api.constraint.ConstraintHandle;
import org.dynamisphysics.api.constraint.ConstraintLimits;
import org.dynamisphysics.api.constraint.ConstraintMotor;
import org.dynamisphysics.api.constraint.ConstraintType;
import org.dynamisphysics.api.constraint.MechanicalConstraintBuilders;
import org.dynamisphysics.api.event.PhysicsEvent;
import org.dynamisphysics.api.material.PhysicsMaterial;
import org.dynamisphysics.api.query.RaycastResult;
import org.dynamisphysics.api.world.PhysicsWorld;
import org.dynamisphysics.api.world.VehicleState;
import org.vectrix.affine.Transformf;
import org.vectrix.core.Matrix4f;
import org.vectrix.core.Quaternionf;
import org.vectrix.core.Vector3f;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Predicate;

final class IntegrationParityScenes {
    static final float DT = 1f / 60f;

    private IntegrationParityScenes() {
    }

    static PhysicsWorld newWorld(PhysicsBackend backend, boolean deterministic) {
        PhysicsWorldConfig d = PhysicsWorldConfig.defaults(backend);
        return PhysicsWorldFactory.create(new PhysicsWorldConfig(
            backend,
            new Vector3f(0f, -9.81f, 0f),
            d.fixedTimeStep(),
            1,
            20,
            d.maxBodies(),
            d.maxConstraints(),
            BroadphaseType.BVH,
            deterministic
        ));
    }

    static RigidBodyHandle spawnGroundBox(PhysicsWorld world) {
        return spawnGroundBox(world, PhysicsMaterial.ASPHALT);
    }

    static RigidBodyHandle spawnGroundBox(PhysicsWorld world, PhysicsMaterial material) {
        return world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.box(50f, 0.5f, 50f), 0f)
            .mode(BodyMode.STATIC)
            .material(material)
            .worldTransform(new Matrix4f().identity().translation(0f, -0.5f, 0f))
            .build());
    }

    static RigidBodyHandle spawnMeshQuadZone(PhysicsWorld world, float minX, float maxX, PhysicsMaterial material) {
        float halfZ = 20f;
        float[] verts = {
            minX, 0f, -halfZ,
            maxX, 0f, -halfZ,
            maxX, 0f, halfZ,
            minX, 0f, halfZ
        };
        int[] idx = {
            0, 1, 2,
            0, 2, 3
        };
        return world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.triangleMesh(verts, idx), 0f)
            .mode(BodyMode.STATIC)
            .material(material)
            .worldTransform(new Matrix4f().identity())
            .build());
    }

    static VehicleHandle spawnVehicle(PhysicsWorld world, float x, float y, float z) {
        VehicleHandle handle = world.spawnVehicle(VehicleDescriptor.simpleCar(CollisionShape.box(1f, 0.4f, 2.2f), 1200f));
        world.teleport(handle.chassisBody(), new Vector3f(x, y, z), new Quaternionf());
        return handle;
    }

    static VehicleHandle spawnAndSettleVehicle(PhysicsWorld world, float x, float y, float z) {
        VehicleHandle handle = spawnVehicle(world, x, y, z);
        step(world, 20);
        world.drainEvents();
        return handle;
    }

    static CharacterHandle spawnCharacter(PhysicsWorld world) {
        return world.spawnCharacter(new CharacterDescriptor(
            1.8f, 0.35f, 80f, 0.35f, 45f, 1000f, 0.05f, PhysicsMaterial.DEFAULT, 1, -1
        ));
    }

    static RagdollHandle spawnRagdoll(PhysicsWorld world, float y) {
        RagdollDescriptor descriptor = new RagdollDescriptor(
            List.of(
                new RagdollBoneDesc("root", CollisionShape.capsule(0.2f, 0.6f), 10f, new Vector3f(), 80f, 8f, 60f),
                new RagdollBoneDesc("spine", CollisionShape.capsule(0.18f, 0.5f), 8f, new Vector3f(), 70f, 7f, 50f)
            ),
            List.of(
                new RagdollJointDesc("root", "spine", ConstraintType.BALL_SOCKET, ConstraintLimits.free())
            ),
            18f
        );
        AnimisPose pose = new AnimisPose(Map.of(
            "root", new Matrix4f().identity().translation(0f, y, 0f),
            "spine", new Matrix4f().identity().translation(0f, y + 1f, 0f)
        ));
        RagdollHandle handle = world.spawnRagdoll(descriptor, pose);
        world.activateRagdoll(handle, 0f);
        return handle;
    }

    static List<RigidBodyHandle> spawnCompounds(PhysicsWorld world, int count, float startX) {
        CollisionShape compound = CollisionShape.compound(
            List.of(CollisionShape.sphere(0.25f), CollisionShape.sphere(0.25f)),
            List.of(
                new Transformf(new Vector3f(-0.4f, 0f, 0f), new Quaternionf(0f, 0f, 0f, 1f), new Vector3f(1f, 1f, 1f)),
                new Transformf(new Vector3f(0.4f, 0f, 0f), new Quaternionf(0f, 0f, 0f, 1f), new Vector3f(1f, 1f, 1f))
            )
        );
        List<RigidBodyHandle> out = new ArrayList<>(count);
        for (int i = 0; i < count; i++) {
            out.add(world.spawnRigidBody(RigidBodyConfig.builder(compound, 5f)
                .worldTransform(new Matrix4f().identity().translation(startX + i * 1.2f, 5f + i * 0.4f, 2f))
                .build()));
        }
        return out;
    }

    static ConstraintModule addSpringModule(PhysicsWorld world, float x) {
        RigidBodyHandle a = world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.box(0.25f, 0.25f, 0.25f), 1f)
            .worldTransform(new Matrix4f().identity().translation(x, 2f, 0f))
            .build());
        RigidBodyHandle b = world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.box(0.25f, 0.25f, 0.25f), 1f)
            .worldTransform(new Matrix4f().identity().translation(x + 0.8f, 2f, 0f))
            .build());
        ConstraintHandle handle = world.addConstraint(new ConstraintDesc(
            ConstraintType.SIX_DOF_SPRING,
            a,
            b,
            new Vector3f(),
            new Vector3f(),
            new Vector3f(0f, 1f, 0f),
            new Vector3f(1f, 0f, 0f),
            new ConstraintLimits(-2f, 2f, -1f, 1f),
            new ConstraintMotor(true, 0f, 0f, 120f, 20f),
            0f,
            0f
        ));
        return new ConstraintModule(handle, a, b);
    }

    static ConstraintModule addMechanicalModule(PhysicsWorld world, float x) {
        RigidBodyHandle a = world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.box(0.25f, 0.25f, 0.25f), 1f)
            .worldTransform(new Matrix4f().identity().translation(x, 2f, -2f))
            .build());
        RigidBodyHandle b = world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.box(0.25f, 0.25f, 0.25f), 1f)
            .worldTransform(new Matrix4f().identity().translation(x + 0.6f, 2f, -2f))
            .build());
        ConstraintHandle handle = world.addConstraint(MechanicalConstraintBuilders.gear(
            a, b, new Vector3f(0f, 1f, 0f), new Vector3f(0f, 1f, 0f), 2f
        ));
        return new ConstraintModule(handle, a, b);
    }

    static void step(PhysicsWorld world, int frames) {
        for (int i = 0; i < frames; i++) {
            world.step(DT, 1);
        }
    }

    static int countEvents(List<PhysicsEvent> events, Predicate<PhysicsEvent> filter) {
        int count = 0;
        for (PhysicsEvent event : events) {
            if (filter.test(event)) {
                count++;
            }
        }
        return count;
    }

    static boolean stateFinite(BodyState state) {
        return finite(state.position()) && finite(state.linearVelocity()) && finite(state.angularVelocity());
    }

    static boolean finite(Vector3f value) {
        return Float.isFinite(value.x()) && Float.isFinite(value.y()) && Float.isFinite(value.z());
    }

    static float minY(PhysicsWorld world, List<RigidBodyHandle> handles) {
        float min = Float.POSITIVE_INFINITY;
        for (RigidBodyHandle handle : handles) {
            min = java.lang.Math.min(min, world.getBodyState(handle).position().y());
        }
        return min;
    }

    static Optional<RaycastResult> downRay(PhysicsWorld world, float x, float y, float z) {
        return world.raycastClosest(new Vector3f(x, y, z), new Vector3f(0f, -1f, 0f), 50f, -1);
    }

    static List<SnapshotState> decodeDynamicStates(PhysicsBackend backend, byte[] snapshot) {
        List<SnapshotState> all = backend == PhysicsBackend.ODE4J ? decodeOde(snapshot) : decodeJolt(snapshot);
        return all.stream().filter(s -> s.mode() == BodyMode.DYNAMIC).toList();
    }

    private static List<SnapshotState> decodeJolt(byte[] snapshot) {
        ByteReader in = new ByteReader(snapshot);
        in.readInt();
        in.readShort();
        in.readShort();
        in.readInt();
        in.readFloat();
        in.readFloat();
        in.readFloat();
        in.readFloat();
        int bodyCount = in.readInt();
        List<SnapshotState> out = new ArrayList<>(bodyCount);
        for (int i = 0; i < bodyCount; i++) {
            int bodyId = in.readInt();
            BodyMode mode = BodyMode.values()[in.readByteUnsigned()];
            skipJoltShape(in);
            in.readFloat(); // mass
            in.readFloat();
            in.readInt();
            in.readInt();
            Vector3f p = new Vector3f(in.readFloat(), in.readFloat(), in.readFloat());
            in.readFloat();
            in.readFloat();
            in.readFloat();
            in.readFloat();
            Vector3f lv = new Vector3f(in.readFloat(), in.readFloat(), in.readFloat());
            in.readFloat();
            in.readFloat();
            in.readFloat();
            out.add(new SnapshotState(bodyId, mode, p, lv));
        }
        out.sort(Comparator.comparingInt(SnapshotState::bodyId));
        return out;
    }

    private static void skipJoltShape(ByteReader in) {
        in.readByteUnsigned();
        int fLen = in.readInt();
        in.skip(fLen * 4);
        int iLen = in.readInt();
        in.skip(iLen * 4);
    }

    private static List<SnapshotState> decodeOde(byte[] snapshot) {
        ByteReader in = new ByteReader(snapshot);
        in.readInt();
        in.readShort();
        in.readShort();
        in.readInt();
        in.readFloat();
        in.readFloat();
        in.readFloat();
        in.readInt();
        in.readFloat();
        int bodyCount = in.readInt();
        List<SnapshotState> out = new ArrayList<>(bodyCount);
        for (int i = 0; i < bodyCount; i++) {
            int bodyId = in.readInt();
            in.readInt();
            BodyMode mode = BodyMode.values()[in.readByteUnsigned()];
            in.readByteUnsigned();
            in.readInt();
            in.readInt();
            in.readFloat();
            in.readFloat();
            in.readFloat();
            in.readFloat();
            in.readFloat();
            in.readFloat();
            int strLen = in.readInt();
            if (strLen > 0) {
                in.skip(strLen);
            }
            skipOdeShape(in);
            Vector3f p = new Vector3f(in.readFloat(), in.readFloat(), in.readFloat());
            in.readFloat();
            in.readFloat();
            in.readFloat();
            in.readFloat();
            Vector3f lv = new Vector3f(in.readFloat(), in.readFloat(), in.readFloat());
            in.readFloat();
            in.readFloat();
            in.readFloat();
            out.add(new SnapshotState(bodyId, mode, p, lv));
        }
        out.sort(Comparator.comparingInt(SnapshotState::bodyId));
        return out;
    }

    private static void skipOdeShape(ByteReader in) {
        int type = in.readByteUnsigned();
        switch (type) {
            case 0 -> in.skip(4);
            case 1 -> in.skip(12);
            case 2, 3 -> in.skip(8);
            case 4 -> in.skip(16);
            case 5, 6 -> {
                int fLen = in.readInt();
                in.skip(fLen * 4);
                int iLen = in.readInt();
                in.skip(iLen * 4);
            }
            case 7 -> {
                int hLen = in.readInt();
                in.skip(hLen * 4);
                in.skip(20);
            }
            case 8 -> {
                int children = in.readInt();
                for (int i = 0; i < children; i++) {
                    skipOdeShape(in);
                    in.skip(40);
                }
            }
            default -> throw new IllegalArgumentException("Unknown ODE shape type: " + type);
        }
    }

    static String summary(
        PhysicsWorld world,
        String label,
        List<RigidBodyHandle> bodies,
        VehicleHandle vehicle,
        CharacterHandle character,
        int eventCount
    ) {
        VehicleState vs = vehicle != null ? world.getVehicleState(vehicle) : VehicleState.ZERO;
        return label + " bodies=" + world.getStats().bodyCount()
            + " constraints=" + world.getStats().constraintCount()
            + " minY=" + minY(world, bodies)
            + " vehicleSpeed=" + vs.speed()
            + " grounded=" + (character != null && world.getCharacterState(character).isGrounded())
            + " events=" + eventCount;
    }

    record ConstraintModule(
        ConstraintHandle handle,
        RigidBodyHandle bodyA,
        RigidBodyHandle bodyB
    ) {
    }

    record SnapshotState(int bodyId, BodyMode mode, Vector3f position, Vector3f linearVelocity) {
    }

    private static final class ByteReader {
        private final ByteBuffer buffer;

        private ByteReader(byte[] bytes) {
            this.buffer = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN);
        }

        int readInt() {
            return buffer.getInt();
        }

        short readShort() {
            return buffer.getShort();
        }

        float readFloat() {
            return buffer.getFloat();
        }

        int readByteUnsigned() {
            return Byte.toUnsignedInt(buffer.get());
        }

        void skip(int bytes) {
            buffer.position(buffer.position() + bytes);
        }
    }
}
