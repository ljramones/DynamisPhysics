package org.dynamisphysics.jolt;

import org.dynamiscollision.shapes.CollisionShape;
import org.dynamisphysics.api.PhysicsWorldFactory;
import org.dynamisphysics.api.body.BodyMode;
import org.dynamisphysics.api.body.RigidBodyConfig;
import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.config.PhysicsBackend;
import org.dynamisphysics.api.config.PhysicsWorldConfig;
import org.dynamisphysics.api.event.ContactEvent;
import org.dynamisphysics.api.event.PhysicsEvent;
import org.dynamisphysics.api.world.PhysicsWorld;
import org.dynamisphysics.ode4j.Ode4jBackendRegistrar;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;
import org.junit.jupiter.api.Test;
import org.vectrix.core.Matrix4f;
import org.vectrix.core.Vector3f;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class SnapshotParityTest {
    private static final int PRE_STEPS = 120;
    private static final int POST_STEPS = 120;
    private static final float DT = 1f / 60f;

    @BeforeAll
    static void registerBackends() {
        new Ode4jBackendRegistrar();
        new JoltBackendRegistrar();
    }

    @ParameterizedTest
    @EnumSource(value = PhysicsBackend.class, names = {"ODE4J", "JOLT"})
    void restoreAndContinueMatchesContinuousRun(PhysicsBackend backend) {
        ScenarioResult continuous = runScenario(backend, false);
        ScenarioResult restored = runScenario(backend, true);

        List<SnapshotBody> continuousBodies = dynamicBodies(decode(backend, continuous.finalSnapshot()));
        List<SnapshotBody> restoredBodies = dynamicBodies(decode(backend, restored.finalSnapshot()));
        assertEquals(continuousBodies.size(), restoredBodies.size());

        float posEps = backend == PhysicsBackend.ODE4J ? 1e-4f : 2e-2f;
        float velEps = backend == PhysicsBackend.ODE4J ? 1e-4f : 2e-2f;
        Map<Integer, SnapshotBody> byId = restoredBodies.stream()
            .collect(Collectors.toMap(SnapshotBody::bodyId, b -> b));
        for (SnapshotBody expected : continuousBodies) {
            SnapshotBody actual = byId.get(expected.bodyId());
            assertTrue(actual != null, "Missing body id " + expected.bodyId());
            assertNear(expected.position(), actual.position(), posEps);
            assertNear(expected.linearVelocity(), actual.linearVelocity(), velEps);
        }
    }

    @ParameterizedTest
    @EnumSource(value = PhysicsBackend.class, names = {"ODE4J", "JOLT"})
    void snapshotRoundtripPreservesBodyCountAndIds(PhysicsBackend backend) {
        PhysicsWorld world = PhysicsWorldFactory.create(PhysicsWorldConfig.defaults(backend));
        try {
            spawnScene(world);
            for (int i = 0; i < 20; i++) {
                world.step(DT, 1);
            }
            List<Integer> beforeIds = decode(backend, world.snapshot()).stream()
                .map(SnapshotBody::bodyId).sorted().toList();
            world.restore(world.snapshot());
            List<Integer> afterIds = decode(backend, world.snapshot()).stream()
                .map(SnapshotBody::bodyId).sorted().toList();
            assertEquals(beforeIds, afterIds);
        } finally {
            world.destroy();
        }
    }

    @Test
    void bothBackendsRestoreAndSimulateWithinBounds() {
        ScenarioResult ode = runScenario(PhysicsBackend.ODE4J, true);
        ScenarioResult jolt = runScenario(PhysicsBackend.JOLT, true);

        BackendMetrics mOde = metrics(decode(PhysicsBackend.ODE4J, ode.finalSnapshot()), ode.initialAverageY());
        BackendMetrics mJolt = metrics(decode(PhysicsBackend.JOLT, jolt.finalSnapshot()), jolt.initialAverageY());

        assertTrue(mOde.minY() > -0.6f, "ODE minY=" + mOde.minY());
        assertTrue(mJolt.minY() > -0.6f, "Jolt minY=" + mJolt.minY());
        assertTrue(mOde.avgY() < mOde.initialAverageY(), "ODE avgY did not fall");
        assertTrue(mJolt.avgY() < mJolt.initialAverageY(), "Jolt avgY did not fall");
        assertTrue(java.lang.Math.abs(mOde.avgY() - mJolt.avgY()) < 0.15f,
            "avgY drift ODE=" + mOde.avgY() + " Jolt=" + mJolt.avgY());

        assertTrue(ode.postEvents().stream().anyMatch(ContactEvent.class::isInstance), "No ODE contact events");
        assertTrue(jolt.postEvents().stream().anyMatch(ContactEvent.class::isInstance), "No Jolt contact events");
    }

    private static ScenarioResult runScenario(PhysicsBackend backend, boolean restoreMidway) {
        PhysicsWorld world = PhysicsWorldFactory.create(PhysicsWorldConfig.defaults(backend));
        try {
            List<RigidBodyHandle> handles = spawnScene(world);
            float initialAverageY = averageY(world, handles);

            for (int i = 0; i < PRE_STEPS; i++) {
                if (i == 20) {
                    applyDeterministicInputs(world, handles);
                }
                world.step(DT, 1);
                world.drainEvents();
            }

            byte[] mid = world.snapshot();
            if (restoreMidway) {
                world.restore(mid);
            }

            List<PhysicsEvent> post = new ArrayList<>();
            for (int i = 0; i < POST_STEPS; i++) {
                world.step(DT, 1);
                post.addAll(world.drainEvents());
            }

            byte[] end = world.snapshot();
            return new ScenarioResult(initialAverageY, mid, end, post);
        } finally {
            world.destroy();
        }
    }

    private static List<RigidBodyHandle> spawnScene(PhysicsWorld world) {
        world.spawnRigidBody(
            RigidBodyConfig.builder(CollisionShape.box(30f, 0.5f, 30f), 0f)
                .mode(BodyMode.STATIC)
                .worldTransform(new Matrix4f().identity().translation(0f, -0.5f, 0f))
                .build()
        );

        List<RigidBodyHandle> handles = new ArrayList<>();
        for (int i = 0; i < 10; i++) {
            float x = (i % 5 - 2) * 0.9f;
            float y = 2f + i * 0.25f;
            float z = (i / 5) * 1.2f;
            handles.add(world.spawnRigidBody(
                RigidBodyConfig.builder(CollisionShape.sphere(0.35f), 1f)
                    .worldTransform(new Matrix4f().identity().translation(x, y, z))
                    .build()
            ));
        }
        return handles;
    }

    private static void applyDeterministicInputs(PhysicsWorld world, List<RigidBodyHandle> handles) {
        for (int i = 0; i < handles.size(); i++) {
            float dir = (i % 2 == 0) ? 1f : -1f;
            world.applyImpulse(handles.get(i), new Vector3f(dir * 1.5f, 0f, 0.6f), new Vector3f());
        }
    }

    private static float averageY(PhysicsWorld world, List<RigidBodyHandle> handles) {
        float sum = 0f;
        for (RigidBodyHandle handle : handles) {
            sum += world.getBodyState(handle).position().y();
        }
        return sum / handles.size();
    }

    private static List<SnapshotBody> decode(PhysicsBackend backend, byte[] snapshot) {
        return backend == PhysicsBackend.ODE4J ? decodeOde(snapshot) : decodeJolt(snapshot);
    }

    private static List<SnapshotBody> decodeJolt(byte[] snapshot) {
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
        List<SnapshotBody> out = new ArrayList<>(bodyCount);
        for (int i = 0; i < bodyCount; i++) {
            int bodyId = in.readInt();
            BodyMode mode = BodyMode.values()[in.readByteUnsigned()];
            skipJoltShape(in);
            float mass = in.readFloat();
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
            out.add(new SnapshotBody(bodyId, mode, mass, p, lv));
        }
        out.sort(Comparator.comparingInt(SnapshotBody::bodyId));
        return out;
    }

    private static void skipJoltShape(ByteReader in) {
        in.readByteUnsigned();
        int flen = in.readInt();
        in.skip(flen * 4);
        int ilen = in.readInt();
        in.skip(ilen * 4);
    }

    private static List<SnapshotBody> decodeOde(byte[] snapshot) {
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
        List<SnapshotBody> out = new ArrayList<>(bodyCount);
        for (int i = 0; i < bodyCount; i++) {
            int bodyId = in.readInt();
            in.readInt();
            BodyMode mode = BodyMode.values()[in.readByteUnsigned()];
            in.readByteUnsigned();
            in.readInt();
            in.readInt();
            float mass = in.readFloat();
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
            out.add(new SnapshotBody(bodyId, mode, mass, p, lv));
        }
        out.sort(Comparator.comparingInt(SnapshotBody::bodyId));
        return out;
    }

    private static void skipOdeShape(ByteReader in) {
        int type = in.readByteUnsigned();
        switch (type) {
            case 0 -> in.skip(4); // sphere
            case 1 -> in.skip(12); // box
            case 2, 3 -> in.skip(8); // capsule, cylinder
            case 4 -> in.skip(16); // plane
            case 5, 6 -> { // convex, mesh
                int fLen = in.readInt();
                in.skip(fLen * 4);
                int iLen = in.readInt();
                in.skip(iLen * 4);
            }
            case 7 -> { // heightfield
                int hLen = in.readInt();
                in.skip(hLen * 4);
                in.skip(4 * 2);
                in.skip(4 * 3);
            }
            case 8 -> { // compound
                int children = in.readInt();
                for (int i = 0; i < children; i++) {
                    skipOdeShape(in);
                    in.skip(12 + 16 + 12); // transform
                }
            }
            default -> throw new IllegalArgumentException("Unknown ODE shape type: " + type);
        }
    }

    private static List<SnapshotBody> dynamicBodies(List<SnapshotBody> all) {
        return all.stream().filter(b -> b.mode() == BodyMode.DYNAMIC).toList();
    }

    private static BackendMetrics metrics(List<SnapshotBody> allBodies, float initialAverageY) {
        List<SnapshotBody> dynamic = dynamicBodies(allBodies);
        float minY = Float.POSITIVE_INFINITY;
        float sumY = 0f;
        for (SnapshotBody body : dynamic) {
            float y = body.position().y();
            minY = java.lang.Math.min(minY, y);
            sumY += y;
        }
        return new BackendMetrics(initialAverageY, sumY / dynamic.size(), minY);
    }

    private static void assertNear(Vector3f a, Vector3f b, float eps) {
        assertTrue(java.lang.Math.abs(a.x() - b.x()) <= eps, "x " + a.x() + " vs " + b.x());
        assertTrue(java.lang.Math.abs(a.y() - b.y()) <= eps, "y " + a.y() + " vs " + b.y());
        assertTrue(java.lang.Math.abs(a.z() - b.z()) <= eps, "z " + a.z() + " vs " + b.z());
    }

    private record ScenarioResult(
        float initialAverageY,
        byte[] midSnapshot,
        byte[] finalSnapshot,
        List<PhysicsEvent> postEvents
    ) {}

    private record SnapshotBody(int bodyId, BodyMode mode, float mass, Vector3f position, Vector3f linearVelocity) {}

    private record BackendMetrics(float initialAverageY, float avgY, float minY) {}

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
            return buffer.get() & 0xFF;
        }

        void skip(int bytes) {
            buffer.position(buffer.position() + bytes);
        }
    }
}
