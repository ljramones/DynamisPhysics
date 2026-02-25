package org.dynamisphysics.jolt;

import org.dynamiscollision.shapes.CollisionShape;
import org.dynamisphysics.api.PhysicsWorldFactory;
import org.dynamisphysics.api.body.BodyMode;
import org.dynamisphysics.api.body.RigidBodyConfig;
import org.dynamisphysics.api.body.RigidBodyHandle;
import org.dynamisphysics.api.config.BroadphaseType;
import org.dynamisphysics.api.config.PhysicsBackend;
import org.dynamisphysics.api.config.PhysicsWorldConfig;
import org.dynamisphysics.api.constraint.MechanicalConstraintBuilders;
import org.dynamisphysics.api.event.PhysicsEvent;
import org.dynamisphysics.api.material.PhysicsMaterial;
import org.dynamisphysics.api.world.PhysicsWorld;
import org.dynamisphysics.ode4j.Ode4jBackendRegistrar;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;
import org.vectrix.core.Matrix4f;
import org.vectrix.core.Vector3f;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class HeavySnapshotParityTest {
    private static final float DT = 1f / 60f;
    private static final int PRE_STEPS = 180;
    private static final int POST_STEPS = 180;

    @BeforeAll
    static void registerBackends() {
        new Ode4jBackendRegistrar();
        new JoltBackendRegistrar();
    }

    @ParameterizedTest
    @EnumSource(value = PhysicsBackend.class, names = {"ODE4J", "JOLT"})
    void restoreAndContinueMatchesContinuousForHeavyMixedScene(PhysicsBackend backend) {
        ScenarioOutcome continuous = runScenario(backend, false);
        ScenarioOutcome restored = runScenario(backend, true);

        List<BodyRec> continuousBodies = dynamicBodies(decode(backend, continuous.finalSnapshot));
        List<BodyRec> restoredBodies = dynamicBodies(decode(backend, restored.finalSnapshot));
        assertEquals(continuousBodies.size(), restoredBodies.size(), "dynamic body count mismatch");

        List<Integer> continuousIds = continuousBodies.stream().map(BodyRec::bodyId).sorted().toList();
        List<Integer> restoredIds = restoredBodies.stream().map(BodyRec::bodyId).sorted().toList();
        assertEquals(continuousIds, restoredIds, "dynamic ID set mismatch");

        Metrics mContinuous = metrics(continuousBodies);
        Metrics mRestored = metrics(restoredBodies);
        assertTrue(Float.isFinite(mRestored.avgY) && Float.isFinite(mRestored.avgSpeed), "non-finite restored metrics");
        assertTrue(mRestored.minY > -5f, "restored scene fell through bounds: minY=" + mRestored.minY);
        assertTrue(java.lang.Math.abs(mContinuous.avgY - mRestored.avgY) < 2f,
            "avgY diverged too much: continuous=" + mContinuous.avgY + " restored=" + mRestored.avgY);
        assertTrue(java.lang.Math.abs(mContinuous.avgSpeed - mRestored.avgSpeed) < 5f,
            "avgSpeed diverged too much: continuous=" + mContinuous.avgSpeed + " restored=" + mRestored.avgSpeed);

        assertTrue(!restored.postEvents.isEmpty(), "expected post-restore events");
    }

    private static ScenarioOutcome runScenario(PhysicsBackend backend, boolean restoreMidway) {
        PhysicsWorld world = createWorld(backend);
        try {
            ScenarioHandles handles = spawnHeavyScene(world);
            List<PhysicsEvent> events = new ArrayList<>();
            for (int i = 0; i < PRE_STEPS; i++) {
                drive(world, handles, i);
                world.step(DT, 1);
                events.addAll(world.drainEvents());
            }

            byte[] mid = world.snapshot();
            if (restoreMidway) {
                world.restore(mid);
                handles = null; // stale handles by contract after restore
            }

            List<PhysicsEvent> postEvents = new ArrayList<>();
            for (int i = 0; i < POST_STEPS; i++) {
                world.step(DT, 1);
                postEvents.addAll(world.drainEvents());
            }
            return new ScenarioOutcome(world.snapshot(), postEvents);
        } finally {
            world.destroy();
        }
    }

    private static PhysicsWorld createWorld(PhysicsBackend backend) {
        PhysicsWorldConfig d = PhysicsWorldConfig.defaults(backend);
        return PhysicsWorldFactory.create(new PhysicsWorldConfig(
            backend, d.gravity(), DT, 1, d.solverIterations(), d.maxBodies(), d.maxConstraints(),
            BroadphaseType.BVH, true
        ));
    }

    private static ScenarioHandles spawnHeavyScene(PhysicsWorld world) {
        world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.box(60f, 0.5f, 60f), 0f)
            .mode(BodyMode.STATIC)
            .material(PhysicsMaterial.ASPHALT)
            .worldTransform(new Matrix4f().identity().translation(0f, -0.5f, 0f))
            .build());

        List<RigidBodyHandle> spheres = new ArrayList<>();
        for (int i = 0; i < 20; i++) {
            float x = (i % 5 - 2) * 1.2f;
            float y = 2f + (i / 5) * 0.7f;
            float z = (i % 4 - 1.5f) * 1.1f;
            spheres.add(world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.sphere(0.3f), 1f)
                .worldTransform(new Matrix4f().identity().translation(x, y, z)).build()));
        }

        RigidBodyHandle gearA = world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.box(0.3f, 0.3f, 0.3f), 1f)
            .worldTransform(new Matrix4f().identity().translation(-6f, 3f, 0f)).build());
        RigidBodyHandle gearB = world.spawnRigidBody(RigidBodyConfig.builder(CollisionShape.box(0.3f, 0.3f, 0.3f), 1f)
            .worldTransform(new Matrix4f().identity().translation(-6f, 5f, 0f)).build());
        world.addConstraint(MechanicalConstraintBuilders.gear(
            gearA, gearB, new Vector3f(0f, 1f, 0f), new Vector3f(0f, 1f, 0f), 2f
        ));

        return new ScenarioHandles(spheres, gearA);
    }

    private static void drive(PhysicsWorld world, ScenarioHandles h, int step) {
        if (step % 120 == 0) {
            world.applyImpulse(h.gearA, new Vector3f(0f, 0f, 0.8f), new Vector3f());
        }
        if (step % 90 == 0) {
            for (int i = 0; i < h.spheres.size(); i += 4) {
                world.applyImpulse(h.spheres.get(i), new Vector3f(0.6f, 0f, 0.2f), new Vector3f());
            }
        }
    }

    private static List<BodyRec> decode(PhysicsBackend backend, byte[] snapshot) {
        return backend == PhysicsBackend.ODE4J ? decodeOde(snapshot) : decodeJolt(snapshot);
    }

    private static List<BodyRec> decodeJolt(byte[] snapshot) {
        ByteReader in = new ByteReader(snapshot);
        in.readInt();
        in.readShort();
        in.readShort();
        in.readInt();
        in.readFloat();
        in.readFloat();
        in.readFloat();
        in.readFloat();
        int count = in.readInt();
        List<BodyRec> out = new ArrayList<>(count);
        for (int i = 0; i < count; i++) {
            int id = in.readInt();
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
            out.add(new BodyRec(id, mode, p, lv));
        }
        out.sort(Comparator.comparingInt(BodyRec::bodyId));
        return out;
    }

    private static void skipJoltShape(ByteReader in) {
        in.readByteUnsigned();
        int fLen = in.readInt();
        in.skip(fLen * 4);
        int iLen = in.readInt();
        in.skip(iLen * 4);
    }

    private static List<BodyRec> decodeOde(byte[] snapshot) {
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
        int count = in.readInt();
        List<BodyRec> out = new ArrayList<>(count);
        for (int i = 0; i < count; i++) {
            int id = in.readInt();
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
            if (strLen > 0) in.skip(strLen);
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
            out.add(new BodyRec(id, mode, p, lv));
        }
        out.sort(Comparator.comparingInt(BodyRec::bodyId));
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
            default -> throw new IllegalArgumentException("unknown shape type " + type);
        }
    }

    private static List<BodyRec> dynamicBodies(List<BodyRec> list) {
        return list.stream().filter(b -> b.mode == BodyMode.DYNAMIC).toList();
    }

    private static Metrics metrics(List<BodyRec> bodies) {
        float minY = Float.POSITIVE_INFINITY;
        float sumY = 0f;
        float sumSpeed = 0f;
        for (BodyRec b : bodies) {
            minY = java.lang.Math.min(minY, b.p.y());
            sumY += b.p.y();
            sumSpeed += b.v.length();
        }
        int n = java.lang.Math.max(1, bodies.size());
        return new Metrics(sumY / n, minY, sumSpeed / n);
    }

    private static final class ByteReader {
        private final ByteBuffer buf;

        private ByteReader(byte[] data) {
            this.buf = ByteBuffer.wrap(data).order(ByteOrder.LITTLE_ENDIAN);
        }

        int readInt() { return buf.getInt(); }
        short readShort() { return buf.getShort(); }
        float readFloat() { return buf.getFloat(); }
        int readByteUnsigned() { return Byte.toUnsignedInt(buf.get()); }
        void skip(int bytes) { buf.position(buf.position() + bytes); }
    }

    private record BodyRec(int bodyId, BodyMode mode, Vector3f p, Vector3f v) {
    }

    private record Metrics(float avgY, float minY, float avgSpeed) {
    }

    private record ScenarioHandles(
        List<RigidBodyHandle> spheres,
        RigidBodyHandle gearA
    ) {
    }

    private record ScenarioOutcome(byte[] finalSnapshot, List<PhysicsEvent> postEvents) {
    }
}
