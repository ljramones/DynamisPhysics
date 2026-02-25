package org.dynamisphysics.jolt.snapshot;

import org.dynamisphysics.api.body.BodyMode;
import org.dynamisphysics.api.body.BodyState;
import org.dynamisphysics.api.body.RigidBodyConfig;
import org.dynamisphysics.jolt.body.JoltBodyHandle;
import org.vectrix.core.Quaternionf;
import org.vectrix.core.Vector3f;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public final class JoltSnapshot {
    private static final int MAGIC = 0x44505953; // DPYS
    private static final short VERSION = 1;

    private JoltSnapshot() {
    }

    public static byte[] write(int stepCount, Vector3f gravity, float timeScale, List<JoltBodyHandle> bodies,
        java.util.function.Function<JoltBodyHandle, BodyState> stateReader) {
        List<JoltBodyHandle> ordered = bodies.stream().sorted(Comparator.comparingInt(JoltBodyHandle::bodyId)).toList();

        int cap = 4 + 2 + 2 + 4 + (3 * 4) + 4 + 4;
        cap += 4;
        cap += ordered.size() * (4 + 1 + 4 + 4 + 4 + 4 + (3 * 4) + (4 * 4) + (3 * 4) + (3 * 4));
        cap += ordered.size() * 64;

        ByteBuffer out = ByteBuffer.allocate(cap).order(ByteOrder.LITTLE_ENDIAN);
        out.putInt(MAGIC);
        out.putShort(VERSION);
        out.putShort((short) 0x1234);
        out.putInt(stepCount);
        out.putFloat(gravity.x());
        out.putFloat(gravity.y());
        out.putFloat(gravity.z());
        out.putFloat(timeScale);
        out.putInt(ordered.size());

        for (JoltBodyHandle handle : ordered) {
            RigidBodyConfig config = handle.config();
            BodyState state = stateReader.apply(handle);
            out.putInt(handle.bodyId());
            out.put((byte) config.mode().ordinal());
            writeShape(config.shape(), out);
            out.putFloat(config.mass());
            out.putFloat(config.gravityScale());
            out.putInt(config.layer());
            out.putInt(config.collidesWith());
            out.putFloat(state.position().x());
            out.putFloat(state.position().y());
            out.putFloat(state.position().z());
            out.putFloat(state.orientation().x());
            out.putFloat(state.orientation().y());
            out.putFloat(state.orientation().z());
            out.putFloat(state.orientation().w());
            out.putFloat(state.linearVelocity().x());
            out.putFloat(state.linearVelocity().y());
            out.putFloat(state.linearVelocity().z());
            out.putFloat(state.angularVelocity().x());
            out.putFloat(state.angularVelocity().y());
            out.putFloat(state.angularVelocity().z());
        }

        byte[] bytes = new byte[out.position()];
        out.flip();
        out.get(bytes);
        return bytes;
    }

    public static RestoredState read(byte[] snapshot) {
        ByteBuffer in = ByteBuffer.wrap(snapshot).order(ByteOrder.LITTLE_ENDIAN);
        int magic = in.getInt();
        if (magic != MAGIC) {
            throw new IllegalArgumentException("Invalid snapshot magic: " + Integer.toHexString(magic));
        }
        short version = in.getShort();
        if (version != VERSION) {
            throw new IllegalArgumentException("Unsupported snapshot version: " + version);
        }
        in.getShort();
        int stepCount = in.getInt();
        Vector3f gravity = new Vector3f(in.getFloat(), in.getFloat(), in.getFloat());
        float timeScale = in.getFloat();

        int bodyCount = in.getInt();
        List<BodySnapshot> bodies = new ArrayList<>(bodyCount);
        for (int i = 0; i < bodyCount; i++) {
            int bodyId = in.getInt();
            BodyMode mode = BodyMode.values()[in.get()];
            ShapeSnapshot shape = readShape(in);
            float mass = in.getFloat();
            float gravityScale = in.getFloat();
            int layer = in.getInt();
            int collidesWith = in.getInt();
            Vector3f p = new Vector3f(in.getFloat(), in.getFloat(), in.getFloat());
            Quaternionf q = new Quaternionf(in.getFloat(), in.getFloat(), in.getFloat(), in.getFloat());
            Vector3f lv = new Vector3f(in.getFloat(), in.getFloat(), in.getFloat());
            Vector3f av = new Vector3f(in.getFloat(), in.getFloat(), in.getFloat());

            bodies.add(new BodySnapshot(bodyId, mode, shape, mass, gravityScale, layer, collidesWith,
                new BodyState(p, q, lv, av, false)));
        }

        return new RestoredState(stepCount, gravity, timeScale, bodies);
    }

    public record RestoredState(int stepCount, Vector3f gravity, float timeScale, List<BodySnapshot> bodies) {
    }

    public record BodySnapshot(int bodyId, BodyMode mode, ShapeSnapshot shape, float mass, float gravityScale,
                               int layer, int collidesWith, BodyState state) {
    }

    public record ShapeSnapshot(byte type, float[] f, int[] i) {
    }

    private static void writeShape(org.dynamiscollision.shapes.CollisionShape shape, ByteBuffer out) {
        out.put((byte) shape.shapeType().ordinal());
        switch (shape.shapeType()) {
            case SPHERE -> {
                float r = ((org.dynamiscollision.shapes.SphereCollisionShape) shape).radius();
                out.putInt(1);
                out.putFloat(r);
                out.putInt(0);
            }
            case BOX -> {
                var b = (org.dynamiscollision.shapes.BoxCollisionShape) shape;
                out.putInt(3);
                out.putFloat(b.halfExtentX());
                out.putFloat(b.halfExtentY());
                out.putFloat(b.halfExtentZ());
                out.putInt(0);
            }
            case CAPSULE -> {
                var c = (org.dynamiscollision.shapes.CapsuleCollisionShape) shape;
                out.putInt(2);
                out.putFloat(c.radius());
                out.putFloat(c.height());
                out.putInt(0);
            }
            case CYLINDER -> {
                var c = (org.dynamiscollision.shapes.CylinderCollisionShape) shape;
                out.putInt(2);
                out.putFloat(c.radius());
                out.putFloat(c.height());
                out.putInt(0);
            }
            case PLANE -> {
                var p = (org.dynamiscollision.shapes.PlaneCollisionShape) shape;
                out.putInt(4);
                out.putFloat(p.normalX());
                out.putFloat(p.normalY());
                out.putFloat(p.normalZ());
                out.putFloat(p.distance());
                out.putInt(0);
            }
            default -> throw new UnsupportedOperationException("Snapshot does not support shape " + shape.shapeType());
        }
    }

    private static ShapeSnapshot readShape(ByteBuffer in) {
        byte type = in.get();
        int flen = in.getInt();
        float[] f = new float[flen];
        for (int i = 0; i < flen; i++) {
            f[i] = in.getFloat();
        }
        int ilen = in.getInt();
        int[] ints = new int[ilen];
        for (int i = 0; i < ilen; i++) {
            ints[i] = in.getInt();
        }
        return new ShapeSnapshot(type, f, ints);
    }
}
