package org.dynamisphysics.ode4j.snapshot;

import org.dynamiscollision.shapes.BoxCollisionShape;
import org.dynamiscollision.shapes.CapsuleCollisionShape;
import org.dynamiscollision.shapes.CollisionShape;
import org.dynamiscollision.shapes.CompoundCollisionShape;
import org.dynamiscollision.shapes.ConvexHullCollisionShape;
import org.dynamiscollision.shapes.CylinderCollisionShape;
import org.dynamiscollision.shapes.HeightfieldCollisionShape;
import org.dynamiscollision.shapes.PlaneCollisionShape;
import org.dynamiscollision.shapes.ShapeType;
import org.dynamiscollision.shapes.SphereCollisionShape;
import org.dynamiscollision.shapes.TriangleMeshCollisionShape;
import org.dynamisphysics.api.body.BodyMode;
import org.dynamisphysics.api.body.BodyState;
import org.dynamisphysics.api.constraint.ConstraintDesc;
import org.dynamisphysics.api.constraint.ConstraintLimits;
import org.dynamisphysics.api.constraint.ConstraintMotor;
import org.dynamisphysics.api.constraint.ConstraintType;
import org.dynamisphysics.api.material.PhysicsMaterial;
import org.dynamisphysics.ode4j.body.Ode4jBodyHandle;
import org.dynamisphysics.ode4j.constraint.Ode4jConstraintHandle;
import org.vectrix.affine.Transformf;
import org.vectrix.core.Quaternionf;
import org.vectrix.core.Vector3f;

import java.util.ArrayList;
import java.util.List;

public final class Ode4jSnapshot {
    private static final int MAGIC = 0x44505953; // DPYS
    private static final int VERSION = 1;
    private static final int ENDIAN_MARKER = 0x1234;

    private Ode4jSnapshot() {}

    public static byte[] write(
        int stepCount,
        Vector3f gravity,
        int solverIterations,
        float timeScale,
        List<Ode4jBodyHandle> bodiesInIdOrder,
        List<Ode4jConstraintHandle> constraintsInIdOrder
    ) {
        Ode4jSnapshotWriter out = new Ode4jSnapshotWriter(64 * 1024);
        out.writeInt(MAGIC);
        out.writeShort(VERSION);
        out.writeShort(ENDIAN_MARKER);
        out.writeInt(stepCount);
        out.writeFloat(gravity.x());
        out.writeFloat(gravity.y());
        out.writeFloat(gravity.z());
        out.writeInt(solverIterations);
        out.writeFloat(timeScale);

        out.writeInt(bodiesInIdOrder.size());
        for (Ode4jBodyHandle body : bodiesInIdOrder) {
            writeBody(out, body);
        }

        out.writeInt(constraintsInIdOrder.size());
        for (Ode4jConstraintHandle c : constraintsInIdOrder) {
            writeConstraint(out, c);
        }

        return out.toByteArray();
    }

    public static RestoredState read(byte[] snapshot) {
        Ode4jSnapshotReader in = new Ode4jSnapshotReader(snapshot);
        int magic = in.readInt();
        if (magic != MAGIC) {
            throw new IllegalArgumentException("Invalid snapshot magic: " + Integer.toHexString(magic));
        }
        int version = in.readShort();
        if (version != VERSION) {
            throw new IllegalArgumentException("Unsupported snapshot version: " + version);
        }
        int endian = in.readShort();
        if (endian != ENDIAN_MARKER) {
            throw new IllegalArgumentException("Unexpected snapshot endianness marker: " + endian);
        }

        Header header = new Header(
            in.readInt(),
            new Vector3f(in.readFloat(), in.readFloat(), in.readFloat()),
            in.readInt(),
            in.readFloat()
        );

        int bodyCount = in.readInt();
        List<BodyRecord> bodies = new ArrayList<>(bodyCount);
        for (int i = 0; i < bodyCount; i++) {
            bodies.add(readBody(in));
        }

        int constraintCount = in.readInt();
        List<ConstraintRecord> constraints = new ArrayList<>(constraintCount);
        for (int i = 0; i < constraintCount; i++) {
            constraints.add(readConstraint(in));
        }

        if (in.hasRemaining()) {
            throw new IllegalArgumentException("Corrupt snapshot: trailing bytes");
        }
        return new RestoredState(header, List.copyOf(bodies), List.copyOf(constraints));
    }

    private static void writeBody(Ode4jSnapshotWriter out, Ode4jBodyHandle body) {
        BodyState state = body.body() == null
            ? new BodyState(
                new Vector3f((float) body.geom().getPosition().get0(), (float) body.geom().getPosition().get1(), (float) body.geom().getPosition().get2()),
                new Quaternionf(),
                new Vector3f(),
                new Vector3f(),
                true
            )
            : new BodyState(
                new Vector3f((float) body.body().getPosition().get0(), (float) body.body().getPosition().get1(), (float) body.body().getPosition().get2()),
                new Quaternionf(
                    (float) body.body().getQuaternion().get1(),
                    (float) body.body().getQuaternion().get2(),
                    (float) body.body().getQuaternion().get3(),
                    (float) body.body().getQuaternion().get0()
                ),
                new Vector3f((float) body.body().getLinearVel().get0(), (float) body.body().getLinearVel().get1(), (float) body.body().getLinearVel().get2()),
                new Vector3f((float) body.body().getAngularVel().get0(), (float) body.body().getAngularVel().get1(), (float) body.body().getAngularVel().get2()),
                !body.body().isEnabled()
            );

        int flags = 0;
        if (state.sleeping()) {
            flags |= 1;
        }
        if (body.config().ccd()) {
            flags |= 1 << 1;
        }
        if (body.config().isSensor()) {
            flags |= 1 << 2;
        }

        out.writeInt(body.bodyId());
        out.writeInt(body.geomId());
        out.writeByte(body.config().mode().ordinal());
        out.writeByte(flags);
        out.writeInt(body.config().layer());
        out.writeInt(body.config().collidesWith());
        out.writeFloat(body.config().mass());
        out.writeFloat(body.config().gravityScale());

        PhysicsMaterial mat = body.config().material();
        out.writeFloat(mat.friction());
        out.writeFloat(mat.restitution());
        out.writeFloat(mat.rollingFriction());
        out.writeFloat(mat.spinningFriction());
        out.writeString(mat.tag());

        writeShape(out, body.config().shape());

        out.writeFloat(state.position().x());
        out.writeFloat(state.position().y());
        out.writeFloat(state.position().z());
        out.writeFloat(state.orientation().x());
        out.writeFloat(state.orientation().y());
        out.writeFloat(state.orientation().z());
        out.writeFloat(state.orientation().w());
        out.writeFloat(state.linearVelocity().x());
        out.writeFloat(state.linearVelocity().y());
        out.writeFloat(state.linearVelocity().z());
        out.writeFloat(state.angularVelocity().x());
        out.writeFloat(state.angularVelocity().y());
        out.writeFloat(state.angularVelocity().z());
    }

    private static BodyRecord readBody(Ode4jSnapshotReader in) {
        int bodyId = in.readInt();
        int geomId = in.readInt();
        BodyMode mode = BodyMode.values()[in.readByte()];
        int flags = in.readByte();
        int layer = in.readInt();
        int collidesWith = in.readInt();
        float mass = in.readFloat();
        float gravityScale = in.readFloat();
        PhysicsMaterial mat = new PhysicsMaterial(
            in.readFloat(),
            in.readFloat(),
            in.readFloat(),
            in.readFloat(),
            in.readString()
        );
        CollisionShape shape = readShape(in);

        Vector3f pos = new Vector3f(in.readFloat(), in.readFloat(), in.readFloat());
        Quaternionf ori = new Quaternionf(in.readFloat(), in.readFloat(), in.readFloat(), in.readFloat());
        Vector3f lv = new Vector3f(in.readFloat(), in.readFloat(), in.readFloat());
        Vector3f av = new Vector3f(in.readFloat(), in.readFloat(), in.readFloat());

        return new BodyRecord(
            bodyId,
            geomId,
            mode,
            (flags & 1) != 0,
            (flags & (1 << 1)) != 0,
            (flags & (1 << 2)) != 0,
            layer,
            collidesWith,
            mass,
            gravityScale,
            mat,
            shape,
            pos,
            ori,
            lv,
            av
        );
    }

    private static void writeConstraint(Ode4jSnapshotWriter out, Ode4jConstraintHandle c) {
        ConstraintDesc d = c.desc();
        int bodyAId = d.bodyA() instanceof Ode4jBodyHandle b ? b.bodyId() : -1;
        int bodyBId = d.bodyB() instanceof Ode4jBodyHandle b ? b.bodyId() : -1;

        out.writeInt(c.constraintId());
        out.writeByte(d.type().ordinal());
        out.writeInt(bodyAId);
        out.writeInt(bodyBId);
        writeVec3(out, d.pivotA());
        writeVec3(out, d.pivotB());
        writeVec3(out, d.axisA());
        writeVec3(out, d.axisB());

        out.writeFloat(d.limits().linearLowerLimit());
        out.writeFloat(d.limits().linearUpperLimit());
        out.writeFloat(d.limits().angularLowerLimit());
        out.writeFloat(d.limits().angularUpperLimit());

        out.writeBoolean(d.motor().enabled());
        out.writeFloat(d.motor().targetVelocity());
        out.writeFloat(d.motor().targetPosition());
        out.writeFloat(d.motor().maxForce());
        out.writeFloat(d.motor().maxTorque());
        out.writeFloat(d.breakForce());
        out.writeFloat(d.breakTorque());
    }

    private static ConstraintRecord readConstraint(Ode4jSnapshotReader in) {
        int constraintId = in.readInt();
        ConstraintType type = ConstraintType.values()[in.readByte()];
        int bodyAId = in.readInt();
        int bodyBId = in.readInt();
        Vector3f pivotA = readVec3(in);
        Vector3f pivotB = readVec3(in);
        Vector3f axisA = readVec3(in);
        Vector3f axisB = readVec3(in);

        ConstraintLimits limits = new ConstraintLimits(
            in.readFloat(),
            in.readFloat(),
            in.readFloat(),
            in.readFloat()
        );
        ConstraintMotor motor = new ConstraintMotor(
            in.readBoolean(),
            in.readFloat(),
            in.readFloat(),
            in.readFloat(),
            in.readFloat()
        );

        return new ConstraintRecord(
            constraintId,
            type,
            bodyAId,
            bodyBId,
            pivotA,
            pivotB,
            axisA,
            axisB,
            limits,
            motor,
            in.readFloat(),
            in.readFloat()
        );
    }

    private static void writeShape(Ode4jSnapshotWriter out, CollisionShape shape) {
        out.writeByte(shape.shapeType().ordinal());
        switch (shape.shapeType()) {
            case SPHERE -> out.writeFloat(((SphereCollisionShape) shape).radius());
            case BOX -> {
                BoxCollisionShape b = (BoxCollisionShape) shape;
                out.writeFloat(b.halfExtentX());
                out.writeFloat(b.halfExtentY());
                out.writeFloat(b.halfExtentZ());
            }
            case CAPSULE -> {
                CapsuleCollisionShape c = (CapsuleCollisionShape) shape;
                out.writeFloat(c.radius());
                out.writeFloat(c.height());
            }
            case CYLINDER -> {
                CylinderCollisionShape c = (CylinderCollisionShape) shape;
                out.writeFloat(c.radius());
                out.writeFloat(c.height());
            }
            case PLANE -> {
                PlaneCollisionShape p = (PlaneCollisionShape) shape;
                out.writeFloat(p.normalX());
                out.writeFloat(p.normalY());
                out.writeFloat(p.normalZ());
                out.writeFloat(p.distance());
            }
            case CONVEX_HULL -> {
                ConvexHullCollisionShape c = (ConvexHullCollisionShape) shape;
                writeFloatArray(out, c.vertices());
                writeIntArray(out, c.indices());
            }
            case TRIANGLE_MESH -> {
                if (shape instanceof TriangleMeshCollisionShape t) {
                    writeFloatArray(out, t.vertices());
                    writeIntArray(out, t.indices());
                } else {
                    throw new UnsupportedOperationException(
                        "Unsupported TRIANGLE_MESH shape impl for snapshot: " + shape.getClass().getName());
                }
            }
            case HEIGHTFIELD -> {
                HeightfieldCollisionShape h = (HeightfieldCollisionShape) shape;
                writeFloatArray(out, h.heights());
                out.writeInt(h.widthSamples());
                out.writeInt(h.depthSamples());
                out.writeFloat(h.worldWidth());
                out.writeFloat(h.worldDepth());
                out.writeFloat(h.maxHeight());
            }
            case COMPOUND -> {
                CompoundCollisionShape c = (CompoundCollisionShape) shape;
                out.writeInt(c.childCount());
                for (int i = 0; i < c.childCount(); i++) {
                    writeShape(out, c.children().get(i));
                    writeTransform(out, c.localTransforms().get(i));
                }
            }
        }
    }

    private static CollisionShape readShape(Ode4jSnapshotReader in) {
        ShapeType type = ShapeType.values()[in.readByte()];
        return switch (type) {
            case SPHERE -> CollisionShape.sphere(in.readFloat());
            case BOX -> CollisionShape.box(in.readFloat(), in.readFloat(), in.readFloat());
            case CAPSULE -> CollisionShape.capsule(in.readFloat(), in.readFloat());
            case CYLINDER -> CollisionShape.cylinder(in.readFloat(), in.readFloat());
            case PLANE -> CollisionShape.plane(in.readFloat(), in.readFloat(), in.readFloat(), in.readFloat());
            case CONVEX_HULL -> CollisionShape.convexHull(readFloatArray(in), readIntArray(in));
            case TRIANGLE_MESH -> CollisionShape.triangleMesh(readFloatArray(in), readIntArray(in));
            case HEIGHTFIELD -> CollisionShape.heightfield(
                readFloatArray(in),
                in.readInt(),
                in.readInt(),
                in.readFloat(),
                in.readFloat(),
                in.readFloat()
            );
            case COMPOUND -> {
                int children = in.readInt();
                List<CollisionShape> shapes = new ArrayList<>(children);
                List<Transformf> transforms = new ArrayList<>(children);
                for (int i = 0; i < children; i++) {
                    shapes.add(readShape(in));
                    transforms.add(readTransform(in));
                }
                yield CollisionShape.compound(shapes, transforms);
            }
        };
    }

    private static void writeTransform(Ode4jSnapshotWriter out, Transformf t) {
        writeVec3(out, t.translation);
        out.writeFloat(t.rotation.x());
        out.writeFloat(t.rotation.y());
        out.writeFloat(t.rotation.z());
        out.writeFloat(t.rotation.w());
        writeVec3(out, t.scale);
    }

    private static Transformf readTransform(Ode4jSnapshotReader in) {
        Vector3f translation = readVec3(in);
        Quaternionf rotation = new Quaternionf(in.readFloat(), in.readFloat(), in.readFloat(), in.readFloat());
        Vector3f scale = readVec3(in);
        return new Transformf(translation, rotation, scale);
    }

    private static void writeVec3(Ode4jSnapshotWriter out, Vector3f v) {
        out.writeFloat(v.x());
        out.writeFloat(v.y());
        out.writeFloat(v.z());
    }

    private static Vector3f readVec3(Ode4jSnapshotReader in) {
        return new Vector3f(in.readFloat(), in.readFloat(), in.readFloat());
    }

    private static void writeFloatArray(Ode4jSnapshotWriter out, float[] values) {
        out.writeInt(values.length);
        for (float value : values) {
            out.writeFloat(value);
        }
    }

    private static float[] readFloatArray(Ode4jSnapshotReader in) {
        int len = in.readInt();
        float[] out = new float[len];
        for (int i = 0; i < len; i++) {
            out[i] = in.readFloat();
        }
        return out;
    }

    private static void writeIntArray(Ode4jSnapshotWriter out, int[] values) {
        out.writeInt(values.length);
        for (int value : values) {
            out.writeInt(value);
        }
    }

    private static int[] readIntArray(Ode4jSnapshotReader in) {
        int len = in.readInt();
        int[] out = new int[len];
        for (int i = 0; i < len; i++) {
            out[i] = in.readInt();
        }
        return out;
    }

    public record Header(int stepCount, Vector3f gravity, int solverIterations, float timeScale) {}

    public record BodyRecord(
        int bodyId,
        int geomId,
        BodyMode mode,
        boolean sleeping,
        boolean ccd,
        boolean sensor,
        int layer,
        int collidesWith,
        float mass,
        float gravityScale,
        PhysicsMaterial material,
        CollisionShape shape,
        Vector3f position,
        Quaternionf orientation,
        Vector3f linearVelocity,
        Vector3f angularVelocity
    ) {}

    public record ConstraintRecord(
        int constraintId,
        ConstraintType type,
        int bodyAId,
        int bodyBId,
        Vector3f pivotA,
        Vector3f pivotB,
        Vector3f axisA,
        Vector3f axisB,
        ConstraintLimits limits,
        ConstraintMotor motor,
        float breakForce,
        float breakTorque
    ) {
        public ConstraintDesc toConstraintDesc(Ode4jBodyHandle bodyA, Ode4jBodyHandle bodyB) {
            return new ConstraintDesc(
                type,
                bodyA,
                bodyB,
                pivotA,
                pivotB,
                axisA,
                axisB,
                limits,
                motor,
                breakForce,
                breakTorque
            );
        }
    }

    public record RestoredState(Header header, List<BodyRecord> bodies, List<ConstraintRecord> constraints) {}
}
