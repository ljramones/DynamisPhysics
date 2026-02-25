package org.dynamisphysics.ode4j.debug;

import org.dynamiscollision.bounds.Aabb;
import org.dynamiscollision.shapes.BoxCollisionShape;
import org.dynamiscollision.shapes.CapsuleCollisionShape;
import org.dynamiscollision.shapes.CollisionShape;
import org.dynamiscollision.shapes.CylinderCollisionShape;
import org.dynamiscollision.shapes.PlaneCollisionShape;
import org.dynamiscollision.shapes.ShapeType;
import org.dynamiscollision.shapes.SphereCollisionShape;
import org.vectrix.affine.Transformf;
import org.vectrix.core.Quaternionf;
import org.vectrix.core.Vector3f;

public final class ShapeWireframes {
    private static final int RING_SEGMENTS = 16;

    private ShapeWireframes() {}

    public static void addShape(
        WireframeBuilder wb,
        CollisionShape shape,
        Vector3f position,
        Quaternionf orientation,
        float[] rgba,
        boolean showAabbs
    ) {
        ShapeType type = shape.shapeType();
        switch (type) {
            case SPHERE -> addSphere(wb, (SphereCollisionShape) shape, position, orientation, rgba);
            case BOX -> addBox(wb, (BoxCollisionShape) shape, position, orientation, rgba);
            case CAPSULE -> addCapsule(wb, (CapsuleCollisionShape) shape, position, orientation, rgba);
            case CYLINDER -> addCylinder(wb, (CylinderCollisionShape) shape, position, orientation, rgba);
            case PLANE -> addPlane(wb, (PlaneCollisionShape) shape, position, orientation, rgba);
            case CONVEX_HULL, TRIANGLE_MESH, HEIGHTFIELD, COMPOUND -> {
                if (showAabbs) {
                    addAabb(wb, shape, position, orientation, rgba);
                }
            }
        }
    }

    private static void addSphere(WireframeBuilder wb, SphereCollisionShape s, Vector3f p, Quaternionf q, float[] c) {
        addRing(wb, p, q, s.radius(), 0, c);
        addRing(wb, p, q, s.radius(), 1, c);
        addRing(wb, p, q, s.radius(), 2, c);
    }

    private static void addBox(WireframeBuilder wb, BoxCollisionShape b, Vector3f p, Quaternionf q, float[] c) {
        Vector3f[] v = new Vector3f[] {
            t(new Vector3f(-b.halfExtentX(), -b.halfExtentY(), -b.halfExtentZ()), p, q),
            t(new Vector3f(+b.halfExtentX(), -b.halfExtentY(), -b.halfExtentZ()), p, q),
            t(new Vector3f(+b.halfExtentX(), +b.halfExtentY(), -b.halfExtentZ()), p, q),
            t(new Vector3f(-b.halfExtentX(), +b.halfExtentY(), -b.halfExtentZ()), p, q),
            t(new Vector3f(-b.halfExtentX(), -b.halfExtentY(), +b.halfExtentZ()), p, q),
            t(new Vector3f(+b.halfExtentX(), -b.halfExtentY(), +b.halfExtentZ()), p, q),
            t(new Vector3f(+b.halfExtentX(), +b.halfExtentY(), +b.halfExtentZ()), p, q),
            t(new Vector3f(-b.halfExtentX(), +b.halfExtentY(), +b.halfExtentZ()), p, q)
        };
        int[][] e = new int[][] {
            {0,1},{1,2},{2,3},{3,0},
            {4,5},{5,6},{6,7},{7,4},
            {0,4},{1,5},{2,6},{3,7}
        };
        for (int[] edge : e) {
            wb.addLine(v[edge[0]], v[edge[1]], c);
        }
    }

    private static void addCapsule(WireframeBuilder wb, CapsuleCollisionShape cap, Vector3f p, Quaternionf q, float[] c) {
        float r = cap.radius();
        float hh = cap.height() * 0.5f;
        addRing(wb, t(new Vector3f(0f, hh, 0f), p, q), q, r, 0, c);
        addRing(wb, t(new Vector3f(0f, -hh, 0f), p, q), q, r, 0, c);
        wb.addLine(t(new Vector3f(r, hh, 0f), p, q), t(new Vector3f(r, -hh, 0f), p, q), c);
        wb.addLine(t(new Vector3f(-r, hh, 0f), p, q), t(new Vector3f(-r, -hh, 0f), p, q), c);
        wb.addLine(t(new Vector3f(0f, hh, r), p, q), t(new Vector3f(0f, -hh, r), p, q), c);
        wb.addLine(t(new Vector3f(0f, hh, -r), p, q), t(new Vector3f(0f, -hh, -r), p, q), c);
    }

    private static void addCylinder(WireframeBuilder wb, CylinderCollisionShape cyl, Vector3f p, Quaternionf q, float[] c) {
        float r = cyl.radius();
        float hh = cyl.height() * 0.5f;
        addRing(wb, t(new Vector3f(0f, hh, 0f), p, q), q, r, 0, c);
        addRing(wb, t(new Vector3f(0f, -hh, 0f), p, q), q, r, 0, c);
        wb.addLine(t(new Vector3f(r, hh, 0f), p, q), t(new Vector3f(r, -hh, 0f), p, q), c);
        wb.addLine(t(new Vector3f(-r, hh, 0f), p, q), t(new Vector3f(-r, -hh, 0f), p, q), c);
    }

    private static void addPlane(WireframeBuilder wb, PlaneCollisionShape plane, Vector3f p, Quaternionf q, float[] c) {
        float s = 5f;
        wb.addLine(t(new Vector3f(-s, 0f, -s), p, q), t(new Vector3f(s, 0f, -s), p, q), c);
        wb.addLine(t(new Vector3f(s, 0f, -s), p, q), t(new Vector3f(s, 0f, s), p, q), c);
        wb.addLine(t(new Vector3f(s, 0f, s), p, q), t(new Vector3f(-s, 0f, s), p, q), c);
        wb.addLine(t(new Vector3f(-s, 0f, s), p, q), t(new Vector3f(-s, 0f, -s), p, q), c);
        Vector3f n = t(new Vector3f(plane.normalX(), plane.normalY(), plane.normalZ()).normalize(), new Vector3f(), q);
        wb.addLine(p, p.add(n.mul(1.0f, new Vector3f()), new Vector3f()), c);
    }

    private static void addAabb(WireframeBuilder wb, CollisionShape shape, Vector3f p, Quaternionf q, float[] c) {
        Aabb aabb = shape.getWorldBounds(new Transformf(p, q, new Vector3f(1f, 1f, 1f)));
        float minX = (float) aabb.minX();
        float minY = (float) aabb.minY();
        float minZ = (float) aabb.minZ();
        float maxX = (float) aabb.maxX();
        float maxY = (float) aabb.maxY();
        float maxZ = (float) aabb.maxZ();
        Vector3f[] v = new Vector3f[] {
            new Vector3f(minX, minY, minZ), new Vector3f(maxX, minY, minZ),
            new Vector3f(maxX, maxY, minZ), new Vector3f(minX, maxY, minZ),
            new Vector3f(minX, minY, maxZ), new Vector3f(maxX, minY, maxZ),
            new Vector3f(maxX, maxY, maxZ), new Vector3f(minX, maxY, maxZ)
        };
        int[][] e = new int[][] {
            {0,1},{1,2},{2,3},{3,0},
            {4,5},{5,6},{6,7},{7,4},
            {0,4},{1,5},{2,6},{3,7}
        };
        for (int[] edge : e) {
            wb.addLine(v[edge[0]], v[edge[1]], c);
        }
    }

    private static void addRing(WireframeBuilder wb, Vector3f center, Quaternionf q, float r, int axis, float[] c) {
        Vector3f prev = null;
        for (int i = 0; i <= RING_SEGMENTS; i++) {
            float t = (float) (i * (java.lang.Math.PI * 2.0) / RING_SEGMENTS);
            float cs = (float) java.lang.Math.cos(t);
            float sn = (float) java.lang.Math.sin(t);
            Vector3f local = switch (axis) {
                case 1 -> new Vector3f(0f, r * cs, r * sn);
                case 2 -> new Vector3f(r * cs, r * sn, 0f);
                default -> new Vector3f(r * cs, 0f, r * sn);
            };
            Vector3f curr = t(local, center, q);
            if (prev != null) {
                wb.addLine(prev, curr, c);
            }
            prev = curr;
        }
    }

    private static Vector3f t(Vector3f local, Vector3f p, Quaternionf q) {
        return q.transform(local, new Vector3f()).add(p, new Vector3f());
    }
}
