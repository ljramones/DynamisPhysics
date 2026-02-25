package org.dynamisphysics.ode4j.shape;

import org.dynamiscollision.bounds.Aabb;
import org.dynamiscollision.geometry.Ray3D;
import org.dynamiscollision.geometry.RaycastResult;
import org.dynamiscollision.shapes.CollisionShape;
import org.vectrix.affine.Transformf;

import java.util.Optional;

/**
 * Temporary primitive shape carrier until collision_detection exposes primitive factories.
 */
public final class Ode4jPrimitiveShape implements CollisionShape {
    public enum Kind { SPHERE, BOX, CAPSULE, CYLINDER, PLANE }

    private final Kind kind;
    private final float x;
    private final float y;
    private final float z;

    private Ode4jPrimitiveShape(Kind kind, float x, float y, float z) {
        this.kind = kind;
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public static Ode4jPrimitiveShape sphere(float radius) { return new Ode4jPrimitiveShape(Kind.SPHERE, radius, 0f, 0f); }
    public static Ode4jPrimitiveShape box(float hx, float hy, float hz) { return new Ode4jPrimitiveShape(Kind.BOX, hx, hy, hz); }
    public static Ode4jPrimitiveShape capsule(float radius, float height) { return new Ode4jPrimitiveShape(Kind.CAPSULE, radius, height, 0f); }
    public static Ode4jPrimitiveShape cylinder(float radius, float height) { return new Ode4jPrimitiveShape(Kind.CYLINDER, radius, height, 0f); }
    public static Ode4jPrimitiveShape plane() { return new Ode4jPrimitiveShape(Kind.PLANE, 0f, 0f, 0f); }

    public Kind kind() { return kind; }
    public float x() { return x; }
    public float y() { return y; }
    public float z() { return z; }

    @Override
    public Aabb getWorldBounds(Transformf transformf) {
        return new Aabb(0, 0, 0, 1, 1, 1);
    }

    @Override
    public Optional<RaycastResult> raycast(Ray3D ray3D, Transformf transformf) {
        return Optional.empty();
    }
}
