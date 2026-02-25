package org.dynamisphysics.test.mock;

import org.dynamiscollision.bounds.Aabb;
import org.dynamiscollision.geometry.Ray3D;
import org.dynamiscollision.geometry.RaycastResult;
import org.dynamiscollision.shapes.CollisionShape;
import org.vectrix.affine.Transformf;

import java.util.Optional;

/**
 * Minimal shape stubs for test fixtures.
 */
public final class TestCollisionShapes {
    private TestCollisionShapes() {}

    public static CollisionShape sphere(float radius) {
        return new NamedShape("sphere(" + radius + ")");
    }

    public static CollisionShape box(float x, float y, float z) {
        return new NamedShape("box(" + x + "," + y + "," + z + ")");
    }

    public static CollisionShape capsule(float radius, float height) {
        return new NamedShape("capsule(" + radius + "," + height + ")");
    }

    private record NamedShape(String name) implements CollisionShape {
        @Override
        public Aabb getWorldBounds(Transformf transform) {
            return new Aabb(0, 0, 0, 1, 1, 1);
        }

        @Override
        public Optional<RaycastResult> raycast(Ray3D ray3D, Transformf transformf) {
            return Optional.empty();
        }

        @Override
        public String toString() {
            return name;
        }
    }
}
