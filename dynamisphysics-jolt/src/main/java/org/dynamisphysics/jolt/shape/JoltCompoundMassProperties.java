package org.dynamisphysics.jolt.shape;

import org.dynamiscollision.shapes.BoxCollisionShape;
import org.dynamiscollision.shapes.CapsuleCollisionShape;
import org.dynamiscollision.shapes.CollisionShape;
import org.dynamiscollision.shapes.CompoundCollisionShape;
import org.dynamiscollision.shapes.CylinderCollisionShape;
import org.dynamiscollision.shapes.SphereCollisionShape;
import org.vectrix.affine.Transformf;
import org.vectrix.core.Vector3f;

import java.util.ArrayList;
import java.util.List;

final class JoltCompoundMassProperties {
    private JoltCompoundMassProperties() {}

    static Vector3f computeCenterOfMass(CompoundCollisionShape compound) {
        if (compound.childCount() == 0) {
            return new Vector3f();
        }
        List<CollisionShape> children = compound.children();
        List<Float> volumes = new ArrayList<>(children.size());
        float total = 0f;
        for (CollisionShape child : children) {
            float v = Math.max(approximateVolume(child), 0.0001f);
            volumes.add(v);
            total += v;
        }
        if (total <= 0f) {
            return new Vector3f();
        }
        Vector3f com = new Vector3f();
        for (int i = 0; i < compound.childCount(); i++) {
            float w = volumes.get(i) / total;
            Transformf t = compound.localTransforms().get(i);
            com.add(t.translation.x() * w, t.translation.y() * w, t.translation.z() * w);
        }
        return com;
    }

    private static float approximateVolume(CollisionShape shape) {
        return switch (shape.shapeType()) {
            case SPHERE -> {
                SphereCollisionShape s = (SphereCollisionShape) shape;
                yield (float) ((4.0 / 3.0) * Math.PI * s.radius() * s.radius() * s.radius());
            }
            case BOX -> {
                BoxCollisionShape b = (BoxCollisionShape) shape;
                yield (b.halfExtentX() * 2f) * (b.halfExtentY() * 2f) * (b.halfExtentZ() * 2f);
            }
            case CAPSULE -> {
                CapsuleCollisionShape c = (CapsuleCollisionShape) shape;
                float cyl = (float) (Math.PI * c.radius() * c.radius() * c.height());
                float hemi = (float) ((4.0 / 3.0) * Math.PI * c.radius() * c.radius() * c.radius());
                yield cyl + hemi;
            }
            case CYLINDER -> {
                CylinderCollisionShape c = (CylinderCollisionShape) shape;
                yield (float) (Math.PI * c.radius() * c.radius() * c.height());
            }
            default -> {
                var aabb = shape.getWorldBounds(new Transformf().identity());
                yield (float) (aabb.sizeX() * aabb.sizeY() * aabb.sizeZ());
            }
        };
    }
}

