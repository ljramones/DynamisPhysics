package org.dynamisphysics.jolt.shape;

import com.github.stephengold.joltjni.BoxShape;
import com.github.stephengold.joltjni.CapsuleShape;
import com.github.stephengold.joltjni.CylinderShape;
import com.github.stephengold.joltjni.Plane;
import com.github.stephengold.joltjni.PlaneShape;
import com.github.stephengold.joltjni.Shape;
import com.github.stephengold.joltjni.SphereShape;
import org.dynamiscollision.shapes.BoxCollisionShape;
import org.dynamiscollision.shapes.CapsuleCollisionShape;
import org.dynamiscollision.shapes.CollisionShape;
import org.dynamiscollision.shapes.CylinderCollisionShape;
import org.dynamiscollision.shapes.PlaneCollisionShape;
import org.dynamiscollision.shapes.SphereCollisionShape;

public final class JoltShapeAdapter {
    private JoltShapeAdapter() {
    }

    public static Shape toJoltShape(CollisionShape shape) {
        return switch (shape.shapeType()) {
            case SPHERE -> new SphereShape(((SphereCollisionShape) shape).radius());
            case BOX -> {
                BoxCollisionShape box = (BoxCollisionShape) shape;
                yield new BoxShape(box.halfExtentX(), box.halfExtentY(), box.halfExtentZ());
            }
            case CAPSULE -> {
                CapsuleCollisionShape capsule = (CapsuleCollisionShape) shape;
                yield new CapsuleShape(capsule.height() * 0.5f, capsule.radius());
            }
            case CYLINDER -> {
                CylinderCollisionShape cylinder = (CylinderCollisionShape) shape;
                yield new CylinderShape(cylinder.height() * 0.5f, cylinder.radius());
            }
            case PLANE -> {
                PlaneCollisionShape p = (PlaneCollisionShape) shape;
                yield new PlaneShape(new Plane(p.normalX(), p.normalY(), p.normalZ(), p.distance()));
            }
            default -> throw new UnsupportedOperationException("Jolt shape not implemented yet: " + shape.shapeType());
        };
    }
}
