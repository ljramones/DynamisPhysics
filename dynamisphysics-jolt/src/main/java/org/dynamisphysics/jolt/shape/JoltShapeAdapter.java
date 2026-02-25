package org.dynamisphysics.jolt.shape;

import com.github.stephengold.joltjni.BoxShape;
import com.github.stephengold.joltjni.CapsuleShape;
import com.github.stephengold.joltjni.CylinderShape;
import com.github.stephengold.joltjni.MutableCompoundShape;
import com.github.stephengold.joltjni.Plane;
import com.github.stephengold.joltjni.PlaneShape;
import com.github.stephengold.joltjni.Quat;
import com.github.stephengold.joltjni.Shape;
import com.github.stephengold.joltjni.SphereShape;
import com.github.stephengold.joltjni.Vec3;
import org.dynamiscollision.shapes.BoxCollisionShape;
import org.dynamiscollision.shapes.CapsuleCollisionShape;
import org.dynamiscollision.shapes.CollisionShape;
import org.dynamiscollision.shapes.CompoundCollisionShape;
import org.dynamiscollision.shapes.CylinderCollisionShape;
import org.dynamiscollision.shapes.PlaneCollisionShape;
import org.dynamiscollision.shapes.SphereCollisionShape;
import org.vectrix.affine.Transformf;
import org.vectrix.core.Vector3f;

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
            case COMPOUND -> {
                CompoundCollisionShape c = (CompoundCollisionShape) shape;
                MutableCompoundShape compound = new MutableCompoundShape();
                Vector3f com = JoltCompoundMassProperties.computeCenterOfMass(c);
                for (int i = 0; i < c.childCount(); i++) {
                    Transformf t = c.localTransforms().get(i);
                    Vector3f local = new Vector3f(t.translation).sub(com);
                    Shape child = toJoltShape(c.children().get(i));
                    compound.addShape(
                        new Vec3(local.x(), local.y(), local.z()),
                        new Quat(t.rotation.x(), t.rotation.y(), t.rotation.z(), t.rotation.w()),
                        child
                    );
                }
                yield compound;
            }
            default -> throw new UnsupportedOperationException("Jolt shape not implemented yet: " + shape.shapeType());
        };
    }
}
