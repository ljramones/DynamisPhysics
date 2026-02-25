package org.dynamisphysics.ode4j.shape;

import org.dynamiscollision.shapes.CollisionShape;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.OdeHelper;

public final class Ode4jShapeAdapter {
    private Ode4jShapeAdapter() {}

    public static DGeom toGeom(CollisionShape shape, DSpace space) {
        return switch (shape.shapeType()) {
            case SPHERE -> {
                var s = (org.dynamiscollision.shapes.SphereCollisionShape) shape;
                yield OdeHelper.createSphere(space, s.radius());
            }
            case BOX -> {
                var b = (org.dynamiscollision.shapes.BoxCollisionShape) shape;
                yield OdeHelper.createBox(space, b.halfExtentX() * 2.0, b.halfExtentY() * 2.0, b.halfExtentZ() * 2.0);
            }
            case CAPSULE -> {
                var c = (org.dynamiscollision.shapes.CapsuleCollisionShape) shape;
                yield OdeHelper.createCapsule(space, c.radius(), c.height());
            }
            case CYLINDER -> {
                var c = (org.dynamiscollision.shapes.CylinderCollisionShape) shape;
                yield OdeHelper.createCylinder(space, c.radius(), c.height());
            }
            case PLANE -> {
                var p = (org.dynamiscollision.shapes.PlaneCollisionShape) shape;
                yield OdeHelper.createPlane(space, p.normalX(), p.normalY(), p.normalZ(), p.distance());
            }
            case CONVEX_HULL, TRIANGLE_MESH, HEIGHTFIELD, COMPOUND ->
                throw new UnsupportedOperationException(shape.shapeType() + " not yet implemented — lands in Step 4");
        };
    }
}
