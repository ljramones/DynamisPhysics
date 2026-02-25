package org.dynamisphysics.ode4j.shape;

import org.dynamiscollision.shapes.CollisionShape;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.OdeHelper;

public final class Ode4jShapeAdapter {
    private Ode4jShapeAdapter() {}

    public static DGeom toGeom(CollisionShape shape, DSpace space) {
        if (!(shape instanceof Ode4jPrimitiveShape primitive)) {
            throw new UnsupportedOperationException(shape + " not yet implemented — lands in Step 4");
        }

        return switch (primitive.kind()) {
            case SPHERE -> OdeHelper.createSphere(space, primitive.x());
            case BOX -> OdeHelper.createBox(space, primitive.x() * 2.0, primitive.y() * 2.0, primitive.z() * 2.0);
            case CAPSULE -> OdeHelper.createCapsule(space, primitive.x(), primitive.y());
            case CYLINDER -> OdeHelper.createCylinder(space, primitive.x(), primitive.y());
            case PLANE -> OdeHelper.createPlane(space, 0.0, 1.0, 0.0, 0.0);
        };
    }
}
