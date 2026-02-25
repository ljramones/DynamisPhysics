package org.dynamisphysics.ode4j.body;

import org.dynamiscollision.shapes.CollisionShape;
import org.dynamisphysics.ode4j.shape.Ode4jPrimitiveShape;
import org.ode4j.math.DMatrix3;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DMass;
import org.ode4j.ode.OdeHelper;
import org.vectrix.core.Vector3f;
import org.vectrix.physics.InertiaTensorf;

public final class Ode4jInertiaComputer {
    private Ode4jInertiaComputer() {}

    public static void applyInertia(DBody body, CollisionShape shape, float mass) {
        if (body == null) {
            return;
        }
        DMass m = OdeHelper.createMass();
        Vector3f diag = new Vector3f();

        if (shape instanceof Ode4jPrimitiveShape primitive) {
            switch (primitive.kind()) {
                case SPHERE -> InertiaTensorf.sphere(mass, primitive.x(), diag);
                case BOX -> InertiaTensorf.box(mass, primitive.x(), primitive.y(), primitive.z(), diag);
                case CAPSULE -> InertiaTensorf.capsule(mass, primitive.x(), primitive.y(), diag);
                case CYLINDER -> InertiaTensorf.cylinder(mass, primitive.x(), primitive.y(), diag);
                default -> InertiaTensorf.sphere(mass, 0.5f, diag);
            }
        } else {
            InertiaTensorf.sphere(mass, 0.5f, diag);
        }

        DMatrix3 inertia = new DMatrix3();
        inertia.set00(diag.x());
        inertia.set11(diag.y());
        inertia.set22(diag.z());
        m.setI(inertia);
        m.setMass(mass);
        body.setMass(m);
    }
}
