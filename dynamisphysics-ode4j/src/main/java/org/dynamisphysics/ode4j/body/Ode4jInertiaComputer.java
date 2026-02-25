package org.dynamisphysics.ode4j.body;

import org.dynamiscollision.shapes.CollisionShape;
import org.dynamiscollision.shapes.BoxCollisionShape;
import org.dynamiscollision.shapes.CapsuleCollisionShape;
import org.dynamiscollision.shapes.CylinderCollisionShape;
import org.dynamiscollision.shapes.SphereCollisionShape;
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

        switch (shape.shapeType()) {
            case SPHERE -> {
                SphereCollisionShape s = (SphereCollisionShape) shape;
                InertiaTensorf.sphere(mass, s.radius(), diag);
            }
            case BOX -> {
                BoxCollisionShape b = (BoxCollisionShape) shape;
                InertiaTensorf.box(mass, b.halfExtentX(), b.halfExtentY(), b.halfExtentZ(), diag);
            }
            case CAPSULE -> {
                CapsuleCollisionShape c = (CapsuleCollisionShape) shape;
                InertiaTensorf.capsule(mass, c.radius(), c.height(), diag);
            }
            case CYLINDER -> {
                CylinderCollisionShape c = (CylinderCollisionShape) shape;
                InertiaTensorf.cylinder(mass, c.radius(), c.height(), diag);
            }
            default -> InertiaTensorf.sphere(mass, 0.5f, diag);
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
