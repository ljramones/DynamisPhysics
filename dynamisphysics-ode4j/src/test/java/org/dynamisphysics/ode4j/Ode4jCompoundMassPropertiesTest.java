package org.dynamisphysics.ode4j;

import org.dynamiscollision.shapes.CollisionShape;
import org.dynamiscollision.shapes.CompoundCollisionShape;
import org.dynamisphysics.ode4j.body.Ode4jCompoundMassProperties;
import org.dynamisphysics.ode4j.body.Ode4jInertiaComputer;
import org.junit.jupiter.api.Test;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DMassC;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeHelper;
import org.vectrix.affine.Transformf;
import org.vectrix.core.Quaternionf;
import org.vectrix.core.Vector3f;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;

class Ode4jCompoundMassPropertiesTest {

    @Test
    void equalSphereCompoundComputesExpectedCom() {
        CompoundCollisionShape compound = (CompoundCollisionShape) CollisionShape.compound(
            List.of(CollisionShape.sphere(0.5f), CollisionShape.sphere(0.5f)),
            List.of(local(-1f, 0f, 0f), local(3f, 0f, 0f))
        );

        Vector3f com = Ode4jCompoundMassProperties.computeCenterOfMass(compound);
        DMassC mass = Ode4jCompoundMassProperties.compute(compound, 2f);
        assertEquals(2.0, mass.getMass(), 1e-6);
        assertEquals(1.0, com.x(), 1e-5);
        assertEquals(0.0, com.y(), 1e-5);
        assertEquals(0.0, com.z(), 1e-5);
        assertEquals(0.0, mass.getC().get0(), 1e-5);
        assertEquals(0.0, mass.getC().get1(), 1e-5);
        assertEquals(0.0, mass.getC().get2(), 1e-5);
    }

    @Test
    void volumeWeightedSphereCompoundComputesExpectedCom() {
        CompoundCollisionShape compound = (CompoundCollisionShape) CollisionShape.compound(
            List.of(CollisionShape.sphere(1f), CollisionShape.sphere(2f)),
            List.of(local(-1f, 0f, 0f), local(3f, 0f, 0f))
        );

        Vector3f com = Ode4jCompoundMassProperties.computeCenterOfMass(compound);
        DMassC mass = Ode4jCompoundMassProperties.compute(compound, 9f);
        assertEquals(9.0, mass.getMass(), 1e-5);
        assertEquals(23.0 / 9.0, com.x(), 1e-4);
        assertEquals(0.0, com.y(), 1e-5);
        assertEquals(0.0, com.z(), 1e-5);
        assertEquals(0.0, mass.getC().get0(), 1e-5);
        assertEquals(0.0, mass.getC().get1(), 1e-5);
        assertEquals(0.0, mass.getC().get2(), 1e-5);
    }

    @Test
    void applyInertiaPropagatesCompoundComToBodyMass() {
        DWorld world = OdeHelper.createWorld();
        DBody body = OdeHelper.createBody(world);
        try {
            CompoundCollisionShape compound = (CompoundCollisionShape) CollisionShape.compound(
                List.of(CollisionShape.sphere(0.5f), CollisionShape.sphere(0.5f)),
                List.of(local(-1f, 0f, 0f), local(3f, 0f, 0f))
            );
            Ode4jInertiaComputer.applyInertia(body, compound, 2f);
            DMassC mass = body.getMass();
            assertEquals(2.0, mass.getMass(), 1e-6);
            assertEquals(0.0, mass.getC().get0(), 1e-5);
            assertEquals(0.0, mass.getC().get1(), 1e-5);
            assertEquals(0.0, mass.getC().get2(), 1e-5);
        } finally {
            body.destroy();
            world.destroy();
        }
    }

    private static Transformf local(float x, float y, float z) {
        return new Transformf(
            new Vector3f(x, y, z),
            new Quaternionf(0f, 0f, 0f, 1f),
            new Vector3f(1f, 1f, 1f)
        );
    }
}
